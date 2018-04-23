/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <uima/api.hpp>

#include <ctype.h>

#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/utils/common.h>
#include <rs/DrawingAnnotator.h>

using namespace uima;

class ClusterMerger : public DrawingAnnotator
{
private:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  cv::Mat color;
  double pointSize;

  //for visualization
  std::vector<std::vector<int> > clusterIndices;
  std::vector<cv::Rect> clusterRois;
public:

  ClusterMerger(): DrawingAnnotator(__func__), cloud(new pcl::PointCloud<pcl::PointXYZRGBA>), pointSize(1.0)
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  std::vector<int> intersection(std::vector<int> &v1, std::vector<int> &v2)
  {
    std::vector<int> v3;
    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());
    std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), back_inserter(v3));

    return v3;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    cas.get(VIEW_CLOUD, *cloud);
    cas.get(VIEW_COLOR_IMAGE, color);

    std::vector<rs::Cluster> clusters;
    std::vector<rs::Identifiable> mergedClusters;
    scene.identifiables.filter(clusters);
    std::vector<bool> duplicates(clusters.size(), false);
    std::vector<int>  duplicateWith(clusters.size(), -1);

    std::vector<bool> keepCluster(clusters.size(), true);

    clusterRois.clear();

    clusterIndices.clear();
    clusterIndices.reserve(clusters.size());

    outInfo("Scene has " << clusters.size() << " clusters");
   for(size_t i = 0; i < clusters.size(); ++i)
    {
      rs::Cluster &cluster1 = clusters[i];
      rs::ImageROI cluster1ImageRoi = cluster1.rois();
      cv::Rect roi1;
      rs::conversion::from(cluster1ImageRoi.roi_hires(), roi1);
      pcl::PointIndicesPtr cluster1Indices(new pcl::PointIndices());
      rs::conversion::from(((rs::ReferenceClusterPoints)cluster1.points.get()).indices.get(), *cluster1Indices);
      if(!cluster1Indices->indices.empty())
      {
        for(size_t j = i + 1; j < clusters.size(); ++j)
        {
          rs::Cluster &cluster2 = clusters[j];
          outInfo("Source: "<<cluster2.source());
          pcl::PointIndicesPtr cluster2Indices(new pcl::PointIndices());
          rs::conversion::from(((rs::ReferenceClusterPoints)cluster2.points.get()).indices.get(), *cluster2Indices);

          rs::ImageROI cluster2ImageRoi = cluster2.rois();
          cv::Rect roi2;
          rs::conversion::from(cluster2ImageRoi.roi_hires(), roi2);
          if(!cluster2Indices->indices.empty())
          {
            int common3DPoints = intersection(cluster1Indices->indices, cluster2Indices->indices).size();

            cv::Rect intersect = roi1 & roi2;
            //first handle the case when a hyp is fully inside another hyp;
            if(intersect.area() == roi1.area())
            {
              keepCluster[i] = false;
              duplicateWith[j] = i;
            }
            else if(intersect.area() == roi2.area())
            {
              keepCluster[j] = false;
              duplicateWith[i] = j;
            }
            else if(common3DPoints != 0)
            {
              outDebug("Cluster " << i << "(" << cluster1.source() << ") has " << common3DPoints << " common 3D points with Cluster " << j << "( " << cluster2.source() << " )");
              outDebug("That is " << (double)common3DPoints / cluster1Indices->indices.size() * 100 << " % of Cluster " << i << "s total points");
              outDebug("That is " << (double)common3DPoints / cluster2Indices->indices.size() * 100 << " % of Cluster " << j << "s total points");

              if(((double)common3DPoints / cluster1Indices->indices.size()) < ((double)common3DPoints / cluster2Indices->indices.size()))
              {
                outDebug("Keeping Cluster: " << i);
                keepCluster[j] = false;
                duplicateWith[i] = j;
              }
              else
              {
                outDebug("Keeping Cluster: " << j);
                keepCluster[i] = false;
                duplicateWith[j] = i;
              }
            }
          }
        }
      }
    }
    for(size_t i = 0; i < keepCluster.size(); ++i)
    {
      if(keepCluster[i])
      {
        if(duplicateWith[i] != -1)
        {
          rs::Cluster &other = clusters[duplicateWith[i]];

          std::vector<rs::Annotation> annotations;
          other.annotations.filter(annotations);
          if(!annotations.empty())
          {
            clusters[i].annotations.append(annotations);
          }
          if(other.source() != clusters[i].source())
          {
            std::vector<rs::PoseAnnotation> poses;
            clusters[i].annotations.filter(poses);
            if(!poses.empty())
            {
              clusters[i].annotations.remove(poses[0]);
            }
          }

        }
        mergedClusters.push_back(clusters[i]);

        //for visualization if cluster has no 3D points add empty vector
        if(!clusters[i].points.has())
        {
          this->clusterIndices.push_back(std::vector<int>());
        }
        else
        {
          pcl::PointIndicesPtr clusterIndices(new pcl::PointIndices());
          rs::conversion::from(((rs::ReferenceClusterPoints)clusters[i].points.get()).indices.get(), *clusterIndices);
          this->clusterIndices.push_back(clusterIndices->indices);
        }
        cv::Rect roi;
        rs::conversion::from(clusters[i].rois().roi(), roi);
        this->clusterRois.push_back(roi);
      }
    }
    scene.identifiables.set(mergedClusters);

    outDebug("BEGIN: After adding new clusters scene has " << scene.identifiables.size() << " identifiables");
    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = color.clone();

    for(size_t i = 0; i < clusterIndices.size(); ++i)
    {
      const std::vector<int> &indices = clusterIndices[i];
      for(size_t j = 0; j < indices.size(); ++j)
      {
        const int index = indices[j];
        disp.at<cv::Vec3b>(index) = rs::common::cvVec3bColors[i % rs::common::numberOfColors];
      }
    }
    for(size_t i = 0; i < clusterRois.size(); ++i)
    {
        cv::rectangle(disp,clusterRois[i], rs::common::cvScalarColors[i % rs::common::numberOfColors]);
    }

  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name;

    for(size_t i = 0; i < clusterIndices.size(); ++i)
    {
      const std::vector<int> &indices = clusterIndices[i];
      for(size_t j = 0; j < indices.size(); ++j)
      {
        const int index = indices[j];
        cloud->points[index].rgba = rs::common::colors[i % rs::common::numberOfColors];
      }
    }

    if(firstRun)
    {
      visualizer.addPointCloud(cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }
};

MAKE_AE(ClusterMerger)
