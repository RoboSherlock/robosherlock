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

  std::vector<std::vector<int> > clusterIndices;
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
    clusterIndices.clear();
    clusterIndices.reserve(clusters.size());

    outInfo("Scene has " << clusters.size() << " clusters");
    for(size_t i = 0; i < clusters.size(); ++i)
    {
      rs::Cluster &cluster1 = clusters[i];
      if(cluster1.rois.has())
      {
        rs::ImageROI image_roisc1 = cluster1.rois.get();
        cv::Rect cluster1Roi;
        rs::conversion::from(image_roisc1.roi_hires(), cluster1Roi);
        for(size_t j = i + 1; j < clusters.size(); ++j)
        {
          rs::Cluster &cluster2 = clusters[j];
          if(cluster2.rois.has())
          {
            rs::ImageROI image_roisc2 = cluster2.rois.get();
            cv::Rect cluster2Roi;
            rs::conversion::from(image_roisc2.roi_hires(), cluster2Roi);
            cv::Rect intersection = cluster1Roi & cluster2Roi;
            if(intersection.area() > cluster1Roi.area() / 10)
            {
              if(cluster1Roi.area() > cluster2Roi.area())
              {
                duplicates[j] = true;
              }
              else
              {
                duplicates[i] = true;
              }
              duplicateWith[i] = j;
              duplicateWith[j] = i;
            }
          }
        }
      }
    }

    for(size_t i = 0; i < duplicates.size(); ++i)
    {
      outInfo("++++++++++Cluster " << i << " BEGIN+++++++++++++++");
      if(!duplicates[i])
      {
        rs::Cluster &cluster = clusters[i];

        /*if the duplicate cluster was found using table top segmentation,
         * delete the pose annotation since if definitely wrong
         **/
        if(duplicateWith[i] != -1)
        {
          outInfo("Cluster " <<  i << " has a duplicate");
          outInfo("other cluster is: Cluster " << duplicateWith[i]);
          rs::Cluster &other = clusters[duplicateWith[i]];

          outInfo("This cluster was found using: " << cluster.source());
          outInfo("Other cluster was found using: " << other.source());

          if(other.source() != cluster.source())
          {
            std::vector<rs::PoseAnnotation> poses;
            cluster.annotations.filter(poses);
            outInfo("This cluster has " << poses.size() << " pose annotations");
            if(!poses.empty())
            {
              cluster.annotations.remove(poses[0]);
            }
          }
        }

        std::vector<rs::PoseAnnotation> poses;
        cluster.annotations.filter(poses);
        outInfo("Adding object with " << poses.size() << "pose annotations");
        mergedClusters.push_back(cluster);

        //for vis purposes
        if(!cluster.points.has())
        {
          this->clusterIndices.push_back(std::vector<int>());
        }
        else
        {
          pcl::PointIndicesPtr clusterIndices(new pcl::PointIndices());
          rs::conversion::from(((rs::ReferenceClusterPoints)cluster.points.get()).indices.get(), *clusterIndices);
          this->clusterIndices.push_back(clusterIndices->indices);
        }
      }
      else
      {
        outInfo("Cluster " << i << " exists twice");
      }
      outInfo("++++++++++Cluster " << i << " END+++++++++++++++");
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
