/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *         Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *         Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
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

#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>
#include <robosherlock/utils/output.h>
#include <robosherlock/utils/common.h>
#include <robosherlock/DrawingAnnotator.h>

using namespace uima;

class ClusterFilter : public DrawingAnnotator
{
private:

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  cv::Mat color, mask;
  double pointSize, maxOverlap;

  std::vector<std::vector<int> > clusterIndices;
public:

  ClusterFilter(): DrawingAnnotator(__func__), cloud(new pcl::PointCloud<pcl::PointXYZRGBA>), pointSize(1.0), maxOverlap(0.055)
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    if(ctx.isParameterDefined("max_allowed_overlap"))
    {
      float tmp = (float)maxOverlap;
      ctx.extractValue("max_allowed_overlap", tmp);
      maxOverlap = tmp;
    }
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
    cas.get(VIEW_MASK, mask);

    std::vector<rs::ObjectHypothesis> clusters;
    std::vector<rs::Identifiable> filteredClusters;
    scene.identifiables.filter(clusters);

    clusterIndices.clear();
    clusterIndices.reserve(clusters.size());

    // Filter clusters that are touching the borders of the depth camera
    for(size_t i = 0; i < clusters.size(); ++i)
    {
      rs::ObjectHypothesis &cluster = clusters[i];

      if(!cluster.points.has())
      {
        outWarn("cluster has no points!");
        this->clusterIndices.push_back(std::vector<int>());
        continue;
      }

      pcl::PointIndicesPtr clusterIndices(new pcl::PointIndices());
      rs::conversion::from(((rs::ReferenceClusterPoints)cluster.points.get()).indices.get(), *clusterIndices);
      int countBorder = 0;

      for(int j = 0; j < clusterIndices->indices.size(); ++j)
      {
        const int index = clusterIndices->indices[j];
        if(mask.at<uint8_t>(index))
        {
          ++countBorder;
        }
      }

      const double ratio = countBorder / (double)clusterIndices->indices.size();
      if(ratio > maxOverlap)
      {
        outInfo("cluster filtered: " << i << " ratio: " << ratio << " (" << countBorder << "/" << clusterIndices->indices.size() << ")");
      }
      else
      {
        filteredClusters.push_back(clusters[i]);
      }

      this->clusterIndices.push_back(clusterIndices->indices);
    }

    scene.identifiables.set(filteredClusters);

    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = color.clone();

    const size_t size = mask.rows * mask.cols;
    const uint8_t *itM = mask.ptr();
    cv::Vec3b *itD = disp.ptr<cv::Vec3b>();

    for(size_t i = 0; i < size; ++i, ++itD, ++itM)
    {
      if(*itM > 0)
      {
        itD->val[2] = 255;
      }
    }
    for(size_t i = 0; i < clusterIndices.size(); ++i)
    {
      const std::vector<int> &indices = clusterIndices[i];
      for(size_t j = 0; j < indices.size(); ++j)
      {
        const int index = indices[j];
        if(mask.at<uint8_t>(index) > 0)
        {
          disp.at<cv::Vec3b>(index) = cv::Vec3b(255, 255, 255);
        }
        else
        {
          disp.at<cv::Vec3b>(index) = rs::common::cvVec3bColors[i % rs::common::numberOfColors];
        }
      }
    }
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name;

    const size_t size = mask.rows * mask.cols;
    const uint8_t *itM = mask.ptr();
    pcl::PointXYZRGBA *itC = &cloud->points[0];

    for(size_t i = 0; i < size; ++i, ++itC, ++itM)
    {
      if(*itM > 0)
      {
        itC->r = 255;
      }
    }
    for(size_t i = 0; i < clusterIndices.size(); ++i)
    {
      const std::vector<int> &indices = clusterIndices[i];
      for(size_t j = 0; j < indices.size(); ++j)
      {
        const int index = indices[j];
        if(mask.at<uint8_t>(index) > 0)
        {
          cloud->points[index].rgba = 0xFFFFFF;
        }
        else
        {
          cloud->points[index].rgba = rs::common::colors[i % rs::common::numberOfColors];
        }
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

MAKE_AE(ClusterFilter)
