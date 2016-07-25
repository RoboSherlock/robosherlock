/*
 * Copyright (c) 2012, Ferenc Balint-Benczed<balintbe@cs.uni-bremen.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <uima/api.hpp>

#include <ctype.h>

#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/utils/common.h>
#include <rs/DrawingAnnotator.h>

using namespace uima;

class ClusterFilter : public DrawingAnnotator
{
private:

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  cv::Mat color, mask;
  double pointSize;

  std::vector<std::vector<int> > clusterIndices;
public:

  ClusterFilter(): DrawingAnnotator(__func__), cloud(new pcl::PointCloud<pcl::PointXYZRGBA>), pointSize(1.0)
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
    cas.get(VIEW_MASK, mask);

    std::vector<rs::Cluster> clusters;
    std::vector<rs::Identifiable> filteredClusters;
    scene.identifiables.filter(clusters);

    clusterIndices.clear();
    clusterIndices.reserve(clusters.size());

    // Filter clusters that are touching the borders of the depth camera
    for(size_t i = 0; i < clusters.size(); ++i)
    {
      rs::Cluster &cluster = clusters[i];

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
      if(ratio > 0.055)
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
