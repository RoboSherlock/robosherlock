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

class ClusterMerger : public DrawingAnnotator
{
private:

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr dispCloud;
  std::vector<rs::Cluster> dispClusters;
  cv::Mat dispRGB;
  double pointSize;
public:

  ClusterMerger(): DrawingAnnotator(__func__),dispCloud(new pcl::PointCloud<pcl::PointXYZRGBA>), pointSize(1.0)
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
    dispCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_CLOUD,*dispCloud);
    cas.get(VIEW_COLOR_IMAGE,dispRGB);
    std::vector<rs::Cluster> clusters;
    std::vector<rs::Identifiable> mergedClusters;
    scene.identifiables.filter(clusters);
    outInfo("Scene has " << clusters.size() << " clusters");
    std::vector<bool> duplicates(clusters.size(), false);
    for(uint8_t i = 0; i < clusters.size(); ++i)
    {
      rs::Cluster &cluster1 = clusters[i];
      if(cluster1.rois.has())
      {
        rs::ImageROI image_roisc1 = cluster1.rois.get();
        cv::Rect cluster1Roi;
        rs::conversion::from(image_roisc1.roi_hires(), cluster1Roi);
        for(uint8_t j = i + 1; j < clusters.size(); ++j)
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
            }
          }
        }
      }
    }

    for(uint8_t i = 0; i < duplicates.size(); ++i)
    {
      if(!duplicates[i])
      {
        mergedClusters.push_back(clusters[i]);
      }
      else
      {
        outInfo("Cluster " << i << "exists twice");
      }
    }

    scene.identifiables.set(mergedClusters);
    dispClusters.clear();
    scene.identifiables.filter(dispClusters);

    for(unsigned int i = 0; i < dispClusters.size(); ++i)
    {
      rs::Cluster &cluster = dispClusters[i];
      if(!cluster.points.has())
      {
        continue;
      }
      pcl::PointIndicesPtr clusterIndices(new pcl::PointIndices());
      rs::conversion::from(((rs::ReferenceClusterPoints)cluster.points.get()).indices.get(), *clusterIndices);
      for(unsigned int k = 0; k < clusterIndices->indices.size(); ++k)
      {
        int index = clusterIndices->indices[k];
        dispCloud->points[index].rgba = rs::common::colors[i % COLOR_SIZE];
      }
    }

    outDebug("BEGIN: After adding new clusters scene has " << scene.identifiables.size() << " identifiables");
    return UIMA_ERR_NONE;
  }
  void drawImageWithLock(cv::Mat &disp)
  {
    disp = dispRGB.clone();
  }
  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {

    const std::string cloudname = this->name;

    if(firstRun)
    {
      visualizer.addPointCloud(dispCloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(dispCloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }

  }

};

MAKE_AE(ClusterMerger)
