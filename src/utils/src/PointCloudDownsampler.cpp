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

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>

using namespace uima;


class PointCloudDownsampler : public DrawingAnnotator
{

private:
  typedef pcl::PointXYZRGBA PointT;
  pcl::PointCloud<PointT>::Ptr cloud_filtered;
  float leaf_size;
  double pointSize;
  Type cloud_type;

public:

  PointCloudDownsampler(): DrawingAnnotator(__func__), leaf_size(0.02), pointSize(1)
  {
    cloud_filtered = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("leaf_size", leaf_size);

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
    outInfo("process start");
    rs::SceneCas cas(tcas);
    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
    (new pcl::PointCloud<PointT>);
    outInfo("Leaf Size =  " << leaf_size);

    cas.get(VIEW_CLOUD, *cloud_ptr);

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud_ptr);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*cloud_filtered);

    outInfo("Downsampled size: " << cloud_filtered->points.size());
    outInfo("Downsampled dim: " << cloud_filtered->width << ", " << cloud_filtered->height);

    cloud_filtered->width = cloud_ptr->width;
    cloud_filtered->height = cloud_ptr->height;
    cas.set(VIEW_CLOUD, *cloud_filtered);

    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {

  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = this->name;

    if(firstRun)
    {
      visualizer.addPointCloud(cloud_filtered, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(cloud_filtered, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(PointCloudDownsampler)
