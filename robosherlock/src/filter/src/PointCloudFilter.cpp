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

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/common/transforms.h>

using namespace uima;


class PointCloudFilter : public DrawingAnnotator
{

private:
  typedef pcl::PointXYZRGBA PointT;
  pcl::PointCloud<PointT>::Ptr cloud_filtered;

  double pointSize;
  float minX, maxX, minY, maxY, minZ, maxZ;
  Type cloud_type;

  std::string source_frame;
  std::string target_frame;

  cv::Mat rgb_;
  tf::TransformListener transformListener;

public:

  PointCloudFilter(): DrawingAnnotator(__func__), pointSize(1)
  {
    cloud_filtered = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("minX", minX);
    ctx.extractValue("maxX", maxX);

    ctx.extractValue("minY", minY);
    ctx.extractValue("maxY", maxY);

    ctx.extractValue("minZ", minZ);
    ctx.extractValue("maxZ", maxZ);

    if(ctx.isParameterDefined("source_frame")) {
        ctx.extractValue("source_frame", source_frame);
    }if(ctx.isParameterDefined("target_frame")) {
          ctx.extractValue("target_frame", target_frame);
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
    outInfo("process start");
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);

    cas.get(VIEW_CLOUD, *cloud_ptr);
    cas.get(VIEW_COLOR_IMAGE, rgb_);

    pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>);
    tf::StampedTransform transform;
    bool was_transformed = false;
    if(transformListener.frameExists(source_frame) && transformListener.frameExists(target_frame)) {
      transformListener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
      Eigen::Affine3d eigenTransform;
      tf::transformTFToEigen(transform, eigenTransform);
      pcl::transformPointCloud<PointT>(*cloud_ptr, *cloud_transformed, eigenTransform);
      was_transformed = true;
    } else {
        outWarn("No point cloud transformation");
        cloud_transformed = cloud_ptr;
    }

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_transformed);
    pass.setKeepOrganized(true);
    pass.setFilterLimits(minX, maxX);
    pass.setFilterFieldName("x");
    pass.filter(*cloud_filtered);

    pass.setFilterLimits(minY, maxY);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.filter(*cloud_filtered);

    pass.setFilterLimits(minZ, maxZ);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.filter(*cloud_filtered);

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    if(was_transformed) {
        outInfo("Transforming matrix back (this will deeply hurt the CPU...");
        Eigen::Affine3d eigenTransform;
        tf::transformTFToEigen(transform.inverse(), eigenTransform);
        pcl::transformPointCloud<PointT>(*cloud_filtered, *cloud, eigenTransform);
    } else {
        cloud = cloud_filtered;
    }
    cas.set(VIEW_CLOUD, *cloud);



    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = rgb_.clone();
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
MAKE_AE(PointCloudFilter)
