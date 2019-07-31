/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *            Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *            Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
 *
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

//this is needed becaus ein 1.8 we get a runtime error
#include <pcl/search/impl/kdtree.hpp>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <rs/DrawingAnnotator.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>

using namespace uima;

class NormalEstimator : public DrawingAnnotator
{
private:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_non_nan;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr thermal_cloud_ptr;
  pcl::PointCloud<pcl::Normal>::Ptr normals_ptr;
  pcl::PointCloud<pcl::Normal>::Ptr normals_non_nan;
  pcl::PointCloud<pcl::Normal>::Ptr thermal_normals_ptr;

  double pointSize;
  double normalsColor[3];

private:
  bool useThermal, useRGB;
  bool receiveRGB;
  float radiusSearch;
  cv::Mat rgb_, normalsImg_;
  enum
  {
    PCL_RGBD,
    PCL_RGBDT
  } pclDispMode;

public:
  NormalEstimator() : DrawingAnnotator(__func__), pointSize(1), normalsColor {1.0, 0.0, 0.0}, pclDispMode(PCL_RGBD)
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    if(ctx.isParameterDefined("useThermal"))
    {
      ctx.extractValue("useThermal", useThermal);
    }
    else
    {
      useThermal = false;
    }
    if(ctx.isParameterDefined("useRGB"))
    {
      ctx.extractValue("useRGB", useRGB);
    }
    else
    {
      useRGB = true;
    }

    if(ctx.isParameterDefined("radiusSearch"))
    {
      ctx.extractValue("radiusSearch", radiusSearch);
    }
    setAnnotatorContext(ctx);
    return UIMA_ERR_NONE;
  }
/*
  TyErrorId reconfigure()
  {
    outError("Reconfiguring");
    AnnotatorContext &ctx = getAnnotatorContext();
    if(ctx.isParameterDefined("radiusSearch"))
    {
      outInfo("radiusSearch is rewritten-ish");
    }
    return UIMA_ERR_NONE;
  }
*/
  TyErrorId destroy()
  {
    outInfo("destroy");

    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");

    // create necessary pcl objects
    cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cloud_non_nan.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    thermal_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    normals_ptr.reset(new pcl::PointCloud<pcl::Normal>);
    normals_non_nan.reset(new pcl::PointCloud<pcl::Normal>);
    thermal_normals_ptr.reset(new pcl::PointCloud<pcl::Normal>);

    pcl::PointIndices non_NaN_ids;

    // create scene cas wrapper for cas and get kinect frame
    rs::SceneCas cas(tcas);
    if(useThermal && cas.get(VIEW_THERMAL_CLOUD, *thermal_cloud_ptr))
    {
      compute_normals_pcl(thermal_cloud_ptr, thermal_normals_ptr);
      cas.set(VIEW_THERMAL_NORMALS, *thermal_normals_ptr);
    }
    if(useRGB && cas.get(VIEW_CLOUD, *cloud_ptr))
    {
      cas.get(VIEW_COLOR_IMAGE, rgb_);
      outInfo("Cloud Size: " << cloud_ptr->points.size());
      if(cloud_ptr->isOrganized())
      {
        compute_normals_pcl(cloud_ptr, normals_ptr);
        cas.set(VIEW_NORMALS, *normals_ptr);
      }
      else
      {
        compute_normals_unOrganizedCloud(cloud_ptr, normals_ptr);
        cas.set(VIEW_NORMALS, *normals_ptr);
      }



      filter_NaN_points(cloud_ptr, normals_ptr, cloud_non_nan, normals_non_nan, non_NaN_ids.indices);

      normalsImg_ = cv::Mat::zeros(rgb_.size(), CV_32FC3);
      for(int i = 0; i < non_NaN_ids.indices.size(); ++i)
      {
        int x =  non_NaN_ids.indices[i] % rgb_.cols;
        int y =  non_NaN_ids.indices[i] / rgb_.cols;
        normalsImg_.at<cv::Vec3f>(y, x) = cv::normalize(cv::Vec3f(normals_ptr->points[non_NaN_ids.indices[i]].normal_x,
                                          normals_ptr->points[non_NaN_ids.indices[i]].normal_y,
                                          normals_ptr->points[non_NaN_ids.indices[i]].normal_z));
      }

      rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
      rcp.indices.set(rs::conversion::to(tcas, non_NaN_ids));
      rcp.cloud.set(rs::conversion::to(tcas, *cloud_non_nan));
      rcp.normals.set(rs::conversion::to(tcas, *normals_non_nan));
      cas.set(VIEW_CLOUD_NON_NAN, rcp);
    }

    return UIMA_ERR_NONE;
  }

  void compute_normals_pcl(pcl::PointCloud< pcl::PointXYZRGBA>::Ptr &cloud_ptr, pcl::PointCloud< pcl::Normal>::Ptr &normals_ptr)
  {
    pcl::IntegralImageNormalEstimation< pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud_ptr);
    ne.compute(*normals_ptr);
  }

  void compute_normals_unOrganizedCloud(pcl::PointCloud< pcl::PointXYZRGBA>::Ptr &cloud_ptr, pcl::PointCloud< pcl::Normal>::Ptr &normals_ptr)
  {
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setInputCloud(cloud_ptr);
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radiusSearch);
    outInfo("Normal Radius search = " << radiusSearch);
    ne.compute(*normals_ptr);
    outInfo("  Normal Cloud Size: " FG_BLUE << normals_ptr->points.size());
  }

  void filter_NaN_points(pcl::PointCloud< pcl::PointXYZRGBA>::Ptr &in_cloud,
                         pcl::PointCloud< pcl::Normal>::Ptr &in_normals,
                         pcl::PointCloud< pcl::PointXYZRGBA>::Ptr &out_cloud,
                         pcl::PointCloud< pcl::Normal>::Ptr &out_normals,
                         std::vector<int> &non_NaN_ids)
  {
    pcl::removeNaNNormalsFromPointCloud(*in_normals, *out_normals, non_NaN_ids);
    outInfo("Cloud size after filter: " << non_NaN_ids.size());
    pcl::copyPointCloud(*in_cloud, non_NaN_ids, *out_cloud);
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";
    const std::string normalsname = this->name + "_normals";
    pcl::PointCloud< pcl::PointXYZRGBA>::Ptr out_cloud_ptr;
    pcl::PointCloud< pcl::Normal>::Ptr out_normals_ptr;

    switch(pclDispMode)
    {
    case PCL_RGBD:
      out_cloud_ptr = cloud_ptr;
      out_normals_ptr = normals_ptr;
      break;
    case PCL_RGBDT:
      out_cloud_ptr = thermal_cloud_ptr;
      out_normals_ptr = thermal_normals_ptr;
      break;
    }

    if(firstRun)
    {
      visualizer.addPointCloud(out_cloud_ptr, cloudname);
      visualizer.addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(out_cloud_ptr, out_normals_ptr, 50, 0.02f, normalsname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, normalsColor[0], normalsColor[1], normalsColor[2], normalsname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(out_cloud_ptr, cloudname);
      visualizer.removePointCloud(normalsname);
      visualizer.addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(out_cloud_ptr, out_normals_ptr, 50, 0.02f, normalsname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, normalsColor[0], normalsColor[1], normalsColor[2], normalsname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }

  bool callbackKey(const int key, const Source source)
  {
    switch(key)
    {
    case '1':
      pclDispMode = PCL_RGBD;
      break;
    case '2':
      pclDispMode = PCL_RGBDT;
      break;
    }
    return true;
  }

  void drawImageWithLock(cv::Mat &disp)
  {

    cv::Mat normalizedImg;
    cv::normalize(normalsImg_, normalizedImg, 0, 1.0, cv::NORM_MINMAX);    // 0 to 1.0
    cv::Mat flowBgr;
    normalizedImg.convertTo(flowBgr, CV_8UC3, 255.0);

    disp = flowBgr.clone();
  }

};

MAKE_AE(NormalEstimator)
