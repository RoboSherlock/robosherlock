/*
 * Copyright (c) 2012, Christian Kerl <christian.kerl@in.tum.de
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

// This is based on DaveDetector.cpp from the uimacpp examples, which is licensed under the Apache License.

#include <uima/api.hpp>

#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>

#include <rs/DrawingAnnotator.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>

using namespace uima;

class NormalEstimator : public DrawingAnnotator
{
private:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr thermal_cloud_ptr;
  pcl::PointCloud<pcl::Normal>::Ptr normals_ptr;
  pcl::PointCloud<pcl::Normal>::Ptr thermal_normals_ptr;

  double pointSize;
  double normalsColor[3];

private:
  bool useThermal, useRGB;

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

    // create necessary pcl objects
    cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    thermal_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    normals_ptr.reset(new pcl::PointCloud<pcl::Normal>);
    thermal_normals_ptr.reset(new pcl::PointCloud<pcl::Normal>);

    // create scene cas wrapper for cas and get kinect frame
    rs::SceneCas cas(tcas);
    if(useThermal && cas.get(VIEW_THERMAL_CLOUD, *thermal_cloud_ptr))
    {
      compute_normals_pcl(thermal_cloud_ptr, thermal_normals_ptr);
      cas.set(VIEW_THERMAL_NORMALS, *thermal_normals_ptr);
    }
    if(useRGB && cas.get(VIEW_CLOUD, *cloud_ptr))
    {
      compute_normals_pcl(cloud_ptr, normals_ptr);
      cas.set(VIEW_NORMALS, *normals_ptr);
    }

    return UIMA_ERR_NONE;
  }

  void compute_normals_pcl(pcl::PointCloud< pcl::PointXYZRGBA>::Ptr &cloud_ptr, pcl::PointCloud< pcl::Normal>::Ptr &normals_ptr)
  {
    pcl::IntegralImageNormalEstimation< pcl::PointXYZRGBA, pcl::Normal> ne;
    //COVARIANCE_MATRIX needed to get curvature
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud_ptr);
    ne.compute(*normals_ptr);
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
};

MAKE_AE(NormalEstimator)
