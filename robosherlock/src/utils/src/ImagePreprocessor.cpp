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

// UIMA
#include <uima/api.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCV
#include <opencv2/opencv.hpp>

// RS
#include <rs/utils/time.h>
#include <rs/utils/exception.h>
#include <rs/scene_cas.h>
#include <rs/utils/DepthImageProcessing.h>
#include <rs/DrawingAnnotator.h>
#include <rs/segmentation/ImageSegmentation.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

using namespace uima;

class ImagePreprocessor : public DrawingAnnotator
{
private:
  bool hasDepth, hasDepthHD, hasColor, hasColorHD, forceNewCloud, hasThermal, useKinect, useThermal;
  cv::Mat depth, depthHD, color, colorHD, alpha, mask, maskHD;
  cv::Mat thermal, thermalColor, thermalDepth, thermalFused;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, thermalCloud;

  cv::Mat lookupX, lookupY, lookupXThermal, lookupYThermal;

  bool enableDepthSmoothing, enableHoleFilling, thresholdThermalImages;
  int thermalImageThreshold, borderErosion, borderDilation;

  double pointSize;

  enum
  {
    COLOR,
    DEPTH,
    THERMAL,
    THERMAL_RGB,
    THERMAL_DEPTH,
    THERMAL_FUSED,
    RGBD,
    DT,
    MASK
  } displayMode;

  enum
  {
    PCL_RGBD,
    PCL_RGBDT
  } pclDispMode;

  ros::NodeHandle nh_;
  ros::Publisher pub_;

public:
  ImagePreprocessor() : DrawingAnnotator(__func__), borderErosion(6), borderDilation(12),
    pointSize(1), displayMode(RGBD), pclDispMode(PCL_RGBD), nh_("~")
  {
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    thermalCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    lookupX = cv::Mat();
    lookupY = cv::Mat();
    lookupXThermal = cv::Mat();
    lookupYThermal = cv::Mat();
    pub_ = nh_.advertise<sensor_msgs::Image>("input_image", 1, true);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    if(ctx.isParameterDefined("enableDepthSmoothing"))
    {
      ctx.extractValue("enableDepthSmoothing", enableDepthSmoothing);
    }
    if(ctx.isParameterDefined("enableHoleFilling"))
    {
      ctx.extractValue("enableHoleFilling", enableHoleFilling);
    }
    if(ctx.isParameterDefined("useKinect"))
    {
      ctx.extractValue("useKinect", useKinect);
    }
    if(ctx.isParameterDefined("useThermal"))
    {
      ctx.extractValue("useThermal", useThermal);
      if(useThermal)
      {
        if(ctx.isParameterDefined("thresholdThermalImages") && ctx.isParameterDefined("thermalImageThreshold"))
        {
          ctx.extractValue("thresholdThermalImages", thresholdThermalImages);
          ctx.extractValue("thermalImageThreshold", thermalImageThreshold);
        }
      }
    }
    if(ctx.isParameterDefined("borderErosion"))
    {
      ctx.extractValue("borderErosion", borderErosion);
    }
    if(ctx.isParameterDefined("borderDilation"))
    {
      ctx.extractValue("borderDilation", borderDilation);
    }

    // Force rewriting of cloud
    forceNewCloud = enableDepthSmoothing || enableHoleFilling;

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
    outInfo("process start");
    rs::SceneCas cas(tcas);

    if(useKinect)
    {
      processColor(cas);
      processDepth(cas);
      createCloud(cas);
    }
    if(useThermal)
    {
      processThermal(cas);
      createThermalCloud(cas);
    }

    return UIMA_ERR_NONE;
  }

  /*******************************************************************************
   * Visualization
   ******************************************************************************/

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
    case 'b':
    case 'B':
      displayMode = RGBD;
      return true;
    case 'c':
    case 'C':
      displayMode = COLOR;
      return true;
    case 'd':
    case 'D':
      displayMode = DEPTH;
      return true;
    case 't':
    case 'T':
      displayMode = (useThermal ? THERMAL : displayMode);
      return true;
    case 'e':
    case 'E':
      displayMode = (useThermal ? THERMAL_RGB : displayMode);
      return true;
    case 'w':
    case 'W':
      displayMode = (useThermal ? THERMAL_DEPTH : displayMode);
      return true;
    case 'f':
    case 'F':
      displayMode = (useThermal ? THERMAL_FUSED : displayMode);
      return true;
    case 's':
    case 'S':
      displayMode = (useThermal ? DT : displayMode);
      return true;
    case 'm':
    case 'M':
      displayMode = MASK;
      return true;
    }
    return false;
  }

  void dispDepth(cv::Mat &disp)
  {
    cv::Mat tmpDepth;
    switch(displayMode)
    {
    case MASK:
    case COLOR:
    case DEPTH:
    case RGBD:
      tmpDepth = depth.clone();
      break;
    case THERMAL:
    case THERMAL_DEPTH:
    case THERMAL_RGB:
    case THERMAL_FUSED:
    case DT:
      tmpDepth = thermalDepth.clone();
      break;
    }

    if(tmpDepth.empty())
    {
      disp = cv::Mat::zeros(480, 640, CV_16U);
      return;
    }

    cv::Mat tmp = cv::Mat(tmpDepth.rows, tmpDepth.cols, CV_8U);
    const int maxValue = 10000, maxInt = 255;

    #pragma omp parallel for
    for(int r = 0; r < tmpDepth.rows; ++r)
    {
      const uint16_t *itI = tmpDepth.ptr<uint16_t>(r);
      uint8_t *itO = tmp.ptr<uint8_t>(r);

      for(int c = 0; c < tmpDepth.cols; ++c, ++itI, ++itO)
      {
        *itO = (uint8_t)(*itI * maxInt / maxValue);
      }
    }
    cv::applyColorMap(tmp, disp, cv::COLORMAP_JET);
  }

  void combine(cv::Mat &disp)
  {
    cv::Mat tmpColor;
    switch(displayMode)
    {
    case MASK:
    case COLOR:
    case DEPTH:
    case RGBD:
      tmpColor = color.clone();
      break;
    case THERMAL:
    case THERMAL_DEPTH:
    case THERMAL_RGB:
    case THERMAL_FUSED:
    case DT:
      if(useThermal)
      {
        combineThermalDepth(disp);
        return;
      }
      break;
    }
    if(tmpColor.empty())
    {
      disp = cv::Mat::zeros(480, 640, CV_8UC3);
      return;
    }
    cv::Mat tmp;
    disp = cv::Mat(tmpColor.rows, tmpColor.cols, CV_8UC3);
    dispDepth(tmp);

    #pragma omp parallel for
    for(int r = 0; r < disp.rows; ++r)
    {
      const cv::Vec3b *itC = tmpColor.ptr<cv::Vec3b>(r);
      const cv::Vec3b *itD = tmp.ptr<cv::Vec3b>(r);
      cv::Vec3b *itO = disp.ptr<cv::Vec3b>(r);

      for(int c = 0; c < disp.cols; ++c, ++itC, ++itD, ++itO)
      {
        itO->val[0] = (itC->val[0] + itD->val[0]) >> 1;
        itO->val[1] = (itC->val[1] + itD->val[1]) >> 1;
        itO->val[2] = (itC->val[2] + itD->val[2]) >> 1;
      }
    }
  }

  void combineThermalDepth(cv::Mat &disp)
  {
    cv::Mat tmp;
    dispDepth(tmp);
    disp = cv::Mat(tmp.rows, tmp.cols, CV_8UC3);
    #pragma omp parallel for
    for(int r = 0; r < disp.rows; ++r)
    {
      const uint8_t *itC = thermal.ptr<uint8_t>(r);
      const cv::Vec3b *itD = tmp.ptr<cv::Vec3b>(r);
      cv::Vec3b *itO = disp.ptr<cv::Vec3b>(r);

      for(int c = 0; c < disp.cols; ++c, ++itC, ++itD, ++itO)
      {
        itO->val[0] = (itD->val[0] + *itC) >> 1;
        itO->val[1] = (itD->val[1] + *itC) >> 1;
        itO->val[2] = (itD->val[2] + *itC) >> 1;
      }
    }
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    switch(displayMode)
    {
    case COLOR:
      if(!color.empty())
      {
        disp = color.clone();
      }
      else
      {
        disp = cv::Mat::zeros(480, 640, CV_8UC3);
      }
      break;
    case THERMAL_DEPTH:
    case DEPTH:
      dispDepth(disp);
      break;
    case THERMAL:
      if(!thermal.empty())
      {
        disp = thermal.clone();
      }
      else
      {
        disp = cv::Mat::zeros(480, 640, CV_8UC3);
      }
      break;
    case THERMAL_RGB:
      if(!thermalColor.empty())
      {
        disp = thermalColor.clone();
      }
      else
      {
        disp = cv::Mat::zeros(480, 640, CV_8UC3);
      }
      break;
    case THERMAL_FUSED:
      if(!thermalFused.empty())
      {
        disp = thermalFused.clone();
      }
      else
      {
        disp = cv::Mat::zeros(480, 640, CV_8UC3);
      }
      break;
    case RGBD:
    case DT:
      combine(disp);
      break;
    case MASK:
      mask.convertTo(disp, CV_GRAY2BGR);
      break;
    }
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = this->name;
    pcl::PointCloud< pcl::PointXYZRGBA>::Ptr outCloud;

    switch(pclDispMode)
    {
    case PCL_RGBD:
      outCloud = cloud;
      break;
    case PCL_RGBDT:
      outCloud = thermalCloud;
      break;
    }

    if(firstRun)
    {
      visualizer.addPointCloud(outCloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(outCloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }

private:
  /*******************************************************************************
   * Color
   ******************************************************************************/

  void processColor(rs::SceneCas &cas)
  {
    hasColor = cas.get(VIEW_COLOR_IMAGE, color);
    hasColorHD = cas.get(VIEW_COLOR_IMAGE_HD, colorHD);

    if(!hasColor && !hasColorHD)
    {
      outError("No color image in CAS!");
      return;
      //throw rs::Exception("No color image in CAS!");
    }
    else if(hasColor && !hasColorHD)
    {
      outDebug("create HD color.");
      cv::resize(color, colorHD, cv::Size(0, 0), 2.0, 2.0, cv::INTER_AREA);
      cas.set(VIEW_COLOR_IMAGE_HD, colorHD);
      hasColorHD = true;
    }
    else if(!hasColor && hasColorHD)
    {
      outDebug("create color.");
      cv::resize(colorHD, color, cv::Size(0, 0), 0.5, 0.5, cv::INTER_AREA);
      cas.set(VIEW_COLOR_IMAGE, color);
      hasColor = true;
    }
    sensor_msgs::Image image_msg;
    cv_bridge::CvImage cv_image;
    cv_image.image = color;
    cv_image.encoding = "bgr8";
    cv_image.toImageMsg(image_msg);
    pub_.publish(image_msg);
  }

  /*******************************************************************************
   * Depth
   ******************************************************************************/

  void processDepth(rs::SceneCas &cas)
  {
    hasDepth = cas.get(VIEW_DEPTH_IMAGE, depth);
    hasDepthHD = cas.get(VIEW_DEPTH_IMAGE_HD, depthHD);

    if(!hasDepth && !hasDepthHD)
    {
      outError("No depth image in CAS!");
      return;
      //throw rs::Exception("No depth image in CAS!");
    }

    if(hasDepthHD)
    {
      getDepthMask(depthHD, maskHD);
      cas.set(VIEW_MASK_HD, maskHD);
      if(!hasDepth)
      {
        cv::resize(maskHD, mask, cv::Size(0, 0), 0.5, 0.5, cv::INTER_NEAREST);
        cas.set(VIEW_MASK, mask);
      }
    }
    if(hasDepth)
    {
      getDepthMask(depth, mask);
      cas.set(VIEW_MASK, mask);
      if(!hasDepthHD)
      {
        cv::resize(mask, maskHD, cv::Size(0, 0), 2.0, 2.0, cv::INTER_NEAREST);
        cas.set(VIEW_MASK_HD, maskHD);
      }
    }

    if((enableDepthSmoothing || enableHoleFilling) && hasDepthHD)
    {
      filterDepth(depthHD);
      cas.set(VIEW_DEPTH_IMAGE_HD, depthHD);
      hasDepth = false;
    }
    else if(enableDepthSmoothing || enableHoleFilling)
    {
      filterDepth(depth);
      cas.set(VIEW_DEPTH_IMAGE, depth);
    }

    if(!hasDepth)
    {
      outDebug("create depth.");
      cv::resize(depthHD, depth, cv::Size(0, 0), 0.5, 0.5, cv::INTER_NEAREST);
      hasDepth = true;
      cas.set(VIEW_DEPTH_IMAGE, depth);
    }

    if(!hasDepthHD)
    {
      outDebug("create HD depth.");
      cv::resize(depth, depthHD, cv::Size(0, 0), 2.0, 2.0, cv::INTER_NEAREST);
      cas.set(VIEW_DEPTH_IMAGE_HD, depthHD);
      hasDepthHD = true;
    }
  }

  void filterDepth(cv::Mat &depth)
  {
    if(enableDepthSmoothing)
    {
      bilateralSmoothing(depth);
    }

    if(enableHoleFilling)
    {
      rs::DepthImageProcessing::fillHoles(depth);
    }
  }

  void getDepthMask(cv::Mat &depth, cv::Mat &mask)
  {
    mask.create(depth.rows, depth.cols, CV_8U);
    cv::Mat maskBorder = cv::Mat(depth.rows, depth.cols, CV_8U);

    const size_t size = depth.rows * depth.cols;
    uint16_t *itD = depth.ptr<uint16_t>();
    uint8_t *itM = mask.ptr<uint8_t>();
    uint8_t *itB = maskBorder.ptr<uint8_t>();

    for(size_t i = 0; i < size; ++i, ++itD, ++itM, ++itB)
    {
      if(*itD == 0)
      {
        *itB = 255;
        *itM = 0;
      }
      else if(*itD >= 20000)
      {
        *itM = 255;
        *itB = 0;
        *itD = 0;
      }
      else
      {
        *itM = 0;
        *itB = 0;
      }
    }

    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,  cv::Size(3, 3));
    cv::erode(maskBorder, maskBorder, kernel, cv::Point(-1, -1), borderErosion, cv::BORDER_CONSTANT, 255);

    const cv::Rect roi(0, 0, depth.cols, depth.rows);
    std::vector<ImageSegmentation::Segment> segments;
    ImageSegmentation::segment(maskBorder, segments, 40000, 10000, roi);

    maskBorder = cv::Mat::zeros(depth.rows, depth.cols, CV_8U);
    for(size_t i = 0; i < segments.size(); ++i)
    {
      ImageSegmentation::drawSegment(maskBorder, CV_RGB(255, 255, 255), CV_RGB(0, 0, 0), segments[i]);
    }

    cv::dilate(maskBorder, maskBorder, kernel, cv::Point(-1, -1), borderDilation, cv::BORDER_CONSTANT, 255);
    cv::dilate(mask, mask, kernel, cv::Point(-1, -1), borderDilation / 2, cv::BORDER_CONSTANT, 0);

    itM = mask.ptr<uint8_t>();
    itB = maskBorder.ptr<uint8_t>();

    for(size_t i = 0; i < size; ++i, ++itM, ++itB)
    {
      if(*itB)
      {
        *itM = 255;
      }
    }
  }

  /*******************************************************************************
   * Cloud
   ******************************************************************************/

  void createCloud(rs::SceneCas &cas)
  {
    if(lookupX.empty())
    {
      sensor_msgs::CameraInfo camInfo;
      if(!cas.get(VIEW_CAMERA_INFO, camInfo))
      {
        return;
      }
      createLookup(camInfo, lookupX, lookupY);
      alpha = cv::Mat::zeros(camInfo.height, camInfo.width, CV_8U);
    }

    if((forceNewCloud || !cas.has(VIEW_CLOUD)) && hasDepth && hasColor)
    {
      outDebug("create point cloud.");
      rs::DepthImageProcessing::project(depth, color, alpha, lookupX, lookupY, cloud);
      sensor_msgs::CameraInfo camInfo;
      cas.get(VIEW_CAMERA_INFO, camInfo);
      pcl_conversions::toPCL(camInfo.header,cloud->header);
      cas.set(VIEW_CLOUD, *cloud);
    }
  }

  /*******************************************************************************
   * Thermal
   ******************************************************************************/

  void processThermal(rs::SceneCas &cas)
  {
    hasThermal = true;
    hasThermal = hasThermal && cas.get(VIEW_THERMAL_IMAGE, thermal);
    hasThermal = hasThermal && cas.get(VIEW_THERMAL_COLOR_IMAGE, thermalColor);
    hasThermal = hasThermal && cas.get(VIEW_THERMAL_DEPTH_IMAGE, thermalDepth);

    if(hasThermal)
    {
      filterDepth(thermalDepth);
      if(thresholdThermalImages)
      {
        thresholdRGBDT(thermalColor, thermal, thermalDepth, thermalImageThreshold);
        cas.set(VIEW_THERMAL_IMAGE, thermal);
        cas.set(VIEW_THERMAL_DEPTH_IMAGE, thermalDepth);
        cas.set(VIEW_THERMAL_COLOR_IMAGE, thermalColor);
      }

      fuseRGBT(thermalColor, thermal, thermalFused);
      cas.set(VIEW_THERMAL_FUSED, thermalFused);
    }
  }

  void createThermalCloud(rs::SceneCas &cas)
  {
    if(lookupXThermal.empty())
    {
      sensor_msgs::CameraInfo camInfo;
      if(!cas.get(VIEW_CAMERA_INFO, camInfo))
      {
        return;
      }
      createLookup(camInfo, lookupXThermal, lookupYThermal);
    }

    if(hasThermal && (thresholdThermalImages || forceNewCloud || !cas.has(VIEW_CLOUD)))
    {
      rs::DepthImageProcessing::project(thermalDepth, thermalColor, thermal, lookupXThermal, lookupYThermal, thermalCloud);
      cas.set(VIEW_THERMAL_CLOUD, *thermalCloud);
    }
  }

  /*******************************************************************************
   * Image processing
   ******************************************************************************/

  void createLookup(const sensor_msgs::CameraInfo &camInfo, cv::Mat &lookupX, cv::Mat &lookupY)
  {
    const float fx = 1.0f / camInfo.K[0];
    const float fy = 1.0f / camInfo.K[4];
    const float cx = camInfo.K[2];
    const float cy = camInfo.K[5];
    float *it;

    lookupY = cv::Mat(1, camInfo.height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < camInfo.height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, camInfo.width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < camInfo.width; ++c, ++it)
    {
      *it = (c - cx) * fx;
    }
  }

  void bilateralSmoothing(const cv::Mat &image)
  {
    cv::Mat in, out;
    image.convertTo(in, CV_32F, 0.001);
    cv::bilateralFilter(in, out, -1, 5, 25, 3);
    out.convertTo(image, CV_16U, 1000);
  }

  void thresholdRGBDT(cv::Mat &color, cv::Mat &thermal, cv::Mat &depth, const int threshold)
  {
    const cv::Vec3b invalid(0, 0, 0);

    #pragma omp parallel for
    for(size_t y = 0; y < (size_t)depth.rows; ++y)
    {
      uchar *itT = thermal.ptr<uchar>(y);
      uint16_t *itD = depth.ptr<uint16_t>(y);
      cv::Vec3b *itC = color.ptr<cv::Vec3b>(y);
      for(size_t x = 0; x < (size_t)depth.cols; ++x, ++itT, ++itD, ++itC)
      {
        if(*itT < threshold)
        {
          *itT = 0;
          *itD = 0;
          *itC = invalid;
        }
      }
    }
  }

  void fuseRGBT(const cv::Mat &color, const cv::Mat &thermal, cv::Mat &fused)
  {
    cv::Mat grey, equalizedGrey, equalizedThermal;
    cv::cvtColor(color, grey, CV_BGR2GRAY);
    cv::equalizeHist(grey, equalizedGrey);
    cv::equalizeHist(thermal, equalizedThermal);

    fused = (0.2 * equalizedGrey) + (0.8 * equalizedThermal);
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ImagePreprocessor)

