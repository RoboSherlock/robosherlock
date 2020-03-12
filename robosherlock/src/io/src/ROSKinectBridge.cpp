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

// ROS
#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>

// RS
#include <robosherlock/io/ROSKinectBridge.h>
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/output.h>
#include <robosherlock/utils/time.h>

//OpenCV
#include <cv_bridge/cv_bridge.h>

ROSKinectBridge::ROSKinectBridge(const boost::property_tree::ptree &pt) : ROSCamInterface(pt), it(nodeHandle)
{
  readConfig(pt);
  initSpinner();
}

ROSKinectBridge::~ROSKinectBridge()
{
//  spinner.stop();
  delete sync;
  delete rgbImageSubscriber;
  delete depthImageSubscriber;
  delete cameraInfoSubscriber;
}

void ROSKinectBridge::initSpinner()
{
  sync = new message_filters::Synchronizer<RGBDSyncPolicy>(RGBDSyncPolicy(5), *rgbImageSubscriber, *depthImageSubscriber, *cameraInfoSubscriber);
  sync->registerCallback(boost::bind(&ROSKinectBridge::cb_, this, _1, _2, _3));
//  spinner.start();
}

void ROSKinectBridge::readConfig(const boost::property_tree::ptree &pt)
{
  std::string depth_topic = pt.get<std::string>("camera_topics.depth");
  std::string color_topic = pt.get<std::string>("camera_topics.color");
  std::string depth_hints = pt.get<std::string>("camera_topics.depthHints", "raw");
  std::string color_hints = pt.get<std::string>("camera_topics.colorHints", "raw");
  std::string cam_info_topic = pt.get<std::string>("camera_topics.camInfo");
  filterBlurredImages = pt.get<bool>("camera.filterBlurredImages", false);
  depthOffset = pt.get<int>("camera.depthOffset", 0);
  scale = pt.get<bool>("camera.scale", true);

  image_transport::TransportHints hintsColor(color_hints);
  image_transport::TransportHints hintsDepth(depth_hints);

  depthImageSubscriber = new image_transport::SubscriberFilter(it, depth_topic, 1, hintsDepth);
  rgbImageSubscriber = new image_transport::SubscriberFilter(it, color_topic, 1, hintsColor);
  cameraInfoSubscriber = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nodeHandle, cam_info_topic, 1);

  outDebug("  Depth topic: " FG_BLUE << depth_topic);
  outDebug("  Color topic: " FG_BLUE << color_topic);
  outDebug("CamInfo topic: " FG_BLUE << cam_info_topic);
  outDebug("  Depth Hints: " FG_BLUE << depth_hints);
  outDebug("  Color Hints: " FG_BLUE << color_hints);
  outDebug("  DepthOffset: " FG_BLUE << depthOffset);
  outDebug("  Blur filter: " FG_BLUE << (filterBlurredImages ? "ON" : "OFF"));
  outDebug("  Scale Input: " FG_BLUE << (scale ? "ON" : "OFF"));
}

void ROSKinectBridge::cb_(const sensor_msgs::Image::ConstPtr rgb_img_msg,
                          const sensor_msgs::Image::ConstPtr depth_img_msg,
                          const sensor_msgs::CameraInfo::ConstPtr camera_info_msg)
{
  //  static int frame = 0;
  //  outWarn("got image: " << frame++);
  cv::Mat color, depth;
  //  bool isHDColor;
  sensor_msgs::CameraInfo cameraInfo, cameraInfoHD;

  cv_bridge::CvImageConstPtr orig_rgb_img;
  orig_rgb_img = cv_bridge::toCvShare(rgb_img_msg, sensor_msgs::image_encodings::BGR8);
  cameraInfo = sensor_msgs::CameraInfo(*camera_info_msg);

  if(!lookupTransform(cameraInfo.header.stamp))
  {
    lock.lock();
    _newData = false;
    lock.unlock();
    return;
  }

  if(filterBlurredImages && detector.detectBlur(orig_rgb_img->image))
  {
    lock.lock();
    _newData = false;
    lock.unlock();
    outWarn("Skipping blurred image!");
    return;
  }
  cv_bridge::CvImageConstPtr orig_depth_img;
  orig_depth_img = cv_bridge::toCvShare(depth_img_msg, depth_img_msg->encoding);

  if(depth_img_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    depth = orig_depth_img->image.clone();
  }
  else if(depth_img_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    orig_depth_img->image.convertTo(depth, CV_16U, 0.001);
  }
  else
  {
    outError("Unknown depth image type!");
    return;
  }

  color = orig_rgb_img->image.clone();
  if(scale)
  {
    if(color.cols == 1280 || color.cols == 1920) // HD or Kinect 2
    {
      //    isHDColor = true;
      if(color.cols == 1280 && color.rows !=720)
      {
        color = color(cv::Rect(0, depthOffset, 1280, 960));
        cameraInfo.K[5] -= depthOffset;
        cameraInfo.height = 960;
      }

      cameraInfoHD = cameraInfo;
      cameraInfo.height /= 2.0;
      cameraInfo.width /= 2.0;
      cameraInfo.roi.height /= 2.0;
      cameraInfo.roi.width /= 2.0;
      cameraInfo.roi.x_offset /= 2.0;
      cameraInfo.roi.y_offset /= 2.0;
      cameraInfo.K[0] /= 2.0;
      cameraInfo.K[2] /= 2.0;
      cameraInfo.K[4] /= 2.0;
      cameraInfo.K[5] /= 2.0;
      cameraInfo.P[0] /= 2.0;
      cameraInfo.P[2] /= 2.0;
      cameraInfo.P[5] /= 2.0;
      cameraInfo.P[6] /= 2.0;
    }
    else if(color.cols == 640)
    {
      //    isHDColor = false;
      cameraInfoHD = cameraInfo;
      cameraInfoHD.height *= 2.0;
      cameraInfoHD.width *= 2.0;
      cameraInfoHD.roi.height *= 2.0;
      cameraInfoHD.roi.width *= 2.0;
      cameraInfoHD.roi.x_offset *= 2.0;
      cameraInfoHD.roi.y_offset *= 2.0;
      cameraInfoHD.K[0] *= 2.0;
      cameraInfoHD.K[2] *= 2.0;
      cameraInfoHD.K[4] *= 2.0;
      cameraInfoHD.K[5] *= 2.0;
      cameraInfoHD.P[0] *= 2.0;
      cameraInfoHD.P[2] *= 2.0;
      cameraInfoHD.P[5] *= 2.0;
      cameraInfoHD.P[6] *= 2.0;
    }
    else if(color.cols == 512)//512*424
    {
      //    isHDColor = false;
      cameraInfoHD = cameraInfo;
      cameraInfoHD.height *= 3.75;
      cameraInfoHD.width *= 3.75;
      cameraInfoHD.roi.height *= 3.75;
      cameraInfoHD.roi.width *= 3.75;
      cameraInfoHD.roi.x_offset *= 3.75;
      cameraInfoHD.roi.y_offset *= 3.75;
      cameraInfoHD.K[0] *= 3.75;
      cameraInfoHD.K[2] *= 3.75;
      cameraInfoHD.K[4] *= 3.75;
      cameraInfoHD.K[5] *= 3.75;
      cameraInfoHD.P[0] *= 3.75;
      cameraInfoHD.P[2] *= 3.75;
      cameraInfoHD.P[5] *= 3.75;
      cameraInfoHD.P[6] *= 3.75;
    }
    else
    {
      outError("Unknown color image size!");
      return;
    }
  }

  lock.lock();

  this->timestamp = cameraInfo.header.stamp;
  this->color = color;
  this->depth = depth;
  this->cameraInfo = cameraInfo;
  if(scale)
  {
    this->cameraInfoHD = cameraInfoHD;
  }
  _newData = true;

  lock.unlock();
}

bool ROSKinectBridge::setData(uima::CAS &tcas, uint64_t ts)
{
  if(!newData())
  {
    return false;
  }
  MEASURE_TIME;

  cv::Mat color, depth;
  sensor_msgs::CameraInfo cameraInfo, cameraInfoHD;

  lock.lock();
  color = this->color;
  depth = this->depth;
  cameraInfo = this->cameraInfo;
  if(scale)
  {
    cameraInfoHD = this->cameraInfoHD;
  }
  _newData = false;
  rs::SceneCas cas(tcas);
  setTransformAndTime(tcas);
  lock.unlock();

  if(scale && color.cols >= 1280)
  {
    cas.set(VIEW_COLOR_IMAGE_HD, color, cam_id_);
    cas.set(VIEW_CAMERA_INFO_HD, cameraInfoHD, cam_id_);
  }
  else
  {
    cas.set(VIEW_COLOR_IMAGE, color, cam_id_);
  }

  if(scale && depth.cols >= 1280)
  {
    cas.set(VIEW_DEPTH_IMAGE_HD, depth, cam_id_);
  }
  else
  {
    cas.set(VIEW_DEPTH_IMAGE, depth, cam_id_);
  }

  cas.set(VIEW_CAMERA_INFO, cameraInfo, cam_id_);

  return true;
}
