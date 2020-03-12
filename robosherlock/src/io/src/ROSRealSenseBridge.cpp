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
#include <robosherlock/io/ROSRealSenseBridge.h>
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/output.h>
#include <robosherlock/utils/time.h>

//OpenCV
#include <cv_bridge/cv_bridge.h>

ROSRealSenseBridge::ROSRealSenseBridge(const boost::property_tree::ptree &pt) : ROSCamInterface(pt), it(nodeHandle)
{
  readConfig(pt);
  initSpinner();
}

ROSRealSenseBridge::~ROSRealSenseBridge()
{
//  spinner.stop();
  delete sync;
  delete rgbImageSubscriber;
  delete depthImageSubscriber;
  delete cameraInfoSubscriber;
}

void ROSRealSenseBridge::initSpinner()
{
  sync = new message_filters::Synchronizer<RGBDSyncPolicy>(RGBDSyncPolicy(5), *rgbImageSubscriber, *depthImageSubscriber, *cameraInfoSubscriber);
  sync->registerCallback(boost::bind(&ROSRealSenseBridge::cb_, this, _1, _2, _3));
//  spinner.start();
}

void ROSRealSenseBridge::readConfig(const boost::property_tree::ptree &pt)
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

  outInfo("  Depth topic: " FG_BLUE << depth_topic);
  outInfo("  Color topic: " FG_BLUE << color_topic);
  outInfo("CamInfo topic: " FG_BLUE << cam_info_topic);
  outInfo("  Depth Hints: " FG_BLUE << depth_hints);
  outInfo("  Color Hints: " FG_BLUE << color_hints);
  outInfo("  DepthOffset: " FG_BLUE << depthOffset);
  outInfo("  Blur filter: " FG_BLUE << (filterBlurredImages ? "ON" : "OFF"));
  outInfo("  Scale Input: " FG_BLUE << (scale ? "ON" : "OFF"));
}

void ROSRealSenseBridge::cb_(const sensor_msgs::Image::ConstPtr rgb_img_msg,
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

  if(!lookupTransform(cameraInfo.header.stamp)) {
    lock.lock();
    _newData = false;
    lock.unlock();
    return;
  }

  if(filterBlurredImages && detector.detectBlur(orig_rgb_img->image)) {
    lock.lock();
    _newData = false;
    lock.unlock();
    outWarn("Skipping blurred image!");
    return;
  }
  cv_bridge::CvImageConstPtr orig_depth_img;
  orig_depth_img = cv_bridge::toCvShare(depth_img_msg, depth_img_msg->encoding);

  if(depth_img_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    depth = orig_depth_img->image.clone();
  }
  else if(depth_img_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    orig_depth_img->image.convertTo(depth, CV_16U, 0.001);
  }
  else {
    outError("Unknown depth image type!");
    return;
  }

  if(depth.cols != 1280 && depth.cols != 848) {
    outError("Depth resolution error! Depth res is: "<<depth.cols<<"x"<<depth.rows<<" height of depth image needs to be 1280x720 or 848x480");
  }
  color = orig_rgb_img->image.clone();
  if(scale) {
    if(color.cols == 1280 || color.cols == 848) {
      cameraInfoHD = cameraInfo;
      cameraInfoHD.height *= 1.5;
      cameraInfoHD.width  *= 1.5;
      cameraInfoHD.roi.height  *= 1.5;
      cameraInfoHD.roi.width *= 1.5;
      cameraInfoHD.roi.x_offset  *= 1.5;
      cameraInfoHD.roi.y_offset  *= 1.5;
      cameraInfoHD.K[0]  *= 1.5;
      cameraInfoHD.K[2]  *= 1.5;
      cameraInfoHD.K[4]  *= 1.5;
      cameraInfoHD.K[5]  *= 1.5;
      cameraInfoHD.P[0]  *= 1.5;
      cameraInfoHD.P[2]  *= 1.5;
      cameraInfoHD.P[5]  *= 1.5;
      cameraInfoHD.P[6]  *= 1.5;
    }
    else if(color.cols == 1920) {
      cameraInfoHD = cameraInfo;
      cameraInfo.height /= 1.5;
      cameraInfo.width /= 1.5;
      cameraInfo.roi.height /= 1.5;
      cameraInfo.roi.width /= 1.5;
      cameraInfo.roi.x_offset /= 1.5;
      cameraInfo.roi.y_offset /= 1.5;
      cameraInfo.K[0] /= 1.5;
      cameraInfo.K[2] /= 1.5;
      cameraInfo.K[4] /= 1.5;
      cameraInfo.K[5] /= 1.5;
      cameraInfo.P[0] /= 1.5;
      cameraInfo.P[2] /= 1.5;
      cameraInfo.P[5] /= 1.5;
      cameraInfo.P[6] /= 1.5;
    }
    else {
      outError("Unknown color image size! For realsense Cameras use HD or FullHD color resolution");
      return;
    }
  }

  lock.lock();

  this->timestamp = cameraInfo.header.stamp;
  this->color = color;
  this->depth = depth;
  this->cameraInfo = cameraInfo;
  if(scale) {
    this->cameraInfoHD = cameraInfoHD;
  }
  _newData = true;

  lock.unlock();
}

bool ROSRealSenseBridge::setData(uima::CAS &tcas, uint64_t ts)
{
  if(!newData()) {
    return false;
  }
  MEASURE_TIME;

  cv::Mat color, depth;
  sensor_msgs::CameraInfo cameraInfo, cameraInfoHD;

  lock.lock();
  color = this->color;
  depth = this->depth;
  cameraInfo = this->cameraInfo;
  if(scale) {
    cameraInfoHD = this->cameraInfoHD;
  }
  _newData = false;
  rs::SceneCas cas(tcas);
  cas.setActiveCamId(this->cam_id_);
  setTransformAndTime(tcas);
  lock.unlock();
  if(color.cols == 1920) {
    cas.set(VIEW_COLOR_IMAGE_HD, color);
    cv::resize(color, color, cv::Size(), 0.66, 0.66, cv::INTER_AREA);
    cas.set(VIEW_COLOR_IMAGE, color);
  }
  else if(color.cols == 1280 || color.cols == 848) {
    cas.set(VIEW_COLOR_IMAGE, color);
    cv::resize(color, color, cv::Size(), 1.5, 1.5, cv::INTER_AREA);
    cas.set(VIEW_COLOR_IMAGE_HD, color);
  }

  cas.set(VIEW_DEPTH_IMAGE, depth);
  cv::resize(depth, depth, cv::Size(), 1.5, 1.5, cv::INTER_AREA);
  cas.set(VIEW_DEPTH_IMAGE_HD, depth);

  cas.set(VIEW_CAMERA_INFO_HD, cameraInfoHD);
  cas.set(VIEW_CAMERA_INFO, cameraInfo);

  return true;
}
