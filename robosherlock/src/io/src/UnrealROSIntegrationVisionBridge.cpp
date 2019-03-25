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

// RS
#include <rs/io/UnrealROSIntegrationVisionBridge.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>

UnrealROSIntegrationVisionBridge::UnrealROSIntegrationVisionBridge(const boost::property_tree::ptree &pt) : ROSCamInterface(pt), it(nodeHandle)
{
  readConfig(pt);
  initSpinner();
}

UnrealROSIntegrationVisionBridge::~UnrealROSIntegrationVisionBridge()
{
//  spinner.stop();
  delete sync;
  delete rgbImageSubscriber;
  delete depthImageSubscriber;
  delete cameraInfoSubscriber;
}

void UnrealROSIntegrationVisionBridge::initSpinner()
{
  sync = new message_filters::Synchronizer<RGBDSyncPolicy>(RGBDSyncPolicy(5), *rgbImageSubscriber, *depthImageSubscriber, *cameraInfoSubscriber);
  sync->registerCallback(boost::bind(&UnrealROSIntegrationVisionBridge::cb_, this, _1, _2, _3));
//  spinner.start();
}

void UnrealROSIntegrationVisionBridge::readConfig(const boost::property_tree::ptree &pt)
{
  std::string depth_topic = pt.get<std::string>("camera_topics.depth");
  std::string color_topic = pt.get<std::string>("camera_topics.color");
  std::string depth_hints = pt.get<std::string>("camera_topics.depthHints", "raw");
  std::string color_hints = pt.get<std::string>("camera_topics.colorHints", "raw");
  std::string cam_info_topic = pt.get<std::string>("camera_topics.camInfo");

  image_transport::TransportHints hintsColor(color_hints);
  image_transport::TransportHints hintsDepth(depth_hints);

  depthImageSubscriber = new image_transport::SubscriberFilter(it, depth_topic, 5, hintsDepth);
  rgbImageSubscriber = new image_transport::SubscriberFilter(it, color_topic, 5, hintsColor);
  cameraInfoSubscriber = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nodeHandle, cam_info_topic, 5);

  outInfo("  Depth topic: " FG_BLUE << depth_topic);
  outInfo("  Color topic: " FG_BLUE << color_topic);
  outInfo("CamInfo topic: " FG_BLUE << cam_info_topic);
  outInfo("  Depth Hints: " FG_BLUE << depth_hints);
  outInfo("  Color Hints: " FG_BLUE << color_hints);
}

void UnrealROSIntegrationVisionBridge::cb_(const sensor_msgs::Image::ConstPtr rgb_img_msg,
    const sensor_msgs::Image::ConstPtr depth_img_msg,
    const sensor_msgs::CameraInfo::ConstPtr camera_info_msg)
{
  cv::Mat color, depth;
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

  cv_bridge::CvImageConstPtr orig_depth_img;
  orig_depth_img = cv_bridge::toCvShare(depth_img_msg, depth_img_msg->encoding);

  color = orig_rgb_img->image.clone();
  orig_depth_img->image.convertTo(depth, CV_16U, 1000);

  lock.lock();

  this->color = color;
  this->depth = depth;
  this->cameraInfo = cameraInfo;
  this->cameraInfoHD = cameraInfoHD;
  _newData = true;

  lock.unlock();
}


bool UnrealROSIntegrationVisionBridge::setData(uima::CAS &tcas, uint64_t ts)
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
  cameraInfoHD = this->cameraInfoHD;
  _newData = false;
  lock.unlock();

  rs::SceneCas cas(tcas);
  setTransformAndTime(tcas);

  cas.set(VIEW_COLOR_IMAGE_HD, color);
  cas.set(VIEW_COLOR_IMAGE, color);

  cas.set(VIEW_DEPTH_IMAGE_HD, depth);
  cas.set(VIEW_DEPTH_IMAGE, depth);

  cas.set(VIEW_CAMERA_INFO, cameraInfo);
  outInfo("Dist Model" << cameraInfo.distortion_model );

  return true;
}

