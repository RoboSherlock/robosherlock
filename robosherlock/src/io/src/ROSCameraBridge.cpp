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
#include <rs/io/ROSCameraBridge.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>

//OpenCV
#include <cv_bridge/cv_bridge.h>

ROSCameraBridge::ROSCameraBridge(const boost::property_tree::ptree &pt) : ROSCamInterface(pt), it(nodeHandle)
{
  readConfig(pt);
  initSpinner();
}

ROSCameraBridge::~ROSCameraBridge()
{
//  spinner.stop();
  delete sync;
  delete rgbImageSubscriber;
  delete cameraInfoSubscriber;
}

void ROSCameraBridge::initSpinner()
{
  sync = new message_filters::Synchronizer<RGBSyncPolicy>(RGBSyncPolicy(10), *rgbImageSubscriber, *cameraInfoSubscriber);
  sync->registerCallback(boost::bind(&ROSCameraBridge::cb_, this, _1, _2));
//  spinner.start();
}

void ROSCameraBridge::readConfig(const boost::property_tree::ptree &pt)
{
  std::string color_topic = pt.get<std::string>("camera_topics.color");
  boost::optional<std::string> color_hints = pt.get_optional<std::string>("camera_topics.colorHints");
  std::string cam_info_topic = pt.get<std::string>("camera_topics.camInfo");
  filterBlurredImages = pt.get<bool>("camera.filterBlurredImages");

  image_transport::TransportHints hintsColor(color_hints ? color_hints.get() : "raw");
  rgbImageSubscriber = new image_transport::SubscriberFilter(it, color_topic, 1, hintsColor);

  cameraInfoSubscriber = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nodeHandle, cam_info_topic, 1);

  outInfo("  Color topic: " FG_BLUE << color_topic);
  outInfo("CamInfo topic: " FG_BLUE << cam_info_topic);
  outInfo("  Color Hints: " FG_BLUE << color_hints);
  outInfo("  Blur filter: " FG_BLUE << (filterBlurredImages ? "ON" : "OFF"));
}

void ROSCameraBridge::cb_(const sensor_msgs::Image::ConstPtr rgb_img_msg,
                          const sensor_msgs::CameraInfo::ConstPtr camera_info_msg)
{
  std::lock_guard<std::mutex> lock_guard(lock);
  cv::Mat color;
  sensor_msgs::CameraInfo cameraInfo;

  cv_bridge::CvImageConstPtr orig_rgb_img;
  orig_rgb_img = cv_bridge::toCvShare(rgb_img_msg, sensor_msgs::image_encodings::BGR8);
  cameraInfo = sensor_msgs::CameraInfo(*camera_info_msg);

  if(!lookupTransform(cameraInfo.header.stamp)) {
    _newData = false;
    return;
  }

  if(filterBlurredImages && detector.detectBlur(orig_rgb_img->image)) {
    _newData = false;
    outWarn("Skipping blurred image!");
    return;
  }

  color = orig_rgb_img->image.clone();
  this->timestamp = cameraInfo.header.stamp;
  this->color = color;
  this->cameraInfo = cameraInfo;
  _newData = true;
}

bool ROSCameraBridge::setData(uima::CAS &tcas, uint64_t ts)
{
  if(!newData()) {
    return false;
  }
  MEASURE_TIME;

  std::lock_guard<std::mutex> lock_guard(lock);
  cv::Mat color;
  sensor_msgs::CameraInfo cameraInfo;
  rs::SceneCas cas(tcas);

  color = this->color;
  cameraInfo = this->cameraInfo;
  _newData = false;
  cas.setActiveCamId(this->cam_id_);
  setTransformAndTime(tcas);


  cas.set(VIEW_COLOR_IMAGE_HD, color);
  cas.set(VIEW_COLOR_IMAGE, color);

  cas.set(VIEW_CAMERA_INFO, cameraInfo);
  cas.set(VIEW_CAMERA_INFO_HD, cameraInfo);

  return true;
}
