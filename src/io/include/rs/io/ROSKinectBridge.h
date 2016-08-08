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

#ifndef __ROS_KINECT_BRIDGE_H__
#define __ROS_KINECT_BRIDGE_H__

// ROS
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/image_transport.h>

//OpenCV
#include <opencv2/opencv.hpp>

// RS
#include <rs/io/ROSCamInterface.h>
#include <rs/utils/BlurDetector.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> RGBDSyncPolicy;

class ROSKinectBridge : public ROSCamInterface
{
private:
  bool filterBlurredImages;
  bool scale;
  BlurDetector detector;

  void initSpinner();
  void readConfig(const boost::property_tree::ptree &pt);

  message_filters::Synchronizer<RGBDSyncPolicy> *sync;
  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *rgbImageSubscriber;
  image_transport::SubscriberFilter *depthImageSubscriber;

  message_filters::Subscriber<sensor_msgs::CameraInfo> *cameraInfoSubscriber;

  void cb_(const sensor_msgs::Image::ConstPtr rgb_img_msg,
           const sensor_msgs::Image::ConstPtr depth_img_msg,
           const sensor_msgs::CameraInfo::ConstPtr camera_info_msg);

  cv::Mat color;
  cv::Mat depth;

  sensor_msgs::CameraInfo cameraInfo;
  sensor_msgs::CameraInfo cameraInfoHD;

  int depthOffset;

public:
  ROSKinectBridge(const boost::property_tree::ptree &pt);
  ~ROSKinectBridge();

  bool setData(uima::CAS &tcas, u_int64_t = 0);
  inline void getColorImage(cv::Mat& c)
  {
    c = this->color.clone();
  }

  inline void getDepthImage(cv::Mat& d)
  {
    d = this->depth.clone();
  }

  void lookupTransform(uima::CAS &tcas, const ros::Time &timestamp);
};

#endif // __ROS_KINECT_BRIDGE_H__
