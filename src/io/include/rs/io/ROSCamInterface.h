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

#ifndef __ROS_CAM_INTERFACE2_H__
#define __ROS_CAM_INTERFACE2_H__

// UIMA
#include <uima/api.hpp>

// ROS
#include <tf/tf.h>
#include <tf/transform_listener.h>

// RS
#include <rs/io/CamInterface.h>

// STL
#include <mutex>

class ROSCamInterface : public CamInterface
{
private:

protected:
  tf::TransformListener *listener;
  std::string tfFrom, tfTo;
  bool lookUpViewpoint, onlyStableViewpoints;
  ros::AsyncSpinner spinner;
  ros::NodeHandle nodeHandle;
  tf::StampedTransform transform, lastTransform;
  ros::Time timestamp;
  double maxViewpointDistance, maxViewpointRotation;

  std::mutex lock;

public:
  ~ROSCamInterface();

protected:
  ROSCamInterface(const boost::property_tree::ptree &pt);

  bool lookupTransform(const ros::Time &timestamp);
  void setTransformAndTime(uima::CAS &tcas);
};

#endif // __ROS_CAM_INTERFACE2_H__
