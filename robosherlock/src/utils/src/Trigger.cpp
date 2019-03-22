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


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Trigger.h>

#include <message_filters/sync_policies/approximate_time.h>

#include <rs/scene_cas.h>
#include <rs/utils/exception.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/io/visualizer.h>

using namespace uima;

class Trigger: public Annotator
{

private:

  bool trigger;
  int last_trigger_value_;
  ros::AsyncSpinner *spinner_;
  ros::NodeHandle *nh_;

  ros::ServiceServer srv_;
  ros::Subscriber joy_sub;

public:

  Trigger(): trigger(false), last_trigger_value_(0)
  {
    if(!ros::ok())
    {
      ros::init(ros::M_string(), std::string("RS_CollectionReader"));
    }
    nh_ = new ros::NodeHandle("~");
    spinner_ = new ros::AsyncSpinner(2);
    joy_sub = nh_->subscribe(std::string("/joy"), 10, &Trigger::joystick_trigger_cb_, this);
    srv_ = nh_->advertiseService("trigger", &Trigger::trigger_service_cb_, this);

    spinner_->start();
    rs::Visualizer::trigger = &trigger;
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    return (TyErrorId) UIMA_ERR_NONE;
  }


  TyErrorId destroy()
  {

    //  delete ros_helper;
    outInfo("destroy");
    spinner_->stop();
    delete spinner_, nh_;
    return (TyErrorId) UIMA_ERR_NONE;
  }


  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process start");

    outInfo("waiting for trigger!");
    while(!(got_trigger()))
    {
      usleep(100);
      check_ros();
    }
    return (TyErrorId) UIMA_ERR_NONE;
  }

  bool trigger_service_cb_(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    outDebug("Triggered from Service");
    trigger = true;
    res.success = true;
    res.message = "Trigger successfull";
    return true;
  }

  void joystick_trigger_cb_(const sensor_msgs::Joy::ConstPtr &joy_msg)
  {
    if(joy_msg->buttons[9] == 1 && last_trigger_value_ == 0)
    {
      outDebug("TRIGGERED FROM JOYSTICK");
      trigger = true;
    }
    last_trigger_value_ = joy_msg->buttons[9];
  }

  bool got_trigger()
  {
    if(trigger)
    {
      outInfo("GOT TRIGGER...PROCESSING!");
      trigger = false;
      return true;
    }

    return false;
  }



};

MAKE_AE(Trigger)
