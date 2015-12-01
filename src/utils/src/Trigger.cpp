/*
 * Copyright (c) 2011, Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
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


#include "uima/api.hpp"


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>

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
    spinner_ = new ros::AsyncSpinner(0);
    joy_sub = nh_->subscribe(std::string("/joy"), 10, &Trigger::joystick_trigger_cb_, this);
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
