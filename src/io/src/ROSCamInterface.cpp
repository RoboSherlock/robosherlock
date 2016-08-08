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

#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/io/ROSCamInterface.h>

// Implementation

ROSCamInterface::ROSCamInterface(const boost::property_tree::ptree &pt)
  : CamInterface(pt), lookUpViewpoint(false), spinner(0), nodeHandle("~")
{
  listener = new tf::TransformListener(nodeHandle,ros::Duration(10.0));
  tfFrom = pt.get<std::string>("tf.from");
  tfTo = pt.get<std::string>("tf.to");
  lookUpViewpoint = pt.get<bool>("tf.lookupViewpoint");

  outInfo("TF Lookup: " FG_BLUE << (lookUpViewpoint ? "ON" : "OFF"));
  outInfo("TF From:   " FG_BLUE << tfFrom);
  outInfo("TF To:     " FG_BLUE << tfTo);
}

ROSCamInterface::~ROSCamInterface()
{
  delete listener;
}
