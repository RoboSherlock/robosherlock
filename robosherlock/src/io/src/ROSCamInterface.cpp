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
  : CamInterface(pt), /*spinner(0),*/ nodeHandle("~")
{
  listener = new rs::TFListenerProxy();
  tfFrom = pt.get<std::string>("tf.from");
  tfTo = pt.get<std::string>("tf.to");
  lookUpViewpoint = pt.get<bool>("tf.lookupViewpoint", false);
  onlyStableViewpoints = pt.get<bool>("tf.onlyStableViewpoints", true);
  maxViewpointDistance = pt.get<double>("tf.maxViewpointDistance", 0.01);
  maxViewpointRotation = pt.get<double>("tf.maxViewpointRotation", 1.0);

  outInfo("             TF Lookup: " FG_BLUE << (lookUpViewpoint ? "ON" : "OFF"));
  outInfo("               TF From: " FG_BLUE << tfFrom);
  outInfo("                 TF To: " FG_BLUE << tfTo);
  outInfo("Only Stable Viewpoints: " FG_BLUE << onlyStableViewpoints);
  outInfo("Max Viewpoint Distance: " FG_BLUE << maxViewpointDistance);
  outInfo("Max Viewpoint Rotation: " FG_BLUE << maxViewpointRotation);
}

ROSCamInterface::~ROSCamInterface()
{
}

bool ROSCamInterface::lookupTransform(const ros::Time &timestamp)
{
  if(lookUpViewpoint)
  {
    try
    {
      //outDebug("lookup viewpoint: " << timestamp);
      listener->listener->waitForTransform(tfTo, tfFrom, timestamp, ros::Duration(2));
      listener->listener->lookupTransform(tfTo, tfFrom, timestamp, transform);
    }
    catch(tf::TransformException &ex)
    {
      outError(ex.what());
      return false;
    }

    if(onlyStableViewpoints)
    {
      const double distance = lastTransform.getOrigin().distance(transform.getOrigin());
      const double angle = lastTransform.getRotation().angleShortestPath(transform.getRotation()) * 180.0 / M_PI;
      lastTransform = transform;

      //outDebug("viewpoint changes: distance: " << distance << " angle: " << angle);
      if(distance > maxViewpointDistance || angle > maxViewpointRotation)
      {
        outDebug("viewpoint changed!");
        return false;
      }
    }
  }
  return true;
}

void ROSCamInterface::setTransformAndTime(uima::CAS &tcas)
{
  rs::Scene scene = rs::SceneCas(tcas).getScene(this->cam_id_);
  if(lookUpViewpoint)
  {
    rs::StampedTransform vp(rs::conversion::to(tcas, transform));
    scene.viewPoint.set(vp);
    outInfo("added viewpoint to scene");
  }
  scene.timestamp.set(timestamp.toNSec());
}
