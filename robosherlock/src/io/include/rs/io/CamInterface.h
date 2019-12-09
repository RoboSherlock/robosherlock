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

#ifndef __CAM_INTERFACE_H__
#define __CAM_INTERFACE_H__

// UIMA
#include <uima/api.hpp>

// Boost
#include <boost/property_tree/ptree.hpp>

// RS
#include <rs/utils/output.h>
#include <rs/scene_cas.h>

class CamInterface
{
protected:
  bool _newData;

  int cam_id_;
//  static int id;
  bool camera_id_registered_ = false;
  std::string aae_name_;

  CamInterface(const boost::property_tree::ptree& pt) : _newData(false)
  {
    // TODO still something todo here?
//    cam_id_ = id++;
//    outInfo("New Camera ID: " FG_BLUE << cam_id_);
//    rs::SceneCas::cam_ids_.push_back(cam_id_);
  }

public:
  virtual ~CamInterface()
  {
    if(camera_id_registered_)
      rs::SceneCas::unregisterCameraInCAS(aae_name_, cam_id_);
  }

  bool newData() const
  {
    return _newData;
  }

  inline int getCameraId(){
    return cam_id_;
  }

  inline bool cameraIdAlreadyRegistered(){
    return camera_id_registered_;
  }

//  static void resetIdCount()
//  {
//    id = 0;
//  }

  void registerCameraInCAS(std::string analysis_engine_name) {
    cam_id_ = rs::SceneCas::registerCameraInCAS(analysis_engine_name);
    camera_id_registered_ = true;
  }

  virtual bool setData(uima::CAS& tcas, uint64_t ts = 0) = 0;
};
#endif  // __CAM_INTERFACE_H__
