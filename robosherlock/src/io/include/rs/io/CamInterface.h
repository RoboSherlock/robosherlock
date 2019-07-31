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
  static int id;

  CamInterface(const boost::property_tree::ptree& pt) : _newData(false)
  {
    cam_id_ = id++;
    outInfo("New Camera ID: " FG_BLUE << cam_id_);
    rs::SceneCas::cam_ids_.push_back(cam_id_);
  }

public:
  virtual ~CamInterface()
  {
    std::vector<int>::iterator it = std::find(rs::SceneCas::cam_ids_.begin(), rs::SceneCas::cam_ids_.end(), cam_id_);
    if (it != rs::SceneCas::cam_ids_.end())
      rs::SceneCas::cam_ids_.erase(it);
  }

  bool newData() const
  {
    return _newData;
  }

  static void resetIdCount()
  {
    id = 0;
  }

  virtual bool setData(uima::CAS& tcas, uint64_t ts = 0) = 0;
};
#endif  // __CAM_INTERFACE_H__
