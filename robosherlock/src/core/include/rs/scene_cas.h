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
#ifndef SCENE_CAS_H_
#define SCENE_CAS_H_

#include <map>
#include <mutex>
#include <memory>

#include <uima/api.hpp>

#include <sensor_msgs/CameraInfo.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

#include <opencv2/opencv.hpp>

#include <boost/tuple/tuple.hpp>

#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>

#include <rs/types/all_types.h>
#include <rs/conversion/conversion.h>
#include <rs/view_names.h>

namespace rs
{
// typedef std::map<std::string, std::vector<rs::SemanticMapObject>> SemanticMap;

class SceneCas
{
private:
  uima::CAS& cas;
  std::shared_ptr<std::mutex> mutex;

public:
  SceneCas(uima::CAS& cas);
  virtual ~SceneCas();

  static std::vector<int> cam_ids_;
  int active_cam_id_;

  rs::Scene getScene(int cam_id = -1);

  bool has(const char* name);
  bool getFS(const char* name, uima::FeatureStructure& fs);
  void setFS(const char* name, const uima::FeatureStructure& fs);

  void reset()
  {
    std::lock_guard<std::mutex> lock(*mutex);
    cas.reset();
  }

  void setText(std::string t)
  {
    std::lock_guard<std::mutex> lock(*mutex);
    cas.setDocumentText(uima::UnicodeStringRef(t.c_str()));
  }

  void setActiveCamId(int id)
  {
    if (std::find(cam_ids_.begin(), cam_ids_.end(), id) != cam_ids_.end())
      active_cam_id_ = id;
    else
        throw std::runtime_error("Settin an cam id that does not exist");
  }

  template <class T>
  bool get(const char* name, T& output, int cam_id = -1)
  {
    std::stringstream ss;
    ss << name << "_" <<(cam_id==-1 ? std::to_string(active_cam_id_): std::to_string(cam_id));
    outInfo("Getting: "<<ss.str());
    return get(ss.str().c_str(), output, std::is_base_of<rs::FeatureStructureProxy, T>());
  }

  template <class T>
  void set(const char* name, const T& input, int cam_id = -1)
  {
    std::stringstream ss;
    ss << name << "_" << (cam_id==-1 ? std::to_string(active_cam_id_): std::to_string(cam_id));
    set(ss.str().c_str(), input, std::is_base_of<rs::FeatureStructureProxy, T>());
  }

  template <class T>
  bool get(const char* name, std::vector<T>& output, int cam_id = -1)
  {
    std::stringstream ss;
    ss << name << "_" << (cam_id==-1 ? std::to_string(active_cam_id_): std::to_string(cam_id));
    outInfo("Getting: "<<ss.str());
    return get(ss.str().c_str(), output, std::is_base_of<rs::FeatureStructureProxy, T>());
  }

  template <class T>
  void set(const char* name, const std::vector<T>& input, int cam_id = -1)
  {
    std::stringstream ss;
    ss << name << "_" << (cam_id==-1 ? std::to_string(active_cam_id_): std::to_string(cam_id));
    set(ss.str().c_str(), input, std::is_base_of<rs::FeatureStructureProxy, T>());
  }

private:
  bool getView(const char* name, uima::CAS*& view);

  template <class T>
  bool get(const char* name, T& output, const std::true_type&)
  {
    uima::FeatureStructure fs;
    if (getFS(name, fs))
    {
      output = T(fs);
      return true;
    }
    return false;
  }

  template <class T>
  bool get(const char* name, T& output, const std::false_type&)
  {
    uima::FeatureStructure fs;
    if (getFS(name, fs))
    {
      rs::conversion::from(fs, output);
      return true;
    }
    return false;
  }

  template <class T>
  void set(const char* name, const T& input, const std::true_type&)
  {
    setFS(name, (uima::FeatureStructure)input);
  }

  template <class T>
  void set(const char* name, const T& input, const std::false_type&)
  {
    setFS(name, conversion::to(cas, input));
  }

  template <class T>
  bool get(const char* name, std::vector<T>& output, const std::true_type&)
  {
    output.clear();
    uima::FeatureStructure fs;

    if (!getFS(name, fs))
    {
      return false;
    }

    uima::ArrayFS array(fs);
    output.reserve(array.size());

    for (size_t i = 0; i < array.size(); ++i)
    {
      output.push_back(T(array.get(i)));
    }
    return true;
  }

  template <class T>
  bool get(const char* name, std::vector<T>& output, const std::false_type&)
  {
    uima::FeatureStructure fs;

    if (!getFS(name, fs))
    {
      return false;
    }

    uima::ArrayFS array(fs);
    output.resize(array.size());

    for (size_t i = 0; i < array.size(); ++i)
    {
      rs::conversion::from(array.get(i), output[i]);
    }
    return true;
  }

  template <class T>
  void set(const char* name, const std::vector<T>& input, const std::true_type&)
  {
    uima::ArrayFS array = cas.createArrayFS(input.size());

    for (int i = 0; i < input.size(); ++i)
    {
      array.set(i, (uima::FeatureStructure)input[i]);
    }
    setFS(name, array);
  }

  template <class T>
  void set(const char* name, const std::vector<T>& input, const std::false_type&)
  {
    uima::ArrayFS array = cas.createArrayFS(input.size());

    for (int i = 0; i < input.size(); ++i)
    {
      array.set(i, conversion::to(cas, input[i]));
    }
    setFS(name, array);
  }
};

} /* namespace rs */
#endif /* SCENE_CAS_H_ */
