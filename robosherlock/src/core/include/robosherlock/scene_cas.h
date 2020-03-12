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

#include <robosherlock/types/all_types.h>
#include <robosherlock/conversion/conversion.h>
#include <robosherlock/view_names.h>

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

//  static std::vector<int> cam_ids_;
  int active_cam_id_;

  static std::map<std::string, std::vector<int>> camera_ids_in_aae_;

  rs::Scene getScene(int cam_id = -1);

  bool has(const char* name);
  bool hasObjets();

  bool getFS(const char* name, uima::FeatureStructure& fs);
  void setFS(const char* name, const uima::FeatureStructure& fs);

  // Register a new camera into the global mapping:
  //    AAE_Name -> list_of_camera_ids
  // Camera IDs are not globally unique. They are only unique in the AAE they have been loaded in
  // TODO: This should be prettier in the sense that the returned camera id will not just be the size
  //       of the list, but rather the first free camera id.
  // TODO: The current state of methods will also be imperfect if only some of the loaded CamInterfaces are deleted
  //       and then new ones are added.
  // TODO: Have a mutex when going for multithreading
  static int registerCameraInCAS(std::string analysis_engine_name){
    // Append camera id and check for camera id
    int current_id_to_assign = camera_ids_in_aae_[analysis_engine_name].size();
    camera_ids_in_aae_[analysis_engine_name].push_back(current_id_to_assign);
    return current_id_to_assign;
  }

  // Returns true if camera id has been found , else otherwise
  // TODO: Have a mutex when going for multithreading
  static bool unregisterCameraInCAS(std::string analysis_engine_name, int camera_id){
    auto cams_in_ae = camera_ids_in_aae_[analysis_engine_name];

    std::vector<int>::iterator it = std::find(cams_in_ae.begin(), cams_in_ae.end(), camera_id);
    if (it != cams_in_ae.end()) {
      cams_in_ae.erase(it);
      return true;
    }
    return false;
  }

  // This method will completly clear the ae -> camera_ids mapping
  static inline void unregisterAllCameraIDs(){
    camera_ids_in_aae_.clear();
  }

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

  /**
   * Get the identifier for this CAS.
   *
   * Currently this reflects the name of the responsible AAE that uses this->cas.
   */
  inline std::string getIdentifier(){
    return cas.getIdentifier();
  }

  void setActiveCamId(int id)
  {
    std::string name_of_aae = getIdentifier();

    if (std::find(camera_ids_in_aae_[name_of_aae].begin(),
                  camera_ids_in_aae_[name_of_aae].end(), id) != camera_ids_in_aae_[name_of_aae].end())
      active_cam_id_ = id;
    else
      throw std::runtime_error("Setting a cam id that does not exist");
  }

  std::string appendCamIdToViewName(const char* name , int cam_id)
  {
    std::stringstream ss;
    ss << "cam"<<(cam_id==-1 ? std::to_string(active_cam_id_): std::to_string(cam_id))<<"."<< name;
    return ss.str();
  }

  template <class T>
  bool get(const char* name, T& output, int cam_id = -1)
  {
    std::string view_name_with_id = appendCamIdToViewName(name, cam_id);
    return get(view_name_with_id.c_str(), output, std::is_base_of<rs::FeatureStructureProxy, T>());
  }

  template <class T>
  void set(const char* name, const T& input, int cam_id = -1)
  {
    std::string view_name_with_id = appendCamIdToViewName(name, cam_id);
    set(view_name_with_id.c_str(), input, std::is_base_of<rs::FeatureStructureProxy, T>());
  }

  template <class T>
  bool get(const char* name, std::vector<T>& output, int cam_id = -1)
  {
    std::string view_name_with_id = appendCamIdToViewName(name, cam_id);
    return get(view_name_with_id.c_str(), output, std::is_base_of<rs::FeatureStructureProxy, T>());
  }

  template <class T>
  void set(const char* name, const std::vector<T>& input, int cam_id = -1)
  {
    std::string view_name_with_id = appendCamIdToViewName(name, cam_id);
    set(view_name_with_id.c_str(), input, std::is_base_of<rs::FeatureStructureProxy, T>());
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
