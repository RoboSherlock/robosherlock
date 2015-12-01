/*
 * Copyright (c) 2012, Christian Kerl <christian.kerl@in.tum.de>
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

#ifndef SCENE_CAS_H_
#define SCENE_CAS_H_

#include <map>

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

//typedef std::map<std::string, std::vector<rs::SemanticMapObject>> SemanticMap;

class SceneCas
{
private:
  uima::CAS &cas;

public:
  SceneCas(uima::CAS &cas);
  virtual ~SceneCas();

  rs::Scene getScene();

  bool has(const char *name);
  bool getFS(const char *name, uima::FeatureStructure &fs);
  void setFS(const char *name, const uima::FeatureStructure &fs);

  template <class T>
  bool get(const char *name, T &output)
  {
    return get(name, output, std::is_base_of<rs::FeatureStructureProxy,T>());
  }

  template <class T>
  void set(const char *name, const T &input)
  {
    set(name, input, std::is_base_of<rs::FeatureStructureProxy,T>());
  }

  template <class T>
  bool get(const char *name, std::vector<T> &output)
  {
    return get(name, output, std::is_base_of<rs::FeatureStructureProxy,T>());
  }

  template <class T>
  void set(const char *name, const std::vector<T> &input)
  {
    set(name, input, std::is_base_of<rs::FeatureStructureProxy,T>());
  }

private:
  bool getView(const char *name, uima::CAS *&view);

  template <class T>
  bool get(const char *name, T &output, const std::true_type &)
  {
    uima::FeatureStructure fs;
    if(getFS(name, fs))
    {
      output = T(fs);
      return true;
    }
    return false;
  }

  template <class T>
  bool get(const char *name, T &output, const std::false_type &)
  {
    uima::FeatureStructure fs;
    if(getFS(name, fs))
    {
      rs::conversion::from(fs, output);
      return true;
    }
    return false;
  }

  template <class T>
  void set(const char *name, const T &input, const std::true_type &)
  {
    setFS(name, (uima::FeatureStructure)input);
  }

  template <class T>
  void set(const char *name, const T &input, const std::false_type &)
  {
    setFS(name, conversion::to(cas, input));
  }

  template <class T>
  bool get(const char *name, std::vector<T> &output, const std::true_type &)
  {
    output.clear();
    uima::FeatureStructure fs;

    if(!getFS(name, fs))
    {
      return false;
    }

    uima::ArrayFS array(fs);
    output.reserve(array.size());

    for(size_t i = 0; i < array.size(); ++i)
    {
      output.push_back(T(array.get(i)));
    }
    return true;
  }

  template <class T>
  bool get(const char *name, std::vector<T> &output, const std::false_type &)
  {
    uima::FeatureStructure fs;

    if(!getFS(name, fs))
    {
      return false;
    }

    uima::ArrayFS array(fs);
    output.resize(array.size());

    for(size_t i = 0; i < array.size(); ++i)
    {
      rs::conversion::from(array.get(i), output[i]);
    }
    return true;
  }

  template <class T>
  void set(const char *name, const std::vector<T> &input, const std::true_type &)
  {
    uima::ArrayFS array = cas.createArrayFS(input.size());

    for(int i = 0; i < input.size(); ++i)
    {
      array.set(i, (uima::FeatureStructure)input[i]);
    }
    setFS(name, array);
  }

  template <class T>
  void set(const char *name, const std::vector<T> &input, const std::false_type &)
  {
    uima::ArrayFS array = cas.createArrayFS(input.size());

    for(int i = 0; i < input.size(); ++i)
    {
      array.set(i, conversion::to(cas, input[i]));
    }
    setFS(name, array);
  }
};

} /* namespace rs */
#endif /* SCENE_CAS_H_ */
