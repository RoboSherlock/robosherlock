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
#ifndef __CONVERSION_H__
#define __CONVERSION_H__

// STL
#include <string>
#include <exception>

// UIMA
#include <uima/api.hpp>

// PCL
#include <pcl/point_types.h>

// RS
#include <rs/feature_structure_proxy.h>

namespace rs
{
namespace conversion
{
class ConversionException : public std::exception
{

public:

  const std::string msg;
  ConversionException(const std::string &msg);
  virtual ~ConversionException() throw();
  const char *what() const throw();
};

template<typename T>
void from(const uima::FeatureStructure &fs, T &output);

template<typename T>
uima::FeatureStructure to(uima::CAS &cas, const T &input);

} // conversion
} // rs

// add more as needed
TYPE_TRAIT(pcl::PointXYZ, "PointXYZ")
TYPE_TRAIT(pcl::RGB, "RGB")
TYPE_TRAIT(pcl::PointXYZRGB, "PointXYZRGB")
TYPE_TRAIT(pcl::PointXYZRGBA, "PointXYZRGBA")
TYPE_TRAIT(pcl::Normal, "Normal")
TYPE_TRAIT(pcl::VFHSignature308, "VFHSignature308")

#endif //__CONVERSION_H__
