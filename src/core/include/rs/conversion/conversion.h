/*
 * Copyright (c) 2011, Nico Blodow <blodow@cs.tum.edu>
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
