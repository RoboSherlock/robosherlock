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


#ifndef ACCESSOR_H_
#define ACCESSOR_H_

#include <uima/api.hpp>

#include <robosherlock/utils/output.h>

namespace rs
{
namespace accessor
{

/**
 * Templated conversion utility method. Default implementation just casts to TargetT.
 */
template<typename SourceT, typename TargetT>
inline TargetT convert(const SourceT &value)
{
  return (TargetT) value;
}

template<>
inline std::string convert<uima::UnicodeStringRef, std::string>(const uima::UnicodeStringRef &str)
{
  return str.asUTF8();
}

template<>
inline icu::UnicodeString convert<std::string, icu::UnicodeString>(const std::string &str)
{
  return icu::UnicodeString(str.c_str(), str.length(), (const char *)NULL);
}

template<>
inline uima::UnicodeStringRef convert<std::string, uima::UnicodeStringRef>(const std::string &str)
{
  return uima::UnicodeStringRef(icu::UnicodeString(str.c_str(), str.length(), (const char *)NULL));
}

/**
 * Implementation for Accessor. Accessor specializations should inherit from this and specify
 * appropriate template arguments.
 */
template<typename T, T(uima::FeatureStructure::*GETTER)(uima::Feature const &) const, void (uima::FeatureStructure::*SETTER)(uima::Feature const &, T)>
struct AccessorImpl
{
  static T get(uima::FeatureStructure &fs, uima::Feature &feature)
  {
    return (fs.*GETTER)(feature);
  }

  static void set(uima::FeatureStructure &fs, uima::Feature &feature, T value)
  {
    (fs.*SETTER)(feature, value);
  }
};

/**
 * Accessor provides templated interface to access values in a uima::FeatureStructure.
 * UIMA only offers methods like getIntValue, getStringValue, so the type of the value
 * has always to be known by the caller. Accessor frees you from this limitation.
 */
template<typename T>
struct Accessor
{
  static T get(uima::FeatureStructure &fs, uima::Feature &feature)
  {

    outError("You are trying to get a FS type. Try chanign yours to ComplexFeatureStructure");
    throw std::exception();
  }

  static void set(uima::FeatureStructure &fs, uima::Feature &feature, T value)
  {
    outError("You are trying to get a FS type. Try chanign yours to ComplexFeatureStructure");
    throw std::exception();
  }
};

template<>
struct Accessor<std::string>
{
  static std::string get(uima::FeatureStructure &fs, uima::Feature &feature)
  {
    return convert<uima::UnicodeStringRef, std::string>(fs.getStringValue(feature));
  }

  static void set(uima::FeatureStructure &fs, uima::Feature &feature, std::string value)
  {
    fs.setStringValue(feature, convert<std::string, icu::UnicodeString>(value));
  }
};

template<>
struct Accessor<bool> :
  public AccessorImpl<bool, &uima::FeatureStructure::getBooleanValue, &uima::FeatureStructure::setBooleanValue>
{
};

template<>
struct Accessor<char> :
  public AccessorImpl<char, &uima::FeatureStructure::getByteValue, &uima::FeatureStructure::setByteValue>
{
};

template<>
struct Accessor<short int> :
  public AccessorImpl<short int, &uima::FeatureStructure::getShortValue, &uima::FeatureStructure::setShortValue>
{
};

template<>
struct Accessor<int> :
  public AccessorImpl<int, &uima::FeatureStructure::getIntValue, &uima::FeatureStructure::setIntValue>
{
};

template<>
struct Accessor<INT64> :
  public AccessorImpl<INT64, &uima::FeatureStructure::getLongValue, &uima::FeatureStructure::setLongValue>
{
};

template<>
struct Accessor<float> :
  public AccessorImpl<float, &uima::FeatureStructure::getFloatValue, &uima::FeatureStructure::setFloatValue>
{
};

template<>
struct Accessor<double> :
  public AccessorImpl<double, &uima::FeatureStructure::getDoubleValue, &uima::FeatureStructure::setDoubleValue>
{
};

} // namespace
} // namespace

#endif /* ACCESSOR_H_ */
