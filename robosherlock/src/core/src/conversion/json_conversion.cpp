/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
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

// STL
#include <map>

//rapidjson
#include <rapidjson/rapidjson.h>

// RS
#include <robosherlock/conversion/json.h>
#include <robosherlock/conversion/conversion.h>

namespace rs
{
namespace conversion
{

/******************************************************************************
 * Conversion:: FeatureStructure
 *****************************************************************************/

template<>
void from(const uima::FeatureStructure &fs, rapidjson::Document &output)
{
  fromFeatureStructure(fs, output);

//  rapidjson::StringBuffer buffer;
//  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
//  output.Accept(writer);
//  std::cerr<<__FILE__<<"::"<<__LINE__<<":"<<buffer.GetString()<<std::endl;
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const rapidjson::Document &input)
{
  return toFeatureStructure(cas, input);
}

} // namespace conversion

} // namespace rs
