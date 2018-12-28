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

// STL
#include <map>

//rapidjson
#include <rapidjson/rapidjson.h>

// RS
#include <rs/conversion/json.h>
#include <rs/conversion/conversion.h>

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
  output = fromFeatureStructure(fs, rapidjson::Document());
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const rapidjson::Document &input)
{
  return toFeatureStructure(cas, input);
}

} // namespace conversion

} // namespace rs
