/** \file config_param.cpp .
-----------------------------------------------------------------------------

 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.

-----------------------------------------------------------------------------

   Description: Resource Specifier Classes for Text Analysis Engines

-----------------------------------------------------------------------------


   01/30/2003  Initial creation

-------------------------------------------------------------------------- */

// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include <uima/pragmas.hpp> //must be first to surpress warnings
#include <uima/config_param.hpp>
#include <uima/err_ids.h>
#include <uima/msg.h>

#include <vector>
using namespace std;
namespace uima {

  UIMA_EXC_CLASSIMPLEMENT(ConfigException, Exception);
  UIMA_EXC_CLASSIMPLEMENT(ConfigParamException, ConfigException);

  ConfigurationGroup::~ConfigurationGroup() {
    map<icu::UnicodeString, ConfigurationParameter *>::iterator entries = iv_configParams.begin();
    while (entries != iv_configParams.end()) {
      delete (*entries).second;
      entries++;
    }
  }

  const vector <const ConfigurationParameter *> ConfigurationGroup::getConfigurationParameters() const {
    vector <const ConfigurationParameter *> params;
    map<icu::UnicodeString, ConfigurationParameter *>::const_iterator ite;
    for (ite = iv_configParams.begin(); ite != iv_configParams.end(); ite++) {
      params.push_back(ite->second);
    }
    return params;
  }

  bool ConfigurationParameter::isDefinedForAnnotatorContext(const icu::UnicodeString & ancKey) const {
    if (iv_vecRestrictedToDelegates.size() == 0 //there are no restrictions
        || ancKey == "") {//the request is from the AnC itself, not a delegate
      return true;
    }
    bool found = false;
    size_t i = 0;
    while (!found && i < iv_vecRestrictedToDelegates.size()) {
      found = (ancKey.compare(iv_vecRestrictedToDelegates[i]) == 0);
      i++;
    }
    return found;
  }



  vector<NameValuePair const *> SettingsForGroup::getNameValuePairs() const {
    vector <const NameValuePair *> params;
    map<icu::UnicodeString, NameValuePair *>::const_iterator ite;
    for (ite = iv_nameValuePairs.begin(); ite != iv_nameValuePairs.end(); ite++) {
      params.push_back(ite->second);
    }
    return params;

  }


  NameValuePair::NameValuePair()
      :MetaDataObject(),
      iv_name(),
      iv_enType(ConfigurationParameter::INTEGER),
      iv_enAggregation(ConfigurationParameter::SINGLE_VALUE) {
    reset();
  }

  void NameValuePair::reset() {
    iv_strValues.clear();
    iv_intValues.clear();
    iv_fltValues.clear();
    iv_boolValues.clear();
    if (!iv_enAggregation) {
      switch (iv_enType) {
      case ConfigurationParameter::STRING  :
        iv_strValues.push_back(icu::UnicodeString());
        break;
      case ConfigurationParameter::INTEGER :
        iv_intValues.push_back((int) 0);
        break;
      case ConfigurationParameter::FLOAT   :
        iv_fltValues.push_back((float)0.0);
        break;
      case ConfigurationParameter::BOOLEAN :
        iv_boolValues.push_back(false);
        break;
      default:
        assertWithMsg(false, "Unexpected (new?) value for enum ConfigurationParameter");
      }
    }
  }
  void NameValuePair::define(ConfigurationParameter::EnParameterType enType, ConfigurationParameter::EnParameterAggregation enIsMulti) {
    iv_enType = enType;
    iv_enAggregation = enIsMulti;
    reset();
  }

  size_t NameValuePair::size() const {
    switch (iv_enType) {
    case ConfigurationParameter::STRING  :
      return iv_strValues.size();
    case ConfigurationParameter::INTEGER :
      return iv_intValues.size();
    case ConfigurationParameter::FLOAT   :
      return iv_fltValues.size();
    case ConfigurationParameter::BOOLEAN :
      return iv_boolValues.size();
    }
    assertWithMsg(false, "Unexpected (new?) value for enum ConfigurationParameter");
    return 0;
  }

  TyErrorId NameValuePair::setValue(int value) {
    ensureType(ConfigurationParameter::INTEGER);
    ensureSingleValue();
    iv_intValues[0] = value;
    return UIMA_ERR_NONE;
  }

  TyErrorId NameValuePair::setValue(float value) {
    ensureType(ConfigurationParameter::FLOAT);
    ensureSingleValue();
    iv_fltValues[0] = value;
    return UIMA_ERR_NONE;
  }

  TyErrorId NameValuePair::setValue(bool value) {
    ensureType(ConfigurationParameter::BOOLEAN);
    ensureSingleValue();
    iv_boolValues[0] = value;
    return UIMA_ERR_NONE;
  }

  TyErrorId NameValuePair::setValue(const icu::UnicodeString & value) {
    ensureSingleValue();
    switch (iv_enType) {
    case ConfigurationParameter::STRING  :
      iv_strValues[0] = value;
      break;
    case ConfigurationParameter::INTEGER :
      iv_intValues[0] = (int) string2Long(convert(value));
      break;
    case ConfigurationParameter::FLOAT   :
      iv_fltValues[0] = (float)string2Double(convert(value));
      break;
    case ConfigurationParameter::BOOLEAN :
      iv_boolValues[0] = string2Bool(convert(value));
      break;
    default:
      assertWithMsg(false, "Unexpected (new?) value for enum ConfigurationParameter");
    }

    return UIMA_ERR_NONE;
  }

  TyErrorId NameValuePair::addValue(int value) {
    ensureType(ConfigurationParameter::INTEGER);
    ensureMultipleValues();
    iv_intValues.push_back(value);
    return UIMA_ERR_NONE;
  }

  TyErrorId NameValuePair::addValue(float value) {
    ensureType(ConfigurationParameter::FLOAT);
    ensureMultipleValues();
    iv_fltValues.push_back(value);
    return UIMA_ERR_NONE;
  }

  TyErrorId NameValuePair::addValue(bool value) {
    ensureType(ConfigurationParameter::BOOLEAN);
    ensureMultipleValues();
    iv_boolValues.push_back(value);
    return UIMA_ERR_NONE;
  }

  TyErrorId NameValuePair::addValue(const icu::UnicodeString & value) {
    ensureMultipleValues();
    switch (iv_enType) {
    case ConfigurationParameter::STRING  :
      iv_strValues.push_back(value);
      break;
    case ConfigurationParameter::INTEGER :
      iv_intValues.push_back((int) string2Long(convert(value)));
      break;
    case ConfigurationParameter::FLOAT   :
      iv_fltValues.push_back((float)string2Double(convert(value)));
      break;
    case ConfigurationParameter::BOOLEAN :
      iv_boolValues.push_back(string2Bool(convert(value)));
      break;
    default:
      assertWithMsg(false, "Unexpected (new?) value for enum ConfigurationParameter");
    }

    return UIMA_ERR_NONE;
  }

  void NameValuePair::ensureType(ConfigurationParameter::EnParameterType t) const {
    if (iv_enType != t) {
      UIMA_EXC_THROW_NEW(ConfigException,
                         UIMA_ERR_CONFIG_INVALID_EXTRACTOR_FOR_TYPE,
                         UIMA_MSG_ID_EXC_CONFIG_VALUE_TYPE_MISMATCH,
                         UIMA_MSG_ID_EXCON_CONFIG_VALUE_EXTRACT,
                         ErrorInfo::recoverable);
    }
  }
  void NameValuePair::ensureSingleValue()  const {
    if (isMultiValued()) {
      UIMA_EXC_THROW_NEW(ConfigException,
                         UIMA_ERR_CONFIG_INVALID_EXTRACTOR_FOR_TYPE,
                         UIMA_MSG_ID_EXC_CONFIG_VALUE_MUST_BE_SINGLE,
                         UIMA_MSG_ID_EXCON_CONFIG_VALUE_EXTRACT,
                         ErrorInfo::recoverable);
    }
    assert(size() == 1);
  }

  void NameValuePair::ensureMultipleValues() const {
    if (!isMultiValued()) {
      UIMA_EXC_THROW_NEW(ConfigException,
                         UIMA_ERR_CONFIG_INVALID_EXTRACTOR_FOR_TYPE,
                         UIMA_MSG_ID_EXC_CONFIG_VALUE_MUST_BE_MULTI,
                         UIMA_MSG_ID_EXCON_CONFIG_VALUE_EXTRACT,
                         ErrorInfo::recoverable);
    }
  }

  bool NameValuePair::extractBoolValue() const {
    ensureType(ConfigurationParameter::BOOLEAN);
    ensureSingleValue();
    return(iv_boolValues[0]);
  }

  vector<bool> const & NameValuePair::extractBoolValues() const {
    ensureType(ConfigurationParameter::BOOLEAN);
    ensureMultipleValues();
    return(iv_boolValues);
  }

  int NameValuePair::extractIntValue() const {
    ensureType(ConfigurationParameter::INTEGER);
    ensureSingleValue();
    return(iv_intValues[0]);
  }

  vector<int> const & NameValuePair::extractIntValues() const {
    ensureType(ConfigurationParameter::INTEGER);
    ensureMultipleValues();
    return(iv_intValues);
  }

  float NameValuePair::extractFloatValue() const {
    ensureType(ConfigurationParameter::FLOAT);
    ensureSingleValue();
    return(iv_fltValues[0]);
  }

  vector<float> const & NameValuePair::extractFloatValues() const {
    ensureType(ConfigurationParameter::FLOAT);
    ensureMultipleValues();
    return(iv_fltValues);
  }

  icu::UnicodeString const & NameValuePair::extractStringValue() const {
    ensureType(ConfigurationParameter::STRING);
    ensureSingleValue();
    return(iv_strValues[0]);
  }

  vector<icu::UnicodeString> const & NameValuePair::extractStringValues() const {
    ensureType(ConfigurationParameter::STRING);
    ensureMultipleValues();
    return(iv_strValues);
  }

  std::string  NameValuePair::extractSingleByteStringValue() const {
    ensureType(ConfigurationParameter::STRING);
    ensureSingleValue();
    //return(convert(iv_strValues[0]));
    //why was this changed? convert reurns a string! (bll)
    std::string svalue( convert(iv_strValues[0]));
    return svalue;
  }

  void NameValuePair::extractSingleByteStringValues( vector<std::string*> &  vecStr ) const  {
    ensureType(ConfigurationParameter::STRING);
    ensureMultipleValues();
    for (size_t i=0; i < iv_strValues.size(); i++) {
      std::string * pvalue = new std::string(convert(iv_strValues[i]));
      vecStr.push_back(pvalue);
    }
  }


  /* inline */ std::string NameValuePair::convert(const icu::UnicodeString & value) const {
    return UnicodeStringRef(value).asUTF8();
  }


} // namespace uima
