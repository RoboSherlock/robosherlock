/** \file config_param.hpp .
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

    \brief MetaDataObjects for configuration parameters and values

   Description:

-----------------------------------------------------------------------------


   01/30/2003  Initial creation

-------------------------------------------------------------------------- */
#ifndef UIMA_CONFIG_PARAM_HPP
#define UIMA_CONFIG_PARAM_HPP
// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include <uima/pragmas.hpp> //must be first to surpress warnings
#include <uima/err_ids.h>
#include <uima/taemetadata.hpp>
#include <uima/strconvert.hpp>
#include <uima/exceptions.hpp>
#include <map>
namespace uima {
  UIMA_EXC_CLASSDECLARE(ConfigException, Exception);
  UIMA_EXC_CLASSDECLARE(ConfigParamException, ConfigException);

  /**
  * Contains the definition of a configuration parameter, for example
  * its name and its type.
  */
  class UIMA_LINK_IMPORTSPEC ConfigurationParameter : public MetaDataObject {
  public:
    enum EnParameterType {
      STRING, INTEGER, FLOAT, BOOLEAN
    };
    enum EnParameterAggregation {
      SINGLE_VALUE = false, MULTIPLE_VALUES = true
    };
    ConfigurationParameter()
        :MetaDataObject(), iv_name(), iv_description(), iv_enParamAggregation(SINGLE_VALUE),
        iv_mandatory(false), iv_vecRestrictedToDelegates() {}


    TyErrorId setName(const icu::UnicodeString  & name) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_name = name;
      return UIMA_ERR_NONE;
    }

    const icu::UnicodeString & getName() const {
      return iv_name;
    }

    TyErrorId setDescription(const icu::UnicodeString  & description) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_description = description;
      return UIMA_ERR_NONE;
    }

    const icu::UnicodeString & getDescription() const {
      return iv_description;
    }

    TyErrorId setType(EnParameterType enParamType) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_enParamType = enParamType;
      return UIMA_ERR_NONE;
    }

    EnParameterType getType() const {
      return iv_enParamType;
    }

    TyErrorId setMultiValued(EnParameterAggregation enMode) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_enParamAggregation = enMode;
      return UIMA_ERR_NONE;
    }

    bool isMultiValued() const {
      return(iv_enParamAggregation == MULTIPLE_VALUES);
    }

    TyErrorId setMandatory(bool man) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_mandatory = man;
      return UIMA_ERR_NONE;
    }

    bool isMandatory() const {
      return iv_mandatory;
    }

    /**
    * Returns true iff settings for this configuration parameter
    * will only be visible to certain delegate TAEs
    **/
    /*       bool hasRestrictions() const{                       */
    /*          return(iv_vecRestrictedToDelegates.size() != 0); */
    /*       }                                                   */

    /**
    * Settings for this configuration parameter will only be visible 
    * for the delegate AnC <code>delegateName</code> 
    **/
    TyErrorId addRestriction(const icu::UnicodeString & delegateName) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }

      iv_vecRestrictedToDelegates.push_back(delegateName);
      return UIMA_ERR_NONE;
    }

    bool isDefinedForAnnotatorContext(const icu::UnicodeString & ancKey) const;

    /**
    * @return The names for the delegate TAEs which are possibly affected by
    * setting this parameter. If the vector is empty, all delegates
    * are possibly affected.
    **/
    /*       const std::vector<icu::UnicodeString> & getRestrictions() const{ */
    /*          return iv_vecRestrictedToDelegates;                           */
    /*       }                                                                */

  private:
    icu::UnicodeString iv_name;
    icu::UnicodeString iv_description;
    EnParameterType iv_enParamType;
    EnParameterAggregation iv_enParamAggregation;
    bool iv_mandatory;
    std::vector<icu::UnicodeString> iv_vecRestrictedToDelegates;


  }
  ; /* ConfigurationParameter */

  /**
  * Contains the value for a certain parameter. The parameter name is determined
  * by<code>getName()</code>.
  * Has several<code>extractValue</code>methods to extract the value according
  * to its type. These methods will return<code>UIMA_ERR_CONFIG_INVALID_EXRACTOR_FOR_TYPE</code>
  * if the value type does not match the type in the<code>extractValue</code>method.
  **/
  class UIMA_LINK_IMPORTSPEC NameValuePair : public MetaDataObject {

  public:
    typedef std::vector < icu::UnicodeString > TyStrValueList;
    typedef std::vector < int >                TyIntValueList;
    typedef std::vector < float >              TyFloatValueList;
    typedef std::vector < bool >               TyBoolValueList;

    NameValuePair();
    /**
    * Does nothing, as reconfiguration of parameter values is possible
    **/
    void commit() {}

    void setName(const icu::UnicodeString & name) {
      iv_name = name;
    }

    const icu::UnicodeString & getName() const {
      return iv_name;
    }

    void define(ConfigurationParameter::EnParameterType type, ConfigurationParameter::EnParameterAggregation enIsMulti);

    bool isMultiValued() const {
      return(iv_enAggregation == ConfigurationParameter::MULTIPLE_VALUES);
    }

    ConfigurationParameter::EnParameterType getType() const {
      return iv_enType;
    }

    // for single value
    TyErrorId               setValue(int value);
    TyErrorId               setValue(float value);
    TyErrorId               setValue(bool value);
    TyErrorId               setValue(const icu::UnicodeString & value);
    // for multiple value
    TyErrorId               addValue(int value);
    TyErrorId               addValue(float value);
    TyErrorId               addValue(bool value);
    TyErrorId               addValue(const icu::UnicodeString & value);

    bool                    extractBoolValue() const;
    std::vector<bool> const &    extractBoolValues() const;

    int                     extractIntValue() const;
    std::vector<int> const &     extractIntValues() const;

    float                   extractFloatValue() const;
    std::vector<float> const &   extractFloatValues() const;

    icu::UnicodeString const &          extractStringValue() const;
    std::vector<icu::UnicodeString> const &  extractStringValues() const;

    //Returns a  UTF-8 string
    std::string extractSingleByteStringValue() const;
    //converts array of values  from UTF-16 to UTF-8
    //Caller assumes memory ownership of string objects in vector
    void extractSingleByteStringValues( std::vector<std::string*> & values) const;


  private:
    size_t size() const;
    void ensureType(ConfigurationParameter::EnParameterType t) const;
    void ensureSingleValue() const;
    void ensureMultipleValues() const;
    void reset();
    std::string convert(const icu::UnicodeString & value) const;

    icu::UnicodeString iv_name;
    ConfigurationParameter::EnParameterType iv_enType;
    ConfigurationParameter::EnParameterAggregation iv_enAggregation;
    TyStrValueList    iv_strValues;
    TyIntValueList    iv_intValues;
    TyFloatValueList  iv_fltValues;
    TyBoolValueList   iv_boolValues;
  };

  /**
  * Contains all<code>ConfigurationParameter</code>objects for a certain configuration group.
  **/
  class UIMA_LINK_IMPORTSPEC ConfigurationGroup:public MetaDataObject {
  public:
    ConfigurationGroup()
        :MetaDataObject(), iv_configParams() {}

    ~ConfigurationGroup();

    void commit() {
      std::map<icu::UnicodeString, ConfigurationParameter *>::iterator entries;
      for (entries = iv_configParams.begin(); entries != iv_configParams.end(); entries++) {
        entries->second->commit();
      }
    }

    /**
    * Note: This object will assume memory ownership of<code>param</code>and will delete it in its
    * destructor !
    **/
    TyErrorId addConfigurationParameter(ConfigurationParameter * param) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_configParams[param->getName()] = param;
      return UIMA_ERR_NONE;
    }

    /**
    * returns TRUE iff a configuration parameter<code>paramName</code>exists in this group.
    **/
    bool hasConfigurationParameter(const icu::UnicodeString & paramName) const {
      std::map<icu::UnicodeString, ConfigurationParameter *>::const_iterator ite = iv_configParams.find(paramName);
      return(ite != iv_configParams.end());
    }

    /**
    * returns the <code>ConfigurationParameter</code> with name <code>paramName</code> or NULL
    * if no such parameter is found.
    **/
    const ConfigurationParameter * getConfigurationParameter(const icu::UnicodeString & paramName) const {
      std::map<icu::UnicodeString, ConfigurationParameter *>::const_iterator ite = iv_configParams.find(paramName);
      if (ite != iv_configParams.end()) {
        return ite->second;
      } else {
        return NULL;
      }
    }

    const std::vector <const ConfigurationParameter *> getConfigurationParameters() const;

  private:
    std::map<icu::UnicodeString, ConfigurationParameter *> iv_configParams;

  };

  /**
  * Contains all<code>NameValuePair</code>objects for a certain configuration group.
  **/
  class UIMA_LINK_IMPORTSPEC SettingsForGroup : public MetaDataObject {
  public:
    SettingsForGroup()
        :MetaDataObject(), iv_nameValuePairs() {}

    ~SettingsForGroup() {
      std::map<icu::UnicodeString, NameValuePair *>::iterator entries = iv_nameValuePairs.begin();
      while (entries != iv_nameValuePairs.end()) {
        delete (*entries).second;
        entries++;
      }

    }

    /**
    * Does nothing, as reconfiguration of parameter values is possible
    **/
    void commit() {}

    /**
    * Note: This object will assume memory ownership of<code>nvPair</code>and will delete it in its
    * destructor !
    **/
    void addNameValuePair(NameValuePair * nvPair) {
      NameValuePair * oldPair = getNameValuePair(nvPair->getName());
      if (EXISTS(oldPair)) {
        delete oldPair;
      }
      iv_nameValuePairs[nvPair->getName()] = nvPair;
    }

    /**
    * Returns the<code>NameValuePair</code>whose name equals<code>paramName</code>or NULL if no such object can be found.
    **/
    NameValuePair * getNameValuePair(const icu::UnicodeString & paramName) const {
      std::map<icu::UnicodeString, NameValuePair *>::const_iterator ite = iv_nameValuePairs.find(paramName);
      if (ite == iv_nameValuePairs.end()) {
        return NULL;
      } else {
        return(*ite).second;
      }
    }

    std::vector<NameValuePair const *> getNameValuePairs() const;

  private:
    std::map<icu::UnicodeString, NameValuePair *> iv_nameValuePairs;
  };


} // namespace uima
#endif
