/** \file taespecifier.cpp .
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
#include <uima/err_ids.h>
#include <uima/msg.h>


#include <uima/taespecifierbuilder.hpp>
#include <uima/typesystemdescription.hpp>
#include <uima/capability.hpp>
#include <uima/config_param.hpp>
#include <uima/taespecifier.hpp>
#include <uima/caswriter_abase.hpp>
using namespace std;
namespace uima {
  UIMA_EXC_CLASSIMPLEMENT(ConfigParamLookupException, ConfigException);

  AnalysisEngineMetaData::~AnalysisEngineMetaData() {
    delete iv_pTypeSystemDesc;
    delete iv_pFlowConstraints;
    if (iv_pOperationalProperties != 0) {
      delete iv_pOperationalProperties;
    }
    size_t i=0;
    for (i=0; i < iv_capabilities.size(); i++) {
      delete iv_capabilities[i];
    }
    for (i=0; i < iv_indexDescs.size(); i++) {
      delete iv_indexDescs[i];
    }
    for (i=0; i < iv_vecpTypePriorities.size(); i++) {
      delete iv_vecpTypePriorities[i];
    }
    for (i=0; i < iv_fsindexImportDescs.size(); i++) {
      delete iv_fsindexImportDescs[i];
    }
    for (i=0; i < iv_typepriorityImportDescs.size(); i++) {
      delete iv_typepriorityImportDescs[i];
    }

  }

  void AnalysisEngineMetaData::validate() {
    size_t i=0;
    //validate the type system
    //validate(*aeMetadata.getTypeSystemDescription());
    getTypeSystemDescription()->validate();

    //validate the capabilities
    AnalysisEngineMetaData::TyVecpCapabilities capabilities = getCapabilites();
    for (i=0; i<capabilities.size(); i++) {
		if ( !(EXISTS(capabilities[i])) ) {
			UIMA_EXC_THROW_NEW(ValidationException,
                           UIMA_ERR_CONFIG_OBJECT_NOT_FOUND,
                           ErrorMessage(UIMA_MSG_ID_EXCON_CHECKING_CAPABILITY_SPEC),
                           UIMA_MSG_ID_EXCON_CHECKING_CAPABILITY_SPEC,
                           ErrorInfo::unrecoverable);
		}
    }

    //validate the index descriptions
    AnalysisEngineMetaData::TyVecpFSIndexDescriptions indexDescs = getFSIndexDescriptions();
    vector <icu::UnicodeString> indexLabels;
    for (i=0; i<indexDescs.size(); i++) {
      if ( !(EXISTS(indexDescs[i])) ){
			UIMA_EXC_THROW_NEW(ValidationException,
                           UIMA_ERR_CONFIG_OBJECT_NOT_FOUND,
                           ErrorMessage(UIMA_MSG_ID_EXC_INVALID_INDEX_OBJECT, indexLabels[i]),
                           UIMA_MSG_ID_EXCON_CHECKING_INDEX_DEFINITION,
                           ErrorInfo::unrecoverable);
	  }
      indexLabels.push_back((*indexDescs[i]).getLabel());
    }

    //check uniqueness of the index description labels
    for (i=0; i<indexLabels.size(); i++) {
      if ( countValues(indexLabels.begin(), indexLabels.end(), indexLabels[i]) != 1) {

        UIMA_EXC_THROW_NEW(ValidationException,
                           UIMA_ERR_CONFIG_DUPLICATE_INDEX_LABEL,
                           ErrorMessage(UIMA_MSG_ID_EXC_DUPLICATE_INDEX_LABEL, indexLabels[i]),
                           UIMA_MSG_ID_EXCON_CHECKING_INDEX_DEFINITION,
                           ErrorInfo::unrecoverable);
      }
    }

    //validate the configuration parameters

    //make sure that there's a value for each mandatory configuration parameter
    const vector <icu::UnicodeString> groupNames = getConfigurationGroupNames();
    for (i=0; i < groupNames.size(); i++) {
      const vector <const ConfigurationParameter *> params = getConfigurationParameters(groupNames[i]);
      for (size_t j=0; j < params.size(); j++) {
        if (params[j]->isMandatory()) {
          NameValuePair * nvPair = getNameValuePair(groupNames[i], params[j]->getName(),
                                   getSearchStrategy());
          if (! EXISTS(nvPair)) {
            if (hasGroups()) {
              ErrorMessage err(UIMA_MSG_ID_EXC_NO_VALUE_FOR_MANDATORY_PARAM_IN_GROUP);
              err.addParam(groupNames[i]);
              err.addParam(params[j]->getName());
              UIMA_EXC_THROW_NEW(ValidationException,
                                 UIMA_ERR_CONFIG_NO_VALUE_FOR_MANDATORY_PARAM_IN_GROUP,
                                 err,
                                 UIMA_MSG_ID_EXCON_VALIDATE_TAE_SPEC,
                                 ErrorInfo::unrecoverable);
            } else {
              ErrorMessage err(UIMA_MSG_ID_EXC_NO_VALUE_FOR_MANDATORY_PARAM);
              err.addParam(params[j]->getName());
              UIMA_EXC_THROW_NEW(ValidationException,
                                 UIMA_ERR_CONFIG_NO_VALUE_FOR_MANDATORY_PARAM,
                                 err,
                                 UIMA_MSG_ID_EXCON_VALIDATE_TAE_SPEC,
                                 ErrorInfo::unrecoverable);

            }
          }
        }
      }
    }
  }

  void AnalysisEngineMetaData::commit() {
    if (EXISTS(iv_pTypeSystemDesc)) {
      iv_pTypeSystemDesc->commit();
    }
    if (EXISTS(iv_pFlowConstraints)) {
      iv_pFlowConstraints->commit();
    }
    TyConfigGroup::iterator ite;
    for (ite = iv_configurationGroups.begin(); ite != iv_configurationGroups.end(); ite++) {
      (*ite).second.commit();
    }
    size_t i;
    for (i=0; i < iv_capabilities.size(); i++) {
      iv_capabilities[i]->commit();
    }
    for (i=0; i < iv_indexDescs.size(); i++) {
      iv_indexDescs[i]->commit();
    }
    for (i=0; i < iv_vecpTypePriorities.size(); i++) {
      iv_vecpTypePriorities[i]->commit();
    }
    if (hasDefaultGroup()) {
      TyConfigGroup::const_iterator ite = iv_configurationGroups.find(iv_defaultGroup);
      if (ite == iv_configurationGroups.end()) {
        iv_configurationGroups [iv_defaultGroup] = ConfigurationGroup();
      }
    }
    iv_bIsModifiable = false;
  }

  /**
  * Note: The configuration group is set during <code>commit</code>, if it doesn't exist already.
  * If we set it here, <code>addConfigurationGroup</code> throws an exception when it encounters
  * the group again.
  **/
  TyErrorId AnalysisEngineMetaData::setDefaultGroupName(const icu::UnicodeString & groupName) {
    if (! isModifiable()) {
      return UIMA_ERR_CONFIG_OBJECT_COMITTED;
    }
    iv_hasDefaultGroup = true;
    iv_defaultGroup = groupName;
    return UIMA_ERR_NONE;
  }


  TyErrorId AnalysisEngineMetaData::addConfigurationGroup(const icu::UnicodeString & groupName) {
    if (! isModifiable()) {
      return UIMA_ERR_CONFIG_OBJECT_COMITTED;
    }
    TyConfigGroup::iterator ite = iv_configurationGroups.find(groupName);
    if (ite == iv_configurationGroups.end()) {
      iv_configurationGroups [groupName] = ConfigurationGroup();
      return UIMA_ERR_NONE;
    } else {
      UIMA_EXC_THROW_NEW(DuplicateConfigElemException,
                         UIMA_ERR_CONFIG_DUPLICATE_GROUP,
                         UIMA_MSG_ID_EXC_CONFIG_DUPLICATE_GROUP,
                         ErrorMessage(UIMA_MSG_ID_EXC_CONFIG_DUPLICATE_GROUP, groupName),
                         ErrorInfo::recoverable);
    }
  }

  TyErrorId AnalysisEngineMetaData::addConfigurationParameter(const icu::UnicodeString & groupName,
      ConfigurationParameter * param) {
    if (! isModifiable()) {
      return UIMA_ERR_CONFIG_OBJECT_COMITTED;
    }

    if (groupName.compare(CONFIG_GROUP_NAME_WHEN_NO_GROUPS) != 0) {
      iv_hasGroups = true;
    }
    TyConfigGroup::iterator ite = iv_configurationGroups.find(groupName);
    if (ite == iv_configurationGroups.end()) {
      iv_configurationGroups [groupName] = ConfigurationGroup();
      ite = iv_configurationGroups.find(groupName);
    }
    ConfigurationGroup & group = (*ite).second;
    if (isParameterDefined(groupName, param->getName(), "")) {
      ErrorMessage msg(UIMA_MSG_ID_EXC_CONFIG_DUPLICATE_CONFIG_PARAM);
      msg.addParam(param->getName());
      UIMA_EXC_THROW_NEW(DuplicateConfigElemException,
                         UIMA_ERR_CONFIG_DUPLICATE_CONFIG_PARAM,
                         UIMA_MSG_ID_EXC_CONFIG_DUPLICATE_CONFIG_PARAM,
                         msg,
                         ErrorInfo::recoverable);
    }
    return group.addConfigurationParameter(param);
  }

  void AnalysisEngineMetaData::addNameValuePair(const icu::UnicodeString & groupName,
      NameValuePair * nvPair) {
    if (! isParameterDefined(groupName, nvPair->getName(), "")) {
      const icu::UnicodeString & paramName = nvPair->getName();
      if (hasGroups()) {
        ErrorMessage msg(UIMA_MSG_ID_EXC_CONFIG_PARAM_NOT_DEFINED_IN_GROUP);
        msg.addParam(paramName);
        msg.addParam(groupName);
        UIMA_EXC_THROW_NEW(ConfigParamLookupException,
                           UIMA_ERR_CONFIG_PARAM_NOT_DEFINED_IN_GROUP,
                           msg,
                           ErrorMessage(UIMA_MSG_ID_EXCON_CONFIG_PARAM_SEARCH, paramName),
                           ErrorInfo::recoverable);
      } else {
        UIMA_EXC_THROW_NEW(ConfigParamLookupException,
                           UIMA_ERR_CONFIG_PARAM_NOT_DEFINED,
                           ErrorMessage(UIMA_MSG_ID_EXC_CONFIG_PARAM_NOT_DEFINED, paramName),
                           ErrorMessage(UIMA_MSG_ID_EXCON_CONFIG_PARAM_SEARCH, paramName),
                           ErrorInfo::recoverable);
      }
    }
    TyConfigSettings::iterator ite = iv_configurationSettings.find(groupName);
    if (ite == iv_configurationSettings.end()) {
      iv_configurationSettings [groupName] = SettingsForGroup();
      ite = iv_configurationSettings.find(groupName);
    }
    SettingsForGroup & settings = (*ite).second;
    settings.addNameValuePair(nvPair);
  }

  NameValuePair * AnalysisEngineMetaData::getNameValuePairPtr(const icu::UnicodeString & groupName,
      const icu::UnicodeString & paramName) {
    TyConfigSettings::iterator ite = iv_configurationSettings.find(groupName);
    if (ite == iv_configurationSettings.end()) { // the group doesn't exist
      return NULL;
    }
    SettingsForGroup & settings = (*ite).second;
    return settings.getNameValuePair(paramName);
  }

  const icu::UnicodeString & AnalysisEngineMetaData::getGroupNameWhenNotSpec() const {
    if (hasGroups()) {
      if (hasDefaultGroup()) {
        return getDefaultGroupName();
      } else {
        UIMA_EXC_THROW_NEW(ConfigParamLookupException,
                           UIMA_ERR_CONFIG_NO_DEFAULT_GROUP_DEFINED,
                           UIMA_MSG_ID_EXC_CONFIG_NO_DEFAULT_GROUP_DEFINED,
                           UIMA_MSG_ID_EXC_CONFIG_NO_DEFAULT_GROUP_DEFINED,
                           ErrorInfo::recoverable);
      }
    } else {
      return CONFIG_GROUP_NAME_WHEN_NO_GROUPS;
    }
  }


  /**
  * This behavior ensures that an aggregate TAESpecifier that contains groups can
  * redefine parameters for TAESpecifiers that don't contain a group:
  * the aggregate has to define the parameter in its default group.
  **/
  NameValuePair * AnalysisEngineMetaData::getNameValuePair(const icu::UnicodeString & paramName,
      const icu::UnicodeString & ancKey) {
    return getNameValuePair(getGroupNameWhenNotSpec(), paramName, getSearchStrategy(), ancKey);
  }

  NameValuePair *  AnalysisEngineMetaData::getNameValuePair(const icu::UnicodeString & groupName,
      const icu::UnicodeString & paramName,
      EnSearchStrategy strategy,
      const icu::UnicodeString & ancKey) {

    NameValuePair * value = getNameValuePairNoFallback(groupName, paramName, ancKey);

    if (EXISTS(value)) {
      return value;
    }

    //check the fallback groups, if any
    vector < icu::UnicodeString> fallbackGroups;
    generateFallbackGroups(groupName, strategy, fallbackGroups);
    size_t i=0;
    while (i< fallbackGroups.size() && (! EXISTS(value))) {
      value = getNameValuePairNoFallback(fallbackGroups[i], paramName, ancKey);
      i++;
    }

    if (EXISTS(value)) {
      return value;
    }

    //if we get here, one of the following holds true:
    // a) the parameter name is invalid
    // b) the parameter is defined, but no value is specified
    //check for a)
    bool isDefined = isParameterDefined(groupName, paramName, ancKey);
    i = 0;
    while (! isDefined && i < fallbackGroups.size()) {
      isDefined = isParameterDefined(fallbackGroups[i], paramName, ancKey);
      i++;
    }

    if (isDefined) { // case b)
      return NULL;
    } else { // a)
      if (hasGroups()) {
        ErrorMessage msg(UIMA_MSG_ID_EXC_CONFIG_PARAM_NOT_DEFINED_IN_GROUP);
        msg.addParam(paramName);
        msg.addParam(groupName);
        UIMA_EXC_THROW_NEW(ConfigParamLookupException,
                           UIMA_ERR_CONFIG_PARAM_NOT_DEFINED_IN_GROUP,
                           msg,
                           ErrorMessage(UIMA_MSG_ID_EXCON_CONFIG_PARAM_SEARCH, paramName),
                           ErrorInfo::recoverable);
      } else {
        UIMA_EXC_THROW_NEW(ConfigParamLookupException,
                           UIMA_ERR_CONFIG_PARAM_NOT_DEFINED,
                           ErrorMessage(UIMA_MSG_ID_EXC_CONFIG_PARAM_NOT_DEFINED, paramName),
                           ErrorMessage(UIMA_MSG_ID_EXCON_CONFIG_PARAM_SEARCH, paramName),
                           ErrorInfo::recoverable);
      }
    }
  }

  NameValuePair * AnalysisEngineMetaData::getNameValuePairNoFallback(const icu::UnicodeString & groupName,
      const icu::UnicodeString & paramName,
      const icu::UnicodeString & ancKey) {
    //sanity check
    if (! hasGroups()
        && (groupName.compare(CONFIG_GROUP_NAME_WHEN_NO_GROUPS) != 0)
       ) {
      // return UIMA_ERR_CONFIG_NO_GROUPS_DEFINED;
      ErrorMessage errmsgContext(UIMA_MSG_ID_EXCON_CONFIG_PARAM_IN_GROUP_SEARCH);
      errmsgContext.addParam(paramName);
      errmsgContext.addParam(groupName);
      UIMA_EXC_THROW_NEW(ConfigParamLookupException,
                         UIMA_ERR_CONFIG_NO_GROUPS_DEFINED,
                         UIMA_MSG_ID_EXC_CONFIG_NO_GROUPS_DEFINED,
                         errmsgContext,
                         ErrorInfo::recoverable);
    }

    //try to find the value for the exact group
    NameValuePair * value = getNameValuePairPtr(groupName, paramName);
    if (EXISTS(value)) {
      //check whether the parameter is defined for that group
      if (isParameterDefined(groupName, paramName, ancKey)) {
        return value;
      } else {
        //return UIMA_ERR_CONFIG_PARAM_NOT_DEFINED_IN_GROUP;
        if (hasGroups()) {
          ErrorMessage errmsgContext(UIMA_MSG_ID_EXCON_CONFIG_PARAM_IN_GROUP_SEARCH);
          errmsgContext.addParam(paramName);
          errmsgContext.addParam(groupName);
          ErrorMessage errmsgExc(UIMA_MSG_ID_EXC_CONFIG_PARAM_NOT_DEFINED_IN_GROUP);
          errmsgExc.addParam(paramName);
          errmsgExc.addParam(groupName);
          UIMA_EXC_THROW_NEW(ConfigParamLookupException,
                             UIMA_ERR_CONFIG_PARAM_NOT_DEFINED_IN_GROUP,
                             errmsgExc,
                             errmsgContext,
                             ErrorInfo::recoverable);
        } else {
          UIMA_EXC_THROW_NEW(ConfigParamLookupException,
                             UIMA_ERR_CONFIG_PARAM_NOT_DEFINED,
                             ErrorMessage(UIMA_MSG_ID_EXC_CONFIG_PARAM_NOT_DEFINED, paramName),
                             ErrorMessage(UIMA_MSG_ID_EXCON_CONFIG_PARAM_SEARCH, paramName),
                             ErrorInfo::recoverable);
        }
        return NULL;
      }
    } else {
      return NULL;
    }
  }

  TyErrorId AnalysisEngineMetaData::setNameValuePair(const icu::UnicodeString & groupName,
      const NameValuePair & nvPair) {

    if (iv_configurationGroups.size() == 0) {
      return UIMA_ERR_CONFIG_PARAM_NOT_DEFINED;
    }
    const icu::UnicodeString & paramName = nvPair.getName();
    TyConfigGroup::const_iterator ite = iv_configurationGroups.find(groupName);
    if (ite == iv_configurationGroups.end()) {
      return UIMA_ERR_CONFIG_INVALID_GROUP_NAME;
    }
    const ConfigurationParameter * pConfigParm = NULL;

    //now check whether the parameter is defined in the group
    //or in the common parameters
    const ConfigurationGroup & confGroup = ite->second;

    if (confGroup.hasConfigurationParameter(paramName)) {
      pConfigParm = confGroup.getConfigurationParameter(paramName);
    } else {
      //check in the common parameters
      ite = iv_configurationGroups.find(CONFIG_GROUP_NAME_COMMON_PARMS);
      if (ite == iv_configurationGroups.end()) { //there are no common parameters
        return UIMA_ERR_CONFIG_INVALID_PARAM_NAME;
      }
      const ConfigurationGroup & commonGroup = ite->second;
      if (commonGroup.hasConfigurationParameter(paramName)) {
        pConfigParm = commonGroup.getConfigurationParameter(paramName);
      } else {
        return UIMA_ERR_CONFIG_INVALID_PARAM_NAME;
      }
    }

    assert(EXISTS(pConfigParm));
    //check if types and multiplicity match
    if (pConfigParm->getType() != nvPair.getType()) {
      return UIMA_ERR_CONFIG_INVALID_TYPE_FOR_PARAM;
    }
    if ((pConfigParm->isMultiValued() && (! nvPair.isMultiValued())
         || ((! pConfigParm->isMultiValued() && nvPair.isMultiValued())))
       ) {
      return UIMA_ERR_CONFIG_INVALID_TYPE_FOR_PARAM;
    }

    NameValuePair * pair = new NameValuePair(nvPair);
    //we can be sure that the configuration parameter is valid
    addNameValuePair(groupName, pair);
    return UIMA_ERR_NONE;
  }

  void AnalysisEngineMetaData::generateFallbackGroups(const icu::UnicodeString & groupName,
      EnSearchStrategy strategy,
      vector < icu::UnicodeString> & fallbackGroups) {
    int i;
    bool cutOffPossible;
    switch (strategy) {
    case NONE:
      //nothing to be done
      break;
    case DEFAULT_FALLBACK:
      fallbackGroups.push_back(getDefaultGroupName());
      break;
    case LANGUAGE_FALLBACK:
      i=groupName.lastIndexOf("-");
      cutOffPossible = (i != -1);
      while (cutOffPossible) {
        icu::UnicodeString fallback;
        groupName.extract(0, i, fallback);
        fallbackGroups.push_back(fallback);
        i=groupName.lastIndexOf("-", 0,i);
        cutOffPossible = (i != -1);
      }
      fallbackGroups.push_back(getDefaultGroupName());
      break;
    default:
      assert(false);
    }
  }

  /**
  * returns TRUE iff paramName is defined and the configuration parameter is visible
  * to the annotator context with key ancKey.
  **/
  bool AnalysisEngineMetaData::isParameterDefined(const icu::UnicodeString & paramName,
      const icu::UnicodeString & ancKey) const {
    return isParameterDefined(CONFIG_GROUP_NAME_WHEN_NO_GROUPS, paramName, ancKey);
  }
  /**
  * returns TRUE iff paramName is either defined for group groupName or in the commonParameter section.
  * Moreover, the configuration parameter must be visible to the annotator context with key ancKey
  **/
  bool AnalysisEngineMetaData::isParameterDefined(const icu::UnicodeString & groupName,
      const icu::UnicodeString & paramName,
      const icu::UnicodeString & ancKey) const {
    //check if it's defined in the group groupName
    TyConfigGroup::const_iterator ite = iv_configurationGroups.find(groupName);
    if (ite != iv_configurationGroups.end()) {
      const ConfigurationGroup & group = ite->second;
      const ConfigurationParameter * pConfigParm = group.getConfigurationParameter(paramName);
      if (EXISTS(pConfigParm)) { //the parameter exists
        return pConfigParm->isDefinedForAnnotatorContext(ancKey);
      }
      /*          if (group.hasConfigurationParameter(paramName)) { */
      /*             return true;                                   */
      /*          }                                                 */
    }
    //either the group does not exist or it doesn't define the parameter
    //we check in the common parameters
    ite = iv_configurationGroups.find(CONFIG_GROUP_NAME_COMMON_PARMS);
    if (ite != iv_configurationGroups.end()) {
      const ConfigurationGroup & group = ite->second;
      const ConfigurationParameter * pConfigParm = group.getConfigurationParameter(paramName);
      if (EXISTS(pConfigParm)) { //the parameter exists
        return pConfigParm->isDefinedForAnnotatorContext(ancKey);
      } else {
        return false;
      }
      //return group.hasConfigurationParameter(paramName);
    } else {
      return false;
    }
  }

  const vector <icu::UnicodeString> AnalysisEngineMetaData::getConfigurationGroupNames() const {
    vector <icu::UnicodeString> groupNames;
    TyConfigGroup::const_iterator ite;
    for (ite = iv_configurationGroups.begin(); ite != iv_configurationGroups.end(); ite++) {
      const icu::UnicodeString & groupName = ite->first;
      if (CONFIG_GROUP_NAME_COMMON_PARMS.compare(groupName) != 0) {
        groupNames.push_back(groupName);
      }
    }
    return groupNames;
  }

  const vector <const ConfigurationParameter *> AnalysisEngineMetaData::getConfigurationParameters(const icu::UnicodeString & groupName) const {
    vector <const ConfigurationParameter *> confParams;
    TyConfigGroup::const_iterator ite = iv_configurationGroups.find(groupName);
    if (ite == iv_configurationGroups.end()) {
      return confParams;
    }

    //collect the parameters from the group
    const ConfigurationGroup & confGroup = ite->second;
    const vector <const ConfigurationParameter *> params = confGroup.getConfigurationParameters();
    size_t i=0;
    for (i=0; i < params.size(); i++) {
      confParams.push_back(params[i]);
    }

    //collect the parameters from the common group, if any
    if (CONFIG_GROUP_NAME_COMMON_PARMS.compare(groupName) != 0) {
      const vector <const ConfigurationParameter *> commonParams = getConfigurationParameters(CONFIG_GROUP_NAME_COMMON_PARMS);
      for (i=0; i < commonParams.size(); i++) {
        confParams.push_back(commonParams[i]);
      }
    }

    return confParams;
  }

  const vector <icu::UnicodeString > AnalysisEngineMetaData::getGroupNamesForParameter(const icu::UnicodeString & paramName) const {
    vector <icu::UnicodeString > groups;
    vector <icu::UnicodeString> allGroups = getConfigurationGroupNames();
    vector <icu::UnicodeString>::const_iterator ite;
    for (ite = allGroups.begin(); ite != allGroups.end(); ite++) {
      if (isParameterDefined(*ite, paramName, "")) {
        groups.push_back(*ite);
      }
    }
    if (isParameterDefined(CONFIG_GROUP_NAME_COMMON_PARMS, paramName, "") && hasDefaultGroup()) {
      groups.push_back(getDefaultGroupName());
    }
    return groups;
  }


  TyErrorId AnalysisEngineMetaData::addTypePriority(TypePriority * prio) {
    if (! isModifiable()) {
      return UIMA_ERR_CONFIG_OBJECT_COMITTED;
    }
    iv_vecpTypePriorities.push_back(prio);
    return UIMA_ERR_NONE;
  }

  void AnalysisEngineDescription::commit() {
    if (EXISTS(iv_pAeMetaData)) {
      iv_pAeMetaData->commit();
    }
    TyMapDelegateSpecs::iterator ite;
    for (ite=iv_pMapDelegateSpecifiers->begin(); ite != iv_pMapDelegateSpecifiers->end(); ite++) {
      AnalysisEngineDescription * del = ite->second;
      del->commit();
    }
    iv_bIsModifiable = false;
  }


  /*-------------------------------------------------------*/
  /*                                                       */
  /*   AnalysisEngineDescription implementation            */
  /*-------------------------------------------------------*/
  AnalysisEngineDescription * AnalysisEngineDescription::getDelegate(const icu::UnicodeString & key) const {
    TyMapDelegateSpecs::iterator ite = iv_pMapDelegateSpecifiers->find(key);
    if (ite == iv_pMapDelegateSpecifiers->end()) {
      return NULL;
    } else {
      return ite->second;
    }
  }

  AnalysisEngineDescription * AnalysisEngineDescription::extractDelegate(const icu::UnicodeString & key) {
    TyMapDelegateSpecs::iterator ite = iv_pMapDelegateSpecifiers->find(key);
    if (ite == iv_pMapDelegateSpecifiers->end()) {
      return NULL;
    } else {
      AnalysisEngineDescription * taeSpec = ite->second;
      iv_pMapDelegateSpecifiers->erase(ite);
      return taeSpec;
    }
  }

  void AnalysisEngineDescription::toXMLBuffer(icu::UnicodeString & s) const {
    s.removeBetween(0, s.length());
    toXMLBufferNoXMLHeader(s);
    s.insert(0, "<?xml version=\"1.0\"?>");
  }

  void AnalysisEngineDescription::validate() {
    try {
      //first, validate the delegate specs
      AnalysisEngineDescription::TyMapDelegateSpecs delegates = getDelegates();
      AnalysisEngineDescription::TyMapDelegateSpecs::iterator ite = delegates.begin();
      while (ite != delegates.end()) {
		ite->second->validate();
        ite++;
      }

      //merge the type systems
      TypeSystemDescription * typeDesc = getAnalysisEngineMetaData()->getTypeSystemDescription();
      for (ite=delegates.begin(); ite != delegates.end(); ite++) {
        TypeSystemDescription * typeDescDel = ite->second->getAnalysisEngineMetaData()->getTypeSystemDescription();
        typeDesc->mergeTypeDescriptions(typeDescDel->getTypeDescriptions());
      }

      getAnalysisEngineMetaData()->validate();
      //tbd: validate the flow constraints
    } catch (ValidationException & rclException) {
      rclException.getErrorInfo().addContext(ErrorMessage(UIMA_MSG_ID_EXCON_VALIDATE_TAE_SPEC_FROM_FILE, getXmlFileLocation()));
      throw rclException;
    }
  }

  void appendCapabilities(Capability::TyVecCapabilityTofs const & v, char const * xmlTag, icu::UnicodeString & s) {
    size_t i;
    for (i=0; i<v.size(); ++i) {
      s.append("<");
      s.append(xmlTag);
      s.append(">");

      s.append(v[i]);

      s.append("</");
      s.append(xmlTag);
      s.append(">");
    }
  }

  void appendSofaName(Capability::TyVecCapabilitySofas const & v, char const * xmlTag, icu::UnicodeString & s) {
    size_t i;
    for (i=0; i<v.size(); ++i) {
      s.append("<");
      s.append(xmlTag);
      s.append(">");

      s.append(v[i]);

      s.append("</");
      s.append(xmlTag);
      s.append(">");
    }
  }

  void appendBool(bool b, icu::UnicodeString & s) {
    if (b) {
      s.append("true");
    } else {
      s.append("false");
    }
  }

  void appendFloat(float f, icu::UnicodeString & s) {
    char c[256];
    sprintf(c, "%f", f);
    s.append(c);
  }

  void appendInt(int i, icu::UnicodeString & s) {
    char c[256];
    sprintf(c, "%d", i);
    s.append(c);
  }


  void AnalysisEngineDescription::appendConfigGroupToXMLBuffer(ConfigurationGroup const & config, icu::UnicodeString & s) const {
    size_t i;
    vector<ConfigurationParameter const *> configParams = config.getConfigurationParameters();
    for (i=0; i<configParams.size(); ++i) {
      ConfigurationParameter const * confParam = configParams[i];
      s.append("<configurationParameter>");
      s.append("<name>");
      s.append(confParam->getName());
      s.append("</name>");
      s.append("<description>");
      s.append(confParam->getDescription());
      s.append("</description>");
      s.append("<type>");
      ConfigurationParameter::EnParameterType paramType = confParam->getType();
      switch (paramType) {
      case ConfigurationParameter::BOOLEAN:
        s.append("Boolean");
        break;
      case ConfigurationParameter::FLOAT:
        s.append("Float");
        break;
      case ConfigurationParameter::INTEGER:
        s.append("Integer");
        break;
      case ConfigurationParameter::STRING:
        s.append("String");
        break;
      default:
        assert(false);
      }
      s.append("</type>");
      s.append("<multiValued>");
      appendBool(confParam->isMultiValued(), s);
      s.append("</multiValued>");
      s.append("<mandatory>");
      appendBool(confParam->isMandatory(), s);
      s.append("</mandatory>");
      s.append("</configurationParameter>");
    }
  }

  void AnalysisEngineDescription::appendConfigGroupSettingsToXMLBuffer(SettingsForGroup const & settings, icu::UnicodeString & s) const {
    size_t i,j;
    vector<NameValuePair const *> nvps = settings.getNameValuePairs();
    for (i=0; i<nvps.size(); ++i) {
      NameValuePair const * nvp = nvps[i];
      s.append("<nameValuePair>");
      s.append("<name>");
      s.append( nvp->getName() );
      s.append("</name>");

      s.append("<value>");

      ConfigurationParameter::EnParameterType paramType = nvp->getType();
      if (nvp->isMultiValued()) {
        s.append("<array>");

        switch (paramType) {
        case ConfigurationParameter::BOOLEAN: {
            vector<bool> const & v = nvp->extractBoolValues();
            for (j=0; j<v.size(); ++j) {
              s.append("<boolean>");
              appendBool(v[j], s);
              s.append("</boolean>");
            }
            break;
          }
        case ConfigurationParameter::FLOAT: {
            vector<float> const & v = nvp->extractFloatValues();
            for (j=0; j<v.size(); ++j) {
              s.append("<float>");
              appendFloat(v[j], s);
              s.append("</float>");
            }
            break;
          }
        case ConfigurationParameter::INTEGER: {
            vector<int> const & v = nvp->extractIntValues();
            for (j=0; j<v.size(); ++j) {
              s.append("<integer>");
              appendInt(v[j], s);
              s.append("</integer>");
            }
            break;
          }
        case ConfigurationParameter::STRING: {
            vector<icu::UnicodeString> const & v = nvp->extractStringValues();
            for (j=0; j<v.size(); ++j) {
              s.append("<string>");
              s.append(v[j]);
              s.append("</string>");
            }
            break;
          }
        default:
          assert(false);
        }

        s.append("</array>");
      } else {
        switch (paramType) {
        case ConfigurationParameter::BOOLEAN: {
            bool v = nvp->extractBoolValue();
            s.append("<boolean>");
            appendBool(v, s);
            s.append("</boolean>");
            break;
          }
        case ConfigurationParameter::FLOAT: {
            float v = nvp->extractFloatValue();
            s.append("<float>");
            appendFloat(v, s);
            s.append("</float>");
            break;
          }
        case ConfigurationParameter::INTEGER: {
            int v = nvp->extractIntValue();
            s.append("<integer>");
            appendInt(v, s);
            s.append("</integer>");
            break;
          }
        case ConfigurationParameter::STRING: {
            icu::UnicodeString const & v = nvp->extractStringValue();
            s.append("<string>");
            s.append(v);
            s.append("</string>");
            break;
          }
        default:
          assert(false);
        }

      }

      s.append("</value>");
      s.append("</nameValuePair>");
    }
  }




  void  AnalysisEngineDescription::appendToXMLBuffer
  (AnalysisEngineMetaData::TyVecpFSIndexDescriptions const & fsDesc,
   icu::UnicodeString & s) {

    size_t i,j;
    s.append("<fsIndexes>");
    AnalysisEngineMetaData::TyVecpFSIndexDescriptions const & ixVec = fsDesc;
    for (i=0; i<ixVec.size(); ++i) {
      FSIndexDescription const * ixDesc = ixVec[i];
      s.append("<fsIndexDescription>");
      s.append("<label>");
      s.append(ixDesc->getLabel());
      s.append("</label>");
      s.append("<typeName>");
      s.append(ixDesc->getTypeName());
      s.append("</typeName>");
      s.append("<kind>");
      FSIndexDescription::EnIndexKind ixKind = ixDesc->getIndexKind();
      switch (ixKind) {
      case FSIndexDescription::BAG:
        s.append("bag");
        break;
      case FSIndexDescription::SET:
        s.append("set");
        break;
      case FSIndexDescription::SORTED:
        s.append("sorted");
        break;
      default:
        assert(false);
      }
      s.append("</kind>");

      FSIndexDescription::TyVecpFSIndexKeys const & keyVec = ixDesc->getFSIndexKeys();
      if (keyVec.size() > 0) {
        s.append("<keys>");
        for (j=0; j<keyVec.size(); ++j) {
          s.append("<fsIndexKey>");
          FSIndexKeyDescription const * ixKeyDesc = keyVec[j];
          if (ixKeyDesc->isTypePriority() ) {
            s.append("<typePriority/>");
          } else {
              s.append("<featureName>");
              s.append(ixKeyDesc->getFeatureName());
              s.append("</featureName>");

              s.append("<comparator>");
              FSIndexKeyDescription::EnComparatorType comp = ixKeyDesc->getComparator();
              switch (comp) {
              case FSIndexKeyDescription::STANDARD:
                s.append("standard");
                break;
              case FSIndexKeyDescription::REVERSE:
                s.append("reverse");
                break;
              default:
                assert(false);
              }
            s.append("</comparator>");
          }
          s.append("</fsIndexKey>");
        }
        s.append("</keys>");
      }

      s.append("</fsIndexDescription>");
    }

    s.append("</fsIndexes>");

  }



  void AnalysisEngineDescription::appendToXMLBuffer
  (AnalysisEngineMetaData::TyVecpTypePriorities const & prioDesc,
   icu::UnicodeString & s) {
    size_t i,j;
    if (prioDesc.size() > 0) {
      s.append("<typePriorities>");
      for (i=0; i<prioDesc.size() ;++i) {
        s.append("<priorityList>");
        TypePriority * prio = prioDesc[i];
        vector <icu::UnicodeString> vecTypeOrder = prio->getTypeOrder();
        for ( j = 0; j < vecTypeOrder.size();  ++j) {

          s.append("<type>");
          s.append(vecTypeOrder[j]);
          s.append("</type>");
        }
        s.append("</priorityList>");
      }
      s.append("</typePriorities>");
    }
  }


  void AnalysisEngineDescription::appendToXMLBuffer
  (AnalysisEngineDescription::TyVecpSofaMappings const & sofaMapDesc,
   icu::UnicodeString & s) {
    size_t i;
    AnalysisEngineDescription::TyVecpSofaMappings const & sofamappingVec = sofaMapDesc;
    if (sofamappingVec.size() >  0) {
      s.append("<sofaMappings>");
      for (i=0; i<sofamappingVec.size(); ++i) {
        s.append("<sofaMapping>");
        s.append("<componentKey>");
        s.append(sofamappingVec[i]->getComponentKey());
        s.append("</componentKey>");

        s.append("<componentSofaName>");
        s.append(sofamappingVec[i]->getComponentSofaName());
        s.append("</componentSofaName>");

        s.append("<aggregateSofaName>");
        s.append(sofamappingVec[i]->getAggregateSofaName());
        s.append("</aggregateSofaName>");

        s.append("</sofaMapping>");
      }
      s.append("</sofaMappings>");
    }
  }



  void AnalysisEngineDescription::appendConfigParamsAndSettingsToXMLBuffer(icu::UnicodeString & s) const  {


    AnalysisEngineMetaData const & md = * getAnalysisEngineMetaData();
    // config params
    if (md.hasGroups()) {
      s.append("<configurationParameters");

      if (md.hasDefaultGroup()) {
        s.append(" defaultGroup=\"");
        s.append( md.iv_defaultGroup );
        s.append("\"");
      }

      s.append(" searchStrategy=\"");
      AnalysisEngineMetaData::EnSearchStrategy searchStrategy = md.getSearchStrategy();
      switch (searchStrategy) {
      case AnalysisEngineMetaData::DEFAULT_FALLBACK:
        s.append("default_fallback");
        break;
      case AnalysisEngineMetaData::LANGUAGE_FALLBACK:
        s.append("language_fallback");
        break;
      case AnalysisEngineMetaData::NONE:
        s.append("none");
        break;
      }
      s.append("\">");

      AnalysisEngineMetaData::TyConfigGroup const & confGroups = md.iv_configurationGroups;
      assert( confGroups.find(md.CONFIG_GROUP_NAME_WHEN_NO_GROUPS) == confGroups.end() );
      AnalysisEngineMetaData::TyConfigGroup::const_iterator cit;
      for ( cit = confGroups.begin(); cit != confGroups.end(); ++cit) {
        icu::UnicodeString const & confGroupName = (*cit).first;
        ConfigurationGroup const & confGroup = (*cit).second;
        bool common = ( confGroupName == md.CONFIG_GROUP_NAME_COMMON_PARMS);
        if (common) {
          s.append("<commonParameters>");
        } else {
          s.append("<configurationGroup names=\"");
          s.append(confGroupName);
          s.append("\">");
        }

        appendConfigGroupToXMLBuffer( confGroup, s);

        if (common) {
          s.append("</commonParameters>");
        } else {
          s.append("</configurationGroup>");
        }
      }
      s.append("</configurationParameters>");

    } else {
      s.append("<configurationParameters>");
      AnalysisEngineMetaData::TyConfigGroup const & confGroups = md.iv_configurationGroups;
      assert( confGroups.find(md.CONFIG_GROUP_NAME_COMMON_PARMS) == confGroups.end() );
      if (confGroups.size() > 0) {
        assert( confGroups.size() == 1);
        AnalysisEngineMetaData::TyConfigGroup::const_iterator cit = confGroups.find(md.CONFIG_GROUP_NAME_WHEN_NO_GROUPS);
        assert( cit != confGroups.end() );
        appendConfigGroupToXMLBuffer( (*cit).second, s);
      }
      s.append("</configurationParameters>");
    }


    s.append("<configurationParameterSettings>");
    // config param settings
    if (md.hasGroups()) {
      AnalysisEngineMetaData::TyConfigSettings const & confSettings = md.iv_configurationSettings;
      AnalysisEngineMetaData::TyConfigSettings::const_iterator cit;
      for (cit = confSettings.begin(); cit != confSettings.end(); ++cit) {
        icu::UnicodeString const & groupName = (*cit).first;
        SettingsForGroup const & settings = (*cit).second;

        s.append("<settingsForGroup name=\"");
        s.append(groupName);
        s.append("\">");
        appendConfigGroupSettingsToXMLBuffer(settings, s);
        s.append("</settingsForGroup>");
      }

    } else {
      AnalysisEngineMetaData::TyConfigSettings const & confSettings = md.iv_configurationSettings;
      if (confSettings.size() > 0) {
        assert( confSettings.size() == 1);
        AnalysisEngineMetaData::TyConfigSettings::const_iterator cit = confSettings.find(md.CONFIG_GROUP_NAME_WHEN_NO_GROUPS);
        assert( cit != confSettings.end() );
        appendConfigGroupSettingsToXMLBuffer( (*cit).second, s);
      }
    }
    s.append("</configurationParameterSettings>");


  }




  void AnalysisEngineDescription::toXMLBufferNoXMLHeader(icu::UnicodeString & s) const {
    size_t i;
    bool isCasConsumer=false;
    if ( getXmlRootTag().compare("casConsumerDescription") == 0) {
      isCasConsumer=true;
    }

    s = "<";
    s.append(getXmlRootTag());
    s.append(" xmlns=\"" UIMA_XML_NAMESPACE "\">");

    // framework
    s.append("<frameworkImplementation>");
    EnFrameworkImplName fwName = getFrameworkImplName();
    switch (fwName) {
    case CPLUSPLUS:
      s.append("org.apache.uima.cpp");
      break;
    case JAVA:
      s.append("org.apache.uima.java");
      break;
    default:
      assert(false);
    };
    s.append("</frameworkImplementation>");


    if (isCasConsumer) {
      s.append("<implementationName>");
      s.append(getAnnotatorImpName());
      s.append("</implementationName>");
    }
    // primitive
    else  {
      // primitive
      s.append("<primitive>");
      appendBool(isPrimitive(), s);
      s.append("</primitive>");

      if (isPrimitive()) {
        // annotator implementation
        s.append("<annotatorImplementationName>");
        s.append(getAnnotatorImpName());
        s.append("</annotatorImplementationName>");
      } else {
        // delegates
        s.append("<delegateAnalysisEngineSpecifiers>");
        TyMapDelegateSpecs const & delegates = getDelegates();
        TyMapDelegateSpecs::const_iterator cit;
        for (cit = delegates.begin(); cit != delegates.end(); ++cit) {
          s.append("<delegateAnalysisEngine key=\"");
          s.append( (*cit).first );
          s.append("\">");
          icu::UnicodeString delBuf;
          (*cit).second->toXMLBufferNoXMLHeader(delBuf);
          s.append(delBuf);
          s.append("</delegateAnalysisEngine>");
        }
        s.append("</delegateAnalysisEngineSpecifiers>");
      }
    }

	toXMLBuffer( *(getAnalysisEngineMetaData()), isCasConsumer, s);

	if (!isCasConsumer) {
      TyVecpSofaMappings const & sofamappingVec = getSofaMappings();
      if (sofamappingVec.size() >  0) {
        s.append("<sofaMappings>");
        for (i=0; i<sofamappingVec.size(); ++i) {
          s.append("<sofaMapping>");
          s.append("<componentKey>");
          s.append(sofamappingVec[i]->getComponentKey());
          s.append("</componentKey>");

          s.append("<componentSofaName>");
          s.append(sofamappingVec[i]->getComponentSofaName());
          s.append("</componentSofaName>");

          s.append("<aggregateSofaName>");
          s.append(sofamappingVec[i]->getAggregateSofaName());
          s.append("</aggregateSofaName>");

          s.append("</sofaMapping>");
        }
        s.append("</sofaMappings>");
      }
    }

    s.append("</");
    s.append(getXmlRootTag());
    s.append(">");


  }

  void AnalysisEngineDescription::toXMLBuffer(AnalysisEngineMetaData const & md,  
													   bool isCasConsumer,
													   icu::UnicodeString & s) const {
    size_t i,j;
    // AE Meta data
	
    if (isCasConsumer) {
      s.append("<processingResourceMetaData>");
    } else s.append("<analysisEngineMetaData xmlns=\"http://uima.apache.org/resourceSpecifier\">");
    

    ///AnalysisEngineMetaData const & md = * getAnalysisEngineMetaData();
    
    //    name
    icu::UnicodeString out;
    s.append("<name>");
    XMLWriterABase::normalize(UnicodeStringRef(md.getName()), out);
    s.append(out);
    s.append("</name>");

    //   description
    s.append("<description>");
    out.setTo("");
    XMLWriterABase::normalize(UnicodeStringRef(md.getDescription()), out);
    s.append( out );
    s.append("</description>");

    //   version
    s.append("<version>");
    s.append( md.getVersion() );
    s.append("</version>");

    //   vendor
    s.append("<vendor>");
    out.setTo("");
     XMLWriterABase::normalize(UnicodeStringRef(md.getVendor()), out);
    s.append( out );
    s.append("</vendor>");


    // config params
    if (md.hasGroups()) {
      s.append("<configurationParameters");

      if (md.hasDefaultGroup()) {
        s.append(" defaultGroup=\"");
        s.append( md.iv_defaultGroup );
        s.append("\"");
      }

      s.append(" searchStrategy=\"");
      AnalysisEngineMetaData::EnSearchStrategy searchStrategy = md.getSearchStrategy();
      switch (searchStrategy) {
      case AnalysisEngineMetaData::DEFAULT_FALLBACK:
        s.append("default_fallback");
        break;
      case AnalysisEngineMetaData::LANGUAGE_FALLBACK:
        s.append("language_fallback");
        break;
      case AnalysisEngineMetaData::NONE:
        s.append("none");
        break;
      }
      s.append("\">");

      AnalysisEngineMetaData::TyConfigGroup const & confGroups = md.iv_configurationGroups;
      assert( confGroups.find(md.CONFIG_GROUP_NAME_WHEN_NO_GROUPS) == confGroups.end() );
      AnalysisEngineMetaData::TyConfigGroup::const_iterator cit;
      for ( cit = confGroups.begin(); cit != confGroups.end(); ++cit) {
        icu::UnicodeString const & confGroupName = (*cit).first;
        ConfigurationGroup const & confGroup = (*cit).second;
        bool common = ( confGroupName == md.CONFIG_GROUP_NAME_COMMON_PARMS);
        if (common) {
          s.append("<commonParameters>");
        } else {
          s.append("<configurationGroup names=\"");
          s.append(confGroupName);
          s.append("\">");
        }

        appendConfigGroupToXMLBuffer( confGroup, s);

        if (common) {
          s.append("</commonParameters>");
        } else {
          s.append("</configurationGroup>");
        }
      }
      s.append("</configurationParameters>");

    } else {
      s.append("<configurationParameters>");
      AnalysisEngineMetaData::TyConfigGroup const & confGroups = md.iv_configurationGroups;
      assert( confGroups.find(md.CONFIG_GROUP_NAME_COMMON_PARMS) == confGroups.end() );
      if (confGroups.size() > 0) {
        assert( confGroups.size() == 1);
        AnalysisEngineMetaData::TyConfigGroup::const_iterator cit = confGroups.find(md.CONFIG_GROUP_NAME_WHEN_NO_GROUPS);
        assert( cit != confGroups.end() );
        appendConfigGroupToXMLBuffer( (*cit).second, s);
      }
      s.append("</configurationParameters>");
    }


    s.append("<configurationParameterSettings>");
    // config param settings
    if (md.hasGroups()) {
      AnalysisEngineMetaData::TyConfigSettings const & confSettings = md.iv_configurationSettings;
      AnalysisEngineMetaData::TyConfigSettings::const_iterator cit;
      for (cit = confSettings.begin(); cit != confSettings.end(); ++cit) {
        icu::UnicodeString const & groupName = (*cit).first;
        SettingsForGroup const & settings = (*cit).second;

        s.append("<settingsForGroup name=\"");
        s.append(groupName);
        s.append("\">");
        appendConfigGroupSettingsToXMLBuffer(settings, s);
        s.append("</settingsForGroup>");
      }

    } else {
      AnalysisEngineMetaData::TyConfigSettings const & confSettings = md.iv_configurationSettings;
      if (confSettings.size() > 0) {
        assert( confSettings.size() == 1);
        AnalysisEngineMetaData::TyConfigSettings::const_iterator cit = confSettings.find(md.CONFIG_GROUP_NAME_WHEN_NO_GROUPS);
        assert( cit != confSettings.end() );
        appendConfigGroupSettingsToXMLBuffer( (*cit).second, s);
      }
    }
    s.append("</configurationParameterSettings>");

    // flow constraints
    if (!isPrimitive()) {
      s.append("<flowConstraints>");
      FlowConstraints const * pFlow = md.getFlowConstraints();
      FlowConstraints * flow = CONST_CAST(FlowConstraints *, pFlow);
      if (flow->getFlowConstraintsType() == FlowConstraints::FIXED) {
        s.append("<fixedFlow>");
      } else if (flow->getFlowConstraintsType() == FlowConstraints::CAPABILITYLANGUAGE) {
        s.append("<capabilityLanguageFlow>");
      }
      vector<icu::UnicodeString> const & v = flow->getNodes();
      for (i=0; i<v.size(); ++i) {
        s.append("<node>");
        s.append(v[i]);
        s.append("</node>");
      }
      if (flow->getFlowConstraintsType() == FlowConstraints::FIXED) {
        s.append("</fixedFlow>");
      }  else if (flow->getFlowConstraintsType() == FlowConstraints::CAPABILITYLANGUAGE) {
        s.append("</capabilityLanguageFlow>");
      }

      s.append("</flowConstraints>");
    }

    //type priorites
    const AnalysisEngineMetaData::TyVecpTypePriorities  & prioDesc =  
                    md.getTypePriorities();
     if (prioDesc.size() > 0) {
      s.append("<typePriorities>");
      for (i=0; i<prioDesc.size() ;++i) {
        s.append("<priorityList>");
        TypePriority * prio = prioDesc[i];
        vector <icu::UnicodeString> vecTypeOrder = prio->getTypeOrder();
        for ( j = 0; j < vecTypeOrder.size();  ++j) {

          s.append("<type>");
          s.append(vecTypeOrder[j]);
          s.append("</type>");
        }
        s.append("</priorityList>");
      }
      s.append("</typePriorities>");
    }

    // type system

      s.append("<typeSystemDescription>");
      TypeSystemDescription const * typeDesc = md.getTypeSystemDescription();
      if (typeDesc != NULL) {
        TypeSystemDescription::TyVecpImportDescriptions const & importVec = typeDesc->getImportDescriptions();
        s.append("<imports>");
        for (i=0; i<importVec.size(); ++i) {
          ImportDescription const * pImport = importVec[i];
          if ( pImport->getName().length() > 0) {
            s.append("<import name=\"");
            s.append(pImport->getName());
            s.append("\"/>");
          } else {
            s.append("<import location=\"");
            s.append(pImport->getLocation());
            s.append("\"/>");
          }
        }
        s.append("</imports><types>");

        TypeSystemDescription::TyVecpTypeDescriptions const & tdVec = typeDesc->getTypeDescriptions();
        for (i=0; i<tdVec.size(); ++i) {
          TypeDescription const * td = tdVec[i];
          s.append("<typeDescription>");
          s.append("<name>");
          s.append(td->getName());
          s.append("</name>");

          s.append("<description>");
          out.setTo("");
          XMLWriterABase::normalize(UnicodeStringRef(td->getDescription()), out);
          s.append(out);
          s.append("</description>");

          s.append("<supertypeName>");
          s.append(td->getSuperTypeName());
          s.append("</supertypeName>");

          TypeDescription::TyVecpFeatureDescriptions const & fdVec = td->getFeatureDescriptions();
          if (fdVec.size() > 0) {
            s.append("<features>");
            for (j=0; j<fdVec.size(); ++j) {
              FeatureDescription const * fs = fdVec[j];
              s.append("<featureDescription>");
              s.append("<name>");
              s.append(fs->getName());
              s.append("</name>");
              s.append("<description>");
              out.setTo("");
              XMLWriterABase::normalize(UnicodeStringRef(fs->getDescription()), out);
              s.append(out);
              s.append("</description>");
              s.append("<rangeTypeName>");
              s.append(fs->getRangeTypeName());
              s.append("</rangeTypeName>");
              if (0 < fs->getElementType().length() ) {
                s.append("<elementType>");
                s.append(fs->getElementType());
                s.append("</elementType>");
              }
              if (fs->isMultipleReferencesAllowed() ) {
                s.append("<multipleReferencesAllowed>");
                s.append("true");
                s.append("</multipleReferencesAllowed>");
              }
              s.append("</featureDescription>");
            }
            s.append("</features>");
          }

          TypeDescription::TyVecpAllowedValues const & allowVec = td->getAllowedValues();
          if (allowVec.size() > 0) {
            s.append("<allowedValues>");
            for (j=0; j<allowVec.size(); ++j) {
              AllowedValue const * value = allowVec[j];
              s.append("<value>");
              s.append("<string>");
              s.append(value->getName());
              s.append("</string>");
              s.append("<description>");
              out.setTo("");
              XMLWriterABase::normalize(UnicodeStringRef(value->getDescription()), out);
              s.append(out);
              s.append("</description>");
              s.append("</value>");
            }
            s.append("</allowedValues>");
          }

          s.append("</typeDescription>");
        }
      }
      s.append("</types></typeSystemDescription>");

    //fsIndexCollection
    s.append("<fsIndexCollection>");
    //fsIndexCollection -- imports
    AnalysisEngineMetaData::TyVecpFSIndexImportDescriptions const & ixImportVec = md.getFSIndexImportDescriptions();
    if (ixImportVec.size() > 0 ) {
      s.append("<imports>");
      for (size_t i=0; i < ixImportVec.size(); i++) {
        ImportDescription const * pImport = ixImportVec[i];
        if ( pImport->getName().length() > 0) {
          s.append("<import name=\"");
          s.append(pImport->getName());
          s.append("\"/>");
        } else {
          s.append("<import location=\"");
          s.append(pImport->getLocation());
          s.append("\"/>");
        }
      }
      s.append("</imports>");
    }

    // fs indexes
    s.append("<fsIndexes>");
    AnalysisEngineMetaData::TyVecpFSIndexDescriptions const & ixVec = md.getFSIndexDescriptions();
    for (i=0; i<ixVec.size(); ++i) {
      FSIndexDescription const * ixDesc = ixVec[i];
      s.append("<fsIndexDescription>");
      s.append("<label>");
      s.append(ixDesc->getLabel());
      s.append("</label>");
      s.append("<typeName>");
      s.append(ixDesc->getTypeName());
      s.append("</typeName>");
      s.append("<kind>");
      FSIndexDescription::EnIndexKind ixKind = ixDesc->getIndexKind();
      switch (ixKind) {
      case FSIndexDescription::BAG:
        s.append("bag");
        break;
      case FSIndexDescription::SET:
        s.append("set");
        break;
      case FSIndexDescription::SORTED:
        s.append("sorted");
        break;
      default:
        assert(false);
      }
      s.append("</kind>");

      FSIndexDescription::TyVecpFSIndexKeys const & keyVec = ixDesc->getFSIndexKeys();
      if (keyVec.size() > 0) {
        s.append("<keys>");
        for (j=0; j<keyVec.size(); ++j) {
          s.append("<fsIndexKey>");
          FSIndexKeyDescription const * ixKeyDesc = keyVec[j];
          if (ixKeyDesc->isTypePriority() ) {
            s.append("<typePriority/>");
          } else {
            s.append("<featureName>");
            s.append(ixKeyDesc->getFeatureName());
            s.append("</featureName>");

            s.append("<comparator>");
            FSIndexKeyDescription::EnComparatorType comp = ixKeyDesc->getComparator();
            switch (comp) {
            case FSIndexKeyDescription::STANDARD:
              s.append("standard");
              break;
            case FSIndexKeyDescription::REVERSE:
              s.append("reverse");
              break;
            default:
              assert(false);
            }
            s.append("</comparator>");
          }
          s.append("</fsIndexKey>");
        }
        s.append("</keys>");
      }

      s.append("</fsIndexDescription>");
    }

    s.append("</fsIndexes>");

    s.append("</fsIndexCollection>");

    // capabilities
    s.append("<capabilities>");
    AnalysisEngineMetaData::TyVecpCapabilities const & capVec = md.getCapabilites();
    for (i=0; i<capVec.size(); ++i) {
      Capability const * cap = capVec[i];
      s.append("<capability>");
      s.append("<inputs>");
      appendCapabilities( cap->getCapabilityTypes(Capability::INPUT), "type", s);
      appendCapabilities( cap->getCapabilityFeatures(Capability::INPUT), "feature", s);
      s.append("</inputs>");
      s.append("<outputs>");
      appendCapabilities( cap->getCapabilityTypes(Capability::OUTPUT), "type", s);
      appendCapabilities( cap->getCapabilityFeatures(Capability::OUTPUT), "feature", s);
      s.append("</outputs>");

      s.append("<inputSofas>");
      appendCapabilities( cap->getCapabilitySofas(Capability::INPUTSOFA), "sofaName", s);
      s.append("</inputSofas>");

      s.append("<outputSofas>");
      appendCapabilities( cap->getCapabilitySofas(Capability::OUTPUTSOFA), "sofaName", s);
      s.append("</outputSofas>");


      s.append("<languagesSupported>");
      Capability::TyVecCapabilityLanguages const & suppLangVec = cap->getSupportedLanguages();
      for (j=0; j<suppLangVec.size(); ++j) {
        s.append("<language>");
        s.append(suppLangVec[j]);
        s.append("</language>");
      }
      s.append("</languagesSupported>");

      s.append("</capability>");
    }

    s.append("</capabilities>");
    
    if (this->getAnalysisEngineMetaData()->getOperationalProperties() != 0) {
      s.append("<operationalProperties>"); 
      s.append ("<modifiesCas>");
      appendBool(this->getAnalysisEngineMetaData()->getOperationalProperties()->getModifiesCas(), s);
      s.append ("</modifiesCas>");
      s.append ("<multipleDeploymentAllowed>");
      appendBool(this->getAnalysisEngineMetaData()->getOperationalProperties()->isMultipleDeploymentAllowed(), s);
      s.append("</multipleDeploymentAllowed>");
      s.append("<outputsNewCASes>");
      appendBool(this->getAnalysisEngineMetaData()->getOperationalProperties()->getOutputsNewCASes(), s);
      s.append("</outputsNewCASes>");
      s.append("</operationalProperties>");
    } else {
      s.append("<operationalProperties>"); 
      s.append ("<modifiesCas>true</modifiesCas>");
      s.append("<multipleDeploymentAllowed>true</multipleDeploymentAllowed>");
      s.append("<outputsNewCASes>false</outputsNewCASes>");
      s.append("</operationalProperties>");
    }
    if (isCasConsumer) {
      s.append("</processingResourceMetaData>");
    } else s.append("</analysisEngineMetaData>");
	
 }

    
} //namespace



