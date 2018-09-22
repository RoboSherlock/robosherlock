/** \file annotator_context.cpp .
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

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */
#include <uima/pragmas.hpp> //must be included first to disable warnings
#include <uima/err_ids.h>
#include <uima/msg.h>
#include <uima/log.hpp>
#include <uima/annotator_context.hpp>
#include <uima/taespecifier.hpp>
#include <uima/config_param.hpp>
#include <uima/taespecifierbuilder.hpp>
#include <uima/internal_casimpl.hpp>

using namespace std;
namespace uima {

#define CTX_DEFAULT_TEXT_SOFA_NAME  "_DefaultTextSofaName"

  AnnotatorContext::AnnotatorContext(AnalysisEngineDescription * delegateSpec,
                                     AnnotatorContext * parent,
                                     const icu::UnicodeString & ancKey)
      :iv_pParentAnC(parent), iv_AnCKey(ancKey),
      iv_pTaeSpecifier(delegateSpec), iv_mapDelegateAncs(), iv_pCasPool(NULL)  {
    buildDelegateAnCs();
    iv_pLogger = new LogFacility( delegateSpec->getAnalysisEngineMetaData()->getName() );
    assert(EXISTS(iv_pLogger) );
    bOwnsTaeSpecifier=false;
  }

  AnnotatorContext::AnnotatorContext(AnalysisEngineDescription * taeSpec)
      :iv_pParentAnC(NULL), iv_AnCKey(""), iv_pTaeSpecifier(taeSpec),
      iv_mapDelegateAncs(), iv_pCasPool(NULL) {
    buildDelegateAnCs();
    iv_pLogger = new LogFacility( taeSpec->getAnalysisEngineMetaData()->getName() );
    assert(EXISTS(iv_pLogger) );
    bOwnsTaeSpecifier=false;
  }



  AnnotatorContext::AnnotatorContext(AnalysisEngineDescription * delegateSpec,
                                     AnnotatorContext * parent,
                                     const icu::UnicodeString & ancKey,
                                     const AnalysisEngineDescription::TyVecpSofaMappings & vecSofaMappings)
      :iv_pParentAnC(parent), iv_AnCKey(ancKey),
      iv_pTaeSpecifier(delegateSpec), iv_mapDelegateAncs(), iv_pCasPool(NULL) {
    //process the sofa mappings
    AnalysisEngineDescription::TyVecpSofaMappings::const_iterator iter;

    for (iter = vecSofaMappings.begin(); iter != vecSofaMappings.end(); ++iter) {
      SofaMapping * sofamap = * iter;
      //if this sofamapping is for this context
      if (sofamap->getComponentKey().compare(ancKey) == 0) {
        // add mapping to this context sofa mapping
        // where component sofa name is the key and
        // aggregate sofaname is the value
        // if component sofa name is not set,
        // set it to the default for tcas sofas.
        if (sofamap->getComponentSofaName().length() == 0) {
          icu::UnicodeString ustr(CAS::NAME_DEFAULT_TEXT_SOFA);
          sofamap->setComponentSofaName(ustr);
        }
        SofaID * pSofaid = new SofaID();
        pSofaid->setSofaId(sofamap->getAggregateSofaName());
        pSofaid->setComponentSofaName(sofamap->getComponentSofaName());
        icu::UnicodeString astr = sofamap->getComponentSofaName();
        iv_mapSofaMappings[astr] = pSofaid;

        //if this delegate is an aggregate and has sofamapping
        //check if somponent sofaname is specified as the
        //aggregate sofa name in this delegate specifier
        //if so update that aggregate sofa name
        if (!delegateSpec->isPrimitive()) {
          AnalysisEngineDescription::TyVecpSofaMappings delegateSofaMappings = delegateSpec->getSofaMappings();
          if (delegateSofaMappings.size() > 0 ) {
            AnalysisEngineDescription::TyVecpSofaMappings::const_iterator iter;

            for (iter = delegateSofaMappings.begin(); iter != delegateSofaMappings.end(); ++iter) {
              SofaMapping  * delsofamap = * iter;
              if (delsofamap->getAggregateSofaName().compare(astr)==0) {
                delsofamap->setAggregateSofaName(pSofaid->getSofaId());
              }
            }


          }
        }


      }

    }

    buildDelegateAnCs();

    iv_pLogger = new LogFacility( delegateSpec->getAnalysisEngineMetaData()->getName() );
    assert(EXISTS(iv_pLogger) );
    bOwnsTaeSpecifier=false;
  }


  AnnotatorContext::~AnnotatorContext(void) {
    TyMapDelegateAnCs::iterator ite;
    for (ite=iv_mapDelegateAncs.begin(); ite != iv_mapDelegateAncs.end(); ite++) {
      delete (ite->second);
    }
    //BSI
    TyMapSofaMappings::iterator ite2;
    for (ite2=iv_mapSofaMappings.begin(); ite2 != iv_mapSofaMappings.end(); ite2++) {
      delete (ite2->second);
    }

    if (EXISTS(iv_pLogger) )
      delete iv_pLogger;

    if (bOwnsTaeSpecifier) {
      delete iv_pTaeSpecifier;
    }

    if (iv_pCasPool != NULL) {
      delete iv_pCasPool;
    }
  }


  CAS & AnnotatorContext::getEmptyCAS()  {

    if (iv_pCasPool)
      return iv_pCasPool->getCAS();
    else {
      UIMA_EXC_THROW_NEW(CASPoolException,
                         UIMA_ERR_CASPOOL_CREATE_CAS,
                         UIMA_MSG_ID_EXC_CREATE_CASPOOL,
                         UIMA_MSG_ID_EXC_CREATE_CASPOOL,
                         ErrorInfo::unrecoverable);
    }
  }

  void AnnotatorContext::releaseCAS(CAS & aCas) {
    if (iv_pCasPool)
      iv_pCasPool->releaseCAS(aCas);
  }

  TyErrorId AnnotatorContext::defineCASPool(size_t numInstances) {
    iv_pCasPool = new CASPool(getTaeSpecifier(),numInstances);
    if (iv_pCasPool == NULL) {
      return UIMA_ERR_USER_ANNOTATOR_OUT_OF_MEMORY;
    }
    return UIMA_ERR_NONE;
  }

  AnnotatorContext * AnnotatorContext::getDelegate(const icu::UnicodeString & key) const {
    TyMapDelegateAnCs::const_iterator ite = iv_mapDelegateAncs.find(key);
    if (ite != iv_mapDelegateAncs.end()) {
      return ite->second;
    } else {
      return NULL;
    }
  }

  AnnotatorContext * AnnotatorContext::extractDelegate(const icu::UnicodeString & key) {
    TyMapDelegateAnCs::iterator ite = iv_mapDelegateAncs.find(key);
    if (ite != iv_mapDelegateAncs.end()) {
      AnnotatorContext * anCon = ite->second;
      iv_mapDelegateAncs.erase(ite);
      return anCon;
    } else {
      return NULL;
    }
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & paramName,
                                          const bool & value) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::BOOLEAN, ConfigurationParameter::SINGLE_VALUE);
    nvPair.setValue(value);
    return setNameValuePair(nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & paramName,
                                          const vector< bool > & values) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::BOOLEAN, ConfigurationParameter::MULTIPLE_VALUES);
    for (size_t i=0; i < values.size(); i++) {
      nvPair.addValue(values[i]);
    }
    return setNameValuePair(nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & paramName,
                                          const int & value) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::INTEGER, ConfigurationParameter::SINGLE_VALUE);
    nvPair.setValue(value);
    return setNameValuePair(nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & paramName,
                                          const vector< int > & values) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::INTEGER, ConfigurationParameter::MULTIPLE_VALUES);
    for (size_t i=0; i < values.size(); i++) {
      nvPair.addValue(values[i]);
    }
    return setNameValuePair(nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & paramName,
                                          const size_t & value) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::INTEGER, ConfigurationParameter::SINGLE_VALUE);
    nvPair.setValue((int)value);
    return setNameValuePair(nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & paramName,
                                          const vector< size_t > & values) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::INTEGER, ConfigurationParameter::MULTIPLE_VALUES);
    for (size_t i=0; i < values.size(); i++) {
      nvPair.addValue((int)values[i]);
    }
    return setNameValuePair(nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & paramName,
                                          const float & value) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::FLOAT, ConfigurationParameter::SINGLE_VALUE);
    nvPair.setValue(value);
    return setNameValuePair(nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & paramName,
                                          const vector< float > & values) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::FLOAT, ConfigurationParameter::MULTIPLE_VALUES);
    for (size_t i=0; i < values.size(); i++) {
      nvPair.addValue(values[i]);
    }
    return setNameValuePair(nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & paramName,
                                          const double & value) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::FLOAT, ConfigurationParameter::SINGLE_VALUE);
    nvPair.setValue((float)value);
    return setNameValuePair(nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & paramName,
                                          const vector< double > & values) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::FLOAT, ConfigurationParameter::MULTIPLE_VALUES);
    for (size_t i=0; i < values.size(); i++) {
      nvPair.addValue((float)values[i]);
    }
    return setNameValuePair(nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & paramName,
                                          const icu::UnicodeString & value) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::STRING, ConfigurationParameter::SINGLE_VALUE);
    nvPair.setValue(value);
    return setNameValuePair(nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & paramName,
                                          const vector< icu::UnicodeString > & values) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::STRING, ConfigurationParameter::MULTIPLE_VALUES);
    for (size_t i=0; i <values.size(); i++) {
      nvPair.addValue(values[i]);
    }
    return setNameValuePair(nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & groupName,
                                          const icu::UnicodeString & paramName,
                                          const bool & value) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::BOOLEAN, ConfigurationParameter::SINGLE_VALUE);
    nvPair.setValue(value);
    return setNameValuePair(groupName, nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & groupName,
                                          const icu::UnicodeString & paramName,
                                          const vector< bool > & values) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::BOOLEAN, ConfigurationParameter::MULTIPLE_VALUES);
    for (size_t i=0; i < values.size(); i++) {
      nvPair.addValue(values[i]);
    }
    return setNameValuePair(groupName, nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & groupName,
                                          const icu::UnicodeString & paramName,
                                          const int & value) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::INTEGER, ConfigurationParameter::SINGLE_VALUE);
    nvPair.setValue(value);
    return setNameValuePair(groupName, nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & groupName,
                                          const icu::UnicodeString & paramName,
                                          const vector< int > & values) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::INTEGER, ConfigurationParameter::MULTIPLE_VALUES);
    for (size_t i=0; i < values.size(); i++) {
      nvPair.addValue(values[i]);
    }
    return setNameValuePair(groupName, nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & groupName,
                                          const icu::UnicodeString & paramName,
                                          const size_t & value) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::INTEGER, ConfigurationParameter::SINGLE_VALUE);
    nvPair.setValue((int)value);
    return setNameValuePair(groupName, nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & groupName,
                                          const icu::UnicodeString & paramName,
                                          const vector< size_t > & values) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::INTEGER, ConfigurationParameter::MULTIPLE_VALUES);
    for (size_t i=0; i < values.size(); i++) {
      nvPair.addValue((int)values[i]);
    }
    return setNameValuePair(groupName, nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & groupName,
                                          const icu::UnicodeString & paramName,
                                          const float & value) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::FLOAT, ConfigurationParameter::SINGLE_VALUE);
    nvPair.setValue(value);
    return setNameValuePair(groupName, nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & groupName,
                                          const icu::UnicodeString & paramName,
                                          const vector< float > & values) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::FLOAT, ConfigurationParameter::MULTIPLE_VALUES);
    for (size_t i=0; i < values.size(); i++) {
      nvPair.addValue(values[i]);
    }
    return setNameValuePair(groupName, nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & groupName,
                                          const icu::UnicodeString & paramName,
                                          const double & value) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::FLOAT, ConfigurationParameter::SINGLE_VALUE);
    nvPair.setValue((float)value);
    return setNameValuePair(groupName, nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & groupName,
                                          const icu::UnicodeString & paramName,
                                          const vector< double > & values) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::FLOAT, ConfigurationParameter::MULTIPLE_VALUES);
    for (size_t i=0; i < values.size(); i++) {
      nvPair.addValue((float)values[i]);
    }
    return setNameValuePair(groupName, nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & groupName,
                                          const icu::UnicodeString & paramName,
                                          const icu::UnicodeString & value) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::STRING, ConfigurationParameter::SINGLE_VALUE);
    nvPair.setValue(value);
    return setNameValuePair(groupName, nvPair);
  }

  TyErrorId AnnotatorContext::assignValue(const icu::UnicodeString & groupName,
                                          const icu::UnicodeString & paramName,
                                          const vector< icu::UnicodeString > & values) {
    NameValuePair nvPair;
    nvPair.setName(paramName);
    nvPair.define(ConfigurationParameter::STRING, ConfigurationParameter::MULTIPLE_VALUES);
    for (size_t i=0; i <values.size(); i++) {
      nvPair.addValue(values[i]);
    }
    return setNameValuePair(groupName, nvPair);
  }

  const AnalysisEngineMetaData::TyVecpFSIndexDescriptions AnnotatorContext::getFSIndexDescriptions() const {
    AnalysisEngineMetaData::TyVecpFSIndexDescriptions allDescs;
    const AnalysisEngineMetaData::TyVecpFSIndexDescriptions & myDescs =
      iv_pTaeSpecifier->getAnalysisEngineMetaData()->getFSIndexDescriptions();
    size_t i;
    for (i=0; i < myDescs.size(); i++) {
      allDescs.push_back(myDescs[i]);
    }
    TyMapDelegateAnCs::const_iterator ite;
    for (ite=iv_mapDelegateAncs.begin(); ite != iv_mapDelegateAncs.end(); ite++) {
      const AnalysisEngineMetaData::TyVecpFSIndexDescriptions & delDesc = ite->second->getFSIndexDescriptions();
      for (i=0; i < delDesc.size(); i++) {
        allDescs.push_back(delDesc[i]);
      }
    }
    return allDescs;
  }

  NameValuePair const *
  AnnotatorContext::findNameValuePair(const icu::UnicodeString & groupName,
                                      const icu::UnicodeString & paramName,
                                      AnalysisEngineMetaData::EnSearchStrategy strategy,
                                      const icu::UnicodeString & ancKey) const {
    NameValuePair const * pValueLocal = iv_pTaeSpecifier->getNameValuePair(groupName, paramName, strategy, ancKey);

    // the request was invalid we got an exception we leave to others to catch

    //now check whether the parent has the value as well
    if (EXISTS(iv_pParentAnC)) {//if it's not the top-most AnC
      //if the parent has the value, errId and nvPair will be set correctly
      NameValuePair const * pValueParent = NULL;
      try {
        pValueParent = iv_pParentAnC->findNameValuePair(groupName, paramName, strategy, iv_AnCKey);
      } catch (ConfigException e) {
        // a parameter may not even be defined at our parent AnC
        // but this is not an error - so we just continue
        assert(pValueParent == NULL);
      }
      if (pValueParent != NULL) {
        return pValueParent;
      }
      //otherwise, nvPair will still contain the NameValuePair found in this AnC
      //(or is still not set, if this AnC doesn't have the value)
    }
    return pValueLocal;
  }

  /**
  * Almost clone of <code>findNameValuePair</code> with 4 params.
  * Reason for cloning: we have to call a different <code>findNameValuePair</code> of the
  * parent than in the other version, i.e. one that returns the default value of the parent, not
  * the value in the parent of the default group of this AnC.
  **/
  NameValuePair const * AnnotatorContext::findNameValuePair(const icu::UnicodeString & paramName,
      const icu::UnicodeString & ancKey) const {
    /*       return findNameValuePair(getGroupNameWhenNotSpec(), paramName, iv_pTaeSpecifier->getSearchStrategy()); */
    NameValuePair const * pValueLocal = pValueLocal = iv_pTaeSpecifier->getNameValuePair(paramName, ancKey);

    // the request was invalid we got an exception we leave to others to catch

    //now check whether the parent has the value as well
    if (EXISTS(iv_pParentAnC)) {//if it's not the top-most AnC
      //if the parent has the value, errId and nvPair will be set correctly
      NameValuePair const * pValueParent = NULL;
      try {
        pValueParent = iv_pParentAnC->findNameValuePair(paramName, iv_AnCKey);
      } catch (ConfigException e) {
        // a parameter may not even be defined at our parent AnC
        // but this is not an error - so we just continue
        assert(pValueParent == NULL);
      }
      if (pValueParent != NULL) {
        return pValueParent;
      }
      //otherwise, nvPair will still contain the NameValuePair found in this AnC
      //(or is still not set, if this AnC doesn't have the value)
    }
    return pValueLocal;
  }

  TyErrorId AnnotatorContext::setNameValuePair(const NameValuePair & nvPair) {

    TyErrorId errId=UIMA_ERR_CONFIG_INVALID_PARAM_NAME;
    if (EXISTS(iv_pParentAnC)) {
      errId = iv_pParentAnC->setNameValuePair(nvPair);
    }

    if (errId != UIMA_ERR_NONE) {//there's either no parent or the nvPair couldn't be set at the parent
      errId = iv_pTaeSpecifier->setNameValuePair(nvPair);
    }
    return errId;
  }

  TyErrorId AnnotatorContext::setNameValuePair(const icu::UnicodeString & groupName,
      const NameValuePair & nvPair) {

    TyErrorId errId=UIMA_ERR_CONFIG_INVALID_PARAM_NAME;
    if (EXISTS(iv_pParentAnC)) {
      errId = iv_pParentAnC->setNameValuePair(groupName, nvPair);
    }

    if (errId != UIMA_ERR_NONE) {//there's either no parent or the nvPair couldn't be set at the parent
      errId = iv_pTaeSpecifier->setNameValuePair(groupName, nvPair);
    }
    return errId;
  }

  void AnnotatorContext::buildDelegateAnCs() {

    assert(EXISTS(iv_pTaeSpecifier));
    const AnalysisEngineDescription::TyMapDelegateSpecs & delegateTaes = iv_pTaeSpecifier->getDelegates();
    AnalysisEngineDescription::TyMapDelegateSpecs::const_iterator ite;

    vector < icu::UnicodeString > delegateNames;
    for (ite = delegateTaes.begin(); ite != delegateTaes.end(); ite++) {
      delegateNames.push_back(ite->first);
    }

    size_t i;
    for (i=0; i < delegateNames.size(); i++) {
      //AnalysisEngineDescription * delSpec = iv_pTaeSpecifier->extractDelegate(delegateNames[i]);
      AnalysisEngineDescription * delSpec = iv_pTaeSpecifier->getDelegate(delegateNames[i]);
      //BSI: AnnotatorContext * delegateAnc =  new AnnotatorContext(delSpec, this, delegateNames[i]);
      AnnotatorContext * delegateAnc =  new AnnotatorContext(delSpec, this,
                                        delegateNames[i],
                                        iv_pTaeSpecifier->getSofaMappings());
      iv_mapDelegateAncs [delegateNames[i]] = delegateAnc;
    }
  }

  bool AnnotatorContext::isParameterDefined(const icu::UnicodeString & paramName) const {
    AnalysisEngineMetaData const * pMetaData = getTaeSpecifier().getAnalysisEngineMetaData();

    assert(EXISTS(pMetaData));
    return pMetaData->isParameterDefined(paramName, "");
  }

  bool AnnotatorContext::isParameterDefined(const icu::UnicodeString & groupName,
      const icu::UnicodeString & paramName) const {
    AnalysisEngineMetaData const * pMetaData = getTaeSpecifier().getAnalysisEngineMetaData();
    assert(EXISTS(pMetaData));
    return pMetaData->isParameterDefined(groupName, paramName, "");
  }


  const set <icu::UnicodeString> AnnotatorContext::getGroupNamesForParameter(const icu::UnicodeString & paramName) const {
      set <icu::UnicodeString> groupNames;
      AnalysisEngineMetaData const * pMetaData = getTaeSpecifier().getAnalysisEngineMetaData();

      assert(EXISTS(pMetaData));

      if (EXISTS(iv_pParentAnC) && pMetaData->hasGroups()) {
        groupNames = iv_pParentAnC->getGroupNamesForParameter(paramName);
      }
      vector <icu::UnicodeString> localGroups = pMetaData->getGroupNamesForParameter(paramName);
      vector <icu::UnicodeString>::const_iterator ite;
      for (ite = localGroups.begin(); ite != localGroups.end(); ite++) {
        groupNames.insert(*ite);
      }
      return groupNames;
    }

  // These 2 are not inlined so they'll use the uima library's heap which is what
  // the extractValue methods appear to use, even though they are declared inline.

  void AnnotatorContext::release(std::string & value) const {
    value.clear();
  }

  void AnnotatorContext::release(vector<std::string*> & returnValues) const {
    for (size_t i=0; i < returnValues.size();i++) {
      delete returnValues.at(i);
    }
    returnValues.clear();
  }

} // namespace uima
