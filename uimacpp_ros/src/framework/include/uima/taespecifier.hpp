/** \file taespecifier.hpp .
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

    \brief Contains uima::FSIndexKeyDescription, uima::FSIndexDescription, uima::AnalysisEngineMetaData uima::AnalysisEngineDescription etc.

   Description: Resource Specifier Classes for Text Analysis Engines

-----------------------------------------------------------------------------


   01/30/2003  Initial creation

-------------------------------------------------------------------------- */
#ifndef UIMA_TAESPECIFIER_HPP
#define UIMA_TAESPECIFIER_HPP
// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include <uima/pragmas.hpp> //must be first to surpress warnings
#include <uima/err_ids.h>

#include <uima/taemetadata.hpp>
#include <uima/typesystemdescription.hpp>
#include <uima/capability.hpp>
#include <uima/config_param.hpp>

#include <uima/sofamapping.hpp>

#include <list>
#include <map>

namespace uima {
  class ConfigurationDescription;

  UIMA_EXC_CLASSDECLARE(ConfigParamLookupException, ConfigException);


  /**
  * Specifies operational properties of a UIMA component.
  **/
  class  UIMA_LINK_IMPORTSPEC OperationalProperties : public MetaDataObject {
  public:

    OperationalProperties()
        :iv_modifiesCas(false), iv_multipleDeploymentAllowed(false), 
                iv_outputsNewCASes(false) {}
    bool getModifiesCas() const {
        return iv_modifiesCas;
    }
    bool isMultipleDeploymentAllowed() const {
      return iv_multipleDeploymentAllowed;
    }
    bool getOutputsNewCASes() const {
      return iv_outputsNewCASes;
    }

    void setModifiesCas(bool val) {
      iv_modifiesCas = val;
    }
    void setMultipleDeploymentAllowed(bool val) {
      iv_multipleDeploymentAllowed = val;
    }
    void setOutputsNewCASes(bool val) {
      iv_outputsNewCASes = val;
    }
  private:
    bool iv_modifiesCas;
    bool iv_multipleDeploymentAllowed;
    bool iv_outputsNewCASes;
  };



  /**
  * Contains the description of an index key, i.e. the feature name
  * and the comparator that should be used when comparing the feature
  **/
  class UIMA_LINK_IMPORTSPEC FSIndexKeyDescription : public MetaDataObject {
  public:
    FSIndexKeyDescription()
        :iv_featureName(), iv_comparator(STANDARD), iv_bIsTypePriority(false) {}

    enum EnComparatorType {
      STANDARD, REVERSE
    };

    TyErrorId setFeatureName(const icu::UnicodeString & feature) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_featureName = feature;
      return UIMA_ERR_NONE;
    }

    const icu::UnicodeString & getFeatureName() const {
      return(iv_featureName);
    }

    TyErrorId setComparator(EnComparatorType comp) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_comparator = comp;
      return UIMA_ERR_NONE;
    }

    EnComparatorType getComparator() const {
      return(iv_comparator);
    }

    TyErrorId setIsTypePriority() {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_bIsTypePriority = true;
      return UIMA_ERR_NONE;
    }

    bool isTypePriority() const {
      return iv_bIsTypePriority;
    }

  private:
    icu::UnicodeString iv_featureName;
    EnComparatorType iv_comparator;
    bool iv_bIsTypePriority;
  };

  /**
  * Defines an index for a specific type. Is identified
  * by its unique label. Contains a vector of index keys that
  * define the features of the type that should be used in the index
  * @see FSIndexKeyDescription
  **/
  class  UIMA_LINK_IMPORTSPEC FSIndexDescription : public MetaDataObject {
  public:
    typedef std::vector < FSIndexKeyDescription * > TyVecpFSIndexKeys;
    enum EnIndexKind {
      SORTED, BAG, SET
    };

    FSIndexDescription()
        :MetaDataObject(), iv_label(), iv_typeName(), iv_kind(SORTED), iv_keys() {}

    ~FSIndexDescription() {
      size_t i;
      for (i=0; i < iv_keys.size(); i++) {
        delete iv_keys[i];
      }
    }

    TyErrorId setLabel(const icu::UnicodeString & name) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_label = name;
      return UIMA_ERR_NONE;
    }

    const icu::UnicodeString & getLabel() const {
      return(iv_label);
    }

    TyErrorId setTypeName(const icu::UnicodeString & name) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_typeName = name;
      return UIMA_ERR_NONE;
    }

    const icu::UnicodeString & getTypeName() const {
      return(iv_typeName);
    }

    TyErrorId setIndexKind(EnIndexKind kind) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_kind = kind;
      return UIMA_ERR_NONE;
    }

    EnIndexKind getIndexKind() const {
      return iv_kind;
    }
    /**
    * NOTE: This object will assume memory ownership of <code>desc</code>,
    * i.e. it will delete it when destroyed!
    **/
    TyErrorId addFSIndexKey(FSIndexKeyDescription * desc) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_keys.push_back(desc);
      return UIMA_ERR_NONE;
    }

    const TyVecpFSIndexKeys & getFSIndexKeys() const {
      return iv_keys;
    }

    void commit() {
      size_t i;
      for (i=0; i < iv_keys.size(); i++) {
        iv_keys[i]->commit();
      }
      iv_bIsModifiable = false;
    }


  private:
    FSIndexDescription(const FSIndexDescription & crOther);

    FSIndexDescription & operator=(FSIndexDescription const & crOther);

    icu::UnicodeString iv_label;
    icu::UnicodeString iv_typeName;
    EnIndexKind iv_kind;
    TyVecpFSIndexKeys iv_keys;
  };



  /**
  * Specifies the order in which Annotators are invoked.
  * The sequence is given by the order in which <code>addNode</code>
  * was called.
  **/
  class  UIMA_LINK_IMPORTSPEC FlowConstraints : public MetaDataObject {
  public:

    enum EnFlowType {
      FIXED, CAPABILITYLANGUAGE
    };


    /**
    * <code>node</code> must uniquely identify an annotator.
    **/
    virtual EnFlowType const & getFlowConstraintsType()=0;
    virtual  std::vector < icu::UnicodeString > const & getNodes() const = 0;
    virtual TyErrorId addNode(const icu::UnicodeString & node)=0;

  };


  /**
  * Specifies the order in which Annotators are invoked.
  * The sequence is given by the order in which <code>addNode</code>
  * was called.
  **/
  class  UIMA_LINK_IMPORTSPEC FixedFlow : public FlowConstraints {
  public:
    FixedFlow()
        :FlowConstraints(), iv_type(FIXED), iv_nodes() {}
    EnFlowType const & getFlowConstraintsType() {
      return iv_type;
    }
    /**
    * <code>node</code> must uniquely identify an annotator.
    **/
    TyErrorId addNode(const icu::UnicodeString & node) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_nodes.push_back(node);
      return UIMA_ERR_NONE;
    }

    std::vector < icu::UnicodeString > const & getNodes() const {
      return(iv_nodes);
    }
  private:
    EnFlowType iv_type;
    std::vector<icu::UnicodeString> iv_nodes;
  };

  class  UIMA_LINK_IMPORTSPEC CapabilityLanguageFlow : public FlowConstraints {
  public:
    CapabilityLanguageFlow()
        :FlowConstraints(),  iv_type(CAPABILITYLANGUAGE), iv_nodes() {}
    EnFlowType const & getFlowConstraintsType() {
      return iv_type;
    }
    /**
    * <code>node</code> must uniquely identify an annotator.
    **/
    TyErrorId addNode(const icu::UnicodeString & node) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_nodes.push_back(node);
      return UIMA_ERR_NONE;
    }

    std::vector < icu::UnicodeString > const & getNodes() const {
      return(iv_nodes);
    }
  private:
    EnFlowType iv_type;
    std::vector<icu::UnicodeString> iv_nodes;
  };







  /**
   * Contains the <code>MetaDataObject</code>s that are defined for both
   * delegate and aggregate <code>TextAnalysisEngine</code>s,
   * for example the description of the types used and the capabilities
   * of the TAE.
   * Normally, applications and annotators don't deal with the
   * <code>AnalysisEngineMetaData</code> directly.
   * They should use the methods on the <code>AnnotatorContext</code>.
   *
   * @see AnalysisEngineDescription
   */
  class  UIMA_LINK_IMPORTSPEC AnalysisEngineMetaData :public MetaDataObject {
  public:
    friend class XMLParser;
    friend class AnalysisEngineDescription;
    friend class AnnotatorContext;

    enum EnSearchStrategy {
      NONE, DEFAULT_FALLBACK, LANGUAGE_FALLBACK
    };

    typedef std::vector <Capability *> TyVecpCapabilities;
    typedef std::vector <FSIndexDescription *> TyVecpFSIndexDescriptions;
    typedef std::vector<TypePriority *> TyVecpTypePriorities;

    typedef std::vector<ImportDescription *> TyVecpFSIndexImportDescriptions;
    typedef std::vector<ImportDescription *> TyVecpTypePriorityImportDescriptions;

    typedef std::map <icu::UnicodeString, ConfigurationGroup> TyConfigGroup;
    typedef std::map <icu::UnicodeString, SettingsForGroup> TyConfigSettings;


    AnalysisEngineMetaData()
        :MetaDataObject(), CONFIG_GROUP_NAME_WHEN_NO_GROUPS("CONFIG_GROUP_NAME_WHEN_NO_GROUPS"),
        CONFIG_GROUP_NAME_COMMON_PARMS("CONFIG_GROUP_NAME_COMMON_PARAMETERS"),
        iv_hasGroups(false), iv_hasDefaultGroup(false), iv_searchStrategy(NONE),
        iv_pTypeSystemDesc(NULL), iv_pFlowConstraints(NULL), iv_vecpTypePriorities(),
        iv_pOperationalProperties(NULL) {}

    ~AnalysisEngineMetaData();

	void validate();
    void commit();

    /**
    * NOTE: This object will assume memory ownership of <code>flowConstraint</code>, i.e.
    * it will be deleted when this object's destructor is called !
    **/
    TyErrorId setFlowConstraints(FlowConstraints * flow) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_pFlowConstraints = flow;
      return UIMA_ERR_NONE;
    }

    FlowConstraints * getFlowConstraints() {
      return(iv_pFlowConstraints);
    }

    FlowConstraints  const * getFlowConstraints() const {
      return(iv_pFlowConstraints);
    }


    FlowConstraints::EnFlowType const & getFlowConstraintsType() {
      return iv_pFlowConstraints->getFlowConstraintsType();
    }


    TyErrorId setName(const icu::UnicodeString & name) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_name = name;
      return UIMA_ERR_NONE;
    }

    const icu::UnicodeString & getName() const {
      return(iv_name);
    }

    TyErrorId setDescription(const icu::UnicodeString & desc) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_description = desc;
      return UIMA_ERR_NONE;
    }

    const icu::UnicodeString & getDescription() const {
      return(iv_description);
    }

    TyErrorId setVersion(const icu::UnicodeString & version) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_version = version;
      return UIMA_ERR_NONE;
    }

    const icu::UnicodeString & getVersion() const {
      return(iv_version);
    }

    TyErrorId setVendor(const icu::UnicodeString & vendor) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_vendor = vendor;
      return UIMA_ERR_NONE;
    }

    const icu::UnicodeString & getVendor() const {
      return(iv_vendor);
    }

    /**
    * Side effect: creates a configuration group <code>groupName</code>,
    * if it doesn't exist already.
    * Reason: otherwise, <code>AnnotatorContext</code>`s <code>assignValue</code>
    * with two parameters will fail if the specifier defines all parameters in the
    * <code>commonParameters</code> section and declares the default group only via
    * the <code>defaultGroup</code> attribute of <code>configurationParameters</code>
    **/
    TyErrorId setDefaultGroupName(const icu::UnicodeString & groupName);

    bool hasDefaultGroup() const {
      return iv_hasDefaultGroup;
    }

    bool hasGroups() const {
      return iv_hasGroups;
    }

    TyErrorId setSearchStrategy(EnSearchStrategy strategy) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_searchStrategy = strategy;
      return UIMA_ERR_NONE;
    }

    EnSearchStrategy getSearchStrategy() const {
      return(iv_searchStrategy);
    }

    /**
    * NOTE: This object will assume memory ownership of <code>desc</code>, i.e.
    * it will be deleted when this object's destructor is called !
    **/
    TyErrorId setTypeSystemDescription(TypeSystemDescription * desc) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_pTypeSystemDesc = desc;
      return UIMA_ERR_NONE;
    }

    TypeSystemDescription * getTypeSystemDescription() {
      return(iv_pTypeSystemDesc);
    }

    TypeSystemDescription const * getTypeSystemDescription() const {
      return(iv_pTypeSystemDesc);
    }

    /**
    * NOTE: This object will assume memory ownership of <code>prio</code>,
    * i.e. it will delete it when destroyed !
    **/
    TyErrorId addTypePriority(TypePriority * prio);

    const TyVecpTypePriorities & getTypePriorities() const {
      return iv_vecpTypePriorities;
    }

    /**
    * @return A group name <code>g</code> for which the following holds true:
    * The behavior of <code>getNameValuePair(param)</code> and
    * <code>getNameValuePair(g, param, getSearchStrategy())</code> is
    * the same.
    * If a default group is defined, returns that group name.
    * If no groups are defined, returns CONFIG_GROUP_NAME_WHEN_NO_GROUPS.
    * If groups are defined, but no default group, throws an exception.
    **/
    const icu::UnicodeString & getGroupNameWhenNotSpec() const;

    /**
    * return a reference to the NameValuePair whose name is paramName
    * iff
    * a) there are no configuration groups defined or there is a default group defined.
    * b) a NameValuePair with the name exists in the configuration settings of this taespecifier
    *
    * Otherwise, return NULL
    **/
    NameValuePair * getNameValuePair(const icu::UnicodeString & paramName,
                                     const icu::UnicodeString & ancKey="");

    /**
    * returns a reference to the NameValuePair whose name is paramName
    * iff
    * a) there are configuration groups defined
    * b) a NameValuePair with the name exists in the configuration settings
    *    of the 'appropriate' group. This group may either be groupName or one of the fallback
    *    groups specified by the searchStrategy and defaultGroup
    *
    * Otherwise, return NULL
    **/
    NameValuePair * getNameValuePair(const icu::UnicodeString & groupName,
                                     const icu::UnicodeString & paramName,
                                     EnSearchStrategy strategy,
                                     const icu::UnicodeString & ancKey="");

    /**
    * TyErrorId == UIMA_ERR_NONE iff
    * a) there are groups: the name of nvPair is a configuration parameter defined
    *    in the common parameters or in the default group
    * b) there are no groups:  the name of nvPair is a configuration parameter
    * If above conditions hold true, nvPair will be added to
    * a) the settings of the default group
    * b) the configuration settings
    **/
    TyErrorId setNameValuePair(const NameValuePair & nvPair) {
      if (hasGroups()) {
        return setNameValuePair(getDefaultGroupName(), nvPair);
      } else {
        return setNameValuePair(CONFIG_GROUP_NAME_WHEN_NO_GROUPS, nvPair);
      }
    }

    /**
    * TyErrorId == UIMA_ERR_NONE iff
    * a) there is a group groupName in the configuration parameters
    * b) the name of nvPair is a configuration parameter defined in groupName
    **/
    TyErrorId setNameValuePair(const icu::UnicodeString & groupName,
                               const NameValuePair & nvPair);


    /**
     * Returns true iff the parameter paramName is defined
     */
    bool isParameterDefined(const icu::UnicodeString & paramName,
                            const icu::UnicodeString & ancKey) const;

    /**
     * returns TRUE iff paramName is either defined for group groupName
     * or in the commonParameter section
     **/
    bool isParameterDefined(const icu::UnicodeString & groupName,
                            const icu::UnicodeString & paramName,
                            const icu::UnicodeString & ancKey) const;

    /**
    * NOTE: This object will assume memory ownership of <code>flow</code>, i.e.
    * it will be deleted when this object's destructor is called !
    **/
    TyErrorId setFixedFlow(FixedFlow * flow) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_pFlowConstraints = flow;
      return UIMA_ERR_NONE;
    }

    FixedFlow * getFixedFlow() {
      if (iv_pFlowConstraints->getFlowConstraintsType() == FlowConstraints::FIXED ) {
        return((FixedFlow*)iv_pFlowConstraints);
      } else return NULL;
    }

    FixedFlow const * getFixedFlow() const {
      if (iv_pFlowConstraints->getFlowConstraintsType() == FlowConstraints::FIXED ) {
        return((FixedFlow*)iv_pFlowConstraints);
      } else return NULL;
    }





    /**
    * NOTE: This object will assume memory ownership of <code>capability</code>, i.e.
    * it will be deleted when this object's destructor is called !
    **/
    TyErrorId addCapability(Capability * capability) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_capabilities.push_back(capability);
      return UIMA_ERR_NONE;
    }

    const TyVecpCapabilities & getCapabilites() const {
      return iv_capabilities;
    }

    void setOperationalProperties(OperationalProperties  * prop) {
      iv_pOperationalProperties = prop;
    }

    const OperationalProperties * getOperationalProperties() const {
       return iv_pOperationalProperties;
    }
    /**
    * NOTE: This object will assume memory ownership of <code>indexDesc</code>, i.e.
    * it will be deleted when this object's destructor is called !
    **/
    TyErrorId addFSIndexDescription(FSIndexDescription * indexDesc) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_indexDescs.push_back(indexDesc);
      return UIMA_ERR_NONE;
    }

    const TyVecpFSIndexDescriptions & getFSIndexDescriptions() const {
      return iv_indexDescs;
    }

    /**
      * NOTE: This object will assume memory ownership of <code>importDescription</code>, i.e.
      * it will be deleted when this object's destructor is called !
      **/
    TyErrorId addFSIndexImportDescription(ImportDescription * importDesc, bool & takesMemoryOwnership) {
      takesMemoryOwnership=false;
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_fsindexImportDescs.push_back(importDesc);
      takesMemoryOwnership=true;
      return UIMA_ERR_NONE;
    }

    const TyVecpFSIndexImportDescriptions & getFSIndexImportDescriptions() const {
      return iv_fsindexImportDescs;
    }



    /**
     * NOTE: This object will assume memory ownership of <code>importDescription</code>, i.e.
     * it will be deleted when this object's destructor is called !
     **/
    TyErrorId addTypePriorityImportDescription(ImportDescription * importDesc,bool & takesMemoryOwnership) {
      takesMemoryOwnership=false;

      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_typepriorityImportDescs.push_back(importDesc);
      takesMemoryOwnership=true;
      return UIMA_ERR_NONE;
    }

    const TyVecpTypePriorityImportDescriptions & getTypePriorityDescriptions() const {
      return iv_typepriorityImportDescs;
    }





    /**
    * @return The names of all groups in this specifier that define <code>paramName</code>.
    * May contain duplicate group names.
    **/
    const std::vector < icu::UnicodeString > getGroupNamesForParameter(const icu::UnicodeString & paramName) const;


  private:
    AnalysisEngineMetaData(AnalysisEngineMetaData const & crOther);

    AnalysisEngineMetaData & operator=(AnalysisEngineMetaData const & crOther);

    const icu::UnicodeString & getDefaultGroupName() const {
      return(iv_defaultGroup);
    }

    /**
    * To be used only when there are no configuration groups defined
    **/
    /*       NameValuePair * getNameValuePairPtr(const icu::UnicodeString & paramName) {  */
    /*          return(getNameValuePairPtr(CONFIG_GROUP_NAME_WHEN_NO_GROUPS, paramName)); */
    /*       }                                                                            */


    /**
    * returns a pointer to the NameValuePair whose name is paramName
    * iff
    * a) there are configuration groups defined
    * b) a NameValuePair with the name exists in the configuration settings
    *    of this group
    * Otherwise, return NULL
    **/
    NameValuePair * getNameValuePairPtr(const icu::UnicodeString & groupName,
                                        const icu::UnicodeString & paramName);

    /**
    * returns a pointer to the NameValuePair whose name is paramName
    * iff
    * a NameValuePair with the name exists in the configuration settings
    *    of this group
    * Throws an exception if a value exists, but <code>paramName</code> is not
    * defined in the configuration parameters.
    * Otherwise, return NULL
    * @see TextAnalysisEngineSpecifierBuilder
    **/
    NameValuePair * getNameValuePairNoFallback(const icu::UnicodeString & groupName,
        const icu::UnicodeString & paramName,
        const icu::UnicodeString & ancKey);
    /**
    * Generates all possible fallback groups for groupName, depending on strategy.
    * Not all of these groups may exist in the configuration settings.
    **/
    void generateFallbackGroups(const icu::UnicodeString & groupName,
                                EnSearchStrategy strategy,
                                std::vector < icu::UnicodeString> & fallbackGroups);

    /**
    * Adds a configuration group named <code>groupName</code>.
    * Throws an exception if the group already exists
    **/
    TyErrorId addConfigurationGroup(const icu::UnicodeString & groupName);

    /**
    * NOTE: This object will assume memory ownership of <code>param</code>, i.e.
    * it will be deleted when this object's destructor is called !
    **/
    TyErrorId addCommonParameter(ConfigurationParameter * param) {
      return addConfigurationParameter(CONFIG_GROUP_NAME_COMMON_PARMS, param);
    }

    /**
    * NOTE: This object will assume memory ownership of <code>param</code>, i.e.
    * it will be deleted when this object's destructor is called !
    **/
    TyErrorId addConfigurationParameter(ConfigurationParameter * param) {
      return addConfigurationParameter(CONFIG_GROUP_NAME_WHEN_NO_GROUPS, param);
    }

    /**
    * NOTE: This object will assume memory ownership of <code>param</code>, i.e.
    * it will be deleted when this object's destructor is called !
    **/
    TyErrorId addConfigurationParameter(const icu::UnicodeString & groupName,
                                        ConfigurationParameter * param);

    /**
    * NOTE: This object will assume memory ownership of <code>nvPair</code>, i.e.
    * it will be deleted when this object's destructor is called !
    + Throws a <code>ConfigParamLookupException</code> if the parameter is not defined .
    **/
    void addNameValuePair(NameValuePair * nvPair) {
      addNameValuePair(CONFIG_GROUP_NAME_WHEN_NO_GROUPS, nvPair);
    }

    /**
    * NOTE: This object will assume memory ownership of <code>nvPair</code>, i.e.
    * it will be deleted when this object's destructor is called !
    + Throws a <code>ConfigParamLookupException</code> if the parameter is not defined
    * for the group.
    **/
    void addNameValuePair(const icu::UnicodeString & groupName,
                          NameValuePair * nvPair);

    /**
    * @return The names of all groups defined in this <code>AnalysisEngineMetaData</code>
    * object. If there are no groups defined, the vector contains only one entry,
    * <code>CONFIG_GROUP_NAME_WHEN_NO_GROUPS</code>. To be used together with
    * <code>getConfigurationParameters</code> to obtain all configuration parameters defined
    * in this <code>AnalysisEngineMetaData</code>.
    * Note: <code>CONFIG_GROUP_NAME_COMMON_PARMS</code> is never contained in the vector.
    * These parameters will be added explicitly in the calls to <code>getConfigurationParameters</code>.
    * 
    **/
    const std::vector <icu::UnicodeString> getConfigurationGroupNames() const;

    /**
    * @return All <code>ConfigurationParameter</code>s defined for <code>groupName</code>.
    * If there are parameters defined in the commonParameters section, they will be returned
    * as well.
    * Returns an empty vector if <code>groupName</code> does not exist.
    **/
    const std::vector <const ConfigurationParameter *> getConfigurationParameters(const icu::UnicodeString & groupName) const;


    //internally, we store configuration parameters and settings always in groups
    //this is the group name we use when there are no groups defined
    icu::UnicodeString CONFIG_GROUP_NAME_WHEN_NO_GROUPS;
    //internally, the common parameters are just another configuration group
    icu::UnicodeString CONFIG_GROUP_NAME_COMMON_PARMS;

    //typedef map <icu::UnicodeString, ConfigurationGroup> TyConfigGroup;
    //typedef map <icu::UnicodeString, SettingsForGroup> TyConfigSettings;
    icu::UnicodeString iv_name;
    icu::UnicodeString iv_description;
    icu::UnicodeString iv_version;
    icu::UnicodeString iv_vendor;
    icu::UnicodeString iv_defaultGroup;
    bool iv_hasGroups;
    bool iv_hasDefaultGroup;
    EnSearchStrategy iv_searchStrategy;
    TyConfigGroup iv_configurationGroups;
    TyConfigSettings iv_configurationSettings;
    TypeSystemDescription * iv_pTypeSystemDesc;
    FlowConstraints * iv_pFlowConstraints;
    TyVecpCapabilities iv_capabilities;
    TyVecpFSIndexDescriptions iv_indexDescs;
    TyVecpTypePriorities iv_vecpTypePriorities;

    TyVecpFSIndexImportDescriptions iv_fsindexImportDescs;
    TyVecpTypePriorityImportDescriptions iv_typepriorityImportDescs;

    OperationalProperties * iv_pOperationalProperties;
  };

  /**
  * Contains all <code>MetaDataObject</code>s for a particular TextAnalysisEngine
  * that are found in a TextAnalysisEngine Resource Specifier XML file.
  * In case of an aggregate TAE, the <code>AnalysisEngineDescription</code> contains
  * all <code>AnalysisEngineDescription</code>s of the delegate TAEs when it is built by
  * the <code>TextAnalysisEngineSpecifierBuilder</code>. However, these delegate specifiers are removed
  * when the <code>AnnotatorContext</code> for the TAE is built.
  * Normally, applications and annotators don't deal with the <code>AnalysisEngineDescription</code> directly.
  * They should use the methods on the <code>AnnotatorContext</code>.
  * @see AnnotatorContext
  **/
  class  UIMA_LINK_IMPORTSPEC AnalysisEngineDescription : public MetaDataObject {
  public:
    friend class XMLParser;
	friend class TextAnalysisEngineSpecifierBuilder; 
    friend class CasDefinition;

    typedef std::map<icu::UnicodeString, AnalysisEngineDescription *> TyMapDelegateSpecs;
    typedef std::vector <SofaMapping *> TyVecpSofaMappings;

    enum EnFrameworkImplName {
      CPLUSPLUS, JAVA
    };

    AnalysisEngineDescription()
        :MetaDataObject(), iv_bPrimitive(false), iv_enFrameImpl(CPLUSPLUS),
        iv_annotatorImpName(), iv_xmlFileLoc(), iv_pAeMetaData(NULL) {
      iv_pMapDelegateSpecifiers = new TyMapDelegateSpecs();
    }


    ~AnalysisEngineDescription() {
      TyMapDelegateSpecs::iterator entries = iv_pMapDelegateSpecifiers->begin();
      while (entries != iv_pMapDelegateSpecifiers->end()) {
        delete (*entries).second;
        entries++;
      }
      delete iv_pMapDelegateSpecifiers;
      delete iv_pAeMetaData;
      //BSI
      size_t i;
      for (i=0; i < iv_SofaMappings.size(); i++) {
        delete   iv_SofaMappings[i];
      }

    }
    void validate();
    void commit();

    /**
    * NOTE: This object will assume memory ownership of <code>metaData</code>, i.e.
    * it will be deleted when this object's destructor is called !
    **/
    TyErrorId setAnalysisEngineMetaData(AnalysisEngineMetaData * metaData) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_pAeMetaData = metaData;
      return UIMA_ERR_NONE;
    }

    AnalysisEngineMetaData * getAnalysisEngineMetaData() {
      return(iv_pAeMetaData);
    }

    AnalysisEngineMetaData const * getAnalysisEngineMetaData() const {
      return(iv_pAeMetaData);
    }

    /**
    * return the search strategy that is applied when looking for configuration parameter values
    **/
    AnalysisEngineMetaData::EnSearchStrategy getSearchStrategy() const {
      return iv_pAeMetaData->getSearchStrategy();
    }

    TyErrorId setPrimitive(bool primitive) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_bPrimitive = primitive;
      return UIMA_ERR_NONE;
    }


    bool isPrimitive() const {
      return iv_bPrimitive;
    }

    EnFrameworkImplName getFrameworkImplName() const {
      return iv_enFrameImpl;
    }


    /**
    * Returns a meaningful value only if isPrimitive() == true
    **/
    const icu::UnicodeString & getAnnotatorImpName() const {
      return iv_annotatorImpName;
    }

    void setXmlFileLocation(const icu::UnicodeString xmlLoc) {
      iv_xmlFileLoc = xmlLoc;
    }

    /**
    * @return The full path name of the XML file used to build up this
    * AnalysisEngineDescription. If it was built from a memory buffer,
    * returns some buffer ID generated by Xerces
    */
    const icu::UnicodeString & getXmlFileLocation() const {
      return iv_xmlFileLoc;
    }

    void setXmlRootTag(const icu::UnicodeString xmlTag) {
      iv_xmlRootTag = xmlTag;
    }

    /**
    * @return The full path name of the XML file used to build up this
    * AnalysisEngineDescription. If it was built from a memory buffer,
    * returns some buffer ID generated by Xerces
    */
    const icu::UnicodeString & getXmlRootTag() const {
      return iv_xmlRootTag;
    }

    /**
    * NOTE: This object will assume memory ownership of <code>taeSpec</code>, i.e.
    * it will be deleted when this object's destructor is called !
    **/
    TyErrorId addDelegate(const icu::UnicodeString & key,
                          AnalysisEngineDescription * taeSpec) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      (*iv_pMapDelegateSpecifiers)[key] = taeSpec;
      return UIMA_ERR_NONE;
    }

    /**
    * Note: This object will still assume memory ownership of the AnalysisEngineDescription
    * that are returned.
    **/
    const TyMapDelegateSpecs & getDelegates() const {
      return *iv_pMapDelegateSpecifiers;
    }



    /** BSI
    * Note: This object will still assume memory ownership of the returned SofAMapping objects
    * that are returned.
    **/
    const TyVecpSofaMappings & getSofaMappings() const {
      return iv_SofaMappings;
    }


    /**  BSI
    * NOTE: This object will assume memory ownership of <code>pSofaMappings</code>, i.e.
    * it will be deleted when this object's destructor is called !
    **/
    TyErrorId setSofaMappings(TyVecpSofaMappings * pSofaMappings) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_SofaMappings = *pSofaMappings;
      return UIMA_ERR_NONE;
    }
    /**
    * NOTE: This object will assume memory ownership of <code>SofaMapping</code>, i.e.
    * it will be deleted when this object's destructor is called !
    **/
    TyErrorId addSofaMapping(SofaMapping * pSofaMapping) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_SofaMappings.push_back(pSofaMapping);
      return UIMA_ERR_NONE;
    }

    /**
    * Note: This object will still assume memory ownership of the AnalysisEngineDescription
    * that is returned.
    * Returns NULL if no delegate can be found for key
    **/
    AnalysisEngineDescription * getDelegate(const icu::UnicodeString & key) const;

    /**
    * Note: This object will delete the AnalysisEngineDescription returned from
    * its map of delegate specifiers. Memory ownership is transferred to the caller
    * of this method !
    * Returns NULL if no delegate can be found for key
    **/
    AnalysisEngineDescription * extractDelegate(const icu::UnicodeString & key);

    /**
    * returns a reference to the NameValuePair whose name is paramName
    * iff
    * a) there are no configuration groups defined or there is a default group defined.
    * b) a NameValuePair with the name exists in the configuration settings of this taespecifier
    *
    * Otherwise, returns NULL.
    **/
    NameValuePair * getNameValuePair(const icu::UnicodeString & paramName,
                                     const icu::UnicodeString & ancKey="") {
      return iv_pAeMetaData->getNameValuePair(paramName, ancKey);
    }

    /**
    * returns a reference to the NameValuePair whose name is paramName
    * iff
    * a) there are configuration groups defined
    * b) a NameValuePair with the name exists in the configuration settings
    *    of the 'appropriate' group. This group may either be groupName or one of the fallback
    *    groups specified by the strategy
    * OR
    * a) groupName == <code>CONFIG_GROUP_NAME_WHEN_NO_GROUPS</code>
    * b) a NameValuePair with the name exists in the configuration settings
    *
    * Throws an exception if a value is found, but the parameter is not defined.
    * Throws an exception if no groups are defined and groupName != <code>CONFIG_GROUP_NAME_WHEN_NO_GROUPS</code>
    *
    * Otherwise, returns NULL.
    **/
    NameValuePair * getNameValuePair(const icu::UnicodeString & groupName,
                                     const icu::UnicodeString & paramName,
                                     AnalysisEngineMetaData::EnSearchStrategy strategy,
                                     const icu::UnicodeString & ancKey="") {
      return iv_pAeMetaData->getNameValuePair(groupName, paramName, strategy, ancKey);
    }

    /**
    * TyErrorId == UIMA_ERR_NONE iff
    * a) there are groups: the name of nvPair is a configuration parameter defined
    *    in the common parameters or in the default group
    * b) there are no groups:  the name of nvPair is a configuration parameter
    * If above conditions hold true, nvPair will be added to
    * a) the settings of the default group
    * b) the configuration settings
    **/
    TyErrorId setNameValuePair(const NameValuePair & nvPair) {
      return iv_pAeMetaData->setNameValuePair(nvPair);
    }

    /**
    * TyErrorId == UIMA_ERR_NONE iff
    * a) there is a group groupName in the configuration parameters
    * b) the name of nvPair is a configuration parameter defined in groupName
    **/
    TyErrorId setNameValuePair(const icu::UnicodeString & groupName,
                               const NameValuePair & nvPair) {
      return iv_pAeMetaData->setNameValuePair(groupName, nvPair);
    }

    /**
     * converts this specifier to an XML buffer.
     */
    void toXMLBuffer(icu::UnicodeString &) const;
	void toXMLBuffer(AnalysisEngineMetaData const & md,  
													   bool isCasConsumer,
													   icu::UnicodeString & s) const;
    void appendConfigParamsAndSettingsToXMLBuffer(icu::UnicodeString & s) const;

    TyErrorId setFrameworkImplName(EnFrameworkImplName impl) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_enFrameImpl = impl;
      return UIMA_ERR_NONE;
    }

    TyErrorId setAnnotatorImpName(const icu::UnicodeString & anName) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_annotatorImpName = anName;
      return UIMA_ERR_NONE;
    }

  protected:
    void appendToXMLBuffer(AnalysisEngineMetaData::TyVecpFSIndexDescriptions const & fsDesc,
                           icu::UnicodeString & s);

    void appendToXMLBuffer(AnalysisEngineMetaData::TyVecpTypePriorities const  & prioDesc,
                           icu::UnicodeString & s);

    void appendToXMLBuffer(TyVecpSofaMappings const & sofaMapDesc,
                           icu::UnicodeString & s);

  private:
    AnalysisEngineDescription(AnalysisEngineDescription const & crOther);
    AnalysisEngineDescription & operator=(AnalysisEngineDescription const & crOther);


    void toXMLBufferNoXMLHeader(icu::UnicodeString & s) const;
    void appendConfigGroupToXMLBuffer(ConfigurationGroup const & config, icu::UnicodeString & s) const;
    void appendConfigGroupSettingsToXMLBuffer(SettingsForGroup const & settings, icu::UnicodeString & s) const;
   
    bool iv_bPrimitive;
    EnFrameworkImplName iv_enFrameImpl;
    icu::UnicodeString iv_annotatorImpName;


    //If the AnalysisEngineDescription was built from an xml file, contains the
    //path to the file on the file system
    icu::UnicodeString iv_xmlFileLoc;
    icu::UnicodeString iv_xmlRootTag;

    //maps the 'key' attribute of a delegateAnalysisEngine to the corresponding specifier
    //had to use pointer here, otherwise it wouldn't compile on MS Visual C++ 6
    TyMapDelegateSpecs * iv_pMapDelegateSpecifiers;
    AnalysisEngineMetaData * iv_pAeMetaData;
    //BSI
    TyVecpSofaMappings iv_SofaMappings;


  };

  class  UIMA_LINK_IMPORTSPEC TextAnalysisEngineSpecifier : public AnalysisEngineDescription {
  public:
    friend class XMLParser;
    friend class CasDefinition;
    TextAnalysisEngineSpecifier()
        :AnalysisEngineDescription() {
    }

  };

}

#endif




