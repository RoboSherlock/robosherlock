#ifndef UIMA_TAESPECIFIERBUILDER_HPP
#define UIMA_TAESPECIFIERBUILDER_HPP
/** \file taespecifierbuilder.hpp .
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

    \brief Contains uima::TextAnalysisEngineSpecifierBuilder

   Description: Builds a AnalysisEngineDescription from an XML file or memory buffer.

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include "xercesc/dom/DOMNode.hpp"
#include "xercesc/dom/DOMElement.hpp"
#include "xercesc/sax/ErrorHandler.hpp"
#include "xercesc/sax/InputSource.hpp"

#include <uima/pragmas.hpp>
#include <uima/exceptions.hpp>
#include <uima/taespecifier.hpp>
#include <uima/config_param.hpp>
#include <uima/capability.hpp>
#include <uima/typesystemdescription.hpp>
#include <uima/resmgr.hpp>
/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
XERCES_CPP_NAMESPACE_USE

namespace uima {
  class XIncludeRemoverHandler;
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {

  UIMA_EXC_CLASSDECLARE(InvalidXMLException, uima::Exception);
  
  /**
   * A UIMA <code>XMLParser</code> parses XML documents and generates UIMA component 
   * descriptor object represented in the XML.
   */
  class UIMA_LINK_IMPORTSPEC XMLParser {
 
  public:

    XMLParser();
    ~XMLParser();

    void setErrorHandler(ErrorHandler*);

	/**
     * Enables or disables XML scheam validation.
     * 
     * @param aEnable
     *          true to enable validation, false to disable validation
     */
    //void enableSchemaValidation(bool aEnable);

	/**
	 * Parses an AnalysisEngineDescription from an XML file.
     * 
     * @param aeDesc
     *          reference to <code>AnalysisEngineDescription</code> object 
	 * @param   fileName
     *          char buffer file containing the XML document to be parsed.
     * 
     * @throws InvalidXMLException
     *     if the input XML is not valid or does not specify a valid AnalysisEngineDescription
	 */
	void parseAnalysisEngineDescription(AnalysisEngineDescription & aeDesc,
										char const * fileName);
    /**
	 * Parses an AnalysisEngineDescription from an XML file.
     * 
     * @param aeDesc
     *          reference to <code>AnalysisEngineDescription</code> object
     * @param   fileName 
	 *          UnicodeStrin object containing the file name of the file cotaining the XML document.
     * 
     * @throws InvalidXMLException
     *     if the input XML is not valid or does not specify a valid AnalysisEngineDescription
	 */
	void parseAnalysisEngineDescription(AnalysisEngineDescription & aeDesc,
										icu::UnicodeString const & fileName);

	/**
	 * Parses an AnalysisEngineDescription from an XML input stream.
     * 
     * @param aeDesc
	 *          reference to <code>AnalysisEngineDescription</code> object.
	 * @param crInputSource
     *          the input source from which to read the XML document.
     * 
     * 
     * @throws InvalidXMLException
     *     if the input XML is not valid or does not specify a valid AnalysisEngineDescription
	 */
    void parseAnalysisEngineDescription(AnalysisEngineDescription & aeDesc, 
										InputSource const & crInputSource);
    /**
	 * Parses an TypeSystemDescription from an XML input stream.
     * 
	 * @param aeDesc
	 *          reference to <code>TypeSystemDescription</code> object.
	 * @param crInputSource
     *          the input source from which to read the XML document.
     * 
     * @throws InvalidXMLException
     *     if the input XML is not valid or does not specify a valid AnalysisEngineDescription
	 */
	void parseTypeSystemDescription(TypeSystemDescription & aeDesc, 
											InputSource const & crInputSource);
    /**
	 * Parses an FSIndexDescriptions from an XML input stream.
	 * Caller assumes ownership of objects in the vector.
     * 
	 * @param fsDesc
	 *          reference to <code>TyVecpFSIndexDescriptions</code> object.
	 * @param crInputSource
     *          the input source from which to read the XML document.
     *
     * 
     * @throws InvalidXMLException
     *     if the input XML is not valid or does not specify a valid AnalysisEngineDescription
	 */
    void parseFSIndexDescription(AnalysisEngineMetaData::TyVecpFSIndexDescriptions  & fsDesc,
											InputSource const &  crInputSource);
    /**
	 * Parses an TypePriorities from an XML input stream.
     * Caller assumes ownership of objects in the vector.
	 *
     * @param prioDesc
	 *          reference to <code>TyVecpTypePriorities</code> object.
	 * @param crInputSource
     *          the input source from which to read the XML document.
	 *
     * @throws InvalidXMLException
     *     if the input XML is not valid or does not specify a valid AnalysisEngineDescription
	 */
    void parseTypePriorities(AnalysisEngineMetaData::TyVecpTypePriorities  & prioDesc,
											InputSource const & crInputSource);
    /**
	 * Parses an SofaMappings from an XML input stream.
     * Caller assumes ownership of objects in the vector.
     * @param sofaMapDesc
	 *          reference to <code>TyVecpTypePriorities</code> object.
	 * @param crInputSource
     *          the input source from which to read the XML document.
     * 
     * @throws InvalidXMLException
     *     if the input XML is not valid or does not specify a valid AnalysisEngineDescription
	 */
    void parseSofaMappings(AnalysisEngineDescription::TyVecpSofaMappings  & sofaMapDesc,
											InputSource const & crInputSource);

    OperationalProperties * buildOperationalProperties(DOMElement * descElem);
    XMLCh const * convert(char const * cpBuf) const;
    icu::UnicodeString convert( XMLCh const * cpUCBuf ) const;
  private:
    
    AnalysisEngineMetaData * buildAEMetaData(DOMElement * specElem, icu::UnicodeString const &);
    
    ConfigurationParameter * buildConfigParam(DOMElement * specElem);
    NameValuePair * buildNameValuePair(DOMElement * specElem);
    void buildValue(NameValuePair & nvPair, DOMElement * specElem);
    ConfigurationParameter::EnParameterType findTypeForValue(DOMNode * descElem);

	void buildTypeSystemSpecifier(TypeSystemDescription & tsDesc, DOMElement* specElem,const icu::UnicodeString & xmlFileLoc);
    void buildTypeSystemDesc(TypeSystemDescription & tsDesc, DOMElement * specElem);
    TypeDescription * buildTypeDesc(DOMElement * specElem);
    FeatureDescription * buildFeatureDesc(DOMElement * specElem);
    AllowedValue * buildAllowedValue(DOMElement * specElem);
    ImportDescription * buildImportDesc(DOMElement * specElem);
    TypePriority * buildTypePriority(DOMElement * specElem);
	void buildTypePriorityFromImportLocation(AnalysisEngineMetaData  & aeDesc,
        icu::UnicodeString const & fileName,
        icu::UnicodeString const & lastFileName,
						 std::vector<icu::UnicodeString> & alreadyImported);
	void buildFSIndexFromImportLocation(AnalysisEngineMetaData  &aeMetaData,
                                        icu::UnicodeString const &  importfn,
					    std::vector<icu::UnicodeString> & alreadyImported,
                                        icu::UnicodeString const &  lastFileName);
    void buildFSIndexCollection(AnalysisEngineMetaData & aeMetaData,
                                DOMElement * specElem,
                                std::vector<icu::UnicodeString> & alreadyImported,
                                icu::UnicodeString const &  lastFileName);
    FSIndexDescription * buildFSIndexDesc(DOMElement * specElem);
    FSIndexKeyDescription * buildFSIndexKeyDesc(DOMElement * specElem);

    void buildCapabilities(AnalysisEngineMetaData & aeMetaData, DOMElement * specElem);
    void buildCapability(AnalysisEngineMetaData & aeMetaData, DOMElement * specElem);

    FixedFlow * buildFixedFlow(DOMElement * specElem);
    CapabilityLanguageFlow * buildCapabilityLanguageFlow(DOMElement * specElem);
    FlowConstraints* buildFlowConstraints(DOMElement * specElem);
    SofaMapping * buildSofaMapping(DOMElement * specElem);
    icu::UnicodeString getSpannedText(DOMNode * node);
    DOMNode * findFirst(DOMNodeList * nodes, char const * tagName);
    DOMElement * findFirstElementNode(DOMNodeList * nodes);
    bool isTrue(const icu::UnicodeString & value) const;
    bool isDefined(const XMLCh * attrVal) const;

  protected:
    /* protected in order to allow access by the subclass 
	 * TextAnalysisEngineSpecifierBuilder.
     */
    void buildAnalysisEngineDescription(AnalysisEngineDescription & taeSpec, DOMElement * specElem,
                  const icu::UnicodeString & xmlFileLoc);
    void buildAnalysisEngineDescription(AnalysisEngineDescription & taeSpec,
                                        DOMElement * descElem,
                                        const icu::UnicodeString & xmlFileLoc,
                                        bool changeOrder);
	void buildTypePriorities(AnalysisEngineMetaData::TyVecpTypePriorities & typePriorities, DOMElement * specElem);
    
	void buildTypePriorities(AnalysisEngineMetaData & aeMetaData,
                                DOMElement * specElem,
								icu::UnicodeString const & lastFileName,
				 std::vector<icu::UnicodeString> & alreadyImported);
   
	void buildFSIndexes(AnalysisEngineMetaData & aeDesc, DOMElement * specElem);
	void buildFSIndexes(AnalysisEngineMetaData::TyVecpFSIndexDescriptions & desc, DOMElement * specElem);

    void buildSofaMappings(AnalysisEngineDescription & taeSpec, DOMElement * specElem);
	void buildSofaMappings(AnalysisEngineDescription::TyVecpSofaMappings & sofaMappings, DOMElement * specElem);
    
	void buildConfigParams(AnalysisEngineMetaData & aeMetaData, DOMElement * specElem);
    void buildConfigParamSettings(AnalysisEngineMetaData & aeMetaData, DOMElement * specElem);

  private:
    ErrorHandler * iv_pXMLErrorHandler;
	bool doSchemaValidation;
	TCHAR * schemaFileName;
   
    static char const * TAG_AE_DESC;
    static char const * TAG_TAE_DESC;
    static char const * TAG_CAS_DESC;
    static char const * TAG_RESMGR_CONFIG_DESC;
    static char const * TAG_CASCONSUMER_DESC;
    static char const * TAG_PROCESSING_RESOURCE_METADATA;
    static char const * TAG_IMPL_NAME;
    static char const * TAG_TAE_PRIMITIVE;
    static char const * TAG_DELEGATE_AES;
    static char const * TAG_DELEGATE_AE;
    static char const * TAG_DELEGATE_AE_INCLUDE;
    static char const * UIMA_FRAMEWORK_IMP;
    static char const * TAG_AN_IMPL_NAME;
    static char const * TAG_EXTERNAL_RESOURCE_DEPENDENCIES;
    static char const * TAG_EXTERNAL_RESOURCES;
    static char const * TAG_AE_METADATA;
    static char const * TAG_AE_NAME;
    static char const * TAG_AE_DESCRIPTION;
    static char const * TAG_AE_VERSION;
    static char const * TAG_AE_VENDOR;
    static char const * TAG_CONFIG_PARAMS;
    static char const * TAG_CONFIG_PARAM_GROUP;
    static char const * TAG_CONFIG_PARAM_COMMON;
    static char const * TAG_CONFIG_PARAM;
    static char const * TAG_CONFIG_PARAM_NAME;
    static char const * TAG_CONFIG_PARAM_DESC;
    static char const * TAG_CONFIG_PARAM_TYPE;
    static char const * TAG_CONFIG_PARAM_MULTIVAL;
    static char const * TAG_CONFIG_PARAM_MANDATORY;
    static char const * TAG_CONFIG_PARAM_RESTRICT;
    static char const * TAG_CONFIG_PARAM_SETTINGS;
    static char const * TAG_CONFIG_PARAM_SETTING_GROUP;
    static char const * TAG_NAME_VALUE_PAIR;
    static char const * TAG_NAME_VALUE_PAIR_NAME;
    static char const * TAG_NAME_VALUE_PAIR_VALUE;
    static char const * TAG_NAME_VALUE_PAIR_VALUE_STRING;
    static char const * TAG_NAME_VALUE_PAIR_VALUE_INT;
    static char const * TAG_NAME_VALUE_PAIR_VALUE_FLOAT;
    static char const * TAG_NAME_VALUE_PAIR_VALUE_BOOL;
    static char const * TAG_NAME_VALUE_PAIR_VALUE_ARRAY;
    static char const * TAG_ENV_VAR_REF;
    static char const * TAG_IMPORTS;
    static char const * TAG_IMPORT_DESC;
    static char const * ATTR_IMPORT_DESC_NAME;
    static char const * ATTR_IMPORT_DESC_LOCATION;
    static char const * TAG_TYPE_SYSTEM_DESC;
    static char const * TAG_TYPES;
    static char const * TAG_TYPE_DESC;
    static char const * TAG_TYPE_DESC_NAME;
    static char const * TAG_TYPE_DESC_SUPER;
    static char const * TAG_TYPE_DESC_DESC;
    static char const * TAG_TYPE_DESC_ALLOWED_VALS;
    static char const * TAG_TYPE_DESC_ALLOWED_VALS_VAL;
    static char const * TAG_TYPE_DESC_ALLOWED_VALS_VAL_STRING;
    static char const * TAG_TYPE_DESC_ALLOWED_VALS_VAL_DESC;
    static char const * TAG_TYPE_DESC_FEATURES;
    static char const * TAG_TYPE_DESC_FEAT_DESC;
    static char const * TAG_TYPE_DESC_FEAT_DESC_NAME;
    static char const * TAG_TYPE_DESC_FEAT_DESC_RANGE;
    static char const * TAG_TYPE_DESC_FEAT_DESC_ELEMENT;
    static char const * TAG_TYPE_DESC_FEAT_DESC_DESC;
	static char const * TAG_TYPE_DESC_FEAT_DESC_MULTREFS;
    static char const * TAG_TYPE_PRIORITIES;
    static char const * TAG_TYPE_PRIORITY_LIST;
    static char const * TAG_TYPE_PRIORITY_LIST_TYPE;
    static char const * TAG_FS_INDEXES;
    static char const * TAG_FS_INDEX_COLLECTION;
    static char const * TAG_FS_INDEX_DESC;
    static char const * TAG_FS_INDEX_DESC_LABEL;
    static char const * TAG_FS_INDEX_DESC_TYPE;
    static char const * TAG_FS_INDEX_DESC_KIND;
    static char const * TAG_FS_INDEX_DESC_KEYS;
    static char const * TAG_FS_INDEX_KEY;
    static char const * TAG_FS_INDEX_KEY_FEAT;
    static char const * TAG_FS_INDEX_KEY_COMP;
    static char const * TAG_FS_INDEX_KEY_TYPE_PRIORITY;
    static char const * TAG_CAPABILITIES;
    static char const * TAG_CAPABILITY;
    static char const * TAG_CAP_INPUTS;
    static char const * TAG_CAP_OUTPUTS;
    static char const * TAG_CAP_TYPE;
    static char const * TAG_CAP_FEATURE;
    static char const * TAG_CAP_INPUT_SOFAS;
    static char const * TAG_CAP_OUTPUT_SOFAS;
    static char const * TAG_CAP_SOFA_NAME;
    static char const * TAG_SOFA_MAPPINGS;
    static char const * TAG_SOFA_MAPPING;
    static char const * TAG_SOFAMAP_COMPONENT_KEY;
    static char const * TAG_SOFAMAP_COMPONENT_SOFA_NAME;
    static char const * TAG_SOFAMAP_AGGREGATE_SOFA_NAME;
    static char const * TAG_CAP_LANG_SUPP;
    static char const * TAG_CAP_LANG;
    static char const * TAG_FLOW;
    static char const * TAG_FLOW_FIX;
    static char const * TAG_FLOW_FIX_NODE;
    static char const * TAG_FLOW_CAPABILITY_LANG;
    static char const * TAG_FLOW_CAPABILITY_LANG_NODE;
    static char const * ATTR_DELEGATE_AE_KEY;
    static char const * ATTR_DELEGATE_AE_INCLUDE_FILE;
    static char const * ATTR_CONFIG_PARAMS_DEF_GROUP;
    static char const * ATTR_CONFIG_PARAMS_SEARCH;
    static char const * ATTR_CONFIG_PARAM_GROUP_NAMES;
    static char const * ATTR_CONFIG_PARAM_SETTING_GROUP_NAMES;
    static char const * ATTR_CAP_FEATURE_ALL;
    static char const * TRUE_VALUE;
    static char const * FRAMEWORK_IMP_CPLUSPLUS;
    static char const * FRAMEWORK_IMP_JAVA;
    static char const * NO_FALLBACK;
    static char const * DEFAULT_FALLBACK;
    static char const * LANGUAGE_FALLBACK;
    static char const * CONFIG_PARAM_STRING_TYPE;
    static char const * CONFIG_PARAM_INTEGER_TYPE;
    static char const * CONFIG_PARAM_FLOAT_TYPE;
    static char const * CONFIG_PARAM_BOOLEAN_TYPE;
    static char const * FS_INDEX_KEY_COMP_STANDARD;
    static char const * FS_INDEX_KEY_COMP_REVERSE;
    static char const * FS_INDEX_KEY_KIND_SORTED;
    static char const * FS_INDEX_KEY_KIND_BAG;
    static char const * FS_INDEX_KEY_KIND_SET;

    static char const * TAG_OPERATIONAL_PROPERTIES;
    static char const * TAG_MODIFIES_CAS;
    static char const * TAG_MULTIPLE_DEPLOY_ALLOWED;
    static char const * TAG_OUTPUTS_NEW_CASES;

	

  };


  //-------------------------------------------------
  // @deprecated
  // TextAnalysisEngineSpecifierBuilder
  //-------------------------------------------------
  class UIMA_LINK_IMPORTSPEC TextAnalysisEngineSpecifierBuilder : public XMLParser {
  
  public:
    /**@deprecated*/
    TextAnalysisEngineSpecifierBuilder();
	 /**@deprecated*/
    ~TextAnalysisEngineSpecifierBuilder();
	 /**@deprecated*/
    void buildTae(AnalysisEngineDescription & taeSpec, DOMElement * specElem,
                  const icu::UnicodeString & xmlFileLoc);
     /**@deprecated*/
    void buildTaeFromFile(AnalysisEngineDescription & taeSpec, icu::UnicodeString const & fileName);
     /**@deprecated*/ 
	void buildTaeFromFile(AnalysisEngineDescription & taeSpec, char const * fileName);
	 /**@deprecated*/
	void buildTaeFromMemory(AnalysisEngineDescription & taeSpec, icu::UnicodeString const & xmlString);
     /**@deprecated*/
	void buildTaeFromMemory(AnalysisEngineDescription & taeSpec, char const * cpszXMLString);
     /**@deprecated*/
    TypeSystemDescription * buildTypeSystemSpecifierFromFile(char const * filename);
     /**@deprecated*/
	TypeSystemDescription * buildTypeSystemSpecifierFromFile(icu::UnicodeString const & fileName);
     /**@deprecated*/
	TypeSystemDescription * buildTypeSystemSpecifierFromMemory(icu::UnicodeString const & xmlString);
     /**@deprecated*/
	TypeSystemDescription * buildTypeSystemSpecifierFromXMLBuffer(char const * xmlString);
	 /**@deprecated*/
    void buildConfigParams(AnalysisEngineMetaData & aeDesc, DOMElement * specElem);
     /**@deprecated*/
    void buildConfigParamSettings(AnalysisEngineMetaData & aeDesc, DOMElement * specElem);
     /**@deprecated*/
    void buildTypePriorities(AnalysisEngineMetaData & aeDesc, DOMElement * specElem, icu::UnicodeString const & xmlFileLoc, std::vector<icu::UnicodeString> & alreadyImported);
     /**@deprecated*/
    void buildTypePriorities(AnalysisEngineMetaData::TyVecpTypePriorities & typePriorityDesc, DOMElement * specElem);
	 /**@deprecated*/
    void buildFSIndexes(AnalysisEngineMetaData & aeDesc, DOMElement * specElem);
     /**@deprecated*/
    void buildFSIndexes(AnalysisEngineMetaData::TyVecpFSIndexDescriptions & vecFSIndexDescs,
                        DOMElement * descElem);
     /**@deprecated*/
    void buildSofaMappings(AnalysisEngineDescription::TyVecpSofaMappings& sofaMapDesc,
                           DOMElement * specElem);
     /**@deprecated*/
    void appendToXMLBuffer(AnalysisEngineMetaData::TyVecpFSIndexDescriptions const & fsDesc,
                           icu::UnicodeString &  xmlString);
     /**@deprecated*/
    void appendToXMLBuffer(AnalysisEngineMetaData::TyVecpTypePriorities const & prioDesc,
                           icu::UnicodeString & xmlString);
     /**@deprecated*/
    void appendToXMLBuffer(AnalysisEngineDescription::TyVecpSofaMappings const & sofaMapDesc,
                           icu::UnicodeString & xmlString);

    //xmlString must have been created by calling one of the appendToXMLBuffer methods above
    //and inserting the xml header at the beginning.
    //Caller assumes memory ownership of objects in the vectors
	/**@deprecated*/
    void buildFromXMLBuffer(AnalysisEngineMetaData::TyVecpFSIndexDescriptions  & fsDesc,
                            icu::UnicodeString const &  xmlString);
    /**@deprecated*/
    void buildFromXMLBuffer(AnalysisEngineMetaData::TyVecpTypePriorities  & prioDesc,
                            icu::UnicodeString const & xmlString);
    /**@deprecated*/
    void buildFromXMLBuffer(AnalysisEngineDescription::TyVecpSofaMappings  & sofaMapDesc,
                            icu::UnicodeString const & xmlString);

};


}

#endif



