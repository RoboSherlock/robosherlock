/** \file typesystemdescription.hpp .
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

    \brief Contains classes uima::TypeDescription uima::FeatureDescription and uima::TypeSystemDescription

   Description: Defines the type system used in a Text Analysis Engine

-----------------------------------------------------------------------------

   01/30/2003  Initial creation

-------------------------------------------------------------------------- */
#ifndef UIMA_TYPESYSTEMDESCRIPTION_HPP
#define UIMA_TYPESYSTEMDESCRIPTION_HPP

// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include <uima/pragmas.hpp> //must be first to surpress warnings
#include <uima/err_ids.h>
#include <uima/taemetadata.hpp>
#include <uima/importdescription.hpp>

#include <vector>
#include <map>

namespace uima {

  /**
  * If a type can only contain an enumeration of values, there is an <code>AllowedValue</code> object
  * for each value. The value itself is returned by <code>getName</code>, <code>getDescription</code>
  * returns a description of the value.
  **/
  class UIMA_LINK_IMPORTSPEC AllowedValue :public MetaDataObject {
  public:
    AllowedValue()
        :MetaDataObject(), iv_name(), iv_description() {}

    AllowedValue(const AllowedValue & crOther) {
      iv_name = crOther.getName();
      iv_description = crOther.getDescription();
    }



    TyErrorId setName(const icu::UnicodeString & name) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_name = name;
      return UIMA_ERR_NONE;
    }

    TyErrorId setDescription(const icu::UnicodeString &  description) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_description = description;
      return UIMA_ERR_NONE;
    }

    const icu::UnicodeString & getName() const {
      return iv_name;
    }

    const icu::UnicodeString & getDescription() const {
      return iv_description;
    }

  private:
    icu::UnicodeString iv_name;
    icu::UnicodeString iv_description;
  };

  /**
  * Defines a feature by its name and the range type that its values can assume.
  * <code>getDescription</code> returns a textual description of the feature.
  **/
  class UIMA_LINK_IMPORTSPEC FeatureDescription:public MetaDataObject {

  public:

    FeatureDescription()
        :MetaDataObject(), iv_name(), iv_description(), iv_rangeType(), 
              iv_elementType(), iv_multipleRefsAllowed(false) {}

    FeatureDescription(const FeatureDescription & crOther) {
      iv_name = crOther.getName();
      iv_description = crOther.getDescription();
      iv_rangeType = crOther.getRangeTypeName();
      iv_elementType = crOther.getElementType();
      iv_multipleRefsAllowed =  crOther.isMultipleReferencesAllowed();
    }

    TyErrorId setName(const icu::UnicodeString & featName) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_name = featName;
      return UIMA_ERR_NONE;
    }

    const icu::UnicodeString & getName() const {
      return iv_name;
    }

    TyErrorId setRangeTypeName(const icu::UnicodeString & rangeName) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_rangeType = rangeName;
      return UIMA_ERR_NONE;
    }

    const icu::UnicodeString & getRangeTypeName() const {
      return iv_rangeType;
    }

    TyErrorId setElementType(const icu::UnicodeString & elementName) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_elementType = elementName;
      return UIMA_ERR_NONE;
    }

    const icu::UnicodeString & getElementType() const {
      return iv_elementType;
    }

    TyErrorId setDescription(const icu::UnicodeString & desc) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_description = desc;
      return UIMA_ERR_NONE;
    }

	const icu::UnicodeString & getDescription() const {
      return iv_description;
    }

	TyErrorId setMultipleReferencesAllowed(bool allowed) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_multipleRefsAllowed = allowed;
      return UIMA_ERR_NONE;
    }

	const bool isMultipleReferencesAllowed()  const {
		return iv_multipleRefsAllowed;
	}

  private:
    icu::UnicodeString iv_name;
    icu::UnicodeString iv_description;
    icu::UnicodeString iv_rangeType;
    icu::UnicodeString iv_elementType;
	bool iv_multipleRefsAllowed;
  };

  /**
  * Defines a type by its name and the name of its super type.
  * If a type defines features or can only assume values from a
  * predefined set, the information is accessible via
  * <code>getFeatureDescriptions</code> and <code>getAllowedValues</code>, resp.
  * <code>getDescription</code> returns a textual description of the type.
  **/
  class UIMA_LINK_IMPORTSPEC TypeDescription :public MetaDataObject {

  public:
    typedef std::vector<FeatureDescription *> TyVecpFeatureDescriptions;
    typedef std::vector<AllowedValue *> TyVecpAllowedValues;

    TypeDescription()
        :MetaDataObject(), iv_typeName(), iv_superTypeName(), iv_description(),
        iv_vecpFeatureDescriptions(), iv_vecpAllowedValues() {};

    TypeDescription(const TypeDescription & crOther);

    ~TypeDescription() {
      size_t i;
      for (i=0; i < iv_vecpFeatureDescriptions.size(); i++) {
        delete iv_vecpFeatureDescriptions[i];
      }
      for (i=0; i < iv_vecpAllowedValues.size(); i++) {
        delete iv_vecpAllowedValues[i];
      }
    }

    void commit() {
      size_t i;
      for (i=0; i < iv_vecpFeatureDescriptions.size(); i++) {
        iv_vecpFeatureDescriptions[i]->commit();
      }
      for (i=0; i < iv_vecpAllowedValues.size(); i++) {
        iv_vecpAllowedValues[i]->commit();
      }
      iv_bIsModifiable = false;
    }

    TyErrorId setName(const icu::UnicodeString & name) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_typeName = name;
      return UIMA_ERR_NONE;
    }

    const icu::UnicodeString & getName() const {
      return iv_typeName;
    }

    TyErrorId setSuperTypeName(const icu::UnicodeString & name) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_superTypeName = name;
      return UIMA_ERR_NONE;
    }

    const icu::UnicodeString & getSuperTypeName() const {
      return iv_superTypeName;
    }

    TyErrorId setDescription(const icu::UnicodeString & desc) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_description = desc;
      return UIMA_ERR_NONE;
    }

    const icu::UnicodeString & getDescription() const {
      return iv_description;
    }

    /**
    * NOTE: This object will assume memory ownership of <code>feature</code>,
    * i.e. it will delete it when destroyed.
    **/
    TyErrorId addFeatureDescription(FeatureDescription * feature, bool & takesMemoryOwnership);

    /**
     * Adds an AllowedValue object to the list of AllowedValues
     * NOTE: This object will assume memory ownership of <code>AllowedValue</code>,
        * i.e. it will delete it when destroyed.
        */
    TyErrorId addAllowedValue(AllowedValue * allowed, bool & takesMemoryOwnership);

    /**
    * Adds all feature descriptions in descs to this TypeDescription.
    * If there are two features with the same name and rangetype, only
    * one will be in this TypeDescription.
    * However, if two features have the same name, but different rangetypes,
    * the TypeDescription will contain both.
    * The error will be identified when validating the TextAnalysisEngineSpecifier.
    * <code>FeatureDescription</code> objects contained in <code>descs</code> will
    * be copied, hence, the memory ownership of the objects in <code>descs</code>
    * will remain with the caller of this method.
    * 
    **/
    void mergeFeatureDescriptions(const TyVecpFeatureDescriptions & descs);

    void mergeAllowedValues(const TyVecpAllowedValues & descs);

    const TyVecpFeatureDescriptions & getFeatureDescriptions() const {
      return iv_vecpFeatureDescriptions;
    }

    const TyVecpAllowedValues & getAllowedValues() const {
      return iv_vecpAllowedValues;
    }

    /**
    * returns a <code>FeatureDescription</code> with name <code>featureName</code>
    * or NULL, if no such feature can be found.
    **/
    FeatureDescription * getFeatureDescription(const icu::UnicodeString & featureName) const;

    AllowedValue * getAllowedValue(const icu::UnicodeString & name) const;

	void validate();
  private:

    TypeDescription & operator=(const TypeDescription & crOther);

    icu::UnicodeString iv_typeName;
    icu::UnicodeString iv_superTypeName;
    icu::UnicodeString iv_description;
    TyVecpFeatureDescriptions iv_vecpFeatureDescriptions;
    TyVecpAllowedValues iv_vecpAllowedValues;
  };

  /**
  * Contains an ordered list of type names. The ordering implies priority,
  * that is, if entry b comes after entry a in the list, then a as a higher
  * priority than b.
  **/
  class UIMA_LINK_IMPORTSPEC TypePriority :public MetaDataObject {
  public:
    TypePriority() :iv_vecTypeOrder() {}

    TyErrorId addType(const icu::UnicodeString & type) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_vecTypeOrder.push_back(type);
      return UIMA_ERR_NONE;
    }

    const std::vector <icu::UnicodeString> & getTypeOrder() const {
      return iv_vecTypeOrder;
    }

  private:
    std::vector <icu::UnicodeString> iv_vecTypeOrder;
  };

  /**
  * Contains all <code>TypeDescription</code> and <code>TypePriority</code> objects 
  * for a <code>TextAnalysisEngine</code>.
  **/
  class UIMA_LINK_IMPORTSPEC TypeSystemDescription :public MetaDataObject {
  public:
    typedef std::vector<TypeDescription *> TyVecpTypeDescriptions;
    typedef std::vector<ImportDescription *> TyVecpImportDescriptions;


    TypeSystemDescription()
        :MetaDataObject(), iv_vecpTypeDescriptions() {};

    ~TypeSystemDescription() {
      size_t i;
      for (i=0; i < iv_vecpTypeDescriptions.size(); i++) {
        delete iv_vecpTypeDescriptions[i];
      }
      for (i=0; i < iv_vecpImportDescriptions.size(); i++) {
        delete iv_vecpImportDescriptions[i];
      }
    }

    void commit() {
      iv_bIsModifiable = false;

      std::vector<icu::UnicodeString> alreadyImportedLocations;
      resolveImports(alreadyImportedLocations);
      size_t i;
      for (i=0; i < iv_vecpTypeDescriptions.size(); i++) {
        iv_vecpTypeDescriptions[i]->commit();
      }
    }

    /**
    * Adds all type descriptions in descs to this type system description.
    * If there are two type descriptions with the same name and same supertype,
    * their features are merged into one TypeDescription.
    * If two descriptions have the same name, but different supertype names
    * (which is not allowed), the TypeSystemDescription will contain both entries.
    * The error will be identified when validating the TextAnalysisEngineSpecifier
    * <code>TypeDescription</code> objects contained in <code>descs</code> will
    * be copied, hence, the memory ownership of the objects in <code>descs</code>
    * will remain with the caller of this method.
    * @see validate()
    **/
    void mergeTypeDescriptions(const TyVecpTypeDescriptions & descs);

    /**
    * NOTE: This object will assume memory ownership of <code>desc</code>,
    * i.e. it will delete it when destroyed !
    **/
    TyErrorId addTypeDescription(TypeDescription * desc, bool & takesMemoryOwnerShip);

    const TyVecpTypeDescriptions & getTypeDescriptions() const {
      return iv_vecpTypeDescriptions;
    }

    TypeDescription * getTypeDescription(const icu::UnicodeString & typeName) const;

    const TypeDescription * getTypeDescriptionConst(const icu::UnicodeString & typeName) const {
      return getTypeDescription(typeName);
    }

    bool hasTypeDescription(const icu::UnicodeString & typeName) const {
      return(EXISTS(getTypeDescriptionConst(typeName)));
    }

    //Import
    void setXmlFileLocation(const icu::UnicodeString xmlLoc) {
      iv_xmlFileLoc = xmlLoc;
    }

    /**
    * @return The full path name of the XML file used to build up this Specifier.
    * If it was built from a memory buffer,
    * returns some buffer ID generated by Xerces
    */
    const icu::UnicodeString & getXmlFileLocation() const {
      return iv_xmlFileLoc;
    }

    TyErrorId addImportDescription(ImportDescription * pDesc, bool & takesMemoryOwnerShip );

    const TyVecpImportDescriptions & getImportDescriptions() const {
      return iv_vecpImportDescriptions;
    }

    TyErrorId resolveImports(std::vector<icu::UnicodeString> & alreadyImportedTypeSystemLocations);

    TyErrorId resolveImport(const icu::UnicodeString & fileLocation,
                            std::vector<icu::UnicodeString> & alreadyImportedTypeSystemLocations,
                            TyVecpTypeDescriptions & result);
	void validate();
  private:
    TypeSystemDescription(const TypeSystemDescription & crOther);
    TypeSystemDescription operator=(const TypeSystemDescription & crOther);

    TyVecpTypeDescriptions iv_vecpTypeDescriptions;
    TyVecpImportDescriptions iv_vecpImportDescriptions;
    icu::UnicodeString iv_xmlFileLoc;
  };



}
#endif



