/** \file typesystemdescription.cpp .
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

   02/05/2003  Initial creation

-------------------------------------------------------------------------- */

#include <uima/typesystemdescription.hpp>
#include <uima/taespecifierbuilder.hpp>
#include <memory>
#include <uima/msg.h>
using namespace std;
namespace uima {

  TypeDescription::TypeDescription(const TypeDescription & crOther) {
    iv_typeName = crOther.getName();
    iv_superTypeName = crOther.getSuperTypeName();
    iv_description = crOther.getDescription();

    TyVecpFeatureDescriptions otherDescs = crOther.getFeatureDescriptions();
    TyVecpFeatureDescriptions::iterator ite;
    for (ite = otherDescs.begin(); ite != otherDescs.end(); ite++) {
      bool takesMemoryOwnership;
      unique_ptr<FeatureDescription> desc ( new FeatureDescription(**ite) );
      addFeatureDescription(desc.get(), takesMemoryOwnership);
      if (takesMemoryOwnership) {
        desc.release();
      }
    }

    TyVecpAllowedValues otherAllowedValues = crOther.getAllowedValues();
    TyVecpAllowedValues::iterator ite1;
    for (ite1 = otherAllowedValues.begin(); ite1 != otherAllowedValues.end(); ite1++) {
      bool takesMemoryOwnership;
      unique_ptr<AllowedValue> allowedval ( new AllowedValue(**ite1) );
      addAllowedValue(allowedval.get(), takesMemoryOwnership);
      if (takesMemoryOwnership) {
        allowedval.release();
      }
    }

  }

  TyErrorId TypeDescription::addFeatureDescription(FeatureDescription * feature, bool & takesMemoryOwnership) {
    takesMemoryOwnership = false;
    if (! isModifiable()) {
      return UIMA_ERR_CONFIG_OBJECT_COMITTED;
    }
    FeatureDescription * pMyFeat = getFeatureDescription(feature->getName());
    if (pMyFeat != NULL) {
      ErrorMessage msg(UIMA_MSG_ID_EXC_DUPLICATE_FEATURE_NAME, getName());
      msg.addParam(feature->getName());
      UIMA_EXC_THROW_NEW(DuplicateConfigElemException,
                         UIMA_ERR_CONFIG_DUPLICATE_FEATURE_NAME,
                         msg,
                         UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                         ErrorInfo::unrecoverable);
    } else {
      takesMemoryOwnership = true;
      iv_vecpFeatureDescriptions.push_back(feature);
    }
    return UIMA_ERR_NONE;
  }

  FeatureDescription * TypeDescription::getFeatureDescription(const icu::UnicodeString & featureName) const {
    bool found = false;
    size_t i=0;
    while ((! found) && i < iv_vecpFeatureDescriptions.size()) {
      found = (featureName.compare(iv_vecpFeatureDescriptions[i]->getName()) == 0);
      i++;
    }
    if (found) {
      return(iv_vecpFeatureDescriptions[i-1]);
    } else {
      return NULL;
    }
  }

  void TypeDescription::mergeFeatureDescriptions(const TyVecpFeatureDescriptions & descs) {
    size_t i;
    for (i=0; i < descs.size(); i++) {
      FeatureDescription * pOtherFeatDesc = descs[i]; // *ite;
      FeatureDescription * pMyFeatDesc = getFeatureDescription(pOtherFeatDesc->getName());
      if (pMyFeatDesc != NULL) {
        //the feature already exists
        //check the rangetype
        if (pMyFeatDesc->getRangeTypeName().compare(pOtherFeatDesc->getRangeTypeName()) == 0) {
          //it's the same feature, we don't have to add it
        } else {
          bool takesMemoryOwnership;
          //error: same name, different range. Will be raised during validation
          //we have to add a copy of FeatureDescription, as we have to get memory ownership
          FeatureDescription * pCopyFeatDesc = new FeatureDescription(*pOtherFeatDesc);
          addFeatureDescription(pCopyFeatDesc, takesMemoryOwnership);
          if (!takesMemoryOwnership) {
            delete pCopyFeatDesc;
          }
        }
      } else {
        //add the feature
        bool takesMemoryOwnership;
        FeatureDescription * pCopyFeatDesc = new FeatureDescription(*pOtherFeatDesc);
        addFeatureDescription(pCopyFeatDesc, takesMemoryOwnership);
        assert( takesMemoryOwnership);
      }
    }
  }

  TyErrorId TypeSystemDescription::addTypeDescription(TypeDescription * pDesc, bool & takesMemoryOwnership) {
    takesMemoryOwnership = false;
    if (! isModifiable()) {
      return UIMA_ERR_CONFIG_OBJECT_COMITTED;
    }

    TypeDescription * pMyTypeDesc = getTypeDescription(pDesc->getName());
    if (pMyTypeDesc != NULL) {
      //the type already exists
      vector<TypeDescription*> v;
      v.push_back(pDesc);
      mergeTypeDescriptions(v);
    } else {
      iv_vecpTypeDescriptions.push_back(pDesc);
      takesMemoryOwnership = true;
    }

    return UIMA_ERR_NONE;
  }

  TypeDescription * TypeSystemDescription::getTypeDescription(const icu::UnicodeString & typeName) const {
    bool found = false;
    size_t i=0;
    while ( (! found) && i<iv_vecpTypeDescriptions.size()) {
      found = (typeName.compare(iv_vecpTypeDescriptions[i]->getName()) == 0);
      i++;
    }
    if (found) {
      return(iv_vecpTypeDescriptions[i-1]);
    } else {
      return NULL;
    }
  }

  void TypeSystemDescription::mergeTypeDescriptions(const TyVecpTypeDescriptions & descs) {
    size_t i;
    for (i=0; i < descs.size(); i++) {
      TypeDescription * pOtherTypeDesc = descs[i];
      TypeDescription * pMyTypeDesc = getTypeDescription(pOtherTypeDesc->getName());
      if (EXISTS(pMyTypeDesc)) {
        //the type already exists
        //check the supertype name
        if (pMyTypeDesc->getSuperTypeName().compare(pOtherTypeDesc->getSuperTypeName()) == 0) {
          //it's the same type
          //merge the feature descriptions
          pMyTypeDesc->mergeFeatureDescriptions(pOtherTypeDesc->getFeatureDescriptions());
          pMyTypeDesc->mergeAllowedValues(pOtherTypeDesc->getAllowedValues());
        } else {
          //error: same name and different supertype name
          UIMA_EXC_THROW_NEW(DuplicateConfigElemException,
                             UIMA_ERR_CONFIG_DUPLICATE_TYPE_NAME,
                             ErrorMessage(UIMA_MSG_ID_EXC_DUPLICATE_TYPE_NAME, pOtherTypeDesc->getName()),
                             UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                             ErrorInfo::unrecoverable);
        }
      } else {
        //it's a new type description, add it
        bool takesMemoryOwnerShip;
        addTypeDescription(new TypeDescription(*pOtherTypeDesc), takesMemoryOwnerShip);
        assert( takesMemoryOwnerShip );
      }
    }
  }


  TyErrorId TypeSystemDescription::addImportDescription(ImportDescription * pDesc,
      bool & takesMemoryOwnerShip)  {
    takesMemoryOwnerShip=false;
    if (! isModifiable()) {
      return UIMA_ERR_CONFIG_OBJECT_COMITTED;
    }

    iv_vecpImportDescriptions.push_back(pDesc);
    takesMemoryOwnerShip=true;
    return UIMA_ERR_NONE;
  }


  TyErrorId TypeSystemDescription::resolveImports(vector<icu::UnicodeString> & alreadyImportedTypeSystemLocations) {

    //holds the import by name descriptors.
    vector<uima::ImportDescription *> vecpImportByNameDescriptions;

    //process each import description
    for (size_t i=0; i < iv_vecpImportDescriptions.size(); i++) {
      ImportDescription * pImport = iv_vecpImportDescriptions[i];
      //only handle import by 'location'.
      //we do not handle import by 'name' which requires looking into
      //classpath and needs to be handle in Java.
      if (pImport->getLocation().length() > 0) {
        const icu::UnicodeString & fileLocation = pImport->findAbsoluteUrl(iv_xmlFileLoc);
        //check it is not in already Imported list
		bool needsToBeResolved=true;
        for (size_t j=0; j < alreadyImportedTypeSystemLocations.size() ;j++) {
          if (fileLocation.compare(alreadyImportedTypeSystemLocations[j]) == 0 )  {
			  needsToBeResolved=false;
			  break;
		  }
        }

		if (needsToBeResolved) {
          //resolve import
          resolveImport(fileLocation,
                      alreadyImportedTypeSystemLocations,
                      iv_vecpTypeDescriptions);
		}
		//delete this import description
        delete iv_vecpImportDescriptions[i];
        iv_vecpImportDescriptions[i] = 0;
      } else {
        // save the import descriptor.
        // will be passed to Java if calling Java
        vecpImportByNameDescriptions.push_back(iv_vecpImportDescriptions[i]);
		iv_vecpImportDescriptions[i] = 0;
      }
	  
    }
    iv_vecpImportDescriptions.clear();
	
    //add back the import by name descriptors
    for (size_t i=0; i < vecpImportByNameDescriptions.size(); i++) {
      iv_vecpImportDescriptions.push_back(vecpImportByNameDescriptions[i]);
    }
    return UIMA_ERR_NONE;
  }

  TyErrorId TypeSystemDescription::resolveImport(const icu::UnicodeString & fileLocation,
      vector<icu::UnicodeString> & alreadyImportedTypeSystemLocations,
      TyVecpTypeDescriptions & result) {

    //build typesystem description
    TextAnalysisEngineSpecifierBuilder builder;
    TypeSystemDescription * pDesc = builder.buildTypeSystemSpecifierFromFile(fileLocation);
    if (EXISTS(pDesc)) {
      //add this file to already imported list
      alreadyImportedTypeSystemLocations.push_back(fileLocation);

      //resolve imports within this descriptor
      pDesc->resolveImports(alreadyImportedTypeSystemLocations);

      //add type descriptions to result
      const TyVecpTypeDescriptions & importedTypes = pDesc->getTypeDescriptions();
      for (size_t i=0; i < importedTypes.size() ; i++) {
        TypeDescription * pOtherTypeDesc = importedTypes[i];
        bool takesMemoryOwnerShip;
        TypeDescription * pType = new TypeDescription(*pOtherTypeDesc);
        addTypeDescription(pType, takesMemoryOwnerShip);
        if (!takesMemoryOwnerShip) {
          delete pType;
        }
      }
      delete pDesc;
      return UIMA_ERR_NONE;
    } else {
      return UIMA_ERR_RESOURCE_CORRUPTED;
    }

  }


  void TypeSystemDescription::validate() {

    size_t i=0;

    TypeSystemDescription::TyVecpTypeDescriptions typeDescs = getTypeDescriptions();

    vector < icu::UnicodeString > typeNames;

    for (i=0; i<typeDescs.size(); i++) {
      typeNames.push_back(typeDescs[i]->getName());
    }

    //ensure that there are no two type descriptions with the same name
    for (i=0; i < typeNames.size(); i++) {
      if (countValues(typeNames.begin(), typeNames.end(), typeNames[i])  > 1) {
        UIMA_EXC_THROW_NEW(ValidationException,
                           UIMA_ERR_CONFIG_DUPLICATE_TYPE_NAME,
                           ErrorMessage(UIMA_MSG_ID_EXC_DUPLICATE_TYPE_NAME, typeNames[i]),
                           UIMA_MSG_ID_EXCON_VALIDATE_TAE_SPEC,
                           ErrorInfo::unrecoverable);

      }
    }

    //ensure that the type descriptions are valid
    for (i=0; i<typeDescs.size(); i++) {
		typeDescs[i]->validate();
    }
  }



  //allowed values
  TyErrorId TypeDescription::addAllowedValue(AllowedValue * allowed, bool & takesMemoryOwnership) {
    takesMemoryOwnership = false;
    if (! isModifiable()) {
      return UIMA_ERR_CONFIG_OBJECT_COMITTED;
    }
    AllowedValue * pMy = getAllowedValue(allowed->getName());
    if (pMy != NULL) {
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_DUPLICATE_ALLOWED_VALUE);
      errMsg.addParam(getName());
      errMsg.addParam(allowed->getName());

      UIMA_EXC_THROW_NEW(DuplicateConfigElemException,
                         UIMA_ERR_CONFIG_DUPLICATE_ALLOWED_VALUE,
                         errMsg,
                         //ErrorMessage(UIMA_MSG_ID_EXC_DUPLICATE_ALLOWED_VALUE, allowed->getName()),
                         UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                         ErrorInfo::unrecoverable);

    } else {
      takesMemoryOwnership = true;
      iv_vecpAllowedValues.push_back(allowed);
    }
    return UIMA_ERR_NONE;
  }

  AllowedValue * TypeDescription::getAllowedValue(const icu::UnicodeString & name) const {
    bool found = false;
    size_t i=0;
    while ((! found) && i < iv_vecpAllowedValues.size()) {
      found = (name.compare(iv_vecpAllowedValues[i]->getName()) == 0);
      i++;
    }
    if (found) {
      return(iv_vecpAllowedValues[i-1]);
    } else {
      return NULL;
    }
  }

  void TypeDescription::mergeAllowedValues(const TyVecpAllowedValues & descs) {
    size_t i;
    for (i=0; i < descs.size(); i++) {
      AllowedValue * pOtherDesc = descs[i]; // *ite;
      AllowedValue * pMyDesc = getAllowedValue(pOtherDesc->getName());
      if (pMyDesc != NULL) {
        //already exists
      } else {
        //add
        bool takesMemoryOwnership;
        AllowedValue * pCopyDesc = new AllowedValue(*pOtherDesc);
        addAllowedValue(pCopyDesc, takesMemoryOwnership);
        assert( takesMemoryOwnership);
      }
    }
  }


  void TypeDescription::validate() {

    // if there are allowed values
    if (getAllowedValues().size() != 0) {
      if (getSuperTypeName() != "uima.cas.String") {
        UIMA_EXC_THROW_NEW(ValidationException,
                           UIMA_ERR_CONFIG_ALLOWED_VALUES_DEFINED_FOR_NON_STRING_TYPE,
                           ErrorMessage( UIMA_MSG_ID_EXC_ALLOWEDVAL_DEF_FOR_NONSTRINGTYPE,getName()),
                           UIMA_MSG_ID_EXCON_VALIDATE_TAE_SPEC,
                           ErrorInfo::unrecoverable);
      }
    }

    size_t i=0;
    TypeDescription::TyVecpFeatureDescriptions featDescs = getFeatureDescriptions();
    vector<icu::UnicodeString> featureNames;

    //ensure that there are no two feature descriptions with the same name
    for (i=0; i<featDescs.size(); i++) {
      if (countValues(featureNames.begin(), featureNames.end(), featDescs[i]->getName()) != 0) {
        ErrorMessage errMsg(UIMA_MSG_ID_EXC_DUPLICATE_FEATURE_NAME);
        errMsg.addParam(getName());
        errMsg.addParam(featureNames[i]);
        UIMA_EXC_THROW_NEW(ValidationException,
                           UIMA_ERR_CONFIG_DUPLICATE_FEATURE_NAME,
                           errMsg,
                           UIMA_MSG_ID_EXCON_VALIDATE_TAE_SPEC,
                           ErrorInfo::unrecoverable);

      } else {
        featureNames.push_back(featDescs[i]->getName());
      }
    }
  }




}

