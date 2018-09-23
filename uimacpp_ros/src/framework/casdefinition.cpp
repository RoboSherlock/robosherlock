/** \file casdefinition.cpp .
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

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp>

#include "xercesc/util/PlatformUtils.hpp"
#include "xercesc/parsers/XercesDOMParser.hpp"
#include "xercesc/parsers/SAXParser.hpp"
#include "xercesc/dom/DOMException.hpp"
#include "xercesc/dom/DOMNamedNodeMap.hpp"
#include "xercesc/dom/DOMDocument.hpp"
#include "xercesc/dom/DOMElement.hpp"
#include "xercesc/dom/DOMNodeList.hpp"

#include "xercesc/sax/ErrorHandler.hpp"
#include "xercesc/sax/AttributeList.hpp"
#include "xercesc/sax/SAXParseException.hpp"
#include "xercesc/framework/LocalFileInputSource.hpp"
#include "xercesc/framework/MemBufInputSource.hpp"

#include <uima/macros.h>
#include <uima/engine.hpp>   // for exception declarations
#include <uima/msg.h>
#include <uima/casdefinition.hpp>
#include <uima/lowlevel_typesystem.hpp>
#include <uima/lowlevel_indexdefinition.hpp>
#include <uima/annotator_context.hpp>
#include <uima/resmgr.hpp>
#include <uima/taespecifierbuilder.hpp>
#include <uima/xmlerror_handler.hpp>

using namespace std;
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */
namespace uima {
  lowlevel::TypeSystem::StTypeInfo gs_arCASTypes[] = {
        { CAS::TYPE_NAME_SOFA,                   CAS::TYPE_NAME_TOP,              "Predefined sofa type"},
        { CAS::TYPE_NAME_ANNOTATION_BASE,        CAS::TYPE_NAME_TOP,             "Predefined annotation type"},
        { CAS::TYPE_NAME_ANNOTATION,             CAS::TYPE_NAME_ANNOTATION_BASE, "Predefined annotation type"},
        { CAS::TYPE_NAME_DOCUMENT_ANNOTATION,    CAS::TYPE_NAME_ANNOTATION,       "Predefined document annotation type"},
      };

  lowlevel::TypeSystem::StFeatureInfo gs_arCASFeatures[] = {
        { CAS::FEATURE_BASE_NAME_SOFANUM,   CAS::TYPE_NAME_SOFA,        CAS::TYPE_NAME_INTEGER, false, "Predefined feature for sofa"},
        { CAS::FEATURE_BASE_NAME_SOFAID,    CAS::TYPE_NAME_SOFA,        CAS::TYPE_NAME_STRING,  false, "Predefined feature for sofa"},
        { CAS::FEATURE_BASE_NAME_SOFAMIME,  CAS::TYPE_NAME_SOFA,        CAS::TYPE_NAME_STRING,  false, "Predefined feature for sofa"},

        { CAS::FEATURE_BASE_NAME_SOFAARRAY,  CAS::TYPE_NAME_LOCALSOFA,   CAS::TYPE_NAME_TOP,     false, "Predefined feature for sofa"},
        { CAS::FEATURE_BASE_NAME_SOFASTRING, CAS::TYPE_NAME_LOCALSOFA,   CAS::TYPE_NAME_STRING,  false, "Predefined feature for sofa"},
        { CAS::FEATURE_BASE_NAME_SOFAURI,    CAS::TYPE_NAME_REMOTESOFA,  CAS::TYPE_NAME_STRING,  false, "Predefined feature for sofa"},

        { CAS::FEATURE_BASE_NAME_SOFA,  CAS::TYPE_NAME_ANNOTATION, CAS::TYPE_NAME_SOFA,    false, "Predefined sofaReference feature for annotations"},
        { CAS::FEATURE_BASE_NAME_BEGIN, CAS::TYPE_NAME_ANNOTATION, CAS::TYPE_NAME_INTEGER, false, "Predefined beginPosition feature for annotations"},
        { CAS::FEATURE_BASE_NAME_END,   CAS::TYPE_NAME_ANNOTATION, CAS::TYPE_NAME_INTEGER, false, "Predefined endPosition feature for annotations"},

        { CAS::FEATURE_BASE_NAME_LANGUAGE, CAS::TYPE_NAME_DOCUMENT_ANNOTATION, CAS::TYPE_NAME_STRING, false, "Predefined language feature for document annotations"},
      };
}

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


namespace uima {

  namespace internal {

    uima::lowlevel::TyFSType mergeType(uima::lowlevel::TypeSystem & rTypeSystem,
                                       icu::UnicodeString const & crTypeToBeCreated,
                                       TypeSystemDescription const & crTSDesc,
                                       icu::UnicodeString const & crCreatorID) {
      uima::lowlevel::TyFSType typeToBeCreated = rTypeSystem.getTypeByName(crTypeToBeCreated);
      if (crTSDesc.hasTypeDescription(crTypeToBeCreated)) {
        TypeDescription const * cpTypeDesc = crTSDesc.getTypeDescription(crTypeToBeCreated);
        icu::UnicodeString const & crSuperTypeName = cpTypeDesc->getSuperTypeName();

        // first merge the parent
        uima::lowlevel::TyFSType parent = mergeType(rTypeSystem, crSuperTypeName, crTSDesc, crCreatorID);

        // if string sub type
        if (parent == uima::internal::gs_tyStringType) {
          TypeDescription::TyVecpAllowedValues const & crAllowedValues = cpTypeDesc->getAllowedValues();
          vector<icu::UnicodeString> allowedStringValues;
          TypeDescription::TyVecpAllowedValues::const_iterator cit;
          for (cit = crAllowedValues.begin(); cit != crAllowedValues.end(); ++cit) {
            AllowedValue const * cpAllowedVal = (*cit);
            allowedStringValues.push_back( cpAllowedVal->getName() );
          }

          if (typeToBeCreated == uima::lowlevel::TypeSystem::INVALID_TYPE) {
            // create new string subtype
            return rTypeSystem.createStringSubtype(crTypeToBeCreated, allowedStringValues, crCreatorID);
          } else {
            // check string subtype
            vector<icu::UnicodeString> const & crStringValues = rTypeSystem.getStringsForStringSubtype(typeToBeCreated);
            bool bAreStringSetsEqual = false;
            if (crStringValues.size() == crAllowedValues.size()) {
              size_t i,j;
              size_t uiStringNum = crStringValues.size();
              bAreStringSetsEqual = true;
              for (i=0; i<uiStringNum; ++i) {
                icu::UnicodeString const & crString = crStringValues[i];
                // check that string occurs in the allowed values
                bool bContainsString = false;
                for (j=0; j<uiStringNum; ++j) {
                  if (crString != allowedStringValues[j]) {
                    bContainsString = true;
                    break;
                  }
                }
                if (! bContainsString) {
                  bAreStringSetsEqual = false;
                  break;
                }
              }
            }
            if (! bAreStringSetsEqual) {
              // throw exception
              // allowed values are different
              UIMA_EXC_THROW_NEW(AllowedStringValuesIncompatibleException,
                                 UIMA_ERR_ALLOWED_STRING_VALUES_INCOMPATIBLE,
                                 ErrorMessage(UIMA_MSG_ID_EXC_ALLOWED_STRING_VALUES_INCOMPATIBLE, crTypeToBeCreated),
                                 ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_TYPESYSTEM_FROM_CONFIG),
                                 ErrorInfo::recoverable);
              return 0;
            }
          }
        } else {
          assert(parent != uima::lowlevel::TypeSystem::INVALID_TYPE);
          if (typeToBeCreated == uima::lowlevel::TypeSystem::INVALID_TYPE) {
            return rTypeSystem.createType(parent, crTypeToBeCreated, crCreatorID);
          } else {
            if (parent != rTypeSystem.getParentType(typeToBeCreated)) {
              UIMA_EXC_THROW_NEW(IncompatibleParentTypesException,
                                 UIMA_ERR_INCOMPATIBLE_PARENT_TYPES,
                                 ErrorMessage(UIMA_MSG_ID_EXC_WRONG_PARENT_TYPE, crTypeToBeCreated),
                                 ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_TYPESYSTEM_FROM_CONFIG),
                                 ErrorInfo::recoverable);
              return 0;
            }
          }
        }
        return typeToBeCreated;
      }



      if (typeToBeCreated == uima::lowlevel::TypeSystem::INVALID_TYPE) {
        // unknown type in XML file

        UIMA_EXC_THROW_NEW(UnknownTypeException,
                           UIMA_ERR_UNKNOWN_TYPE,
                           ErrorMessage(UIMA_MSG_ID_EXC_UNKNOWN_TYPE_NAME, crTypeToBeCreated),
                           ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_TYPESYSTEM_FROM_CONFIG),
                           ErrorInfo::recoverable);

      }
      return typeToBeCreated;
    }



    void CASDefinition::mergeTypeSystem(AnalysisEngineDescription const & crTAESpecifier) {
      icu::UnicodeString const & crCreatorID = crTAESpecifier.getAnalysisEngineMetaData()->getName();

      TypeSystemDescription const * crTSDesc = crTAESpecifier.getAnalysisEngineMetaData()->getTypeSystemDescription();
      mergeTypeSystem(*crTSDesc,crCreatorID);

    }



    void CASDefinition::addTypePriorities(AnalysisEngineDescription const & crTAESpecifier) {
      uima::AnalysisEngineMetaData::TyVecpTypePriorities const & crTypePriorities = crTAESpecifier.getAnalysisEngineMetaData()->getTypePriorities();
      addTypePriorities(crTypePriorities);
    }

    void CASDefinition::createIndexesFromANC(AnnotatorContext const & crANC) {
      AnalysisEngineMetaData::TyVecpFSIndexDescriptions const & crIndexDescVec = crANC.getFSIndexDescriptions();
      createIndexesFromSpecifier(crIndexDescVec);
    }

    CASDefinition * CASDefinition::createCASDefinition(AnnotatorContext const & anc) {
      CASDefinition * result = new CASDefinition(& anc);
      assert( EXISTS(result) );
      result->init();
      result->commit();
      return result;
    }

    void CASDefinition::init() {
      createTypes();
    }

    void CASDefinition::commitTypeSystem() {
      iv_typeSystem->commit();
      createIndexes();
    }

    void CASDefinition::commitIndexDefinition() {
      iv_indexDefinition->commit();
    }

    void CASDefinition::commit() {
      commitTypeSystem();
      commitIndexDefinition();
    }

    void CASDefinition::createTypes() {
      createPredefinedCASTypes();
      if ( iv_annotatorContext != NULL ) {
        mergeTypeSystem(iv_annotatorContext->getTaeSpecifier());
        addTypePriorities(iv_annotatorContext->getTaeSpecifier());
      }
    }
    void CASDefinition::createIndexes() {
      createPredefinedCASIndexes();
      if ( iv_annotatorContext != NULL ) {
        createIndexesFromANC(*iv_annotatorContext);
      }
    }

    void CASDefinition::createPredefinedCASTypes() {
      // the CAS predefined types are created in lowlevel::TypeSystem CTOR
      // add CAS types here
      assert( EXISTS(iv_typeSystem) );
      size_t uiType;
      for (uiType =  0; uiType < NUMBEROF(gs_arCASTypes); ++uiType) {
        lowlevel::TyFSType tyNewType = iv_typeSystem->createType( gs_arCASTypes[uiType], CAS::ustrCREATOR_ID_CAS );
        assert( iv_typeSystem->isValidType(tyNewType) );
      }

      size_t uiFeature;
      for (uiFeature = 0; uiFeature < NUMBEROF(gs_arCASFeatures); ++uiFeature) {
        lowlevel::TyFSFeature tyNewFeature = iv_typeSystem->createFeature( gs_arCASFeatures[uiFeature], CAS::ustrCREATOR_ID_CAS );
        assert( iv_typeSystem->isValidFeature(tyNewFeature) );
      }

    }


    void CASDefinition::createPredefinedCASIndexes() {
      assert( EXISTS(iv_typeSystem) );
      assert( EXISTS(iv_indexDefinition) );

      vector<lowlevel::TyFSFeature> vecKeyFeats;
      vector<uima::lowlevel::IndexComparator::EnKeyFeatureComp> vecComparators;

      // create sofa index
      UIMA_TPRINT("creating sofa index");
      vecKeyFeats.clear();
      vecComparators.clear();
      vecKeyFeats.push_back( uima::internal::gs_tySofaNumFeature );
      vecComparators.push_back( uima::lowlevel::IndexComparator::STANDARD_COMPARE );

      assert( iv_typeSystem->isValidType( iv_typeSystem->getTypeByName(CAS::TYPE_NAME_SOFA) ) );
      iv_indexDefinition->defineIndex( uima::lowlevel::IndexDefinition::enSet,
                                       iv_typeSystem->getTypeByName(CAS::TYPE_NAME_SOFA),
                                       vecKeyFeats,
                                       vecComparators,
                                       CAS::INDEXID_SOFA);

      // create annotation index
      UIMA_TPRINT("creating default annotation index");
      vecKeyFeats.clear();
      vecComparators.clear();

      assert( areTypeShortcutsCorrect(*iv_typeSystem) );

      vecKeyFeats.push_back( uima::internal::gs_tyBeginPosFeature );
      vecComparators.push_back( uima::lowlevel::IndexComparator::STANDARD_COMPARE );
      vecKeyFeats.push_back( uima::internal::gs_tyEndPosFeature );
      vecComparators.push_back( uima::lowlevel::IndexComparator::REVERSE_STANDARD_COMPARE );
      vecKeyFeats.push_back( uima::lowlevel::TypeSystem::INVALID_FEATURE ); // means that type priority should be respected
      vecComparators.push_back( uima::lowlevel::IndexComparator::STANDARD_COMPARE );

      assert( iv_typeSystem->isValidType( iv_typeSystem->getTypeByName(CAS::TYPE_NAME_ANNOTATION) ) );
      iv_indexDefinition->defineIndex( uima::lowlevel::IndexDefinition::enOrdered,
                                       iv_typeSystem->getTypeByName(CAS::TYPE_NAME_ANNOTATION),
                                       vecKeyFeats,
                                       vecComparators,
                                       CAS::INDEXID_ANNOTATION);

    }

    CASDefinition::CASDefinition(AnnotatorContext const * anc)
        : iv_typeSystem(NULL),
        iv_indexDefinition(NULL),
        iv_annotatorContext(anc) {
      iv_typeSystem = new uima::lowlevel::TypeSystem();
      bOwnsTypeSystem = true;
      assert( EXISTS(iv_typeSystem) );
      iv_indexDefinition = new uima::lowlevel::IndexDefinition(*iv_typeSystem);
      assert( EXISTS(iv_indexDefinition) );

    }



    CASDefinition::~CASDefinition() {

      if (iv_typeSystem != NULL && bOwnsTypeSystem) {
        delete iv_typeSystem;
      }
      if (iv_indexDefinition != NULL) {
        delete iv_indexDefinition;
      }
    }


    uima::lowlevel::TypeSystem & CASDefinition::getTypeSystem() {
      assert( EXISTS(iv_typeSystem) );
      return *iv_typeSystem;
    }

    uima::lowlevel::TypeSystem const & CASDefinition::getTypeSystem() const {
      assert( EXISTS(iv_typeSystem) );
      return *iv_typeSystem;
    }

    uima::lowlevel::IndexDefinition & CASDefinition::getIndexDefinition() {
      assert(EXISTS(iv_indexDefinition));
      return *iv_indexDefinition;
    }

    uima::lowlevel::IndexDefinition const & CASDefinition::getIndexDefinition() const {
      assert(EXISTS(iv_indexDefinition));
      return *iv_indexDefinition;
    }

    void CASDefinition::mergeTypeSystem(TypeSystemDescription const & crTSDesc,
                                        icu::UnicodeString const & crCreatorID) {
      //icu::UnicodeString const & crCreatorID = crAEMDSpecifier.getName();

      uima::lowlevel::TypeSystem & rTypeSystem = * iv_typeSystem;

      TypeSystemDescription::TyVecpTypeDescriptions const & crTypeDescVector = crTSDesc.getTypeDescriptions();
      TypeSystemDescription::TyVecpTypeDescriptions::const_iterator cit;

      // create types
      for (cit = crTypeDescVector.begin(); cit != crTypeDescVector.end(); ++cit) {
        TypeDescription const * pTypeDesc = *cit;
        icu::UnicodeString const & crTypeName = pTypeDesc->getName();
        mergeType( rTypeSystem, crTypeName, crTSDesc, crCreatorID );
      }

      // create features
      for (cit = crTypeDescVector.begin(); cit != crTypeDescVector.end(); ++cit) {
        TypeDescription const * pTypeDesc = *cit;
        uima::lowlevel::TyFSType introType = rTypeSystem.getTypeByName( pTypeDesc->getName() );
        // type must have been merged before
        assert( rTypeSystem.isValidType(introType) );

        TypeDescription::TyVecpFeatureDescriptions const & crFeatDescs = pTypeDesc->getFeatureDescriptions();
        TypeDescription::TyVecpFeatureDescriptions::const_iterator citFeats;
        // for all features introduced on introType
        for (citFeats = crFeatDescs.begin(); citFeats != crFeatDescs.end(); ++ citFeats) {
          FeatureDescription const * pFeatDesc = *citFeats;
          icu::UnicodeString const & crFeatureName = pFeatDesc->getName();
          icu::UnicodeString const & crRangeTypeName = pFeatDesc->getRangeTypeName();
		  bool multiRefs = pFeatDesc->isMultipleReferencesAllowed();
          uima::lowlevel::TyFSType rangeType = rTypeSystem.getTypeByName(crRangeTypeName);
          if ( !rTypeSystem.isValidType(rangeType) ) {
            // throw exception
            // unknown range type
            ErrorMessage errMessage(UIMA_MSG_ID_EXC_INVALID_RANGE_TYPE, crFeatureName);
            errMessage.addParam( crRangeTypeName );
            UIMA_EXC_THROW_NEW(UnknownRangeTypeException,
                               UIMA_ERR_UNKNOWN_RANGE_TYPE,
                               errMessage,
                               ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_TYPESYSTEM_FROM_CONFIG),
                               ErrorInfo::recoverable);
          }

          uima::lowlevel::TyFSFeature feat = rTypeSystem.getFeatureByBaseName( introType, crFeatureName );
          // if feature exists
          if ( rTypeSystem.isValidFeature(feat) ) {
            // check that range type is correct
            uima::lowlevel::TyFSType actualRangeType = rTypeSystem.getRangeType(feat);
            if (actualRangeType != rangeType) {
              // throw exception
              // incorrect range type of existing feature
              ErrorMessage errMessage(UIMA_MSG_ID_EXC_INCOMPATIBLE_RANGE_TYPE, crFeatureName);
              errMessage.addParam( crRangeTypeName );
              UIMA_EXC_THROW_NEW(IncompatibleRangeTypeException,
                                 UIMA_ERR_INCOMPATIBLE_RANGE_TYPE,
                                 errMessage,
                                 ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_TYPESYSTEM_FROM_CONFIG),
                                 ErrorInfo::recoverable);
            }
          } else {
            // create feature
            rTypeSystem.createFeature(introType, rangeType, multiRefs, crFeatureName, crCreatorID);
          }
        }
      }
    }


    void CASDefinition::addTypePriorities(uima::AnalysisEngineMetaData::TyVecpTypePriorities const & crTypePriorities) {
      uima::lowlevel::TypeSystem & rTypeSystem = *iv_typeSystem;

      //uima::AnalysisEngineMetaData::TyVecpTypePriorities const & crTypePriorities = crAEMDSpecifier.getTypePriorities();
      uima::AnalysisEngineMetaData::TyVecpTypePriorities::const_iterator cit;
      // for all type priority chains
      for (cit = crTypePriorities.begin(); cit != crTypePriorities.end(); ++cit) {
        TypePriority const * cpTypePriority = *cit;
        assert( EXISTS(cpTypePriority) );
        vector<icu::UnicodeString> const & crTypeChain = cpTypePriority->getTypeOrder();
        size_t i;
        // walk along the chain
        for (i=0; i<crTypeChain.size()-1; ++i) {
          uima::lowlevel::TyFSType t1 = rTypeSystem.getTypeByName(crTypeChain[i]);
          if (t1 == uima::lowlevel::TypeSystem::INVALID_TYPE) {
            UIMA_EXC_THROW_NEW(UnknownTypeException,
                               UIMA_ERR_UNKNOWN_TYPE,
                               ErrorMessage(UIMA_MSG_ID_EXC_UNKNOWN_TYPE_NAME, crTypeChain[i]),
                               ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_TYPEPRIORITIES_FROM_CONFIG),
                               ErrorInfo::recoverable);
          }
          uima::lowlevel::TyFSType t2 = rTypeSystem.getTypeByName(crTypeChain[i+1]);
          if (t2 == uima::lowlevel::TypeSystem::INVALID_TYPE) {
            UIMA_EXC_THROW_NEW(UnknownTypeException,
                               UIMA_ERR_UNKNOWN_TYPE,
                               ErrorMessage(UIMA_MSG_ID_EXC_UNKNOWN_TYPE_NAME, crTypeChain[i+1]),
                               ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_TYPEPRIORITIES_FROM_CONFIG),
                               ErrorInfo::recoverable);
          }
          // t1 should have priority over t2
          bool bTPCouldBeAdded = rTypeSystem.addTypePriority(t1, t2);
          if (!bTPCouldBeAdded) {
            ErrorMessage errMsg(UIMA_MSG_ID_EXC_TYPE_PRIORITY_CONFLICT);
            errMsg.addParam( crTypeChain[i] );
            errMsg.addParam( crTypeChain[i+1] );

            UIMA_EXC_THROW_NEW(TypePriorityConflictException,
                               UIMA_ERR_TYPE_PRIORITY_CONFLICT,
                               errMsg,
                               ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_TYPEPRIORITIES_FROM_CONFIG),
                               ErrorInfo::recoverable);
          }
        }
      }

    }


    void CASDefinition::createIndexesFromSpecifier(AnalysisEngineMetaData::TyVecpFSIndexDescriptions const & crIndexDescVec) {
      uima::lowlevel::TypeSystem const & crTypeSystem = *iv_typeSystem;

      vector<uima::lowlevel::TyFSFeature> keyFeatures;
      vector<uima::lowlevel::IndexComparator::EnKeyFeatureComp> comparators;

      AnalysisEngineMetaData::TyVecpFSIndexDescriptions::const_iterator citIndex;
      // for all index descriptions
      for (citIndex = crIndexDescVec.begin(); citIndex != crIndexDescVec.end(); ++citIndex) {
        FSIndexDescription const * cpIndexDesc = *citIndex;

        // determine index kind
        FSIndexDescription::EnIndexKind indexDescKind = cpIndexDesc->getIndexKind();
        uima::lowlevel::IndexDefinition::EnIndexKind kind = uima::lowlevel::IndexDefinition::enOrdered;
        switch (indexDescKind) {
        case FSIndexDescription::SORTED:
          kind = uima::lowlevel::IndexDefinition::enOrdered;
          break;
        case FSIndexDescription::SET:
          kind = uima::lowlevel::IndexDefinition::enSet;
          break;
        case FSIndexDescription::BAG:
          kind = uima::lowlevel::IndexDefinition::enFIFO;
          break;
        default:
          assert(false);
        }

        icu::UnicodeString const & crTypeName = cpIndexDesc->getTypeName();
        // type is the type the index is defined on
        uima::lowlevel::TyFSType type = crTypeSystem.getTypeByName(crTypeName);
        if (! crTypeSystem.isValidType(type) ) {
          UIMA_EXC_THROW_NEW(UnknownTypeException,
                             UIMA_ERR_UNKNOWN_TYPE,
                             ErrorMessage(UIMA_MSG_ID_EXC_UNKNOWN_TYPE_NAME, crTypeName),
                             ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_INDEXES_FROM_CONFIG),
                             ErrorInfo::recoverable);
        }

        icu::UnicodeString const & crIndexLabel = cpIndexDesc->getLabel();

        // determine key features and comparison scheme for each feature
        keyFeatures.clear();
        comparators.clear();

        uima::FSIndexDescription::TyVecpFSIndexKeys const & crIndexKeys = cpIndexDesc->getFSIndexKeys();
        uima::FSIndexDescription::TyVecpFSIndexKeys::const_iterator citKeys;
        // for each key feature
        for (citKeys = crIndexKeys.begin(); citKeys != crIndexKeys.end(); ++citKeys) {
          FSIndexKeyDescription const * cpIndexKeyDesc = *citKeys;

          uima::lowlevel::TyFSFeature feat;
          bool bIsTypePriority = cpIndexKeyDesc->isTypePriority();
          if ( ! bIsTypePriority) {
            icu::UnicodeString const & crFeatureName = cpIndexKeyDesc->getFeatureName();

            feat = crTypeSystem.getFeatureByBaseName( type, crFeatureName);
            if (! crTypeSystem.isValidFeature(feat) ) {
              UIMA_EXC_THROW_NEW(UnknownFeatureException,
                                 UIMA_ERR_UNKNOWN_FEATURE,
                                 ErrorMessage(UIMA_MSG_ID_EXC_UNKNOWN_FEATURE_NAME, crFeatureName),
                                 ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_INDEXES_FROM_CONFIG),
                                 ErrorInfo::recoverable);

            }
            assert( crTypeSystem.isValidFeature(feat) );
          } else {
            // use INVALID_FEATURE as a marker for type priority
            feat = uima::lowlevel::TypeSystem::INVALID_FEATURE;
          }

          keyFeatures.push_back(feat);


          FSIndexKeyDescription::EnComparatorType enComp = FSIndexKeyDescription::STANDARD;
          // this if statement is not necessary when type priorities can have different kind of comparators!!
          if (! bIsTypePriority) {
            enComp = cpIndexKeyDesc->getComparator();
          }
          switch (enComp) {
          case FSIndexKeyDescription::STANDARD:
            comparators.push_back( uima::lowlevel::IndexComparator::STANDARD_COMPARE );
            break;
          case FSIndexKeyDescription::REVERSE:
            comparators.push_back( uima::lowlevel::IndexComparator::REVERSE_STANDARD_COMPARE );
            break;
          default:
            assert(false);
          }

        }

        iv_indexDefinition->defineIndex( kind,
                                         type,
                                         keyFeatures,
                                         comparators,
                                         crIndexLabel );
      }


    }

    //creates typesystem and index definition
    CASDefinition * CASDefinition::createCASDefinition(AnalysisEngineDescription const & taeSpecifier) {
      //resolve imports
      if (taeSpecifier.isModifiable() ) {
        AnalysisEngineDescription & taespec = CONST_CAST(AnalysisEngineDescription &, taeSpecifier);
        taespec.commit();
      }

      CASDefinition * result = new CASDefinition(taeSpecifier);
      assert( EXISTS(result) );
      //result->init();
      result->createPredefinedCASTypes();
      result->mergeTypeSystem(*taeSpecifier.getAnalysisEngineMetaData()->getTypeSystemDescription(),
                              taeSpecifier.getAnalysisEngineMetaData()->getName());
      result->addTypePriorities(taeSpecifier.getAnalysisEngineMetaData()->getTypePriorities());

      //result->commit();
      result->commitTypeSystemOnly();
      result->createPredefinedCASIndexes();
      result->createIndexesFromSpecifier(taeSpecifier.getAnalysisEngineMetaData()->getFSIndexDescriptions());
      result->commitIndexDefinition();

      return result;
    }

    //creates typesystem and index definition from
    CASDefinition * CASDefinition::createCASDefinition(AnalysisEngineMetaData const & aeDesc) {
      if (aeDesc.isModifiable()) {
        AnalysisEngineMetaData & ae = CONST_CAST(AnalysisEngineMetaData &, aeDesc);
        ae.commit();
      }

      CASDefinition * result = new CASDefinition();
      assert( EXISTS(result) );

      result->createPredefinedCASTypes();
      //result->mergeTypeSystem(casSpecifier);
      result->mergeTypeSystem(*aeDesc.getTypeSystemDescription(),
                              aeDesc.getName());

      result->addTypePriorities(aeDesc.getTypePriorities());
      result->commitTypeSystemOnly();

      result->createPredefinedCASIndexes();
      result->createIndexesFromSpecifier(aeDesc.getFSIndexDescriptions());
      result->commitIndexDefinition();

      return result;
    }

    //creates a CASDefinition with input TypeSystem object and creates index definition
    //based on the specifications
    CASDefinition * CASDefinition::createCASDefinition(uima::TypeSystem & typeSystem,
        AnalysisEngineMetaData const & aeDesc) {

      CASDefinition * result = new CASDefinition(typeSystem);
      assert( EXISTS(result) );

      result->createPredefinedCASIndexes();
      result->createIndexesFromSpecifier(aeDesc.getFSIndexDescriptions());
      result->commitIndexDefinition();

      return result;

    }

    CASDefinition * CASDefinition::createCASDefinition(TypeSystem   & typeSystem,
        AnalysisEngineMetaData::TyVecpFSIndexDescriptions  const & fsDesc,
        AnalysisEngineMetaData::TyVecpTypePriorities   const & prioDesc) {

      CASDefinition * result = new CASDefinition(typeSystem);
      assert( EXISTS(result) );

      result->createPredefinedCASIndexes();
      result->createIndexesFromSpecifier(fsDesc);
      //TODO: addTypePriorities
      result->commitIndexDefinition();

      return result;

    }

    CASDefinition * CASDefinition::createCASDefinition(TypeSystem   & typeSystem,
        AnalysisEngineMetaData::TyVecpFSIndexDescriptions  const & fsDesc) {

      CASDefinition * result = new CASDefinition(typeSystem);
      assert( EXISTS(result) );

      result->createPredefinedCASIndexes();
      result->createIndexesFromSpecifier(fsDesc);

      result->commitIndexDefinition();

      return result;

    }




    //creates a CASDefition based on input TypeSystem and creates the default/builtin index definition
    CASDefinition * CASDefinition::createCASDefinition(uima::TypeSystem & typeSystem) {
      CASDefinition * result = new CASDefinition(typeSystem);
      assert( EXISTS(result) );
      result->createPredefinedCASIndexes();
      result->commitIndexDefinition();
      return result;
    }


    uima::lowlevel::TypeSystem * CASDefinition::createTypeSystem(AnalysisEngineMetaData const & casSpecifier) {

      uima::lowlevel::TypeSystem * pTypeSystem = new uima::lowlevel::TypeSystem();
      assert( EXISTS(pTypeSystem) );
      CASDefinition * result = new CASDefinition(*pTypeSystem);
      assert( EXISTS(result) );
      result->createPredefinedCASTypes();
      result->mergeTypeSystem(*casSpecifier.getTypeSystemDescription(),
                              casSpecifier.getName());

      result->addTypePriorities(casSpecifier.getTypePriorities());
      pTypeSystem->commit();
      delete result;
      return pTypeSystem;
    }

    uima::lowlevel::TypeSystem * CASDefinition::createTypeSystem(TypeSystemDescription const & crTSDesc,
        icu::UnicodeString const & tsName) {
      //resolve imports
      if (crTSDesc.isModifiable() ) {
        TypeSystemDescription & tsDesc = CONST_CAST(TypeSystemDescription &, crTSDesc);
        tsDesc.commit();
      }

      uima::lowlevel::TypeSystem * pTypeSystem = new uima::lowlevel::TypeSystem();
      assert( EXISTS(pTypeSystem) );
      CASDefinition * result = new CASDefinition(*pTypeSystem);
      assert( EXISTS(result) );
      result->createPredefinedCASTypes();
      result->mergeTypeSystem(crTSDesc, tsName);
      //result->addTypePriorities(casSpecifier);
      pTypeSystem->commit();
      delete result;
      return pTypeSystem;
    }


    uima::lowlevel::TypeSystem * CASDefinition::createTypeSystem(TypeSystemDescription const & crTSDesc,
        AnalysisEngineMetaData::TyVecpTypePriorities const & typePriorities,
        icu::UnicodeString const & tsName) {
      //resolve imports
      if (crTSDesc.isModifiable() ) {
        TypeSystemDescription & tsDesc = CONST_CAST(TypeSystemDescription &, crTSDesc);
        tsDesc.commit();
      }


      uima::lowlevel::TypeSystem * pTypeSystem = new uima::lowlevel::TypeSystem();
      assert( EXISTS(pTypeSystem) );
      CASDefinition * result = new CASDefinition(*pTypeSystem);
      assert( EXISTS(result) );
      result->createPredefinedCASTypes();
      result->mergeTypeSystem(crTSDesc, tsName);
      result->addTypePriorities(typePriorities);
      pTypeSystem->commit();
      delete result;
      return pTypeSystem;
    }


    void CASDefinition::commitTypeSystemOnly() {
      iv_typeSystem->commit();
    }

    CASDefinition::CASDefinition(AnalysisEngineDescription const & taeSpecifier)
        : iv_typeSystem(NULL),
        iv_indexDefinition(NULL),
        iv_annotatorContext(NULL) {

      //resolve imports
      if (taeSpecifier.isModifiable() ) {
        AnalysisEngineDescription & taespec = CONST_CAST(AnalysisEngineDescription &, taeSpecifier);
        taespec.commit();
      }
      iv_typeSystem = new uima::lowlevel::TypeSystem();
      bOwnsTypeSystem = true;
      assert( EXISTS(iv_typeSystem) );
      iv_indexDefinition = new uima::lowlevel::IndexDefinition(*iv_typeSystem);
      assert( EXISTS(iv_indexDefinition) );

    }


    CASDefinition::CASDefinition(uima::TypeSystem  & typeSystem)
        : iv_typeSystem(NULL),
        iv_indexDefinition(NULL),
        iv_annotatorContext(NULL) {
      iv_typeSystem= (uima::lowlevel::TypeSystem*) &typeSystem;
      assert( EXISTS(iv_typeSystem) );
      iv_indexDefinition = new uima::lowlevel::IndexDefinition(*iv_typeSystem);
      assert( EXISTS(iv_indexDefinition) );
      bOwnsTypeSystem = false;
    }


    CASDefinition::CASDefinition()
        : iv_typeSystem(NULL),
        iv_indexDefinition(NULL),
        iv_annotatorContext(NULL) {
      iv_typeSystem = new uima::lowlevel::TypeSystem();
      bOwnsTypeSystem = true;
      assert( EXISTS(iv_typeSystem) );
      iv_indexDefinition = new uima::lowlevel::IndexDefinition(*iv_typeSystem);
      assert( EXISTS(iv_indexDefinition) );

    }


  }
}

