/** \file internal_capability_container.cpp .
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


   02/05/2003  Initial creation

-------------------------------------------------------------------------- */
//#define DEBUG_VERBOSE
// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------
#include <uima/pragmas.hpp>
using namespace std;


#include <uima/macros.h>
#include <uima/internal_capability_container.hpp>
#include <uima/err_ids.h>
#include <uima/msg.h>

namespace uima {

  UIMA_EXC_CLASSIMPLEMENT(CapabilityException, Exception);

  namespace internal {


    CapabilityContainer::CapabilityContainer( )
        :  iv_psetInputTOFsForUnspecifiedLang(NULL),
        iv_psetOutputTOFsForUnspecifiedLang(NULL),
        iv_bHasEmptyOutputTOFsForUnspecifiedLang(false) {}



    /*static*/
    void
    CapabilityContainer::initTypeOrFeatures(
      TyMapLang2TypeOrFeatures & rMap, // output argument
      Capability::TyVecCapabilityTofs const & vecTofs,
      Capability::TyVecCapabilityLanguages const & vecLangs,
      TypeSystem const & typeSystem) {
      const int BUF_SIZE = 0x100;
      char  cBuf[BUF_SIZE];
      Capability::TyVecCapabilityLanguages::const_iterator itLang;
      for (itLang = vecLangs.begin(); itLang != vecLangs.end(); ++itLang) {
        // convert unicode lang string to single-byte lang string
        assert((*itLang).length() < BUF_SIZE);
        // Note: this conversion can be used only for invariant characters
        u_UCharsToChars((*itLang).getBuffer(), cBuf, (*itLang).length());
        // zero terminate single-byte buffer
        cBuf[(*itLang).length()] = '\0';
        // Special workaround code for the new way to specify the unspecified language
        if (strcasecmp(cBuf, "x-unspecified") == 0) {
          strcpy(cBuf, Language::UNSPECIFIED);
        }
        // create a language object based on single-byte lang string
        Language lang(cBuf);
        if (!lang.isValid()) {
          /* taph 06.02.2003: once we have more detailed information about
          the origin of the error we need to replace "unknown configuration file"
          with the filename of the XML file (and maybe line number?) */
          UIMA_EXC_THROW_NEW(uima::CapabilityException,          // exc-type
                             UIMA_ERR_ENGINE_LANGUAGE_INVALID,   // error code
                             ErrorMessage(UIMA_MSG_ID_EXC_INVALID_LANGUAGE, cBuf), // error message
                             ErrorMessage(UIMA_MSG_ID_EXCON_CHECKING_CAPABILITY_SPEC,"unknown configuration file"),  // error context message
                             ErrorInfo::recoverable);
        }
        // create a new empty vector in the map for this lang
        TySetTypeOrFeatures & setTofs = rMap[lang];
        // now fill the vector with tof objects created from tof strings
        Capability::TyVecCapabilityTofs::const_iterator itTof;
        for (itTof = vecTofs.begin(); itTof != vecTofs.end(); ++itTof) {
          TypeOrFeature tof;
          // the tof string may be a type...
          Type t = typeSystem.getType(*itTof);
          if (t.isValid()) {
            tof = TypeOrFeature(t);
          } else {
            // or the tof string may be a feature
            Feature f = typeSystem.getFeatureByFullName(*itTof);
            if (f.isValid()) {
              tof = TypeOrFeature(f);
            } else {
              /* taph 06.02.2003: once we have more detailed information about
              the origin of the error we need to replace "unknown configuration file"
              with the filename of the XML file (and maybe line number?) */
              if (tof.isType()) {
                UIMA_EXC_THROW_NEW(uima::CapabilityException,        // exc-type
                                   UIMA_ERR_INVALID_FSTYPE_OBJECT,   // error code
                                   ErrorMessage(UIMA_MSG_ID_EXC_UNKNOWN_TYPE_NAME, *itTof), // error message
                                   ErrorMessage(UIMA_MSG_ID_EXCON_CHECKING_CAPABILITY_SPEC, "unknown configuration file"),  // error context message
                                   ErrorInfo::recoverable);
              } else {
                UIMA_EXC_THROW_NEW(uima::CapabilityException,        // exc-type
                                   UIMA_ERR_INVALID_FSFEATURE_OBJECT,// error code
                                   ErrorMessage(UIMA_MSG_ID_EXC_UNKNOWN_FEATURE_NAME, *itTof), // error message
                                   ErrorMessage(UIMA_MSG_ID_EXCON_CHECKING_CAPABILITY_SPEC, "unknown configuration file"),  // error context message
                                   ErrorInfo::recoverable);
              }
            }
          }
          assert(tof.isValid());
          setTofs.insert(tof);
        } // for all tof strings
      } // for all lang strings
    }


    void CapabilityContainer::computeClosure(TyMapLang2TypeOrFeatures & rMap, TyMapLang2TypeOrFeatures & noTerritoryMap) {
      assert( noTerritoryMap.empty() );
      // 1. all original TOFS from iv_mapLang2*TypesOrFeatures for language codes without territory
      // are copied as iv_mapLang2*TypesOrFeaturesNoTerritory
      TyMapLang2TypeOrFeatures::const_iterator cit;
      for (cit = rMap.begin(); cit != rMap.end(); ++cit) {
        Language const & lang = (*cit).first;
        if ( !lang.hasTerritory() ) {
          noTerritoryMap[lang] = (*cit).second;
        }
      }

      // 2. all TOFs from en are appended to en_US, en_GB etc. for iv_mapLang2*TypesOrFeatures
      TyMapLang2TypeOrFeatures::iterator it;
      for (it = rMap.begin(); it != rMap.end(); ++it) {
        Language const & lang = (*it).first;
        if ( lang.hasTerritory() ) {
          // put all TOFs "down" from pt to pt_BR
          Language langWithoutTerritory(lang.getLanguage());
          TySetTypeOrFeatures & rSet = rMap[langWithoutTerritory];
          TySetTypeOrFeatures & specificSet = (*it).second;
          // MSV6 compile error: specificSet.insert( rSet.begin(), rSet.end() );
          TySetTypeOrFeatures::const_iterator setit;
          for (setit = rSet.begin(); setit != rSet.end(); ++setit) {
            specificSet.insert( *setit );
          }
        }
      }

      // 3. all TOFs from en_GB, en_US, etc. are appended to en for iv_mapLang2*TypesOrFeatures
      for (it = rMap.begin(); it != rMap.end(); ++it) {
        Language const & lang = (*it).first;
        if ( lang.hasTerritory() ) {
          // put all TOFs "up" from pt_BR to pt
          Language langWithoutTerritory(lang.getLanguage());
          TySetTypeOrFeatures & rSet = (*it).second;
          // MSV6 compile error:       rMap[langWithoutTerritory].insert( rSet.begin(), rSet.end() );
          TySetTypeOrFeatures::const_iterator setit;
          for (setit = rSet.begin(); setit != rSet.end(); ++setit) {
            rMap[langWithoutTerritory].insert(*setit);
          }

        }
      }

#ifdef DEBUG_VERBOSE
      UIMA_TPRINT("Standard Map:");
      for (cit = rMap.begin(); cit != rMap.end(); ++cit) {
        UIMA_TPRINT("Language: " << (*cit).first);
        UIMA_TPRINT("   TOFs:");
        TySetTypeOrFeatures const & set = (*cit).second;
        TySetTypeOrFeatures::const_iterator setcit;
        for (setcit = set.begin(); setcit != set.end(); ++setcit) {
          UIMA_TPRINT("       " << (*setcit).getName());
        }
      }

#endif

    }


    void
    CapabilityContainer::init( vector<Capability*> const & vecCap, TypeSystem const & typeSystem, EnMatchPolicy enMatchPolicy ) {
      iv_enMatchPolicy = enMatchPolicy;
      vector<Capability*>::const_iterator it;
      assert(iv_mapLang2InputTypesOrFeatures.size() == 0);
      assert(iv_mapLang2OutputTypesOrFeatures.size() == 0);
      for (it = vecCap.begin(); it != vecCap.end(); ++it) {
        assert(EXISTS(*it));
        Capability const & cap = (*(*it));
        initTypeOrFeatures(
          iv_mapLang2InputTypesOrFeatures,
          cap.getCapabilityTypes(Capability::INPUT),
          cap.getSupportedLanguages(),
          typeSystem);
        initTypeOrFeatures(
          iv_mapLang2InputTypesOrFeatures,
          cap.getCapabilityFeatures(Capability::INPUT),
          cap.getSupportedLanguages(),
          typeSystem);
        initTypeOrFeatures(
          iv_mapLang2OutputTypesOrFeatures,
          cap.getCapabilityTypes(Capability::OUTPUT),
          cap.getSupportedLanguages(),
          typeSystem);
        initTypeOrFeatures(
          iv_mapLang2OutputTypesOrFeatures,
          cap.getCapabilityFeatures(Capability::OUTPUT),
          cap.getSupportedLanguages(),
          typeSystem);
      }

      // try to find input tofs for unspecifed language
      assert(iv_psetInputTOFsForUnspecifiedLang == NULL);
      TySetTypeOrFeatures const * pSetInputTOFs =
        findInputTOFs(Language(Language::UNSPECIFIED));
      if (pSetInputTOFs != NULL) {
        iv_psetInputTOFsForUnspecifiedLang = pSetInputTOFs;
      }
      // try to find output tofs for unspecifed language
      assert(iv_psetOutputTOFsForUnspecifiedLang == NULL);
      TySetTypeOrFeatures const * pSetOutputTOFs =
        findOutputTOFs(Language(Language::UNSPECIFIED));
      if (pSetOutputTOFs != NULL) {
        iv_psetOutputTOFsForUnspecifiedLang = pSetOutputTOFs;
      }

      copyEntriesForUnspecifedLanguage();
      if (hasEmptyOutputTypeOrFeatures(Language(Language::UNSPECIFIED))) {
        iv_bHasEmptyOutputTOFsForUnspecifiedLang = true;
      }

      computeClosure(iv_mapLang2OutputTypesOrFeatures, iv_mapLang2OutputTypesOrFeaturesNoTerritory );
      computeClosure(iv_mapLang2InputTypesOrFeatures, iv_mapLang2InputTypesOrFeaturesNoTerritory );
    }

    void CapabilityContainer::copyEntriesForUnspecifedLanguage() {
      if (iv_psetInputTOFsForUnspecifiedLang == NULL) {
        // if it is not in the input it can't be in the output either
        assert(findOutputTOFs(Language(Language::UNSPECIFIED)) == NULL);
        return;
      }
      assert(EXISTS(iv_psetInputTOFsForUnspecifiedLang));
      if (iv_psetInputTOFsForUnspecifiedLang->size() != 0) {
        TyMapLang2TypeOrFeatures::iterator it;
        for (it = iv_mapLang2InputTypesOrFeatures.begin(); it != iv_mapLang2InputTypesOrFeatures.end(); ++it) {
          if (&(*it).second != iv_psetInputTOFsForUnspecifiedLang) { // don't append onto ourselves
            if ((*it).second.size() > 0) { // don't append to empty
#ifndef NDEBUG
              size_t uiOldSize = (*it).second.size();
#endif /* NDEBUG */
              // MSV6 compile error: (*it).second.insert(iv_psetInputTOFsForUnspecifiedLang->begin(), iv_psetInputTOFsForUnspecifiedLang->end());
              TySetTypeOrFeatures::const_iterator tofit;
              for (tofit = iv_psetInputTOFsForUnspecifiedLang->begin(); tofit != iv_psetInputTOFsForUnspecifiedLang->end(); ++tofit) {
                (*it).second.insert( *tofit );
              }
              assert((*it).second.size() >= uiOldSize);
            }
          }
        }

        // Now do the same copying for the output

        // if it is in the input it must be in the output too
        assert(findOutputTOFs(Language(Language::UNSPECIFIED)) != NULL);
        assert(EXISTS(iv_psetOutputTOFsForUnspecifiedLang));

        for (it = iv_mapLang2OutputTypesOrFeatures.begin(); it != iv_mapLang2OutputTypesOrFeatures.end(); ++it) {
          if (&(*it).second != iv_psetOutputTOFsForUnspecifiedLang) { // don't append onto ourselves
            if ((*it).second.size() > 0) { // don't append to empty
#ifndef NDEBUG
              size_t uiOldSize = (*it).second.size();
#endif /* NDEBUG */
              // MSV6 compile error: (*it).second.insert(iv_psetOutputTOFsForUnspecifiedLang->begin(), iv_psetOutputTOFsForUnspecifiedLang->end());
              TySetTypeOrFeatures::const_iterator tofit;
              for (tofit = iv_psetOutputTOFsForUnspecifiedLang->begin(); tofit != iv_psetOutputTOFsForUnspecifiedLang->end(); ++tofit) {
                (*it).second.insert(*tofit);
              }
              assert((*it).second.size() >= uiOldSize);
            }
          }
        }
      }
    }


    /*static*/ CapabilityContainer::TySetTypeOrFeatures const *
    CapabilityContainer::findTOFs(TyMapLang2TypeOrFeatures const & crMap,
                                  TyMapLang2TypeOrFeatures const & noTerritoryMap,
                                  Language const & lang,  EnMatchPolicy enMatchPolicy) {
      // temporary
      assert( enMatchPolicy == enStrictThenLooseMatch );
      //first try strict match using built-in find function
      TyMapLang2TypeOrFeatures::const_iterator itMap = crMap.find(lang);
      if (itMap != crMap.end()) {
        return (&(*itMap).second);
      }

      if (enMatchPolicy == enStrictMatchOnly) {
        return NULL;
      }
      assert(enMatchPolicy == enStrictThenLooseMatch);
      // language was not found directly
      // now try without territory
      Language newLang(lang.getLanguage());
      assert( !newLang.hasTerritory() );
      for (itMap = noTerritoryMap.begin(); itMap != noTerritoryMap.end(); ++itMap) {
        assert( !(*itMap).first.hasTerritory() );
        if ((*itMap).first.getLanguage() == lang.getLanguage()) {
          return (&(*itMap).second);
        }
      }
      return NULL;
    }

    CapabilityContainer::TySetTypeOrFeatures const *
    CapabilityContainer::findInputTOFs(Language const & lang) const {
      // try to find the set for the language
      TySetTypeOrFeatures const * pSetTOFs =
        findTOFs(iv_mapLang2InputTypesOrFeatures, iv_mapLang2InputTypesOrFeaturesNoTerritory, lang, iv_enMatchPolicy);
      // if there is one return that one
      if (pSetTOFs != NULL) {
        return pSetTOFs;
      }
      // else return whatever we may have for the unspecified language
      return (iv_psetInputTOFsForUnspecifiedLang);
    }

    CapabilityContainer::TySetTypeOrFeatures const *
    CapabilityContainer::findOutputTOFs(Language const & lang) const {
      // try to find the set for the language
      TySetTypeOrFeatures const * pSetTOFs =
        findTOFs(iv_mapLang2OutputTypesOrFeatures, iv_mapLang2OutputTypesOrFeaturesNoTerritory, lang, iv_enMatchPolicy);
      // if there is one return that one
      if (pSetTOFs != NULL) {
        return pSetTOFs;
      }
      // else return whatever we may have for the unspecified language
      return (iv_psetOutputTOFsForUnspecifiedLang);
    }


    CapabilityContainer::TySetTypeOrFeatures const &
    CapabilityContainer::getInputTypeOrFeatures( Language const & lang) const {
      // try to find it
      TySetTypeOrFeatures const * pSetTOFs = findInputTOFs(lang);
      if (pSetTOFs != NULL) {
        return (*pSetTOFs);
      }
      // don't use the [] operator because would insert and return a
      // newly created empty vector
      assert(iv_setEmptyTypeOrFeaturesSet.size() == 0);
      return iv_setEmptyTypeOrFeaturesSet;
    }

    bool
    CapabilityContainer::hasInputTypeOrFeature( TypeOrFeature const & tof, Language const & lang) const {
      // try to find it
      TySetTypeOrFeatures const * pSetTOFs = findInputTOFs(lang);
      if (pSetTOFs == NULL) {
        return false;
      }
      TySetTypeOrFeatures::const_iterator itSet = pSetTOFs->find(tof);
      if (itSet == pSetTOFs->end()) {
        return false;
      }
      return true;
    }

    CapabilityContainer::TySetTypeOrFeatures const &
    CapabilityContainer::getOutputTypeOrFeatures( Language const & lang) const {
      // try to find it
      TySetTypeOrFeatures const * pSetTOFs = findOutputTOFs(lang);
      if (pSetTOFs != NULL) {
        return (*pSetTOFs);
      }
      // don't use the [] operator because will insert and return a
      // newly created empty vector
      assert(iv_setEmptyTypeOrFeaturesSet.size() == 0);
      return iv_setEmptyTypeOrFeaturesSet;
    }

    bool
    CapabilityContainer::hasOutputTypeOrFeature( TypeOrFeature const & tof, Language const & lang) const {
      // try to find it
      TySetTypeOrFeatures const * pSetTOFs = findOutputTOFs(lang);
      if (pSetTOFs == NULL) {
        return false;
      }
      TySetTypeOrFeatures::const_iterator itSet = pSetTOFs->find(tof);
      if (itSet == pSetTOFs->end()) {
        return false;
      }
      return true;
    }

    bool
    CapabilityContainer::hasEmptyOutputTypeOrFeatures( Language const & lang) const {
      // try to find it in output
      TySetTypeOrFeatures const * pSetTOFs = findOutputTOFs(lang);
      if (pSetTOFs == NULL) {
        // we must have an explicit entry for that language in our map
        // or maybe we have an empty entry for the unspecifed lang
        return iv_bHasEmptyOutputTOFsForUnspecifiedLang;
      }
      // but that explicit entry must be empty
      if (pSetTOFs->size() > 0) {
        return false;
      }
      return true;

    }

  } // namespace internal
} // namespace uima


