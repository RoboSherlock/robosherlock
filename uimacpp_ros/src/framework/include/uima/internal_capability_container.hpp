/** \file internal_capability_container.hpp .
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

    \brief  Contains uima::internal::CapabilityContainer

   Description:

-----------------------------------------------------------------------------


   01/30/2003  Initial creation

-------------------------------------------------------------------------- */
#ifndef UIMA_INTERNAL_CAPABILITY_CONTAINER_HPP
#define UIMA_INTERNAL_CAPABILITY_CONTAINER_HPP

// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include <uima/pragmas.hpp> //must be first to surpress warnings
#include <uima/err_ids.h>
#include <uima/capability.hpp>
#include <uima/type_or_feature.hpp>
#include <uima/language.hpp>

#include <vector>
#include <map>
#include <set>

namespace uima {

  UIMA_EXC_CLASSDECLARE(CapabilityException, Exception);

  namespace internal {

    /**
     * This object represents the capability specification information for
     * a TAE in a way that makes it quickly accessible.
     * It is created from the string based description of the capabilites
     * (see Capability) derived from the XML file.
     * Instead of strings it used type, feature and language objects.
     * The ASB/AnnotatorManager associations one CapabilityContainer with each
     * TAE that is part of a compound TAE to quickly decide if a component TAE
     * should be called for a given language.
     */
    class UIMA_LINK_IMPORTSPEC CapabilityContainer {
    public:
      typedef std::set< TypeOrFeature  > TySetTypeOrFeatures;
      /// Enum to encode the match strategies
      typedef enum {
        enStrictMatchOnly,      /// match language and territory part
        enStrictThenLooseMatch  /// first try to match strict; if no match found try again and match ignoring territory
      }
      EnMatchPolicy;

      CapabilityContainer();
      /**
       * initialize from the string-based description derived from configuration XML file
       */
      void init( std::vector<Capability*> const & vecCap, TypeSystem const & typeSystem, EnMatchPolicy enMatchPolicy = enStrictThenLooseMatch);
      /**
       * Return the container of all output types/features that are defined
       * for a given language lang using match policy enMatchPolicy
       */
      TySetTypeOrFeatures const &
      getInputTypeOrFeatures( Language const & lang ) const;
      /**
       * Return the container of all input types/features that are defined
       * for a given language lang using match policy enMatchPolicy
       */
      TySetTypeOrFeatures const &
      getOutputTypeOrFeatures( Language const & lang) const;
      /**
       * Return true if the outpur type/feature tof is defined
       * for a given language lang using match policy enMatchPolicy
       */
      bool
      hasOutputTypeOrFeature( TypeOrFeature const & tof, Language const & lang) const;
      /**
       * Return true if the input type/feature tof is defined
       * for a given language lang using match policy enMatchPolicy
       */
      bool
      hasInputTypeOrFeature( TypeOrFeature const & tof, Language const & lang) const;
      /**
       * Return true if the neither input types/features nor
       * output types/features are defined
       * for a given language lang using match policy enMatchPolicy
       */
      bool
      hasEmptyOutputTypeOrFeatures( Language const & lang) const;

    protected:
      /* --- functions -------------------------------------------------------- */
      /* --- variables -------------------------------------------------------- */
    private:
      typedef std::map< Language, TySetTypeOrFeatures  > TyMapLang2TypeOrFeatures;
      /* --- functions -------------------------------------------------------- */
      // tool function called during init() to fill map from the strings
      static void
      initTypeOrFeatures(
        TyMapLang2TypeOrFeatures & map, // output argument
        Capability::TyVecCapabilityTofs const & vecTofs,
        Capability::TyVecCapabilityLanguages const & vecLangs,
        TypeSystem const & typeSystem
      );
      // tool function to find a type/feature given a lang and a match policy
      static TySetTypeOrFeatures const *
      findTOFs(TyMapLang2TypeOrFeatures const & map,
               TyMapLang2TypeOrFeatures const & noterritoryMap,
               Language const & lang,
               EnMatchPolicy);
      // tool function to find an input type/feature given a lang and a match policy
      TySetTypeOrFeatures const *
      findInputTOFs(Language const & lang) const;
      // tool function to find an output type/feature given a lang and a match policy
      TySetTypeOrFeatures const *
      findOutputTOFs(Language const & lang) const;

      /* tool function that checks if a special entry for the unspecified
         language is there. If so it copies all entries for the unspecified
         language ("**") to the entries for all languages which have been
         specified directly.
         If someone specifies the capabilities uima.tt.Lemma:en and
         uima.tt.Sentence:** we would naively get the following entries:
         en ->{uima.tt.Lemma} ** -> {uima.tt.Sentence}

         But we really need the following entries:
         en -> {uima.tt.Lemma, uima.tt.Sentence} ** -> {uima.tt.Sentence}
         So we need for each "normal" language the set-union with **
      */
      void copyEntriesForUnspecifedLanguage();

      /*
      perform closure in the sense that capabilities for "full" language codes like
      pt_BR are added to the entries for pt and the like.
      */
      static void computeClosure(TyMapLang2TypeOrFeatures & map, TyMapLang2TypeOrFeatures & noTerritoryMap);


      /* COPY CONSTRUCTOR NOT SUPPORTED */
      CapabilityContainer(CapabilityContainer const &); //lint !e1704
      /* ASSIGNMENT OPERATOR NOT SUPPORTED */
      CapabilityContainer & operator=(CapabilityContainer const &);
      /* --- variables -------------------------------------------------------- */
      TyMapLang2TypeOrFeatures      iv_mapLang2InputTypesOrFeatures;
      TyMapLang2TypeOrFeatures      iv_mapLang2OutputTypesOrFeatures;

      TyMapLang2TypeOrFeatures      iv_mapLang2InputTypesOrFeaturesNoTerritory;
      TyMapLang2TypeOrFeatures      iv_mapLang2OutputTypesOrFeaturesNoTerritory;

      TySetTypeOrFeatures           iv_setEmptyTypeOrFeaturesSet;
      TySetTypeOrFeatures const *   iv_psetInputTOFsForUnspecifiedLang;
      TySetTypeOrFeatures const *   iv_psetOutputTOFsForUnspecifiedLang;
      bool                          iv_bHasEmptyOutputTOFsForUnspecifiedLang;
      EnMatchPolicy iv_enMatchPolicy;

    };



  } // namespace internal
} // namespace uima

#endif // UIMA_INTERNAL_CAPABILITY_CONTAINER_HPP
