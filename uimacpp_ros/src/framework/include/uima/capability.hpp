/** \file capability.hpp .
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

    \brief  Contains class uima::Capability

   Description: A Capability object defines the capabilities of a TextAnalysisEngine

-----------------------------------------------------------------------------


   01/30/2003  Initial creation

-------------------------------------------------------------------------- */
#ifndef UIMA_CAPABILITY_HPP
#define UIMA_CAPABILITY_HPP

// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include <uima/pragmas.hpp> //must be first to surpress warnings
#include <uima/err_ids.h>

#include <uima/taemetadata.hpp>
#include <vector>

namespace uima {

  /**
  * Defines the capability of a <code>TextAnalysisEngine</code> in terms of the required input and output
  * types and features and the supported languages. A <code>TextAnalysisEngine</code> can be described 
  * by more than one <code>Capability</code>, for example, 
  * if the input and output types/features differ, depending on
  * the language.
  * The Capability is expressed in terms of input and output type, feature and Sofa names.
  **/
  class UIMA_LINK_IMPORTSPEC Capability :public MetaDataObject {
  public:
    typedef std::vector <icu::UnicodeString > TyVecCapabilityTofs;
    typedef std::vector <icu::UnicodeString > TyVecCapabilityLanguages;
    typedef std::vector <icu::UnicodeString > TyVecCapabilitySofas;

    enum EnTypeStyle {
      INPUT, OUTPUT, INPUTSOFA, OUTPUTSOFA
    };

    Capability()
        :MetaDataObject(), iv_inputFeatures(), iv_inputTypes(), iv_outputFeatures(),
        iv_outputTypes(), iv_supportedLanguages() {}

    /**
    * @param type The name of the type to be added
    * @param typeStyle Determines whether it's an input or output type
    **/
    TyErrorId addCapabilityType(const icu::UnicodeString & type, EnTypeStyle typeStyle) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      switch (typeStyle) {
      case INPUT:
        iv_inputTypes.push_back(type);
        break;
      case OUTPUT:
        iv_outputTypes.push_back(type);
        break;
      default:
        //TBD: error handling
        std::cerr << "Wrong output type " << typeStyle << std::endl;
        break;
      }

      return UIMA_ERR_NONE;
    }

    /**
    * @param feature The name of the feature to be added
    * @param typeStyle Determines whether it's an input or output feature
    **/
    TyErrorId addCapabilityFeature(const icu::UnicodeString & feature, EnTypeStyle typeStyle) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      switch (typeStyle) {
      case INPUT:
        iv_inputFeatures.push_back(feature);
        break;
      case OUTPUT:
        iv_outputFeatures.push_back(feature);
        break;
      default:
        //TBD: error handling
        assert(false);
        break;
      }
      return UIMA_ERR_NONE;
    }



    /**
    * Set the input or output Sofa name
    * @param sofa The name of the sofa to be added
    * @param typeStyle Determines whether it's an input or output sofa
    **/
    TyErrorId addCapabilitySofa(const icu::UnicodeString & sofa, EnTypeStyle typeStyle) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      switch (typeStyle) {
      case INPUTSOFA:
        iv_inputSofas.push_back(sofa);
        break;
      case OUTPUTSOFA:
        iv_outputSofas.push_back(sofa);
        break;
      default:
        //TBD: error handling
        assert(false);
        break;
      }
      return UIMA_ERR_NONE;
    }


    /**
     * Get input/output Types
     */
    const TyVecCapabilityTofs & getCapabilityTypes(EnTypeStyle typeStyle) const {
      switch (typeStyle) {
      case INPUT:
        return iv_inputTypes;
        break;
      case OUTPUT:
        return iv_outputTypes;
        break;
      default:
        assert(false);
        return iv_inputTypes;
      }
    }


    /**
     * Get the input/output features
     */
    const TyVecCapabilityTofs & getCapabilityFeatures(EnTypeStyle typeStyle) const {
      switch (typeStyle) {
      case INPUT:
        return iv_inputFeatures;
        break;
      case OUTPUT:
        return iv_outputFeatures;
        break;
      default:
        assert(false);
        return iv_inputFeatures;
      }
    }

    /**
     * Get the input/output Sofas
     */
    const TyVecCapabilitySofas & getCapabilitySofas(EnTypeStyle typeStyle) const {
      switch (typeStyle) {
      case INPUTSOFA:
        return iv_inputSofas;
        break;
      case OUTPUTSOFA:
        return iv_outputSofas;
        break;
      default:
        assert(false);
        return iv_inputSofas;
      }
    }



    TyErrorId addSupportedLanguage(const icu::UnicodeString & language) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_supportedLanguages.push_back(language);
      return UIMA_ERR_NONE;
    }

    const TyVecCapabilityLanguages & getSupportedLanguages() const {
      return iv_supportedLanguages;
    }


  private:
    /*       Capability(const Capability & crOther); */
    Capability & operator=(const Capability & crOther);

    TyVecCapabilityTofs iv_inputFeatures;
    TyVecCapabilityTofs iv_inputTypes;
    TyVecCapabilityTofs iv_outputFeatures;
    TyVecCapabilityTofs iv_outputTypes;
    TyVecCapabilityLanguages iv_supportedLanguages;
    TyVecCapabilitySofas iv_inputSofas;
    TyVecCapabilitySofas iv_outputSofas;

  };
}

#endif
