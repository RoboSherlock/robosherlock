#ifndef UIMA_CONFIG_TOOLS_HPP
#define UIMA_CONFIG_TOOLS_HPP
/** \file config_tools.hpp .
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

    \brief  Contains ConfigBase

   Description:

-----------------------------------------------------------------------------


   Initial creation:

-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp>
#include <uima/strconvert.hpp>  // for string conversion functions
#include <uima/stltools.hpp>
#include <uima/strtools.hpp>
#include <uima/annotator_context.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {



  /**
     A little helper class to provide to tools you need to be able to
     define "constants" in you source code for each config option an
     application/annotator accesses.
  */
  class UIMA_LINK_IMPORTSPEC ConfigOptionInfo {
  public:
    /// Enum to be used with struct <TT>StConfigOptionInfo</TT> below
    enum EnValueType {
      // Integral Number
      enValueType_IntegralNumber,
      // Real Number
      enValueType_RealNumber,
      // String
      enValueType_String,
      // Boolean
      enValueType_Boolean
    };

    /**
       This struct can be used by annotators and applications to keep track of
       all the options they access in the configuration.
       They can create a constant array as a table of of all config options.
       Example:
       \code
       \endcode
    */
    typedef struct StOptionInfo_ {
      const TCHAR        * cpszOptionName;
      EnValueType          enValueType;
      bool                 bOptionIsMultiValued;
      size_t               uiNbrOfValuesRequired;
      const TCHAR        * cpszDefaultValueAsString;
      const TCHAR        * cpszComment;
    }
    StOptionInfo;
  }
  ; // ConfigOptionInfo

  /**
     This function takes an <TT>ConfigOptionInfo::StOptionInfo</TT>
     and uses the information in it to extract the value from the
     config into the output parameter <TT>rclTargetVariable</TT>.
     <TT>crclOptionInfo.bOptionIsMultiValued</TT> is required to be false.
     This function should realy be a (template) member function of class
     <TT>ConfigOptionInfo</TT> above, but our compiler can't do that right now.
  */
  template < class T >
  TyErrorId
  extractConfigOption(
    const AnnotatorContext                     & crclConfig,
    const ConfigOptionInfo::StOptionInfo & crclOptionInfo,
    T                                    & rclTargetVariable
  ) {
    return extractConfigOption(crclConfig, NULL, crclOptionInfo, rclTargetVariable);
    /*       assert(!crclOptionInfo.bOptionIsMultiValued);                                             */
    /*       // assume the worst                                                                       */
    /*       TyErrorId utErrId = UIMA_ERR_CONFIG_SECTION_NOT_FOUND;                                     */
    /*       assert(crclOptionInfo.uiNbrOfValuesRequired <= 1);                                        */
    /*       utErrId = crclConfig.extractValue(crclOptionInfo.cpszOptionName, rclTargetVariable);      */
    /*       if (utErrId != UIMA_ERR_NONE) { // could not find option or value(s)                       */
    /*          if (   crclOptionInfo.uiNbrOfValuesRequired != 0                                       */
    /*                 || crclOptionInfo.cpszDefaultValueAsString == NULL) {                           */
    /*             // required value not there: return error we got from config                        */
    /*             return utErrId;                                                                     */
    /*          }                                                                                      */
    /*          convertFromString((string)crclOptionInfo.cpszDefaultValueAsString, rclTargetVariable); */
    /*          // we used the provided default: this is not an error so we return OK                  */
    /*          utErrId = UIMA_ERR_NONE;                                                                */
    /*       }                                                                                         */
    /*       return utErrId;                                                                           */
  }

  /**
     This function takes an <TT>ConfigOptionInfo::StOptionInfo</TT>
     and uses the information in it to extract the value from the
     config of group <code>crclConfigGroup</code> into the output parameter <TT>rclTargetVariable</TT>.
     If <code>crclConfigGroup == NULL</code>, extracts the default value from the config.
     <TT>crclOptionInfo.bOptionIsMultiValued</TT> is required to be false.
     This function should really be a (template) member function of class
     <TT>ConfigOptionInfo</TT> above, but our compiler can't do that right now.
  */

  template < class T >
  TyErrorId
  extractConfigOption(
    const AnnotatorContext                     & crclConfig,
    const icu::UnicodeString *   cpclConfigGroup,
    const ConfigOptionInfo::StOptionInfo & crclOptionInfo,
    T                                    & rclTargetVariable
  ) {
    assert(!crclOptionInfo.bOptionIsMultiValued);
    // assume the worst
    TyErrorId utErrId = UIMA_ERR_CONFIG_SECTION_NOT_FOUND;
    assert(crclOptionInfo.uiNbrOfValuesRequired <= 1);
    if (EXISTS(cpclConfigGroup)) {
      //utErrId = crclConfig.extractValue(crclOptionInfo.cpszOptionName, rclTargetVariable);
      utErrId = crclConfig.extractValue(*cpclConfigGroup, crclOptionInfo.cpszOptionName, rclTargetVariable);
    } else {
      utErrId = crclConfig.extractValue(crclOptionInfo.cpszOptionName, rclTargetVariable);
    }
    if (utErrId != UIMA_ERR_NONE) { // could not find option or value(s)
      if (   crclOptionInfo.uiNbrOfValuesRequired != 0
             || crclOptionInfo.cpszDefaultValueAsString == NULL) {
        // required value not there: return error we got from config
        return utErrId;
      }
      convertFromString((std::string)crclOptionInfo.cpszDefaultValueAsString, rclTargetVariable);
      // we used the provided default: this is not an error so we return OK
      utErrId = UIMA_ERR_NONE;
    }
    return utErrId;
  }

  /**
     Version of <TT>ExtractConfigOption</TT> for boolean values.
  */
  inline TyErrorId
  extractConfigOptionBoolean(
    const AnnotatorContext                     & crclConfig,
    const ConfigOptionInfo::StOptionInfo & crclOptionInfo,
    bool                                 & rclTargetVariable
  ) {
    assert(crclOptionInfo.enValueType == ConfigOptionInfo::enValueType_Boolean);
//      BoolValue clBool(rclTargetVariable);
    TyErrorId utErrId = extractConfigOption(crclConfig, crclOptionInfo, rclTargetVariable);
//      rclTargetVariable = clBool;
    return utErrId;
  }

  /**
     Version of <TT>ExtractConfigOption</TT> for boolean values.
  */
  inline TyErrorId
  extractConfigOptionBoolean(
    const AnnotatorContext                     & crclConfig,
    const icu::UnicodeString *   cpclConfigGroup ,
    const ConfigOptionInfo::StOptionInfo & crclOptionInfo,
    bool                                 & rclTargetVariable
  ) {
    assert(crclOptionInfo.enValueType == ConfigOptionInfo::enValueType_Boolean);
//      BoolValue clBool(rclTargetVariable);
    TyErrorId utErrId = extractConfigOption(crclConfig, cpclConfigGroup, crclOptionInfo, rclTargetVariable);
//      rclTargetVariable = clBool;
    return utErrId;
  }

  template < class ContainerType, class ElementType >
  TyErrorId
  extractConfigOptionListImpl(
    const AnnotatorContext                     & crclConfig,
    const ConfigOptionInfo::StOptionInfo & crclOptionInfo,
    ContainerType                        & rclTargetContainer,
    ElementType                          * pElem /*not used, just for type information since ContainerType::value_type does not work with HP compiler*/
  ) {
    return extractConfigOptionListImpl(crclConfig, NULL, crclOptionInfo, rclTargetContainer, pElem);
    /*       assert(crclOptionInfo.bOptionIsMultiValued);                                                   */
    /*       TyErrorId utErrId = UIMA_ERR_NONE;                                                              */
    /*       size_t i;                                                                                      */
    /* #if defined(__HPX_ACC__) || defined(__xlC__) || defined(__GNUC__)                                    */
    /*       ElementType tTemp;                                                                             */
    /* #else                                                                                                */
    /*       ContainerType::value_type tTemp;                                                               */
    /* #endif                                                                                               */
    /*                                                                                                      */
    /*       vector<ElementType> elements;                                                                  */
    /*       crclConfig.extractValue(crclOptionInfo.cpszOptionName, elements);                              */
    /*       for (i=0; i<elements.size(); ++i) {                                                            */
    /*          rclTargetContainer.insert(rclTargetContainer.end(), elements[i]);                           */
    /*       }                                                                                              */
    /*                                                                                                      */
    /*       if (utErrId != UIMA_ERR_NONE || (elements.size() == 0 )) { // could not find option or value(s) */
    /*          if (   crclOptionInfo.uiNbrOfValuesRequired != 0                                            */
    /*                 || crclOptionInfo.cpszDefaultValueAsString == NULL) {                                */
    /*             // required value not there: return error we got from config                             */
    /*             return utErrId;                                                                          */
    /*          }                                                                                           */
    /*          vector< string > vecTmpStrings;                                                             */
    /*          delimitedString2Vector(                                                                     */
    /*                                vecTmpStrings,                                                        */
    /*                                (string)crclOptionInfo.cpszDefaultValueAsString,                      */
    /*                                ",",                                                                  */
    /*                                true,   // trim strings                                               */
    /*                                false   // insert empty strings                                       */
    /*                                );                                                                    */
    /*          // our default value too must have the required nbr of values                               */
    /*          assert(vecTmpStrings.size() >= crclOptionInfo.uiNbrOfValuesRequired);                       */
    /*                                                                                                      */
    /*          for (i = 0; i < vecTmpStrings.size(); ++i) {                                                */
    /*             convertFromString(vecTmpStrings[i], tTemp);                                              */
    /*             // assumes rclTargetContainer to be an STL container                                     */
    /*             rclTargetContainer.insert(rclTargetContainer.end(), tTemp);                              */
    /*          }                                                                                           */
    /*       }                                                                                              */
    /*       if (i < crclOptionInfo.uiNbrOfValuesRequired) {                                                */
    /*          taph 8/6/1999: ?? maybe we should have a more precise error id:                          */
    /*             UIMA_ERR_CONFIG_REQUIRED_OPTION_HAS_NOT_ENOUGH_VALUES                                     */
    /*                                                                                                   */
    /*          return UIMA_ERR_CONFIG_REQUIRED_OPTION_IS_EMPTY;                                             */
    /*       }                                                                                              */
    /*       return utErrId;                                                                                */
  }

  template < class ContainerType, class ElementType >
  TyErrorId
  extractConfigOptionListImpl(
    const AnnotatorContext                     & crclConfig,
    const icu::UnicodeString *   cpclConfigGroup,
    const ConfigOptionInfo::StOptionInfo & crclOptionInfo,
    ContainerType                        & rclTargetContainer,
    ElementType                          * /*not used, just for type information since ContainerType::value_type does not work with HP compiler*/
  ) {
    assert(crclOptionInfo.bOptionIsMultiValued);
    TyErrorId utErrId = UIMA_ERR_NONE;
    size_t i;
#if defined(__HPX_ACC__) || defined(__xlC__) || defined(__GNUC__)
    ElementType tTemp;
#else
    ContainerType::value_type tTemp;
#endif

#if defined(__SUNPRO_CC)
    std::vector<ContainerType::value_type> elements;
#else
    std::vector<ElementType> elements;
#endif

    if (EXISTS(cpclConfigGroup)) {
      crclConfig.extractValue(*cpclConfigGroup, crclOptionInfo.cpszOptionName, elements);
    } else {
      crclConfig.extractValue(crclOptionInfo.cpszOptionName, elements);
    }

    for (i=0; i<elements.size(); ++i) {
      rclTargetContainer.insert(rclTargetContainer.end(), elements[i]);
    }

    if (utErrId != UIMA_ERR_NONE || (elements.size() == 0 )) { // could not find option or value(s)
      if (   crclOptionInfo.uiNbrOfValuesRequired != 0
             || crclOptionInfo.cpszDefaultValueAsString == NULL) {
        // required value not there: return error we got from config
        return utErrId;
      }
      std::vector< std::string > vecTmpStrings;
      delimitedString2Vector(
        vecTmpStrings,
        (std::string)crclOptionInfo.cpszDefaultValueAsString,
        ",",
        true,   // trim strings
        false   // insert empty strings
      );
      // our default value too must have the required nbr of values
      assert(vecTmpStrings.size() >= crclOptionInfo.uiNbrOfValuesRequired);

      for (i = 0; i < vecTmpStrings.size(); ++i) {
        convertFromString(vecTmpStrings[i], tTemp);
        // assumes rclTargetContainer to be an STL container
        rclTargetContainer.insert(rclTargetContainer.end(), tTemp);
      }
    }
    if (i < crclOptionInfo.uiNbrOfValuesRequired) {
      /* taph 8/6/1999: ?? maybe we should have a more precise error id:
         UIMA_ERR_CONFIG_REQUIRED_OPTION_HAS_NOT_ENOUGH_VALUES
      */
      return UIMA_ERR_CONFIG_REQUIRED_OPTION_IS_EMPTY;
    }
    return utErrId;
  }

  /**
     This function takes an <TT>ConfigOptionInfo::StOptionInfo</TT>
     and uses the information in it to extract multiple values from the
     config into the output parameter <TT>rclTargetContainer</TT>.
     This works for config values with more than one value.
     <TT>rclTargetContainer</TT> is assumed to be an STL container.
     <TT>crclOptionInfo.bOptionIsMultiValued</TT> is required to be true.
     This function should realy be a (template) member function of class
     <TT>ConfigOptionInfo</TT> above, but our compiler can't do that right now.
  */
  template < class ContainerType >
  inline TyErrorId
  extractConfigOptionList(
    const AnnotatorContext                     & crclConfig,
    const ConfigOptionInfo::StOptionInfo & crclOptionInfo,
    ContainerType                        & rclTargetContainer
  ) {
    return extractConfigOptionListImpl(crclConfig, crclOptionInfo, rclTargetContainer, STL_ITER2PTR(rclTargetContainer.begin()));
  }

  /**
     This function takes an <TT>ConfigOptionInfo::StOptionInfo</TT>
     and uses the information in it to extract multiple values from the
     config of group <code>crclConfigGroup</code> into the output parameter <TT>rclTargetContainer</TT>.
     This works for config values with more than one value.
     <TT>rclTargetContainer</TT> is assumed to be an STL container.
     <TT>crclOptionInfo.bOptionIsMultiValued</TT> is required to be true.
     This function should realy be a (template) member function of class
     <TT>ConfigOptionInfo</TT> above, but our compiler can't do that right now.
  */
  template < class ContainerType >
  inline TyErrorId
  extractConfigOptionList(
    const AnnotatorContext                     & crclConfig,
    const icu::UnicodeString *   cpclConfigGroup,
    const ConfigOptionInfo::StOptionInfo & crclOptionInfo,
    ContainerType                        & rclTargetContainer
  ) {
    return extractConfigOptionListImpl(crclConfig, cpclConfigGroup, crclOptionInfo, rclTargetContainer, STL_ITER2PTR(rclTargetContainer.begin()));
  }

}

#endif /* UIMA_CONFIG_TOOLS_HPP */
/* <EOF> */

