/** \file annotator_aux.hpp .
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

    \brief  Contains AnnotatorLanguageMatch used internally by AnnotatorManager

   Description:

-----------------------------------------------------------------------------


   10/27/1999  Initial creation

-------------------------------------------------------------------------- */
#ifndef UIMA_ANNOTATOR_AUX_HPP
#define UIMA_ANNOTATOR_AUX_HPP

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <uima/annotator.hpp>
#include <uima/language.hpp>

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
//lint -e1714 -e1914 -e1526 -e1931 -e754
  /** we need this class for the STL find_if() procedure */
  class UIMA_LINK_IMPORTSPEC AnnotatorLanguageMatch : public unary_function< AnnotatorFile, bool > {                                                 //lint !e1932
  public:
    AnnotatorLanguageMatch(bool bProcessUnspecifiedTerritories, const Language & crclLanguage);
    bool                       operator() (const AnnotatorLangTargetSets::value_type & crclValue) const;
  private:
    bool                       iv_bProcessUnspecifiedTerritories;
    const Language &      iv_crclLanguage;
    /* --- functions --- */
    /* BASE CONSTRUCTOR NOT SUPPORTED */
    AnnotatorLanguageMatch(void);   //lint !e1704
  private:
    /* ASSIGNMENT OPERATOR NOT SUPPORTED */
    AnnotatorLanguageMatch & operator=(const AnnotatorLanguageMatch & crclObject);
  }
  ;                                                 //lint !e1509 !e1907 !e754

  /** we need this class for the STL find_if() procedure */
  class UIMA_LINK_IMPORTSPEC AnnotatorStrictLanguageMatch : public unary_function< AnnotatorFile, bool > {                                                 //lint !e1932
  public:
    AnnotatorStrictLanguageMatch(bool bProcessUnspecifiedTerritories, const Language & crclLanguage);
    bool                       operator() (const AnnotatorLangTargetSets::value_type & crclValue) const;
  private:
    bool                       iv_bProcessUnspecifiedTerritories;
    const Language &      iv_crclLanguage;
    /* --- functions --- */
    /* BASE CONSTRUCTOR NOT SUPPORTED */
    AnnotatorStrictLanguageMatch(void);   //lint !e1704
  private:
    /* ASSIGNMENT OPERATOR NOT SUPPORTED */
    AnnotatorStrictLanguageMatch & operator=(const AnnotatorStrictLanguageMatch & crclObject);
  }
  ;                                                 //lint !e1509 !e1907 !e754

  /* ----------------------------------------------------------------------- */
  /*       Implementation                                                    */
  /* ----------------------------------------------------------------------- */

  inline AnnotatorLanguageMatch::AnnotatorLanguageMatch(bool bProcessUnspecifiedTerritories,
      const Language & crclLanguage) :
      iv_bProcessUnspecifiedTerritories(bProcessUnspecifiedTerritories),
      iv_crclLanguage(crclLanguage)
      /* ----------------------------------------------------------------------- */
  {                                                     //lint !e1928
    ;
  }

  inline bool AnnotatorLanguageMatch::operator() (const AnnotatorLangTargetSets::value_type & crclValue) const
  /* ----------------------------------------------------------------------- */
  {
    if (!crclValue.first.hasLanguage())                 // Language unspecified?
    {
      return(true);
    }
    if (iv_crclLanguage.getLanguage() == crclValue.first.getLanguage()) {
      if (!iv_bProcessUnspecifiedTerritories) {
#ifndef NDEBUG
        /* code is same as below but allows for better debugging */
        if (iv_crclLanguage.getTerritory() == crclValue.first.getTerritory()) {
          return(true);
        } else {
          return(false);
        }
#else /* no NDEBUG = SHIP */
        return(iv_crclLanguage.getTerritory() == crclValue.first.getTerritory());
#endif
      } else {
        return(true);
      }
    }
    return(false);
  }
  /* ----------------------------------------------------------------------- */
  /*       Implementation                                                    */
  /* ----------------------------------------------------------------------- */

  inline AnnotatorStrictLanguageMatch::AnnotatorStrictLanguageMatch(bool bProcessUnspecifiedTerritories,
      const Language & crclLanguage) :
      iv_bProcessUnspecifiedTerritories(bProcessUnspecifiedTerritories),
      iv_crclLanguage(crclLanguage)
      /* ----------------------------------------------------------------------- */
  {                                                     //lint !e1928
    ;
  }

  inline bool AnnotatorStrictLanguageMatch::operator() (const AnnotatorLangTargetSets::value_type & crclValue) const
  /* ----------------------------------------------------------------------- */
  {
    if (iv_crclLanguage.getLanguage() == crclValue.first.getLanguage()) {
      if (!iv_bProcessUnspecifiedTerritories) {
#ifndef NDEBUG
        /* code is same as below but allows for better debugging */
        if (iv_crclLanguage.getTerritory() == crclValue.first.getTerritory()) {
          return(true);
        } else {
          return(false);
        }
#else /* no NDEBUG = SHIP */
        return(iv_crclLanguage.getTerritory() == crclValue.first.getTerritory());
#endif
      } else {
        return(true);
      }
    }
    return(false);
  }

}
/* ----------------------------------------------------------------------- */
#endif /* UIMA_ANNOTATOR_AUX_HPP */

/* <EOF> */

