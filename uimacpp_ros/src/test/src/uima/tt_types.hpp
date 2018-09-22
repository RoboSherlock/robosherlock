#ifndef UIMA_TT_TYPES_HPP
#define UIMA_TT_TYPES_HPP
/** \file tt_types.hpp .
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

    \brief Contains class uima::TT

   Description: Constants of Types for UIMA/Talent (TT)

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include "uima/pragmas.hpp" // must be first file to be included to get pragmas
#include "uima/types.h"

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

#define AMIU_LINK_IMPORTSPEC

namespace uima {

  /**
   * The class <code>TT</code> contains the constants for the UIMA/Talent converged
   * type hierarchy.
   */
  class TT {
  public:
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * NAME_SPACE_UIMA;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * NAME_SPACE_UIMA_TT;

    /** @addtogroup PreDefTypes Predefined Types
     *  @{
     */
    /** @name TT string constants for the names of predefined types.
     *  @{
     */

    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_TT_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_TOKEN_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_SENTENCE_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_PARAGRAPH_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_KEY_STRING_ENTRY;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_LEMMA;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_CANONICAL_FORM;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_TOPIC_SEGMENT_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_SECTION_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_SYNONYM;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_COMP_PART_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_TERM_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_ABBREVIATION_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_NAME_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_PERSON_NAME_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_PLACE_NAME_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_ORG_NAME_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_MISC_NAME_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_EXPRESSION_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_MONETARY_EXPRESSION_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_DATE_EXPRESSION_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_TIME_EXPRESSION_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_ORDINAL_EXPRESSION_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_CARDINAL_EXPRESSION_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_PERCENT_EXPRESSION_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_LEXICAL_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_SYNTACTIC_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_DOC_STRUCTURE_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_DISCOURSE_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_MARKUP_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_TITLE_MARKUP;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_HEADING_MARKUP;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_LIST_MARKUP;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_ORDERED_LIST_MARKUP;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_UNORDERED_LIST_MARKUP;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_TABLE_MARKUP;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_CAPTION_MARKUP;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_MISC_MARKUP;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_MULTI_TOKEN_ANNOTATION;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_CATEGORY_CONFIDENCE_PAIR;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_LANGUAGE_CONFIDENCE_PAIR;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_RELATION;

    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_UNILEX;
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_CLAUSAL_ANNOTATION;

    /// temp: types for html/xml parsing
    static AMIU_LINK_IMPORTSPEC char const * TYPE_NAME_INTENDED_SUMMARY;

    /** @} */
    /** @} addtogroup*/


    /** @addtogroup PreDefFeatures Predefined Features
     *  @{
     */
    /** @name TT string constants for the names of predefined features.
     *  @{
     */
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_LEMMA_ENTRIES;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_LEMMA_ENTRIES;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_TOKEN_NUMBER;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_TOKEN_NUMBER;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_TOKEN_PROPERTIES;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_TOKEN_PROPERTIES;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_STOPWORD_TOKEN;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_STOPWORD_TOKEN;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_SENTENCE_NUMBER;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_SENTENCE_NUMBER;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_PARAGRAPH_NUMBER;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_PARAGRAPH_NUMBER;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_KEY;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_KEY;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_PART_OF_SPEECH;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_PART_OF_SPEECH;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_MORPH_ID;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_MORPH_ID;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_DOCUMENT_LANGUAGE_AS_UIMA_NBR;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_DOCUMENT_LANGUAGE_AS_UIMA_NBR;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_DOCUMENT_ID;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_DOCUMENT_ID;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_MARKUP_TAG;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_MARKUP_TAG;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_SYNONYM_ENTRIES;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_SYNONYM_ENTRIES;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_PARENT;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_PARENT;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_CHILDREN;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_CHILDREN;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_INFLECTED_FORMS;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_INFLECTED_FORMS;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_SPELL_AID;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_SPELL_AID;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_DOCUMENT_SUMMARY;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_DOCUMENT_SUMMARY;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_DOCUMENT_KEYWORDS;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_DOCUMENT_KEYWORDS;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_POSITIVE_SENTENCE_SCORES_LIST;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_POSITIVE_SENTENCE_SCORES_LIST;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_NEGATIVE_SENTENCE_SCORES_LIST;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_NEGATIVE_SENTENCE_SCORES_LIST;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_POSITIVE_KEYWORD_SCORES_LIST;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_POSITIVE_KEYWORD_SCORES_LIST;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_NEGATIVE_KEYWORD_SCORES_LIST;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_NEGATIVE_KEYWORD_SCORES_LIST;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_DOCUMENT_CATEGORIES;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_DOCUMENT_CATEGORIES;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_CATEGORY_STRING;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_CATEGORY_STRING;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_CATEGORY_CONFIDENCE;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_CATEGORY_CONFIDENCE;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_DOCUMENT_LANGUAGE_CANDIDATES;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_DOCUMENT_LANGUAGE_CANDIDATES;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_LANGUAGE_CONFIDENCE;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_LANGUAGE_CONFIDENCE;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_LANGUAGE_ID;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_LANGUAGE_ID;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_TOPIC_SEGMENT_NUMBER;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_TOPIC_SEGMENT_NUMBER;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_LEX_CANONICAL_FORM;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_LEX_CANONICAL_FORM;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_LEX_CANONICAL_FORM_CONFIDENCE;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_LEX_CANONICAL_FORM_CONFIDENCE;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_CANONICAL_FORM_KIND;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_CANONICAL_FORM_KIND;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_CANONICAL_FORM_FREQUENCY;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_CANONICAL_FORM_FREQUENCY;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_RELATION_NAME;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_RELATION_FREQUENCY;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_RELATION_ARGUMENTS;

    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_UNISYN;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_UNISYN;

    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_UNIMORPH;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_UNIMORPH;

    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_XML_DOCUMENT_META_CONTENT;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_XML_DOCUMENT_META_CONTENT;

    static AMIU_LINK_IMPORTSPEC char const * FEATURE_BASE_NAME_XML_DOCUMENT_META_NAME;
    static AMIU_LINK_IMPORTSPEC char const * FEATURE_FULL_NAME_XML_DOCUMENT_META_NAME;

    /** @} */
    /** @} addtogroup*/


    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * INDEXID_LEMMA;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * INDEXID_SYNONYM;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * INDEXID_CANONICALFORM;
    /// Use this instead of a string literal
    static AMIU_LINK_IMPORTSPEC char const * INDEXID_RELATION;


    static AMIU_LINK_IMPORTSPEC char const * ANNOSFX;

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


/* ----------------------------------------------------------------------- */


#endif

