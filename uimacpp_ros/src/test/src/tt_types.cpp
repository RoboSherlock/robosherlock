/** \file tt_types.cpp .

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

-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include "uima/tt_types.hpp"
#include "uima/cas.hpp"

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */
#define UIMA_NAMESPACE "uima"
#define UIMA_TT_NAMESPACE UIMA_NAMESPACE UIMA_NAMESPACE_SEPARATOR "tt"
#define UIMA_TT_PFX UIMA_TT_NAMESPACE UIMA_NAMESPACE_SEPARATOR

// type defines
#define DOCUMENT_ANNOTATION "uima.tcas.DocumentAnnotation"
#define DOC_STRUCTURE_ANNOTATION UIMA_TT_PFX "DocStructureAnnotation"
#define KEY_STRING_ENTRY UIMA_TT_PFX "KeyStringEntry"
#define LEXICAL_ANNOTATION UIMA_TT_PFX "LexicalAnnotation"
#define LANGUAGE_CONFIDENCE_PAIR UIMA_TT_PFX "LanguageConfidencePair"
#define TOKEN_ANNOTATION UIMA_TT_PFX "TokenAnnotation"
#define SENTENCE_ANNOTATION UIMA_TT_PFX "SentenceAnnotation"
#define PARAGRAPH_ANNOTATION UIMA_TT_PFX "ParagraphAnnotation"
#define LEMMA UIMA_TT_PFX "Lemma"
#define CANONICAL_FORM UIMA_TT_PFX "CanonicalForm"
#define CATEGORY_CONFIDENCE_PAIR UIMA_TT_PFX "CategoryConfidencePair"
#define TT_ANNOTATION UIMA_TT_PFX "TTAnnotation"

// feature defines
#define DOCUMENT_ID "id"
#define MARKUP_TAG "markupTag"
#define DOCUMENT_CATEGORIES "categories"
#define KEY "key"
#define LEX_CANONICAL_FORM "lexCanonicalForm"
#define LEX_CANONICAL_FORM_CONFIDENCE "lexCanonicalFormConfidence"
#define DOCUMENT_LANGUAGE "language"
#define DOCUMENT_LANGUAGE_AS_UIMA_NBR "languageAsUIMANbr"
#define DOCUMENT_LANGUAGE_CANDIDATES "languageCandidates"
#define LANGUAGE_CONFIDENCE "languageConfidence"
#define LANGUAGE_ID "languageID"
#define INFLECTED_FORMS "inflectedForms"
#define SPELL_AID "spellAid"
#define POSITIVE_SENTENCE_SCORES_LIST "positiveSentenceScoresList"
#define NEGATIVE_SENTENCE_SCORES_LIST "negativeSentenceScoresList"
#define POSITIVE_KEYWORD_SCORES_LIST "positiveKeywordScoresList"
#define NEGATIVE_KEYWORD_SCORES_LIST "negativeKeywordScoresList"
#define DOCUMENT_KEYWORDS "keywords"
#define DOCUMENT_SUMMARY "summary"
#define PART_OF_SPEECH "partOfSpeech"
#define LEMMA_ENTRIES "lemmaEntries"
#define CANONICAL_FORM_FREQUENCY "canonicalFormFrequency"
#define STOPWORD_TOKEN "stopwordToken"
#define CATEGORY_CONFIDENCE "categoryConfidence"
#define CATEGORY_STRING "categoryString"
#define SENTENCE_NUMBER "sentenceNumber"
#define CANONICAL_FORM_KIND "canonicalFormKind"
#define UNISYN "uniSyn"
#define UNIMORPH "uniMorph"
#define MORPHID "morphID"
#define TOPIC_SEGMENT_ANNOTATION UIMA_TT_PFX "TopicSegmentAnnotation"


namespace uima {
  char const * TT::NAME_SPACE_UIMA = UIMA_NAMESPACE;
  char const * TT::NAME_SPACE_UIMA_TT = UIMA_TT_NAMESPACE;

  char const * TT::TYPE_NAME_TT_ANNOTATION           = TT_ANNOTATION;
  char const * TT::TYPE_NAME_TOKEN_ANNOTATION        = TOKEN_ANNOTATION;
  char const * TT::TYPE_NAME_SENTENCE_ANNOTATION     = SENTENCE_ANNOTATION;
  char const * TT::TYPE_NAME_PARAGRAPH_ANNOTATION    = PARAGRAPH_ANNOTATION;
  char const * TT::TYPE_NAME_KEY_STRING_ENTRY        = KEY_STRING_ENTRY;
  char const * TT::TYPE_NAME_LEMMA                   = LEMMA;
  char const * TT::TYPE_NAME_TOPIC_SEGMENT_ANNOTATION= TOPIC_SEGMENT_ANNOTATION;
  char const * TT::TYPE_NAME_CANONICAL_FORM          = CANONICAL_FORM;
  char const * TT::TYPE_NAME_TERM_ANNOTATION         = UIMA_TT_PFX "TermAnnotation";
  char const * TT::TYPE_NAME_ABBREVIATION_ANNOTATION = UIMA_TT_PFX "AbbreviationAnnotation";
  char const * TT::TYPE_NAME_NAME_ANNOTATION         = UIMA_TT_PFX "NameAnnotation";
  char const * TT::TYPE_NAME_PERSON_NAME_ANNOTATION  = UIMA_TT_PFX "PersonName";
  char const * TT::TYPE_NAME_PLACE_NAME_ANNOTATION   = UIMA_TT_PFX "PlaceName";
  char const * TT::TYPE_NAME_ORG_NAME_ANNOTATION     = UIMA_TT_PFX "OrgName";
  char const * TT::TYPE_NAME_MISC_NAME_ANNOTATION    = UIMA_TT_PFX "MiscName";
  char const * TT::TYPE_NAME_EXPRESSION_ANNOTATION   = UIMA_TT_PFX "ExpressionAnnotation";
  char const * TT::TYPE_NAME_MONETARY_EXPRESSION_ANNOTATION = UIMA_TT_PFX "MonetaryExpression";
  char const * TT::TYPE_NAME_DATE_EXPRESSION_ANNOTATION = UIMA_TT_PFX "DateExpression";
  char const * TT::TYPE_NAME_TIME_EXPRESSION_ANNOTATION = UIMA_TT_PFX "TimeExpression";
  char const * TT::TYPE_NAME_ORDINAL_EXPRESSION_ANNOTATION  = UIMA_TT_PFX "OrdinalExpression";
  char const * TT::TYPE_NAME_CARDINAL_EXPRESSION_ANNOTATION = UIMA_TT_PFX "CardinalExpression";
  char const * TT::TYPE_NAME_PERCENT_EXPRESSION_ANNOTATION = UIMA_TT_PFX "PercentExpression";
  char const * TT::TYPE_NAME_SECTION_ANNOTATION     = UIMA_TT_PFX "SectionAnnotation";
  char const * TT::TYPE_NAME_SYNONYM               = UIMA_TT_PFX "Synonym";
  char const * TT::TYPE_NAME_COMP_PART_ANNOTATION    = UIMA_TT_PFX "CompPartAnnotation";
  char const * TT::TYPE_NAME_LEXICAL_ANNOTATION     = LEXICAL_ANNOTATION;
  char const * TT::TYPE_NAME_SYNTACTIC_ANNOTATION   = UIMA_TT_PFX "SyntacticAnnotation";
  char const * TT::TYPE_NAME_DOC_STRUCTURE_ANNOTATION = DOC_STRUCTURE_ANNOTATION;
  char const * TT::TYPE_NAME_DISCOURSE_ANNOTATION   = UIMA_TT_PFX "DiscourseAnnotation";
  char const * TT::TYPE_NAME_MARKUP_ANNOTATION      = UIMA_TT_PFX "MarkupAnnotation";
  char const * TT::TYPE_NAME_TITLE_MARKUP           = UIMA_TT_PFX "TitleMarkup";
  char const * TT::TYPE_NAME_HEADING_MARKUP         = UIMA_TT_PFX "HeadingMarkup";
  char const * TT::TYPE_NAME_LIST_MARKUP            = UIMA_TT_PFX "ListMarkup";
  char const * TT::TYPE_NAME_UNORDERED_LIST_MARKUP   = UIMA_TT_PFX "UnorderedListMarkup";
  char const * TT::TYPE_NAME_ORDERED_LIST_MARKUP     = UIMA_TT_PFX "OrderedListMarkup";
  char const * TT::TYPE_NAME_TABLE_MARKUP           = UIMA_TT_PFX "TableMarkup";
  char const * TT::TYPE_NAME_CAPTION_MARKUP         = UIMA_TT_PFX "CaptionMarkup";
  char const * TT::TYPE_NAME_MISC_MARKUP            = UIMA_TT_PFX "MiscMarkup";
  char const * TT::TYPE_NAME_MULTI_TOKEN_ANNOTATION  = UIMA_TT_PFX "MultiTokenAnnotation";
  char const * TT::TYPE_NAME_CATEGORY_CONFIDENCE_PAIR      = CATEGORY_CONFIDENCE_PAIR;
  char const * TT::TYPE_NAME_LANGUAGE_CONFIDENCE_PAIR = LANGUAGE_CONFIDENCE_PAIR;
  char const * TT::TYPE_NAME_RELATION              = UIMA_TT_PFX "Relation";

  char const * TT::TYPE_NAME_UNILEX                 = UIMA_TT_PFX "Unilex";
  char const * TT::TYPE_NAME_CLAUSAL_ANNOTATION      = UIMA_TT_PFX "ClausalAnnotation";
  char const * TT::TYPE_NAME_INTENDED_SUMMARY       = UIMA_TT_PFX "SUM_IntendedSummary";


  char const * TT::FEATURE_BASE_NAME_LEMMA_ENTRIES       =  LEMMA_ENTRIES;
  char const * TT::FEATURE_FULL_NAME_LEMMA_ENTRIES       =  TOKEN_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR LEMMA_ENTRIES;

#define TOKEN_NUMBER "tokenNumber"
  char const * TT::FEATURE_BASE_NAME_TOKEN_NUMBER        = TOKEN_NUMBER;
  char const * TT::FEATURE_FULL_NAME_TOKEN_NUMBER        = TOKEN_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR TOKEN_NUMBER ;

#define TOKEN_PROPERTIES "tokenProperties"
  char const * TT::FEATURE_BASE_NAME_TOKEN_PROPERTIES    = TOKEN_PROPERTIES;
  char const * TT::FEATURE_FULL_NAME_TOKEN_PROPERTIES    = TOKEN_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR TOKEN_PROPERTIES;

  char const * TT::FEATURE_BASE_NAME_STOPWORD_TOKEN      = STOPWORD_TOKEN;
  char const * TT::FEATURE_FULL_NAME_STOPWORD_TOKEN      = TOKEN_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR STOPWORD_TOKEN;

  char const * TT::FEATURE_BASE_NAME_SENTENCE_NUMBER     = SENTENCE_NUMBER;
  char const * TT::FEATURE_FULL_NAME_SENTENCE_NUMBER     = SENTENCE_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR SENTENCE_NUMBER;

#define PARAGRAPH_NUMBER "paragraphNumber"
  char const * TT::FEATURE_BASE_NAME_PARAGRAPH_NUMBER    = PARAGRAPH_NUMBER;
  char const * TT::FEATURE_FULL_NAME_PARAGRAPH_NUMBER    = PARAGRAPH_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR PARAGRAPH_NUMBER;

  char const * TT::FEATURE_BASE_NAME_KEY                 = KEY;
  char const * TT::FEATURE_FULL_NAME_KEY                 = KEY_STRING_ENTRY UIMA_TYPE_FEATURE_SEPARATOR KEY;

  char const * TT::FEATURE_BASE_NAME_PART_OF_SPEECH      = PART_OF_SPEECH;
  char const * TT::FEATURE_FULL_NAME_PART_OF_SPEECH      = LEMMA UIMA_TYPE_FEATURE_SEPARATOR PART_OF_SPEECH;

  char const * TT::FEATURE_BASE_NAME_MORPH_ID            = MORPHID;
  char const * TT::FEATURE_FULL_NAME_MORPH_ID            = LEMMA UIMA_TYPE_FEATURE_SEPARATOR MORPHID;

  char const * TT::FEATURE_BASE_NAME_DOCUMENT_LANGUAGE_AS_UIMA_NBR   = DOCUMENT_LANGUAGE_AS_UIMA_NBR;
  char const * TT::FEATURE_FULL_NAME_DOCUMENT_LANGUAGE_AS_UIMA_NBR   = DOCUMENT_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR DOCUMENT_LANGUAGE_AS_UIMA_NBR;

  char const * TT::FEATURE_BASE_NAME_DOCUMENT_ID         =  DOCUMENT_ID;
  char const * TT::FEATURE_FULL_NAME_DOCUMENT_ID = DOCUMENT_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR DOCUMENT_ID;

  char const * TT::FEATURE_BASE_NAME_MARKUP_TAG          =  MARKUP_TAG;
  char const * TT::FEATURE_FULL_NAME_MARKUP_TAG = DOC_STRUCTURE_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR MARKUP_TAG;

#define SYNONYM_ENTRIES "synonymEntries"
  char const * TT::FEATURE_BASE_NAME_SYNONYM_ENTRIES     =  SYNONYM_ENTRIES;
  char const * TT::FEATURE_FULL_NAME_SYNONYM_ENTRIES     =  TOKEN_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR SYNONYM_ENTRIES;

#define PARENT "parent"
  char const * TT::FEATURE_BASE_NAME_PARENT             =  PARENT;
  char const * TT::FEATURE_FULL_NAME_PARENT             =  TT_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR PARENT;

#define CHILDREN "children"
  char const * TT::FEATURE_BASE_NAME_CHILDREN           =  CHILDREN;
  char const * TT::FEATURE_FULL_NAME_CHILDREN           =  TT_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR CHILDREN;

  char const * TT::FEATURE_BASE_NAME_INFLECTED_FORMS     =  INFLECTED_FORMS;
  char const * TT::FEATURE_FULL_NAME_INFLECTED_FORMS     =  TOKEN_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR INFLECTED_FORMS;

  char const * TT::FEATURE_BASE_NAME_SPELL_AID           =  SPELL_AID;
  char const * TT::FEATURE_FULL_NAME_SPELL_AID           =  TOKEN_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR SPELL_AID;

  char const * TT::FEATURE_BASE_NAME_DOCUMENT_SUMMARY    = DOCUMENT_SUMMARY;
  char const * TT::FEATURE_FULL_NAME_DOCUMENT_SUMMARY    = DOCUMENT_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR DOCUMENT_SUMMARY;

  char const * TT::FEATURE_BASE_NAME_DOCUMENT_KEYWORDS   = DOCUMENT_KEYWORDS;
  char const * TT::FEATURE_FULL_NAME_DOCUMENT_KEYWORDS   = DOCUMENT_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR DOCUMENT_KEYWORDS;

  char const * TT::FEATURE_BASE_NAME_POSITIVE_SENTENCE_SCORES_LIST = POSITIVE_SENTENCE_SCORES_LIST;
  char const * TT::FEATURE_FULL_NAME_POSITIVE_SENTENCE_SCORES_LIST = SENTENCE_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR POSITIVE_SENTENCE_SCORES_LIST;

  char const * TT::FEATURE_BASE_NAME_NEGATIVE_SENTENCE_SCORES_LIST = NEGATIVE_SENTENCE_SCORES_LIST;
  char const * TT::FEATURE_FULL_NAME_NEGATIVE_SENTENCE_SCORES_LIST = SENTENCE_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR NEGATIVE_SENTENCE_SCORES_LIST;

  char const * TT::FEATURE_BASE_NAME_POSITIVE_KEYWORD_SCORES_LIST  = POSITIVE_KEYWORD_SCORES_LIST;
  char const * TT::FEATURE_FULL_NAME_POSITIVE_KEYWORD_SCORES_LIST  = LEXICAL_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR POSITIVE_KEYWORD_SCORES_LIST;

  char const * TT::FEATURE_BASE_NAME_NEGATIVE_KEYWORD_SCORES_LIST  = NEGATIVE_KEYWORD_SCORES_LIST;
  char const * TT::FEATURE_FULL_NAME_NEGATIVE_KEYWORD_SCORES_LIST  = LEXICAL_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR NEGATIVE_KEYWORD_SCORES_LIST;

  char const * TT::FEATURE_BASE_NAME_DOCUMENT_CATEGORIES = DOCUMENT_CATEGORIES;
  char const * TT::FEATURE_FULL_NAME_DOCUMENT_CATEGORIES = DOCUMENT_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR DOCUMENT_CATEGORIES;

  char const * TT::FEATURE_BASE_NAME_CATEGORY_STRING     =  CATEGORY_STRING;
  char const * TT::FEATURE_FULL_NAME_CATEGORY_STRING     =  CATEGORY_CONFIDENCE_PAIR UIMA_TYPE_FEATURE_SEPARATOR CATEGORY_STRING;

  char const * TT::FEATURE_BASE_NAME_CATEGORY_CONFIDENCE = CATEGORY_CONFIDENCE;
  char const * TT::FEATURE_FULL_NAME_CATEGORY_CONFIDENCE = CATEGORY_CONFIDENCE_PAIR UIMA_TYPE_FEATURE_SEPARATOR CATEGORY_CONFIDENCE;

  char const * TT::FEATURE_BASE_NAME_DOCUMENT_LANGUAGE_CANDIDATES =  DOCUMENT_LANGUAGE_CANDIDATES;
  char const * TT::FEATURE_FULL_NAME_DOCUMENT_LANGUAGE_CANDIDATES = DOCUMENT_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR DOCUMENT_LANGUAGE_CANDIDATES;

  char const * TT::FEATURE_BASE_NAME_LANGUAGE_CONFIDENCE = LANGUAGE_CONFIDENCE;
  char const * TT::FEATURE_FULL_NAME_LANGUAGE_CONFIDENCE = LANGUAGE_CONFIDENCE_PAIR UIMA_TYPE_FEATURE_SEPARATOR LANGUAGE_CONFIDENCE;

  char const * TT::FEATURE_BASE_NAME_LANGUAGE_ID         =  LANGUAGE_ID;
  char const * TT::FEATURE_FULL_NAME_LANGUAGE_ID         =  LANGUAGE_CONFIDENCE_PAIR UIMA_TYPE_FEATURE_SEPARATOR LANGUAGE_ID;

#define TOPIC_SEGMENT_NUMBER "topicSegmentNumber"
  char const * TT::FEATURE_BASE_NAME_TOPIC_SEGMENT_NUMBER=  TOPIC_SEGMENT_NUMBER;
  char const * TT::FEATURE_FULL_NAME_TOPIC_SEGMENT_NUMBER=  TOPIC_SEGMENT_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR TOPIC_SEGMENT_NUMBER;

  char const * TT::FEATURE_BASE_NAME_LEX_CANONICAL_FORM      =  LEX_CANONICAL_FORM;
  char const * TT::FEATURE_FULL_NAME_LEX_CANONICAL_FORM      =  LEXICAL_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR LEX_CANONICAL_FORM;

  char const * TT::FEATURE_BASE_NAME_LEX_CANONICAL_FORM_CONFIDENCE =  LEX_CANONICAL_FORM_CONFIDENCE;
  char const * TT::FEATURE_FULL_NAME_LEX_CANONICAL_FORM_CONFIDENCE = LEXICAL_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR LEX_CANONICAL_FORM_CONFIDENCE;

  char const * TT::FEATURE_BASE_NAME_CANONICAL_FORM_KIND      =  CANONICAL_FORM_KIND;
  char const * TT::FEATURE_FULL_NAME_CANONICAL_FORM_KIND      =  CANONICAL_FORM UIMA_TYPE_FEATURE_SEPARATOR CANONICAL_FORM_KIND;

  char const * TT::FEATURE_BASE_NAME_CANONICAL_FORM_FREQUENCY = CANONICAL_FORM_FREQUENCY;
  char const * TT::FEATURE_FULL_NAME_CANONICAL_FORM_FREQUENCY = CANONICAL_FORM UIMA_TYPE_FEATURE_SEPARATOR CANONICAL_FORM_FREQUENCY;

  char const * TT::FEATURE_BASE_NAME_RELATION_ARGUMENTS  =  "relationArguments";

  char const * TT::FEATURE_BASE_NAME_RELATION_NAME       =  "relationName";

  char const * TT::FEATURE_BASE_NAME_RELATION_FREQUENCY  =  "relationFrequency";

  char const * TT::FEATURE_BASE_NAME_UNISYN              = UNISYN;
  char const * TT::FEATURE_FULL_NAME_UNISYN              = LEMMA UIMA_TYPE_FEATURE_SEPARATOR UNISYN;

  char const * TT::FEATURE_BASE_NAME_UNIMORPH            = UNIMORPH;
  char const * TT::FEATURE_FULL_NAME_UNIMORPH            = LEMMA UIMA_TYPE_FEATURE_SEPARATOR UNIMORPH;

#define DOCUMENT_META_NAME "xml_DocumentMetaName"
  char const * TT::FEATURE_BASE_NAME_XML_DOCUMENT_META_NAME = DOCUMENT_META_NAME;
  char const * TT::FEATURE_FULL_NAME_XML_DOCUMENT_META_NAME = DOCUMENT_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR DOCUMENT_META_NAME;

#define DOCUMENT_META_CONTENT "xml_DocumentMetaContent"
  char const * TT::FEATURE_BASE_NAME_XML_DOCUMENT_META_CONTENT = DOCUMENT_META_CONTENT;
  char const * TT::FEATURE_FULL_NAME_XML_DOCUMENT_META_CONTENT = DOCUMENT_ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR DOCUMENT_META_CONTENT;


  char const * TT::ANNOSFX                         = "Annotation";


  char const * TT::INDEXID_LEMMA = "LemmaIndex";
  char const * TT::INDEXID_CANONICALFORM = "CanonicalFormIndex";
  char const * TT::INDEXID_RELATION = "RelationIndex";
  char const * TT::INDEXID_SYNONYM = "SynonymIndex";

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



