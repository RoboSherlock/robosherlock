/** \file err_ids.h .
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

    \brief  Contains all UIMACPP error Ids.

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */

#ifndef UIMA_ERR_IDS_H
#define UIMA_ERR_IDS_H

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <uima/text.h>

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {

  /** A type for all UIMACPP error ids. */
  typedef  signed long   TyErrorId;

#ifdef UIMA_ENGINE_MAIN_CPP
  typedef struct _StErrorId2StringMapping {
    TyErrorId               iv_utErrorId;
    const TCHAR *           iv_cpszErrorId;
  }
  StErrorId2StringMapping;
#endif


#ifdef UIMA_ENGINE_MAIN_CPP
  static StErrorId2StringMapping gs_astErrorId2StringMapping[] = {
        /* Unfortunately, this construction with the opening bracket confuses DOC++
           so that it generates bullshit.
           This is why we need a dummy closing bracket - encapsulated in an #ifdef
           which will never be hit */
#ifdef DOCPP_REQUIRES_THIS_BRACKET_SO_THAT_IT_WORKS_PROPERLY
      };
  {
    ; /* taph 4/20/00: added a dummy open bracket to avoid doc++ warning
          about non-matching brackets.*/
#endif
#endif


    /* ----------------------------------------------------------------------- */
    /*       Constants                                                         */
    /* ----------------------------------------------------------------------- */

    /** A UIMACPP function executed successfully. */
#define UIMA_ERR_NONE                                        ((uima::TyErrorId) 0)

    /* Please Note: NO UIMACPP ERROR MESSAGE MAY EXCEED UIMA_ERROR_USER_LAST!!!
           user application: 1000..999,999        UIMA_ERROR_USER_FIRST..UIMA_ERROR_USER_LAST
           (may be unnecessary to reserve first 1000 now we use APR)
    */
#define UIMA_ERROR_USER_FIRST            ((uima::TyErrorId) 1000)
#define UIMA_ERROR_USER_LAST             ((uima::TyErrorId) 999999)

// Slip the APR errors in starting at 100
#define UIMA_ERROR_APR_OFFSET            ((uima::TyErrorId) 100)

#define UIMA_ERR_APR_FAILURE     ((uima::TyErrorId)(   1 + UIMA_ERROR_APR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_APR_FAILURE, _TEXT("UIMA_ERR_APR_FAILURE") },
#endif

    /** @name UIMA_ERR_RESMGR ... */
    /*@{*/
#define UIMA_ERR_RESMGR_OFFSET                               ((uima::TyErrorId)(   0 + UIMA_ERROR_USER_FIRST))
    /** The UIMACPP resource manager could not allocate vital resources.
        UIMACPP resource manager could not be initialized, which means that
        instantiation of a UIMACPP Engine is not possible. */
#define UIMA_ERR_RESMGR_OUT_OF_MEMORY                        ((uima::TyErrorId)(   0 + UIMA_ERR_RESMGR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_RESMGR_OUT_OF_MEMORY, _TEXT("UIMA_ERR_RESMGR_OUT_OF_MEMORY") },
#endif

    /** The resource manager could not load the specified resource.
        The resource manager could not a resource file for the specified language. */
#define UIMA_ERR_RESMGR_COULD_NOT_LOAD_RESOURCE              ((uima::TyErrorId)(   1 + UIMA_ERR_RESMGR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_RESMGR_COULD_NOT_LOAD_RESOURCE, _TEXT("UIMA_ERR_RESMGR_COULD_NOT_LOAD_RESOURCE") },
#endif

    /** The stopword filter resource could not be loaded.
        The resource manager could not find a stopword file for the document language. */
#define UIMA_ERR_RESMGR_RESOURCE_HAS_INVALID_ENTRY           ((uima::TyErrorId)(   2 + UIMA_ERR_RESMGR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_RESMGR_RESOURCE_HAS_INVALID_ENTRY, _TEXT("UIMA_ERR_RESMGR_RESOURCE_HAS_INVALID_ENTRY") },
#endif

#define UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE        ((uima::TyErrorId)(   3 + UIMA_ERR_RESMGR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE, _TEXT("UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE") },
#endif

#define UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_XML4C           ((uima::TyErrorId)(   4 + UIMA_ERR_RESMGR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_XML4C, _TEXT("UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_XML4C") },
#endif

#define UIMA_ERR_RESMGR_INVALID_RESOURCE                     ((uima::TyErrorId)(   5 + UIMA_ERR_RESMGR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_RESMGR_INVALID_RESOURCE , _TEXT("UIMA_ERR_RESMGR_INVALID_RESOURCE") },
#endif

#define UIMA_ERR_RESMGR_NO_RESOURCE_FACTORY_FOR_KIND         ((uima::TyErrorId)(   6 + UIMA_ERR_RESMGR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_RESMGR_NO_RESOURCE_FACTORY_FOR_KIND , _TEXT("UIMA_ERR_RESMGR_NO_RESOURCE_FACTORY_FOR_KIND") },
#endif

#define UIMA_ERR_RESMGR_COULD_NOT_TERMINATE_XML4C           ((uima::TyErrorId)(   7 + UIMA_ERR_RESMGR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_RESMGR_COULD_NOT_TERMINATE_XML4C, _TEXT("UIMA_ERR_RESMGR_COULD_NOT_TERMINATE_XML4C") },
#endif

#define UIMA_ERR_RESMGR_DATA_DIR_DOES_NOT_EXIST           ((uima::TyErrorId)(   8 + UIMA_ERR_RESMGR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_RESMGR_DATA_DIR_DOES_NOT_EXIST, _TEXT("UIMA_ERR_RESMGR_DATA_DIR_DOES_NOT_EXIST") },
#endif

#define UIMA_ERR_RESMGR_WORK_DIR_DOES_NOT_EXIST           ((uima::TyErrorId)(   9 + UIMA_ERR_RESMGR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_RESMGR_WORK_DIR_DOES_NOT_EXIST, _TEXT("UIMA_ERR_RESMGR_WORK_DIR_DOES_NOT_EXIST") },
#endif


    /*@}*/

    /** @name UIMA_ERR_ANNOTATOR ... */
    /*@{*/
#define UIMA_ERR_ANNOTATOR_OFFSET                               ((uima::TyErrorId)(1000 + UIMA_ERROR_USER_FIRST))
    /** The specified annotator could not be found. */
#define UIMA_ERR_ANNOTATOR_COULD_NOT_FIND                       ((uima::TyErrorId)(   0 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_COULD_NOT_FIND, _TEXT("UIMA_ERR_ANNOTATOR_COULD_NOT_FIND") },
#endif

    /** The specified annotator could not be loaded. */
#define UIMA_ERR_ANNOTATOR_COULD_NOT_LOAD                       ((uima::TyErrorId)(   1 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_COULD_NOT_LOAD, _TEXT("UIMA_ERR_ANNOTATOR_COULD_NOT_LOAD") },
#endif

    /** The specified annotator could not be created as
        the annotator function <TT>tafAnnotatorCreate()</TT> returned 0. */
#define UIMA_ERR_ANNOTATOR_COULD_NOT_CREATE                     ((uima::TyErrorId)(   2 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_COULD_NOT_CREATE, _TEXT("UIMA_ERR_ANNOTATOR_COULD_NOT_CREATE") },
#endif

    /** The specified annotator does not contain the required function <TT>tafAnnotatorEnumerateInfo()</TT>. */
#define UIMA_ERR_ANNOTATOR_MISSING_INFO                         ((uima::TyErrorId)(   3 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MISSING_INFO, _TEXT("UIMA_ERR_ANNOTATOR_MISSING_INFO") },
#endif

    /** The specified annotator does not return a valid value for a call to <TT>tafAnnotatorEnumerateInfo()</TT>. */
#define UIMA_ERR_ANNOTATOR_MISSING_INFO_NAME                    ((uima::TyErrorId)(   4 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MISSING_INFO_NAME, _TEXT("UIMA_ERR_ANNOTATOR_MISSING_INFO_NAME") },
#endif

    /** The specified annotator does not contain the required function <TT>tafAnnotatorCreate()</TT>. */
#define UIMA_ERR_ANNOTATOR_MISSING_CREATE                       ((uima::TyErrorId)(   5 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MISSING_CREATE, _TEXT("UIMA_ERR_ANNOTATOR_MISSING_CREATE") },
#endif

    /** The specified annotator does not contain the required function <TT>tafAnnotatorInit()</TT>. */
#define UIMA_ERR_ANNOTATOR_MISSING_INIT                         ((uima::TyErrorId)(   6 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MISSING_INIT, _TEXT("UIMA_ERR_ANNOTATOR_MISSING_INIT") },
#endif

    /** The specified annotator does not contain the required function <TT>tafAnnotatorCacheTypes()</TT>. */
#define UIMA_ERR_ANNOTATOR_MISSING_TYPESYSTEMINIT                   ((uima::TyErrorId)(   7 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MISSING_TYPESYSTEMINIT, _TEXT("UIMA_ERR_ANNOTATOR_MISSING_TYPESYSTEMINIT") },
#endif

    /** The specified annotator does not contain the required function <TT>tafAnnotatorDeInit()</TT>. */
#define UIMA_ERR_ANNOTATOR_MISSING_DEINIT                       ((uima::TyErrorId)(   8 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MISSING_DEINIT, _TEXT("UIMA_ERR_ANNOTATOR_MISSING_DEINIT") },
#endif

    /** The specified annotator does not contain the required function <TT>tafPlugProcessDocument()</TT>. */
#define UIMA_ERR_ANNOTATOR_MISSING_PROCESSDOCUMENT               ((uima::TyErrorId)(   9 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MISSING_PROCESSDOCUMENT, _TEXT("UIMA_ERR_ANNOTATOR_MISSING_PROCESSDOCUMENT") },
#endif

    /** The specified annotator does not contain the required function <TT>tafPlugConfig()</TT>. */
#define UIMA_ERR_ANNOTATOR_MISSING_CONFIG                       ((uima::TyErrorId)(  11 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MISSING_CONFIG, _TEXT("UIMA_ERR_ANNOTATOR_MISSING_CONFIG") },
#endif

    /** The specified annotator does not contain the required function <TT>tafAnnotatorEnumerateTypesGenerated()</TT>. */
#define UIMA_ERR_ANNOTATOR_MISSING_ENUMERATETYPESGENERATED      ((uima::TyErrorId)(  12 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MISSING_ENUMERATETYPESGENERATED, _TEXT("UIMA_ERR_ANNOTATOR_MISSING_ENUMERATETYPESGENERATED") },
#endif

    /** The specified annotator does not contain the required function <TT>tafAnnotatorEnumerateTypesRequired()</TT>. */
#define UIMA_ERR_ANNOTATOR_MISSING_ENUMERATETYPESREQUIRED       ((uima::TyErrorId)(  13 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MISSING_ENUMERATETYPESREQUIRED, _TEXT("UIMA_ERR_ANNOTATOR_MISSING_ENUMERATETYPESREQUIRED") },
#endif

    /** The specified annotator does not contain the required function <TT>tafAnnotatorEnumerateTypesRecommended()</TT>. */
#define UIMA_ERR_ANNOTATOR_MISSING_ENUMERATETYPESRECOMMENDED    ((uima::TyErrorId)(  14 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MISSING_ENUMERATETYPESRECOMMENDED, _TEXT("UIMA_ERR_ANNOTATOR_MISSING_ENUMERATETYPESRECOMMENDED") },
#endif

    /** The specified annotator does not contain the required function <TT>tafAnnotatorEnumerateLangSupported()</TT>. */
#define UIMA_ERR_ANNOTATOR_MISSING_ENUMERATELANGSUPPORTED       ((uima::TyErrorId)(  15 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MISSING_ENUMERATELANGSUPPORTED, _TEXT("UIMA_ERR_ANNOTATOR_MISSING_ENUMERATELANGSUPPORTED") },
#endif

    /** The specified annotator has been called using an enumeration function, with an enumeration
        value of 1024. The annotator manager will stop calling the annotator if an enumeration
        function still returns valid values for such an enumeration value. */
#define UIMA_ERR_ANNOTATOR_ENUMERATION_OVERFLOW                 ((uima::TyErrorId)(  16 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_ENUMERATION_OVERFLOW, _TEXT("UIMA_ERR_ANNOTATOR_ENUMERATION_OVERFLOW") },
#endif

    /** The specified annotator does not support the current document language. */
#define UIMA_ERR_ANNOTATOR_LANG_NOT_SUPPORTED                   ((uima::TyErrorId)(  17 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_LANG_NOT_SUPPORTED, _TEXT("UIMA_ERR_ANNOTATOR_LANG_NOT_SUPPORTED") },
#endif

    /** The annotator manager has been called with an invalid annotator specification. */
#define UIMA_ERR_ANNOTATOR_MGR_INVAL_ANNOTATOR_REQUEST             ((uima::TyErrorId)( 100 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MGR_INVAL_ANNOTATOR_REQUEST, _TEXT("UIMA_ERR_ANNOTATOR_MGR_INVAL_ANNOTATOR_REQUEST") },
#endif

    /** The annotator manager has been called with an invalid annotator filename. */
#define UIMA_ERR_ANNOTATOR_MGR_INVAL_ANNOTATOR_FILENAME            ((uima::TyErrorId)( 101 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MGR_INVAL_ANNOTATOR_FILENAME, _TEXT("UIMA_ERR_ANNOTATOR_MGR_INVAL_ANNOTATOR_FILENAME") },
#endif

    /** The annotator manager has been called with an invalid annotator symbolic name. */
#define UIMA_ERR_ANNOTATOR_MGR_INVAL_ANNOTATOR_SYMNAME             ((uima::TyErrorId)( 102 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MGR_INVAL_ANNOTATOR_SYMNAME, _TEXT("UIMA_ERR_ANNOTATOR_MGR_INVAL_ANNOTATOR_SYMNAME") },
#endif

    /** The annotator manager has been called through method <TT>Engine::processDocument()</TT>,
        and some annotators could not handle the document using the current document
        language. The error is more of an information message
        and will not be generated if the configuration option
        <TT>ProcessUnsupportedLanguages</TT> is set to a <TT>true</TT> value.  */
#define UIMA_ERR_ANNOTATOR_MGR_LANG_NOT_SUPPORTED_FOR_ANNOTATOR    ((uima::TyErrorId)( 103 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MGR_LANG_NOT_SUPPORTED_FOR_ANNOTATOR, _TEXT("UIMA_ERR_ANNOTATOR_MGR_LANG_NOT_SUPPORTED_FOR_ANNOTATOR") },
#endif

    /* No annotators have been specified in the UIMA configuration
       for the specified UIMACPP application key */
#define UIMA_ERR_ANNOTATOR_MGR_CONFIG_NO_ANNOTATORS_SPECIFIED      ((uima::TyErrorId)( 104 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MGR_CONFIG_NO_ANNOTATORS_SPECIFIED, _TEXT("UIMA_ERR_ANNOTATOR_MGR_CONFIG_NO_ANNOTATORS_SPECIFIED") },
#endif

    /* No target types have been specified in the UIMA configuration
       for the specified UIMACPP application key */
#define UIMA_ERR_ANNOTATOR_MGR_CONFIG_NO_TARGETTYPES_SPECIFIED  ((uima::TyErrorId)( 105 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MGR_CONFIG_NO_TARGETTYPES_SPECIFIED, _TEXT("UIMA_ERR_ANNOTATOR_MGR_CONFIG_NO_TARGETTYPES_SPECIFIED") },
#endif

    /* An invalid target type has been specified in the UIMA configuration for the
       specified UIMACPP application key. Verify that the target type specified in the
       configuration is a registered type. */
#define UIMA_ERR_ANNOTATOR_MGR_CONFIG_INVALID_RESULTSPECIFICATION       ((uima::TyErrorId)( 106 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MGR_CONFIG_INVALID_RESULTSPECIFICATION, _TEXT("UIMA_ERR_ANNOTATOR_MGR_CONFIG_INVALID_RESULTSPECIFICATION") },
#endif

    /* An target type has been specified in the UIMA configuration for a specific
       annotator but the annotator does not generate this type.
       Verify that the target type specified in the configuration is a registered
       type and that the annotator's function <TT>tafAnnotatorEnumerateTypesGenerated()</TT>
       enumerates this type */
#define UIMA_ERR_ANNOTATOR_MGR_CONFIG_TARGETTYPE_NOT_GENERATED  ((uima::TyErrorId)( 107 + UIMA_ERR_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ANNOTATOR_MGR_CONFIG_TARGETTYPE_NOT_GENERATED, _TEXT("UIMA_ERR_ANNOTATOR_MGR_CONFIG_TARGETTYPE_NOT_GENERATED") },
#endif
    /*@}*/

    /** @name UIMA_ERR_CONFIG ... */
    /*@{*/
#define UIMA_ERR_CONFIG_OFFSET                               ((uima::TyErrorId)(2000 + UIMA_ERROR_USER_FIRST))
#define UIMA_ERR_CONFIG_FILE_NOT_FOUND                       ((uima::TyErrorId)(   0 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_FILE_NOT_FOUND, _TEXT("UIMA_ERR_CONFIG_FILE_NOT_FOUND") },
#endif

#define UIMA_ERR_CONFIG_REQUIRED_SECTION_NOT_FOUND           ((uima::TyErrorId)(   1 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_REQUIRED_SECTION_NOT_FOUND, _TEXT("UIMA_ERR_CONFIG_REQUIRED_SECTION_NOT_FOUND") },
#endif

#define UIMA_ERR_CONFIG_REQUIRED_OPTION_NOT_FOUND            ((uima::TyErrorId)(   2 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_REQUIRED_OPTION_NOT_FOUND, _TEXT("UIMA_ERR_CONFIG_REQUIRED_OPTION_NOT_FOUND") },
#endif

#define UIMA_ERR_CONFIG_SPECIFIED_ANNOTATOR_NOT_FOUND           ((uima::TyErrorId)(   3 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_SPECIFIED_ANNOTATOR_NOT_FOUND, _TEXT("UIMA_ERR_CONFIG_SPECIFIED_ANNOTATOR_NOT_FOUND") },
#endif

#define UIMA_ERR_CONFIG_NO_ANNOTATORS_SPECIFIED                 ((uima::TyErrorId)(   4 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_NO_ANNOTATORS_SPECIFIED, _TEXT("UIMA_ERR_CONFIG_NO_ANNOTATORS_SPECIFIED") },
#endif

#define UIMA_ERR_CONFIG_FILE_NOT_LOADED                      ((uima::TyErrorId)(   5 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_FILE_NOT_LOADED, _TEXT("UIMA_ERR_CONFIG_FILE_NOT_LOADED") },
#endif

#define UIMA_ERR_CONFIG_OPTION_NOT_FOUND                     ((uima::TyErrorId)(   6 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_OPTION_NOT_FOUND, _TEXT("UIMA_ERR_CONFIG_OPTION_NOT_FOUND") },
#endif

#define UIMA_ERR_CONFIG_REQUIRED_OPTION_IS_EMPTY             ((uima::TyErrorId)(   7 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_REQUIRED_OPTION_IS_EMPTY, _TEXT("UIMA_ERR_CONFIG_REQUIRED_OPTION_IS_EMPTY") },
#endif

#define UIMA_ERR_CONFIG_OPTION_IS_EMPTY                      ((uima::TyErrorId)(   8 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_OPTION_IS_EMPTY, _TEXT("UIMA_ERR_CONFIG_OPTION_IS_EMPTY") },
#endif

#define UIMA_ERR_CONFIG_INDEX_OUT_OF_BOUNDS                  ((uima::TyErrorId)(   9 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_INDEX_OUT_OF_BOUNDS, _TEXT("UIMA_ERR_CONFIG_INDEX_OUT_OF_BOUNDS") },
#endif

#define UIMA_ERR_CONFIG_SAVE_FILE_NOT_VALID                  ((uima::TyErrorId)(  10 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_SAVE_FILE_NOT_VALID, _TEXT("UIMA_ERR_CONFIG_SAVE_FILE_NOT_VALID") },
#endif

#define UIMA_ERR_CONFIG_INVALID_OPTION_VALUE                 ((uima::TyErrorId)(  11 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_INVALID_OPTION_VALUE, _TEXT("UIMA_ERR_CONFIG_INVALID_OPTION_VALUE") },
#endif

#define UIMA_ERR_CONFIG_PART_EXISTS                          ((uima::TyErrorId)(  12 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_PART_EXISTS, _TEXT("UIMA_ERR_CONFIG_PART_EXISTS") },
#endif

#define UIMA_ERR_CONFIG_PART_DOES_NOT_EXIST                  ((uima::TyErrorId)(  13 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_PART_DOES_NOT_EXIST, _TEXT("UIMA_ERR_CONFIG_PART_DOES_NOT_EXIST") },
#endif

#define UIMA_ERR_CONFIG_SECTION_FOR_APPKEY_NOT_FOUND         ((uima::TyErrorId)(  14 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_SECTION_FOR_APPKEY_NOT_FOUND, _TEXT("UIMA_ERR_CONFIG_SECTION_FOR_APPKEY_NOT_FOUND") },
#endif

#define UIMA_ERR_CONFIG_SECTION_FOR_ANNOTATOR_NOT_FOUND         ((uima::TyErrorId)(  15 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_SECTION_FOR_ANNOTATOR_NOT_FOUND, _TEXT("UIMA_ERR_CONFIG_SECTION_FOR_ANNOTATOR_NOT_FOUND") },
#endif

#define UIMA_ERR_CONFIG_SECTION_FOR_TYPE_NOT_FOUND           ((uima::TyErrorId)(  16 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_SECTION_FOR_TYPE_NOT_FOUND, _TEXT("UIMA_ERR_CONFIG_SECTION_FOR_TYPE_NOT_FOUND") },
#endif

#define UIMA_ERR_CONFIG_SECTION_FOR_FILTER_NOT_FOUND         ((uima::TyErrorId)(  17 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_SECTION_FOR_FILTER_NOT_FOUND, _TEXT("UIMA_ERR_CONFIG_SECTION_FOR_FILTER_NOT_FOUND") },
#endif

#define UIMA_ERR_CONFIG_SECTION_NOT_FOUND                    ((uima::TyErrorId)(  18 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_SECTION_NOT_FOUND, _TEXT("UIMA_ERR_CONFIG_SECTION_NOT_FOUND") },
#endif

#define UIMA_ERR_CONFIG_INVALID_EXTRACTOR_FOR_TYPE                    ((uima::TyErrorId)(  19 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_INVALID_EXTRACTOR_FOR_TYPE, _TEXT("UIMA_ERR_CONFIG_INVALID_EXTRACTOR_FOR_TYPE") },
#endif

#define UIMA_ERR_CONFIG_DUPLICATE_INDEX_LABEL                    ((uima::TyErrorId)(  20 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_DUPLICATE_INDEX_LABEL, _TEXT("UIMA_ERR_CONFIG_DUPLICATE_INDEX_LABEL") },
#endif

#define UIMA_ERR_CONFIG_DUPLICATE_TYPE_NAME                    ((uima::TyErrorId)(  21 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_DUPLICATE_TYPE_NAME, _TEXT("UIMA_ERR_CONFIG_DUPLICATE_TYPE_NAME") },
#endif

#define UIMA_ERR_CONFIG_DUPLICATE_FEATURE_NAME                    ((uima::TyErrorId)(  22 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_DUPLICATE_FEATURE_NAME, _TEXT("UIMA_ERR_CONFIG_DUPLICATE_FEATURE_NAME") },
#endif

#define UIMA_ERR_CONFIG_COULD_NOT_INITIALIZE_XML4C                    ((uima::TyErrorId)(  23 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_COULD_NOT_INITIALIZE_XML4C, _TEXT("UIMA_ERR_CONFIG_COULD_NOT_INITIALIZE_XML4C") },
#endif

#define UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND                    ((uima::TyErrorId)(  24 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND, _TEXT("UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND") },
#endif

#define UIMA_ERR_CONFIG_NO_DEFAULT_GROUP_DEFINED                    ((uima::TyErrorId)(  25 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_NO_DEFAULT_GROUP_DEFINED, _TEXT("UIMA_ERR_CONFIG_NO_DEFAULT_GROUP_DEFINED") },
#endif

#define UIMA_ERR_CONFIG_NO_GROUPS_DEFINED                    ((uima::TyErrorId)(  26 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_NO_GROUPS_DEFINED, _TEXT("UIMA_ERR_CONFIG_NO_GROUPS_DEFINED") },
#endif

#define UIMA_ERR_CONFIG_INVALID_GROUP_NAME                    ((uima::TyErrorId)(  27 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_INVALID_GROUP_NAME, _TEXT("UIMA_ERR_CONFIG_INVALID_GROUP_NAME") },
#endif

#define UIMA_ERR_CONFIG_INVALID_PARAM_NAME                    ((uima::TyErrorId)(  28 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_INVALID_PARAM_NAME, _TEXT("UIMA_ERR_CONFIG_INVALID_PARAM_NAME") },
#endif

#define UIMA_ERR_CONFIG_OBJECT_COMITTED                    ((uima::TyErrorId)(  29 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_OBJECT_COMITTED, _TEXT("UIMA_ERR_CONFIG_OBJECT_COMITTED") },
#endif

#define UIMA_ERR_CONFIG_INVALID_XML_TAG                    ((uima::TyErrorId)(  30 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_INVALID_XML_TAG, _TEXT("UIMA_ERR_CONFIG_INVALID_XML_TAG") },
#endif

#define UIMA_ERR_CONFIG_PARAM_NOT_DEFINED_IN_GROUP                    ((uima::TyErrorId)(  31 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_PARAM_NOT_DEFINED_IN_GROUP, _TEXT("UIMA_ERR_CONFIG_PARAM_NOT_DEFINED_IN_GROUP") },
#endif

#define UIMA_ERR_CONFIG_PARAM_NOT_DEFINED                    ((uima::TyErrorId)(  32 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_PARAM_NOT_DEFINED, _TEXT("UIMA_ERR_CONFIG_PARAM_NOT_DEFINED") },
#endif

#define UIMA_ERR_CONFIG_INVALID_TYPE_FOR_PARAM                    ((uima::TyErrorId)(  33 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_INVALID_TYPE_FOR_PARAM, _TEXT("UIMA_ERR_CONFIG_INVALID_TYPE_FOR_PARAM") },
#endif

#define UIMA_ERR_CONFIG_NO_VALUE_FOR_MANDATORY_PARAM                    ((uima::TyErrorId)(  34 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_NO_VALUE_FOR_MANDATORY_PARAM, _TEXT("UIMA_ERR_CONFIG_NO_VALUE_FOR_MANDATORY_PARAM") },
#endif

#define UIMA_ERR_CONFIG_NO_VALUE_FOR_MANDATORY_PARAM_IN_GROUP                    ((uima::TyErrorId)(  35 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_NO_VALUE_FOR_MANDATORY_PARAM_IN_GROUP, _TEXT("UIMA_ERR_CONFIG_NO_VALUE_FOR_MANDATORY_PARAM_IN_GROUP") },
#endif

#define UIMA_ERR_CONFIG_DUPLICATE_CONFIG_PARAM                    ((uima::TyErrorId)(  36 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_DUPLICATE_CONFIG_PARAM, _TEXT("UIMA_ERR_CONFIG_DUPLICATE_CONFIG_PARAM") },
#endif

#define UIMA_ERR_CONFIG_DUPLICATE_NAME_VALUE_PAIR                    ((uima::TyErrorId)(  37 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_DUPLICATE_NAME_VALUE_PAIR, _TEXT("UIMA_ERR_CONFIG_DUPLICATE_NAME_VALUE_PAIR") },
#endif

#define UIMA_ERR_CONFIG_DUPLICATE_GROUP                    ((uima::TyErrorId)(  38 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_DUPLICATE_GROUP, _TEXT("UIMA_ERR_CONFIG_DUPLICATE_GROUP") },
#endif

#define UIMA_ERR_CONFIG_ALLOWED_VALUES_DEFINED_FOR_NON_STRING_TYPE                    ((uima::TyErrorId)(  39 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_ALLOWED_VALUES_DEFINED_FOR_NON_STRING_TYPE, _TEXT("UIMA_ERR_CONFIG_ALLOWED_VALUES_DEFINED_FOR_NON_STRING_TYPE") },
#endif

#define UIMA_ERR_CONFIG_CYCLIC_XINCLUSION                    ((uima::TyErrorId)(  40 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_CYCLIC_XINCLUSION, _TEXT("UIMA_ERR_CONFIG_CYCLIC_XINCLUSION") },
#endif

#define UIMA_ERR_CONFIG_INVALID_XINCLUDE_TAG                    ((uima::TyErrorId)(  41 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_INVALID_XINCLUDE_TAG, _TEXT("UIMA_ERR_CONFIG_INVALID_XINCLUDE_TAG") },
#endif

#define UIMA_ERR_CONFIG_INVALID_XML_ATTRIBUTE_VALUE                    ((uima::TyErrorId)(  42 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_INVALID_XML_ATTRIBUTE_VALUE, _TEXT("UIMA_ERR_CONFIG_INVALID_XML_ATTRIBUTE_VALUE") },
#endif
#define UIMA_ERR_CONFIG_DUPLICATE_ALLOWED_VALUE                    ((uima::TyErrorId)(  43 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_DUPLICATE_ALLOWED_VALUE, _TEXT("UIMA_ERR_CONFIG_DUPLICATE_ALLOWED_VALUE") },
#endif

#define UIMA_ERR_IMPORT_INVALID_XML_ATTRIBUTE                    ((uima::TyErrorId)(  44 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_IMPORT_INVALID_XML_ATTRIBUTE , _TEXT("UIMA_ERR_IMPORT_INVALID_XML_ATTRIBUTE") },
#endif

#define UIMA_ERR_CONFIG_OBJECT_NOT_FOUND                    ((uima::TyErrorId)(  45 + UIMA_ERR_CONFIG_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CONFIG_OBJECT_NOT_FOUND , _TEXT("UIMA_ERR_CONFIG_OBJECT_NOT_FOUND") },
#endif

    /*@}*/

    /** @name UIMA_ERR_FILTER ... */
    /*@{*/
#define UIMA_ERR_FILTER_OFFSET                               ((uima::TyErrorId)(3000 + UIMA_ERROR_USER_FIRST))
    /** The UIMACPP filter manager could not allocate memory, which means that some
        filters could not be loaded. See the trace for details. */
#define UIMA_ERR_FILTER_OUT_OF_MEMORY                        ((uima::TyErrorId)(   0 + UIMA_ERR_FILTER_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_FILTER_OUT_OF_MEMORY, _TEXT("UIMA_ERR_FILTER_OUT_OF_MEMORY") },
#endif

    /** The UIMACPP filter manager could initialize the filter as specified in the UIMA configuration. */
#define UIMA_ERR_FILTER_COULD_NOT_INIT                       ((uima::TyErrorId)(   1 + UIMA_ERR_FILTER_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_FILTER_COULD_NOT_INIT, _TEXT("UIMA_ERR_FILTER_COULD_NOT_INIT") },
#endif

    /** An invalid filter type (keyword) is specified in the UIMA configuration. */
#define UIMA_ERR_FILTER_INVALID_TYPE_SPECIFIED               ((uima::TyErrorId)(   3 + UIMA_ERR_FILTER_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_FILTER_INVALID_TYPE_SPECIFIED, _TEXT("UIMA_ERR_FILTER_INVALID_TYPE_SPECIFIED") },
#endif
    /*@}*/

    /** @name UIMA_ERR_ENGINE ... */
    /*@{*/
#define UIMA_ERR_ENGINE_OFFSET                               ((uima::TyErrorId)(4000 + UIMA_ERROR_USER_FIRST))
    /** The UIMACPP engine could not allocate enough memory.
        Check and decrease the settings for the memory pool. */
#define UIMA_ERR_ENGINE_OUT_OF_MEMORY                        ((uima::TyErrorId)(   0 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_OUT_OF_MEMORY, _TEXT("UIMA_ERR_ENGINE_OUT_OF_MEMORY") },
#endif

    /** The UIMACPP engine functions were called in an inappropriate sequence.
        Refer to the UIMACPP manual for more details on the correct calling sequence. */
#define UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE             ((uima::TyErrorId)(   1 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE, _TEXT("UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE") },
#endif

    /** An unexpected exception occured in the engine.
        This is an internal error and should be reported to the UIMA team. */
#define UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION                 ((uima::TyErrorId)(   2 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, _TEXT("UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION") },
#endif

    /** The memory pool for the document buffer could not be initalized as
        there is not enough memory for the pool size specification in the UIMA configuration. */
#define UIMA_ERR_ENGINE_COULD_NOT_INIT_MEMPOOL_DOC           ((uima::TyErrorId)(   3 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_COULD_NOT_INIT_MEMPOOL_DOC, _TEXT("UIMA_ERR_ENGINE_COULD_NOT_INIT_MEMPOOL_DOC") },
#endif

    /** The memory pool for miscellaneous items could not be initalized as
        there is not enough memory for the pool size specification in the UIMA configuration. */
#define UIMA_ERR_ENGINE_COULD_NOT_INIT_MEMPOOL_MISC          ((uima::TyErrorId)(   4 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_COULD_NOT_INIT_MEMPOOL_MISC, _TEXT("UIMA_ERR_ENGINE_COULD_NOT_INIT_MEMPOOL_MISC") },
#endif

    /** The memory pool for instance data could not be initalized as
        there is not enough memory for the pool size specification in the UIMA configuration. */
#define UIMA_ERR_ENGINE_COULD_NOT_INIT_MEMPOOL_INST          ((uima::TyErrorId)(   5 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_COULD_NOT_INIT_MEMPOOL_INST, _TEXT("UIMA_ERR_ENGINE_COULD_NOT_INIT_MEMPOOL_INST") },
#endif

    /** The memory pool for collection data could not be initalized as
        there is not enough memory for the pool size specification in the UIMA configuration. */
#define UIMA_ERR_ENGINE_COULD_NOT_INIT_MEMPOOL_COLL          ((uima::TyErrorId)(   6 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_COULD_NOT_INIT_MEMPOOL_COLL, _TEXT("UIMA_ERR_ENGINE_COULD_NOT_INIT_MEMPOOL_COLL") },
#endif

    /** An attempt has been made to call method <TT>Engine::init()</TT> multiple times. */
#define UIMA_ERR_ENGINE_ALREADY_INITIALIZED                  ((uima::TyErrorId)(   7 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_ALREADY_INITIALIZED, _TEXT("UIMA_ERR_ENGINE_ALREADY_INITIALIZED") },
#endif

    /** The configuration name passed to the UIMACPP engine as a C string pointer is an invalid pointer. */
#define UIMA_ERR_ENGINE_CONFIGNAME_INVALPTR                  ((uima::TyErrorId)(   8 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_CONFIGNAME_INVALPTR, _TEXT("UIMA_ERR_ENGINE_CONFIGNAME_INVALPTR") },
#endif

    /** The configuration name passed to the UIMACPP engine as a C string pointer is invalid, and probably empty. */
#define UIMA_ERR_ENGINE_CONFIGNAME_INVALID                   ((uima::TyErrorId)(   9 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_CONFIGNAME_INVALID, _TEXT("UIMA_ERR_ENGINE_CONFIGNAME_INVALID") },
#endif

    /** The application name passed to the UIMACPP engine as a C string pointer is an invalid pointer. */
#define UIMA_ERR_ENGINE_APPNAME_INVALPTR                     ((uima::TyErrorId)(  10 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_APPNAME_INVALPTR, _TEXT("UIMA_ERR_ENGINE_APPNAME_INVALPTR") },
#endif

    /** The application name passed to the UIMACPP engine as a C string pointer is invalid, and probably empty. */
#define UIMA_ERR_ENGINE_APPNAME_INVALID                      ((uima::TyErrorId)(  11 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_APPNAME_INVALID, _TEXT("UIMA_ERR_ENGINE_APPNAME_INVALID") },
#endif

    /** The application name passed to the UIMACPP engine as a C string pointer has an invalid prefix, which is probably "UIMACPP". */
#define UIMA_ERR_ENGINE_APPNAME_USES_INVALID_PREFIX          ((uima::TyErrorId)(  12 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_APPNAME_USES_INVALID_PREFIX, _TEXT("UIMA_ERR_ENGINE_APPNAME_USES_INVALID_PREFIX") },
#endif


    /* taph 11/27/00: It used to be an error if an application would try to
       call UIMACPP with an empty (size zero) document. Since this condition is
       somtimes hard to check (e.g. in XML parsing) for UIMACPP applications we
       now handle this in Engine.processDocument() and no longer return an error.
     */
    /** An attempt has been made to process an empty document. Reset the engine and continue processing. */
    /*
    #define UIMA_ERR_ENGINE_EMPTY_DOCUMENT                       ((uima::TyErrorId)(  13 + UIMA_ERR_ENGINE_OFFSET))
    #ifdef UIMA_ENGINE_MAIN_CPP
       { UIMA_ERR_ENGINE_EMPTY_DOCUMENT, _TEXT("UIMA_ERR_ENGINE_EMPTY_DOCUMENT") },
    #endif
    */

    /** UIMACPP version 1 does not support multiple languages per document. */
#define UIMA_ERR_ENGINE_MULTILANG_PER_DOC_NOT_SUPPORTED      ((uima::TyErrorId)(  14 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_MULTILANG_PER_DOC_NOT_SUPPORTED, _TEXT("UIMA_ERR_ENGINE_MULTILANG_PER_DOC_NOT_SUPPORTED") },
#endif

    /** The document data passed to the UIMACPP engine as a pointer is an invalid pointer. */
#define UIMA_ERR_ENGINE_DOC_DATA_INVALPTR                    ((uima::TyErrorId)(  15 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_DOC_DATA_INVALPTR, _TEXT("UIMA_ERR_ENGINE_DOC_DATA_INVALPTR") },
#endif

    /** The specified CCSID is not supported. There is no CCSID
        to UCS-2 converter available on the system. */
#define UIMA_ERR_ENGINE_CCSID_NOT_SUPPORTED                  ((uima::TyErrorId)(  16 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_CCSID_NOT_SUPPORTED, _TEXT("UIMA_ERR_ENGINE_CCSID_NOT_SUPPORTED") },
#endif

    /** The specified CCSID is not a valid CCSID. */
#define UIMA_ERR_ENGINE_CCSID_INVALID                        ((uima::TyErrorId)(  17 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_CCSID_INVALID, _TEXT("UIMA_ERR_ENGINE_CCSID_INVALID") },
#endif

    /** An attempt has been made to use the UIMACPP engine without having initialized the resource manager. */
#define UIMA_ERR_ENGINE_RESMGR_NOT_INITIALIZED               ((uima::TyErrorId)(  18 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_RESMGR_NOT_INITIALIZED, _TEXT("UIMA_ERR_ENGINE_RESMGR_NOT_INITIALIZED") },
#endif

    /** The specified language is not a valid language. */
#define UIMA_ERR_ENGINE_LANGUAGE_INVALID                     ((uima::TyErrorId)(  19 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_LANGUAGE_INVALID, _TEXT("UIMA_ERR_ENGINE_LANGUAGE_INVALID") },
#endif

    /** The UIMACPP engine could not allocate enough memory for its memory pools.
        Check and decrease the settings for the memory pools. */
#define UIMA_ERR_ENGINE_POOL_OUT_OF_MEMORY                   ((uima::TyErrorId)(  20 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_POOL_OUT_OF_MEMORY, _TEXT("UIMA_ERR_ENGINE_POOL_OUT_OF_MEMORY") },
#endif

    /** A Windows C exception has occured.
        */
#define UIMA_ERR_ENGINE_WINDOWS_EXCEPTION                    ((uima::TyErrorId)(  21 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_WINDOWS_EXCEPTION, _TEXT("UIMA_ERR_ENGINE_WINDOWS_EXCEPTION") },
#endif

// feature structure related errors

#define UIMA_ERR_INVALID_FS_OBJECT                        ((uima::TyErrorId)( 22 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_INVALID_FS_OBJECT, _TEXT("UIMA_ERR_INVALID_FS_OBJECT") },
#endif

#define UIMA_ERR_INVALID_FSTYPE_OBJECT                 ((uima::TyErrorId)( 23 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_INVALID_FSTYPE_OBJECT, _TEXT("UIMA_ERR_INVALID_FSTYPE_OBJECT") },
#endif

#define UIMA_ERR_INVALID_FSFEATURE_OBJECT              ((uima::TyErrorId)( 24 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_INVALID_FSFEATURE_OBJECT, _TEXT("UIMA_ERR_INVALID_FSFEATURE_OBJECT") },
#endif

#define UIMA_ERR_FEATURE_NOT_APPROPRIATE          ((uima::TyErrorId)( 25 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_FEATURE_NOT_APPROPRIATE, _TEXT("UIMA_ERR_FEATURE_NOT_APPROPRIATE") },
#endif

#define UIMA_ERR_INCOMPATIBLE_RANGE_TYPE          ((uima::TyErrorId)( 26 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_INCOMPATIBLE_RANGE_TYPE, _TEXT("UIMA_ERR_INCOMPATIBLE_RANGE_TYPE") },
#endif

#define UIMA_ERR_INVALID_INDEX_OBJECT          ((uima::TyErrorId)( 27 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_INVALID_INDEX_OBJECT, _TEXT("UIMA_ERR_INVALID_INDEX_OBJECT") },
#endif

#define UIMA_ERR_FS_IS_NOT_STRING          ((uima::TyErrorId)( 28 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_FS_IS_NOT_STRING, _TEXT("UIMA_ERR_FS_IS_NOT_STRING") },
#endif

#define UIMA_ERR_TYPESYSTEM_ALREADY_COMMITTED          ((uima::TyErrorId)( 29 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_TYPESYSTEM_ALREADY_COMMITTED, _TEXT("UIMA_ERR_TYPESYSTEM_ALREADY_COMMITTED") },
#endif

#define UIMA_ERR_FS_IS_NOT_ARRAY          ((uima::TyErrorId)( 30 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_FS_IS_NOT_ARRAY, _TEXT("UIMA_ERR_FS_IS_NOT_ARRAY") },
#endif

#define UIMA_ERR_WRONG_FSTYPE             ((uima::TyErrorId)( 31 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_WRONG_FSTYPE, _TEXT("UIMA_ERR_WRONG_FSTYPE") },
#endif

#define UIMA_ERR_LIST_IS_EMPTY             ((uima::TyErrorId)( 32 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_LIST_IS_EMPTY, _TEXT("UIMA_ERR_LIST_IS_EMPTY") },
#endif

#define UIMA_ERR_LIST_IS_CIRCULAR     ((uima::TyErrorId)(   33 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_LIST_IS_CIRCULAR, _TEXT("UIMA_ERR_LIST_IS_EMPTY") },
#endif

#define UIMA_ERR_FS_IS_NOT_LIST             ((uima::TyErrorId)( 34 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_FS_IS_NOT_LIST, _TEXT("UIMA_ERR_FS_IS_NOT_LIST_EXCEPTION") },
#endif

#define UIMA_ERR_TYPESYSTEM_NOT_YET_COMMITTED        ((uima::TyErrorId)( 35 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_TYPESYSTEM_NOT_YET_COMMITTED, _TEXT("UIMA_ERR_TYPESYSTEM_NOT_YET_COMMITTED") },
#endif

#define UIMA_ERR_INDEX_ALREADY_EXISTS        ((uima::TyErrorId)( 36 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_INDEX_ALREADY_EXISTS, _TEXT("UIMA_ERR_INDEX_ALREADY_EXISTS") },
#endif

#define UIMA_ERR_WRONG_FSTYPE_FOR_INDEX        ((uima::TyErrorId)( 37 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_WRONG_FSTYPE_FOR_INDEX, _TEXT("UIMA_ERR_WRONG_FSTYPE_FOR_INDEX") },
#endif

#define UIMA_ERR_INVALID_INDEX_ID        ((uima::TyErrorId)( 38 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_INVALID_INDEX_ID, _TEXT("UIMA_ERR_INVALID_INDEX_ID") },
#endif

#define UIMA_ERR_RESOURCE_NOT_FOUND     ((uima::TyErrorId)(   39 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_RESOURCE_NOT_FOUND, _TEXT("UIMA_ERR_RESOURCE_NOT_FOUND") },
#endif

#define UIMA_ERR_RESOURCE_EMPTY     ((uima::TyErrorId)(   40 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_RESOURCE_EMPTY, _TEXT("UIMA_ERR_RESOURCE_EMPTY") },
#endif

#define UIMA_ERR_RESOURCE_CORRUPTED     ((uima::TyErrorId)(   41 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_RESOURCE_CORRUPTED, _TEXT("UIMA_ERR_RESOURCE_CORRUPTED") },
#endif

#define UIMA_ERR_XMLTYPESYSTEMREADER     ((uima::TyErrorId)(   42 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_XMLTYPESYSTEMREADER, _TEXT("UIMA_ERR_XMLTYPESYSTEMREADER") },
#endif

#define UIMA_ERR_FS_ARRAY_OUT_OF_BOUNDS     ((uima::TyErrorId)(   43 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_FS_ARRAY_OUT_OF_BOUNDS, _TEXT("UIMA_ERR_FS_ARRAY_OUT_OF_BOUNDS") },
#endif

#define UIMA_ERR_NOT_YET_IMPLEMENTED     ((uima::TyErrorId)(   44 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_NOT_YET_IMPLEMENTED, _TEXT("UIMA_ERR_NOT_YET_IMPLEMENTED") },
#endif

#define UIMA_ERR_TYPE_CREATION_FAILED     ((uima::TyErrorId)(   45 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_TYPE_CREATION_FAILED, _TEXT("UIMA_ERR_TYPE_CREATION_FAILED") },
#endif

#define UIMA_ERR_FEATURE_INTRO_FAILED     ((uima::TyErrorId)(   46 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_FEATURE_INTRO_FAILED, _TEXT("UIMA_ERR_FEATURE_INTRO_FAILED") },
#endif

#define UIMA_ERR_COULD_NOT_CREATE_FS_OF_FINAL_TYPE     ((uima::TyErrorId)(   47 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_COULD_NOT_CREATE_FS_OF_FINAL_TYPE, _TEXT("UIMA_ERR_COULD_NOT_CREATE_FS_OF_FINAL_TYPE") },
#endif

#define UIMA_ERR_UNKNOWN_TYPE     ((uima::TyErrorId)(   48 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_UNKNOWN_TYPE, _TEXT("UIMA_ERR_UNKNOWN_TYPE") },
#endif

#define UIMA_ERR_UNKNOWN_RANGE_TYPE     ((uima::TyErrorId)(   49 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_UNKNOWN_RANGE_TYPE, _TEXT("UIMA_ERR_UNKNOWN_RANGE_TYPE") },
#endif

#define UIMA_ERR_UNKNOWN_FEATURE     ((uima::TyErrorId)(   50 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_UNKNOWN_FEATURE, _TEXT("UIMA_ERR_UNKNOWN_FEATURE") },
#endif

#define UIMA_ERR_WRONG_STRING_VALUE     ((uima::TyErrorId)(   51 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_WRONG_STRING_VALUE, _TEXT("UIMA_ERR_WRONG_STRING_VALUE") },
#endif

#define UIMA_ERR_ALLOWED_STRING_VALUES_INCOMPATIBLE     ((uima::TyErrorId)(   52 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ALLOWED_STRING_VALUES_INCOMPATIBLE, _TEXT("UIMA_ERR_ALLOWED_STRING_VALUES_INCOMPATIBLE") },
#endif

#define UIMA_ERR_TYPE_PRIORITY_CONFLICT     ((uima::TyErrorId)(   53 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_TYPE_PRIORITY_CONFLICT, _TEXT("UIMA_ERR_TYPE_PRIORITY_CONFLICT") },
#endif

#define UIMA_ERR_INCOMPATIBLE_INDEX_DEFINITIONS     ((uima::TyErrorId)(   54 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_INCOMPATIBLE_INDEX_DEFINITIONS, _TEXT("UIMA_ERR_INCOMPATIBLE_INDEX_DEFINITIONS") },
#endif

#define UIMA_ERR_WRONG_DESERIALIZED_DATA     ((uima::TyErrorId)(   55 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_WRONG_DESERIALIZED_DATA, _TEXT("UIMA_ERR_WRONG_DESERIALIZED_DATA") },
#endif

#define UIMA_ERR_INCOMPATIBLE_PARENT_TYPES     ((uima::TyErrorId)(   56 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_INCOMPATIBLE_PARENT_TYPES, _TEXT("UIMA_ERR_INCOMPATIBLE_PARENT_TYPES") },
#endif

#define UIMA_ERR_JAVA_EXCEPTION     ((uima::TyErrorId)(   57 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_JAVA_EXCEPTION, _TEXT("UIMA_ERR_JAVA_EXCEPTION") },
#endif

#define UIMA_ERR_COULD_NOT_LOAD_JAVA_DLL     ((uima::TyErrorId)(   58 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_COULD_NOT_LOAD_JAVA_DLL, _TEXT("UIMA_ERR_COULD_NOT_LOAD_JAVA_DLL") },
#endif

#define UIMA_ERR_ENGINE_INCOMPATIBLE_CAS     ((uima::TyErrorId)(   59 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_INCOMPATIBLE_CAS, _TEXT("UIMA_ERR_ENGINE_INCOMPATIBLE_CAS") },
#endif

#define UIMA_ERR_ENGINE_NO_CAS     ((uima::TyErrorId)(   60 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_ENGINE_NO_CAS, _TEXT("UIMA_ERR_ENGINE_NO_CAS") },
#endif

#define UIMA_ERR_DUPLICATE_EXISTS     ((uima::TyErrorId)(   61 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_DUPLICATE_EXISTS, _TEXT("UIMA_ERR_DUPLICATE_EXISTS") },
#endif

#define UIMA_ERR_INVALID_FEATURE_VALUE_BEGIN     ((uima::TyErrorId)(   62 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_INVALID_FEATURE_VALUE_BEGIN, _TEXT("UIMA_ERR_INVALID_FEATURE_VALUE_BEGIN") },
#endif


#define UIMA_ERR_INVALID_FEATURE_VALUE_END     ((uima::TyErrorId)(   63 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_INVALID_FEATURE_VALUE_END, _TEXT("UIMA_ERR_INVALID_FEATURE_VALUE_END") },
#endif

    /** Could not define the CAS Definition. */
#define UIMA_ERR_CASPOOL_CREATE_CASDEFINITION        ((uima::TyErrorId)(  64 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CASPOOL_CREATE_CASDEFINITION, _TEXT("UIMA_ERR_CASPOOL_CREATE_CASDEFINITION") },
#endif

    /** Could not define the CASPool from the CAS Definition. */
#define UIMA_ERR_CASPOOL_CREATE_CAS                 ((uima::TyErrorId)(  65 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CASPOOL_CREATE_CAS, _TEXT("UIMA_ERR_CASPOOL_CREATE_CAS") },
#endif

    /** Could  not get stream access sofa data  */
#define UIMA_ERR_SOFADATASTREAM                ((uima::TyErrorId)(  66 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_SOFADATASTREAM , _TEXT("UIMA_ERR_SOFADATASTREAM") },
#endif

    /** Could not process and output a CAS. */
#define UIMA_ERR_PROCESS_OUTPUT_CAS                ((uima::TyErrorId)(  67 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_PROCESS_OUTPUT_CAS , _TEXT("UIMA_ERR_PROCESS_OUTPUT_CAS") },
#endif

    /** Codepage conversion errors */
#define UIMA_ERR_CODEPAGE                ((uima::TyErrorId)(  68 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CODEPAGE , _TEXT("UIMA_ERR_CODEPAGE") },
#endif

    /** Codepage conversion errors */
#define UIMA_ERR_INVALID_BASE_CAS_METHOD  ((uima::TyErrorId)(  69 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CODEPAGE , _TEXT("UIMA_ERR_INVALID_BASE_CAS_METHOD") },
#endif

    /** CASPool errors */
#define UIMA_ERR_CASPOOL_GET_CAS  ((uima::TyErrorId)(  70 + UIMA_ERR_ENGINE_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CODEPAGE , _TEXT("UIMA_ERR_CASPOOL_GET_CAS") },
#endif


    /*@}*/



    /** @name UIMA_ERR_DOCUMENT ... */
    /*@{*/
#define UIMA_ERR_DOCUMENT_OFFSET                             ((uima::TyErrorId)(7000 + UIMA_ERROR_USER_FIRST))
    /** An attempt has been made to create a document reference, or document position
        with an invalid document index. */
#define UIMA_ERR_DOCUMENT_INVALID_INDEX                      ((uima::TyErrorId)(   0 + UIMA_ERR_DOCUMENT_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_DOCUMENT_INVALID_INDEX, _TEXT("UIMA_ERR_DOCUMENT_INVALID_INDEX") },
#endif

    /** An attempt has been made to create an empty document reference, such as
        a document reference with a length 0. */
#define UIMA_ERR_DOCUMENT_EMPTY_REFERENCE                    ((uima::TyErrorId)(   2 + UIMA_ERR_DOCUMENT_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_DOCUMENT_EMPTY_REFERENCE, _TEXT("UIMA_ERR_DOCUMENT_EMPTY_REFERENCE") },
#endif

    /** An attempt has been made to extract a contiguous document text from a document
        reference using the<TT>DocReference::copyTextToUString()</TT> method.
        Contiguous document text must be extracted from the document using
        the <TT>DocReference::getAsULString()</TT> method. */
#define UIMA_ERR_DOCUMENT_INVAL_COPY_OP                      ((uima::TyErrorId)(   3 + UIMA_ERR_DOCUMENT_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_DOCUMENT_INVAL_COPY_OP, _TEXT("UIMA_ERR_DOCUMENT_INVAL_COPY_OP") },
#endif

    /** Error during document parsing error . */
#define UIMA_ERR_DOCUMENT_PARSING                            ((uima::TyErrorId)(   4 + UIMA_ERR_DOCUMENT_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_DOCUMENT_PARSING, _TEXT("UIMA_ERR_DOCUMENT_PARSING") },
#endif

    /*@}*/

    /** @name UIMA_ERR_CPCONVERSION ... */
    /*@{*/
#define UIMA_ERR_CPCONVERSION_OFFSET                         ((uima::TyErrorId)(8000 + UIMA_ERROR_USER_FIRST))
    /** An invalid conversion function has been invoked.
        This is an internal error and should be reported to the UIMA team. */
#define UIMA_ERR_CPCONVERSION_ILLEGAL_CALL                   ((uima::TyErrorId)(   0 + UIMA_ERR_CPCONVERSION_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CPCONVERSION_ILLEGAL_CALL, _TEXT("UIMA_ERR_CPCONVERSION_ILLEGAL_CALL") },
#endif

    /** An attempt has been made to convert a buffer of bytes from one CCSID to another,
        where the target buffer for the conversion was not large enough to hold all characters. */
#define UIMA_ERR_CPCONVERSION_OVERFLOW                       ((uima::TyErrorId)(   1 + UIMA_ERR_CPCONVERSION_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CPCONVERSION_OVERFLOW, _TEXT("UIMA_ERR_CPCONVERSION_OVERFLOW") },
#endif

    /** The source buffer for a character conversion from one CCSID to another
        contains an invalid UTF-8 or DBCS sequence. */
#define UIMA_ERR_CPCONVERSION_INVALID_SRCSEQ                 ((uima::TyErrorId)(   2 + UIMA_ERR_CPCONVERSION_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CPCONVERSION_INVALID_SRCSEQ, _TEXT("UIMA_ERR_CPCONVERSION_INVALID_SRCSEQ") },
#endif

    /** The source buffer for a character conversion from one CCSID to another
        contains an DBCS sequence. */
#define UIMA_ERR_CPCONVERSION_INVALID_DBCS_SRC               ((uima::TyErrorId)(   3 + UIMA_ERR_CPCONVERSION_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CPCONVERSION_INVALID_DBCS_SRC, _TEXT("UIMA_ERR_CPCONVERSION_INVALID_DBCS_SRC") },
#endif

    /** An illegal attempt has been made to use a converter. Before the converter
        can be used, a call to method <TT>ConverterABase::isSupported()</TT>
        must be performed in order to detect whether the converter can be used
        or not. */
#define UIMA_ERR_CPCONVERSION_NOT_SUPPORTED                  ((uima::TyErrorId)(   4 + UIMA_ERR_CPCONVERSION_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_CPCONVERSION_NOT_SUPPORTED, _TEXT("UIMA_ERR_CPCONVERSION_NOT_SUPPORTED") },
#endif
    /*@}*/

    /** @name UIMA_ERR_USER_ANNOTATOR ... */
    /*@{*/
#define UIMA_ERR_USER_ANNOTATOR_OFFSET                          ((uima::TyErrorId)(9000 + UIMA_ERROR_USER_FIRST))
    /** The user annotator could not allocate enough memory for its mission. */
#define UIMA_ERR_USER_ANNOTATOR_OUT_OF_MEMORY                   ((uima::TyErrorId)(   0 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_OUT_OF_MEMORY, _TEXT("UIMA_ERR_USER_ANNOTATOR_OUT_OF_MEMORY") },
#endif

    /** The user annotator could not find or allocate a vital resource. */
#define UIMA_ERR_USER_ANNOTATOR_RESOURCE_NOT_FOUND              ((uima::TyErrorId)(   1 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_RESOURCE_NOT_FOUND, _TEXT("UIMA_ERR_USER_ANNOTATOR_RESOURCE_NOT_FOUND") },
#endif

    /** The user annotator could not read a vital resource. */
#define UIMA_ERR_USER_ANNOTATOR_RESOURCE_NOT_READABLE          ((uima::TyErrorId)(    2 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_RESOURCE_NOT_READABLE, _TEXT("UIMA_ERR_USER_ANNOTATOR_RESOURCE_NOT_READABLE") },
#endif

    /** The user annotator detected that one of its vital resources is corrupted and therefore, cannot be used or loaded. */
#define UIMA_ERR_USER_ANNOTATOR_RESOURCE_CORRUPTED              ((uima::TyErrorId)(   3 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_RESOURCE_CORRUPTED, _TEXT("UIMA_ERR_USER_ANNOTATOR_RESOURCE_CORRUPTED") },
#endif

    /** The user annotator detected that one of its vital resources is empty and therefore, cannot be used or loaded. */
#define UIMA_ERR_USER_ANNOTATOR_RESOURCE_EMPTY                  ((uima::TyErrorId)(   4 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_RESOURCE_EMPTY, _TEXT("UIMA_ERR_USER_ANNOTATOR_RESOURCE_EMPTY") },
#endif

    /** The user annotator detected that it cannot execute due to unfulfilled prerequisites. */
#define UIMA_ERR_USER_ANNOTATOR_PREREQ_VIOLATION                ((uima::TyErrorId)(   5 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_PREREQ_VIOLATION, _TEXT("UIMA_ERR_USER_ANNOTATOR_PREREQ_VIOLATION") },
#endif

    /** The user annotator could not process due to a general I/O problem. */
#define UIMA_ERR_USER_ANNOTATOR_IO_PROBLEM                      ((uima::TyErrorId)(   6 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_IO_PROBLEM, _TEXT("UIMA_ERR_USER_ANNOTATOR_IO_PROBLEM") },
#endif

    /** The user annotator could not process due to a general write problem. */
#define UIMA_ERR_USER_ANNOTATOR_IO_WRITE_PROBLEM                ((uima::TyErrorId)(   7 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_IO_WRITE_PROBLEM, _TEXT("UIMA_ERR_USER_ANNOTATOR_IO_WRITE_PROBLEM") },
#endif

    /** The user annotator could not process due to a general read problem. */
#define UIMA_ERR_USER_ANNOTATOR_IO_READ_PROBLEM                 ((uima::TyErrorId)(   8 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_IO_READ_PROBLEM, _TEXT("UIMA_ERR_USER_ANNOTATOR_IO_READ_PROBLEM") },
#endif

    /** The user annotator could not process due to a general I/O permission problem. */
#define UIMA_ERR_USER_ANNOTATOR_IO_PERMISSION_DENIED            ((uima::TyErrorId)(   9 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_IO_PERMISSION_DENIED, _TEXT("UIMA_ERR_USER_ANNOTATOR_IO_PERMISSION_DENIED") },
#endif

    /** The user annotator could not process due to an I/O resource, which is unavailable. */
#define UIMA_ERR_USER_ANNOTATOR_IO_NOT_EXISTENT                 ((uima::TyErrorId)(  10 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_IO_NOT_EXISTENT, _TEXT("UIMA_ERR_USER_ANNOTATOR_IO_NOT_EXISTENT") },
#endif

    /** The user annotator could not process due to an I/O resource, which by exclusive access is already locked
        by another process. */
#define UIMA_ERR_USER_ANNOTATOR_IO_ALREADY_IN_USE               ((uima::TyErrorId)(  11 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_IO_ALREADY_IN_USE, _TEXT("UIMA_ERR_USER_ANNOTATOR_IO_ALREADY_IN_USE") },
#endif

    /** The user annotator could not process due to an I/O resource, which by exclusive access is still locked
        by another process. */
#define UIMA_ERR_USER_ANNOTATOR_IO_STILL_IN_USE                 ((uima::TyErrorId)(  12 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_IO_STILL_IN_USE, _TEXT("UIMA_ERR_USER_ANNOTATOR_IO_STILL_IN_USE") },
#endif

    /** The user annotator could not process due to an invalid I/O resource name. */
#define UIMA_ERR_USER_ANNOTATOR_IO_INVALID_NAME                 ((uima::TyErrorId)(  13 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_IO_INVALID_NAME, _TEXT("UIMA_ERR_USER_ANNOTATOR_IO_INVALID_NAME") },
#endif

    /** The user annotator could not initialize itself. */
#define UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT                  ((uima::TyErrorId)(  14 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT, _TEXT("UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT") },
#endif

    /** The user annotator could not process document data. */
#define UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS               ((uima::TyErrorId)(  15 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS, _TEXT("UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS") },
#endif

    /** The user annotator could not reconfigure itself. */
#define UIMA_ERR_USER_ANNOTATOR_COULD_NOT_CONFIG                ((uima::TyErrorId)(  16 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_COULD_NOT_CONFIG, _TEXT("UIMA_ERR_USER_ANNOTATOR_COULD_NOT_CONFIG") },
#endif

    /** The user annotator could not deinitialize itself. */
#define UIMA_ERR_USER_ANNOTATOR_COULD_NOT_DEINIT                ((uima::TyErrorId)(  17 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_COULD_NOT_DEINIT, _TEXT("UIMA_ERR_USER_ANNOTATOR_COULD_NOT_DEINIT") },
#endif

    /** The user annotator is missing a vital parameter from the annotator configuration. */
#define UIMA_ERR_USER_ANNOTATOR_CONFIG_MISSING_PARAM            ((uima::TyErrorId)(  18 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_CONFIG_MISSING_PARAM, _TEXT("UIMA_ERR_USER_ANNOTATOR_CONFIG_MISSING_PARAM") },
#endif

    /** The user annotator detected an invalid parameter in the annotator configuration. */
#define UIMA_ERR_USER_ANNOTATOR_CONFIG_INVALID_PARAM            ((uima::TyErrorId)(  19 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_CONFIG_INVALID_PARAM, _TEXT("UIMA_ERR_USER_ANNOTATOR_CONFIG_INVALID_PARAM") },
#endif

    /** The user annotator detected an in a legacy subsystem. */
#define UIMA_ERR_USER_ANNOTATOR_ERROR_IN_SUBSYSTEM            ((uima::TyErrorId)(  20 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_ERROR_IN_SUBSYSTEM, _TEXT("UIMA_ERR_USER_ANNOTATOR_ERROR_IN_SUBSYSTEM") },
#endif

    /** The user annotator has an unspecified error. */
#define UIMA_ERR_USER_ANNOTATOR_ERROR_UNSPECIFIED             ((uima::TyErrorId)(  21 + UIMA_ERR_USER_ANNOTATOR_OFFSET))
#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_USER_ANNOTATOR_ERROR_UNSPECIFIED,  _TEXT("UIMA_ERR_USER_ANNOTATOR_ERROR_UNSPECIFIED") },
#endif
    /*@}*/

#ifdef UIMA_ENGINE_MAIN_CPP
    { UIMA_ERR_NONE, _TEXT("UIMA_ERR_NONE") }
  };
#endif

} //namespace uima


/* ----------------------------------------------------------------------- */
#endif /* UIMA_ERR_IDS_H */

/* <EOF> */

