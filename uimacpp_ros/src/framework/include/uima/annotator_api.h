/** \file annotator_api.h .
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

    \brief  Contains the function declarations for a UIMACPP Annotator

   Description:

-----------------------------------------------------------------------------


   5/3/1999    Initial creation
   5/10/1999   New Annotator method tafAnnotatorCreate

-------------------------------------------------------------------------- */

#ifndef UIMA_ANNOTATOR_API_H
#define UIMA_ANNOTATOR_API_H

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas

#include <uima/err_ids.h>
#include <uima/cas.hpp>
#include <uima/types.h>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

typedef uima::CAS * TyCasHandle;

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

#ifdef __cplusplus
namespace uima {
  class AnnotatorContext;
  class ResultSpecification;
  class Language;
  class CAS;
  class TypeSystem;
}
#endif

// Compiler-dependent link dcls
#if defined(__GNUC__)
#  define UIMA_ANNOTATOR_LINK_SPEC
#  define UIMA_ANNOTATOR_LINK_IMPORTSPEC
#elif defined(_MSC_VER)
#  define UIMA_ANNOTATOR_LINK_SPEC   __cdecl
#  define UIMA_ANNOTATOR_LINK_IMPORTSPEC    __declspec(dllexport)
#else
#  error Code requires port to host Compiler!
#endif

#ifdef __cplusplus
extern "C" {
#endif

  UIMA_ANNOTATOR_LINK_IMPORTSPEC uima::TyHandle  UIMA_ANNOTATOR_LINK_SPEC tafAnnotatorCreate(void);
  UIMA_ANNOTATOR_LINK_IMPORTSPEC uima::TyErrorId UIMA_ANNOTATOR_LINK_SPEC tafAnnotatorInit(uima::AnnotatorContext &, uima::TyHandle);
  UIMA_ANNOTATOR_LINK_IMPORTSPEC uima::TyErrorId UIMA_ANNOTATOR_LINK_SPEC tafAnnotatorDeInit(uima::TyHandle);
  UIMA_ANNOTATOR_LINK_IMPORTSPEC uima::TyErrorId UIMA_ANNOTATOR_LINK_SPEC tafAnnotatorTypeSystemInit(uima::TypeSystem const &, uima::TyHandle);
  UIMA_ANNOTATOR_LINK_IMPORTSPEC uima::TyErrorId UIMA_ANNOTATOR_LINK_SPEC tafAnnotatorProcessDocument(uima::CAS &, const uima::ResultSpecification &, uima::TyHandle);
  UIMA_ANNOTATOR_LINK_IMPORTSPEC uima::TyErrorId UIMA_ANNOTATOR_LINK_SPEC tafAnnotatorReConfig(uima::TyHandle);
  UIMA_ANNOTATOR_LINK_IMPORTSPEC uima::TyErrorId UIMA_ANNOTATOR_LINK_SPEC tafAnnotatorBatchProcessComplete(uima::TyHandle);
  UIMA_ANNOTATOR_LINK_IMPORTSPEC uima::TyErrorId UIMA_ANNOTATOR_LINK_SPEC tafAnnotatorCollectionProcessComplete(uima::TyHandle);
  UIMA_ANNOTATOR_LINK_IMPORTSPEC bool           UIMA_ANNOTATOR_LINK_SPEC tafAnnotatorHasNext(uima::TyHandle);
  UIMA_ANNOTATOR_LINK_IMPORTSPEC uima::CAS *     UIMA_ANNOTATOR_LINK_SPEC tafAnnotatorNext(uima::TyHandle);
  UIMA_ANNOTATOR_LINK_IMPORTSPEC int            UIMA_ANNOTATOR_LINK_SPEC tafAnnotatorGetCasInstancesRequired(uima::TyHandle);


#ifdef __cplusplus
}
#endif

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */

#endif /* UIMA_ANNOTATOR_API_H */

/* <EOF> */

