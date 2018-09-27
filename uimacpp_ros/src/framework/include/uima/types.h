/** \file types.h .
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

    \brief  Contains types definitions used throughout UIMACPP

   Description:

-----------------------------------------------------------------------------


   5/11/1999  Initial creation

-------------------------------------------------------------------------- */

#ifndef UIMA_TYPES_H
#define UIMA_TYPES_H

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/configure.h>
#include <uima/pragmas.hpp>

#include <stdlib.h>

#include "apr.h"

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/** Importing and Exporting global objects, such as variables and functions. */
#if defined(__GNUC__)
#  define UIMA_LINK_IMPORTSPEC
#  define UIMA_ANNOTATOR_LINK_SPEC
#  define UIMA_ANNOTATOR_LINK_IMPORTSPEC
#elif defined(_MSC_VER)
#  define UIMA_ANNOTATOR_LINK_SPEC   __cdecl
#  if defined(_UIMA_LIBRARY_)
#     define UIMA_LINK_IMPORTSPEC    __declspec(dllexport)
#  define UIMA_ANNOTATOR_LINK_IMPORTSPEC    __declspec(dllimport)
#  else
#     define UIMA_LINK_IMPORTSPEC    __declspec(dllimport)
#  define UIMA_ANNOTATOR_LINK_IMPORTSPEC    __declspec(dllexport)
#  endif
#else
#  error Code requires port to host Compiler!
#endif

/** the type uima::TyHandle is used as a handle to a resource */
/** the type uima::ComponentId is used in trace msgs */

namespace uima {

  typedef long            TyDocTextDistance;

  typedef unsigned long   TyMessageId;

  typedef size_t          TyDocIndex;

  typedef void *          TyHandle;

  typedef unsigned short  TyComponentId;

} // namespace uima

// Types formerly defined in obsolete headers

typedef  apr_byte_t         WORD8;
typedef  apr_uint16_t       WORD16;
typedef  apr_uint32_t       WORD32;
typedef  apr_uint64_t       WORD64;
typedef  apr_int32_t        INT32;
typedef  apr_int64_t        INT64;

// "bool" is already defined on Windows (MS IDE 7.1.3088)

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */

#endif /* UIMA_TYPES_H */

/* <EOF> */

