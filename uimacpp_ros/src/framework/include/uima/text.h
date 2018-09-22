/** \file text.h .
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

    \brief Types and operations for text C strings (Support for UNICODE, SBCS and DBCS)


-------------------------------------------------------------------------- */

#ifndef __UIMA_TEXT_H
#define __UIMA_TEXT_H

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation dependencies                                       */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

#if defined(UNICODE) && !defined(_UNICODE)
#define _UNICODE
#endif

// A couple of text macros for other than MS VC++

/** @name UNICODE character support */
/*@{*/
#ifdef _MSC_VER

#include <tchar.h>

#else /* all other platforms */

/* support for "TCHAR", "_TEXT", and "_T" in case there is no UNICODE support */
#ifdef _UNICODE
#error Conflict, current host OS supports UNICODE
#endif

#if defined(__tchar_h) || defined(__tchar) || defined(__tint) || defined(_INC_TCHAR)
#error Conflict, current host OS supports TCHAR
#endif

/* for UNICODE characters use the TCHAR type */

typedef char TCHAR;

/* for UNICODE text constants use the _TEXT() macro */

#define _TEXT(x)    x

#endif
/*@}*/

/* ----------------------------------------------------------------------- */
/*       Globals                                                           */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Function declarations                                             */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Macro definitions                                                 */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Private implementation                                            */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Protected implementation                                          */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Public implementation                                             */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

#endif /* __UIMA_TEXT_H */

/* <EOF> */
