/** \file macros.h .
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

   \brief Common Macros

-------------------------------------------------------------------------- */

#ifndef __UIMA_MACROS_H
#define __UIMA_MACROS_H

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */

#include <string.h>

/* ----------------------------------------------------------------------- */
/*       Implementation dependencies                                       */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

// XML namespace used as initial value for schemaLocation attribute pair

///#define UIMA_XML_NAMESPACE       "http://uima.apache.org/resourceSpecifier"

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Globals                                                           */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Function declarations                                             */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Macro definitions                                                 */
/* ----------------------------------------------------------------------- */

/** @name Common macros */
/*@{*/

/** check whether something exists (= is unequal to zero) */
#ifndef EXISTS
#define EXISTS(x) ((x) != 0)
#endif

/** check whether something does not exist (= is zero) */
#ifndef NOTEXISTS
#define NOTEXISTS(x)  ((x) == 0)
#endif

/** Dump in hex format (WORD16 units) */
#ifdef DUMP_HEX
#define DUMPHEX(ostr,addr,len) dumpHex(ostr,addr,2*len)
#else
#define DUMPHEX(ostr,addr,len)
#endif

/** a loop-forever statement */
#ifndef FOREVER
#define FOREVER               for(;;)
#endif

/* Not needed! */
#ifdef NEVER
/** the ususal MINimum value macro - not type-safe! */
#ifndef MIN
#define MIN(x,y)              ((x) < (y) ? (x) : (y))
#endif

/** the ususal MAXimum value macro - not type-safe! */
#ifndef MAX
#define MAX(x,y)              ((x) > (y) ? (x) : (y))
#endif
/*@}*/
#endif  /* NEVER */

/** @name Useful macros */
/*@{*/

/** evaluate the number of elements in an array of any type */
#ifndef NUMBEROF
#define NUMBEROF(array)   (sizeof(array)/sizeof(array[0]))
#endif

/** stringification of macro argument.
macro argument will be converted into string constant.
if the argument itself is a macro, it will be expanded before
stringification */
#ifndef UIMA_STRINGIFY
#define UIMA_STRINGIFY(s)      UIMA_IMP_STRINGIFY(s)
#endif
#ifndef UIMA_IMP_STRINGIFY
#define UIMA_IMP_STRINGIFY(s)  # s
#endif

/* Linkage for bsearch callack in taf_htmlparser.cpp */
#if defined(__GNUC__)
#define UIMA_LINK_CONV_CRT     static int
#elif defined(_MSC_VER)
#define UIMA_LINK_CONV_CRT     static int __cdecl
#else
#error Code requires port to host Compiler!
#endif

/*@}*/

#ifdef USE_TPRINT
# ifdef __FUNCTION__
#  define UIMA_TPRINT(f)            cout << "[" << __FUNCTION__ << " : " << __LINE__ <<" ] " << f <<endl;
# else
#  define UIMA_TPRINT(f)            cout << "[" << __FILE__ << ") : " << __LINE__ <<" ] " << f <<endl;
# endif
#else
#define UIMA_TPRINT(f)
#endif

/* ----------------------------------------------------------------------- */
/*       Cast operators                                                    */
/* ----------------------------------------------------------------------- */

/** @name C++ Macros for compilers which do not support cast operators */
/*@{*/
#ifdef __cplusplus
#define STATIC_CAST(type, expr)        static_cast<type>(expr)
#define CONST_CAST(type, expr)         const_cast<type>(expr)
#define REINTERPRET_CAST(type, expr)   reinterpret_cast<type>(expr)
#define DYNAMIC_CAST(type, expr)       dynamic_cast<type>(expr)
#else   /* __cplusplus */
#define STATIC_CAST(type, expr)        (type) (expr)
#define CONST_CAST(type, expr)         (type) (expr)
#define REINTERPRET_CAST(type, expr)   (type) (expr)

/* dynamic_cast does not make sense in pure C */
#endif  /* __cplusplus */

/*@}*/

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

#endif /* __UIMA_MACROS_H */

/* <EOF> */
