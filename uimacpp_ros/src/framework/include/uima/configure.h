/** \file configure.h .
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

    \brief  Contains compiler dependent configuration defines.

   Description:

-----------------------------------------------------------------------------


   12/13/2001   Initial creation

-------------------------------------------------------------------------- */

#ifndef UIMA_CONFIGURE_H
#define UIMA_CONFIGURE_H

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Defines                                                           */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Compiler dependencies                                             */
/* ----------------------------------------------------------------------- */

#if   defined(__GNUC__)

#elif defined(_MSC_VER)
#  define UIMA_COMP_REQ_PUBLIC_TYPES
#  define UIMA_NO_HASH_CONTAINERS_SUPPORTED
#  include <minmax.h> // for min
// Disable MS warning C4290: C++ exception specification ignored except to indicate a function is not __declspec(nothrow)
#  pragma warning(disable:4290)

#else
#  error Code requires port to host OS!
#endif

#define UIMA_THROW0()           throw()
#define UIMA_THROW(x1)          throw(x1)


#ifdef __cplusplus
#include <string>
#endif

#ifndef U_OVERRIDE_CXX_ALLOCATION
// this is a ICU define
// since we currently do not support alternative memory management we put it here
#  define U_OVERRIDE_CXX_ALLOCATION 0
#endif

/* ----------------------------------------------------------------------- */
#endif /* UIMA_CONFIGURE_H */

/* <EOF> */

