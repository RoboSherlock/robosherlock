/** \file pragmas.hpp .
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

    \brief  Contains pragmas to be included in each file

-------------------------------------------------------------------------- */

#ifndef UIMA_PRAGMAS_HPP
#define UIMA_PRAGMAS_HPP

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */



#if defined( _MSC_VER )

// for level 4
#  pragma warning( disable: 4100 ) // unreferenced formal parameter
#  pragma warning( disable: 4127 ) // conditional expression is constant
#  pragma warning( disable: 4244 ) // conversion with possible loss of data
#  pragma warning( disable: 4267 ) // conversion, possible loss of data
#  pragma warning( disable: 4355 ) // this used in base initializer list
#  pragma warning( disable: 4389 ) // signed unsigned mismatch
#  pragma warning( disable: 4503 ) // decorated name length truncated
#  pragma warning( disable: 4511 ) // copy ctor could not be generated
#  pragma warning( disable: 4512 ) // assignment operator could not be generated
#  pragma warning( disable: 4702 ) // unreachable code
#  pragma warning( disable: 4786 ) // debug information truncated
// #  pragma warning( disable: 4788 ) // gone?
#  pragma warning( disable: 4804 ) // 'operation' : unsafe use of type 'bool' in operation
#  pragma warning( disable: 4251 ) // dll-interface
#  pragma warning( disable: 4275 ) // non dll-interface base class
#  pragma warning( disable: 4800 ) // forcing value to bool 'true' or 'false' (performance warning)

// special includes to get line information in memory leak output
#if !defined( NDEBUG ) && !defined(UIMA_NO_MAP_ALLOC)
//       Can't have _CRTDBG_MAP_ALLOC defined as it redefines malloc et al in
//       crtdbg.h BEFORE they are declared in malloc.h (which APR drags in)
//       malloc.h should have a matching ifndef _CRTDBG_MAP_ALLOC around the dcls
//#        ifdef CRTDBG_BUG_FIXED
// Appears to be fixed in VC++ 8 
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
//#endif
#endif

#elif defined(__BORLANDC__)
#pragma warn -inl
#endif

#include <uima/configure.h>
#include <uima/macros.h>         /* For EXISTS etc. */
#include <uima/assertmsg.h>

/* ----------------------------------------------------------------------- */
#endif /* UIMA_PRAGMAS_HPP */

/* <EOF> */


