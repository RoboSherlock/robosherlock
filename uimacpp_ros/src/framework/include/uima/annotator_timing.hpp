/** \file annotator_timing.hpp .
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

    \brief  Contains macro definitions for annotator timing functions

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */

#ifndef UIMA_ANNOTATOR_TIMING_HPP
#define UIMA_ANNOTATOR_TIMING_HPP

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> //must be included first to disable warnings

/* ----------------------------------------------------------------------- */
/*       Implementation dependencies                                       */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Macros                                                            */
/* ----------------------------------------------------------------------- */

/**
   This is a compile time switch which we turn on (define) for now.
   It enables the output of timing data in the trace file.
   If this seems to performance consuming we can just comment out the
   #define UIMA_DEBUG_ANNOTATOR_TIMING
   statement.
   But note that UIMA_DEBUG_ANNOTATOR_TIMING must always be defined if the larger
   DEBUG_TIMING is turned on for a specific timing driver

   All timing messages can be suppressed for reproducible regression tests.
*/

#ifndef UIMA_SUPPRESS_TIMING
#define UIMA_DEBUG_ANNOTATOR_TIMING
#endif

// UIMA_DEBUG_ANNOTATOR_TIMING must be defined if the larger DEBUG_TIMING is turned on
#if defined(DEBUG_TIMING) && !defined(UIMA_DEBUG_ANNOTATOR_TIMING)
#define UIMA_DEBUG_ANNOTATOR_TIMING
#endif

#ifdef UIMA_DEBUG_ANNOTATOR_TIMING
#  ifdef UIMA_ANNOTATOR_TIMING
#     undef UIMA_ANNOTATOR_TIMING
#  endif
/** A macro in which our timing statements are hidden,
   if UIMA_DEBUG_ANNOTATOR_TIMING is <EM>not</EM> defined.
   Using a timer <TT>clTimer</TT> of type <TT>Timer</TT> the macro can be used
   in statements like:
   <TT>UIMA_ANNOTATOR_TIMING(clTimer.start)</TT> and <TT>UIMA_ANNOTATOR_TIMING(clTimer.start)</TT>
*/
#  define UIMA_ANNOTATOR_TIMING( expr ) ( expr )
#else
#  ifdef UIMA_ANNOTATOR_TIMING
#     undef UIMA_ANNOTATOR_TIMING
#  endif
#  define UIMA_ANNOTATOR_TIMING( ignore ) ( ( void )0 )
#endif


#endif /* UIMA_ANNOTATOR_TIMING_HPP */

/* <EOF> */

