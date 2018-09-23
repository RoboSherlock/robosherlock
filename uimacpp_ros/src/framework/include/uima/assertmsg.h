/** \file assertmsg.h .
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

   \brief An extension of the assert routine to print a message on failure

-------------------------------------------------------------------------- */

#ifndef __UIMA_ASSERTMSG_H
#define __UIMA_ASSERTMSG_H

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */

#include <assert.h>

/* ----------------------------------------------------------------------- */
/*       Implementation dependencies                                       */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

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

/**
   <tt> assert</tt> prints a diagnostic message to stderr and aborts the program
   if expression is false (zero).

   Use <tt> assert</tt> to identify program logic errors.
   Choose an expression that holds true only if the program is operating as
   you intend.  After you have debugged the program, you can use the special
   no-debug identifier NDEBUG to remove the <tt> assert</tt> calls from the program.
   If you define NDEBUG to any value with a \#define directive, the C
   preprocessor expands all <tt> assert</tt> invocations to void expressions.
   If you use NDEBUG, you must define it before you include "uima/assertmsg.h" in
   the program.

   There is no return value.

   Example:
   @code
   foo(const void * cpvP1, const void * cpvP2, int i)
   {
      assert(EXISTS(cpvP1));
      assert(EXISTS(cpvP2));
      assert((i > 0) && (i < 100));
      ...
   }
   @endcode

   \note <tt> assert</tt> is no longer implemented here ... the system macro is used.
   Do not use the \#undef directive with <tt> assert</tt>.

   

   <tt> assertWithMsg</tt> prints a diagnostic message to stderr and aborts the
   program if expression is false (zero).

   <tt> assertWithMsg</tt> has the same purpose as <tt> assert</tt> with the
   additional feature that you can pass an additional message string to
   the function which gets displayed in case the specified expression
   evaluates to false (zero).

   Use <tt> assertWithMsg</tt> to identify program logic errors.
   Choose an expression that holds true only if the program is operating as
   you intend.  After you have debugged the program, you can use the special
   no-debug identifier NDEBUG to remove the <tt> assertWithMsg</tt> calls from the
   program. If you define NDEBUG to any value with a \#define directive, the
   C preprocessor expands all <tt> assertWithMsg</tt> invocations to void
   expressions.
   If you use NDEBUG, you must define it before you include "uima/assertmsg.h" in
   the program.

   There is no return value.

   Example:
   @code
   foo(const void * cpvP1, const void * cpvP2, int i)
   {
      assert(EXISTS(cpvP1));
      assert(EXISTS(cpvP2));
      assertWithMsg(((i > 0) && (i < 100)), "invalid value specified");
      ...
   }
   @endcode
   \note <tt> assertWithMsg</tt> is implemented as a macro.
   Do not use the \#undef directive with <tt> assertWithMsg</tt>.

   @see assert
*/

#ifndef NDEBUG

#ifdef assertWithMsg
#undef assertWithMsg
#endif

// plugin_annotator_test would not link when used C++ I/O !
//     e.g.  if (!(expr)) {cerr<<"Assert msg:"<<msg<<endl;assert(expr);}
#define assertWithMsg(expr,msg) if (!(expr)) {fprintf(stderr,"Assert msg: %s\n",msg);assert(expr);}

#else   /* NDEBUG defined */

#ifdef assertWithMsg
#undef assertWithMsg
#endif
#define assertWithMsg( ignore, msg ) ( ( void )0 )

#endif

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

#endif /* __UIMA_ASSERTMSG_H */

/* <EOF> */
