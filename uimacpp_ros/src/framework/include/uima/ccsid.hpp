/** \file ccsid.hpp .
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

    \brief  Contains CCSID a class to represent a CCSID (Coded Character Set Identifiers)

   Description:

-----------------------------------------------------------------------------


   5/21/1999  Initial creation

-------------------------------------------------------------------------- */

#ifndef UIMA_CCSID_HPP
#define UIMA_CCSID_HPP

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp>
#include <uima/text.h>
#include "unicode/ucnv.h"

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  /**
   * The class <TT>CCSID</TT> used to represent a CCSID,
   * now a char* encoding string
   * @see
   */
  class CCSID {

  public:
    /// replaces CCSID::getSystemCCSID() - call ICU ucnv_getDefaultName()
    /// which is described as:
    /// Get the default converter name that is currently used by ICU and the operating system.
    /// TODO: this also replaces CCSid::getConsoleCCSID() is this the
    ///       right thing to do ?
    static char const * getDefaultName();

    /// return a string version of the default single byte input ccsid
    static char const * getDefaultSBCSInputCCSID( void );

    /// return a string version of the default single byte output ccsid
    static char const * getDefaultSBCSOutputCCSID( void );

    // Hide all constructors !? Replace the class with "const char* encoding"
  private:
    /** @name Constructors */
    /*@{*/
    /** Construct a CCSID based on a CCSID specification. */
    CCSID(long lCCSID);
    /** Construct a CCSID based on a CCSID specification as a C string. */
    CCSID(const char * cpszCCSID);
    /*@}*/
    /* --- functions --- */
    /* --- functions --- */
    /* BASE CONSTRUCTOR NOT SUPPORTED */
    CCSID(void); //lint !e1704
  }
  ; /* CCSID */
}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

namespace uima {

  /*  static */
  inline char const * CCSID::getDefaultName( void ) {
    return ucnv_getDefaultName();
  }

  // Assume ASII, not EBCDIC!

  /*static */
  inline char const * CCSID::getDefaultSBCSInputCCSID( void ) {
    return "UTF-8";
  }
  /*static */
  inline char const * CCSID::getDefaultSBCSOutputCCSID( void ) {
    return "ibm-819";
  }

} //namespace uima

#endif /* UIMA_CCSID_HPP */

/* <EOF> */

