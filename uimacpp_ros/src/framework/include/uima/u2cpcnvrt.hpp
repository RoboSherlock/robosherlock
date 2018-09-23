/** \file u2cpcnvrt.hpp .
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

    \brief  Contains Unicode2CodePageConverter a class to convert from UNICODE to several other codepages

   Description:

-----------------------------------------------------------------------------


   5/21/1999  Initial creation

-------------------------------------------------------------------------- */

#ifndef UIMA_U2CPCNVRT_HPP
#define UIMA_U2CPCNVRT_HPP

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <uima/exceptions.hpp>
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
   * The class <TT>Unicode2CodePageConverter</TT> is used to convert all
   * characters in a specified buffer from UNICODE (CCSID: CCSID::EnCCSID_UCS2),
   * to another buffer with specified CCSID.
   * \code
     \endcode
   * @see CodePage2UnicodeConverter
   */
  class UIMA_LINK_IMPORTSPEC Unicode2CodePageConverter  {
  public:
    /** @name Constructors */
    /*@{*/
    Unicode2CodePageConverter(const char * converterName);
    ~Unicode2CodePageConverter();
    /*@}*/
    /** @name Miscellaneous */
    /*@{*/

    /** the maximum size in bytes required when converted from Unicode to
        using this code page converter */
    size_t getMaximumSize(const UChar * cpclSource,
                          size_t uiSourceLength);

    /** Convert the specified source buffer <TT>cpacSource</TT>, with size
        <TT>uiSourceSize</TT> in bytes to target buffer <TT>pclTarget</TT>,
        with size <TT>uiTargetMaxSize</TT> in bytes from the UNICODE using 
        this codepage converter. Return the number of bytes converted. */
    size_t convertFromUnicode(char * pacTarget,
                              size_t uiTargetMaxLength,
                              const UChar * cpclSource,
                              size_t uiSourceLength);
    /*@}*/

  protected:
    /* --- functions --- */
  private:
    /* --- functions --- */
    /* BASE CONSTRUCTOR NOT SUPPORTED */
    Unicode2CodePageConverter(void); //lint !e1704
    /* COPY CONSTRUCTOR NOT SUPPORTED */
    Unicode2CodePageConverter(const Unicode2CodePageConverter & ); //lint !e1704
    /* ASSIGNMENT OPERATOR NOT SUPPORTED */
    Unicode2CodePageConverter & operator=(const Unicode2CodePageConverter & crclObject);
    /* --- variables --- */
    UConverter * iv_uconverter;
  }
  ; /* Unicode2CodePageConverter */
}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

#endif /* UIMA_U2CPCNVRT_HPP */

/* <EOF> */


