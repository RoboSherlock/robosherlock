/** \file cp2ucnvrt.hpp .
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

    \brief  Contains CodePage2UnicodeConverter a class to convert from different codepages to UNICODE

   Description:

-----------------------------------------------------------------------------


   5/21/1999   Initial creation
   8/18/1999   CodePage2UTF8Converter added

-------------------------------------------------------------------------- */

#ifndef UIMA_CP2UCNVRT_HPP
#define UIMA_CP2UCNVRT_HPP

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
   * The class <TT>CodePage2UnicodeConverter</TT> converts all characters
   * in a specified buffer into UNICODE (CCSID: CCSID::EnCCSID_UCS2)
   *
   * @see Unicode2CodePageConverter CodePage2UTF8Converter
   */
  class UIMA_LINK_IMPORTSPEC CodePage2UnicodeConverter {
  public:
    /** @name Constructors */
    /*@{*/
    /** Create an instance for a converter from CCSID <TT>crclCCSID</TT> to
        CCSID CCSID::EnCCSID_UCS2. */
    CodePage2UnicodeConverter(const char * crConverterName);
    ~CodePage2UnicodeConverter();

    /** TODO: fix
     * implementation of convertBytes is probably wrong.
    * used in taf_doc_buffer.cpp and taf_htmlparser.cpp to support test suite
    *
    */
    /*@}*/
    /** @name Miscellaneous */
    /*@{*/
    /** From this CCSID, convert the specified source buffer <TT>cpacSource</TT> with size
        <TT>uiSourceSize</TT> in bytes, to target buffer <TT>pclTarget</TT>
        with size <TT>uiTargetMaxSize</TT> in bytes to the UNICODE
        CCSID (CCSID::EnCCSID_UCS2), and return
        the number of bytes converted. */
    size_t                     convertBytes(UChar * pclTarget, size_t uiTargetMaxSize, const char * cpacSource, size_t uiSourceSize) ;
    /*@}*/

    /** the maximum size in bytes required when converted  to Unicode
        using this code page converter */
    size_t getMaximumLength(const char * cpacSource, size_t uiSourceLength) const;

     /*@{*/
    /** Using this code page converter, convert the specified source buffer
     <TT>cpacSource</TT> with size <TT>uiSourceSize</TT> in bytes, to target 
    buffer <TT>pclTarget</TT> with size <TT>uiTargetMaxSize</TT> in bytes to 
    UNICODE and return the number of unicode characters converted. */
    size_t convertToUnicode(UChar * pclTarget, size_t uiTargetMaxLength,
                            const char * cpacSource, size_t uiSourceLength);



    /*@}*/
  protected:
    /* --- functions --- */
  private:
    /* --- functions --- */
    /* BASE CONSTRUCTOR NOT SUPPORTED */
    CodePage2UnicodeConverter(void); //lint !e1704
    /* COPY CONSTRUCTOR NOT SUPPORTED */
    CodePage2UnicodeConverter(const CodePage2UnicodeConverter & ); //lint !e1704
    /* ASSIGNMENT OPERATOR NOT SUPPORTED */
    CodePage2UnicodeConverter & operator=(const CodePage2UnicodeConverter & crclObject);
    /* --- variables --- */
    UConverter * iv_uconverter;
  }
  ; /* CodePage2UnicodeConverter */


}
/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */

#endif /* UIMA_CP2UCNVRT_HPP */

/* <EOF> */


