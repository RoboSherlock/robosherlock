/** @name cp2ucnvrt.cpp
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

   Description: This file contains class CodePage2UnicodeConverter

-----------------------------------------------------------------------------


   5/21/1999   Initial creation
   8/18/1999   CodePage2UTF8Converter added

-------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/cp2ucnvrt.hpp>

#include <uima/assertmsg.h>
#include <uima/msg.h>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Private Implementation                                            */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */
using namespace std;
namespace uima {

  CodePage2UnicodeConverter::CodePage2UnicodeConverter( const char * crConverterName) :
      iv_uconverter(NULL)
      /* ----------------------------------------------------------------------- */
  {
    UErrorCode err=(UErrorCode)0;
    iv_uconverter = ucnv_open(crConverterName, &err);
    if (!U_SUCCESS(err)) {
      cerr << "CodePage2UnicodeConverter ERROR could not open converter for " << crConverterName << endl;
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_CODEPAGE_CONV_ERROR);
      errMsg.addParam(err);
      errMsg.addParam(crConverterName);
      UIMA_EXC_THROW_NEW(CodePageConversionException,
                         UIMA_ERR_CODEPAGE,
                         errMsg,
                         UIMA_MSG_ID_CODEPAGE_CONV_ERROR,
                         ErrorInfo::unrecoverable);
    }
    assert(iv_uconverter != NULL);
  }



  CodePage2UnicodeConverter::~CodePage2UnicodeConverter( )
  {
      ucnv_close(iv_uconverter) ;
  }

  size_t CodePage2UnicodeConverter::convertBytes(UChar * pclTarget,
      size_t uiTargetMaxSize,
      const char * cpacSource,
      size_t uiSourceSize)
  /* ----------------------------------------------------------------------- */

  {
    size_t                     uiTargetSize;
    uiTargetSize = convertToUnicode(pclTarget, uiTargetMaxSize, cpacSource, uiSourceSize);
    return(uiSourceSize);
  }



  size_t CodePage2UnicodeConverter::getMaximumLength(const char * cpacSource,
      size_t uiSourceLength) const
  /* ----------------------------------------------------------------------- */
  {
    size_t                     uiTargetSize;

    assert(iv_uconverter !=NULL);
    UErrorCode err=(UErrorCode)0;
    uiTargetSize = ucnv_toUChars(iv_uconverter, NULL, 0, cpacSource, uiSourceLength, &err);

    if (!U_SUCCESS(err) && err != U_BUFFER_OVERFLOW_ERROR) {
      cerr << "CodePage2UnicodeConverter::getMaximumLength() rc= " << err << endl;
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_CODEPAGE_CONV_ERROR);
      errMsg.addParam(err);
      UIMA_EXC_THROW_NEW(CodePageConversionException,
                         UIMA_ERR_CODEPAGE,
                         errMsg,
                         UIMA_MSG_ID_CODEPAGE_CONV_ERROR,
                         ErrorInfo::unrecoverable);
    }

    ////return(uiTargetSize / sizeof(UChar));       /* as characters */
    return (uiTargetSize);
  }




  size_t CodePage2UnicodeConverter::convertToUnicode(UChar * pclTarget,
      size_t uiTargetMaxLength,
      const char * cpacSource,
      size_t uiSourceLength)
  /* ----------------------------------------------------------------------- */
  {
    size_t                     uiTargetSize;

    assert(iv_uconverter !=NULL);
    UErrorCode err=(UErrorCode)0;
    uiTargetSize = ucnv_toUChars(iv_uconverter, pclTarget, uiTargetMaxLength, cpacSource, uiSourceLength, &err);

    if (!U_SUCCESS(err) &&  err != U_BUFFER_OVERFLOW_ERROR) {
      cout << "ERROR: convertToUnicode " << err << endl;
      ///cerr << "CodePage2UnicodeConverter::getMaximumLength() rc= " << err << endl;
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_CODEPAGE_CONV_ERROR);
      errMsg.addParam(err);
      UIMA_EXC_THROW_NEW(CodePageConversionException,
                         UIMA_ERR_CODEPAGE,
                         errMsg,
                         UIMA_MSG_ID_CODEPAGE_CONV_ERROR,
                         ErrorInfo::unrecoverable);
    }
    return uiTargetSize;
    //// return(uiTargetSize / sizeof(UChar));       /* as characters */
  }

}
