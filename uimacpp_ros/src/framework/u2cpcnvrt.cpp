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


   5/21/1999  Initial creation

-------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/u2cpcnvrt.hpp>

#include <uima/comp_ids.h>

#include <uima/assertmsg.h>
#include <uima/trace.hpp>
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

  Unicode2CodePageConverter::Unicode2CodePageConverter(const char * crConverterName) :
      iv_uconverter(NULL)
      /* ----------------------------------------------------------------------- */
  {
    UErrorCode err=(UErrorCode)0;
    iv_uconverter =  ucnv_open(crConverterName, &err);
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

  Unicode2CodePageConverter::~Unicode2CodePageConverter( ) {
     ucnv_close(iv_uconverter) ;
  }
  size_t Unicode2CodePageConverter::getMaximumSize(const UChar * cpclSource,
      size_t uiSourceLength) {
    size_t                     uiTargetSize;
    assert(iv_uconverter !=NULL);
    UErrorCode err=(UErrorCode)0;
    uiTargetSize =  ucnv_fromUChars(iv_uconverter, NULL,
                                    0, cpclSource, uiSourceLength, &err);

    if (!U_SUCCESS(err) &&  err != U_BUFFER_OVERFLOW_ERROR) {
      ///cerr << "ERROR getMaximumSize() " << err << endl;
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_CODEPAGE_CONV_ERROR);
      errMsg.addParam(err);
      UIMA_EXC_THROW_NEW(CodePageConversionException,
                         UIMA_ERR_CODEPAGE,
                         errMsg,
                         UIMA_MSG_ID_CODEPAGE_CONV_ERROR,
                         ErrorInfo::unrecoverable);
    }

    assert(sizeof(char) == 1);
    return(uiTargetSize);                              /* as bytes */
  }

  size_t Unicode2CodePageConverter::convertFromUnicode(char * pacTarget,
      size_t uiTargetMaxLength,
      const UChar * cpclSource,
      size_t uiSourceLength)
  /* ----------------------------------------------------------------------- */
  {
    size_t                     uiTargetSize;
    assert(iv_uconverter !=NULL);
    UErrorCode err=(UErrorCode)0;
    uiTargetSize =  ucnv_fromUChars(iv_uconverter, pacTarget,
                                    uiTargetMaxLength,
                                    cpclSource, uiSourceLength, &err);

    if (!U_SUCCESS(err) &&  err != U_BUFFER_OVERFLOW_ERROR) {
      ///cerr << "ERROR getMaximumSize() " << err << endl;
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_CODEPAGE_CONV_ERROR);
      errMsg.addParam(err);
      UIMA_EXC_THROW_NEW(CodePageConversionException,
                         UIMA_ERR_CODEPAGE,
                         errMsg,
                         UIMA_MSG_ID_CODEPAGE_CONV_ERROR,
                         ErrorInfo::unrecoverable);
    }

    assert(sizeof(char) == 1);
    return(uiTargetSize);                              /* as bytes */
  }

}

/* <EOF> */


