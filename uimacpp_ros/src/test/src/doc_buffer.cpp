/** @name doc_buffer.cpp

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

-------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include "uima/doc_buffer.hpp"

#include "uima/ccsid.hpp"
#include "uima/macros.h"
#include "uima/macros.h"
#include "uima/trace.hpp"

#include "uima/cp2ucnvrt.hpp"
#include "uima/comp_ids.h"
#include "uima/err_ids.h"
#include "uima/msg.h"

#include <algorithm>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

#define UIMA_DOC_BUFFER_RESERVE_SIZE             (64 * 1024)

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Private                                                           */
/* ----------------------------------------------------------------------- */

namespace uima {

  void DocBuffer::addDocPartImp(const char * cpacDocPartText,
                                size_t uDocPartSizeInBytes,
                                CodePage2UnicodeConverter & crclConverter)
  /* ----------------------------------------------------------------------- */
  {
    size_t                  uEstPartSizeRequired;
    size_t                  uEstNewSize;
    size_t                  uCurrentSize;
    size_t                  uSizeConverted;
    size_t                  uSizeAvailable;
    UChar *                 pw16Target;

    assert(EXISTS(cpacDocPartText));
    assert(uDocPartSizeInBytes > 0);
    /////assert(crclConverter.isSupported());
    //ee      uEstPartSizeRequired = crclConverter.getMaximumLength(cpacDocPartText, uDocPartSizeInBytes);
    //ee - use the safest estimate
    uEstPartSizeRequired = sizeof(UChar) * uDocPartSizeInBytes;
    uCurrentSize = iv_uLength * sizeof(UChar);
    uEstNewSize = uCurrentSize + uEstPartSizeRequired;
    UIMA_TPRINT("input: uDocPartSizeInBytes: " << uDocPartSizeInBytes);
    UIMA_TPRINT("       uEstPartSizeRequired: " << uEstPartSizeRequired);
    UIMA_TPRINT("       uEstNewSize: " << uEstNewSize);
    UIMA_TPRINT("uCurrentSize: " << uCurrentSize);
    UIMA_TPRINT("iv_uSizeAllocated: " << iv_uSizeAllocated);

    /* we already have allocated the initial block in the ctor */
    /* check whether we need to re-allocate */
    if (uEstNewSize > iv_uSizeAllocated) {
      /* this block does not fit into the first block of the memory pool -
         we need to allocate a new block with no limitations by the pool */
      const UChar *        cpw16DocumentCurrent = iv_cpw16Document;

      iv_uSizeAllocated = uEstNewSize + iv_uMemPoolReserve;
      UIMA_TPRINT("*** new iv_uSizeAllocated: " << iv_uSizeAllocated << "***");
      iv_cpw16Document = (const UChar *) malloc(iv_uSizeAllocated);
      assert(EXISTS(iv_cpw16Document));
      /* and we need to copy the old document buffer into the newly allocated one */
      pw16Target = CONST_CAST(UChar *, iv_cpw16Document);
      memcpy((char *) pw16Target, (const char *) cpw16DocumentCurrent, uCurrentSize);
      free((void*)cpw16DocumentCurrent);            // Release too-small block
    }
    pw16Target = CONST_CAST(UChar *, (iv_cpw16Document + iv_uLength));
    uSizeAvailable = iv_uSizeAllocated - uCurrentSize;
    assert(EXISTS(pw16Target));
    assert(uSizeAvailable > 0);
    UIMA_TPRINT("uSizeAvailable: " << uSizeAvailable);
    uSizeConverted = crclConverter.convertBytes(pw16Target,
                     uSizeAvailable,
                     cpacDocPartText,
                     uDocPartSizeInBytes);
    UIMA_TPRINT("uSizeConverted: " << uSizeConverted);
    iv_uLength += (uSizeConverted / sizeof(UChar));
    UIMA_TPRINT("new iv_uiLength: " << iv_uLength);
  }

  void DocBuffer::resetMemPool(void)
  /* ----------------------------------------------------------------------- */
  {
    /* Allocate if necessary */
    if (iv_cpw16Document == 0) {
      iv_uSizeAllocated = iv_uMemPoolInitialSize;
      iv_cpw16Document = (const UChar *) malloc(iv_uSizeAllocated);
    }

    assert(EXISTS(iv_cpw16Document));
    iv_uLength = 0;
  }

  /* ----------------------------------------------------------------------- */
  /*       Public                                                            */
  /* ----------------------------------------------------------------------- */

  DocBuffer::DocBuffer() :
      iv_uMemPoolInitialSize(100000),
      iv_uMemPoolReserve(UIMA_DOC_BUFFER_RESERVE_SIZE),
      iv_cpw16Document(0),
      iv_uLength(0),
      iv_uSizeAllocated(0) {
    init();
  }

  // Replace pool by a malloc'd buffer
  DocBuffer::DocBuffer(size_t uMemPoolInitialSize, size_t) :
      iv_uMemPoolInitialSize(uMemPoolInitialSize),
      iv_uMemPoolReserve(UIMA_DOC_BUFFER_RESERVE_SIZE),
      iv_cpw16Document(0),
      iv_uLength(0),
      iv_uSizeAllocated(0)
      /* ----------------------------------------------------------------------- */
  {
    init();
  }

  DocBuffer::~DocBuffer()
  /* ----------------------------------------------------------------------- */
  {
    if ( iv_cpw16Document != 0 )
      free((void*)iv_cpw16Document);
  }

  void DocBuffer::init()
  /* ----------------------------------------------------------------------- */
  {
    resetMemPool();
  }

  bool DocBuffer::isValid(void) const
  /* ----------------------------------------------------------------------- */
  {
    return(iv_cpw16Document != 0);
  }

  UnicodeStringRef DocBuffer::getText(TyDocIndex uIndexBegin,
                                      TyDocIndex uIndexEnd) const UIMA_THROW(ExcDocBuffer)
  /* ----------------------------------------------------------------------- */
  {
    assert(EXISTS(iv_cpw16Document));
    assert(uIndexBegin <= uIndexEnd);
    /* in case the assert is gone in ship mode */
    if (!isValidIndex(uIndexBegin)) {
      UIMA_EXC_THROW_NEW(ExcDocBuffer,
                         UIMA_ERR_DOCUMENT_INVALID_INDEX,
                         UIMA_MSG_ID_EXC_DOCUMENT_INVALID_IDX,
                         ErrorMessage(UIMA_MSG_ID_EXCON_DOCUMENT_INVALID_IDX, (unsigned long) uIndexBegin),
                         ErrorInfo::recoverable);
    }
    if (!isValidIndex(uIndexEnd)) {
      UIMA_EXC_THROW_NEW(ExcDocBuffer,
                         UIMA_ERR_DOCUMENT_INVALID_INDEX,
                         UIMA_MSG_ID_EXC_DOCUMENT_INVALID_IDX,
                         ErrorMessage(UIMA_MSG_ID_EXCON_DOCUMENT_INVALID_IDX, (unsigned long) uIndexEnd),
                         ErrorInfo::recoverable);
    }
    return(UnicodeStringRef(iv_cpw16Document + uIndexBegin, (uIndexEnd - uIndexBegin + 1)));
  }

  void DocBuffer::addDocPart(const char * cpacDocPartText,
                             size_t uDocPartSizeInBytes,
                             CodePage2UnicodeConverter & crclConverter)
  /* ----------------------------------------------------------------------- */
  {
    /////assert(crclConverter.isSupported());
    addDocPartImp(cpacDocPartText, uDocPartSizeInBytes, crclConverter);
  }


  void DocBuffer::addDocPart(const char * cpacDocPartText,
                             size_t uDocPartSize,
                             const char * crclCCSID) {
    CodePage2UnicodeConverter converter(crclCCSID);
    addDocPartImp(cpacDocPartText, uDocPartSize, converter);
  }


  void DocBuffer::addDocPart(const UChar * cpclDocPartText,
                             size_t uDocPartLength)
  /* ----------------------------------------------------------------------- */
  {
    CodePage2UnicodeConverter clConverter("UTF16_PlatformEndian");
    size_t                     uDocPartSizeInBytes;

    //// assert(clConverter.getTargetCCSID().isUCS2HostEndian());
    ////assert(clConverter.getSourceCCSID().isUCS2HostEndian());
    ////assert(clConverter.isSupported());
    ////assert(clConverter.isBuiltIn());
    uDocPartSizeInBytes = uDocPartLength * sizeof(UChar);
    addDocPartImp((const char *) cpclDocPartText, uDocPartSizeInBytes, clConverter);
  }

  void DocBuffer::reset(void)
  /* ----------------------------------------------------------------------- */
  {
    resetMemPool();
  }


}

/* <EOF> */

