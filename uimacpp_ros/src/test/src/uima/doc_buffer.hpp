/** \file doc_buffer.hpp .
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


   \brief  Contains DocBuffer a document buffer for storing a document

-------------------------------------------------------------------------- */

#ifndef UIMA_DOC_BUFFER_HPP
#define UIMA_DOC_BUFFER_HPP

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include "uima/pragmas.hpp" //must be included first to disable warnings
#include "unicode/uchar.h"
#include "uima/types.h"
#include "uima/exceptions.hpp"
#include "uima/unistrref.hpp"

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
  class CodePage2UnicodeConverter;
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */


namespace uima {

  /**
  * The class <TT>DocBuffer</TT> is used to
  * \code
  \endcode
  * @see
  */
  class DocBuffer {
  public:
    /** @name Constructors */
    /*@{*/
    DocBuffer();
    DocBuffer(size_t uMemPoolInitialSize, size_t uMemPoolGrowSize);
    /*@}*/
    ~DocBuffer(void);
    /** @name Properties */
    /*@{*/
    /** Return TRUE, if the specified index is a valid for this buffer. */
    bool                    isValidIndex(TyDocIndex uIndex) const;
    /** Return TRUE, if the document buffer is empty. */
    bool                    isEmpty(void) const                          {
      return(iv_uLength == 0);
    }
    /** Return TRUE if this could be initialized correctly */
    bool                    isValid(void) const;
    /** Return the number of characters as stored in this document buffer. */
    size_t                  getLength(void) const                        {
      return(iv_uLength);
    }
    /** Return a pointer to text for the specified index. <TT>ruLength</TT> is set
        to the length of the text area that can be accessed following the pointer
        given as the return value. */
    const UChar *           getDocBuffer(void) const                     {
      return(iv_cpw16Document);
    }
    /** Return a reference to the text for the specified text index. */
    UnicodeStringRef        getText(TyDocIndex uIndexBegin,
                                    TyDocIndex uIndexEnd) const UIMA_THROW(ExcDocBuffer);
    /*@}*/
    /** @name Miscellaneous */
    /*@{*/
    /** Add a complete document as a buffer starting at address <TT>cpacDocText</TT>,
        with size <TT>uDocLengthInBytes</TT> in characters, and with CCSID EnCCSID_UCS2.
        An optional handle to user data may be provided as <TT>hUserDocInfo</TT>. */
//      void                    addDocInMemory(const UChar * cpclDocText,
//                                             size_t uDocLength);
    void                    addDocPart(const char * cpacDocPartText,
                                       size_t uDocPartSize,
                                       const char * crclCCSID);

    /** Add part of a document as a buffer starting at address <TT>cpacDocPartText</TT>,
        with size <TT>uDocPartSize</TT> in bytes, and using the specified converter
        <TT>crclConverter</TT> for codepage conversion.
        An optional handle to user data may be provided as <TT>hUserDocPartInfo</TT>. */
    void                    addDocPart(const char * cpacDocPartText,
                                       size_t uDocPartSize,
                                       CodePage2UnicodeConverter & crclConverter);
    /** Add part of a document as a buffer starting at address <TT>cpacDocPartText</TT>,
        with size <TT>uDocPartLength</TT> in characters, and with CCSID EnCCSID_UCS2.
        An optional handle to user data may be provided as <TT>hUserDocPartInfo</TT>. */
    void                    addDocPart(const UChar * cpclDocPartText,
                                       size_t uDocPartLength);
    /** This method clears the document buffer, removes all document data
        and resets all internal settings. */
    void                    reset(void);
    /*@}*/
  protected:
    /* --- functions --- */
  private:
    /* --- variables --- */
    size_t                  iv_uMemPoolInitialSize;
    size_t                  iv_uMemPoolReserve;
    const UChar *           iv_cpw16Document;
    size_t                  iv_uLength;
    size_t                  iv_uSizeAllocated;
    /* --- functions --- */
    void init();
    void                    addDocPartImp(const char * cpacDocPartText, size_t uDocPartSize, CodePage2UnicodeConverter & crclConverter);
    void                    resetMemPool(void);
    /* COPY CONSTRUCTOR NOT SUPPORTED */
    DocBuffer(const DocBuffer & ); //lint !e1704
    /* ASSIGNMENT OPERATOR NOT SUPPORTED */
    DocBuffer & operator=(const DocBuffer & crclObject);
  }
  ; /* DocBuffer */

}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

namespace uima {

  inline bool DocBuffer::isValidIndex(TyDocIndex uIndex) const
  /* ----------------------------------------------------------------------- */
  {
    return((uIndex >= 0) && (iv_uLength > 0) && (uIndex <= iv_uLength - 1));
  }
}
#endif /* UIMA_DOC_BUFFER_HPP */

/* <EOF> */

