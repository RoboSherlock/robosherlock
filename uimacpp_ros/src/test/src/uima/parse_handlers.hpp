/** \file parse_handlers.hpp .

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


   \brief  Generic SAX-like parse hander class definitions

-------------------------------------------------------------------------- */
#ifndef __UIMA_PARSE_HANDLERS_HPP
#define __UIMA_PARSE_HANDLERS_HPP


// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include "uima/pragmas.hpp" //must be first to surpress warnings
#include <map>
#include <stack>
#include <utility>
#include "uima/parser_config.hpp"
#include "uima/doc_buffer.hpp"
#include "uima/tcas.hpp"
#include "uima/parser_interface.hpp"


namespace uima {

  /**
     The class <TT>ParseHandlers</TT> is used as a generic SAX-like
     parse hander class.

     @see XMLParseHandlers
  */
  class ParseHandlers {
  public:
    /**
     * Typedefs for data structure for communication between the beginElement()
     * and endElement() function.
     * We get attribute information in beginElement() and need to pass this
     * information to endElement() because we can only do the mapping
     * once we know the end of an annotation
     * @{*/

    /// a struct to hold information about a single XML attribute
    class StXMLAttrInfo {
    public:
      icu::UnicodeString  ustrName;
      icu::UnicodeString  ustrType;
      icu::UnicodeString  ustrValue;
      // OS STL need this to be a full STL compliant class
      bool operator < (const StXMLAttrInfo & crclRHS) const {
        return(bool)(ustrName < crclRHS.ustrName);
      }
      bool operator ==(const StXMLAttrInfo & crclRHS) const {
        return(bool)(ustrName == crclRHS.ustrName);
      }
    };
    /// a container to hold the list of all XML attributes of a given XML element
    typedef vector< StXMLAttrInfo > TyXMLAttrInfoList;
    /*@}*/

  public:
    // -----------------------------------------------------------------------
    //  Constructors and Destructor
    // -----------------------------------------------------------------------
    ParseHandlers();
    virtual ~ParseHandlers();

    // -----------------------------------------------------------------------
    //  init method
    // -----------------------------------------------------------------------

    bool
    init(
      TCAS                    & rTCAS,
      ParserConfiguration const & rclConfig,
      bool                        bVerbose = false
    );

    bool deInit();

    void setMultiDocCallback(ParserInterface::MultiDocCallbackInterface &);

    TyErrorId beginDoc();
    TyErrorId endDoc();

    // -----------------------------------------------------------------------
    //  Getter methods
    // -----------------------------------------------------------------------
    size_t getNumberOfDocumentsParsed() const;

    size_t getNumberOfBytesParsed() const;

    bool   isMultiDocFile() const;

    // -----------------------------------------------------------------------
    //  Handlers for the DocumentHandler interface
    // -----------------------------------------------------------------------
    void endElement(const UChar* cpuCName, size_t uiLength);
    void startElement(const UChar* cpucName, size_t uiLength, const TyXMLAttrInfoList & crvecAttributes);
    void characters(const UChar* cpucChars, size_t uiLength);


    void processWarning(const char* cpszErrorId, const UChar * cpszErrorContext);

    UnicodeStringRef getDocumentText() const;

  protected:

    AnnotationFS findLastAnnOfType(size_t uiBeginPos, Type type) const;

    // -----------------------------------------------------------------------
    // we need a stack of those containers for each XML element
    // so we define a map from the XML element name to a pair of
    // 1: the begin index of the element with those attrs
    // 2: the attr of the element at that position
    typedef pair< TyDocIndex, TyXMLAttrInfoList > TyIndexAttrsPair;
    typedef stack< TyIndexAttrsPair, deque< TyIndexAttrsPair > > TyStack;
    typedef map< icu::UnicodeString, TyStack, less< icu::UnicodeString > >
    TyPosStack;
    // -----------------------------------------------------------------------
    //  Private data members
    // -----------------------------------------------------------------------
    ParserConfiguration const * iv_pclConfig;

    DocBuffer             iv_docBuffer;
    TCAS *                iv_pTCAS;

    bool                  iv_bVerbose;
    bool                  iv_bIsMultiDocFile;
    size_t                iv_uiMultiDocNbr;
    size_t                iv_uiMultiDocOffset;

    size_t                iv_uiInputSize;
    long                  iv_lLastEndIndex;
    TyPosStack            iv_clPosStack;
    size_t                iv_uiInIgnoreTag;
    ParserInterface::MultiDocCallbackInterface * iv_pCallbackObject;
  };

} // namespace uima

#endif //__UIMA_PARSE_HANDLERS_HPP
