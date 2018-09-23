/** \file xmlparse_handlers.hpp .

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


   \brief  XML4C SAX handler routines

-------------------------------------------------------------------------- */
#ifndef __UIMA_XMLPARSE_HANDLERS_HPP
#define __UIMA_XMLPARSE_HANDLERS_HPP


// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include "uima/pragmas.hpp" //must be first to surpress warnings
#include <map>
#include <stack>
#include <utility>
#include "xercesc/sax/HandlerBase.hpp"
#include "uima/timedatetools.hpp"
#include "uima/parse_handlers.hpp"

namespace uima {

  /**
     The class <TT>ParseHandlers</TT> is used as a generic SAX-like
     parse hander class.

     @see ParseHandlers
  */
  class XMLParseHandlers : public HandlerBase, public ParseHandlers {
  public:
    // -----------------------------------------------------------------------
    //  Constructors and Destructor
    // -----------------------------------------------------------------------
    XMLParseHandlers();
    ~XMLParseHandlers();

    // -----------------------------------------------------------------------
    //  Handlers for the SAX DocumentHandler interface
    // -----------------------------------------------------------------------
    void endElement(const XMLCh* const name);
    void startElement(const XMLCh* const name, AttributeList& attributes);
    void characters(const XMLCh* const chars, const unsigned int length);
    void ignorableWhitespace(const XMLCh* const chars, const unsigned int length);
    void resetDocument();


    // -----------------------------------------------------------------------
    //  Handlers for the SAX ErrorHandler interface
    // -----------------------------------------------------------------------
    void warning(const SAXParseException& exception);
    void error(const SAXParseException& exception);
    void fatalError(const SAXParseException& exception);

  };

} // namespace uima

#endif //__UIMA_XMLPARSE_HANDLERS_HPP

