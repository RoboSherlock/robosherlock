/** \file xmlerror_handler.hpp .
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

    \brief Handler for XML error interface mapping XML to UIMACPP exceptions

   Description:

-----------------------------------------------------------------------------


   09/23/2002  Initial creation

-------------------------------------------------------------------------- */
#ifndef _UIMA_XMLERROR_HANDLER_HPP
#define _UIMA_XMLERROR_HANDLER_HPP


// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include <uima/pragmas.hpp> //must be first to surpress warnings
#include <map>
#include <stack>
#include <utility>
#include <uima/types.h>
#include "xercesc/sax/HandlerBase.hpp"
#include <uima/timedatetools.hpp>

XERCES_CPP_NAMESPACE_USE

namespace uima {

  /**
   * The class <TT>XMLErrorHandler</TT> is used as a generic Handler
   * for the XML4C SAX error interface mapping XML to UIMACPP exceptions.
   * It can be used with any XML4C SAX or DOM parser class as argument for the
   * setErrorHandler() function.
   * It will throw an <code>ExcIllFormedInputError</code> for each warning, error
   * or fatalError of the parser.
   * The error id will be <code>UIMA_ERR_RESOURCE_CORRUPTED</code>.
   * The message will contain line, column and description.
   */
  class UIMA_LINK_IMPORTSPEC XMLErrorHandler : public HandlerBase {
  public:
    // -----------------------------------------------------------------------
    //  Constructors and Destructor
    // -----------------------------------------------------------------------
    XMLErrorHandler();
    ~XMLErrorHandler();

    // -----------------------------------------------------------------------
    //  Handlers for the SAX ErrorHandler interface
    // -----------------------------------------------------------------------
    void warning(const SAXParseException& exception);
    void error(const SAXParseException& exception);
    void fatalError(const SAXParseException& exception);

  };

} // namespace uima

#endif //_UIMA_XMLERROR_HANDLER_HPP

