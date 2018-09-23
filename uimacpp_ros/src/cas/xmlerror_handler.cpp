/** \file xmlerror_handler.cpp .
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

   Description: Handler for XML error interface mapping XML to UIMACPP exceptions

-----------------------------------------------------------------------------


   09/23/2002  Initial creation

-------------------------------------------------------------------------- */

// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------
#include <uima/pragmas.hpp>
#include <iostream>
#include <algorithm>
using namespace std;

#include "xercesc/sax/AttributeList.hpp"
#include "xercesc/sax/SAXParseException.hpp"
#include "xercesc/sax/SAXException.hpp"
#include <uima/xmlerror_handler.hpp>
#include <uima/msg.h>
#include <uima/exceptions.hpp>

namespace uima {

// ---------------------------------------------------------------------------
//  XMLErrorHandler: Constructors and Destructor
// ---------------------------------------------------------------------------
  XMLErrorHandler::XMLErrorHandler() {}


  XMLErrorHandler::~XMLErrorHandler()   {}


// ---------------------------------------------------------------------------
//  XMLErrorHandler: Overrides of the SAX ErrorHandler interface
// ---------------------------------------------------------------------------
  void XMLErrorHandler::error(const SAXParseException& e) {
    ErrorInfo errInfo;
    errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
    ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_ERROR);
    assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
    msg.addParam( (UChar const *) e.getSystemId());
    msg.addParam(e.getLineNumber());
	msg.addParam(e.getColumnNumber());
	msg.addParam(UnicodeString(e.getMessage(),XMLString::stringLen(e.getMessage())));
    errInfo.setMessage(msg);
    errInfo.setSeverity(ErrorInfo::unrecoverable);
    ExcIllFormedInputError exc(errInfo);
    throw exc;
  }

  void XMLErrorHandler::fatalError(const SAXParseException& e) {
    ErrorInfo errInfo;
    errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
    ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
    assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
    msg.addParam( (UChar const *) e.getSystemId());
    msg.addParam(e.getLineNumber());
    msg.addParam(e.getColumnNumber());
	msg.addParam(UnicodeString(e.getMessage(),XMLString::stringLen(e.getMessage())));
    errInfo.setMessage(msg);
    errInfo.setSeverity(ErrorInfo::unrecoverable);
    ExcIllFormedInputError exc(errInfo);
    throw exc;
  }

  void XMLErrorHandler::warning(const SAXParseException& e) {
    ErrorInfo errInfo;
    errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
    ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_WARNING);
    assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
    msg.addParam( (UChar const *) e.getSystemId());
    msg.addParam(e.getLineNumber());
    msg.addParam(e.getColumnNumber());
	msg.addParam(UnicodeString(e.getMessage(),XMLString::stringLen(e.getMessage())));
    errInfo.setMessage(msg);
    errInfo.setSeverity(ErrorInfo::unrecoverable);
    ExcIllFormedInputError exc(errInfo);
    throw exc;
  }

} // namespace uima

