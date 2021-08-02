/** \file xmideserializer.cpp .
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

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp>
#include <memory>

#include <xercesc/sax2/XMLReaderFactory.hpp>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/DefaultHandler.hpp>


#include <uima/xmlerror_handler.hpp>
#include <uima/xmideserializer.hpp>
#include <uima/xmideserializer_handler.hpp>

#include <uima/arrayfs.hpp>
#include <uima/lowlevel_indexrepository.hpp>
#include <uima/lowlevel_indexiterator.hpp>
#include <uima/internal_casimpl.hpp>
#include <uima/internal_fspromoter.hpp>
#include <uima/fsindexrepository.hpp>
#include <uima/annotator_context.hpp>

#include <uima/msg.h>
#include <uima/resmgr.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */
using namespace std;
namespace uima {

  XmiDeserializer::XmiDeserializer() {}

  XmiDeserializer::~XmiDeserializer() {}

  void XmiDeserializer::deserialize(char const * xmiFilename, CAS & cas, bool lenient) {

    XMLCh* native = XMLString::transcode(xmiFilename);
	LocalFileInputSource fileIS (native);
	XMLString::release(&native);
    XmiDeserializer::deserialize(fileIS, cas, lenient);

  }

  void XmiDeserializer::deserialize(icu::UnicodeString & xmiFilename, CAS & cas, bool lenient) {
    char buff[1024];
    xmiFilename.extract(0, xmiFilename.length(), buff);
    XMLCh* native = XMLString::transcode(buff);
    LocalFileInputSource fileIS (native);
	XMLString::release(&native);
    XmiDeserializer::deserialize(fileIS, cas, lenient);
  }

  void XmiDeserializer::deserialize(InputSource const & crInputSource, CAS & cas, bool lenient) {
    // Create a SAX2 parser object.
    SAX2XMLReader* parser = XMLReaderFactory::createXMLReader();

    //register content handler
    XmiDeserializerHandler * contentHandler = new XmiDeserializerHandler(cas, NULL, lenient);
    parser->setContentHandler(contentHandler);

    //register error handler
    XMLErrorHandler* errorHandler = new XMLErrorHandler();
    parser->setErrorHandler(errorHandler);

    // Parse the XML document.
    // Document content sent to registered ContentHandler instance.
    try {
      parser->parse(crInputSource);
    } catch (const XMLException& e) {
      char* message = XMLString::transcode(e.getMessage());
      cerr << "XMLException message is: \n"
      << message << "\n";
	  delete message;

      ErrorInfo errInfo;
      errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
      ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
      assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
      msg.addParam((UChar const *) e.getMessage() );
      msg.addParam( 0 );
      msg.addParam( 0 );
      errInfo.setMessage(msg);
      errInfo.setSeverity(ErrorInfo::unrecoverable);
      ExcIllFormedInputError exc(errInfo);
      delete contentHandler;
      delete errorHandler;
      delete parser;
      throw exc;
    }
    catch (const SAXParseException& e) {
      char* message = XMLString::transcode(e.getMessage());
      cerr << "SaxParseException message is: \n"
      << message << "\n";
	  delete message;

      ErrorInfo errInfo;
      errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
      ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
      assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
      msg.addParam((UChar const *) e.getMessage() );
      msg.addParam( 0 );
      msg.addParam( 0 );
      errInfo.setMessage(msg);
      errInfo.setSeverity(ErrorInfo::unrecoverable);
      ExcIllFormedInputError exc(errInfo);
      delete contentHandler;
      delete errorHandler;
      delete parser; 
      throw exc;
    } catch (Exception e) {
       ErrorInfo errInfo;
      errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
      ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
      assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
      msg.addParam( e.asString() );
      msg.addParam( 0 );
      msg.addParam( 0 );
      errInfo.setMessage(msg);
      errInfo.setSeverity(ErrorInfo::unrecoverable);
      ExcIllFormedInputError exc(errInfo);
      delete contentHandler;
      delete errorHandler;
      delete parser;
      throw exc;

    } catch (...) {
       ErrorInfo errInfo;
      errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
      ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
      assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
      msg.addParam("Unknown Exception when parsing XMI document." );
      msg.addParam( 0 );
      msg.addParam( 0 );
      errInfo.setMessage(msg);
      errInfo.setSeverity(ErrorInfo::unrecoverable);
      ExcIllFormedInputError exc(errInfo);
      if (contentHandler != NULL) {
        delete contentHandler;
      }
      if (errorHandler !=NULL) {
        delete errorHandler;
      }
      if (parser != NULL) {
        delete parser;
      }
      throw exc;
    }

    // Delete the parser instance and handlers
    delete contentHandler;
    delete errorHandler;
    delete parser;
  }


  void XmiDeserializer::deserialize(InputSource const & crInputSource, 
								CAS & cas,
								XmiSerializationSharedData & sharedData) {
    // Create a SAX2 parser object.
    SAX2XMLReader* parser = XMLReaderFactory::createXMLReader();

    //register content handler
    XmiDeserializerHandler * contentHandler = new XmiDeserializerHandler(cas, &sharedData);
    parser->setContentHandler(contentHandler);

    //register error handler
    XMLErrorHandler* errorHandler = new XMLErrorHandler();
    parser->setErrorHandler(errorHandler);

    // Parse the XML document.
    // Document content sent to registered ContentHandler instance.
    try {
      parser->parse(crInputSource);
    } catch (const XMLException& e) {
      char* message = XMLString::transcode(e.getMessage());
      cerr << "XMLException message is: \n"
      << message << "\n";
	  delete message;

      ErrorInfo errInfo;
      errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
      ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
      assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
      msg.addParam((UChar const *) e.getMessage() );
      msg.addParam( 0 );
      msg.addParam( 0 );
      errInfo.setMessage(msg);
      errInfo.setSeverity(ErrorInfo::unrecoverable);
      ExcIllFormedInputError exc(errInfo);
      delete contentHandler;
    delete errorHandler;
    delete parser;
      throw exc;
    }
    catch (const SAXParseException& e) {
      char* message = XMLString::transcode(e.getMessage());
      cerr << "SaxParseException message is: \n"
      << message << "\n";
	  delete message;

      ErrorInfo errInfo;
      errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
      ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
      assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
      msg.addParam((UChar const *) e.getMessage() );
      msg.addParam( 0 );
      msg.addParam( 0 );
      errInfo.setMessage(msg);
      errInfo.setSeverity(ErrorInfo::unrecoverable);
      ExcIllFormedInputError exc(errInfo);
      delete contentHandler;
    delete errorHandler;
    delete parser;
      throw exc;
    }  catch (Exception e) {
       ErrorInfo errInfo;
      errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
      ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
      assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
      msg.addParam( e.asString() );
      msg.addParam( 0 );
      msg.addParam( 0 );
      errInfo.setMessage(msg);
      errInfo.setSeverity(ErrorInfo::unrecoverable);
      ExcIllFormedInputError exc(errInfo);
      delete contentHandler;
      delete errorHandler;
      delete parser;
      throw exc;

    } catch (...) {
       ErrorInfo errInfo;
      errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
      ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
      assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
      msg.addParam("Unknown Exception when parsing XMI document." );
      msg.addParam( 0 );
      msg.addParam( 0 );
      errInfo.setMessage(msg);
      errInfo.setSeverity(ErrorInfo::unrecoverable);
      ExcIllFormedInputError exc(errInfo);
      if (contentHandler != NULL) {
        delete contentHandler;
      }
      if (errorHandler !=NULL) {
        delete errorHandler;
      }
      if (parser != NULL) {
        delete parser;
      }
      throw exc;
    }


    // Delete the parser instance and handlers
    delete contentHandler;
    delete errorHandler;
    delete parser;
  }

 void XmiDeserializer::deserialize(char const * xmiFilename, 
									CAS & cas,
									XmiSerializationSharedData & sharedData) {

    XMLCh* native = XMLString::transcode(xmiFilename);
	LocalFileInputSource fileIS (native);
	XMLString::release(&native);
    XmiDeserializer::deserialize(fileIS, cas, sharedData);

 }

 void XmiDeserializer::deserialize(icu::UnicodeString & xmiFilename, 
									CAS & cas,
									XmiSerializationSharedData & sharedData) {
    char buff[1024];
    xmiFilename.extract(0, xmiFilename.length(), buff);
    XMLCh* native = XMLString::transcode(buff);
    LocalFileInputSource fileIS (native);
	XMLString::release(&native);
    XmiDeserializer::deserialize(fileIS, cas, sharedData);
  }
}





