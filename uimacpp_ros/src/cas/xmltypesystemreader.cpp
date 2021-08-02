/** \file xmltypesystemreader.cpp .
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
// #define DEBUG_VERBOSE

#include <uima/pragmas.hpp>

#include "xercesc/util/PlatformUtils.hpp"
#include "xercesc/sax/SAXParseException.hpp"
#include "xercesc/parsers/XercesDOMParser.hpp"
#include "xercesc/dom/DOMException.hpp"
#include "xercesc/dom/DOMNamedNodeMap.hpp"

#include "xercesc/sax/ErrorHandler.hpp"
#include "xercesc/dom/DOMDocument.hpp"
#include "xercesc/dom/DOMElement.hpp"
#include "xercesc/dom/DOMNodeList.hpp"
#include "xercesc/framework/LocalFileInputSource.hpp"
#include "xercesc/framework/MemBufInputSource.hpp"

#include <uima/xmltypesystemreader.hpp>
#include <uima/lowlevel_typesystem.hpp>

#include <uima/internal_xmlconstants.hpp>
#include <uima/internal_casimpl.hpp>
#include <uima/msg.h>
#include <uima/xmlerror_handler.hpp>
#include <uima/macros.h>
#include <uima/casdefinition.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */
#define MAXXMLCHBUFF 256
/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

#define DEBUG_VERBOSE

namespace uima {

  UIMA_EXC_CLASSIMPLEMENT(XMLTypeSystemReaderException, uima::Exception);

  class RethrowErrorHandler : public ErrorHandler {
  public:

    void error(const SAXParseException& e) {
      throw e;
    }

    void fatalError(const SAXParseException& e) {
      throw e;
    }

    void warning(const SAXParseException& e) {
      throw e;
    }

    void resetErrors() {}
  };


  static XMLCh gs_tempXMLChBuffer[ MAXXMLCHBUFF ];

  XMLCh const * convert(char const * cpBuf) {
    bool bTranscodeSuccess = XMLString::transcode( cpBuf, gs_tempXMLChBuffer, MAXXMLCHBUFF -1 );
    assert( bTranscodeSuccess );
    return gs_tempXMLChBuffer;
  }

  icu::UnicodeString convert( XMLCh const * cpUCBuf ) {
    assertWithMsg( sizeof(XMLCh) == sizeof(UChar), "Port required!");
    unsigned int uiLen = XMLString::stringLen( cpUCBuf );
    return icu::UnicodeString( (UChar const *) cpUCBuf, uiLen);
  }


  void XMLTypeSystemReader::checkValidityCondition(bool bCondition) const {
    if (!bCondition) {
      UIMA_EXC_THROW_NEW(XMLTypeSystemReaderException,
                         UIMA_ERR_XMLTYPESYSTEMREADER,
                         UIMA_MSG_ID_EXC_XMLTYPESYSTEMREADER,
                         uima::ErrorMessage(UIMA_MSG_ID_EXCON_READING_TYPESYSTEM_FROM_XML),
                         uima::ErrorInfo::unrecoverable
                        );
    }
  }

  void XMLTypeSystemReader::checkValidityCondition(bool bCondition, TyMessageId tyMessage, icu::UnicodeString const & crString) const {
    if (!bCondition) {
      uima::ErrorMessage msg(tyMessage);
      msg.addParam( crString );
      UIMA_EXC_THROW_NEW(XMLTypeSystemReaderException,
                         UIMA_ERR_XMLTYPESYSTEMREADER,
                         UIMA_MSG_ID_EXC_XMLTYPESYSTEMREADER,
                         msg,
                         uima::ErrorInfo::unrecoverable
                        );
    }
  }


  void XMLTypeSystemReader::checkValidityCondition(bool bCondition, TyMessageId tyMessage, icu::UnicodeString const & crString1, icu::UnicodeString const & crString2) const {
    if (!bCondition) {
      uima::ErrorMessage msg(tyMessage);
      msg.addParam( crString1 );
      msg.addParam( crString2 );
      UIMA_EXC_THROW_NEW(XMLTypeSystemReaderException,
                         UIMA_ERR_XMLTYPESYSTEMREADER,
                         UIMA_MSG_ID_EXC_XMLTYPESYSTEMREADER,
                         msg,
                         uima::ErrorInfo::unrecoverable
                        );
    }
  }


  XMLTypeSystemReader::XMLTypeSystemReader(TypeSystem & rTypeSystem)
      : iv_rTypeSystem(uima::lowlevel::TypeSystem::promoteTypeSystem( rTypeSystem )),
      iv_pXMLErrorHandler(NULL) {}

  XMLTypeSystemReader::XMLTypeSystemReader(uima::internal::CASDefinition & casDef)
      : iv_rTypeSystem( casDef.getTypeSystem() ),
      iv_pXMLErrorHandler(NULL) {}

  XMLTypeSystemReader::~XMLTypeSystemReader() {}

  void XMLTypeSystemReader::createFeatures(DOMElement * pTopTypeElement) {
    DOMNodeList * featureList = pTopTypeElement->getElementsByTagName( convert(uima::internal::XMLConstants::TAGNAME_FEATURE) );
    unsigned int i=0;
    for (i=0; i<featureList->getLength(); ++i) {
      DOMNode * featureNode = featureList->item(i);
      assert( featureNode->getNodeType() == DOMNode::ELEMENT_NODE );
      assert( XMLString::compareString( featureNode->getNodeName(), convert(uima::internal::XMLConstants::TAGNAME_FEATURE)) == 0 );
      DOMElement * featureElement = (DOMElement*) featureNode;

      icu::UnicodeString rangeTypeName = convert( featureElement->getAttribute( convert(uima::internal::XMLConstants::ATTRIBUTENAME_RANGE) ) );
      icu::UnicodeString featureName = convert( featureElement->getAttribute( convert(uima::internal::XMLConstants::ATTRIBUTENAME_NAME)) );
      icu::UnicodeString multiRefs = convert( featureElement->getAttribute( convert(uima::internal::XMLConstants::ATTRIBUTENAME_MULTIREFS)) );
      UIMA_TPRINT("Checking for feature  : " << featureName << " with range type " << rangeTypeName);

      DOMNode * introTypeNode = featureNode->getParentNode();
      assert( introTypeNode->getNodeType() == DOMNode::ELEMENT_NODE );

      checkValidityCondition( XMLString::compareString( introTypeNode->getNodeName(), convert(uima::internal::XMLConstants::TAGNAME_TYPE) ) == 0,
                              UIMA_MSG_ID_EXC_WRONG_XML_TYPESYSTEM_FORMAT,
                              featureName
                            );

      DOMElement * pIntroTypeElement = (DOMElement *) introTypeNode;
      icu::UnicodeString introTypeName = convert( pIntroTypeElement->getAttribute(convert(uima::internal::XMLConstants::ATTRIBUTENAME_NAME)) );
      UIMA_TPRINT("Checking for feature  : " << featureName << " with range type " << rangeTypeName << " at intro type " << introTypeName);

      lowlevel::TyFSType tyIntro = iv_rTypeSystem.getTypeByName(introTypeName);
      checkValidityCondition( iv_rTypeSystem.isValidType(tyIntro),
                              UIMA_MSG_ID_EXC_INVALID_INTRO_TYPE,
                              introTypeName );
      lowlevel::TyFSType tyRange = iv_rTypeSystem.getTypeByName(rangeTypeName);
      checkValidityCondition( iv_rTypeSystem.isValidType(tyRange),
                              UIMA_MSG_ID_EXC_INVALID_RANGE_TYPE,
                              rangeTypeName );
      lowlevel::TyFSFeature tyFeature = iv_rTypeSystem.getFeatureByBaseName(tyIntro, featureName );
      if (tyFeature != lowlevel::TypeSystem::INVALID_FEATURE) {
        // check that intro and range types are correct
        checkValidityCondition( tyIntro == iv_rTypeSystem.getIntroType(tyFeature),
                                UIMA_MSG_ID_EXC_INVALID_INTRO_TYPE,
                                featureName,
                                introTypeName );
        checkValidityCondition( tyRange == iv_rTypeSystem.getRangeType(tyFeature),
                                UIMA_MSG_ID_EXC_INVALID_RANGE_TYPE,
                                featureName,
                                rangeTypeName );
      } else {
        UIMA_TPRINT("Creating feature  : " << featureName << " with range type " << rangeTypeName << " at intro type " << introTypeName);
        // create the feature
		bool mr = (multiRefs == icu::UnicodeString("true"));
        tyFeature = iv_rTypeSystem.createFeature( tyIntro, tyRange, mr, featureName, iv_ustrCreatorID );
      }
    }
  }


  void XMLTypeSystemReader::createType(lowlevel::TyFSType tyParentType, DOMElement * pNewTypeElement) {
    UIMA_TPRINT("entering createType");
    assert( XMLString::compareString( pNewTypeElement->getNodeName(), convert(uima::internal::XMLConstants::TAGNAME_TYPE)) == 0 );
    lowlevel::TyFSType tyNewType = lowlevel::TypeSystem::INVALID_TYPE;
    if (tyParentType != lowlevel::TypeSystem::INVALID_TYPE) {
      assert( iv_rTypeSystem.isValidType( tyParentType ) );
      // create the type
      icu::UnicodeString newTypeName = convert( pNewTypeElement->getAttribute( convert(uima::internal::XMLConstants::ATTRIBUTENAME_NAME )) );

      tyNewType = iv_rTypeSystem.getTypeByName( newTypeName );
      UIMA_TPRINT("Checking for type : " << newTypeName);
      if (tyNewType == lowlevel::TypeSystem::INVALID_TYPE) {
        UIMA_TPRINT("Creating type : " << newTypeName);
        tyNewType = iv_rTypeSystem.createType(tyParentType, newTypeName, iv_ustrCreatorID);
      } else {
        checkValidityCondition( iv_rTypeSystem.getParentType(tyNewType) == tyParentType,
                                UIMA_MSG_ID_EXC_WRONG_PARENT_TYPE,
                                iv_rTypeSystem.getTypeName(tyNewType) );
      }
    } else {
      tyNewType = iv_rTypeSystem.getTopType();
    }
    assert( iv_rTypeSystem.isValidType( tyNewType ) );


    DOMNodeList * childTypes = pNewTypeElement->getChildNodes();
    unsigned int i=0;
    for (i=0; i<childTypes->getLength(); ++i) {
      // filter type children
      DOMNode * kid = childTypes->item(i);
      bool bIsElement = ( kid->getNodeType() == DOMNode::ELEMENT_NODE );
      bool bIsTypeTag = ( XMLString::compareString(kid->getNodeName(), convert(uima::internal::XMLConstants::TAGNAME_TYPE)) == 0 );
      if (bIsTypeTag && bIsElement) {
        DOMElement * pChildrenTypeElement = (DOMElement*) kid;
        createType(tyNewType, pChildrenTypeElement);
      }
    }


    UIMA_TPRINT("exiting createType");
  }


  void XMLTypeSystemReader::readMemory(icu::UnicodeString const & xmlString, icu::UnicodeString const & creatorID) {
    UChar const * xmlChars = xmlString.getBuffer();
    size_t uiBytes = xmlString.length() * 2;
    UnicodeStringRef uref(xmlString);
    readMemory(uref.asUTF8().c_str(), creatorID);
  }


  void XMLTypeSystemReader::readMemory(char const * cpszXMLString, icu::UnicodeString const & creatorID) {
    MemBufInputSource memIS((XMLByte const *) cpszXMLString, strlen(cpszXMLString), "sysID");
    read(memIS, creatorID );
  }


  void XMLTypeSystemReader::readFile(char const * fileName, icu::UnicodeString const & creatorID) {
    // convert to unicode using the default converter for the platform (W/1252 U/utf-8)
    icu::UnicodeString ustrFileName(fileName);
    readFile( ustrFileName, creatorID );
  }

  void XMLTypeSystemReader::readFile(icu::UnicodeString const & fileName, icu::UnicodeString const & creatorID) {
    size_t uiLen = fileName.length();
    UChar* arBuffer = new UChar[uiLen + 1];
    assert( arBuffer != NULL );

    fileName.extract(0, uiLen, arBuffer);
    arBuffer[uiLen] = 0; // terminate the buffer with 0

    LocalFileInputSource fileIS((XMLCh const *) arBuffer );

    read(fileIS, creatorID );

    delete[] arBuffer;
  }


  void XMLTypeSystemReader::setErrorHandler(ErrorHandler * pErrorHandler) {
    iv_pXMLErrorHandler = pErrorHandler;
  }


  void XMLTypeSystemReader::read(InputSource const & crInputSource, icu::UnicodeString const & creatorID) {
    UIMA_TPRINT("read() entered");
    iv_ustrCreatorID = creatorID;
    XercesDOMParser parser;
    parser.setValidationScheme(XercesDOMParser::Val_Auto);
    parser.setDoNamespaces(false);
    parser.setDoSchema(false);

    bool bHasOwnErrorHandler = false;
    if (iv_pXMLErrorHandler == NULL) {
      iv_pXMLErrorHandler = new XMLErrorHandler();
      assert( iv_pXMLErrorHandler != NULL );
      bHasOwnErrorHandler = true;
    }
    parser.setErrorHandler(iv_pXMLErrorHandler);

    parser.parse( crInputSource);
    DOMDocument* doc = parser.getDocument();
    assert(EXISTS(doc));

    // get top node
    DOMElement * rootElem = doc->getDocumentElement();
    assert(EXISTS(rootElem));

    /* taph 02.10.2002: do we need to do the validity checking ourselves?
       Adding an (inline) DTD does that better then we could ever do it.
       And it is expensive because of the conversions. */
    icu::UnicodeString ustrTAGNAME_TYPEHIERARCHY(uima::internal::XMLConstants::TAGNAME_TYPEHIERARCHY);
    icu::UnicodeString ustrTAGNAME_TYPE(uima::internal::XMLConstants::TAGNAME_TYPE);
    assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
    icu::UnicodeString ustrRootName( (UChar const *) rootElem->getNodeName());
    UIMA_TPRINT("root element name: "<< ustrRootName );
    checkValidityCondition( ustrRootName == ustrTAGNAME_TYPEHIERARCHY,
                            UIMA_MSG_ID_EXC_WRONG_XML_TYPESYSTEM_FORMAT,
                            ustrRootName );

    DOMNodeList * children = rootElem->getChildNodes();
    assert(EXISTS(children));

    checkValidityCondition( children->getLength() > 0 );
    unsigned int i=0;
    while (i<children->getLength() ) {
      DOMNode * kid = children->item(i);
      assert(EXISTS(kid));
      // kid should be the element of the top type
      if ( kid->getNodeType() == DOMNode::ELEMENT_NODE ) {
        UIMA_TPRINT("in element node block");

        DOMElement * kidElem = (DOMElement*) kid;
        /* taph 02.10.2002: do we need to do the validity checking ourselves?
           Adding an (inline) DTD does that better then we could ever do it.
           And it is expensive because of the conversions. */
        assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
        icu::UnicodeString ustrKidName((UChar const *) kidElem->getNodeName());
        checkValidityCondition( ustrKidName == ustrTAGNAME_TYPE,
                                UIMA_MSG_ID_EXC_WRONG_XML_TYPESYSTEM_FORMAT,
                                ustrKidName );

        createType(lowlevel::TypeSystem::INVALID_TYPE, kidElem );
        createFeatures(kidElem);
        break;
      }
      ++i;
    }

    if (bHasOwnErrorHandler) {
      assert( EXISTS(iv_pXMLErrorHandler) );
      delete iv_pXMLErrorHandler;
      iv_pXMLErrorHandler = NULL;
    }
    UIMA_TPRINT("Exiting read()");
  }


} // namespace uima

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */




