/** \file taespecifierbuilder.cpp
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
#include <uima/engine.hpp>
#include <uima/typesystem.hpp>
//#define DEBUG_VERBOSE

#include "xercesc/util/PlatformUtils.hpp"
#include "xercesc/parsers/XercesDOMParser.hpp"
#include "xercesc/parsers/SAXParser.hpp"
#include "xercesc/dom/DOMException.hpp"
#include "xercesc/dom/DOMNamedNodeMap.hpp"
#include "xercesc/dom/DOMDocument.hpp"
#include "xercesc/dom/DOMElement.hpp"
#include "xercesc/dom/DOMNodeList.hpp"

#include "xercesc/sax/ErrorHandler.hpp"
#include "xercesc/sax/AttributeList.hpp"
#include "xercesc/sax/SAXParseException.hpp"
#include "xercesc/framework/LocalFileInputSource.hpp"
#include "xercesc/framework/MemBufInputSource.hpp"
#include "xercesc/util/XMLString.hpp"

#include <uima/taespecifierbuilder.hpp>
#include <uima/xmlerror_handler.hpp>
#include <uima/strtools.hpp>
#include <uima/stltools.hpp>
#include <uima/err_ids.h>
#include <uima/resmgr.hpp>

#include <uima/location.hpp>
#include <uima/envvar.hpp>

#include <uima/msg.h>
#define MAXXMLCHBUFF 512
using namespace std;
namespace uima {

  UIMA_EXC_CLASSIMPLEMENT(InvalidXMLException, uima::Exception);

  static XMLCh gs_tempXMLChBuffer[ MAXXMLCHBUFF ];

  XMLParser::XMLParser()
      :iv_pXMLErrorHandler(NULL) {}

  XMLParser::~XMLParser() {}

  
  void XMLParser::setErrorHandler(ErrorHandler * pErrorHandler) {
    iv_pXMLErrorHandler = pErrorHandler;
  }

   void XMLParser::parseAnalysisEngineDescription(AnalysisEngineDescription & aeSpec,
      char const * fileName) {
    // convert to unicode using default converter for platform
    icu::UnicodeString ustrFileName(fileName);
    parseAnalysisEngineDescription(aeSpec, ustrFileName );
  }


  void XMLParser::parseAnalysisEngineDescription(AnalysisEngineDescription & taeSpec,
      icu::UnicodeString const & fileName) {

    icu::UnicodeString const & fn = ResourceManager::resolveFilename(fileName, ".");
    size_t uiLen = fn.length();
    auto_array<UChar> arBuffer( new UChar[uiLen + 1] );
    assert( EXISTS(arBuffer.get()));

    fn.extract(0, uiLen, arBuffer.get());
    (arBuffer.get())[uiLen] = 0; // terminate the buffer with 0
    assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");

    // the following try catch block just to trigger an exception.
    // The constructor of LocalFileInputSource throws an exception on the UNIXes if the file does not
    // exist. On Windows, the parser throws this exception.
    try {
      LocalFileInputSource fileISOnlyForException((XMLCh const *) arBuffer.get() );
    } catch (XMLException const & e) {
      ErrorInfo errInfo;
      errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
      ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
      assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
      msg.addParam( arBuffer.get() );
      msg.addParam( 0 );
      msg.addParam( 0 );
      msg.addParam( (UChar const *) e.getMessage());
      errInfo.setMessage(msg);
      errInfo.setSeverity(ErrorInfo::unrecoverable);
      ExcIllFormedInputError exc(errInfo);
      throw exc;
    }

    LocalFileInputSource fileIS((XMLCh const *) arBuffer.get() ); 
    parseAnalysisEngineDescription(taeSpec, fileIS);
  }


  void XMLParser::parseAnalysisEngineDescription(AnalysisEngineDescription & taeSpec, InputSource const & crInputSource) {

    XercesDOMParser parser;
    parser.setValidationScheme(XercesDOMParser::Val_Auto);
    parser.setDoNamespaces(true);
    if ( ResourceManager::getInstance().doSchemaValidation() ) {
      parser.setDoSchema( ResourceManager::getInstance().isSchemaAvailable());
    } else {
      parser.setDoSchema(false);
    }
    parser.setExternalSchemaLocation( ResourceManager::getInstance().getSchemaInfo());
    bool bHasOwnErrorHandler = false;
    
    if (iv_pXMLErrorHandler == NULL) {
      iv_pXMLErrorHandler = new XMLErrorHandler();
      assert( EXISTS(iv_pXMLErrorHandler) );
      bHasOwnErrorHandler = true;
    }

    parser.setErrorHandler(iv_pXMLErrorHandler);
    
    try {
      parser.parse( crInputSource );
    } catch (Exception e){
      if (bHasOwnErrorHandler) {
        delete iv_pXMLErrorHandler;
        iv_pXMLErrorHandler = NULL; 
      }
      throw(e);      
    }
    
    DOMDocument* doc = parser.getDocument();
    assert(EXISTS(doc));

    // get top node
    DOMElement * rootElem = doc->getDocumentElement();
    assert(EXISTS(rootElem));

    buildAnalysisEngineDescription(taeSpec, rootElem, convert(crInputSource.getSystemId()));

    if (bHasOwnErrorHandler) {
      delete iv_pXMLErrorHandler;
      iv_pXMLErrorHandler = NULL;
    }
  }


  void XMLParser::parseTypeSystemDescription(TypeSystemDescription & tsDesc, InputSource const & crInputSource) {

    XercesDOMParser parser;
    parser.setValidationScheme(XercesDOMParser::Val_Auto);
    parser.setDoNamespaces(true);
    parser.setDoSchema( ResourceManager::getInstance().isSchemaAvailable());
    parser.setExternalSchemaLocation( ResourceManager::getInstance().getSchemaInfo());
    bool bHasOwnErrorHandler = false;
    if (iv_pXMLErrorHandler == NULL) {
      iv_pXMLErrorHandler = new XMLErrorHandler();
      assert( EXISTS(iv_pXMLErrorHandler) );
      bHasOwnErrorHandler = true;
    }

    parser.setErrorHandler(iv_pXMLErrorHandler);
    try {
      parser.parse( crInputSource );
    } catch (Exception const & e){
      if (bHasOwnErrorHandler) {
        delete iv_pXMLErrorHandler;
        iv_pXMLErrorHandler = NULL; 
      }
      throw e;
    }


    DOMDocument* doc = parser.getDocument();
    assert(EXISTS(doc));

    // get top node
    DOMElement * rootElem = doc->getDocumentElement();
    assert(EXISTS(rootElem));

    assert( XMLString::compareString(rootElem->getNodeName(), convert(TAG_TYPE_SYSTEM_DESC)) == 0);
    //TypeSystemDescription * tsSpec = buildTypeSystemDesc(rootElem);
    buildTypeSystemDesc(tsDesc, rootElem);
    if (bHasOwnErrorHandler) {
      delete iv_pXMLErrorHandler;
      iv_pXMLErrorHandler = NULL;
    }

  }


  void XMLParser::parseFSIndexDescription(AnalysisEngineMetaData::TyVecpFSIndexDescriptions & fsDesc,
       InputSource const & crInputSource) {
    
    XercesDOMParser parser;
    parser.setValidationScheme(XercesDOMParser::Val_Auto);
    parser.setDoNamespaces(true);
    parser.setDoSchema(false);
    parser.setExternalSchemaLocation(UIMA_XML_NAMESPACE " ");

    bool bHasOwnErrorHandler = false;
    if (iv_pXMLErrorHandler == NULL) {
      iv_pXMLErrorHandler = new XMLErrorHandler();
      assert( EXISTS(iv_pXMLErrorHandler) );
      bHasOwnErrorHandler = true;
    }
    parser.setErrorHandler(iv_pXMLErrorHandler);
    try {
    parser.parse( crInputSource );
    } catch (Exception const & e){
      if (bHasOwnErrorHandler) {
        delete iv_pXMLErrorHandler;
        iv_pXMLErrorHandler = NULL; 
      }
      throw e;
    }

    DOMDocument* doc = parser.getDocument();
    // get top node
    DOMElement * descElem = doc->getDocumentElement();

    assert(EXISTS(descElem));

    if (bHasOwnErrorHandler) {
      delete iv_pXMLErrorHandler;
      iv_pXMLErrorHandler = NULL;
    }
    buildFSIndexes(fsDesc, descElem);
  }

  void XMLParser::parseTypePriorities(AnalysisEngineMetaData::TyVecpTypePriorities  & prioDesc,
      InputSource const & crInputSource) {
	
    XercesDOMParser parser;
    parser.setValidationScheme(XercesDOMParser::Val_Auto);
    parser.setDoNamespaces(true);
    parser.setDoSchema(false);
    parser.setExternalSchemaLocation(UIMA_XML_NAMESPACE " ");

    bool bHasOwnErrorHandler = false;
    if (iv_pXMLErrorHandler == NULL) {
      iv_pXMLErrorHandler = new XMLErrorHandler();
      assert( EXISTS(iv_pXMLErrorHandler) );
      bHasOwnErrorHandler = true;
    }

    parser.setErrorHandler(iv_pXMLErrorHandler);
    try {
      parser.parse( crInputSource );
    } catch (Exception const & e){
      if (bHasOwnErrorHandler) {
        delete iv_pXMLErrorHandler;
        iv_pXMLErrorHandler = NULL; 
      }
      throw e;
    }

    DOMDocument* doc = parser.getDocument();
    // get top node
    DOMElement * descElem = doc->getDocumentElement();

    assert(EXISTS(descElem));

    if (bHasOwnErrorHandler) {
      delete iv_pXMLErrorHandler;
      iv_pXMLErrorHandler = NULL;
    }

    buildTypePriorities(prioDesc, descElem);
  }


  void XMLParser::parseSofaMappings(AnalysisEngineDescription::TyVecpSofaMappings & sofaMapDesc,
      InputSource const & crInputSource) {

    XercesDOMParser parser;
    parser.setValidationScheme(XercesDOMParser::Val_Auto);
    parser.setDoNamespaces(true);
    parser.setDoSchema(false);
    parser.setExternalSchemaLocation(UIMA_XML_NAMESPACE " ");

    bool bHasOwnErrorHandler = false;
    if (iv_pXMLErrorHandler == NULL) {
      iv_pXMLErrorHandler = new XMLErrorHandler();
      assert( EXISTS(iv_pXMLErrorHandler) );
      bHasOwnErrorHandler = true;
    }

    parser.setErrorHandler(iv_pXMLErrorHandler);
    try {
    parser.parse( crInputSource );
    } catch (Exception const & e){
      if (bHasOwnErrorHandler) {
        delete iv_pXMLErrorHandler;
        iv_pXMLErrorHandler = NULL; 
      }
      throw e;
    }

    DOMDocument* doc = parser.getDocument();
    // get top node
    DOMElement * descElem = doc->getDocumentElement();
    assert(EXISTS(descElem));

    if (bHasOwnErrorHandler) {
      delete iv_pXMLErrorHandler;
      iv_pXMLErrorHandler = NULL;
    }

    buildSofaMappings(sofaMapDesc, descElem);
  }


  void XMLParser::buildAnalysisEngineDescription(AnalysisEngineDescription & taeSpec,
      DOMElement * descElem,
      const icu::UnicodeString & xmlFileLoc) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_TAE_DESC)) == 0 ||
            XMLString::compareString(descElem->getNodeName(), convert(TAG_AE_DESC)) == 0 ||
            XMLString::compareString(descElem->getNodeName(), convert(TAG_CASCONSUMER_DESC)) == 0 );
    //save the root node name
    if (XMLString::compareString(descElem->getNodeName(), convert(TAG_CASCONSUMER_DESC)) == 0) {
      taeSpec.setPrimitive(true);
    }
    taeSpec.setXmlRootTag(convert(descElem->getNodeName()));
    DOMNodeList * children = descElem->getChildNodes();
    assert( EXISTS(children) );

    try {
      taeSpec.setXmlFileLocation(xmlFileLoc);

      size_t i;
      for (i=0; i < children->getLength(); i++) {
        if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
          continue;
        }
        const icu::UnicodeString & childTag = convert((children->item(i))->getNodeName());
        if (childTag.compare(TAG_TAE_PRIMITIVE) == 0) {
          taeSpec.setPrimitive(isTrue(getSpannedText(children->item(i))));
        } else if (childTag.compare(TAG_AN_IMPL_NAME) == 0 ||
                   childTag.compare(TAG_IMPL_NAME) == 0) {
          taeSpec.setAnnotatorImpName(getSpannedText(children->item(i)));
        } else if (childTag.compare(TAG_DELEGATE_AES) == 0) {
          DOMNodeList * delegateAEs = ((children->item(i))->getChildNodes());
          if (EXISTS(delegateAEs)) {
            size_t j;
            for (j=0; j < delegateAEs->getLength(); j++) {
              DOMNode * node = delegateAEs->item(j);
              if (node->getNodeType() != DOMNode::ELEMENT_NODE) {
                continue;
              }
              assert(XMLString::compareString(node->getNodeName(), convert(TAG_DELEGATE_AE)) == 0);
              DOMElement * delegateAE =  (DOMElement *)node;
              assert(EXISTS(delegateAE));
              icu::UnicodeString key = convert(delegateAE->getAttribute(convert(ATTR_DELEGATE_AE_KEY)));
              AnalysisEngineDescription * pdelegateTaeSpec = new AnalysisEngineDescription();
              DOMNodeList * childNodes = delegateAE->getChildNodes();

              DOMElement * delegateSpec = (DOMElement *) findFirst(childNodes, TAG_AE_DESC);
              if (delegateSpec == NULL)
                delegateSpec = (DOMElement *) findFirst(childNodes, TAG_TAE_DESC);
              if (delegateSpec == NULL)
                delegateSpec = (DOMElement *) findFirst(childNodes, TAG_CASCONSUMER_DESC);

              if (EXISTS(delegateSpec)) {           //the delegate is explicitly copied into the aggregate xml file
                buildAnalysisEngineDescription(*pdelegateTaeSpec, delegateSpec, xmlFileLoc);
                taeSpec.addDelegate(key, pdelegateTaeSpec);
              } else {
                //import element
                delegateSpec = (DOMElement *) findFirst (childNodes, TAG_IMPORT_DESC);
                if (delegateSpec != NULL) {
                  icu::UnicodeString loc = convert(delegateSpec->getAttribute(convert(ATTR_IMPORT_DESC_LOCATION)));
				  icu::UnicodeString fn = ResourceManager::resolveFilename(loc, xmlFileLoc);
                  if (loc.length() > 0) {
					parseAnalysisEngineDescription(*pdelegateTaeSpec, fn);
                    taeSpec.addDelegate(key, pdelegateTaeSpec);
                  } else {
                    //throw exception when import location attribute is not set.
                    ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
                    errMsg.addParam(childTag);
                    errMsg.addParam(xmlFileLoc);

                    UIMA_EXC_THROW_NEW(InvalidXMLException,
                                       UIMA_ERR_IMPORT_INVALID_XML_ATTRIBUTE,
                                       errMsg,
                                       UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                                       ErrorInfo::unrecoverable);
                  }
                } else {
                  //check for xi:include element, try to get filename
                  DOMElement * delegateInc = (DOMElement *) findFirst(childNodes,
                                             TAG_DELEGATE_AE_INCLUDE);
                  if (EXISTS(delegateInc)) {
                    icu::UnicodeString newFileName = ResourceManager::resolveFilename(getSpannedText(delegateInc),
                                                     xmlFileLoc);
                    parseAnalysisEngineDescription(*pdelegateTaeSpec, newFileName);
                    taeSpec.addDelegate(key, pdelegateTaeSpec);

                  }
                }  //include
              }
            }
          }
        } else if (childTag.compare(TAG_AE_METADATA) == 0  ||
                   childTag.compare(TAG_PROCESSING_RESOURCE_METADATA) == 0 ) {
          taeSpec.setAnalysisEngineMetaData(buildAEMetaData((DOMElement *) (children->item(i)), xmlFileLoc));
          taeSpec.getAnalysisEngineMetaData()->getTypeSystemDescription()->setXmlFileLocation(xmlFileLoc);
          std::vector<icu::UnicodeString> alreadyImportedTypeSystemLocations;
          taeSpec.getAnalysisEngineMetaData()->getTypeSystemDescription()->resolveImports(alreadyImportedTypeSystemLocations);
        } else if (childTag.compare(UIMA_FRAMEWORK_IMP) == 0 ) {
          const icu::UnicodeString & impName = getSpannedText(children->item(i));
          if (impName.compare(FRAMEWORK_IMP_CPLUSPLUS) == 0)
            taeSpec.setFrameworkImplName(AnalysisEngineDescription::CPLUSPLUS);
          else if (impName.compare(FRAMEWORK_IMP_JAVA) == 0)
            taeSpec.setFrameworkImplName(AnalysisEngineDescription::JAVA);

          else {
            /**
            ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
            errMsg.addParam(impName);
            errMsg.addParam(schemaFileName);

            UIMA_EXC_THROW_NEW(InvalidXMLException,
                              UIMA_ERR_CONFIG_INVALID_XML_TAG,
                              errMsg,
                              UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                              ErrorInfo::unrecoverable);
            **/
          }
        } else if (childTag.compare(TAG_EXTERNAL_RESOURCE_DEPENDENCIES) == 0) {
          /* taph 15.04.2004: we don't directly support ressources yet but we accept them in the XML to pass them to Jedii*/
        } else if (childTag.compare(TAG_EXTERNAL_RESOURCES) == 0) {
          /* taph 15.04.2004: we don't directly support ressources yet but we accept them in the XML to pass them to Jedii*/
        } else if (childTag.compare(TAG_SOFA_MAPPINGS) == 0) {
          buildSofaMappings(taeSpec, (DOMElement *) (children->item(i)) );
        } else if (childTag.compare(TAG_RESMGR_CONFIG_DESC) == 0) {
          //ignored
        }
        else {
          /**
          ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
          errMsg.addParam(childTag);
          errMsg.addParam(schemaFileName);

          UIMA_EXC_THROW_NEW(InvalidXMLException,
                            UIMA_ERR_CONFIG_INVALID_XML_TAG,
                            errMsg,
                            UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                            ErrorInfo::unrecoverable);
          **/
        }
      }
    } catch (InvalidXMLException & rclException) {
      rclException.getErrorInfo().addContext(ErrorMessage(UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC_FROM_FILE, xmlFileLoc));
      throw rclException;
    } catch (DuplicateConfigElemException & rclException) {
      rclException.getErrorInfo().addContext(ErrorMessage(UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC_FROM_FILE, xmlFileLoc));
      throw rclException;
    }

  }

  void XMLParser::buildAnalysisEngineDescription(AnalysisEngineDescription & taeSpec,
      DOMElement * descElem,
      const icu::UnicodeString & xmlFileLoc,
      bool changeOrder) {

      if (changeOrder == false) {
          buildAnalysisEngineDescription(taeSpec, descElem, xmlFileLoc);
          return;
      }

    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_TAE_DESC)) == 0 ||
            XMLString::compareString(descElem->getNodeName(), convert(TAG_AE_DESC)) == 0 ||
            XMLString::compareString(descElem->getNodeName(), convert(TAG_CASCONSUMER_DESC)) == 0 );
    //save the root node name
    if (XMLString::compareString(descElem->getNodeName(), convert(TAG_CASCONSUMER_DESC)) == 0) {
      taeSpec.setPrimitive(true);
    }
    taeSpec.setXmlRootTag(convert(descElem->getNodeName()));
    DOMNodeList * children = descElem->getChildNodes();
    assert( EXISTS(children) );

    try {
      taeSpec.setXmlFileLocation(xmlFileLoc);

      size_t i;
      size_t delegateIdx;
      size_t primitiveIdx;

      // parse delegate first!!!
      for (i=0; i < children->getLength(); i++) {
          if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
              continue;
          }
          const icu::UnicodeString & childTag = convert((children->item(i))->getNodeName());
          if (childTag.compare(TAG_TAE_PRIMITIVE) == 0) {
              taeSpec.setPrimitive(isTrue(getSpannedText(children->item(i))));
              primitiveIdx = i;
          }
          if (childTag.compare(TAG_DELEGATE_AES) == 0) {
            delegateIdx = i;
            DOMNodeList * delegateAEs = ((children->item(i))->getChildNodes());
            if (EXISTS(delegateAEs)) {
              size_t j;
              for (j=0; j < delegateAEs->getLength(); j++) {
                DOMNode * node = delegateAEs->item(j);
                if (node->getNodeType() != DOMNode::ELEMENT_NODE) {
                  continue;
                }
                assert(XMLString::compareString(node->getNodeName(), convert(TAG_DELEGATE_AE)) == 0);
                DOMElement * delegateAE =  (DOMElement *)node;
                assert(EXISTS(delegateAE));
                icu::UnicodeString key = convert(delegateAE->getAttribute(convert(ATTR_DELEGATE_AE_KEY)));
                AnalysisEngineDescription * pdelegateTaeSpec = new AnalysisEngineDescription();
                DOMNodeList * childNodes = delegateAE->getChildNodes();

                DOMElement * delegateSpec = (DOMElement *) findFirst(childNodes, TAG_AE_DESC);
                if (delegateSpec == NULL)
                  delegateSpec = (DOMElement *) findFirst(childNodes, TAG_TAE_DESC);
                if (delegateSpec == NULL)
                  delegateSpec = (DOMElement *) findFirst(childNodes, TAG_CASCONSUMER_DESC);

                if (EXISTS(delegateSpec)) {           //the delegate is explicitly copied into the aggregate xml file
                  buildAnalysisEngineDescription(*pdelegateTaeSpec, delegateSpec, xmlFileLoc);
                  taeSpec.addDelegate(key, pdelegateTaeSpec);
                } else {
                  //import element
                  delegateSpec = (DOMElement *) findFirst (childNodes, TAG_IMPORT_DESC);
                  if (delegateSpec != NULL) {
                    icu::UnicodeString loc = convert(delegateSpec->getAttribute(convert(ATTR_IMPORT_DESC_LOCATION)));
                    icu::UnicodeString fn = ResourceManager::resolveFilename(loc, xmlFileLoc);
                    if (loc.length() > 0) {
                      parseAnalysisEngineDescription(*pdelegateTaeSpec, fn);
                      taeSpec.addDelegate(key, pdelegateTaeSpec);
                    } else {
                      //throw exception when import location attribute is not set.
                      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
                      errMsg.addParam(childTag);
                      errMsg.addParam(xmlFileLoc);

                      UIMA_EXC_THROW_NEW(InvalidXMLException,
                                         UIMA_ERR_IMPORT_INVALID_XML_ATTRIBUTE,
                                         errMsg,
                                         UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                                         ErrorInfo::unrecoverable);
                    }
                  } else {
                    //check for xi:include element, try to get filename
                    DOMElement * delegateInc = (DOMElement *) findFirst(childNodes,
                                               TAG_DELEGATE_AE_INCLUDE);
                    if (EXISTS(delegateInc)) {
                      icu::UnicodeString newFileName = ResourceManager::resolveFilename(getSpannedText(delegateInc),
                                                       xmlFileLoc);
                      parseAnalysisEngineDescription(*pdelegateTaeSpec, newFileName);
                      taeSpec.addDelegate(key, pdelegateTaeSpec);

                    }
                  }  //include
                }
              }
            }
            break;
          }
      }

      for (i=0; i < children->getLength(); i++) {

        if (i == delegateIdx || i == primitiveIdx) continue;
        if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
          continue;
        }
        const icu::UnicodeString & childTag = convert((children->item(i))->getNodeName());
        if (childTag.compare(TAG_TAE_PRIMITIVE) == 0) {
          taeSpec.setPrimitive(isTrue(getSpannedText(children->item(i))));
        } else if (childTag.compare(TAG_AN_IMPL_NAME) == 0 ||
                   childTag.compare(TAG_IMPL_NAME) == 0) {
          taeSpec.setAnnotatorImpName(getSpannedText(children->item(i)));
        } else if (childTag.compare(TAG_DELEGATE_AES) == 0) {
          DOMNodeList * delegateAEs = ((children->item(i))->getChildNodes());
          if (EXISTS(delegateAEs)) {
            size_t j;
            for (j=0; j < delegateAEs->getLength(); j++) {
              DOMNode * node = delegateAEs->item(j);
              if (node->getNodeType() != DOMNode::ELEMENT_NODE) {
                continue;
              }
              assert(XMLString::compareString(node->getNodeName(), convert(TAG_DELEGATE_AE)) == 0);
              DOMElement * delegateAE =  (DOMElement *)node;
              assert(EXISTS(delegateAE));
              icu::UnicodeString key = convert(delegateAE->getAttribute(convert(ATTR_DELEGATE_AE_KEY)));
              AnalysisEngineDescription * pdelegateTaeSpec = new AnalysisEngineDescription();
              DOMNodeList * childNodes = delegateAE->getChildNodes();

              DOMElement * delegateSpec = (DOMElement *) findFirst(childNodes, TAG_AE_DESC);
              if (delegateSpec == NULL)
                delegateSpec = (DOMElement *) findFirst(childNodes, TAG_TAE_DESC);
              if (delegateSpec == NULL)
                delegateSpec = (DOMElement *) findFirst(childNodes, TAG_CASCONSUMER_DESC);

              if (EXISTS(delegateSpec)) {           //the delegate is explicitly copied into the aggregate xml file
                buildAnalysisEngineDescription(*pdelegateTaeSpec, delegateSpec, xmlFileLoc);
                taeSpec.addDelegate(key, pdelegateTaeSpec);
              } else {
                //import element
                delegateSpec = (DOMElement *) findFirst (childNodes, TAG_IMPORT_DESC);
                if (delegateSpec != NULL) {
                  icu::UnicodeString loc = convert(delegateSpec->getAttribute(convert(ATTR_IMPORT_DESC_LOCATION)));
				  icu::UnicodeString fn = ResourceManager::resolveFilename(loc, xmlFileLoc);
                  if (loc.length() > 0) {
					parseAnalysisEngineDescription(*pdelegateTaeSpec, fn);
                    taeSpec.addDelegate(key, pdelegateTaeSpec);
                  } else {
                    //throw exception when import location attribute is not set.
                    ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
                    errMsg.addParam(childTag);
                    errMsg.addParam(xmlFileLoc);

                    UIMA_EXC_THROW_NEW(InvalidXMLException,
                                       UIMA_ERR_IMPORT_INVALID_XML_ATTRIBUTE,
                                       errMsg,
                                       UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                                       ErrorInfo::unrecoverable);
                  }
                } else {
                  //check for xi:include element, try to get filename
                  DOMElement * delegateInc = (DOMElement *) findFirst(childNodes,
                                             TAG_DELEGATE_AE_INCLUDE);
                  if (EXISTS(delegateInc)) {
                    icu::UnicodeString newFileName = ResourceManager::resolveFilename(getSpannedText(delegateInc),
                                                     xmlFileLoc);
                    parseAnalysisEngineDescription(*pdelegateTaeSpec, newFileName);
                    taeSpec.addDelegate(key, pdelegateTaeSpec);

                  }
                }  //include
              }
            }
          }
        } else if (childTag.compare(TAG_AE_METADATA) == 0  ||
                   childTag.compare(TAG_PROCESSING_RESOURCE_METADATA) == 0 ) {
          taeSpec.setAnalysisEngineMetaData(buildAEMetaData((DOMElement *) (children->item(i)), xmlFileLoc));
          taeSpec.getAnalysisEngineMetaData()->getTypeSystemDescription()->setXmlFileLocation(xmlFileLoc);
          std::vector<icu::UnicodeString> alreadyImportedTypeSystemLocations;
          taeSpec.getAnalysisEngineMetaData()->getTypeSystemDescription()->resolveImports(alreadyImportedTypeSystemLocations);
        } else if (childTag.compare(UIMA_FRAMEWORK_IMP) == 0 ) {
          const icu::UnicodeString & impName = getSpannedText(children->item(i));
          if (impName.compare(FRAMEWORK_IMP_CPLUSPLUS) == 0)
            taeSpec.setFrameworkImplName(AnalysisEngineDescription::CPLUSPLUS);
          else if (impName.compare(FRAMEWORK_IMP_JAVA) == 0)
            taeSpec.setFrameworkImplName(AnalysisEngineDescription::JAVA);

          else {
            /**
            ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
            errMsg.addParam(impName);
            errMsg.addParam(schemaFileName);

            UIMA_EXC_THROW_NEW(InvalidXMLException,
                              UIMA_ERR_CONFIG_INVALID_XML_TAG,
                              errMsg,
                              UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                              ErrorInfo::unrecoverable);
            **/
          }
        } else if (childTag.compare(TAG_EXTERNAL_RESOURCE_DEPENDENCIES) == 0) {
          /* taph 15.04.2004: we don't directly support ressources yet but we accept them in the XML to pass them to Jedii*/
        } else if (childTag.compare(TAG_EXTERNAL_RESOURCES) == 0) {
          /* taph 15.04.2004: we don't directly support ressources yet but we accept them in the XML to pass them to Jedii*/
        } else if (childTag.compare(TAG_SOFA_MAPPINGS) == 0) {
          buildSofaMappings(taeSpec, (DOMElement *) (children->item(i)) );
        } else if (childTag.compare(TAG_RESMGR_CONFIG_DESC) == 0) {
          //ignored
        }
        else {
          /**
          ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
          errMsg.addParam(childTag);
          errMsg.addParam(schemaFileName);

          UIMA_EXC_THROW_NEW(InvalidXMLException,
                            UIMA_ERR_CONFIG_INVALID_XML_TAG,
                            errMsg,
                            UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                            ErrorInfo::unrecoverable);
          **/
        }
      }
    } catch (InvalidXMLException & rclException) {
      rclException.getErrorInfo().addContext(ErrorMessage(UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC_FROM_FILE, xmlFileLoc));
      throw rclException;
    } catch (DuplicateConfigElemException & rclException) {
      rclException.getErrorInfo().addContext(ErrorMessage(UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC_FROM_FILE, xmlFileLoc));
      throw rclException;
    }

  }

  void XMLParser::buildTypeSystemSpecifier(TypeSystemDescription & tsDesc,
    DOMElement * descElem,
    const icu::UnicodeString & xmlFileLoc) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_TYPE_SYSTEM_DESC)) == 0);
    
    buildTypeSystemDesc(tsDesc, descElem);
	
  }

  AnalysisEngineMetaData * XMLParser::buildAEMetaData(DOMElement * descElem,
      icu::UnicodeString const & xmlFileLoc) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_AE_METADATA)) == 0 ||
            XMLString::compareString(descElem->getNodeName(), convert(TAG_PROCESSING_RESOURCE_METADATA)) == 0 );

    AnalysisEngineMetaData * aeMetaData = new AnalysisEngineMetaData();

    DOMNodeList * children = descElem->getChildNodes();
    assert( EXISTS(children) );

    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      DOMElement * child = (DOMElement *) children->item(i);
      const icu::UnicodeString & childTag = convert(child->getNodeName());
      if (childTag.compare(TAG_AE_NAME) == 0) {
        aeMetaData->setName(getSpannedText(child));
      } else if (childTag.compare(TAG_AE_DESCRIPTION) == 0) {
        aeMetaData->setDescription(getSpannedText(child));
      } else if (childTag.compare(TAG_AE_VERSION) == 0) {
        aeMetaData->setVersion(getSpannedText(child));
      } else if (childTag.compare(TAG_AE_VENDOR) == 0) {
        aeMetaData->setVendor(getSpannedText(child));
      } else if (childTag.compare(TAG_CONFIG_PARAMS) == 0) {
        buildConfigParams(*aeMetaData, child);
      } else if (childTag.compare(TAG_CONFIG_PARAM_SETTINGS) == 0) {
        buildConfigParamSettings(*aeMetaData, child);
      } else if (childTag.compare(TAG_TYPE_SYSTEM_DESC) == 0) {
		  if (aeMetaData->getTypeSystemDescription() == NULL) {
		    TypeSystemDescription * tsDesc = new TypeSystemDescription();
		    aeMetaData->setTypeSystemDescription(tsDesc);
		  }
          buildTypeSystemDesc( *(aeMetaData->getTypeSystemDescription()), child);
      } else if (childTag.compare(TAG_TYPE_PRIORITIES) == 0) {
        vector <icu::UnicodeString> alreadyImported;
        buildTypePriorities(*aeMetaData, child, xmlFileLoc, alreadyImported);
      } else if (childTag.compare(TAG_FS_INDEXES) == 0) {
        buildFSIndexes(*aeMetaData, child);
      } else if (childTag.compare(TAG_FS_INDEX_COLLECTION) == 0) {
        vector <icu::UnicodeString> alreadyImported;
        buildFSIndexCollection(*aeMetaData, child, alreadyImported, xmlFileLoc);
      } else if (childTag.compare(TAG_CAPABILITIES) == 0) {
        buildCapabilities(*aeMetaData, child);
      } else if (childTag.compare(TAG_FLOW) == 0) {
        aeMetaData->setFlowConstraints(buildFlowConstraints(child));
      } else if (childTag.compare(TAG_OPERATIONAL_PROPERTIES) == 0) {
        aeMetaData->setOperationalProperties(buildOperationalProperties(child));
      } else {
        /**
        ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
        errMsg.addParam(childTag);
        errMsg.addParam(schemaFileName);

        UIMA_EXC_THROW_NEW(InvalidXMLException,
                          UIMA_ERR_CONFIG_INVALID_XML_TAG,
                          errMsg,
                          UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                          ErrorInfo::unrecoverable);
        **/
      }
    }
    //aggregate TAEs don't contain a type system description,
    //but we have to ensure that one is set, otherwise merging the type systems will fail
    TypeSystemDescription * tsDesc = aeMetaData->getTypeSystemDescription();
    if (! (EXISTS(tsDesc)) ) {
      aeMetaData->setTypeSystemDescription(new TypeSystemDescription());
    }
    return aeMetaData;
  }

 

  void XMLParser::buildConfigParams(AnalysisEngineMetaData & aeMetaData,
      DOMElement * descElem) {
    //check for default group and search strategy
    const XMLCh* defaultGroup = descElem->getAttribute(convert(ATTR_CONFIG_PARAMS_DEF_GROUP));
    if (isDefined(defaultGroup)) {
      aeMetaData.setDefaultGroupName(convert(defaultGroup));
    }
    const XMLCh * searchStrategy = descElem->getAttribute(convert(ATTR_CONFIG_PARAMS_SEARCH));
    if (isDefined(searchStrategy)) {
      const icu::UnicodeString & strategy = convert(searchStrategy);
      if (strategy.compare(NO_FALLBACK) == 0) {
        aeMetaData.setSearchStrategy(AnalysisEngineMetaData::NONE);
      } else if (strategy.compare(DEFAULT_FALLBACK) == 0) {
        aeMetaData.setSearchStrategy(AnalysisEngineMetaData::DEFAULT_FALLBACK);
      } else if (strategy.compare(LANGUAGE_FALLBACK) == 0) {
        aeMetaData.setSearchStrategy(AnalysisEngineMetaData::LANGUAGE_FALLBACK);
      } else {
        /**
        ErrorMessage errMsg(UIMA_MSG_ID_EXC_CONFIG_XML_ATTRIBUTE_VALUE_NOT_ALLOWED);
        icu::UnicodeString tagName = convert( descElem->getTagName() );
        errMsg.addParam(strategy);
        errMsg.addParam(ATTR_CONFIG_PARAMS_SEARCH);
        errMsg.addParam(tagName);
        UIMA_EXC_THROW_NEW(InvalidXMLException,
                          UIMA_ERR_CONFIG_INVALID_XML_ATTRIBUTE_VALUE,
                          errMsg,
                          UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                          ErrorInfo::unrecoverable);
        **/
      }
    }

    DOMNodeList * children = descElem->getChildNodes();
    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }

      DOMElement * child = (DOMElement *) children->item(i);
      const icu::UnicodeString & childTag = convert(child->getNodeName());

      if (childTag.compare(TAG_CONFIG_PARAM_COMMON) == 0) {
        DOMNodeList * commonParms = child->getChildNodes();
        size_t j;
        for (j=0; j < commonParms->getLength(); j++) {
          if ((commonParms->item(j))->getNodeType() != DOMNode::ELEMENT_NODE) {
            continue;
          }
          aeMetaData.addCommonParameter(buildConfigParam((DOMElement *) commonParms->item(j)));
        }
      } else if (childTag.compare(TAG_CONFIG_PARAM) == 0) {
        aeMetaData.addConfigurationParameter(buildConfigParam(child));
      } else if (childTag.compare(TAG_CONFIG_PARAM_GROUP) == 0) {
        //contains the groups as space-separated string
        const icu::UnicodeString & groupNameString = convert(child->getAttribute(convert(ATTR_CONFIG_PARAM_GROUP_NAMES)));
        vector <std::string> groupNames;
        //TBD: change this to true unicode processing
        delimitedString2Vector(groupNames,
                               UnicodeStringRef(groupNameString).asUTF8(),
                               " ", true, false);

        //add the configuration groups, even if they don't define any parameters themselves
        //this way, later assignValue calls will succeed, otherwise, they would fail, as
        //assignValue requires that the configuration group must already exist
        for (size_t m=0; m < groupNames.size(); m++) {
          const icu::UnicodeString & groupName = icu::UnicodeString(groupNames[m].c_str(), "UTF-8");
          aeMetaData.addConfigurationGroup(groupName);
        }

        //get the config params defined for that group
        DOMNodeList * confParmsInGroup = child->getChildNodes();
        size_t k;
        for (k=0; k < confParmsInGroup->getLength(); k++) {
          if ((confParmsInGroup->item(k))->getNodeType() != DOMNode::ELEMENT_NODE) {
            continue;
          }

          //add the config param for each group
          size_t l;
          for (l=0; l < groupNames.size(); l++) {
            //we can't put the same configParam into the groups, as they will delete their
            //member in their destructor
            //tbd: build only once, copy the result, pass the copies into 'add'
            ConfigurationParameter * configParam = buildConfigParam((DOMElement *) confParmsInGroup->item(k));
            aeMetaData.addConfigurationParameter(icu::UnicodeString(groupNames[l].c_str(), "UTF-8"), configParam);
          }
        }
      }
    }
  }

  ConfigurationParameter * XMLParser::buildConfigParam(DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_CONFIG_PARAM)) == 0);
    DOMNodeList * children = descElem->getChildNodes();
    assert( EXISTS(children) );

    ConfigurationParameter * configParm = new ConfigurationParameter();

    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      DOMElement * child = (DOMElement *) children->item(i);
      const icu::UnicodeString & childTag = convert(child->getNodeName());
      if (childTag.compare(TAG_CONFIG_PARAM_NAME) == 0) {
        configParm->setName(getSpannedText(child));
      } else if (childTag.compare(TAG_CONFIG_PARAM_DESC) == 0) {
        configParm->setDescription(getSpannedText(child));
      } else if (childTag.compare(TAG_CONFIG_PARAM_TYPE) == 0) {
        const icu::UnicodeString & typeName = getSpannedText(child);
        if (typeName.compare(CONFIG_PARAM_BOOLEAN_TYPE) == 0) {
          configParm->setType(ConfigurationParameter::BOOLEAN);
        } else if (typeName.compare(CONFIG_PARAM_FLOAT_TYPE) == 0) {
          configParm->setType(ConfigurationParameter::FLOAT);
        } else if (typeName.compare(CONFIG_PARAM_INTEGER_TYPE) == 0) {
          configParm->setType(ConfigurationParameter::INTEGER);
        } else if (typeName.compare(CONFIG_PARAM_STRING_TYPE) == 0) {
          configParm->setType(ConfigurationParameter::STRING);
        } else {
          /**
          ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
          errMsg.addParam(typeName);
          errMsg.addParam(schemaFileName);
          UIMA_EXC_THROW_NEW(InvalidXMLException,
                            UIMA_ERR_CONFIG_INVALID_XML_TAG,
                            errMsg,
                            UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                            ErrorInfo::unrecoverable);        
          **/
        }
      } else if (childTag.compare(TAG_CONFIG_PARAM_MANDATORY) == 0) {
        configParm->setMandatory(isTrue(getSpannedText(child)));
      } else if (childTag.compare(TAG_CONFIG_PARAM_MULTIVAL) == 0) {
        configParm->setMultiValued((ConfigurationParameter::EnParameterAggregation)
                                   isTrue(getSpannedText(child)));
      } else if (childTag.compare(TAG_CONFIG_PARAM_RESTRICT) == 0) {
        //contains the AnC keys as space-separated string
        const icu::UnicodeString & ancKeyString = getSpannedText(child);
        vector <std::string> ancKeys;
        //TBD: change this to true unicode processing
        delimitedString2Vector(ancKeys,
                               UnicodeStringRef(ancKeyString).asUTF8(),
                               " ", true, false);

        for (size_t i=0; i < ancKeys.size(); i++) {
          const icu::UnicodeString & key = icu::UnicodeString(ancKeys[i].c_str(), "UTF-8");
          configParm->addRestriction(key);
        }
      } else {  /**
                        ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
                        errMsg.addParam(childTag);
                        errMsg.addParam(schemaFileName);
                        UIMA_EXC_THROW_NEW(InvalidXMLException,
                                          UIMA_ERR_CONFIG_INVALID_XML_TAG,
                                          errMsg,
                                          UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                                          ErrorInfo::unrecoverable);
                         **/
      }
    }

    return configParm;
  }

  
 

  void XMLParser::buildConfigParamSettings(AnalysisEngineMetaData & aeMetadata,
      DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_CONFIG_PARAM_SETTINGS)) == 0);
    DOMNodeList * children = descElem->getChildNodes();

    if (! EXISTS(children)) { //there are no settings defined
      return;
    }

    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      DOMElement * child = (DOMElement *) children->item(i);
      const icu::UnicodeString & childTag = convert(child->getNodeName());
      if (childTag.compare(TAG_CONFIG_PARAM_SETTING_GROUP) == 0) {
        //extract the group names
        const icu::UnicodeString & groupNameString = convert(child->getAttribute(convert(ATTR_CONFIG_PARAM_SETTING_GROUP_NAMES)));
        vector <std::string> groupNames;
        //TBD: change this to true unicode processing
        delimitedString2Vector(groupNames,
                               UnicodeStringRef(groupNameString).asUTF8(),
                               " ", true, false);
        DOMNodeList * nvPairsForGroup = child->getChildNodes();
        size_t k;
        for (k=0; k < nvPairsForGroup->getLength(); k++) {
          if ((nvPairsForGroup->item(k))->getNodeType() != DOMNode::ELEMENT_NODE) {
            continue;
          }
          size_t l;
          for (l=0; l < groupNames.size(); l++) {
            //we can't put the same nvPair into the groups, as they will delete their
            //member in their destructor
            //tbd: build only once, copy the result, pass the copies into 'add'
            NameValuePair * nvPair = buildNameValuePair((DOMElement *) nvPairsForGroup->item(k));
            icu::UnicodeString groupName(groupNames[l].c_str(), "UTF-8");
            //check for duplicate name value pairs
            //as a side effect, this also ensures that all name value pairs are defined
            //as configuration parameters
            if (EXISTS(aeMetadata.getNameValuePairNoFallback(groupName, nvPair->getName(), ""))) {
              ErrorMessage msg(UIMA_MSG_ID_EXC_CONFIG_DUPLICATE_NAME_VALUE_PAIR);
              msg.addParam(nvPair->getName());
              UIMA_EXC_THROW_NEW(DuplicateConfigElemException,
                                 UIMA_ERR_CONFIG_DUPLICATE_NAME_VALUE_PAIR,
                                 msg,
                                 UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                                 ErrorInfo::recoverable);
            }
            aeMetadata.addNameValuePair(groupName, nvPair);
          }
        }
      } else if (childTag.compare(TAG_NAME_VALUE_PAIR) == 0) {
        NameValuePair * nvPair = buildNameValuePair(child);
        //check for duplicate name value pairs
        //as a side effect, this also ensures that all name value pairs are defined
        //as configuration parameters
        if (EXISTS(aeMetadata.getNameValuePair(nvPair->getName()))) {
          ErrorMessage msg(UIMA_MSG_ID_EXC_CONFIG_DUPLICATE_NAME_VALUE_PAIR);
          msg.addParam(nvPair->getName());
          UIMA_EXC_THROW_NEW(DuplicateConfigElemException,
                             UIMA_ERR_CONFIG_DUPLICATE_NAME_VALUE_PAIR,
                             msg,
                             UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                             ErrorInfo::recoverable);
        }
        aeMetadata.addNameValuePair(nvPair);
      } else {
        /**
        ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
        errMsg.addParam(childTag);
        errMsg.addParam(schemaFileName);
        UIMA_EXC_THROW_NEW(InvalidXMLException,
                          UIMA_ERR_CONFIG_INVALID_XML_TAG,
                          errMsg,
                          UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                          ErrorInfo::unrecoverable);
        **/
      }
    }
  }


  NameValuePair * XMLParser::buildNameValuePair(DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_NAME_VALUE_PAIR)) == 0);
    DOMNodeList * children = descElem->getChildNodes();
    assert( EXISTS(children) );

    NameValuePair * nvPair = new NameValuePair();
    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      DOMElement * child = (DOMElement *) children->item(i);
      const icu::UnicodeString & childTag = convert(child->getNodeName());
      if (childTag.compare(TAG_NAME_VALUE_PAIR_NAME) == 0) {
        nvPair->setName(getSpannedText(child));
      } else if (childTag.compare(TAG_NAME_VALUE_PAIR_VALUE) == 0) {
        buildValue(*nvPair, child);
      } else {
        /**
        ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
        errMsg.addParam(childTag);
        errMsg.addParam(schemaFileName);
        UIMA_EXC_THROW_NEW(InvalidXMLException,
                          UIMA_ERR_CONFIG_INVALID_XML_TAG,
                          errMsg,
                          UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                          ErrorInfo::unrecoverable);
        **/
      }
    }
    return nvPair;
  }

  void XMLParser::buildValue(NameValuePair & nvPair, DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_NAME_VALUE_PAIR_VALUE)) == 0);
    DOMNodeList * children = descElem->getChildNodes();
    assert( EXISTS(children) );

    //assert( children->getLength() == 1);
    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }

      DOMElement * child = (DOMElement *) children->item(i);
      const icu::UnicodeString & childTag = convert(child->getNodeName());

      if (childTag.compare(TAG_NAME_VALUE_PAIR_VALUE_ARRAY) == 0) {
        DOMNodeList * arrayEntries = child->getChildNodes();
        assert( EXISTS(arrayEntries) );
        //the type must be the same for all elements in the array
        nvPair.define(findTypeForValue(findFirstElementNode(arrayEntries)), ConfigurationParameter::MULTIPLE_VALUES);
        //now iterate over entries, adding the values
        size_t j;
        for (j=0; j < arrayEntries->getLength(); j++) {
          if ((arrayEntries->item(j))->getNodeType() != DOMNode::ELEMENT_NODE) {
            continue;
          }
          nvPair.addValue(getSpannedText(arrayEntries->item(j)));
        }
      } else {
        nvPair.define(findTypeForValue(child), ConfigurationParameter::SINGLE_VALUE); // findFirstElementNode(child->getChildNodes())));
        nvPair.setValue(getSpannedText(child));
      }
    }
  }

  ConfigurationParameter::EnParameterType XMLParser::findTypeForValue(DOMNode * descElem) {
    assert(EXISTS(descElem));
    const icu::UnicodeString & tagName = convert(descElem->getNodeName());
    ConfigurationParameter::EnParameterType type;
    if (tagName.compare(TAG_NAME_VALUE_PAIR_VALUE_STRING) == 0) {
      type = ConfigurationParameter::STRING;
    } else if (tagName.compare(TAG_NAME_VALUE_PAIR_VALUE_INT) == 0) {
      type = ConfigurationParameter::INTEGER;
    } else if (tagName.compare(TAG_NAME_VALUE_PAIR_VALUE_FLOAT) == 0) {
      type = ConfigurationParameter::FLOAT;
    } else if (tagName.compare(TAG_NAME_VALUE_PAIR_VALUE_BOOL) == 0) {
      type = ConfigurationParameter::BOOLEAN;
    } else {
      ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
      errMsg.addParam(tagName);
      errMsg.addParam(ResourceManager::getInstance().getSchemaInfo() );
      UIMA_EXC_THROW_NEW(InvalidXMLException,
                         UIMA_ERR_CONFIG_INVALID_XML_TAG,
                         errMsg,
                         UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                         ErrorInfo::unrecoverable);

    }
    return type;
  }

  void XMLParser::buildTypeSystemDesc(TypeSystemDescription & typeSystemDesc, DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_TYPE_SYSTEM_DESC)) == 0);
    DOMNodeList * children = descElem->getChildNodes();
    assert( EXISTS(children) );
    //assert( children->getLength() == 1); //currently, there's only the 'types' element

    ///TypeSystemDescription * typeSystemDesc = new TypeSystemDescription();

    size_t j;
    for (j=0; j < children->getLength(); j++) {
      if ((children->item(j))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      DOMElement * child = (DOMElement *) children->item(j);
      const icu::UnicodeString & childTag = convert(child->getNodeName());
      if (childTag.compare(TAG_TYPES) == 0) {
        DOMNodeList * typeDescs = (children->item(j))->getChildNodes();
        if (EXISTS(typeDescs)) {
          size_t i;
          for (i=0; i < typeDescs->getLength(); i++) {
            if ((typeDescs->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
              continue;
            }
            bool takesMemoryOwnerShip;
            auto_ptr<TypeDescription> desc( buildTypeDesc((DOMElement *) typeDescs->item(i)) );
            typeSystemDesc.addTypeDescription(desc.get(), takesMemoryOwnerShip);
            if (takesMemoryOwnerShip) {
              desc.release();
            }
          }
        }
      } else if (childTag.compare(TAG_IMPORT_DESC) == 0) {
        bool takesMemoryOwnerShip;
        auto_ptr<ImportDescription> desc( buildImportDesc((DOMElement *) children->item(j)) );
        typeSystemDesc.addImportDescription(desc.get(), takesMemoryOwnerShip);
        if (takesMemoryOwnerShip) {
          desc.release();
        }
      } else if (childTag.compare(TAG_IMPORTS) == 0) {
        DOMNodeList * importDescs = (children->item(j))->getChildNodes();
        if (EXISTS(importDescs)) {
          size_t i;
          for (i=0; i < importDescs->getLength(); i++) {
            if ((importDescs->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
              continue;
            }
            bool takesMemoryOwnerShip;
            auto_ptr<ImportDescription> desc( buildImportDesc((DOMElement *) importDescs->item(i)) );
            typeSystemDesc.addImportDescription(desc.get(), takesMemoryOwnerShip);
            if (takesMemoryOwnerShip) {
              desc.release();
            }
          }
        }
      }
    }
    return;
  }

  ImportDescription * XMLParser::buildImportDesc(DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_IMPORT_DESC)) == 0);

    ImportDescription * importDesc = new ImportDescription();

    const XMLCh* location = descElem->getAttribute(convert(ATTR_IMPORT_DESC_LOCATION));
    if (isDefined(location)) {
      importDesc->setLocation(convert(location));
    } else {
      const XMLCh* name = descElem->getAttribute(convert(ATTR_IMPORT_DESC_NAME));
      if (isDefined(name)) {
        importDesc->setName(convert(name));
      } else {
        ErrorMessage errMsg(UIMA_MSG_ID_EXC_CONFIG_XML_ATTRIBUTE_VALUE_NOT_ALLOWED);
        icu::UnicodeString tagName = convert( descElem->getTagName() );
        errMsg.addParam(TAG_IMPORT_DESC);
        UIMA_EXC_THROW_NEW(InvalidXMLException,
                           UIMA_ERR_CONFIG_INVALID_XML_ATTRIBUTE_VALUE,
                           errMsg,
                           UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                           ErrorInfo::unrecoverable);
      }
    }

    return importDesc;
  }

  TypeDescription * XMLParser::buildTypeDesc(DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_TYPE_DESC)) == 0);
    DOMNodeList * children = descElem->getChildNodes();
    assert( EXISTS(children) );

    TypeDescription * typeDesc = new TypeDescription();

    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      DOMElement * child = (DOMElement *) children->item(i);
      const icu::UnicodeString & childTag = convert(child->getNodeName());
      if (childTag.compare(TAG_TYPE_DESC_NAME) == 0) {
        typeDesc->setName(getSpannedText(child));
      } else if (childTag.compare(TAG_TYPE_DESC_SUPER) == 0) {
        typeDesc->setSuperTypeName(getSpannedText(child));
      } else if (childTag.compare(TAG_TYPE_DESC_DESC) == 0) {
        typeDesc->setDescription(getSpannedText(child));
      } else if (childTag.compare(TAG_TYPE_DESC_FEATURES) == 0) {
        DOMNodeList * features = child->getChildNodes();
        if (EXISTS(features)) {
          size_t j;
          for (j=0; j < features->getLength(); j++) {
            if ((features->item(j))->getNodeType() != DOMNode::ELEMENT_NODE) {
              continue;
            }
            auto_ptr<FeatureDescription> desc( buildFeatureDesc((DOMElement *) features->item(j)) );
            bool takesMemoryOwnership;
            typeDesc->addFeatureDescription(desc.get(), takesMemoryOwnership);
            if (takesMemoryOwnership) {
              desc.release();
            }
          }
        }
      } else if (childTag.compare(TAG_TYPE_DESC_ALLOWED_VALS) == 0) {
        DOMNodeList * allowedVals = child->getChildNodes();
        if (EXISTS(allowedVals)) {
          size_t j;
          for (j=0; j< allowedVals->getLength(); j++) {
            if ((allowedVals->item(j))->getNodeType() != DOMNode::ELEMENT_NODE) {
              continue;
            }
            auto_ptr<AllowedValue> desc( buildAllowedValue((DOMElement *) allowedVals->item(j)) );
            bool takesMemoryOwnership;
            typeDesc->addAllowedValue(desc.get(), takesMemoryOwnership);
            if (takesMemoryOwnership) {
              desc.release();
            }
          }
        }
      } else {
        /**
        ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
        errMsg.addParam(childTag);
        errMsg.addParam(schemaFileName);
        UIMA_EXC_THROW_NEW(InvalidXMLException,
                          UIMA_ERR_CONFIG_INVALID_XML_TAG,
                          errMsg,
                          UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                          ErrorInfo::unrecoverable);
        **/
      }
    }
    return typeDesc;
  }

  FeatureDescription * XMLParser::buildFeatureDesc(DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_TYPE_DESC_FEAT_DESC)) == 0);
    DOMNodeList * children = descElem->getChildNodes();
    assert( EXISTS(children) );

    FeatureDescription * featureDesc = new FeatureDescription();
    size_t i;
    for (i=0; i< children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      DOMElement * child = (DOMElement *) children->item(i);
      const icu::UnicodeString & childTag = convert(child->getNodeName());
      if (childTag.compare(TAG_TYPE_DESC_FEAT_DESC_NAME) == 0) {
        featureDesc->setName(getSpannedText(child));
      } else if (childTag.compare(TAG_TYPE_DESC_FEAT_DESC_RANGE) == 0) {
        featureDesc->setRangeTypeName(getSpannedText(child));
      } else if (childTag.compare(TAG_TYPE_DESC_FEAT_DESC_ELEMENT) == 0) {
        featureDesc->setElementType(getSpannedText(child));
      } else if (childTag.compare(TAG_TYPE_DESC_FEAT_DESC_DESC) == 0) {
        featureDesc->setDescription(getSpannedText(child));
	  } else if (childTag.compare(TAG_TYPE_DESC_FEAT_DESC_MULTREFS) == 0) {
		  if (isTrue(getSpannedText(child))) {
			featureDesc->setMultipleReferencesAllowed(true);
		  }
      } else {
        /**
        ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
        errMsg.addParam(childTag);
        errMsg.addParam(schemaFileName);
        UIMA_EXC_THROW_NEW(InvalidXMLException,
                          UIMA_ERR_CONFIG_INVALID_XML_TAG,
                          errMsg,
                          UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                          ErrorInfo::unrecoverable);
        **/
      }
    }
    return featureDesc;
  }

  AllowedValue * XMLParser::buildAllowedValue(DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_TYPE_DESC_ALLOWED_VALS_VAL)) == 0);
    DOMNodeList * children = descElem->getChildNodes();
    assert( EXISTS(children) );
    AllowedValue * allVal = new AllowedValue();

    size_t i;
    for (i=0; i< children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      DOMElement * child = (DOMElement *) children->item(i);
      const icu::UnicodeString & childTag = convert(child->getNodeName());
      if (childTag.compare(TAG_TYPE_DESC_ALLOWED_VALS_VAL_STRING) == 0) {
        allVal->setName(getSpannedText(child));
      } else if (childTag.compare(TAG_TYPE_DESC_ALLOWED_VALS_VAL_DESC) == 0) {
        allVal->setDescription(getSpannedText(child));
      } else {
        /**
        ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
        errMsg.addParam(childTag);
        errMsg.addParam(schemaFileName);
        UIMA_EXC_THROW_NEW(InvalidXMLException,
                          UIMA_ERR_CONFIG_INVALID_XML_TAG,
                          errMsg,
                          UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                          ErrorInfo::unrecoverable);
        **/
      }
    }
    return allVal;
  }


  void XMLParser::buildTypePriorities(AnalysisEngineMetaData::TyVecpTypePriorities & vecpTypePriorities,
      DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_TYPE_PRIORITIES)) == 0);
    DOMNodeList * children = descElem->getChildNodes();

    if (! EXISTS(children)) { // there may be no index descriptions
      return;
    }
    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      TypePriority * prio =  buildTypePriority((DOMElement *)children->item(i));
      vecpTypePriorities.push_back(prio);
    }
  }



  void XMLParser::buildTypePriorities(AnalysisEngineMetaData & aeMetaData,
      DOMElement * descElem,
      icu::UnicodeString const & xmlFileLoc,
      vector<icu::UnicodeString> & alreadyImportedLocations) {

    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_TYPE_PRIORITIES)) == 0);
    DOMNodeList * children = descElem->getChildNodes();

    if (! EXISTS(children)) { // there may be no type priorities
      return;
    }

    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }

      DOMElement * child = (DOMElement *) children->item(i);
      const icu::UnicodeString & childTag = convert(child->getNodeName());
      if (childTag.compare(TAG_TYPE_PRIORITY_LIST) == 0) {
        TypePriority * prio =  buildTypePriority((DOMElement *)child);
        aeMetaData.addTypePriority(prio);
      } else if (childTag.compare(TAG_IMPORTS)==0) {
        DOMNodeList * importDescs = child->getChildNodes();
        if (EXISTS(importDescs)) {
          size_t j;
          for (j=0; j < importDescs->getLength(); j++) {
            if ((importDescs->item(j))->getNodeType() != DOMNode::ELEMENT_NODE) {
              continue;
            }
            UnicodeString location = convert(((DOMElement *) importDescs->item(j))->getAttribute(convert(ATTR_IMPORT_DESC_LOCATION)));
            if (location.length() > 0)
              buildTypePriorityFromImportLocation(aeMetaData, location, xmlFileLoc, alreadyImportedLocations);
            else {
              //throw exception if import location not specified
              UnicodeString name = convert(((DOMElement *) importDescs->item(j))->getAttribute(convert(ATTR_IMPORT_DESC_NAME)));
              //throw exception if import location not set.
              ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_UNSUPPORTED_XML_ATTRIBUTE);
              errMsg.addParam(xmlFileLoc);
              if (name.length() > 0) {
                errMsg.addParam(ATTR_IMPORT_DESC_NAME);
              }
              UIMA_EXC_THROW_NEW(InvalidXMLException,
                                 UIMA_ERR_IMPORT_INVALID_XML_ATTRIBUTE,
                                 errMsg,
                                 UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                                 ErrorInfo::unrecoverable);

            }
            /**bool takesMemoryOwnerShip;
            auto_ptr<ImportDescription> desc( buildImportDesc((DOMElement *) importDescs->item(j)) );
            aeMetaData.addTypePriorityImportDescription(desc.get(), takesMemoryOwnerShip);
            if (takesMemoryOwnerShip) {
                desc.release();
            }**/
          }
        }
      } //TAG_IMPORT
    }   //for
  }

  TypePriority * XMLParser::buildTypePriority(DOMElement * specElem) {
    assert(EXISTS(specElem));
    assert( XMLString::compareString(specElem->getNodeName(), convert(TAG_TYPE_PRIORITY_LIST)) == 0);
    DOMNodeList * children = specElem->getChildNodes();
    assert(EXISTS(children));

    TypePriority * pPrio = new TypePriority();
    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      DOMElement * child = (DOMElement *) children->item(i);
      const icu::UnicodeString & childTag = convert(child->getNodeName());
      if (childTag.compare(TAG_TYPE_PRIORITY_LIST_TYPE) == 0) {
        pPrio->addType(getSpannedText(child));
      } else {
        /**
        ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
        errMsg.addParam(childTag);
        errMsg.addParam(schemaFileName);
        UIMA_EXC_THROW_NEW(InvalidXMLException,
                          UIMA_ERR_CONFIG_INVALID_XML_TAG,
                          errMsg,
                          UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                          ErrorInfo::unrecoverable);
        **/
      }
    }
    return pPrio;
  }


  void XMLParser::buildFSIndexes(AnalysisEngineMetaData::TyVecpFSIndexDescriptions & vecFSIndexDesc,
      DOMElement * descElem) {
    assert(EXISTS(descElem));


	if ( XMLString::compareString(descElem->getNodeName(), convert(TAG_FS_INDEX_COLLECTION)) == 0) {
      DOMNodeList * children = descElem->getChildNodes();
      if (! EXISTS(children)) { // there may be no index descriptions
        return;
      }
	  for (size_t i=0; i < children->getLength(); i++) {
        if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
          continue;
        }
		descElem = (DOMElement*) children->item(i);
		break;
	  }
	}

    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_FS_INDEXES)) == 0);
    DOMNodeList * children = descElem->getChildNodes();

    if (! EXISTS(children)) { // there may be no index descriptions
      return;
    }

    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      FSIndexDescription * indexDesc = buildFSIndexDesc((DOMElement *) children->item(i));
      vecFSIndexDesc.push_back(indexDesc);
      //aeMetaData.addFSIndexDescription(buildFSIndexDesc((DOMElement *) children->item(i)));
    }
  }

  void XMLParser::buildFSIndexes(AnalysisEngineMetaData & aeMetaData,
      DOMElement * descElem)   {
    assert(EXISTS(descElem));

    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_FS_INDEXES)) == 0);
    DOMNodeList * children = descElem->getChildNodes();

    if (! EXISTS(children)) { // there may be no index descriptions
      return;
    }

    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      aeMetaData.addFSIndexDescription(buildFSIndexDesc((DOMElement *) children->item(i)));
    }
  }


  void XMLParser::buildFSIndexCollection(AnalysisEngineMetaData & aeMetaData,
      DOMElement * descElem,
      vector<icu::UnicodeString> & alreadyImportedLocations,
      UnicodeString const & lastFileName) {

    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_FS_INDEX_COLLECTION)) == 0
            || XMLString::compareString(descElem->getNodeName(), convert(TAG_FS_INDEXES)) == 0);
    DOMNodeList * children = descElem->getChildNodes();

    if (! EXISTS(children)) { // there may be no index descriptions
      return;
    }

    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      DOMElement * child = (DOMElement *) children->item(i);
      const icu::UnicodeString & childTag = convert(child->getNodeName());
      if (childTag.compare(TAG_FS_INDEXES) == 0) {
        buildFSIndexes(aeMetaData, child);
      } else if (childTag.compare(TAG_IMPORTS)==0) {

        DOMNodeList * importDescs = child->getChildNodes();
        if (EXISTS(importDescs)) {
          size_t j;
          for (j=0; j < importDescs->getLength(); j++) {
            if ((importDescs->item(j))->getNodeType() != DOMNode::ELEMENT_NODE) {
              continue;
            }
            UnicodeString loc = convert(((DOMElement *) importDescs->item(j))->getAttribute(convert(ATTR_IMPORT_DESC_LOCATION)));
            if (loc.length() > 0) {
              buildFSIndexFromImportLocation(aeMetaData,
                                             loc,
                                             alreadyImportedLocations,
                                             lastFileName);
            } else {
              UnicodeString name = convert(((DOMElement *) importDescs->item(j))->getAttribute(convert(ATTR_IMPORT_DESC_NAME)));
              //throw exception if import location not set.
              ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_UNSUPPORTED_XML_ATTRIBUTE);
              errMsg.addParam(lastFileName);
              if (name.length() > 0) {
                errMsg.addParam(ATTR_IMPORT_DESC_NAME);
              }
              UIMA_EXC_THROW_NEW(InvalidXMLException,
                                 UIMA_ERR_IMPORT_INVALID_XML_ATTRIBUTE,
                                 errMsg,
                                 UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                                 ErrorInfo::unrecoverable);
            }
          }
        }
      }
    }
  }

  FSIndexDescription * XMLParser::buildFSIndexDesc(DOMElement * descElem) {
    assert(EXISTS(descElem));
	//cout << __FILE__ <<  " " << UnicodeString((UChar*)descElem->getNodeName()) << endl;
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_FS_INDEX_DESC)) == 0);
    DOMNodeList * children = descElem->getChildNodes();
    assert(EXISTS(children));

    FSIndexDescription * fsIndexDesc = new FSIndexDescription();

    size_t i;
    for (i=0; i< children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      DOMElement * child = (DOMElement *) children->item(i);
      const icu::UnicodeString & childTag = convert(child->getNodeName());
      if (childTag.compare(TAG_FS_INDEX_DESC_LABEL) == 0) {
        fsIndexDesc->setLabel(getSpannedText(child));
      } else if (childTag.compare(TAG_FS_INDEX_DESC_TYPE) == 0) {
        fsIndexDesc->setTypeName(getSpannedText(child));
      } else if (childTag.compare(TAG_FS_INDEX_DESC_KEYS) == 0) {
        DOMNodeList * indexKeys = child->getChildNodes();
        assert(EXISTS(indexKeys));
        size_t j;
        for (j=0; j< indexKeys->getLength(); j++ ) {
          if ((indexKeys->item(j))->getNodeType() != DOMNode::ELEMENT_NODE) {
            continue;
          }
          fsIndexDesc->addFSIndexKey(buildFSIndexKeyDesc((DOMElement *) indexKeys->item(j)));
        }
      } else if (childTag.compare(TAG_FS_INDEX_DESC_KIND) == 0) {
        const icu::UnicodeString & kind = getSpannedText(child);
        if (kind.compare(FS_INDEX_KEY_KIND_SORTED) == 0) {
          fsIndexDesc->setIndexKind(FSIndexDescription::SORTED);
        } else if (kind.compare(FS_INDEX_KEY_KIND_BAG) == 0) {
          fsIndexDesc->setIndexKind(FSIndexDescription::BAG);
        } else if (kind.compare(FS_INDEX_KEY_KIND_SET) == 0) {
          fsIndexDesc->setIndexKind(FSIndexDescription::SET);
        } else {
          /**
          ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
          errMsg.addParam(childTag);
          errMsg.addParam(schemaFileName);
          UIMA_EXC_THROW_NEW(InvalidXMLException,
                            UIMA_ERR_CONFIG_INVALID_XML_TAG,
                            errMsg,
                            UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                            ErrorInfo::unrecoverable);
          **/
        }
      } else {
        /**
        ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
        errMsg.addParam(childTag);
        errMsg.addParam(schemaFileName);
        UIMA_EXC_THROW_NEW(InvalidXMLException,
                          UIMA_ERR_CONFIG_INVALID_XML_TAG,
                          errMsg,
                          UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                          ErrorInfo::unrecoverable);
        **/
      }
    }
    return fsIndexDesc;
  }

  FSIndexKeyDescription * XMLParser::buildFSIndexKeyDesc(DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_FS_INDEX_KEY)) == 0);
    DOMNodeList * children = descElem->getChildNodes();
    assert(EXISTS(children));

    FSIndexKeyDescription * indexKeyDesc = new FSIndexKeyDescription();

    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      DOMElement * child = (DOMElement *) children->item(i);
      const icu::UnicodeString & childTag = convert(child->getNodeName());
      if (childTag.compare(TAG_FS_INDEX_KEY_FEAT) == 0) {
        indexKeyDesc->setFeatureName(getSpannedText(child));
      } else if (childTag.compare(TAG_FS_INDEX_KEY_COMP) == 0) {
        const icu::UnicodeString & compType = getSpannedText(child);
        if (compType.compare(FS_INDEX_KEY_COMP_STANDARD) == 0) {
          indexKeyDesc->setComparator(FSIndexKeyDescription::STANDARD);
        } else if (compType.compare(FS_INDEX_KEY_COMP_REVERSE) == 0) {
          indexKeyDesc->setComparator(FSIndexKeyDescription::REVERSE);
        } else {
          /**
          ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
          errMsg.addParam(compType);
          errMsg.addParam(schemaFileName);
          UIMA_EXC_THROW_NEW(InvalidXMLException,
                            UIMA_ERR_CONFIG_INVALID_XML_TAG,
                            errMsg,
                            UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                            ErrorInfo::unrecoverable);
          **/
        }
      } else if (childTag.compare(TAG_FS_INDEX_KEY_TYPE_PRIORITY) == 0) {
        indexKeyDesc->setIsTypePriority();
      } else {
        /**
        ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
        errMsg.addParam(childTag);
        errMsg.addParam(schemaFileName);
        UIMA_EXC_THROW_NEW(InvalidXMLException,
                          UIMA_ERR_CONFIG_INVALID_XML_TAG,
                          errMsg,
                          UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                          ErrorInfo::unrecoverable);
        **/
      }
    }
    return indexKeyDesc;
  }

  void XMLParser::buildCapabilities(AnalysisEngineMetaData & aeMetaData,
      DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_CAPABILITIES)) == 0);
    DOMNodeList * children = descElem->getChildNodes();
    assert(EXISTS(children));
    assert(children->getLength() > 0);

    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      buildCapability(aeMetaData, (DOMElement *) children->item(i));
    }
  }

  /**
  * Builds the capability from <code>descElem</code>. Uses the TypeSystemDescription in
  * aeMetaData to expand types. If these types do not exist in the TypeSystemDescription,
  * no expansion takes place: only they are added to the input/outputtypes.
  **/
  void XMLParser::buildCapability(AnalysisEngineMetaData & aeMetaData,
      DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_CAPABILITY)) == 0);
    DOMNodeList * children = descElem->getChildNodes();
    assert(EXISTS(children));

    Capability * capability = new Capability();
    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      DOMElement * child = (DOMElement *) children->item(i);
      const icu::UnicodeString & childTag = convert(child->getNodeName());

      if (childTag.compare(TAG_CAP_LANG_SUPP) == 0) {
        DOMNodeList * languages = child->getChildNodes();
        size_t j;
        for (j=0; j < languages->getLength(); j++) {
          if ((languages->item(j))->getNodeType() != DOMNode::ELEMENT_NODE) {
            continue;
          }
          assert(XMLString::compareString((languages->item(j))->getNodeName(), convert(TAG_CAP_LANG)) == 0);
          capability->addSupportedLanguage(getSpannedText(languages->item(j)));
        }
      } else {//it's either input or output capabilities
        Capability::EnTypeStyle typeStyle;
        if (childTag.compare(TAG_CAP_INPUTS) == 0) {
          typeStyle = Capability::INPUT;
        } else if (childTag.compare(TAG_CAP_OUTPUTS) == 0) {
          typeStyle = Capability::OUTPUT;
        }//BSI
        else if (childTag.compare(TAG_CAP_INPUT_SOFAS) == 0) {
          typeStyle = Capability::INPUTSOFA;
        } else if (childTag.compare(TAG_CAP_OUTPUT_SOFAS) == 0) {
          typeStyle = Capability::OUTPUTSOFA;
        } else {

          ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
          errMsg.addParam(childTag);
          errMsg.addParam(ResourceManager::getInstance().getSchemaInfo());
          UIMA_EXC_THROW_NEW(InvalidXMLException,
                             UIMA_ERR_CONFIG_INVALID_XML_TAG,
                             errMsg,
                             UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                             ErrorInfo::unrecoverable);

        }

        DOMNodeList * types = child->getChildNodes();
        size_t j;
        for (j=0; j < types->getLength(); j++) {
          DOMNode * node = types->item(j);
          if (node->getNodeType() != DOMNode::ELEMENT_NODE) {
            continue;
          }
          DOMElement * nodeElem = (DOMElement *) node;
          const icu::UnicodeString & nodeTag = convert(nodeElem->getNodeName());

          if (nodeTag.compare(TAG_CAP_TYPE) == 0) {
            capability->addCapabilityType(getSpannedText(nodeElem), typeStyle);
            bool expand = isTrue(convert(nodeElem->getAttribute(convert(ATTR_CAP_FEATURE_ALL))));
            if (expand) {
              //add all features of this type as well, based on the type information
              //in the current specifier
              icu::UnicodeString typeName(getSpannedText(nodeElem));
              TypeSystemDescription * cpTypeSystemDesc =  aeMetaData.getTypeSystemDescription();
              if (EXISTS(cpTypeSystemDesc)) { //if there's already a type description in place
                const TypeDescription * cpTypeDesc =  cpTypeSystemDesc ->getTypeDescriptionConst(typeName);
                if (EXISTS(cpTypeDesc)) {
                  const TypeDescription::TyVecpFeatureDescriptions & featDescs = cpTypeDesc->getFeatureDescriptions();
                  TypeDescription::TyVecpFeatureDescriptions::const_iterator ite;
                  for (ite = featDescs.begin(); ite != featDescs.end(); ite++) {
                    UnicodeString featName(typeName);
                    featName += UIMA_TYPE_FEATURE_SEPARATOR;
                    featName += (*ite)->getName();
                    capability->addCapabilityFeature(featName, typeStyle);
                  }
                }
              }
            }
          } else if (nodeTag.compare(TAG_CAP_FEATURE) == 0) {
            capability->addCapabilityFeature(getSpannedText(nodeElem), typeStyle);
          }
          //BSI
          else if (nodeTag.compare(TAG_CAP_SOFA_NAME) == 0) {
            capability->addCapabilitySofa(getSpannedText(nodeElem), typeStyle);
          } else {
            /**
            ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
            errMsg.addParam(nodeTag);
            errMsg.addParam(schemaFileName);
            UIMA_EXC_THROW_NEW(InvalidXMLException,
                              UIMA_ERR_CONFIG_INVALID_XML_TAG,
                              errMsg,
                              UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                              ErrorInfo::unrecoverable);
            **/
          }
        }
      }
    }
    aeMetaData.addCapability(capability);
  }


  FlowConstraints * XMLParser::buildFlowConstraints(DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString( descElem->getNodeName(), convert(TAG_FLOW) ) == 0);
    DOMNodeList * children = descElem->getChildNodes();
    assert(EXISTS(children));
    assert(children->getLength() > 0);

    FlowConstraints * flow = NULL;

    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      DOMElement * child = (DOMElement *) children->item(i);
      const icu::UnicodeString & childTag = convert(child->getNodeName());


      if (childTag.compare(TAG_FLOW_FIX) == 0) {
        flow = buildFixedFlow(child);
      } else if (childTag.compare(TAG_FLOW_CAPABILITY_LANG)==0) {
        flow = buildCapabilityLanguageFlow(child);
      } else {
        /**
        ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
        errMsg.addParam(childTag);
        errMsg.addParam(schemaFileName);
        UIMA_EXC_THROW_NEW(InvalidXMLException,
                          UIMA_ERR_CONFIG_INVALID_XML_TAG,
                          errMsg,
                          UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                          ErrorInfo::unrecoverable);
        **/
      }
    }
    return flow;
  }


  OperationalProperties * XMLParser::buildOperationalProperties(DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString( descElem->getNodeName(), convert(TAG_OPERATIONAL_PROPERTIES) ) == 0);
    DOMNodeList * children = descElem->getChildNodes();
    assert(EXISTS(children));
    assert(children->getLength() > 0);

    OperationalProperties * op = new OperationalProperties();

    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      DOMElement * child = (DOMElement *) children->item(i);
      const icu::UnicodeString & childTag = convert(child->getNodeName());

      if (childTag.compare(TAG_MODIFIES_CAS) == 0) {
        op->setModifiesCas(isTrue(getSpannedText(child)));
      } else if (childTag.compare(TAG_MULTIPLE_DEPLOY_ALLOWED)==0) {
        op->setMultipleDeploymentAllowed(isTrue(getSpannedText(child)));
      } else if (childTag.compare(TAG_OUTPUTS_NEW_CASES)==0) {
        op->setOutputsNewCASes(isTrue(getSpannedText(child)));
      } else {
        /**
        ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
        errMsg.addParam(childTag);
        errMsg.addParam(schemaFileName);
        UIMA_EXC_THROW_NEW(InvalidXMLException,
                          UIMA_ERR_CONFIG_INVALID_XML_TAG,
                          errMsg,
                          UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                          ErrorInfo::unrecoverable);
        **/
      }
    }
    return op;
  }




  FixedFlow * XMLParser::buildFixedFlow(DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_FLOW_FIX)) == 0);
    DOMNodeList * nodes = descElem->getChildNodes();
    assert(EXISTS(nodes));
    assert(nodes->getLength() > 0);

    FixedFlow * flow = new FixedFlow();

    size_t i;
    for (i=0; i < nodes->getLength(); i++) {
      if ((nodes->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      assert(XMLString::compareString((nodes->item(i))->getNodeName(), convert(TAG_FLOW_FIX_NODE)) == 0);
      flow->addNode(getSpannedText(nodes->item(i)));
    }
    return flow;
  }

  CapabilityLanguageFlow * XMLParser::buildCapabilityLanguageFlow(DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_FLOW_CAPABILITY_LANG)) == 0);
    DOMNodeList * nodes = descElem->getChildNodes();
    assert(EXISTS(nodes));
    assert(nodes->getLength() > 0);

    CapabilityLanguageFlow * flow = new CapabilityLanguageFlow();

    size_t i;
    for (i=0; i < nodes->getLength(); i++) {
      if ((nodes->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      assert(XMLString::compareString((nodes->item(i))->getNodeName(), convert(TAG_FLOW_CAPABILITY_LANG_NODE)) == 0);
      flow->addNode(getSpannedText(nodes->item(i)));
    }
    return flow;
  }


  void XMLParser::buildSofaMappings(AnalysisEngineDescription::TyVecpSofaMappings & sofaMappings,
      DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_SOFA_MAPPINGS)) == 0);
    DOMNodeList * children = descElem->getChildNodes();
    assert(EXISTS(children));
    assert(children->getLength() > 0);

    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      SofaMapping * pSofaMapping = buildSofaMapping( (DOMElement *) children->item(i));
      sofaMappings.push_back(pSofaMapping);
      ;
    }
  }

  void XMLParser::buildSofaMappings(AnalysisEngineDescription & taeSpec,
      DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_SOFA_MAPPINGS)) == 0);
    DOMNodeList * children = descElem->getChildNodes();
    assert(EXISTS(children));
    assert(children->getLength() > 0);

    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      SofaMapping * pSofaMapping = buildSofaMapping( (DOMElement *) children->item(i));
      taeSpec.addSofaMapping(pSofaMapping);
    }

  }

  /**
  * Builds the capability from <code>descElem</code>. Uses the TypeSystemDescription in
  * aeMetaData to expand types. If these types do not exist in the TypeSystemDescription,
  * no expansion takes place: only they are added to the input/outputtypes.
  **/
  SofaMapping * XMLParser::buildSofaMapping(DOMElement * descElem) {
    assert(EXISTS(descElem));
    assert( XMLString::compareString(descElem->getNodeName(), convert(TAG_SOFA_MAPPING)) == 0);
    DOMNodeList * children = descElem->getChildNodes();
    assert(EXISTS(children));

    SofaMapping * sofamapping = new SofaMapping();
    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if ((children->item(i))->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }
      DOMElement * child = (DOMElement *) children->item(i);
      const icu::UnicodeString & childTag = convert(child->getNodeName());


      if (childTag.compare(TAG_SOFAMAP_COMPONENT_KEY) == 0) {
        sofamapping->setComponentKey(getSpannedText(child));
      } else if (childTag.compare(TAG_SOFAMAP_COMPONENT_SOFA_NAME) == 0) {
        sofamapping->setComponentSofaName(getSpannedText(child));
      } else if (childTag.compare(TAG_SOFAMAP_AGGREGATE_SOFA_NAME) == 0) {
        sofamapping->setAggregateSofaName(getSpannedText(child));
      } else {
        /**
        ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
        errMsg.addParam(childTag);
        errMsg.addParam(schemaFileName);
        UIMA_EXC_THROW_NEW(InvalidXMLException,
                          UIMA_ERR_CONFIG_INVALID_XML_TAG,
                          errMsg,
                          UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                          ErrorInfo::unrecoverable);
        **/
      }
    }
    return sofamapping;
  }

void XMLParser::buildFSIndexFromImportLocation(AnalysisEngineMetaData& fsDesc,
      icu::UnicodeString const & fileName,
      vector<icu::UnicodeString> & alreadyImportedLocations,
      icu::UnicodeString const & lastFileName) {

    UnicodeString importfn = ResourceManager::resolveFilename(fileName, lastFileName);

    for (size_t i=0; i < alreadyImportedLocations.size() ;i++) {
      if (importfn.compare(alreadyImportedLocations[i]) == 0 )  {
        return;
      }
    }

    alreadyImportedLocations.push_back(importfn);
    size_t uiLen = importfn.length();
    auto_array<UChar> arBuffer( new UChar[uiLen + 1] );
    assert( EXISTS(arBuffer.get()));

    importfn.extract(0, uiLen, arBuffer.get());
    (arBuffer.get())[uiLen] = 0; // terminate the buffer with 0
    assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");

    // the following try catch block just to trigger an exception.
    // The constructor of LocalFileInputSource throws an exception on the UNIXes if the file does not
    // exist. On Windows, the parser throws this exception.
    try {
      LocalFileInputSource fileISOnlyForException((XMLCh const *) arBuffer.get() );
    } catch (XMLException const & e) {
      ErrorInfo errInfo;
      errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
      ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
      assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
      msg.addParam( arBuffer.get() );
      msg.addParam( 0 );
      msg.addParam( 0 );
      msg.addParam( (UChar const *) e.getMessage());
      errInfo.setMessage(msg);
      errInfo.setSeverity(ErrorInfo::unrecoverable);
      ExcIllFormedInputError exc(errInfo);
      throw exc;
    }

    LocalFileInputSource fileIS((XMLCh const *) arBuffer.get() );

    XercesDOMParser parser;
    parser.setValidationScheme(XercesDOMParser::Val_Auto);
    parser.setDoNamespaces(true);
    parser.setDoSchema(false);
    parser.setExternalSchemaLocation(UIMA_XML_NAMESPACE " ");

    bool bHasOwnErrorHandler = false;
    if (iv_pXMLErrorHandler == NULL) {
      iv_pXMLErrorHandler = new XMLErrorHandler();
      assert( EXISTS(iv_pXMLErrorHandler) );
      bHasOwnErrorHandler = true;
    }

    parser.setErrorHandler(iv_pXMLErrorHandler);
    try {
      parser.parse( fileIS );
    } catch (Exception e) {
      if (bHasOwnErrorHandler) {
        delete iv_pXMLErrorHandler;
        iv_pXMLErrorHandler=NULL;
      }
      throw  e;
    }
    DOMDocument* doc = parser.getDocument();
    // get top node
    DOMElement * descElem = doc->getDocumentElement();

    assert(EXISTS(descElem));

    if (bHasOwnErrorHandler ) {
      delete iv_pXMLErrorHandler;
      iv_pXMLErrorHandler =  NULL;
    }
    buildFSIndexCollection(fsDesc, descElem, alreadyImportedLocations, importfn);
  }

  void XMLParser::buildTypePriorityFromImportLocation(AnalysisEngineMetaData  & aeDesc,
      icu::UnicodeString const & fileName,
      icu::UnicodeString const & lastFileName,
      vector<icu::UnicodeString> & alreadyImportedLocations) {



    UnicodeString importfn = ResourceManager::resolveFilename(fileName, lastFileName);

    for (size_t i=0; i < alreadyImportedLocations.size(); i++) {
      if (importfn.compare(alreadyImportedLocations[i]) == 0 )  {
        return;
      }
    }
    alreadyImportedLocations.push_back(importfn);

    size_t uiLen = importfn.length();
    auto_array<UChar> arBuffer( new UChar[uiLen + 1] );
    assert( EXISTS(arBuffer.get()));

    importfn.extract(0, uiLen, arBuffer.get());
    (arBuffer.get())[uiLen] = 0; // terminate the buffer with 0
    assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");

    // the following try catch block just to trigger an exception.
    // The constructor of LocalFileInputSource throws an exception on the UNIXes if the file does not
    // exist. On Windows, the parser throws this exception.
    try {
      LocalFileInputSource fileISOnlyForException((XMLCh const *) arBuffer.get() );
    } catch (XMLException const & e) {
      ErrorInfo errInfo;
      errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
      ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
      assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
      msg.addParam( arBuffer.get() );
      msg.addParam( 0 );
      msg.addParam( 0 );
      msg.addParam( (UChar const *) e.getMessage());
      errInfo.setMessage(msg);
      errInfo.setSeverity(ErrorInfo::unrecoverable);
      ExcIllFormedInputError exc(errInfo);
      throw exc;
    }

    LocalFileInputSource fileIS((XMLCh const *) arBuffer.get() );

    XercesDOMParser parser;
    parser.setValidationScheme(XercesDOMParser::Val_Auto);
    parser.setDoNamespaces(true);
    parser.setDoSchema(false);
    parser.setExternalSchemaLocation(UIMA_XML_NAMESPACE " ");

    bool bHasOwnErrorHandler = false;
    if (iv_pXMLErrorHandler == NULL) {
      iv_pXMLErrorHandler = new XMLErrorHandler();
      assert( EXISTS(iv_pXMLErrorHandler) );
      bHasOwnErrorHandler = true;
    }
    parser.setErrorHandler(iv_pXMLErrorHandler);
    try {
      parser.parse( fileIS );
    } catch (Exception e) {
      if (bHasOwnErrorHandler) {
        delete iv_pXMLErrorHandler;
        iv_pXMLErrorHandler = NULL;
      }
      throw e;
    }
    DOMDocument* doc = parser.getDocument();
    // get top node
    DOMElement * descElem = doc->getDocumentElement();

    assert(EXISTS(descElem));

    if (bHasOwnErrorHandler) {
      delete iv_pXMLErrorHandler;
      iv_pXMLErrorHandler = NULL;
    }

    buildTypePriorities(aeDesc, descElem, importfn, alreadyImportedLocations);
  }



  //--------------------------------------------------------------------
  //
  //TextAnalysisEngineSpecifierBuilder implementation
  //
  //--------------------------------------------------------------------
  TextAnalysisEngineSpecifierBuilder::TextAnalysisEngineSpecifierBuilder()
      :XMLParser(){}

  TextAnalysisEngineSpecifierBuilder::~TextAnalysisEngineSpecifierBuilder() {}
  void TextAnalysisEngineSpecifierBuilder::buildTae(AnalysisEngineDescription & taeSpec, DOMElement * specElem,
	  const icu::UnicodeString & xmlFileLoc) {
		buildAnalysisEngineDescription(taeSpec, specElem, xmlFileLoc);
  }

  void TextAnalysisEngineSpecifierBuilder::buildTaeFromMemory(AnalysisEngineDescription & taeSpec,
      char const * cpszXMLString) {
    MemBufInputSource memIS((XMLByte const *) cpszXMLString, strlen(cpszXMLString), "sysID");
    parseAnalysisEngineDescription(taeSpec, memIS);
  }


  void TextAnalysisEngineSpecifierBuilder::buildTaeFromMemory(AnalysisEngineDescription & taeSpec,
      icu::UnicodeString const & xmlString) {
	UnicodeStringRef uref(xmlString);
    MemBufInputSource memIS((XMLByte const *) uref.asUTF8().c_str(), uref.asUTF8().length(), "sysID");
    parseAnalysisEngineDescription(taeSpec, memIS);
  }

  void TextAnalysisEngineSpecifierBuilder::buildTaeFromFile(AnalysisEngineDescription & aeDesc,
      char const * fileName) {
	parseAnalysisEngineDescription(aeDesc, fileName);
  }


  void TextAnalysisEngineSpecifierBuilder::buildTaeFromFile(AnalysisEngineDescription & aeDesc,
      icu::UnicodeString const & fileName) {
	parseAnalysisEngineDescription(aeDesc, fileName);
  }

  TypeSystemDescription * TextAnalysisEngineSpecifierBuilder::buildTypeSystemSpecifierFromFile(
    char const * fileName) {
    // convert to unicode using default converter for platform
    icu::UnicodeString ustrFileName(fileName);
    return buildTypeSystemSpecifierFromFile(ustrFileName);
  }


  TypeSystemDescription * TextAnalysisEngineSpecifierBuilder::buildTypeSystemSpecifierFromFile(
    icu::UnicodeString const & fileName) {
    size_t uiLen = fileName.length();
    auto_array<UChar> arBuffer( new UChar[uiLen + 1] );
    assert( EXISTS(arBuffer.get()));

    fileName.extract(0, uiLen, arBuffer.get());
    (arBuffer.get())[uiLen] = 0; // terminate the buffer with 0
    assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");

    // the following try catch block just to trigger an exception.
    // The constructor of LocalFileInputSource throws an exception on the UNIXes if the file does not
    // exist. On Windows, the parser throws this exception.
    try {
      LocalFileInputSource fileISOnlyForException((XMLCh const *) arBuffer.get() );
    } catch (XMLException const & e) {
      ErrorInfo errInfo;
      errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
      ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
      assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
      msg.addParam( arBuffer.get() );
      msg.addParam( 0 );
      msg.addParam( 0 );
      msg.addParam( (UChar const *) e.getMessage());
      errInfo.setMessage(msg);
      errInfo.setSeverity(ErrorInfo::unrecoverable);
      ExcIllFormedInputError exc(errInfo);
      throw exc;
    }

    LocalFileInputSource fileIS((XMLCh const *) arBuffer.get() );
    TypeSystemDescription*  pDesc = new TypeSystemDescription();
	if ( !(EXISTS(pDesc)) ) {
		//TODO
		return NULL;
	}

    pDesc->setXmlFileLocation(fileName);
    parseTypeSystemDescription(*pDesc, fileIS);
    return pDesc;
  }


  TypeSystemDescription * TextAnalysisEngineSpecifierBuilder::buildTypeSystemSpecifierFromMemory(icu::UnicodeString const & xmlString) {

    std::string cpszXMLString = UnicodeStringRef(xmlString).asUTF8();
    return buildTypeSystemSpecifierFromXMLBuffer(cpszXMLString.c_str());
  }

  TypeSystemDescription * TextAnalysisEngineSpecifierBuilder::buildTypeSystemSpecifierFromXMLBuffer(char const * xmlString) {

    MemBufInputSource memIS((XMLByte const *) xmlString, strlen(xmlString), "sysID");
    TypeSystemDescription * tsDesc = new TypeSystemDescription();
	if (!EXISTS(tsDesc)) {
		//TODO: log error
		return NULL;
	}
	parseTypeSystemDescription(*tsDesc, memIS);
    return tsDesc;	
  }

  
  void TextAnalysisEngineSpecifierBuilder::buildTypePriorities(AnalysisEngineMetaData & aeMetaData,
												DOMElement * descElem,
												icu::UnicodeString const & xmlFileLoc,
												vector<icu::UnicodeString> & alreadyImportedLocations) {
	XMLParser::buildTypePriorities(aeMetaData, descElem, xmlFileLoc, alreadyImportedLocations);
  }


  void TextAnalysisEngineSpecifierBuilder::buildTypePriorities(AnalysisEngineMetaData::TyVecpTypePriorities & vecpTypePriorities,
												DOMElement * descElem) {
	XMLParser::buildTypePriorities(vecpTypePriorities, descElem);
  } 


  void TextAnalysisEngineSpecifierBuilder::buildFSIndexes(AnalysisEngineMetaData & aeMetaData,
												DOMElement * descElem)   {
	XMLParser::buildFSIndexes(aeMetaData, descElem);
  }
     

  void TextAnalysisEngineSpecifierBuilder::buildFSIndexes(AnalysisEngineMetaData::TyVecpFSIndexDescriptions & vecFSIndexDesc,
												DOMElement * descElem) {
	XMLParser::buildFSIndexes(vecFSIndexDesc, descElem);
  }


  void TextAnalysisEngineSpecifierBuilder::buildSofaMappings(AnalysisEngineDescription::TyVecpSofaMappings & sofaMappings,
												DOMElement * descElem) {
	XMLParser::buildSofaMappings(sofaMappings, descElem);
  }


  void TextAnalysisEngineSpecifierBuilder::buildConfigParams(AnalysisEngineMetaData & aeMetaData,
												DOMElement * descElem) {
	XMLParser::buildConfigParams(aeMetaData, descElem);
  }


  void TextAnalysisEngineSpecifierBuilder::buildConfigParamSettings(AnalysisEngineMetaData & aeMetadata,
												DOMElement * descElem) {
	XMLParser::buildConfigParamSettings(aeMetadata, descElem);
  }


  void TextAnalysisEngineSpecifierBuilder::appendToXMLBuffer(AnalysisEngineMetaData::TyVecpFSIndexDescriptions const & fsDesc,
												icu::UnicodeString &  xmlString) {
    AnalysisEngineDescription taeSpec;
    taeSpec.appendToXMLBuffer(fsDesc,xmlString);
  }


  void TextAnalysisEngineSpecifierBuilder::appendToXMLBuffer(AnalysisEngineMetaData::TyVecpTypePriorities const & prioDesc,
												icu::UnicodeString & xmlString) {
    AnalysisEngineDescription taeSpec;
    taeSpec.appendToXMLBuffer(prioDesc,xmlString);
  }


  void TextAnalysisEngineSpecifierBuilder::appendToXMLBuffer(AnalysisEngineDescription::TyVecpSofaMappings const & sofaMapDesc,
												icu::UnicodeString &  xmlString) {
    AnalysisEngineDescription taeSpec;
    taeSpec.appendToXMLBuffer(sofaMapDesc,xmlString);
  }


  void TextAnalysisEngineSpecifierBuilder::buildFromXMLBuffer(AnalysisEngineMetaData::TyVecpFSIndexDescriptions & fsDesc,
												icu::UnicodeString const & xmlString) {
    std::string xmlStr = UnicodeStringRef(xmlString).asUTF8();
    MemBufInputSource memIS((XMLByte const *) xmlStr.c_str(), xmlStr.length(), "sysID");
    parseFSIndexDescription(fsDesc, memIS);
  }

  void TextAnalysisEngineSpecifierBuilder::buildFromXMLBuffer(AnalysisEngineMetaData::TyVecpTypePriorities  & prioDesc,
												icu::UnicodeString const & xmlString) {
	std::string xmlStr = UnicodeStringRef(xmlString).asUTF8();
    MemBufInputSource memIS((XMLByte const *) xmlStr.c_str(), xmlStr.length(), "sysID");   
    parseTypePriorities(prioDesc, memIS);
  }


  void TextAnalysisEngineSpecifierBuilder::buildFromXMLBuffer(AnalysisEngineDescription::TyVecpSofaMappings & sofaMapDesc,
												icu::UnicodeString const & xmlString) {
	std::string xmlStr = UnicodeStringRef(xmlString).asUTF8();
    MemBufInputSource memIS((XMLByte const *) xmlStr.c_str(), xmlStr.length(), "sysID");
    parseSofaMappings(sofaMapDesc, memIS);
  }


  

 
  //----------------------------------------------------
  //
  //XMLParser private methods.
  //----------------------------------------------------
  bool XMLParser::isTrue(const icu::UnicodeString & value) const {
    /*       cout << "Buffer:" << convert(cpUCBuf) << endl;                       */
    /*       return(XMLString::compareString(cpUCBuf, convert(TRUE_VALUE)) == 0); */
    return(value.compare(TRUE_VALUE) == 0);
  }

  XMLCh const * XMLParser::convert(char const * cpBuf) const {
    bool bTranscodeSuccess = XMLString::transcode( cpBuf, gs_tempXMLChBuffer, MAXXMLCHBUFF -1 );
    assert( bTranscodeSuccess );
    return gs_tempXMLChBuffer;
  }

  UnicodeString XMLParser::convert( XMLCh const * cpUCBuf ) const {
    assertWithMsg( sizeof(XMLCh) == sizeof(UChar), "Port required!");
    if (EXISTS(cpUCBuf)) {
      unsigned int uiLen = XMLString::stringLen( cpUCBuf );
      return UnicodeString( (UChar const *) cpUCBuf, uiLen);
    } else {
      return "";
    }
  }

  icu::UnicodeString XMLParser::getSpannedText(DOMNode * node) {
    assert(EXISTS(node));
    DOMNodeList * children = node->getChildNodes();
    assert(EXISTS(children));
    icu::UnicodeString spannedText;
    size_t i;
    for (i=0; i < children->getLength(); i++) {
      if (children->item(i)->getNodeType() == DOMNode::TEXT_NODE) {
        spannedText.append(convert(children->item(i)->getNodeValue()));
      } else {
        const icu::UnicodeString & childTag = convert((children->item(i))->getNodeName());
        if (childTag.compare(TAG_ENV_VAR_REF) == 0) {
          icu::UnicodeString envVarName(convert(children->item(i)->getFirstChild()->getNodeValue()));
          util::EnvironmentVariableQueryOnly clEnvVar(UnicodeStringRef(envVarName).asUTF8().c_str());
          spannedText.append(clEnvVar.getValue());
        } else {
          /**
          ErrorMessage errMsg(UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG);
          errMsg.addParam(childTag);
          errMsg.addParam(schemaFileName);
          UIMA_EXC_THROW_NEW(InvalidXMLException,
                            UIMA_ERR_CONFIG_INVALID_XML_TAG,
                            errMsg,
                            UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC,
                            ErrorInfo::unrecoverable);
          **/
        }
      }
    }
    spannedText.trim();
    return spannedText;
  }

  DOMNode * XMLParser::findFirst(DOMNodeList * nodes,
      char const * tagName) {
    assert(EXISTS(nodes));
    DOMNode * found = NULL;
    size_t i;
    for (i=0; i < nodes->getLength(); i++) {
      if (nodes->item(i)->getNodeType() != DOMNode::ELEMENT_NODE) {
        continue;
      }

      if (XMLString::compareString((nodes->item(i))->getNodeName(), convert(tagName)) == 0) {
        found = nodes->item(i);
        break;
      }
    }
    return found;
  }

  DOMElement * XMLParser::findFirstElementNode(DOMNodeList * nodes) {
    assert(EXISTS(nodes));
    DOMElement * found = NULL;
    size_t i;
    for (i=0; i < nodes->getLength() && (! EXISTS(found)) ; i++) {
      if (nodes->item(i)->getNodeType() == DOMNode::ELEMENT_NODE) {
        found = (DOMElement *) nodes->item(i);
      }
    }
    return found;
  }

  /**
   * If an attribute doesn't exist, xerces doesn't return a null pointer,
   * but an empty string
  **/
  bool XMLParser::isDefined(const XMLCh * attrVal) const {
    return(EXISTS(attrVal) && (XMLString::compareString(attrVal, convert("")) != 0));
  }

  char const * XMLParser::TAG_AE_DESC="analysisEngineDescription";
  char const * XMLParser::TAG_TAE_DESC="taeDescription";
  char const * XMLParser::TAG_CAS_DESC="casDescription";

  char const * XMLParser::TAG_RESMGR_CONFIG_DESC="resourceManagerConfiguration";
  char const * XMLParser::TAG_CASCONSUMER_DESC="casConsumerDescription";
  char const * XMLParser::TAG_PROCESSING_RESOURCE_METADATA="processingResourceMetaData";
  char const * XMLParser::TAG_IMPL_NAME="implementationName";

  char const * XMLParser::TAG_TAE_PRIMITIVE="primitive";
  char const * XMLParser::TAG_DELEGATE_AES="delegateAnalysisEngineSpecifiers";
  char const * XMLParser::TAG_DELEGATE_AE="delegateAnalysisEngine";

  char const * XMLParser::TAG_DELEGATE_AE_INCLUDE="includeFile";
  char const * XMLParser::TAG_AN_IMPL_NAME="annotatorImplementationName";
  char const * XMLParser::UIMA_FRAMEWORK_IMP="frameworkImplementation";

  char const * XMLParser::TAG_EXTERNAL_RESOURCE_DEPENDENCIES="externalResourceDependencies";
  char const * XMLParser::TAG_EXTERNAL_RESOURCES="externalResources";

  char const * XMLParser::TAG_AE_METADATA="analysisEngineMetaData";
  char const * XMLParser::TAG_AE_NAME="name";
  char const * XMLParser::TAG_AE_DESCRIPTION="description";
  char const * XMLParser::TAG_AE_VERSION="version";
  char const * XMLParser::TAG_AE_VENDOR="vendor";

  char const * XMLParser::TAG_CONFIG_PARAMS="configurationParameters";

  char const * XMLParser::TAG_CONFIG_PARAM_GROUP="configurationGroup";
  char const * XMLParser::TAG_CONFIG_PARAM_COMMON="commonParameters";

  char const * XMLParser::TAG_CONFIG_PARAM="configurationParameter";
  char const * XMLParser::TAG_CONFIG_PARAM_NAME="name";
  char const * XMLParser::TAG_CONFIG_PARAM_DESC="description";
  char const * XMLParser::TAG_CONFIG_PARAM_TYPE="type";
  char const * XMLParser::TAG_CONFIG_PARAM_MULTIVAL="multiValued";
  char const * XMLParser::TAG_CONFIG_PARAM_MANDATORY="mandatory";
  char const * XMLParser::TAG_CONFIG_PARAM_RESTRICT="restrictOverrideTo";

  char const * XMLParser::TAG_CONFIG_PARAM_SETTINGS="configurationParameterSettings";

  char const * XMLParser::TAG_CONFIG_PARAM_SETTING_GROUP="settingsForGroup";
  char const * XMLParser::TAG_NAME_VALUE_PAIR="nameValuePair";
  char const * XMLParser::TAG_NAME_VALUE_PAIR_NAME="name";
  char const * XMLParser::TAG_NAME_VALUE_PAIR_VALUE="value";
  char const * XMLParser::TAG_NAME_VALUE_PAIR_VALUE_STRING="string";
  char const * XMLParser::TAG_NAME_VALUE_PAIR_VALUE_INT="integer";
  char const * XMLParser::TAG_NAME_VALUE_PAIR_VALUE_FLOAT="float";
  char const * XMLParser::TAG_NAME_VALUE_PAIR_VALUE_BOOL="boolean";
  char const * XMLParser::TAG_NAME_VALUE_PAIR_VALUE_ARRAY="array";

  char const * XMLParser::TAG_ENV_VAR_REF="envVarRef";

  char const * XMLParser::TAG_IMPORTS="imports";
  char const * XMLParser::TAG_IMPORT_DESC="import";
  char const * XMLParser::ATTR_IMPORT_DESC_NAME="name";
  char const * XMLParser::ATTR_IMPORT_DESC_LOCATION="location";

  char const * XMLParser::TAG_TYPE_SYSTEM_DESC="typeSystemDescription";

  char const * XMLParser::TAG_TYPES="types";
  char const * XMLParser::TAG_TYPE_DESC="typeDescription";
  char const * XMLParser::TAG_TYPE_DESC_NAME="name";
  char const * XMLParser::TAG_TYPE_DESC_SUPER="supertypeName";
  char const * XMLParser::TAG_TYPE_DESC_DESC="description";
  char const * XMLParser::TAG_TYPE_DESC_ALLOWED_VALS="allowedValues";
  char const * XMLParser::TAG_TYPE_DESC_ALLOWED_VALS_VAL="value";
  char const * XMLParser::TAG_TYPE_DESC_ALLOWED_VALS_VAL_STRING="string";
  char const * XMLParser::TAG_TYPE_DESC_ALLOWED_VALS_VAL_DESC="description";
  char const * XMLParser::TAG_TYPE_DESC_FEATURES="features";
  char const * XMLParser::TAG_TYPE_DESC_FEAT_DESC="featureDescription";
  char const * XMLParser::TAG_TYPE_DESC_FEAT_DESC_NAME="name";
  char const * XMLParser::TAG_TYPE_DESC_FEAT_DESC_RANGE="rangeTypeName";
  char const * XMLParser::TAG_TYPE_DESC_FEAT_DESC_ELEMENT="elementType";
  char const * XMLParser::TAG_TYPE_DESC_FEAT_DESC_DESC="description";
  char const * XMLParser::TAG_TYPE_DESC_FEAT_DESC_MULTREFS="multipleReferencesAllowed";

  char const * XMLParser::TAG_TYPE_PRIORITIES="typePriorities";
  char const * XMLParser::TAG_TYPE_PRIORITY_LIST="priorityList";
  char const * XMLParser::TAG_TYPE_PRIORITY_LIST_TYPE="type";

  char const * XMLParser::TAG_FS_INDEXES="fsIndexes";
  char const * XMLParser::TAG_FS_INDEX_COLLECTION="fsIndexCollection";


  char const * XMLParser::TAG_FS_INDEX_DESC="fsIndexDescription";
  char const * XMLParser::TAG_FS_INDEX_DESC_LABEL="label";
  char const * XMLParser::TAG_FS_INDEX_DESC_TYPE="typeName";
  char const * XMLParser::TAG_FS_INDEX_DESC_KIND="kind";
  char const * XMLParser::TAG_FS_INDEX_DESC_KEYS="keys";
  char const * XMLParser::TAG_FS_INDEX_KEY="fsIndexKey";
  char const * XMLParser::TAG_FS_INDEX_KEY_FEAT="featureName";
  char const * XMLParser::TAG_FS_INDEX_KEY_COMP="comparator";
  char const * XMLParser::TAG_FS_INDEX_KEY_TYPE_PRIORITY="typePriority";

  char const * XMLParser::TAG_CAPABILITIES="capabilities";

  char const * XMLParser::TAG_CAPABILITY="capability";
  char const * XMLParser::TAG_CAP_INPUTS="inputs";
  char const * XMLParser::TAG_CAP_OUTPUTS="outputs";
  char const * XMLParser::TAG_CAP_TYPE="type";
  /*    char const * XMLParser::TAG_CAP_FEATURES="features"; */
  char const * XMLParser::TAG_CAP_FEATURE="feature";
  char const * XMLParser::TAG_CAP_LANG_SUPP="languagesSupported";
  char const * XMLParser::TAG_CAP_LANG="language";

  char const * XMLParser::TAG_CAP_INPUT_SOFAS="inputSofas";
  char const * XMLParser::TAG_CAP_OUTPUT_SOFAS="outputSofas";
  char const * XMLParser::TAG_CAP_SOFA_NAME="sofaName";
  char const * XMLParser::TAG_SOFA_MAPPINGS="sofaMappings";
  char const * XMLParser::TAG_SOFA_MAPPING="sofaMapping";
  char const * XMLParser::TAG_SOFAMAP_COMPONENT_KEY="componentKey";
  char const * XMLParser::TAG_SOFAMAP_COMPONENT_SOFA_NAME="componentSofaName";
  char const * XMLParser::TAG_SOFAMAP_AGGREGATE_SOFA_NAME="aggregateSofaName";

  char const * XMLParser::TAG_FLOW="flowConstraints";
  char const * XMLParser::TAG_FLOW_FIX="fixedFlow";
  char const * XMLParser::TAG_FLOW_FIX_NODE="node";

  char const * XMLParser::TAG_FLOW_CAPABILITY_LANG="capabilityLanguageFlow";
  char const * XMLParser::TAG_FLOW_CAPABILITY_LANG_NODE="node";

  char const * XMLParser::ATTR_DELEGATE_AE_KEY="key";
  char const * XMLParser::ATTR_DELEGATE_AE_INCLUDE_FILE="href";

  char const * XMLParser::ATTR_CONFIG_PARAMS_DEF_GROUP="defaultGroup";
  char const * XMLParser::ATTR_CONFIG_PARAMS_SEARCH="searchStrategy";

  char const * XMLParser::ATTR_CONFIG_PARAM_GROUP_NAMES="names";

  char const * XMLParser::ATTR_CONFIG_PARAM_SETTING_GROUP_NAMES="name";

  /*    char const * XMLParser::ATTR_CAP_TYPE_NAME="name";    */
  /*    char const * XMLParser::ATTR_CAP_FEATURE_NAME="name"; */

  char const * XMLParser::ATTR_CAP_FEATURE_ALL="allAnnotatorFeatures";

  char const * XMLParser::TRUE_VALUE="true";

  char const *  XMLParser::FRAMEWORK_IMP_CPLUSPLUS="org.apache.uima.cpp";
  char const *  XMLParser::FRAMEWORK_IMP_JAVA="org.apache.uima.java";

  char const * XMLParser::NO_FALLBACK="none";
  char const * XMLParser::DEFAULT_FALLBACK="default_fallback";
  char const * XMLParser::LANGUAGE_FALLBACK="language_fallback";

  char const * XMLParser::CONFIG_PARAM_STRING_TYPE="String";
  char const * XMLParser::CONFIG_PARAM_INTEGER_TYPE="Integer";
  char const * XMLParser::CONFIG_PARAM_FLOAT_TYPE="Float";
  char const * XMLParser::CONFIG_PARAM_BOOLEAN_TYPE="Boolean";

  char const * XMLParser::FS_INDEX_KEY_COMP_STANDARD="standard";
  char const * XMLParser::FS_INDEX_KEY_COMP_REVERSE="reverse";

  char const * XMLParser::FS_INDEX_KEY_KIND_SORTED="sorted";
  char const * XMLParser::FS_INDEX_KEY_KIND_BAG="bag";
  char const * XMLParser::FS_INDEX_KEY_KIND_SET="set";

  char const * XMLParser::TAG_OPERATIONAL_PROPERTIES="operationalProperties";
  char const * XMLParser::TAG_MODIFIES_CAS="modifiesCas";
  char const * XMLParser::TAG_MULTIPLE_DEPLOY_ALLOWED="multipleDeploymentAllowed";
  char const * XMLParser::TAG_OUTPUTS_NEW_CASES="outputsNewCASes";



  
}
