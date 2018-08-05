#include <rs/utils/RSXMLParser.h>


void RSXMLParser::parseAnalysisEngineDescription(uima::AnalysisEngineDescription& taeSpec,
                                                 const icu::UnicodeString& fileName,
                                                 const std::unordered_map<std::string, std::string>& delegateEngines)
{
  XMLPlatformUtils::Initialize();
  icu::UnicodeString const & fn = uima::ResourceManager::resolveFilename(fileName, ".");
  size_t uiLen = fn.length();
  uima::auto_array<UChar> arBuffer( new UChar[uiLen + 1] );
  assert( EXISTS(arBuffer.get()));

  fn.extract(0, uiLen, arBuffer.get());
  (arBuffer.get())[uiLen] = 0; // terminate the buffer with 0
  assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");

  // the following try catch block just to trigger an exception.
  // The constructor of LocalFileInputSource throws an exception on the UNIXes if the file does not
  // exist. On Windows, the parser throws this exception.
  try {
    LocalFileInputSource crInputSource((XMLCh const *) arBuffer.get() );
  } catch (XMLException const & e) {
    uima::ErrorInfo errInfo;
    errInfo.setErrorId((uima::TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
    uima::ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
    assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
    msg.addParam( arBuffer.get() );
    msg.addParam( 0 );
    msg.addParam( 0 );
    msg.addParam( (UChar const *) e.getMessage());
    errInfo.setMessage(msg);
    errInfo.setSeverity(uima::ErrorInfo::unrecoverable);
    uima::ExcIllFormedInputError exc(errInfo);
    throw exc;
  }

  LocalFileInputSource crInputSource((XMLCh const *) arBuffer.get() );

  XercesDOMParser parser;
  parser.setValidationScheme(XercesDOMParser::Val_Auto);
  parser.setDoNamespaces(true);
  if ( uima::ResourceManager::getInstance().doSchemaValidation() ) {
    parser.setDoSchema( uima::ResourceManager::getInstance().isSchemaAvailable());
  } else {
    parser.setDoSchema(false);
  }
  parser.setExternalSchemaLocation( uima::ResourceManager::getInstance().getSchemaInfo());
  bool bHasOwnErrorHandler = false;

  // if (iv_pXMLErrorHandler == NULL) {
  //   iv_pXMLErrorHandler = new XMLErrorHandler();
  //   assert( EXISTS(iv_pXMLErrorHandler) );
  //   bHasOwnErrorHandler = true;
  // }

  // parser.setErrorHandler(iv_pXMLErrorHandler);

  try {
    parser.parse( crInputSource );
  } catch (uima::Exception e){
    // if (bHasOwnErrorHandler) {
    //   delete iv_pXMLErrorHandler;
    //   iv_pXMLErrorHandler = NULL;
    // }
    throw(e);
  }

  DOMDocument* p_DOMDocument = parser.getDocument();
  assert(EXISTS(p_DOMDocument));

  // get top node
  DOMElement * p_RootElem = p_DOMDocument->getDocumentElement();
  assert(EXISTS(p_RootElem));

  // Add the delegate engines into the dom
  // DOMElement* child = rootElem->getFirstElementChild();
  // XMLSize_t cnt = rootElem->getChildElementCount();
  // for ( XMLSize_t i = 0; i < cnt; i++ )
  // {
  //   char* name = XMLString::transcode( child->getTagName() );
  //   outInfo( "========" << name << std::endl );
  //   XMLString::release( &name);

  //   char* value = XMLString::transcode(child->getTextContent());
  //   outInfo( "-------" << value << std::endl );
  //   XMLString::release( &value );

  //   child = child->getNextElementSibling();
  // }

  DOMElement * p_DelegateElement = NULL;

  XMLCh *tmpXMLCh = XMLString::transcode("delegateAnalysisEngineSpecifiers");
  p_DelegateElement = p_DOMDocument->createElement(tmpXMLCh);
  XMLString::release(&tmpXMLCh);

  for (auto del : delegateEngines)
  {
    tmpXMLCh = XMLString::transcode("delegateAnalysisEngine");
    DOMElement * p_DelAnno = p_DOMDocument->createElement(tmpXMLCh);
    XMLString::release(&tmpXMLCh);

    const XMLCh * annoName = XMLString::transcode(del.first.c_str());
    tmpXMLCh = XMLString::transcode("key");
    p_DelAnno->setAttribute(tmpXMLCh, annoName);

    tmpXMLCh = XMLString::transcode("import");
    DOMElement * p_Location = p_DOMDocument->createElement(tmpXMLCh);
    XMLString::release(&tmpXMLCh);

    tmpXMLCh = XMLString::transcode("location");
    const XMLCh * annoLoc = XMLString::transcode(del.second.c_str());
    p_Location->setAttribute(tmpXMLCh, annoLoc);
    XMLString::release(&tmpXMLCh);
    p_DelAnno->appendChild(p_Location);

    p_DelegateElement->appendChild(p_DelAnno);
  }

  DOMNodeList * children = p_DelegateElement->getChildNodes();

  p_RootElem->appendChild(p_DelegateElement);

  buildAnalysisEngineDescription(taeSpec, p_RootElem, convert(crInputSource.getSystemId()), true);
}
