#include <rs/utils/RSXMLParser.h>

// void DoOutput2File(DOMDocument* pmyDOMDocument, const char * FullFilePath )
// {

//   DOMImplementation    *pImplement     = NULL;
//   DOMLSSerializer      *pSerializer    = NULL;
//   XMLFormatTarget      *pTarget        = NULL;

//   /*
//   Return the first registered implementation that
//   has the desired features. In this case, we are after
//   a DOM implementation that has the LS feature... or Load/Save.
//   */
//   XMLCh *tmpXMLCh = XMLString::transcode("LS");
//   pImplement = DOMImplementationRegistry::getDOMImplementation(tmpXMLCh);

//   /*
//   From the DOMImplementation, create a DOMWriter.
//   DOMWriters are used to serialize a DOM tree [back] into an XML document.
//   */
//   pSerializer = ((DOMImplementationLS*)pImplement)->createLSSerializer();


//   /*
//   This line is optional. It just sets a feature
//   of the Serializer to make the output
//   more human-readable by inserting line-feeds,
//   without actually inserting any new elements/nodes
//   into the DOM tree. (There are many different features to set.)
//   Comment it out and see the difference.
//   */
//   DOMConfiguration* pDomConfiguration = pSerializer->getDomConfig();
//   pDomConfiguration->setParameter(XMLUni::fgDOMWRTFormatPrettyPrint, true);


//   /*
//   Choose a location for the serialized output. The 3 options are:
//       1) StdOutFormatTarget     (std output stream -  good for debugging)
//       2) MemBufFormatTarget     (to Memory)
//       3) LocalFileFormatTarget  (save to file)
//       (Note: You'll need a different header file for each one)
//       Don't forget to escape file-paths with a backslash character, or
//       just specify a file to be saved in the exe directory.

//         eg. c:\\example\\subfolder\\pfile.xml

//   */
//   pTarget = new LocalFileFormatTarget(FullFilePath);
//   // Write the serialized output to the target.
//   DOMLSOutput* pDomLsOutput = ((DOMImplementationLS*)pImplement)->createLSOutput();
//   pDomLsOutput->setByteStream(pTarget);

//   pSerializer->write(pmyDOMDocument, pDomLsOutput);
// }

void RSXMLParser::parseAnalysisEngineDescription(uima::AnalysisEngineDescription& taeSpec,
                                                 const std::unordered_map<std::string, std::string>& delegateEngines,
                                                 const icu::UnicodeString& fileName)
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

  // -----------------------------------------------------------------------
  DOMElement * p_DelegateElement = NULL;

  XMLCh *tmpXMLCh = XMLString::transcode("delegateAnalysisEngineSpecifiers");
  p_DelegateElement = p_DOMDocument->createElement(tmpXMLCh);
  XMLString::release(&tmpXMLCh);

  for (auto del : delegateEngines)
  {
    XMLCh * tmpDelegateEngine = XMLString::transcode("delegateAnalysisEngine");
    DOMElement * p_DelAnno = p_DOMDocument->createElement(tmpDelegateEngine);
    XMLString::release(&tmpDelegateEngine);

    XMLCh * annoName = XMLString::transcode(del.first.c_str());
    XMLCh * tmpKey = XMLString::transcode("key");
    p_DelAnno->setAttribute(tmpKey, annoName);
    XMLString::release(&tmpKey);
    XMLString::release(&annoName);

    XMLCh * tmpImport = XMLString::transcode("import");
    DOMElement * p_Location = p_DOMDocument->createElement(tmpImport);
    XMLString::release(&tmpImport);

    XMLCh * tmpLocation = XMLString::transcode("location");
    XMLCh * annoLoc = XMLString::transcode(del.second.c_str());
    p_Location->setAttribute(tmpLocation, annoLoc);
    XMLString::release(&tmpLocation);
    XMLString::release(&annoLoc);
    p_DelAnno->appendChild(p_Location);

    p_DelegateElement->appendChild(p_DelAnno);
  }

  p_RootElem->appendChild(p_DelegateElement);

  // -----------------------------------------------------------------------
  // DoOutput2File(p_DOMDocument, XMLString::transcode("/home/shixin.li/output.xml"));

  buildAnalysisEngineDescription(taeSpec, p_RootElem, convert(crInputSource.getSystemId()), true);
}
