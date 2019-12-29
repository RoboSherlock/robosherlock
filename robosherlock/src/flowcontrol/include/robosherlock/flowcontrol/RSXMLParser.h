#ifndef RSXMLPARSER_H
#define RSXMLPARSER_H

#include <robosherlock/utils/common.h>

#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/DOMNode.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/sax/ErrorHandler.hpp>
#include <xercesc/sax/InputSource.hpp>
#include <xercesc/framework/LocalFileInputSource.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLString.hpp>
//Required for outputing a Xerces DOMDocument to a standard output stream (Also see: XMLFormatTarget)
#include <xercesc/framework/StdOutFormatTarget.hpp>
//Required for outputing a Xerces DOMDocument to the file system (Also see: XMLFormatTarget)
#include <xercesc/framework/LocalFileFormatTarget.hpp>

#include <uima/api.hpp>
#include <uima/taespecifierbuilder.hpp>

#include <unicode/unistr.h>

#include <unordered_map>

XERCES_CPP_NAMESPACE_USE

class RSXMLParser : public uima::XMLParser
{
public:

  RSXMLParser() : uima::XMLParser() {};

  void parseAnalysisEngineDescription(uima::AnalysisEngineDescription& taeSpec,
                                      const std::unordered_map<std::string, std::string>& delegateEngines,
                                      const icu::UnicodeString& fileName);
};

#endif
