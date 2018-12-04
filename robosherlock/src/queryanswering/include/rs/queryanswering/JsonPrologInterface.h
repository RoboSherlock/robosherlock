#ifndef JSONPROLOGINTERFACE_H
#define JSONPROLOGINTERFACE_H

#ifdef WITH_JSON_PROLOG
//boost
#include <boost/algorithm/string.hpp>

//xerces
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/util/PlatformUtils.hpp>

//YAML parsing
#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/mark.h>
#include <yaml-cpp/yaml.h>

//ros
#include <ros/package.h>

//robosherlock
#include <rs/utils/output.h>
#include <rs/queryanswering/KRDefinitions.h>
#include <rs/utils/common.h>
#include <rs/queryanswering/query_rules.h>


//json_prolog interface
#include <json_prolog/prolog.h>

//STD
#include <memory>
#include <map>
#include <vector>
#include <utility>
#include <unordered_set>

//json
#include <rapidjson/document.h>

//wrapper class for Prolog Engine based on SWI-C++
class JsonPrologInterface
{
  std::vector<std::string> krNamespaces;

public:

  JsonPrologInterface();
  ~JsonPrologInterface()
  {
       xercesc::XMLPlatformUtils::Terminate();
  }

  /*
   * in: vector of keys extracted from query
   * out: vector of annotator names forming the pipeline
   */
  bool planPipelineQuery(const std::vector<std::string> &keys,
                         std::vector<std::string> &pipeline);

  /*brief
   * ask prolog if child is of type parent
   * */
  bool q_subClassOf(std::string child, std::string parent);

  bool addNamespace(const std::string &entry, std::string &results);

  bool addNamespace(std::string &entry);

  /*brief
   * check for a class property
   * */
  bool q_classProperty(std::string className, std::string property, std::string value);

  /*brief
   *extract the keys that serve as input for pipeline planning
   * in query as a designator
   * out vector of keys
   * */
  bool extractQueryKeysFromDesignator(std::string *desig,
                                      std::vector<std::string> &keys);

  /*brief
   * in desig: query as designator
   * out prologQuery: the Prolog Query as a string
   * returns true or false /success or fail
   * */
  bool buildPrologQueryFromDesignator(std::string *desig,
                                      std::string &prologQuery);


  bool retractAllAnnotators();

  /*brief
   * create individuals for the anntators in the list
   * in: map containing annotator names and capability information
   * return true on succes:
   * */
  bool assertAnnotators(const std::map<std::string,rs::AnnotatorCapabilities> &annotCap);


  bool expandToFullUri(std::string &entry);

  /* brief:assert capabilities of an annotator of to the knowledge base
   * in: annotator capabilities (I/O types and restrictions on them)
   * returns: true for succes
   * */
  bool assertAnnotatorMetaInfo(std::pair<std::string,rs::AnnotatorCapabilities> , std::string);

  std::string buildPrologQueryFromKeys(const std::vector<std::string> &keys);

  /*brief
   * Create a vector of Annotator Names from the result of the knowrob_rs library.
   * This vector can be used as input for RSAnalysisEngine::setNextPipelineOrder
   */
  std::vector<std::string> createPipelineFromPrologResult(std::string result);

};

#endif //WITH_JSON_PROLOG
#endif //JSONPROLOGINTERFACE_H
