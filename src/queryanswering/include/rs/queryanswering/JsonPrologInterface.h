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

  typedef std::map< std::string,
                    std::pair< std::unordered_set<std::string>,
                               std::unordered_set<std::string> > > AnnotatorDependencies;

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

  bool retrieveAnnotatorsInputOutput(std::vector<std::string> &annotators,
                                     AnnotatorDependencies &dependencies);

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
   * in: vector containing annotator names
   * return true on succes:
   * */
  bool assertAnnotators(std::vector<std::string> annotatorNames);


  bool expandToFullUri(std::string &entry);

  /* brief: parse the annotator xmls and assert ceratin parts of it to the knowledgebase
   * in: annotator name
   * returns: true for succes
   * */
  bool assertAnnotatorMetaInfo(std::string , std::string);

  bool lookupAnnotatorDomain(std::string annotatorName, std::vector<std::string> &domain);

  std::string buildPrologQueryFromKeys(const std::vector<std::string> &keys);

  /*brief
   * Create a vector of Annotator Names from the result of the knowrob_rs library.
   * This vector can be used as input for RSControledAnalysisEngine::setNextPipelineOrder
   */
  std::vector<std::string> createPipelineFromPrologResult(std::string result);

};

#endif //WITH_JSON_PROLOG
#endif //JSONPROLOGINTERFACE_H
