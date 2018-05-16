#ifndef JSONPROLOGINTERFACE_H
#define JSONPROLOGINTERFACE_H

#ifdef WITH_JSON_PROLOG
//boost
#include <boost/algorithm/string.hpp>

//ros
#include <ros/package.h>

//robosherlock
#include <rs/utils/output.h>
#include <rs/queryanswering/KRDefinitions.h>


//json_prolog interface
#include <json_prolog/prolog.h>

//STD
#include <memory>

//json
#include <rapidjson/document.h>

//wrapper class for Prolog Engine based on SWI-C++
class JsonPrologInterface
{
#ifdef WITH_JSON_PROLOG

//  typedef std::shared_ptr<PlEngine> PlEnginePtr;
//  PlEnginePtr engine;
#endif

  std::vector<std::string> krNamespaces;

public:
  JsonPrologInterface();
  ~JsonPrologInterface()
  {
  }


  /*brief
   * initialize the necessary knowrob packages
   */
  void init();

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

  std::string buildPrologQueryFromKeys(const std::vector<std::string> &keys);

  /*brief
   * Create a vector of Annotator Names from the result of the knowrob_rs library.
   * This vector can be used as input for RSControledAnalysisEngine::setNextPipelineOrder
   */
  std::vector<std::string> createPipelineFromPrologResult(std::string result);

};

#endif //WITH_JSON_PROLOG
#endif //JSONPROLOGINTERFACE_H
