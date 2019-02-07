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
#include <rs/utils/common.h>

//json_prolog interface
#include <json_prolog/prolog.h>

//STD
#include <memory>
#include <map>
#include <vector>
#include <utility>
#include <unordered_set>
#include <algorithm>
#include <mutex>

//json
#include <rapidjson/document.h>

#include <rs/queryanswering/KnowledgeEngine.h>

class JsonPrologInterface: public rs::KnowledgeEngine
{

  std::vector<std::string> krNamespaces_;
  json_prolog::Prolog pl_;
  std::mutex lock_;

public:

  JsonPrologInterface();
  ~JsonPrologInterface(){};

  /**
   * @brief planPipelineQuery
   * @param keys vector of keys extracted from query
   * @param pipeline vector of annotator names forming the pipeline
   * @return true on success
   */
  bool planPipelineQuery(const std::vector<std::string> &keys,
                         std::vector<std::string> &pipeline);

  /**
   * @brief q_subClassOf
   * @param child name of child
   * @param parent name of parent
   * @return
   */
  bool q_subClassOf(std::string child, std::string parent);

  /**
   * @brief checkValidQueryTerm verify if term of query language has been defined in KB
   * @param term term to verify
   * @return true if defined
   */
  bool checkValidQueryTerm(const std::string &term);

  /**
   * @brief assertValueForKey assert a key value pair to the knowledge base
   * @param key
   * @param value
   * @return true on success
   */
  bool assertValueForKey(const  std::string &key, const std::string &value);

  bool addNamespace(const std::string &entry, std::string &results);

  bool addNamespace(std::string &entry);

  /*brief
   * check for a class property
   * */
  bool q_classProperty(std::string className, std::string property, std::string value);

  /*
   * assert terms of the query language and types that correspond to these terms
   * in: map of keyword to types in the typesystem corresponding to the keys;
   * out true on success
   * */
  bool assertQueryLanguage(std::map<std::string, std::vector<std::string>> &query_terms);

  bool retractQueryLanguage();

  /*brief
   * create individuals for the anntators in the list
   * in: map containing annotator names and capability information
   * return true on succes:
   * */
  bool assertAnnotators(const std::map<std::string, rs::AnnotatorCapabilities> &annotCap);

  bool retractAllAnnotators();

  bool retractQueryKvPs();

  bool expandToFullUri(std::string &entry);

  /* brief:assert capabilities of an annotator of to the knowledge base
   * in: annotator capabilities (I/O types and restrictions on them)
   * returns: true for succes
   * */
  bool assertAnnotatorMetaInfo(std::pair<std::string, rs::AnnotatorCapabilities> , std::string);

  std::string buildPrologQueryFromKeys(const std::vector<std::string> &keys);

  /*brief
   * Create a vector of Annotator Names from the result of the knowrob_rs library.
   * This vector can be used as input for RSAnalysisEngine::setNextPipelineOrder
   */
  std::vector<std::string> createPipelineFromPrologResult(std::string result);

  json_prolog::PrologQueryProxy queryWithLock(const std::string &query);

};

#endif //WITH_JSON_PROLOG
#endif //JSONPROLOGINTERFACE_H
