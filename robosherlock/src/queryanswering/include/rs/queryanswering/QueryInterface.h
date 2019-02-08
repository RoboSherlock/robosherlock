#ifndef __QUERY_INTERFACE_H__
#define __QUERY_INTERFACE_H__


#ifdef WITH_JSON_PROLOG

//json
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/pointer.h>

#include <vector>

#include <rs/utils/output.h>
#include <rs/utils/exception.h>
#include <rs/queryanswering/JsonPrologInterface.h>

#undef PL_A1 //without this there is a conflicting declaration between SWI-cpp and Eigen;
//TODO can this undef have unwanted consequences?
#include <rs/queryanswering/ObjectDesignatorFactory.h>
#include <rs/queryanswering/SWIPLInterface.h>


// Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

//RapidJson


class QueryInterface
{
private:

  std::shared_ptr<rs::KnowledgeEngine> knowledgeEngine_;

  struct QueryTermProperties {
    std::string key;
    std::vector<std::string> location;
    std::vector<std::string> check;
  };

  typedef std::map <std::string, QueryTermProperties> QueryTermDefinitions;
  QueryTermDefinitions queryTermDefs_;

  rapidjson::Document query_;

public:

  enum class QueryType {NONE, INSPECT, DETECT, SCAN};

  QueryInterface(std::shared_ptr<rs::KnowledgeEngine> ke)
  {
    knowledgeEngine_ = ke;
    query_ = rapidjson::Document(rapidjson::kObjectType);
//    knowledgeEngine_ = std::make_shared<rs::SWIPLInterface>();
    getQueryConfig();
  }

  ~QueryInterface()
  {
  }

  bool handleDetect(std::vector<std::string> &newPipelineOrder);

  bool handleInspect(std::vector<std::string> &newPipelineOrder);

  bool handleScan(std::vector<std::string> &newPipelineOrder);

  bool getQueryConfig();

  bool parseQuery(std::string query_);

  QueryType processQuery(std::vector<std::string> &newPipelineOrder);

  void filterResults(std::vector<std::string> &resultDesignators, std::vector<std::string> &filteredResponse, std::vector<bool> &designatorsToKeep);

  bool checkSubClass(const std::string &resultValue, const std::string &queryValue);

  bool checkThresholdOnList(rapidjson::Value &list, const float threshold, std::string requestedKey, bool keepLower);

  bool extractQueryKeysFromDesignator(rapidjson::Value &desig, std::vector<std::string> &keys);

};

#endif // WITH_JSON_PROLOG

#endif // __QUERYINTERFACE_H__
