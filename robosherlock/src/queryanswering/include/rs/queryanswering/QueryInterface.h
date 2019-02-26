#ifndef __QUERY_INTERFACE_H__
#define __QUERY_INTERFACE_H__

//rapidjson
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/pointer.h>

//STL
#include <vector>

//RS
#include <rs/utils/output.h>
#include <rs/utils/exception.h>
#include <rs/queryanswering/ObjectDesignatorFactory.h>
#include <rs/queryanswering/KnowledgeEngine.h>

// Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string.hpp>

// ROS
#include <ros/package.h>

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

#endif // __QUERYINTERFACE_H__
