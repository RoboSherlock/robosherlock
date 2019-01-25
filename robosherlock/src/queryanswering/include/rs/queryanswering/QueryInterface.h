#ifndef __QUERY_INTERFACE_H__
#define __QUERY_INTERFACE_H__


#ifdef WITH_JSON_PROLOG
//json
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>

#include <vector>
#include <rs/utils/output.h>

#include <rs/utils/exception.h>

#include <rs/queryanswering/JsonPrologInterface.h>

#include <std_srvs/Trigger.h>


class QueryInterface
{
private:

  JsonPrologInterface *jsonPrologInterface;

    bool handleDetect(std::vector<std::string> &newPipelineOrder,const rapidjson::Value &rapidJsonVale);
    bool handleInspect(std::vector<std::vector<std::string>> &newPipelineOrder);
    bool handleScan(std::vector<std::vector<std::string>> &newPipelineOrder);
    bool handleTrack(std::vector<std::string> &newPipelineOrder,const rapidjson::Value &rapidJsonVale);

  struct QueryTermProperties {
    std::string key;
    std::vector<std::string> location;
    std::vector<std::string> check;
  };

  typedef std::map <std::string, QueryTermProperties> QueryTermDefinitions;
  QueryTermDefinitions queryTermDefs_;

public:

  enum class QueryType {NONE, INSPECT, DETECT, SCAN, TRACK};

  rapidjson::Document query;

  QueryInterface()
  {
    query = rapidjson::Document(rapidjson::kObjectType);
    jsonPrologInterface = new JsonPrologInterface();
    getQueryConfig();
  }

  ~QueryInterface()
  {
    delete jsonPrologInterface;
  }
  bool getQueryConfig();

  bool parseQuery(std::string query);

  QueryType processQuery(std::vector<std::vector<std::string>> &newPipelineOrder);

  void filterResults(std::vector<std::string> &resultDesignators,
                     std::vector<std::string> &filteredResponse,
                     std::vector<bool> &designatorsToKeep);

  bool checkSubClass(const std::string &resultValue, const std::string &queryValue);
  bool checkThresholdOnList(rapidjson::Value &list, const float threshold, std::string requestedKey, bool keepLower);

};

#endif // WITH_JSON_PROLOG

#endif // __QUERYINTERFACE_H__
