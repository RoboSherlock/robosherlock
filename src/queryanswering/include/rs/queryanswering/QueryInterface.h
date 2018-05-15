
#ifdef WITH_JSON_PROLOG
//json
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>

#include <vector>
#include <rs/utils/output.h>

#include <rs/flowcontrol/RSControledAnalysisEngine.h>

#include <rs/queryanswering/JsonPrologInterface.h>

#include <std_srvs/Trigger.h>


class QueryInterface
{
private:

    JsonPrologInterface* jsonPrologInterface;

    bool handleDetect(std::vector<std::string> &newPipelineOrder);
    bool handleInspect(std::vector<std::string> &newPipelineOrder);
    bool handleScan(std::vector<std::string> &newPipelineOrder);

public:

  enum class QueryType {NONE, INSPECT, DETECT, SCAN};

  rapidjson::Document query;

  QueryInterface(){
      query = rapidjson::Document(rapidjson::kObjectType);
      jsonPrologInterface = new JsonPrologInterface();
  }
  ~QueryInterface()
  {
   delete jsonPrologInterface;
  }

  bool parseQuery(std::string query);

  QueryType processQuery(std::vector<std::string> &newPipelineOrder);

  void filterResults(std::vector<std::string> &resultDesignators,
                     std::vector<std::string> &filteredResponse,
                     std::vector<bool> &designatorsToKeep);

  bool checkSubClass(const std::string &resultValue, const std::string &queryValue);
  bool checkThresholdOnList(rapidjson::Value &list, const float threshold, std::string requestedKey, bool keepLower);

};
#endif //WITH_JSON_PROLOG
