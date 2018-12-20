
#include<rs/queryanswering/QueryInterface.h>

#ifdef WITH_JSON_PROLOG
#include<rs/queryanswering/DesignatorWrapper.h>

// Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

//RapidJson
#include "rapidjson/pointer.h"
#include "rapidjson/writer.h"

bool QueryInterface::parseQuery(std::string query)
{
    /**
    rapidjson::Document d;
    d.Parse(query);
    if(d.HasMember("track")){
        rapidjson::Value o(rapidjson::kObjectType);
        {
            rapidjson::Value contacts(rapidjson::kArrayType);
            // adding elements to contacts array.
            o.AddMember("contacts", contacts, d.GetAllocator());  // deep clone contacts (may be with lots of allocations)
            // destruct contacts.
        }
    }
     **/

    this->query.Parse(query);
  return true;
}


/**
 * @brief QueryInterface::processQuery
 * @param[out] res vector of planed pipelines;
 * @return
 */
QueryInterface::QueryType QueryInterface::processQuery(std::vector<std::vector<std::string>> &res)
{
  if(query.HasMember("detect"))
  {
    const rapidjson::Value &val = query["detect"];
    std::vector<std::string> detPipeline;
    handleDetect(detPipeline, val);
    res.push_back(detPipeline);
    return QueryType::DETECT;
  }

  else if(query.HasMember("inspect"))
  {
    handleInspect(res);
    return QueryType::INSPECT;
  }

  else if(query.HasMember("scan"))
  {
    handleScan(res);
    return QueryType::SCAN;
  }

  else if(query.HasMember("track")) {
      //create json string for detection

      // Makes detect query from track query.
      if (query.HasMember("command"))
      {
          rapidjson::Value::MemberIterator commandIterator = query.FindMember("command");
          if(commandIterator->value.GetString() == "stop"){
              // TODO: Do whatever is necessary to stop tracking
          }

      } else {

          const rapidjson::Value &valTrack = query["track"];

          // is it really needed to literally replace the "detect" with a "track"?
          // Where in the code is the first member (detect/track/...) relevant again after the else if above?
          if (query.HasMember("track")) {
              rapidjson::Value::MemberIterator trackIterator = query.FindMember("track");
              trackIterator->value.SetString("detect", query.GetAllocator());
          }

          rapidjson::StringBuffer sb;
          rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
          query.Accept(writer);
          std::string queryDetect(sb.GetString(), sb.GetSize());

          // What was this for again? Why does this pass just the uppermost rapidjon::Value to the handle-functions?
          // Now that query has been modified, use it as a detect query.
          const rapidjson::Value &valDetect = query["detect"];

          std::vector<std::string> detPipeline, trackingPipeline;
          handleDetect(detPipeline, valDetect);
          handleTrack(trackingPipeline, valTrack);
          res.push_back(detPipeline);
          res.push_back(trackingPipeline);
          return QueryType::TRACK;
      }
  }

  return QueryType::NONE;
}


bool QueryInterface::handleScan(std::vector<std::vector<std::string>> &res)
{
  //TODO implement logic here
  return true;
}

bool QueryInterface::handleInspect(std::vector<std::vector<std::string>> &res)
{
  outInfo("Inspecting an object: [needs implementation]");
  return true;
}

bool QueryInterface::handleDetect(std::vector<std::string> &res, const rapidjson::Value &rapidJsonValue)
{

  const rapidjson::Value &val = rapidJsonValue;
  rapidjson::StringBuffer strBuff;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(strBuff);
  val.Accept(writer);

  std::string req = strBuff.GetString();

  std::vector<std::string> keys;
  std::vector<std::string> new_pipeline_order;
  jsonPrologInterface->extractQueryKeysFromDesignator(&req, keys);
  try
  {
    jsonPrologInterface->planPipelineQuery(keys, new_pipeline_order);
  }
  catch(std::exception e)
  {
    outError("calling json_prolog was not successful. Is the node running?");
    return false;
  }

  if(new_pipeline_order.empty())
  {
    outInfo("Can't find solution for pipeline planning");
    return false; // Indicate failure
  }
  outInfo("New Pipeline: ");
  std::for_each(new_pipeline_order.begin(), new_pipeline_order.end(), [](std::string & p)
  {
    outInfo(p);
  });

  //always estimate a pose...these could go directly into the planning phase in Prolog?
  if(std::find(new_pipeline_order.begin(), new_pipeline_order.end(), "Cluster3DGeometryAnnotator") == new_pipeline_order.end())
  {
    std::vector<std::string>::iterator it = std::find(new_pipeline_order.begin(), new_pipeline_order.end(), "ClusterMerger");
    if(it != new_pipeline_order.end())
    {
      new_pipeline_order.insert(it + 1, "Cluster3DGeometryAnnotator");
    }
  }

  //  for debugging advertise TF
  //  new_pipeline_order.push_back("TFBroadcaster");

  //whatever happens do ID res and spawn to gazebo...this is also pretty weird
  if(std::find(new_pipeline_order.begin(), new_pipeline_order.end(), "ObjectIdentityResolution") == new_pipeline_order.end())
  {
    new_pipeline_order.push_back("ObjectIdentityResolution");
    new_pipeline_order.push_back("BeliefToKnowRob");
  }
  new_pipeline_order.push_back("StorageWriter");
  res.insert(res.end(), new_pipeline_order.begin(), new_pipeline_order.end());

  return true;

}

bool QueryInterface::handleTrack(std::vector<std::string> &res, const rapidjson::Value &rapidJsonValue)
{
  res.push_back("CollectionReader");
  res.push_back("ImagePreprocessor");
  //res.push_back("PointCloudClusterExtractor"); // 3D tracking
  //res.push_back("PointCloudFilter"); // 3D tracking
  res.push_back("KalmanTrackingAnnotator");
  //res.push_back("PCLTrackingAnnotator"); // 3D tracking
  res.push_back("StorageWriter");
  outInfo("Planned tracking pipeline: ");
  for(auto const &r:res){
    outInfo(r);
  }
  return true;
}

bool getConfigForKey(std::string key, std::vector<std::string> &location,
                     std::vector<std::string> &check,
                     double &thresh,
                     bool &keepLower)
{
  const std::string &configFile = ros::package::getPath("robosherlock") + "/config/filter_config.ini";

  outInfo("Path to config file: " FG_BLUE << configFile);
  boost::property_tree::ptree pt;
  try
  {
    boost::property_tree::ini_parser::read_ini(configFile, pt);
    if(pt.find(key) == pt.not_found())
      return false;

    std::string l = pt.get<std::string>(key + ".location", "/" + key);
    std::string c = pt.get<std::string>(key + ".check", "EQUAL");
    thresh = pt.get<double> (key + ".threshold", 0.f);
    keepLower = pt.get<bool> (key + ".keepLower", true);

    boost::split(location, l, boost::is_any_of(","), boost::token_compress_on);
    boost::split(check, c, boost::is_any_of(","), boost::token_compress_on);

    return true;
  }
  catch(boost::property_tree::ini_parser::ini_parser_error &e)
  {
    throw_exception_message("Error opening config file: " + configFile);
    return false;
  }
  return false;
}

bool QueryInterface::checkSubClass(const std::string &resultValue, const std::string &queryValue)
{
  bool ok = false;
  try
  {
    ok = jsonPrologInterface->q_subClassOf(resultValue, queryValue);
  }
  catch(std::exception &e)
  {
    outError("Prolog Exception: Malformed owl_subclass_of. Child or superclass undefined:");
    outError("     Child: " << resultValue);
    outError("     Parent: " << queryValue);
  }
  return ok;
}

bool QueryInterface::checkThresholdOnList(rapidjson::Value &list, const float threshold, std::string requestedKey, bool keepLower)
{
  for(rapidjson::Value::ConstMemberIterator listIt = list.MemberBegin(); listIt != list.MemberEnd(); ++listIt)
  {
    if(listIt->name == requestedKey)
    {
      if(!keepLower)
      {
        if(listIt->value.GetDouble() >= threshold)
          return true;
      }
      else if(listIt->value.GetDouble() < threshold)
        return true;

    }
  }
  return false;
}

void QueryInterface::filterResults(std::vector<std::string> &resultDesignators,
                                   std::vector<std::string> &filteredResponse,
                                   std::vector<bool> &designatorsToKeep)
{

  const rapidjson::Value &detectQuery = query["detect"];
  designatorsToKeep.resize(resultDesignators.size(), true);

  for(rapidjson::Value::ConstMemberIterator queryIt = detectQuery.MemberBegin(); queryIt != detectQuery.MemberEnd(); ++queryIt)
  {
    std::vector<std::string> location, check;
    std::string key = queryIt->name.GetString();
    double thresh;
    bool keepLower;
    if(!getConfigForKey(key, location, check, thresh, keepLower)) continue;
    const std::string queryValue = queryIt->value.GetString();

    if (queryValue == "") continue;

    outInfo("No. of resulting Object Designators: " << resultDesignators.size());
    for(size_t i = 0; i < resultDesignators.size(); ++i)
    {
      rapidjson::Document resultJson;
      resultJson.Parse(resultDesignators[i].c_str());

      std::vector<bool> matchingDescription(location.size(), true);
      for(size_t j = 0; j < location.size(); ++j)
      {
        //check if this query key exists in the result
        if(rapidjson::Value *value = rapidjson::Pointer(location[j]).Get(resultJson))
        {
          if(check[j] == "EQUAL")
          {
            if(value->GetType() == rapidjson::Type::kStringType)
            {
              std::string resultValue = value->GetString();;
              if(resultValue != queryValue && queryValue != "")
                matchingDescription[j] = false;
            }
            else
              matchingDescription[j] = false;
          }
          else if(check[j] == "CLASS")
          {
            const std::string resultValue = value->GetString();
            if(!checkSubClass(resultValue, queryValue))
              matchingDescription[j] = false;
          }
          else if(check[j] == "GEQ")
          {
            float volumeofCurrentObj = value->GetDouble();
            float volumeAsked = atof(queryValue.c_str());
            outWarn("Volume asked as float: " << volumeAsked);
            if(volumeAsked > volumeofCurrentObj)
            {
              matchingDescription[j] = false;
            }
          }
          else if(check[j] == "THRESHLIST")
          {
            if(!checkThresholdOnList(*value, thresh, queryValue, keepLower) && queryValue != "")
            {
              matchingDescription[j] = false;
            }
          }
          else if(check[j] == "CONTAINS")
          {
            bool found = false;
            for(auto &v : value->GetArray())
              if(v == queryValue || queryValue == "")
                found = true;
            if(!found)
              matchingDescription[j] = false;
          }
          else
          {
            outWarn("There is no such check: " + check[j] + ". Please check the filter_config.ini");
            matchingDescription[j] = false;
          }
        }
        else if(check[j] == "CONTAINSEQUAL")
        {
          std::string delimiter = "*";
          int delLoc = location[j].find(delimiter);
          std::string prefix = location[j].substr(0, delLoc - 1);
          std::string suffix = location[j].substr(delLoc + 1, location[j].size());
          if(rapidjson::Value *suffixVal = rapidjson::Pointer(prefix).Get(resultJson))
          {
            for(int i = 0; i < suffixVal->Size(); i ++)
            {
              std::string newLocation = prefix + " / " + std::to_string(i) + suffix;
              if(rapidjson::Value *value = rapidjson::Pointer(newLocation).Get(resultJson))
              {
                std::string resultValue = value->GetString();;
                if(resultValue != queryValue)
                  matchingDescription[j] = false;
              }
            }
          }
        }
        else
          matchingDescription[j] = false;
      }
      bool what =false;
      for(auto m : matchingDescription)
      {
         what =m| what;
      }

      designatorsToKeep[i] = what & designatorsToKeep[i];
    }
  }
  for(int i = 0; i < designatorsToKeep.size(); ++i)
  {
    if(designatorsToKeep[i])
    {
      filteredResponse.push_back(resultDesignators[i]);
    }
  }
  outInfo("Matching Object Descriptions: " << filteredResponse.size());
}

#endif //WITH_JSON_PROLOG
