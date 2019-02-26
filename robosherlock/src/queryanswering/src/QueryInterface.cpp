#include<rs/queryanswering/QueryInterface.h>

bool QueryInterface::parseQuery(std::string query)
{
  outInfo("parsing query:" << query);

  rapidjson::Document doc;
  doc.Parse(query);
  if(doc.IsObject())
  {
    query_.Swap(doc);
    return true;
  }
  else
    return false;
  }

QueryInterface::QueryType QueryInterface::processQuery(std::vector<std::string> &res)
{
  if(query_.HasMember("detect"))
  {
    handleDetect(res);
    return QueryType::DETECT;
  }
  else if(query_.HasMember("inspect"))
  {
    handleInspect(res);
    return QueryType::INSPECT;
  }
  else if(query_.HasMember("scan"))
  {
    handleScan(res);
    return QueryType::SCAN;
  }
  return QueryType::NONE;
}


bool QueryInterface::handleScan(std::vector<std::string> &res)
{
  //TODO implement logic here
  return true;
}

bool QueryInterface::handleInspect(std::vector<std::string> &res)
{
  outInfo("Inspecting an object: [needs implementation]");
  return true;
}

bool QueryInterface::extractQueryKeysFromDesignator(rapidjson::Value &json,
    std::vector<std::string> &keys)
{
  knowledgeEngine_->retractQueryKvPs();

  //add the ones that are interpretable to the queriedKeys;
  for(rapidjson::Value::ConstMemberIterator iter = json.MemberBegin(); iter != json.MemberEnd(); ++iter)
  {
    if(knowledgeEngine_->checkValidQueryTerm(iter->name.GetString()))
    {
      keys.push_back(iter->name.GetString());
      //for a select member of keys (type, class, shape, color) let's add value reasoning; TODO: get rid of this somehow;
      std::vector<std::string> special_keys = {"type", "class", "shape", "color", "size"};
      if(std::find(special_keys.begin(), special_keys.end(),
                   iter->name.GetString()) != std::end(special_keys))
      {
        std::string d = iter->value.GetString();
        d[0] = std::toupper(d[0]);
        if(d != "" && !knowledgeEngine_->addNamespace(d))
        {
          outWarn("No OWL definitions for " << d << " under any of the known namespaces");
          continue;
        }
        if(!knowledgeEngine_->assertValueForKey(iter->name.GetString(), d))
          return false;
      }
    }
    else
      outWarn(iter->name.GetString() << " is not a valid query-language term");
  }
  return true;
}


bool QueryInterface::handleDetect(std::vector<std::string> &res)
{
  rapidjson::Value &val = query_["detect"];
  std::vector<std::string> keys;
  std::vector<std::string> new_pipeline_order;
  extractQueryKeysFromDesignator(val, keys);
  try
  {
    knowledgeEngine_->planPipelineQuery(keys, new_pipeline_order);
  }
  catch(std::exception e)
  {
    outError("calling json_prolog was not successfull. Is the node running?");
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
      new_pipeline_order.insert(it + 1, "Cluster3DGeometryAnnotator");
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


bool QueryInterface::getQueryConfig()
{
  std::vector<std::string> searchPaths;
  searchPaths.push_back(ros::package::getPath("robosherlock") + "/config");
  std::vector<std::string> child_packages;
  ros::package::command("depends-on robosherlock", child_packages);

  for(size_t i = 0; i < child_packages.size(); ++i)
    searchPaths.push_back(ros::package::getPath(child_packages[i]) + "/config");

  std::vector<std::string> configPaths;
  for(auto p : searchPaths)
  {
    boost::filesystem::path filePath(p + "/query_specifications.ini");
    if(boost::filesystem::exists(filePath))
      configPaths.push_back(filePath.string());

  }

  knowledgeEngine_->retractQueryLanguage();

  std::map <std::string, std::vector<std::string>> queryLangSpecs;
  for(auto p : configPaths)
  {
    outInfo("Path to config file: " FG_BLUE << p);
    boost::property_tree::ptree pt;
    try
    {
      boost::property_tree::ini_parser::read_ini(p, pt);

      for(auto property : pt)
      {
        if(property.first == "keys")
        {
          for(auto entry : property.second)
          {
            std::vector<std::string> types;
            std::string typesEntry = pt.get<std::string>(property.first + "." + entry.first);
            boost::algorithm::split(types, typesEntry, boost::is_any_of(" ,"), boost::token_compress_on);
            queryLangSpecs[entry.first].insert(queryLangSpecs[entry.first].end(), types.begin(), types.end());
          }
        }
        else
        {
          std::shared_ptr<QueryTermProperties> qProp;

          if(queryTermDefs_.find(property.first) != queryTermDefs_.end())
            qProp = std::make_shared<QueryTermProperties>(queryTermDefs_[property.first]);
          else
            qProp = std::make_shared<QueryTermProperties>();

          qProp->key = property.first;
          std::string l = pt.get<std::string>(property.first + ".location");
          //          std::string c = pt.get<std::string>(property.first + ".check");
          std::vector<std::string> locations;
          boost::split(locations, l, boost::is_any_of("|& "), boost::token_compress_on);
          std::vector<std::string> checks(locations.size(), "ARRAY-VAL-CHECK");
          //          boost::split(checks, c, boost::is_any_of(",|& "), boost::token_compress_on);
          qProp->location.insert(qProp->location.end(), locations.begin(), locations.end());
          qProp->check.insert(qProp->check.end(), checks.begin(), checks.end());
          queryTermDefs_[property.first] = *qProp;
        }
      }
    }
    catch(boost::property_tree::ini_parser::ini_parser_error &e)
    {
      outError("Error opening config file: " << p);
      return false;
    }
  }

  knowledgeEngine_->assertQueryLanguage(queryLangSpecs);
  return false;
}


bool QueryInterface::checkSubClass(const std::string &resultValue, const std::string &queryValue)
{
  bool ok = false;
  try
  {
    ok = knowledgeEngine_->q_subClassOf(resultValue, queryValue);
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
        if(listIt->value.GetDouble() >= static_cast<double>(threshold))
          return true;
      }
      else if(listIt->value.GetDouble() < static_cast<double>(threshold))
        return true;
    }
  }
  return false;
}


void QueryInterface::filterResults(std::vector<std::string> &resultDesignators,
    std::vector<std::string> &filteredResponse,
    std::vector<bool> &designatorsToKeep)
{

  const rapidjson::Value &detectQuery = query_["detect"];
  designatorsToKeep.resize(resultDesignators.size(), true);

  for(rapidjson::Value::ConstMemberIterator queryIt = detectQuery.MemberBegin(); queryIt != detectQuery.MemberEnd(); ++queryIt)
  {
    std::vector<std::string> location, check;
    std::string key = queryIt->name.GetString();
    double thresh;
    bool keepLower;
    if(queryTermDefs_.find(key) == queryTermDefs_.end()) continue;
    const std::string queryValue = queryIt->value.GetString();

    if(queryValue == "") continue;
    check = queryTermDefs_[key].check;
    location = queryTermDefs_[key].location;

    outInfo("No. of resulting Object Designators: " << resultDesignators.size());
    for(size_t i = 0; i < resultDesignators.size(); ++i)
    {
      rapidjson::Document resultJson;
      resultJson.Parse(resultDesignators[i].c_str());

      std::vector<bool> matchingDescription(location.size(), true);
      for(size_t j = 0; j < location.size(); ++j)
      {
        //check if this query key exists in the result
        rapidjson::Pointer p(location[j]);

        if(!p.IsValid())
          outError("Location: [" << location[j] << "] is not a valid rapidjson location. Check the documentation of rapidjson;");
        rapidjson::Value *value = rapidjson::GetValueByPointer(resultJson, p);
        if(value != nullptr)
        {
          if(check[j] == "EQUAL")
          {
            if(value->GetType() == rapidjson::Type::kStringType)
            {
              std::string resultValue = value->GetString();;
              if((resultValue != queryValue && queryValue != "") || !checkSubClass(resultValue, queryValue))
                matchingDescription[j] = false;
            }
            else
              matchingDescription[j] = false;
          }
          else if(check[j] == "GEQ")
          {
            double volumeofCurrentObj = value->GetDouble();
            double volumeAsked = atof(queryValue.c_str());
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
        else if(check[j] == "ARRAY-VAL-CHECK")
        {
          std::string delimiter = "*";
          size_t delLoc = location[j].find(delimiter);
          std::string prefix = location[j].substr(0, delLoc - 1);
          std::string suffix = location[j].substr(delLoc + 1, location[j].size());
          bool matched = false;
          if(rapidjson::Value *suffixVal = rapidjson::Pointer(prefix).Get(resultJson))
          {
            for(uint8_t i = 0; i < suffixVal->Size(); i ++)
            {
              std::string newLocation = prefix + "/" + std::to_string(i) + suffix;
              if(rapidjson::Value *value = rapidjson::Pointer(newLocation).Get(resultJson))
              {
                std::string resultValue = value->GetString();;
                if(resultValue == queryValue || checkSubClass(resultValue, queryValue))
                  matched = true;
              }
            }
          }
          matchingDescription[j] = matched;
        }
        else
          matchingDescription[j] = false;
      }
      bool what = false;
      for(auto m : matchingDescription)
      {
        what = m | what;
      }

      designatorsToKeep[i] = what & designatorsToKeep[i];
    }
  }
  for(size_t i = 0; i < designatorsToKeep.size(); ++i)
  {
    if(designatorsToKeep[i])
    {
      filteredResponse.push_back(resultDesignators[i]);
    }
  }
  outInfo("Matching Object Descriptions: " << filteredResponse.size());
}
