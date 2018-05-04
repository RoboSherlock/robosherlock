#include <rs/queryanswering/PrologInterface.h>


PrologInterface::PrologInterface(bool json_prolog)
{
  useJsonProlog = json_prolog;
  if(!this->useJsonProlog)
  {
    outInfo("Opening own Prolog Engine");
    char *argv[4];
    int argc = 0;
    argv[argc++] = "PrologEngine";
    argv[argc++] = "-f";
    std::string rosPrologInit = ros::package::getPath("rosprolog") + "/prolog/init.pl";
    argv[argc] = new char[rosPrologInit.size() + 1];
    std::copy(rosPrologInit.begin(), rosPrologInit.end(), argv[argc]);
    argv[argc++][rosPrologInit.size()] = '\0';
    argv[argc] = NULL;
    engine  = std::make_shared<PlEngine>(argc, argv);
    init();
  }
  else
  {
    outInfo("Creating ROS Service client for json_prolog");
  }
}

void PrologInterface::init()
{
  outInfo("Initializing Prolog Engine");
  PlTerm av("rs_prolog_interface");
  try
  {
    PlCall("register_ros_package", av);
  }
  catch(PlException &ex)
  {
    outInfo((char *)ex);
  }
}

bool PrologInterface::extractQueryKeysFromDesignator(std::string *desig,
                                    std::vector<std::string> &keys)
{
  if(!desig)
  {
    outError("NULL POINTER PASSED TO buildPrologQueryFromDesignator");
    return false;
  }

  rapidjson::Document json;
  json.Parse(desig->c_str());

  //add the ones that are interpretable to the queriedKeys;
  for (rapidjson::Value::ConstMemberIterator iter = json.MemberBegin(); iter != json.MemberEnd(); ++iter)
  {
    if(std::find(rs_queryanswering::rsQueryTerms.begin(), rs_queryanswering::rsQueryTerms.end(),
                 iter->name.GetString()) != std::end(rs_queryanswering::rsQueryTerms))
      keys.push_back(iter->name.GetString());
    else
      outWarn(iter->name.GetString() << " is not a valid query-language term");
  }
  if(json.HasMember("type"))
  {
    std::string det = json["type"].GetString();
    if(det == "PANCAKE" || det == "pancake")
    {
      keys.push_back("pancakedetector");
    }
  }
  if(json.HasMember("obj-part"))
  {
    std::string det = json["obj-part"].GetString();
    if(det.find(det)!=std::string::npos)
    {
      keys.clear();
      keys.push_back("handle");
    }
  }
  if(json.HasMember("class"))
  {
    std::string det = json["class"].GetString();
    if(det == "FoodOrDrinkOrIngredient")
    {
      keys.push_back("pancakedetector");
    }
  }
  return true;
}


bool PrologInterface::buildPrologQueryFromDesignator(std::string *desig,
    std::string &prologQuery)
{
    prologQuery = "build_single_pipeline_from_predicates([";
    std::vector<std::string> queriedKeys;
    extractQueryKeysFromDesignator(desig,queriedKeys);
    for(int i = 0; i < queriedKeys.size(); i++)
    {
      prologQuery += queriedKeys.at(i);
      if(i < queriedKeys.size() - 1)
      {
        prologQuery += ",";
      }
    }
    prologQuery += "], A)";
    return true;
}


std::string PrologInterface::buildPrologQueryFromKeys(const std::vector<std::string> &keys)
{
  std::string prologQuery = "build_single_pipeline_from_predicates([";
  for(int i = 0; i < keys.size(); i++)
  {
    prologQuery += keys.at(i);
    if(i < keys.size() - 1)
      prologQuery += ",";
  }
  prologQuery += "], A)";
  return prologQuery;
}

std::vector< std::string > PrologInterface::createPipelineFromPrologResult(std::string queryResult)
{
  std::vector<std::string> new_pipeline;

  // Strip the braces from the result
  queryResult.erase(queryResult.end() - 1);
  queryResult.erase(queryResult.begin());

  std::stringstream resultstream(queryResult);

  std::string token;
  while(std::getline(resultstream, token, ','))
  {
    // erase leading whitespaces
    token.erase(token.begin(), std::find_if(token.begin(), token.end(), std::bind1st(std::not_equal_to<char>(), ' ')));
    outDebug("Planned Annotator by Prolog Planner " << token);

    // From the extracted tokens, remove the prefix
    std::string prefix("http://knowrob.org/kb/rs_components.owl#");
    int prefix_length = prefix.length();

    // Erase by length, to avoid string comparison
    token.erase(0, prefix_length);
    // outInfo("Annotator name sanitized: " << token );

    new_pipeline.push_back(token);
  }
  return new_pipeline;
}
