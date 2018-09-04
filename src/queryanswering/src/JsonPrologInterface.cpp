#include <rs/queryanswering/JsonPrologInterface.h>
#ifdef WITH_JSON_PROLOG

using namespace xercesc;

JsonPrologInterface::JsonPrologInterface()
{
  outInfo("Creating ROS Service client for json_prolog");
  try
  {
    XMLPlatformUtils::Initialize();
  }
  catch(const XMLException &toCatch)
  {
    outInfo("error starting an xml parser");
  }

}


bool JsonPrologInterface::extractQueryKeysFromDesignator(std::string *desig,
    std::vector<std::string> &keys)
{
  if(!desig)
  {
    outError("NULL POINTER PASSED TO buildPrologQueryFromDesignator");
    return false;
  }

  rapidjson::Document json;
  json.Parse(desig->c_str());

  json_prolog::Prolog pl;
  json_prolog::PrologQueryProxy bdgs = pl.query("retract(requestedValueForKey(_,_))");

  //add the ones that are interpretable to the queriedKeys;
  for(rapidjson::Value::ConstMemberIterator iter = json.MemberBegin(); iter != json.MemberEnd(); ++iter)
  {
    if(std::find(rs_queryanswering::rsQueryTerms.begin(), rs_queryanswering::rsQueryTerms.end(),
                 iter->name.GetString()) != std::end(rs_queryanswering::rsQueryTerms))
    {
      keys.push_back(iter->name.GetString());
      //for a select member of keys (type, class, shape, color) let's add value reasoning;
      std::vector<std::string> special_keys = {"type", "class", "shape", "color"};
      if(std::find(special_keys.begin(), special_keys.end(),
                   iter->name.GetString()) != std::end(special_keys))
      {
        json_prolog::Prolog pl;
        std::string d = iter->value.GetString();
        d[0] = std::toupper(d[0]);
        if(!addNamespace(d))
        {
          outWarn("No OWL definitions for " << d << " under any of the known namespaces");
          continue;
        }
        std::stringstream assertionQuery;
        assertionQuery << "assert(requestedValueForKey(" << iter->name.GetString() << "," << d << "))";
        outInfo("Calilng query: " << assertionQuery.str());
        json_prolog::PrologQueryProxy bdgs = pl.query(assertionQuery.str());
        if(bdgs.begin() != bdgs.end())
        {
          outInfo("Asserted: " << assertionQuery.str());
        }
      }
    }
    else
    {
      outWarn(iter->name.GetString() << " is not a valid query-language term");
    }
  }
  return true;
}


bool JsonPrologInterface::buildPrologQueryFromDesignator(std::string *desig,
    std::string &prologQuery)
{
  prologQuery = "build_pipeline_from_predicates([";
  std::vector<std::string> queriedKeys;
  extractQueryKeysFromDesignator(desig, queriedKeys);
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


std::string JsonPrologInterface::buildPrologQueryFromKeys(const std::vector<std::string> &keys)
{
  std::string prologQuery = "pipeline_from_predicates_with_domain_constraint([";
  for(int i = 0; i < keys.size(); i++)
  {
    prologQuery += keys.at(i);
    if(i < keys.size() - 1)
    {
      prologQuery += ",";
    }
  }
  prologQuery += "], A)";
  return prologQuery;
}

bool JsonPrologInterface::planPipelineQuery(const std::vector<std::string> &keys,
    std::vector<std::string> &pipeline)
{

  outInfo("Calling Json Prolog");
  json_prolog::Prolog pl;
  json_prolog::PrologQueryProxy bdgs = pl.query(buildPrologQueryFromKeys(keys));
  if(bdgs.begin() == bdgs.end())
  {
    outInfo("Can't find solution for pipeline planning");
    return false; // Indicate failure
  }
  for(auto bdg : bdgs)
  {
    pipeline = createPipelineFromPrologResult(bdg["A"].toString());
  }
  return true;

}

std::vector< std::string > JsonPrologInterface::createPipelineFromPrologResult(std::string queryResult)
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

bool JsonPrologInterface::retrieveAnnotatorsInputOutput(std::vector<std::string> &annotators,
    AnnotatorDependencies &dependencies)
{
  json_prolog::Prolog pl;
  std::string query;

  if(annotators.empty())
  {
    outWarn("Annotator list is empty!");
    return false;
  }

  for(int it  = 0; it < annotators.size(); it++)
  {
    dependencies[annotators[it]] = std::make_pair(std::unordered_set<std::string>(), std::unordered_set<std::string>());
    std::string token;

    query = QUERY_ANNOTATOR_INPUTS(annotators[it]);
    for(auto bdg : pl.query(query))
    {
      token = bdg[QUERY_ANNOTATOR_INPUTS_VAR].toString();
      token.erase(0, std::string(ROBOSHERLOCK_QUERY_PREFIX).length() + 1);
      token.pop_back(); // erase last character

      dependencies[annotators[it]].first.insert(token);
    }

    query = QUERY_ANNOTATOR_OUTPUTS(annotators[it]);
    for(auto bdg : pl.query(query))
    {
      token = bdg[QUERY_ANNOTATOR_OUTPUTS_VAR].toString();
      token.erase(0, std::string(ROBOSHERLOCK_QUERY_PREFIX).length() + 1);
      token.pop_back(); // erase last character

      dependencies[annotators[it]].second.insert(token);
    }
  }

  return true;
}

bool JsonPrologInterface::q_subClassOf(std::string child, std::string parent)
{

  std::stringstream prologQuery;
  json_prolog::Prolog pl;

  if(addNamespace(child) && addNamespace(parent))
  {
    prologQuery << "owl_subclass_of(" << child << "," << parent << ").";
    outInfo("Asking Query: " << prologQuery.str());
    json_prolog::PrologQueryProxy bdgs = pl.query(prologQuery.str());
    if(bdgs.begin() != bdgs.end())
    {
      outInfo(child << " IS " << parent);
      return true;
    }
  }
  else
  {
    outError("Child or parent are not defined in the ontology under any of the known namespaces");
    outError(" Child : " << child);
    outError(" Parent: " << parent);
    return false;
  }
  return false;

}

bool JsonPrologInterface::retractAllAnnotators()
{
  std::stringstream query;
  query << "owl_subclass_of(S,rs_components:'RoboSherlockComponent'),rdf_retractall(_,rdf:type,S)";
  json_prolog::Prolog pl;
  pl.query(query.str());
  return true;
}

bool JsonPrologInterface::assertAnnotators( const  std::map<std::string,rs::AnnotatorCapabilities> &annotatorCapabilities)
{
  json_prolog::Prolog pl;
  for(auto a : annotatorCapabilities)
  {
    std::string nameWithNS;
    if(addNamespace(a.first, nameWithNS))
    {
      std::stringstream prologQuery;
      prologQuery << "owl_instance_from_class(" << nameWithNS << "," << "I)";
      json_prolog::PrologQueryProxy bdgs = pl.query(prologQuery.str());
      for(auto bdg : bdgs)
      {
        //outInfo("Individual generated: " << bdg["I"]);
        assertAnnotatorMetaInfo(a, bdg["I"]);
      }
    }
    else
    {
      outError("Annotator not modelled in KB:" << a.first);
      continue;
    }
  }
  return true;
}


bool JsonPrologInterface::assertAnnotatorMetaInfo(std::pair<std::string,rs::AnnotatorCapabilities> annotatorData, std::string individualOfAnnotator)
{
  std::vector<std::string> resultDomain;
  if(lookupAnnotatorDomain(annotatorData.first, resultDomain))
  {
    std::vector<std::string> resultDomainInKnowRob;
    for(auto d : resultDomain)
    {
      d[0] = std::toupper(d[0]);
      if(!addNamespace(d))
      {
        outWarn("output domain element: [ " << d << " ] is not defined in Ontology. Will not be considered durin query answering");
        continue;
      }
      outInfo(d);
      resultDomainInKnowRob.push_back(d);
    }

    std::stringstream query;
    query << "set_annotator_domain(" << individualOfAnnotator << ",[";
    std::string separator = ",";
    for(auto it = resultDomainInKnowRob.begin(); it != resultDomainInKnowRob.end(); ++it)
    {
      if(std::next(it) == resultDomainInKnowRob.end()) separator = "";
      query << *it << separator;
    }
    query << "]).";

    outInfo("Query: " << query.str());
    json_prolog::Prolog pl;
    json_prolog::PrologQueryProxy bdgs = pl.query(query.str());
    if(bdgs.begin() != bdgs.end())
    {
      return true;
    }
    return false;
  }
  return false;
}

bool JsonPrologInterface::lookupAnnotatorDomain(std::string annotatorName, std::vector<std::string> &domain)
{
  //  std::string pathToAnnot = rs::common::getAnnotatorPath(annotatorName);
  boost::filesystem::path pathToAnnot(rs::common::getAnnotatorPath(annotatorName));

  if(pathToAnnot.extension() == ".yaml")
  {
    YAML::Node config;
    config = YAML::LoadFile(pathToAnnot.string());
    for(YAML::const_iterator it = config.begin(); it != config.end(); ++it)
    {
      YAML::Node key = it->first;
      YAML::Node value = it->second;
      if(key.Type() == YAML::NodeType::Scalar)
      {
        std::string nodeName = key.as<std::string>();
        if(nodeName == "capabilities")
        {
          if(value.Type() == YAML::NodeType::Map)
          {
            for(YAML::const_iterator mit = value.begin(); mit != value.end(); ++mit)
            {
              std::string name = mit->first.as<std::string>();
              if(name == "domain")
              {
                domain = mit->second.as<std::vector<std::string>>();
                return true;
              }
            }
          }
        }
      }
    }
  }
  return false;
}


bool JsonPrologInterface::expandToFullUri(std::string &entry)
{
  json_prolog::Prolog pl;
  std::stringstream prologQuery;
  prologQuery << "rdf_global_id(" << entry << ",A).";
  json_prolog::PrologQueryProxy bdgs = pl.query(prologQuery.str());
  for(auto bdg : bdgs)
  {
    std::string newentry = bdg["A"];
    entry = newentry;
    return true;
  }
  return false;
}

bool JsonPrologInterface::addNamespace(const std::string &entry, std::string &results)
{
  results = entry;
  return addNamespace(results);
}

bool JsonPrologInterface::addNamespace(std::string &entry)
{
  json_prolog::Prolog pl;
  for(auto ns : rs_queryanswering::krNamespaces)
  {
    std::stringstream prologQuery;
    prologQuery << "rdf_has(" << ns << ":'" << entry << "',rdf:type, owl:'Class').";
    json_prolog::PrologQueryProxy bdgs = pl.query(prologQuery.str());
    if(bdgs.begin() != bdgs.end())
    {
      entry = ns + ":'" + entry + "'";
      return true;
    }
  }
  return false;
}

bool JsonPrologInterface::q_classProperty(std::string className, std::string property, std::string value)
{

  outInfo("Calling Json Prolog");
  return false;
}

#endif
