#include <rs/queryanswering/JsonPrologInterface.h>
#ifdef WITH_JSON_PROLOG

using namespace xercesc;

JsonPrologInterface::JsonPrologInterface()
{
  outInfo("Creating ROS Service client for json_prolog");
  try {
    XMLPlatformUtils::Initialize();
  }
  catch (const XMLException& toCatch) {
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
        outInfo("Asserting value key pair: " << iter->value.GetString());
        std::stringstream assertionQuery;
        assertionQuery << "assert(requestedValueForKey(" << iter->name.GetString() << "," << iter->value.GetString() << ")";
        json_prolog::Prolog pl;
        json_prolog::PrologQueryProxy bdgs = pl.query(assertionQuery.str());
        if(bdgs.begin() != bdgs.end())
        {
          outInfo("Asserted: " << assertionQuery.str());
          return true;
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
  std::string prologQuery = "build_single_pipeline_from_predicates([";
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
     query<< "owl_subclass_of(S,rs_components:'RoboSherlockComponent'),rdf_retractall(_,rdf:type,S)";
     json_prolog::Prolog pl;
     pl.query(query.str());
     return true;
}

bool JsonPrologInterface::assertAnnotators(std::vector<std::string> annotatorNames)
{
  json_prolog::Prolog pl;
  for(auto a : annotatorNames)
  {
    if(addNamespace(a))
    {
      std::stringstream prologQuery;
      prologQuery << "owl_instance_from_class(" << a << "," << "I)";
      json_prolog::PrologQueryProxy bdgs = pl.query(prologQuery.str());
      for(auto bdg : bdgs)
      {
        outInfo("Individual generated: " << bdg["I"]);
      }
    }
    else
    {
      outError("Annotator not modelled in KB:" << a);
      continue;
    }
  }
  return true;
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
