#include <robosherlock/queryanswering/JsonPrologInterface.h>

#ifdef WITH_ROS_PROLOG

namespace rs
{
JsonPrologInterface::JsonPrologInterface()
{
  outInfo("Creating ROS Service client for rosprolog");
}

bool JsonPrologInterface::assertValueForKey(const std::string& key, const std::string& value)
{
  std::stringstream assertionQuery;
  assertionQuery << "assert(requestedValueForKey(" << key << "," << (value == "" ? "\'\'" : value) << "))";
  outInfo("Calling query: " << assertionQuery.str());
  PrologQuery bdgs = queryWithLock(assertionQuery.str());
  if (bdgs.begin() != bdgs.end())
  {
    outInfo("Asserted: " << assertionQuery.str());
    return true;
  }
  else
  {
    outError("Asserttion: " << assertionQuery.str() << " failed!");
    return false;
  }
}

bool JsonPrologInterface::retractQueryKvPs()
{
  queryWithLock("retract(requestedValueForKey(_,_))");
  return true;
}

bool JsonPrologInterface::checkValidQueryTerm(const std::string& term)
{
  std::stringstream checkTermQuery;
  checkTermQuery << "rs_query_predicate(" << term << ")";
  PrologQuery bindings = queryWithLock(checkTermQuery.str());
  if (bindings.begin() != bindings.end())
    return true;
  else
    return false;
}

std::string JsonPrologInterface::buildPrologQueryFromKeys(const std::vector<std::string>& keys)
{
  std::string prologQuery = "pipeline_from_predicates_with_domain_constraint([";
  for (unsigned int i = 0; i < keys.size(); i++)
  {
    prologQuery += keys.at(i);
    if (i < keys.size() - 1)
    {
      prologQuery += ",";
    }
  }
  prologQuery += "], A)";
  return prologQuery;
}

bool JsonPrologInterface::planPipelineQuery(const std::vector<std::string>& keys, std::vector<std::string>& pipeline)
{
  outInfo("Calling Json Prolog");
  PrologQuery bdgs = queryWithLock(buildPrologQueryFromKeys(keys));
  if (bdgs.begin() == bdgs.end())
  {
    outInfo("Can't find solution for pipeline planning");
    return false;  // Indicate failure
  }
  for (auto bdg : bdgs)
  {
    pipeline = createPipelineFromPrologResult(bdg["A"].toString());
  }
  return true;
}

std::vector<std::string> JsonPrologInterface::createPipelineFromPrologResult(std::string queryResult)
{
  std::vector<std::string> new_pipeline;

  // Strip the braces from the result
  queryResult.erase(queryResult.end() - 1);
  queryResult.erase(queryResult.begin());

  std::stringstream resultstream(queryResult);

  std::string token;
  while (std::getline(resultstream, token, ','))
  {
    // erase leading whitespaces
    token.erase(token.begin(), std::find_if(token.begin(), token.end(), std::bind1st(std::not_equal_to<char>(), ' ')));
    outDebug("Planned Annotator by Prolog Planner " << token);

    // From the extracted tokens, remove the prefix
    std::string prefix("http://knowrob.org/kb/rs_components.owl#");
    uint8_t prefix_length = prefix.length();

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
  if (addNamespace(child) && addNamespace(parent))
  {
    prologQuery << "owl_subclass_of(" << child << "," << parent << ").";
    outInfo("Asking Query: " << prologQuery.str());
    PrologQuery bdgs = queryWithLock(prologQuery.str());
    if (bdgs.begin() != bdgs.end())
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

bool JsonPrologInterface::retractQueryLanguage()
{
  try
  {
    std::stringstream query;
    query << "retractall(rs_type_for_predicate(_,_))";
    queryWithLock(query.str());
    query.str("");
    query << "retractall(rs_query_predicate(_))";
    queryWithLock(query.str());
  }
  catch (...)
  {
  }
  return true;
}

bool JsonPrologInterface::q_hasClassProperty(std::string subject, std::string relation, std::string object)
{
  outWarn("HAS CLASS PROPERTY IS NOT IMPLEMENTED FOR JSON PROLOG INTERFACE");
  return false;
}
bool JsonPrologInterface::q_getClassProperty(std::string subject, std::string relation, std::string object)
{
    outWarn("GET CLASS PROPERTY IS NOT IMPLEMENTED FOR JSON PROLOG INTERFACE");
    return false;
}


bool JsonPrologInterface::assertQueryLanguage(
    std::vector<std::tuple<std::string, std::vector<std::string>, int>>& query_terms)
{
  for (auto term : query_terms)
  {
    std::string q_predicate;
    std::vector<std::string> types;
    int type_of_predicate;
    std::tie(q_predicate, types, type_of_predicate) = term;

    std::stringstream query;
    query << "assert(rs_query_predicate(" << q_predicate << "))";
    PrologQuery bdgs = queryWithLock(query.str());
    bool res1 = false, res2 = false;
    if (bdgs.begin() != bdgs.end())
    {
      outInfo("Asserted " << q_predicate << " as a query language term");
      res1 = true;
    }
    if (type_of_predicate == 1)
    {
      query.str("");
      query << "assert(key_is_property(" << q_predicate << "))";
      bdgs = queryWithLock(query.str());
      if (bdgs.begin() != bdgs.end())
      {
        outDebug("Assertion successfull: " << query.str());
        outDebug("Asserted " << q_predicate << " as a property request in query language term");
        res2 = true;
      }
    }

    for (auto type : types)
    {
      std::string token;
      std::stringstream ss(type), krType;
      while (std::getline(ss, token, '.'))
      {
        std::transform(token.begin(), token.end(), token.begin(), ::tolower);
        token[0] = std::toupper(token[0]);
        krType << token;
      }
      query.str("");
      std::string krTypeClass;
      KnowledgeEngine::addNamespace(krType.str(), krTypeClass);
      query << "rdf_global_id(" << krTypeClass << ",A),assert(rs_type_for_predicate(" << q_predicate << ",A))";
      PrologQuery bdgs = queryWithLock(query.str());
      if (bdgs.begin() != bdgs.end())
      {
        outInfo("Asserted " << krTypeClass << " as a type for " << q_predicate);
      }
    }
  }
  return true;
}  // namespace rs

bool JsonPrologInterface::instanceFromClass(const std::string& class_name, std::vector<std::string>& individualsOF)
{
  std::stringstream prologQuery;
  prologQuery << "owl_instance_from_class(" << class_name << ","
              << "I)";
  PrologQuery bdgs = queryWithLock(prologQuery.str());
  for (auto bdg : bdgs)
    individualsOF.push_back(bdg["I"]);
  return true;
}

bool JsonPrologInterface::retractAllAnnotators()
{
  std::stringstream query;
  query << "owl_subclass_of(S,rs_components:'RoboSherlockComponent'),rdf_retractall(_,rdf:type,S)";
  queryWithLock(query.str());
  return true;
}

bool JsonPrologInterface::expandToFullUri(std::string& entry)
{
  std::stringstream prologQuery;
  prologQuery << "rdf_global_id(" << entry << ",A).";
  PrologQuery bdgs = queryWithLock(prologQuery.str());
  for (auto bdg : bdgs)
  {
    std::string newentry = bdg["A"];
    entry = newentry;
    return true;
  }
  return false;
}

bool JsonPrologInterface::assertInputTypeConstraint(const std::string& individual,
                                                    const std::vector<std::string>& values, std::string& type)
{
  std::stringstream query;
  query << "set_annotator_input_type_constraint(" << individual << ",[";
  std::string separator = ",";
  for (auto it = values.begin(); it != values.end(); ++it)
  {
    if (std::next(it) == values.end())
      separator = "";
    query << *it << separator;
  }
  query << "], " << type << ").";

  outInfo("Query: " << query.str());
  PrologQuery bdgs = queryWithLock(query.str());
  if (bdgs.begin() != bdgs.end())
    return true;
  else
    return false;
}

bool JsonPrologInterface::assertOutputTypeRestriction(const std::string& individual,
                                                      const std::vector<std::string>& values, std::string& type)
{
  std::stringstream query;
  query << "set_annotator_output_type_domain(" << individual << ",[";
  std::string separator = ",";
  for (auto it = values.begin(); it != values.end(); ++it)
  {
    if (std::next(it) == values.end())
      separator = "";
    query << *it << separator;
  }
  query << "], " << type << ").";

  outInfo("Query: " << query.str());
  PrologQuery bdgs = queryWithLock(query.str());
  if (bdgs.begin() != bdgs.end())
    return true;
  else
    return false;
}

bool JsonPrologInterface::addNamespace(std::string& entry, std::string entry_type)
{
  if (krNamespaces_.empty())
  {
    std::stringstream getNamespacesQuery;
    getNamespacesQuery << "rdf_current_ns(A,_)";
    PrologQuery bdgs = queryWithLock(getNamespacesQuery.str());
    for (auto bdg : bdgs)
    {
      std::string ns = bdg["A"].toString();
      ns = ns.substr(1, ns.size() - 2);
      if (std::find(rs::NS_TO_SKIP.begin(), rs::NS_TO_SKIP.end(), ns) == rs::NS_TO_SKIP.end())
      {
        krNamespaces_.push_back(ns);
      }
    }
  }
  for (auto ns : krNamespaces_)
  {
    std::stringstream prologQuery;
    if (entry_type == "class")
      prologQuery << "rdf_has(" << ns << ":'" << entry << "',rdf:type, owl:'Class').";
    else if (entry_type == "obj-property")
      prologQuery << "rdf_has(" << ns << ":'" << entry << "',rdf:type, owl:'ObjectProperty').";
    PrologQuery bdgs = queryWithLock(prologQuery.str());
    if (bdgs.begin() != bdgs.end())
    {
      entry = ns + ":'" + entry + "'";
      return true;
    }
  }
  return false;
}

PrologQuery JsonPrologInterface::queryWithLock(const std::string& query)
{
  std::lock_guard<std::mutex> lock(lock_);
  return pl_.query(query);
}

}  // namespace rs
#endif
