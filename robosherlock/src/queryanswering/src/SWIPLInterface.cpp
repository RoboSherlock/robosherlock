#include <robosherlock/queryanswering/SWIPLInterface.h>

namespace rs
{
SWIPLInterface::SWIPLInterface()
{
  std::lock_guard<std::mutex> lock(lock_);
  int argc = 0;
  argv[argc++] = (char*)"robosherlock";
  argv[argc++] = "-g";
  argv[argc++] = "true";
  // Inhibit any signal handling by Prolog
  argv[argc++] = (char*)"--signals=false";
  // set the file to load at startup
  argv[argc++] = "-f";
  std::string rosPrologInit = ros::package::getPath("robosherlock") + "/prolog/init_rs_internal.pl";
  argv[argc] = new char[rosPrologInit.size() + 1];
  std::copy(rosPrologInit.begin(), rosPrologInit.end(), argv[argc]);
  argv[argc++][rosPrologInit.size()] = '\0';
  argv[argc++] = "-O";
  argv[argc] = NULL;
  PL_initialise(argc, argv);

  std::setlocale(LC_ALL, "C");  // PL_initialise will get whatever locale you have set on you system and change the
                                // current settings; so we need to set it back to C style;
#if SWIPL_VERSION < 8
  attributes_.local_size = 100000000;
  attributes_.global_size = 100000000;
  attributes_.trail_size = 100000000;
  attributes_.argument_size = 0;
#else
  attributes_.stack_limit = 10000000;
#endif
  attributes_.alias = 0;
  attributes_.cancel = 0;
  attributes_.flags = 0;

  outInfo("SWI-Prolog engine initialized.");
}

void SWIPLInterface::setEngine()
{
  PL_engine_t current_engine;
  PL_set_engine(PL_ENGINE_CURRENT, &current_engine);
  if (current_engine == 0)
  {
    PL_engine_t new_engine = PL_create_engine(&attributes_);
    engine_pool_.insert(new_engine);  // store new engine so we can delete them at the end;
    int result = PL_set_engine(new_engine, 0);
    //    }
    if (result != PL_ENGINE_SET)
    {
      if (result == PL_ENGINE_INVAL)
        throw std::runtime_error("Engine is invalid.");
      else if (result == PL_ENGINE_INUSE)
        throw std::runtime_error("Engine is invalid.");
      else
        throw std::runtime_error("Unknown Response when setting PL_engine");
    }
  }
}

void SWIPLInterface::printObjPropertyKeys()
{
  std::lock_guard<std::mutex> lock(lock_);
  setEngine();
  outDebug("Printing loaded query keys that refer to object properties in the KB: ");
  PlTermv av(1);
  try
  {
    std::shared_ptr<PlQuery> q(new PlQuery("key_is_property", av));
    while (q->next_solution())
    {
      outDebug((char*)av[0]);
    }
  }
  catch (PlException& ex)
  {
    outError(static_cast<char*>(ex));
  }
}

void SWIPLInterface::printQueryKeys()
{
  std::lock_guard<std::mutex> lock(lock_);
  setEngine();
  outDebug("Printing loaded query keys: ");
  PlTermv av(2);
  try
  {
    std::shared_ptr<PlQuery> q(new PlQuery("rs_type_for_predicate", av));
    while (q->next_solution())
    {
      outDebug((char*)av[0] << " " << (char*)av[1]);
    }
  }
  catch (PlException& ex)
  {
    outError(static_cast<char*>(ex));
  }
}

bool SWIPLInterface::planPipelineQuery(const std::vector<std::string>& keys, std::vector<std::string>& pipeline)
{
  std::lock_guard<std::mutex> lock(lock_);
  outInfo("Planning Pipeline");
  setEngine();
  PlTermv av(2);
  PlTail l(av[0]);
  for (auto key : keys)
    l.append(key.c_str());
  l.close();
  try
  {
    std::shared_ptr<PlQuery> q(new PlQuery("pipeline_from_predicates_with_domain_constraint", av));
    std::string prefix("http://knowrob.org/kb/rs_components.owl#");
    while (q->next_solution())
    {
      //      std::cerr<<(char*)av[1]<<std::endl;
      PlTail res(av[1]);  // result is a list
      PlTerm e;           // elements of that list
      while (res.next(e))
      {
        std::string element((char*)e);
        element.erase(0, prefix.length());
        pipeline.push_back(element);
      }
    }
    q.reset();
  }
  catch (PlException& ex)
  {
    outError(static_cast<char*>(ex));
  }
  // interesting; if I

  //  releaseEngine();
  return true;
}

bool SWIPLInterface::q_subClassOf(std::string child, std::string parent)
{
  if (!addNamespace(child))
  {
    outWarn(child << " is not found under any of the namespaces");
    return false;
  }
  if (!addNamespace(parent))
  {
    outWarn(parent << " is not found under any of the namespaces");
    return false;
  }
  std::lock_guard<std::mutex> lock(lock_);
  setEngine();
  PlTermv av(2);
  std::stringstream query;
  query << "owl_subclass_of(" << child << "," << parent << ")";
  av[0] = child.c_str();
  av[1] = parent.c_str();
  int res;  // = PlCall("owl_subclass_of", av);
  outDebug("Calling Query: " << query.str());
  try
  {
    res = PlCall(query.str().c_str());
    outDebug("result of PlCall:" << res);
    if (res)
    {
      outDebug(child << " is subclass of " << parent);
      //      releaseEngine();
      return true;
    }
    else
    {
      outDebug(child << " is NOT subclass of " << parent);
      //      releaseEngine();
      return false;
    }
  }
  catch (PlException& ex)
  {
    outError((char*)ex);
    //    releaseEngine();
    return false;
  }
}


bool SWIPLInterface::q_getClassProperty(std::string subject, std::string relation, std::string value)
{
//  if (!addNamespace(object))
//  {
//    outWarn(object << " is not found under any of the namespaces");
//    return false;
//  }
  std::stringstream ss;
  ss<<"literal(type('http://www.w3.org/2001/XMLSchema#float','"<<value<<"'))";
  value = ss.str();
  if (!addNamespace(subject))
  {
    outWarn(subject << " is not found under any of the namespaces");
    return false;
  }
  if (!addNamespace(relation, "data-property"))
  {
    outWarn(relation << " is not found under any of the namespaces");
    return false;
  }
  std::lock_guard<std::mutex> lock(lock_);
  setEngine();
  PlTermv av(3);
  std::stringstream query;
  query << "owl_class_properties(" << subject << "," << relation << "," << value << ")";
  av[0] = subject.c_str();
  av[1] = relation.c_str();
  av[2] = value.c_str();
  int res;  // = PlCall("owl_subclass_of", av);
  outDebug("Calling Query: " << query.str());
  try
  {
    res = PlCall(query.str().c_str());
    outDebug("result of PlCall:" << res);
    if (res)
    {
      outDebug(subject << " is in  " << relation << " with: " << value);
      return true;
    }
    else
    {
      outDebug(subject << " is not in  " << relation << " with: " << value);
      return false;
    }
  }
  catch (PlException& ex)
  {
    outError((char*)ex);
    return false;
  }
  return true;
}


bool SWIPLInterface::q_hasClassProperty(std::string subject, std::string relation, std::string object)
{
  if (!addNamespace(object))
  {
    outWarn(object << " is not found under any of the namespaces");
    return false;
  }
  if (!addNamespace(subject))
  {
    outWarn(subject << " is not found under any of the namespaces");
    return false;
  }
  if (!addNamespace(relation, "obj-property"))
  {
    outWarn(relation << " is not found under any of the namespaces");
    return false;
  }
  std::lock_guard<std::mutex> lock(lock_);
  setEngine();
  PlTermv av(3);
  std::stringstream query;
  query << "owl_class_properties(" << subject << "," << relation << "," << object << ")";
  av[0] = subject.c_str();
  av[1] = relation.c_str();
  av[2] = object.c_str();
  int res;  // = PlCall("owl_subclass_of", av);
  outDebug("Calling Query: " << query.str());
  try
  {
    res = PlCall(query.str().c_str());
    outDebug("result of PlCall:" << res);
    if (res)
    {
      outDebug(subject << " is in  " << relation << " with: " << object);
      return true;
    }
    else
    {
      outDebug(subject << " is not in  " << relation << " with: " << object);
      return false;
    }
  }
  catch (PlException& ex)
  {
    outError((char*)ex);
    return false;
  }
  return true;
}

bool SWIPLInterface::assertQueryLanguage(
    std::vector<std::tuple<std::string, std::vector<std::string>, int>>& query_terms)
{
  MEASURE_TIME;
  outDebug("Asserting query language specific knowledge");
  setEngine();
  try
  {
    for (auto term : query_terms)
    {
      std::string q_predicate;
      std::vector<std::string> types;
      int type_of_predicate;
      std::tie(q_predicate, types, type_of_predicate) = term;
      std::stringstream query;
      query << "assert(rs_query_reasoning:rs_query_predicate(" << q_predicate << "))";
      int res;
      {
        std::lock_guard<std::mutex> lock(lock_);
        res = PlCall(query.str().c_str());
      }
      int res2 = true;
      if (type_of_predicate == 1)
      {
        query.str("");
        query << "assert(rs_query_reasoning:key_is_property(" << q_predicate << "))";
        {
          std::lock_guard<std::mutex> lock(lock_);
          res2 = PlCall(query.str().c_str());
          if (res2)
          {
            outDebug("Assertion successfull: " << query.str());
            outDebug("Asserted " << q_predicate << " as a property request in query language term");
          }
        }
      }

      if (res && res2)
      {
        outDebug("Asserted " << q_predicate << " as a query language term");
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
          if (!KnowledgeEngine::addNamespace(krType.str(), krTypeClass))
          {
            outWarn(krType.str() << "Was not found in ontology");
            continue;
          }
          query << "assert(rs_type_for_predicate(" << q_predicate << "," << krTypeClass << "))";
          if (PlCall(query.str().c_str()))
            outDebug("Assertion successfull: " << query.str());
        }
      }
    }
  }
  catch (PlException& ex)
  {
    outError(static_cast<char*>(ex));
  }
  printQueryKeys();
  // releaseEngine();
  return true;
}

bool SWIPLInterface::assertOutputTypeRestriction(const std::string& individual, const std::vector<std::string>& values,
                                                 std::string& type)
{
  std::lock_guard<std::mutex> lock(lock_);
  setEngine();

  std::stringstream query;
  query << "rs_query_reasoning:set_annotator_output_type_domain(" << individual << ",[";
  std::string separator = ",";
  for (auto it = values.begin(); it != values.end(); ++it)
  {
    if (std::next(it) == values.end())
      separator = "";
    query << *it << separator;
  }
  query << "], " << type << ").";
  outDebug("Query: " << query.str());
  try
  {
    PlCall(query.str().c_str());
  }
  catch (PlException& ex)
  {
    outDebug(static_cast<char*>(ex));
    return false;
  }
  return true;
}

bool SWIPLInterface::assertInputTypeConstraint(const std::string& individual, const std::vector<std::string>& values,
                                               std::string& type)
{
  std::lock_guard<std::mutex> lock(lock_);
  setEngine();

  std::stringstream query;
  query << "rs_query_reasoning:set_annotator_input_type_constraint(" << individual << ",[";
  std::string separator = ",";
  for (auto it = values.begin(); it != values.end(); ++it)
  {
    if (std::next(it) == values.end())
      separator = "";
    query << *it << separator;
  }
  query << "], " << type << ").";
  outDebug("Query: " << query.str());
  try
  {
    PlCall(query.str().c_str());
  }
  catch (PlException& ex)
  {
    outError(static_cast<char*>(ex));
    return false;
  }
  return true;
}

bool SWIPLInterface::assertValueForKey(const std::string& key, const std::string& value)
{
  std::lock_guard<std::mutex> lock(lock_);
  setEngine();

  std::stringstream assertionQuery;
  assertionQuery << "assert(requestedValueForKey(" << key << "," << (value == "" ? "\'\'" : value) << "))";
  outDebug("Calling query: " << assertionQuery.str());
  int res;
  try
  {
    res = PlCall(assertionQuery.str().c_str());
  }
  catch (PlException& ex)
  {
    outError(static_cast<char*>(ex));
    return false;
  }
  if (res)
  {
    outDebug("Asserted: " << assertionQuery.str());
    return true;
  }
  else
  {
    outError("Asserttion: " << assertionQuery.str() << " failed!");
    return false;
  }
}

bool SWIPLInterface::retractQueryLanguage()
{
  std::lock_guard<std::mutex> lock(lock_);
  outDebug("Retracting Query language");
  setEngine();

  try
  {
    PlCall("retractall(rs_type_for_predicate(_,_))");
    PlCall("retractall(rs_query_predicate(_))");
  }
  catch (PlException& ex)
  {
    outError(static_cast<char*>(ex));
  }

  //  q.reset();
  //  releaseEngine();
  return true;
}

bool SWIPLInterface::retractQueryKvPs()
{
  std::lock_guard<std::mutex> lock(lock_);
  outDebug("Retractking Query KvPs");
  setEngine();
  try
  {
    PlCall("retract(requestedValueForKey(_,_))");
  }
  catch (PlException& ex)
  {
    outError(static_cast<char*>(ex));
    return false;
  }
  //    releaseEngine();
  return true;
}

bool SWIPLInterface::checkValidQueryTerm(const std::string& term)
{
  std::lock_guard<std::mutex> lock(lock_);
  setEngine();
  std::stringstream checkTermQuery;
  checkTermQuery << "rs_query_reasoning:rs_query_predicate(" << term << ")";
  int res;
  try
  {
    res = PlCall(checkTermQuery.str().c_str());
  }
  catch (PlException& ex)
  {
    outError(static_cast<char*>(ex));
    return false;
  }
  if (res)
    return true;
  else
    return false;
}

bool SWIPLInterface::instanceFromClass(const std::string& class_name, std::vector<std::string>& individualsOF)
{
  std::lock_guard<std::mutex> lock(lock_);
  setEngine();
  PlTermv av(2);
  av[0] = class_name.substr(1, class_name.size() - 2).c_str();

  try
  {
    std::shared_ptr<PlQuery> query(new PlQuery("owl_instance_from_class", av));
    while (query->next_solution())
    {
      std::string indiv = "'" + std::string(static_cast<char*>(av[1])) + "'";
      individualsOF.push_back(indiv);
    }
  }
  catch (PlException& ex)
  {
    outError(static_cast<char*>(ex));
    return false;
  }
  return true;
}

bool SWIPLInterface::retractAllAnnotators()
{
  std::lock_guard<std::mutex> lock(lock_);
  outDebug("Retracting all annotators");
  setEngine();

  PlTermv av(2);
  av[1] = "http://knowrob.org/kb/rs_components.owl#RoboSherlockComponent";

  try
  {
    std::shared_ptr<PlQuery> check(new PlQuery("owl_individual_of", av));
    while (check->next_solution())
    {
      std::stringstream query;
      query << "rdf_db:rdf_retractall('" << static_cast<char*>(av[0])
            << "','http://www.w3.org/1999/02/22-rdf-syntax-ns#type',_)";
      PlCall(query.str().c_str());
    }
  }
  catch (PlException& ex)
  {
    outError(static_cast<char*>(ex));
    return false;
  }
  //  releaseEngine();
  return true;
}

bool SWIPLInterface::addNamespace(std::string& s, std::string type)
{
  std::lock_guard<std::mutex> lock(lock_);
  //  outInfo("Adding namespace to: " << s);
  setEngine();
  try
  {
    if (krNamespaces_.empty())
    {
      PlTermv av(2);
      std::shared_ptr<PlQuery> q(new PlQuery("rdf_current_prefix", av));
      while (q->next_solution())
      {
        std::string ns = std::string(static_cast<char*>(av[1]));
        if (std::find(rs::NS_TO_SKIP.begin(), rs::NS_TO_SKIP.end(), ns) == rs::NS_TO_SKIP.end())
          krNamespaces_.push_back(ns);
      }
      //      q.reset();
    }
    for (auto ns : krNamespaces_)
    {
      std::stringstream prologQuery;
      if (type == "class")
        prologQuery << "rdf_has('" << ns << s << "'," << RDF_TYPE << ",'http://www.w3.org/2002/07/owl#Class').";
      else if (type == "obj-property")
        prologQuery << "rdf_has('" << ns << s << "'," << RDF_TYPE
                    << ",'http://www.w3.org/2002/07/owl#ObjectProperty').";
      else if (type == "data-property")
        prologQuery << "rdf_has('" << ns << s << "'," << RDF_TYPE
                    << ",'http://www.w3.org/2002/07/owl#DatatypeProperty').";
      if (PlCall(prologQuery.str().c_str()))
      {
        s = "'" + ns + s + "'";
        return true;
      }
    }
  }
  catch (PlException& ex)
  {
    outError(static_cast<char*>(ex));
    return false;
  }
  //  releaseEngine();
  return false;
}

}  // namespace rs
