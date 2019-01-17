#include <rs/queryanswering/JsonPrologInterface.h>
#ifdef WITH_JSON_PROLOG

using namespace xercesc;

JsonPrologInterface::JsonPrologInterface()
{
  outInfo("Creating ROS Service client for json_prolog");
  try {
    XMLPlatformUtils::Initialize();
  }
  catch(const XMLException &toCatch) {
    outInfo("error starting an xml parser: " << toCatch.getMessage());
  }
  std::stringstream getNamespacesQuery;
  getNamespacesQuery << "rdf_current_ns(A,_)";
  json_prolog::PrologQueryProxy bdgs = pl_.query(getNamespacesQuery.str());
  for(auto bdg : bdgs) {
    std::string ns = bdg["A"].toString();
    if(std::find(NS_TO_SKIP.begin(), NS_TO_SKIP.end(), ns) == NS_TO_SKIP.end())
      krNamespaces_.push_back(ns);
  }
}


bool JsonPrologInterface::extractQueryKeysFromDesignator(std::string &desig,
    std::vector<std::string> &keys)
{
  rapidjson::Document json;
  json.Parse(desig.c_str());
  if(!json.IsObject()) {
    outError("Query could not be converted to json object!");
    return false;
  }

  json_prolog::Prolog pl;
  json_prolog::PrologQueryProxy bdgs = pl.query("retract(requestedValueForKey(_,_))");

  //add the ones that are interpretable to the queriedKeys;
  for(rapidjson::Value::ConstMemberIterator iter = json.MemberBegin(); iter != json.MemberEnd(); ++iter) {
    std::stringstream checkKeyQuery;
    checkKeyQuery << "rs_query_predicate(" << iter->name.GetString() << ")";
    json_prolog::PrologQueryProxy bindings = pl.query(checkKeyQuery.str());
    if(bindings.begin() != bindings.end()) {
      keys.push_back(iter->name.GetString());
      //for a select member of keys (type, class, shape, color) let's add value reasoning; TODO: get rid of this somehow;
      std::vector<std::string> special_keys = {"type", "class", "shape", "color", "size"};
      if(std::find(special_keys.begin(), special_keys.end(),
                   iter->name.GetString()) != std::end(special_keys)) {
        std::string d = iter->value.GetString();
        d[0] = std::toupper(d[0]);
        if(d != "" && !addNamespace(d)) {
          outWarn("No OWL definitions for " << d << " under any of the known namespaces");
          continue;
        }
        std::stringstream assertionQuery;
        assertionQuery << "assert(requestedValueForKey(" << iter->name.GetString() << "," << (d == "" ? "\'\'" : d) << "))";
        outInfo("Calilng query: " << assertionQuery.str());
        json_prolog::PrologQueryProxy bdgs = pl.query(assertionQuery.str());
        if(bdgs.begin() != bdgs.end()) {
          outInfo("Asserted: " << assertionQuery.str());
        }
      }
    }
    else {
      outWarn(iter->name.GetString() << " is not a valid query-language term");
    }
  }
  return true;
}


bool JsonPrologInterface::buildPrologQueryFromDesignator(std::string &desig,
    std::string &prologQuery)
{
  prologQuery = "build_pipeline_from_predicates([";
  std::vector<std::string> queriedKeys;
  extractQueryKeysFromDesignator(desig, queriedKeys);
  for(uint8_t i = 0; i < queriedKeys.size(); i++) {
    prologQuery += queriedKeys.at(i);
    if(i < queriedKeys.size() - 1) {
      prologQuery += ",";
    }
  }
  prologQuery += "], A)";
  return true;
}


std::string JsonPrologInterface::buildPrologQueryFromKeys(const std::vector<std::string> &keys)
{
  std::string prologQuery = "pipeline_from_predicates_with_domain_constraint([";
  for(unsigned int i = 0; i < keys.size(); i++) {
    prologQuery += keys.at(i);
    if(i < keys.size() - 1) {
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
  if(bdgs.begin() == bdgs.end()) {
    outInfo("Can't find solution for pipeline planning");
    return false; // Indicate failure
  }
  for(auto bdg : bdgs) {
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
  while(std::getline(resultstream, token, ',')) {
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
  json_prolog::Prolog pl;

  if(addNamespace(child) && addNamespace(parent)) {
    prologQuery << "owl_subclass_of(" << child << "," << parent << ").";
    outInfo("Asking Query: " << prologQuery.str());
    json_prolog::PrologQueryProxy bdgs = pl.query(prologQuery.str());
    if(bdgs.begin() != bdgs.end()) {
      outInfo(child << " IS " << parent);
      return true;
    }
  }
  else {
    outError("Child or parent are not defined in the ontology under any of the known namespaces");
    outError(" Child : " << child);
    outError(" Parent: " << parent);
    return false;
  }
  return false;

}

bool JsonPrologInterface::retractQueryLanguage()
{
  try {
    std::stringstream query;
    query << "retractall(rs_type_for_predicate(_,_))";
    json_prolog::Prolog pl;
    pl.query(query.str());
    query.str("");
    query << "retractall(rs_query_predicate(_))";
    pl.query(query.str());
  }
  catch(...) {

  }
  return true;
}

bool JsonPrologInterface::assertQueryLanguage(std::map<std::string, std::vector<std::string> > &query_terms)
{
  json_prolog::Prolog pl;
  for(auto term : query_terms) {
    std::stringstream query;
    query << "assert(rs_query_predicate(" << term.first << "))";
    json_prolog::PrologQueryProxy bdgs = pl.query(query.str());
    if(bdgs.begin() != bdgs.end()) {
      outInfo("Asserted " << term.first << " as a query language term");
      for(auto type : term.second) {
        std::string token;
        std::stringstream ss(type), krType;
        while(std::getline(ss, token, '.')) {
          std::transform(token.begin(), token.end(), token.begin(), ::tolower);
          token[0] = std::toupper(token[0]);
          krType << token;
        }
        query.str("");
        std::string krTypeClass;
        addNamespace(krType.str(), krTypeClass);
        query << "rdf_global_id(" << krTypeClass << ",A),assert(rs_type_for_predicate(" << term.first << ",A))";
        json_prolog::PrologQueryProxy bdgs = pl.query(query.str());
        if(bdgs.begin() != bdgs.end()) {
          outInfo("Asserted " << krTypeClass << " as a type for " << term.first);
        }
      }
    }
  }
  return true;
}

bool JsonPrologInterface::assertAnnotators(const  std::map<std::string, rs::AnnotatorCapabilities> &annotatorCapabilities)
{
  json_prolog::Prolog pl;
  for(auto a : annotatorCapabilities) {
    std::string nameWithNS;
    if(addNamespace(a.first, nameWithNS)) {
      std::stringstream prologQuery;
      prologQuery << "owl_instance_from_class(" << nameWithNS << "," << "I)";
      json_prolog::PrologQueryProxy bdgs = pl.query(prologQuery.str());
      for(auto bdg : bdgs) {
        //outInfo("Individual generated: " << bdg["I"]);
        assertAnnotatorMetaInfo(a, bdg["I"]);
      }
    }
    else {
      outError("Annotator not modelled in KB:" << a.first);
      continue;
    }
  }
  return true;
}

bool JsonPrologInterface::retractAllAnnotators()
{
  std::stringstream query;
  query << "owl_subclass_of(S,rs_components:'RoboSherlockComponent'),rdf_retractall(_,rdf:type,S)";
  json_prolog::Prolog pl;
  pl.query(query.str());
  return true;
}



bool JsonPrologInterface::assertAnnotatorMetaInfo(std::pair<std::string, rs::AnnotatorCapabilities> annotatorData, std::string individualOfAnnotator)
{
  std::map<std::string, std::vector<std::string>> inputRestrictions = annotatorData.second.iTypeValueRestrictions;
  std::map<std::string, std::vector<std::string>> outputDomains = annotatorData.second.oTypeValueDomains;

  if(!inputRestrictions.empty()) {
    for(auto iR : inputRestrictions) {
      if(!iR.second.empty()) {
        std::vector<std::string> inputTypeConstraintInKnowRob;
        for(auto d : iR.second) {
          d[0] = std::toupper(d[0]);
          if(!addNamespace(d)) {
            outWarn("output domain element: [ " << d << " ] is not defined in Ontology. Will not be considered durin query answering");
            continue;
          }
          outInfo(d);
          inputTypeConstraintInKnowRob.push_back(d);
        }

        std::string typeName = iR.first, typeClass;
        std::vector<std::string> typeSplit;
        boost::algorithm::split(typeSplit, typeName, boost::is_any_of("."), boost::algorithm::token_compress_on);
        for(auto &t : typeSplit) {
          std::transform(t.begin(), t.end(), t.begin(), ::tolower);
          t[0] = std::toupper(t[0]);
          typeClass.append(t);
        }
        outInfo(typeName << ":" << typeClass);

        std::stringstream query;
        query << "set_annotator_input_type_constraint(" << individualOfAnnotator << ",[";
        std::string separator = ",";
        for(auto it = inputTypeConstraintInKnowRob.begin(); it != inputTypeConstraintInKnowRob.end(); ++it) {
          if(std::next(it) == inputTypeConstraintInKnowRob.end()) separator = "";
          query << *it << separator;
        }
        query << "], rs_components:'" << typeClass << "').";

        outInfo("Query: " << query.str());
        json_prolog::Prolog pl;
        json_prolog::PrologQueryProxy bdgs = pl.query(query.str());
      }
    }
  }

  if(!outputDomains.empty()) {
    for(auto oD : outputDomains) {
      if(!oD.second.empty()) {
        std::vector<std::string> resultDomainInKnowRob;
        for(auto d : oD.second) {
          d[0] = std::toupper(d[0]);
          if(!addNamespace(d)) {
            outWarn("output domain element: [ " << d << " ] is not defined in Ontology. Will not be considered durin query answering");
            continue;
          }
          outInfo(d);
          resultDomainInKnowRob.push_back(d);
        }

        std::string typeName = oD.first, typeClass;
        std::vector<std::string> typeSplit;
        boost::algorithm::split(typeSplit, typeName, boost::is_any_of("."), boost::algorithm::token_compress_on);
        for(auto &t : typeSplit) {
          std::transform(t.begin(), t.end(), t.begin(), ::tolower);
          t[0] = std::toupper(t[0]);
          typeClass.append(t);
        }
        outInfo(typeName << ":" << typeClass);

        std::stringstream query;
        query << "set_annotator_output_type_domain(" << individualOfAnnotator << ",[";
        std::string separator = ",";
        for(auto it = resultDomainInKnowRob.begin(); it != resultDomainInKnowRob.end(); ++it) {
          if(std::next(it) == resultDomainInKnowRob.end()) separator = "";
          query << *it << separator;
        }
        query << "], rs_components:'" << typeClass << "').";

        outInfo("Query: " << query.str());
        json_prolog::Prolog pl;
        json_prolog::PrologQueryProxy bdgs = pl.query(query.str());
      }
    }
  }
  return true;
}


bool JsonPrologInterface::expandToFullUri(std::string &entry)
{
  json_prolog::Prolog pl;
  std::stringstream prologQuery;
  prologQuery << "rdf_global_id(" << entry << ",A).";
  json_prolog::PrologQueryProxy bdgs = pl.query(prologQuery.str());
  for(auto bdg : bdgs) {
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
  for(auto ns : krNamespaces_) {
    std::stringstream prologQuery;
    prologQuery << "rdf_has(" << ns << ":'" << entry << "',rdf:type, owl:'Class').";
    json_prolog::PrologQueryProxy bdgs = pl.query(prologQuery.str());
    if(bdgs.begin() != bdgs.end()) {
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
