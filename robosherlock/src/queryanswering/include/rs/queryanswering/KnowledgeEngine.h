#ifndef KNOWLEDGEENGINE_H
#define KNOWLEDGEENGINE_H

#include <vector>
#include <string>
#include <map>
#include <rs/utils/common.h>
#include <mutex>
#include <boost/algorithm/string.hpp>

namespace rs
{

//LIST OF rdf namespaces that we will never have objects ar perception entities defined under;
static const std::vector<std::string> NS_TO_SKIP = {"rdf", "rdfs", "owl", "xsd", "dc", "dcterms", "eor", "skos", "foaf", "void", "serql", "swrl", "swrla"};

/**
 * @brief The KnowledgeEngine class unified abstract interface for switching between
 * internal prolog engine and KnowRob's json_prolog interface (for now)
 */
class KnowledgeEngine
{

public:
  KnowledgeEngine() {}

  ~KnowledgeEngine() {}

  /**
   * @brief planPipelineQuery
   * @param keys vector of keys extracted from query
   * @param pipeline vector of annotator names forming the pipeline
   * @return true on success
   */
  virtual bool planPipelineQuery(const std::vector<std::string> &keys,
                                 std::vector<std::string> &pipeline) = 0;

  /**
   * @brief q_subClassOf
   * @param child name of child
   * @param parent name of parent
   * @return
   */
  virtual bool q_subClassOf(std::string child, std::string parent) = 0;

  /**
   * @brief checkValidQueryTerm verify if term of query language has been defined in KB
   * @param term term to verify
   * @return true if defined
   */
  virtual bool checkValidQueryTerm(const std::string &term) = 0;

  /**
   * @brief assertValueForKey assert a key value pair to the knowledge base
   * @param key
   * @param value
   * @return true on success
   */
  virtual bool assertValueForKey(const  std::string &key, const std::string &value) = 0;

  virtual bool retractQueryKvPs() = 0;

  virtual bool retractQueryLanguage() = 0;

  virtual bool assertQueryLanguage(std::map <std::string, std::vector<std::string>> &) = 0;

  virtual bool addNamespace(std::string &) = 0;

  /**
   * @brief addNamespace alternative implementation of adding a namespace
   * @param kb entry entry to add namespace to
   * @param results resulting namespaced kb entry
   * @return true if success false if entry was not found under any of known namespaces
   */
  virtual bool addNamespace(const std::string &entry, std::string &results)
  {
    results = entry;
    return addNamespace(results);
  }

  virtual bool retractAllAnnotators() = 0;

  virtual bool individualOf(const std::string &, std::vector<std::string> &) = 0;

  virtual bool assertInputTypeConstraint(const std::string &individual, const std::vector<std::string>& values, std::string& type)=0;

  virtual bool assertOutputTypeRestriction(const std::string &individual, const std::vector<std::string>& values, std::string& type)=0;

  bool assertAnnotatorMetaInfo(std::pair<std::string, rs::AnnotatorCapabilities> annotatorData, std::string individualOfAnnotator)
  {
    std::map<std::string, std::vector<std::string>> inputRestrictions = annotatorData.second.iTypeValueRestrictions;
    std::map<std::string, std::vector<std::string>> outputDomains = annotatorData.second.oTypeValueDomains;

    if(!inputRestrictions.empty())
    {
      for(auto iR : inputRestrictions)
      {
        if(!iR.second.empty())
        {
          std::vector<std::string> inputTypeConstraintInKnowRob;
          for(auto d : iR.second)
          {
            d[0] = std::toupper(d[0]);
            if(!addNamespace(d))
            {
              outWarn("output domain element: [ " << d << " ] is not defined in Ontology. Will not be considered durin query answering");
              continue;
            }
            outInfo(d);
            inputTypeConstraintInKnowRob.push_back(d);
          }

          std::string typeName = iR.first, typeClass;
          std::vector<std::string> typeSplit;
          boost::algorithm::split(typeSplit, typeName, boost::is_any_of("."), boost::algorithm::token_compress_on);
          for(auto &t : typeSplit)
          {
            std::transform(t.begin(), t.end(), t.begin(), ::tolower);
            t[0] = std::toupper(t[0]);
            typeClass.append(t);
          }
          outInfo(typeName << ":" << typeClass);
          addNamespace(typeClass);
          assertInputTypeConstraint(individualOfAnnotator, inputTypeConstraintInKnowRob, typeClass);
        }
      }
    }

    if(!outputDomains.empty())
    {
      for(auto oD : outputDomains)
      {
        if(!oD.second.empty())
        {
          std::vector<std::string> resultDomainInKnowRob;
          for(auto d : oD.second)
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

          std::string typeName = oD.first, typeClass;
          std::vector<std::string> typeSplit;
          boost::algorithm::split(typeSplit, typeName, boost::is_any_of("."), boost::algorithm::token_compress_on);
          for(auto &t : typeSplit)
          {
            std::transform(t.begin(), t.end(), t.begin(), ::tolower);
            t[0] = std::toupper(t[0]);
            typeClass.append(t);
          }
          outInfo(typeName << ":" << typeClass);
          addNamespace(typeClass);
          assertOutputTypeRestriction(individualOfAnnotator, resultDomainInKnowRob, typeClass);
        }
      }
    }
    return true;
  }

  bool assertAnnotators(const std::map<std::string, rs::AnnotatorCapabilities> &caps)
  {
    outInfo("Asserting annotators to KB");
    for(const std::pair<std::string, rs::AnnotatorCapabilities> &a : caps)
    {
      std::string nameWithNS;
      if(addNamespace(a.first, nameWithNS))
      {
        std::vector<std::string> individualsOf;
        individualOf(nameWithNS, individualsOf);
        for(auto i : individualsOf)
        {
          outInfo(i);
          assertAnnotatorMetaInfo(a, i);
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

};

}

#endif
