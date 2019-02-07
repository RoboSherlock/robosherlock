#ifndef KNOWLEDGEENGINE_H
#define KNOWLEDGEENGINE_H

#include <vector>
#include <string>
#include <map>

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

  ~KnowledgeEngine() {};

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


};

}

#endif
