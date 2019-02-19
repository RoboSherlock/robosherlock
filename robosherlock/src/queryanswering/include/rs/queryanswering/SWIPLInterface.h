#ifndef SWIPLINTERFACE_H
#define SWIPLINTERFACE_H

// robosherlock
#include <rs/utils/output.h>
#include <rs/utils/time.h>
// ROS
#include <ros/package.h>

// STD
#include <memory>
#include <mutex>

#include <rs/queryanswering/KnowledgeEngine.h>

// SWI Prolog
#include <SWI-cpp.h>
#include <SWI-Prolog.h>

#define RDF_TYPE "'http://www.w3.org/1999/02/22-rdf-syntax-ns#type'"

namespace rs
{
// wrapper class for Prolog Engine based on SWI-C++
class SWIPLInterface : public KnowledgeEngine
{
  std::mutex lock_;
  PL_engine_t engine1_;
  PL_thread_attr_t attributes_;

  std::set<PL_engine_t> engine_pool_;

  std::vector<std::string> krNamespaces_;


  //TODO: we probably need an Engine and a Query class
  //  class Engine
  //  {
  //    PL_engine_t engine_;
  //    std::string name_;
  //    bool acquire(){

  //    }
  //    bool release(){

  //    }
  //  };

public:
  SWIPLInterface();

  ~SWIPLInterface()
  {
    for(void *a : engine_pool_)
    {
      PL_destroy_engine(a);
    }

  }

  /**
   * @brief setEnginge for multi-threading SWI-prolog needs to attach an engine to each thread; call this when executing
   * from a thread; this is a hack for now; this shoudl happen inside each query;
   */
  void setEngine();

  /**
   * @brief for future reference: we could release the engine at the end of each query;
   * this way we only need a finita amount of engines, otherwise we get as many
   * engines as there are threads calling queries;
   *
   * An alternative solution to this is to just create as many engines as there are threads calling this
   * As a ROS node there will not be more queries than threads that spin;
   *
   * NOTE: releaseing an Engine before the destrutcion of a PlQuery object can cause nasty segfaults so
   * treat with care and we should wrap around this;
   */
  void releaseEngine()
  {
    PL_set_engine(0, 0);
  }

  /**
   * @brief
   * @param keys vector of keys extracted from query
   * @param pipeline vector of annotator names forming the pipeline
   */
  bool planPipelineQuery(const std::vector<std::string> &keys, std::vector<std::string> &pipeline);

  bool q_subClassOf(std::string child, std::string parent);

  bool checkValidQueryTerm(const std::string &term);

  bool assertValueForKey(const std::string &key, const std::string &value);

  bool retractQueryKvPs();

  bool individualOf(const std::string &class_name, std::vector<std::string> &individualsOF);

  bool assertInputTypeConstraint(const std::string &individual, const std::vector<std::string> &values, std::string &type);

  bool assertOutputTypeRestriction(const std::string &individual, const std::vector<std::string> &values, std::string &type);

  bool retractQueryLanguage();

  bool retractAllAnnotators();

  bool assertQueryLanguage(std::map<std::string, std::vector<std::string>> &);

  bool addNamespace(std::string &s);

  bool assertAnnotators(const std::map<std::string, rs::AnnotatorCapabilities> &caps);

  bool assertTestPipelnie()
  {
    setEngine();
    try
    {
      if(!PlCall("rs_query_reasoning:assert_query_lang"))
        return false;
      if(!PlCall("rs_query_reasoning:assert_test_pipeline"))
        return false;
      outInfo("Asserted query lang and pipeline for testing");
    }
    catch(PlException &ex)
    {
      outError(static_cast<char *>(ex));
    }
    return true;
  }

};
}  // namespace rs
#endif  // SWIPLINTERFACE_H
