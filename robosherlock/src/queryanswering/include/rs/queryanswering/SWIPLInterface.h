#ifndef SWIPLINTERFACE_H
#define SWIPLINTERFACE_H

// robosherlock
#include <rs/utils/output.h>

// ROS
#include <ros/package.h>

// STD
#include <memory>
#include <mutex>

#include <rs/queryanswering/KnowledgeEngine.h>

// SWI Prolog
#include <SWI-cpp.h>
#include <SWI-Prolog.h>

namespace rs
{
// wrapper class for Prolog Engine based on SWI-C++
class SWIPLInterface : public KnowledgeEngine
{
  std::mutex lock_;
  PL_engine_t engine1_, engine2_;
  PL_thread_attr_t attributes_;

public:
  SWIPLInterface();

  ~SWIPLInterface()
  {
    PL_destroy_engine(engine1_);
  }

  /**
   * @brief setEnginge for multi-threading SWI-prolog needs to attach an engine to each thread; call this when executing
   * from a thread; this is a hack for now; this shoudl happen inside each query;
   */
  void setEngine()
  {
    PL_engine_t engine;
    int result = PL_set_engine(PL_ENGINE_CURRENT, &engine);
    outError("PL_CURRENT_ENGINE: " << engine);
    outError("ENGINE1_: " << engine1_);
    if (engine == 0)
    {
      engine1_ = PL_create_engine(&attributes_);
      result = PL_set_engine(engine1_, &engine);
      if (result != PL_ENGINE_SET)
      {
        if (result == PL_ENGINE_INVAL)
          throw std::exception("Engine is invalid.");
        else if (result == PL_ENGINE_INUSE)
          throw std::exception("Engine is invalid.");
        else
          throw std::exception("Unknown Response when setting PL_engine");
      }
      else
      {
        outError("Set: " << engine1_ << "As active engine");
      }
    }
  }
  /*
   * in: vector of keys extracted from query
   * out: vector of annotator names forming the pipeline
   */
  bool planPipelineQuery(const std::vector<std::string>& keys, std::vector<std::string>& pipeline);

  bool q_subClassOf(std::string child, std::string parent);

  bool checkValidQueryTerm(const std::string& term)
  {
    std::lock_guard<std::mutex> lock(lock_);
    outInfo("Checking validity of term");
    setEngine();
    return true;
  }

  bool assertValueForKey(const std::string& key, const std::string& value)
  {
    std::lock_guard<std::mutex> lock(lock_);
    outInfo("Asserting value [" << value << "] for key [" << key << "]");
    setEngine();
    PlTermv av(1);
    return true;
  }

  bool retractQueryKvPs()
  {
    std::lock_guard<std::mutex> lock(lock_);
    outInfo("Retracting all query KvPs");
    setEngine();
    PlTerm t;
    return true;
  }

  bool retractQueryLanguage()
  {
    std::lock_guard<std::mutex> lock(lock_);
    outInfo("Retracting Query language");
    setEngine();
    return true;
  }

  bool retractAllAnnotators()
  {
    std::lock_guard<std::mutex> lock(lock_);
    outInfo("Retracting all annotators");
    setEngine();
    return true;
  }

  bool assertQueryLanguage(std::map<std::string, std::vector<std::string>>&)
  {
    std::lock_guard<std::mutex> lock(lock_);
    outInfo("Asserting query language specific knowledge");
    setEngine();
    PlTermv av(0), av2(1);
    try
    {
      PlQuery q("assert_query_lang", av);
      q.next_solution();
      PlQuery q2("rs_query_reasoning", "rs_query_predicate", av2);
      while (q2.next_solution())
      {
        std::cerr << static_cast<char*>(av2[0]) << std::endl;
      }
    }
    catch (PlException& ex)
    {
      std::cerr << (char*)ex << std::endl;
    }
    return true;
  }

  bool addNamespace(std::string& s)
  {
    std::lock_guard<std::mutex> lock(lock_);
    outInfo("Adding namespace to: " << s);
    setEngine();
    return true;
  }

  bool assertAnnotators(const std::map<std::string, rs::AnnotatorCapabilities>& caps)
  {
    std::lock_guard<std::mutex> lock(lock_);
    outInfo("ASSERTING ANNOTATORS TO KB");
    setEngine();
    return true;
  }

  void simple_query()
  {
    std::lock_guard<std::mutex> lock(lock_);
    setEngine();

    PlTermv av(0);
    PlTermv av2(1);
    try
    {
      PlQuery q("assert_query_lang", av);
      q.next_solution();
      PlQuery q2("rs_query_reasoning", "rs_query_predicate", av2);
      while (q2.next_solution())
      {
        std::cerr << static_cast<char*>(av2[0]) << std::endl;
      }
    }
    catch (PlException& ex)
    {
      std::cerr << (char*)ex << std::endl;
    }
  }
};
}  // namespace rs
#endif  // SWIPLINTERFACE_H
