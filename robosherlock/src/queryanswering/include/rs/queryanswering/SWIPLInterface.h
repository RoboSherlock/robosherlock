#ifndef SWIPLINTERFACE_H
#define SWIPLINTERFACE_H

//robosherlock
#include <rs/utils/output.h>

//ROS
#include <ros/package.h>

//STD
#include <memory>
#include <mutex>

#include <rs/queryanswering/KnowledgeEngine.h>

//SWI Prolog
#include <SWI-cpp.h>

namespace rs
{

//wrapper class for Prolog Engine based on SWI-C++
class SWIPLInterface : public KnowledgeEngine
{
  std::mutex lock_;
  std::shared_ptr<PlEngine> engine_;
public:

  SWIPLInterface();

  ~SWIPLInterface()
  {
  }

  /*
   * in: vector of keys extracted from query
   * out: vector of annotator names forming the pipeline
   */
  bool planPipelineQuery(const std::vector<std::string> &keys,
                         std::vector<std::string> &pipeline);


  bool q_subClassOf(std::string child, std::string parent);

  bool checkValidQueryTerm(const std::string &term)
  {
    std::lock_guard<std::mutex> lock(lock_);
    outInfo("Checking validity of term");
    return true;
  }

  bool assertValueForKey(const  std::string &key, const std::string &value)
  {
      std::lock_guard<std::mutex> lock(lock_);
    outInfo("Asserting value for key");
    return true;
  }

  bool retractQueryKvPs()
  {
      std::lock_guard<std::mutex> lock(lock_);
    outInfo("Retracting all query KvPs");
    PlTermv av(2);
    return true;
  }

  bool retractQueryLanguage()
  {
      std::lock_guard<std::mutex> lock(lock_);
    outInfo("Retracting Query language");
    return true;
  }

  bool retractAllAnnotators()
  {
      std::lock_guard<std::mutex> lock(lock_);
    outInfo("Retracting Query language");
    return true;
  }


  bool assertQueryLanguage(std::map <std::string, std::vector<std::string>> &)
  {
      std::lock_guard<std::mutex> lock(lock_);
    outInfo("Asserting query language specific knowledge");
    PlTermv av(0);
    PlTermv av2(1);
    try
    {
      PlQuery q("assert_query_lang", av);
      q.next_solution();
      PlQuery q2("rs_query_reasoning", "rs_query_predicate", av2);
      while(q2.next_solution())
      {
        std::cerr << static_cast<char *>(av2[0]) << std::endl;
      }
    }
    catch(PlException &ex)
    {
      std::cerr << (char *)ex << std::endl;
    }
    return true;
  }

  bool addNamespace(std::string &s)
  {
      std::lock_guard<std::mutex> lock(lock_);
    outInfo("Adding namespace to: " << s);
    return true;
  }

  bool assertAnnotators(const std::map < std::string, rs::AnnotatorCapabilities> &caps)
  {
      std::lock_guard<std::mutex> lock(lock_);
    outInfo("ASSERTING ANNOTATORS TO KB");
    return true;
  }

  void simple_query()
  {
      std::lock_guard<std::mutex> lock(lock_);
    PlTermv av(0);
    PlTermv av2(1);
    try
    {
      PlQuery q("assert_query_lang", av);
      q.next_solution();
      PlQuery q2("rs_query_reasoning","rs_query_predicate",av2);
      while(q2.next_solution())
      {
        std::cerr << static_cast<char *>(av2[0]) << std::endl;
      }
    }
    catch(PlException &ex)
    {
      std::cerr << (char *)ex << std::endl;
    }
  }




};
}
#endif //SWIPLINTERFACE_H


