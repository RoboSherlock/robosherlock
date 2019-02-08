#ifndef SWIPLINTERFACE_H
#define SWIPLINTERFACE_H

//robosherlock
#include <rs/utils/output.h>

//ROS
#include <ros/package.h>

//SWI Prolog
#include <SWI-cpp.h>

//STD
#include <memory>

//wrapper class for Prolog Engine based on SWI-C++
class SWIPLInterface
{

//  typedef std::unique_ptr<PlEngine> PlEngineUPtr;
  PlEngine* engine_;
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

  bool checkValidQueryTerm(const std::string &term) {
      outInfo("Checking validity of term");

  }

  bool assertValueForKey(const  std::string &key, const std::string &value) {
      outInfo("Asserting value for key");

  }

  bool retractQueryKvPs()
  {
    outInfo("Retracting all query KvPs");
  }

  bool retractQueryLanguage()
  {
    outInfo("Retracting Query language");
  }

  bool assertQueryLanguage(std::map <std::string, std::vector<std::string>> &) {
      outInfo("Asserting query language specific knowledge");
  }

  bool addNamespace(std::string &s) {
      outInfo("Adding namespace to: "<<s);
  }


};

#endif //SWIPLINTERFACE_H


