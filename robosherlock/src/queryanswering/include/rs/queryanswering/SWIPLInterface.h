#ifndef SWIPLINTERFACE_H
#define SWIPLINTERFACE_H

//boost
#include <boost/algorithm/string.hpp>

//ros
#include <ros/package.h>

//robosherlock
#include <rs/utils/output.h>

//SWI Prolog
#include <SWI-cpp.h>

//STD
#include <memory>

//wrapper class for Prolog Engine based on SWI-C++
class PrologInterface
{

  typedef std::shared_ptr<PlEngine> PlEnginePtr;
  PlEnginePtr engine;

public:
  PrologInterface();
  ~PrologInterface()
  {
  }


  /*brief
   * initialize the necessary knowrob packages
   */
  void init();

  /*
   * in: vector of keys extracted from query
   * out: vector of annotator names forming the pipeline
   */
  static void planPipelineQuery(const std::vector<std::string> &keys,
                                std::vector<std::string> &pipeline)
  {
    PlTermv av(2);
    PlTail l(av[0]);
    for(auto key : keys)
    {
      l.append(key.c_str());
    }
    l.close();
    PlQuery q("build_single_pipeline_from_predicates", av);
    std::string prefix("http://knowrob.org/kb/rs_components.owl#");
    while(q.next_solution())
    {
      //      std::cerr<<(char*)av[1]<<std::endl;
      PlTail res(av[1]);//result is a list
      PlTerm e;//elements of that list
      while(res.next(e))
      {
          std::string element((char*)e);
          element.erase(0,prefix.length());
          pipeline.push_back(element);
      }
    }
  }

  /*brief
   * ask prolog if child is of type parent
   * */
  static bool subClassOf(std::string child, std::string parent)
  {
    PlTermv av(2);
    av[0] =  "child";
    av[1] =  "parent";
    try
    {
      if(PlCall("owl_subclass_of", av))
      {
        outInfo(child << " is subclass of " << parent );
        return true;
      }
      else
      {
        outInfo(child << " is NOT subclass of " << parent );
        return false;
      }
    }
    catch(PlException &ex)
    {
      outError((char *)ex);
      return false;
    }
    return false;
  }

  /*brief
   * check for a class property
   * */
  static bool class_property(std::string className, std::string property, std::string value)
  {
    PlTermv av(3);
    av[0] =  className.c_str();
    av[1] =  property.c_str();
    av[2] =  value.c_str();
    try
    {
      if(PlCall("class_properties", av))
      {
        outInfo(className << " " <<property <<" " <<value);
        return true;
      }
      else
      {
        outInfo(className << " has NO " <<property <<" " <<value);
        return false;
      }
    }
    catch(PlException &ex)
    {
      std::cerr << (char *)ex << std::endl;
      return false;
    }
    return false;
  }

  /*brief
   *extract the keys that serve as input for pipeline planning
   * in query as a designator
   * out vector of keys
   * */
  bool extractQueryKeysFromDesignator(std::string &desig,
                                      std::vector<std::string> &keys);

  /*brief
   * in desig: query as designator
   * out prologQuery: the Prolog Query as a string
   * returns true or false /success or fail
   * */
  bool buildPrologQueryFromDesignator(std::string &desig,
                                      std::string &prologQuery);

  /*brief
   * Create a vector of Annotator Names from the result of the knowrob_rs library.
   * This vector can be used as input for RSControledAnalysisEngine::setNextPipelineOrder
   */
  std::vector<std::string> createPipelineFromPrologResult(std::string result);

};

#endif //SWIPLINTERFACE_H


