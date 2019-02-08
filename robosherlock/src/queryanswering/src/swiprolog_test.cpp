#include <iostream>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <SWI-Prolog.h>
#include <SWI-cpp.h>
#include <memory>
#include <mutex>
#include <ros/package.h>
#include <algorithm>


class PrologEngine
{
  std::mutex lock;


public:
//  typedef std::shared_ptr<PlEngine> PlEnginePtr;

  std::shared_ptr<PlEngine> engine;
  ros::ServiceServer srv_;
  ros::NodeHandle *nh_;

  PrologEngine()
  {
    char *argv[4];
    int argc = 0;
    argv[argc++] = "PrologEngine";
    argv[argc++] = "-s";
    std::string rosPrologInit = ros::package::getPath("robosherlock") + "/prolog/init.pl";
    argv[argc] = new char[rosPrologInit.size() + 1];
    std::copy(rosPrologInit.begin(), rosPrologInit.end(), argv[argc]);
    argv[argc++][rosPrologInit.size()] = '\0';
    argv[argc] = NULL;
    engine  = std::make_shared<PlEngine>(argc, argv);

    nh_ = new ros::NodeHandle("ROS_SWIPL");

    srv_ = nh_->advertiseService("trigger", &PrologEngine::trigger_service_cb_, this);
  }

  bool trigger_service_cb_(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    simple_query();
    res.success = true;
    res.message = "Trigger successfull";
    return true;
  }

  void simple_query()
  {
    lock.lock();
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
    lock.unlock();
  }

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
    while(q.next_solution())
    {
      //      std::cerr<<(char*)av[1]<<std::endl;
      PlTail res(av[1]);//result is a list
      PlTerm e;//elements of that list
      while(res.next(e))
      {
        pipeline.push_back(std::string((char *)e));
      }
    }
  }

  static void subClassOf(std::string child, std::string parent)
  {
    PlTermv av(2);
    av[0] = child.c_str();
    av[1] = parent.c_str();
    try
    {
      if(PlCall("owl_subclass_of", av))
      {
        std::cerr << child << " is subclass of " << parent << std::endl;
      }
      else
      {
        std::cerr << child << " is NOT subclass of " << parent << std::endl;
      }
    }
    catch(PlException &ex)
    {
      std::cerr << (char *)ex << std::endl;
    }
  }
};



int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_swi");
    PrologEngine pe;

  std::vector<std::string> plannedPipeline;
  ros::spin();
//  pe.planPipelineQuery(std::vector<std::string> {"shape"}, plannedPipeline);
//  std::for_each(plannedPipeline.begin(), plannedPipeline.end(), [](std::string & p)
//  {
//    std::cerr << p << std::endl;
//  });

//  pe.subClassOf("toaster", "Container");
  return 0;
}


//#include "SWI-cpp.h"
//#include <iostream>
//using namespace std;

///* Usage:
//   likes      prints who likes what
//   likes x      prints what is liked by x
//   likes x y      Test whether x likes y
//   likes -happy     See who is happy
//   Compile using:
//   plld -o likes -ld g++ -goal true likes.cpp likes.pl
//*/

//int
//body(int argc, char **argv)
//{
//  if(argc == 1)
//  {
//    if(strcmp(argv[0], "-happy") == 0)
//    {
//      PlTermv av(1);      /* likes - happy */

//      cout << "Happy people:" << endl;
//      PlQuery q("happy", av);
//      while(q.next_solution())
//      {
//        cout << "\t" << (char *)av[0] << endl;
//      }
//    }
//    else
//    {
//      PlTermv av(2);

//      cout << argv[0] << " likes:" << endl;
//      av[0] = argv[0];
//      PlQuery q("likes", av);
//      while(q.next_solution())
//      {
//        cout << "\t" << (char *)av[1] << endl;
//      }
//    }
//  }
//  else if(argc == 2)
//  {
//    PlTermv av(2);

//    av[0] = argv[0];
//    av[1] = argv[1];
//    if(PlCall("likes", av))
//    {
//      cout << "yes" << endl;
//    }
//    else
//    {
//      cout << "no" << endl;
//    }
//  }
//  else
//  {
//    cout << "Usage: likes x [y] or likes -happy" << endl;
//  }

//  return 0;
//}


//int
//main(int argc, char **argv)
//{
//  char *av[4];
//  int ac;
//  av[ac++] = argv[0];
//  av[ac++] = "-f";
//  av[ac++] = "likes.pl";
//  av[ac] = NULL;

//  PlEngine e(ac ,av);

//  try
//  {
//    return body(argc - 1, argv + 1);
//  }
//  catch(PlException &ex)
//  {
//    cerr << (char *) ex << endl;
//    exit(1);
//  }
//}

