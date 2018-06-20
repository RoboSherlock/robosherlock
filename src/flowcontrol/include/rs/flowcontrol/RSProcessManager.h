#ifndef __RSPROCESS_MANAGER_H__
#define __RSPROCESS_MANAGER_H__

#include <rs/flowcontrol/RSAnalysisEngineManager.h>

#include <rs/flowcontrol/RSControledAnalysisEngine.h>
#include <rs/queryanswering/KRDefinitions.h>

#ifdef WITH_JSON_PROLOG
#include <rs/flowcontrol/RSParallelPipelinePlanner.h>
#include <rs/queryanswering/QueryInterface.h>
#endif

#include <rs/queryanswering/DesignatorWrapper.h>

#include <robosherlock_msgs/SetRSContext.h>
#include <robosherlock_msgs/RSQueryService.h>

#include <mongo/client/dbclient.h>

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/prettywriter.h>

#include <std_srvs/Trigger.h>

//TODO: Make this the ROS communication interface class
class RSProcessManager
{

public:

  RSControledAnalysisEngine engine_;
  RSControledAnalysisEngine inspectionEngine_;
  #ifdef WITH_JSON_PROLOG
  QueryInterface *queryInterface;
  RSParallelPipelinePlanner parallelPlanner_;
  #endif

  mongo::client::GlobalInstance instance;


  ros::NodeHandle nh_;
  ros::ServiceServer service, singleService, setContextService, jsonService;


  bool waitForServiceCall_;
  const bool useVisualizer_;

  bool useIdentityResolution_;
  bool pause_;
  bool inspectFromAR_;


  std::mutex processing_mutex_;

  rs::Visualizer visualizer_;

  std::string configFile_;
  std::vector<std::string> lowLvlPipeline_;

  RSProcessManager(const bool useVisualizer, const bool &waitForServiceCall,
                   ros::NodeHandle n, const std::string& savePath);

  ~RSProcessManager();

  void init(std::string &xmlFile, std::string configFile_, bool pervasive);

  void run();

  void stop();


  bool resetAECallback(robosherlock_msgs::SetRSContext::Request &req,
                       robosherlock_msgs::SetRSContext::Response &res);
#ifdef WITH_JSON_PROLOG

  bool jsonQueryCallback(robosherlock_msgs::RSQueryService::Request &req,
                         robosherlock_msgs::RSQueryService::Response &res);

  bool handleQuery(std::string& req, std::vector<std::string>& res);
#endif
  //special case for offscreen rendering the beliefstate using Unreal Engine
  bool renderOffscreen(std::string object);

  //reset the pipeline in the AE;
  bool resetAE(std::string);

  //set the AE for inspection tasks (allows for a different parametrization of components)
  void setInspectionAE(std::string);


  inline void setUseIdentityResolution(bool useIdentityResoltuion)
  {
    useIdentityResolution_ = useIdentityResoltuion;
    engine_.useIdentityResolution(useIdentityResoltuion);
  }

  inline void setInspectFromAR(bool b)
  {
    inspectFromAR_ = b;
  }

  inline std::string getEngineName()
  {
    return engine_.getCurrentAEName();
  }

  inline void pause()
  {
    processing_mutex_.lock();
    pause_ = !pause_;
    processing_mutex_.unlock();
  }

  inline void setLowLvlPipeline(std::vector<std::string> llp)
  {
    lowLvlPipeline_.assign(llp.begin(), llp.end());
  }


};

#endif // __RSPROCESS_MANAGER_H__
