#ifndef RSCONTROLEDAEMANAGER_H
#define RSCONTROLEDAEMANAGER_H

#include <rs/flowcontrol/RSAnalysisEngineManager.h>

#include <rs/flowcontrol/RSControledAnalysisEngine.h>
#include <rs/queryanswering/KRDefinitions.h>

#ifdef WITH_JSON_PROLOG
#include <rs/queryanswering/QueryInterface.h>
#endif

#include <rs/queryanswering/DesignatorWrapper.h>

#include <iai_robosherlock_msgs/SetRSContext.h>
#include <iai_robosherlock_msgs/RSQueryService.h>


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
  #endif

  ros::NodeHandle nh_;
  ros::Publisher desig_pub_;
  ros::ServiceServer service, singleService, setContextService, jsonService;


  bool waitForServiceCall_;
  const bool useVisualizer_;

  bool withJsonProlog_;
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


  bool resetAECallback(iai_robosherlock_msgs::SetRSContext::Request &req,
                       iai_robosherlock_msgs::SetRSContext::Response &res);
#ifdef WITH_JSON_PROLOG

  bool jsonQueryCallback(iai_robosherlock_msgs::RSQueryService::Request &req,
                         iai_robosherlock_msgs::RSQueryService::Response &res);

  virtual bool handleQuery(std::string& req, std::vector<std::string>& res);
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

  inline void setUseJsonPrologInterface(bool useJsonProlog)
  {
    withJsonProlog_ = useJsonProlog;
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

#endif // RSCONTROLEDAEMANAGER_H
