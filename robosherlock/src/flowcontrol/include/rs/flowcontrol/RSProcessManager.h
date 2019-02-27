#ifndef __RSPROCESS_MANAGER_H__
#define __RSPROCESS_MANAGER_H__

#include <rs/flowcontrol/RSAnalysisEngine.h>

#include <rs/io/visualizer.h>

#include <robosherlock_msgs/SetRSContext.h>
#include <robosherlock_msgs/RSQueryService.h>
#include <robosherlock_msgs/RSObjectDescriptions.h>
#include <robosherlock_msgs/RSVisControl.h>
#include <robosherlock_msgs/ExecutePipeline.h>

#include <mongo/client/dbclient.h>

#include <rs/queryanswering/QueryInterface.h>
#include <rs/queryanswering/SWIPLInterface.h>
#include <rs/queryanswering/JsonPrologInterface.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl_ros/point_cloud.h>
#include <memory>

//TODO: Make this the ROS communication interface class
class RSProcessManager
{

public:

  RSAnalysisEngine engine_;

  enum class KnowledgeEngineType {JSON_PROLOG, SWI_PROLOG};


  KnowledgeEngineType ke_type_;
  QueryInterface *queryInterface;
  std::shared_ptr<rs::KnowledgeEngine> knowledgeEngine_;

  mongo::client::GlobalInstance instance;

  ros::NodeHandle nh_;
  ros::ServiceServer setContextService_, queryService_, visService_, setFlowService_;

  ros::Publisher result_pub;
  ros::Publisher pc_pub_;
  image_transport::Publisher image_pub_;
  image_transport::ImageTransport it_;


  bool waitForServiceCall_;
  bool useVisualizer_;
  bool useIdentityResolution_;
  bool parallel_;

  std::mutex processing_mutex_;

  rs::Visualizer visualizer_;

  std::string configFile_;
  std::vector<std::string> lowLvlPipeline_;


  /**
   * @brief RSProcessManager::RSProcessManager constructror: the one and only...for now
   * @param useVisualizer flag for starting visualization window; If false it runs in headless mode, advertising partial results on a topic
   * @param waitForServiceCall run engine in synchroniouse mode, waiting for queries to arrive
   * @param keType set the knowledge Engine you would like to use. options are knowrob (JSON_PROLOG) or SWI_PROLOG
   * @param n ros NodeHandle
   * @param savePath path where to save images to from visualizer to; if emtpy iages are saved to working dir;
   */
  RSProcessManager(const bool useVisualizer, const bool &waitForServiceCall,
                   RSProcessManager::KnowledgeEngineType keType,
                   ros::NodeHandle n, const std::string &savePath);

  /**
    * @brief destructor
    */
  ~RSProcessManager();


  /**
   * @brief RSProcessManager::init initialize the RSProcessManager; The engine and all of it's components need initialization; This method does that;
   * without initialization you can not use the algos in the engine;
   * @param engineFile engine file to load
   * @param configFile extra config file; Deprecated; will be removed
   * @param pervasive flag to run in pervasive mode; (overrides waitForService)
   * @param parallel flag for parallelizing execution based on I/O definitions of annotators
   */
  void init(std::string &xmlFile, std::string configFile_, bool pervasive, bool parallel);

  /**
   * @brief RSProcessManager::run run the pipeline active pipeline defined in the engine
   * in a continuous loop;
   */
  void run();

  void stop();


  /**
   * @brief RSProcessManager::resetAECallback callback funciton for resetting the engine;
   * @param req name of new AAE file
   * @param res no results specified
   * @return true or false
   */
  bool resetAECallback(robosherlock_msgs::SetRSContext::Request &req,
                       robosherlock_msgs::SetRSContext::Response &res);

  bool visControlCallback(robosherlock_msgs::RSVisControl::Request &req,
                          robosherlock_msgs::RSVisControl::Response &res);

  /**
   * @brief RSProcessManager::executePipelineCallback execute a s
   * equence of annotators given in the request message;This callback assumes a complete list of components;
   * @param req request containing list of annotators to execute;
   * @param res object descriptions as json;
   * @return true if successfull
   */
  bool executePipelineCallback(robosherlock_msgs::ExecutePipeline::Request &req,
                               robosherlock_msgs::ExecutePipeline::Response &res);

  /**
   * @brief RSProcessManager::jsonQueryCallback the callback function
   * @param req request string (json formated nested key-value pairs)
   * @param res response array of json descriptions
   * @return
   */
  bool jsonQueryCallback(robosherlock_msgs::RSQueryService::Request &req,
                         robosherlock_msgs::RSQueryService::Response &res);

  /**
   * @brief RSProcessManager::handleQuery
   * @param request query in a json format
   * @param result object descriptions in a Json format
   * @return
   */
  bool virtual handleQuery(std::string &req, std::vector<std::string> &res);


  /**
   * @brief RSProcessManager::resetAE reset analysis engine that was instantiated; Use this method i
   * if you want to change the AAE loaded at startup
   * @param newAAEName name of the new aggregate analysis engine;
   * @return true/false
   */
  bool resetAE(std::string);

  /**
   * @brief setUseIdentityResolution run identiy resolution for tracking objects over multiple scenes
   * @param useIdentityResoltuion
   */
  inline void setUseIdentityResolution(bool useIdentityResoltuion)
  {
    useIdentityResolution_ = useIdentityResoltuion;
    engine_.useIdentityResolution(useIdentityResoltuion);
  }

  /**
   * @brief getEngineName
   * @return return the name of the loaded Aggregate analysis engine
   */
  inline std::string getEngineName()
  {
    return engine_.getCurrentAEName();
  }

  static void signalHandler(int signum)
  {
    outWarn("Interrupt signal "<< signum <<" recevied. Exiting!");
    exit(signum);
  }

};

#endif // __RSPROCESS_MANAGER_H__
