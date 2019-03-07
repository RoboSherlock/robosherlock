#ifndef __RSPROCESS_MANAGER_H__
#define __RSPROCESS_MANAGER_H__

#include <rs/flowcontrol/RSAggregateAnalysisEngine.h>

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
#include <rs/io/CamInterface.h>
#include <rs/feature_structure_proxy.h>
#include <rs/types/all_types.h>


#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <memory>

//neded for visualization
#include <tf_conversions/tf_eigen.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

//TODO: Make this the collection processing engine
class RSProcessManager
{

public:



  RSAggregateAnalysisEngine *engine_;
  std::shared_ptr<rs::KnowledgeEngine> knowledgeEngine_;
  QueryInterface *queryInterface;

  CamInterface *cameras;


  rs::KnowledgeEngine::KnowledgeEngineType ke_type_;

  //this is needed for legacy mongo interface
  mongo::client::GlobalInstance instance;

  //ROS interface related members
  ros::NodeHandle nh_;
  ros::ServiceServer setContextService_, queryService_, setFlowService_;
  ros::Publisher result_pub_;
  ros::Publisher pc_pub_;
  image_transport::Publisher image_pub_;
  image_transport::ImageTransport it_;


  bool waitForServiceCall_;
  bool useVisualizer_;
  bool use_identity_resolution_;
  bool parallel_;

  std::mutex processing_mutex_;

  rs::Visualizer visualizer_;
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
                   rs::KnowledgeEngine::KnowledgeEngineType keType,
                   ros::NodeHandle n, const std::string &savePath);

  /**
    * @brief destructor
    */
  ~RSProcessManager();


  /**
   * @brief RSProcessManager::init initialize the RSProcessManager; The engine and all of it's components need initialization; This method does that;
   * without initialization you can not use the algos in the engine;
   * @param engineFile engine file to load
   * @param pervasive flag to run in pervasive mode; (overrides waitForService)
   * @param parallel flag for parallelizing execution based on I/O definitions of annotators
   */
  void init(std::string &xmlFile, bool pervasive, bool parallel);

  /**
   * @brief RSProcessManager::run run the pipeline active pipeline defined in the engine
   * in a continuous loop;
   */
  void run();


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
  void setUseIdentityResolution(bool useIdentityResoltuion)
  {
    use_identity_resolution_ = useIdentityResoltuion;
  }

  //draw results on an image
  template <class T>
  bool drawResultsOnImage(const std::vector<bool> &filter, const std::vector<std::string> &resultDesignators,
                          std::string &requestJson, cv::Mat &resImage);

  template <class T>
  bool highlightResultsInCloud(const std::vector<bool> &filter, const std::vector<std::string> &resultDesignators,
                               std::string &requestJson, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

  static void signalHandler(int signum)
  {
    outWarn("Interrupt signal " << signum << " recevied. Exiting!");
    exit(signum);
  }

};

#endif // __RSPROCESS_MANAGER_H__
