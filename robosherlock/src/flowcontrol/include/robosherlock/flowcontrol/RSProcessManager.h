#ifndef __RSPROCESS_MANAGER_H__
#define __RSPROCESS_MANAGER_H__

#include <robosherlock/flowcontrol/RSAggregateAnalysisEngine.h>

#include <robosherlock/io/visualizer.h>

#include <robosherlock/scene_cas.h>

#include <robosherlock_msgs/SetRSContext.h>
#include <robosherlock_msgs/RSQueryService.h>
#include <robosherlock_msgs/RSObjectDescriptions.h>
#include <robosherlock_msgs/RSVisControl.h>
#include <robosherlock_msgs/ExecutePipeline.h>

#include <mongo/client/dbclient.h>

#include <robosherlock/queryanswering/QueryInterface.h>
#include <robosherlock/queryanswering/SWIPLInterface.h>
#include <robosherlock/queryanswering/RosPrologInterface.h>
#include <robosherlock/io/CollectionReader.h>

#include <robosherlock/feature_structure_proxy.h>
#include <robosherlock/types/all_types.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <memory>

// neded for visualization
#include <tf_conversions/tf_eigen.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

// forward class for action server(question answering)
class RSQueryActionServer;

// TODO: Make this the collection processing engine
class RSProcessManager
{
public:

  RSAggregateAnalysisEngine *engine_;
  std::shared_ptr<rs::KnowledgeEngine> knowledge_engine_;
  std::shared_ptr<QueryInterface> query_interface_;

  rs::KnowledgeEngine::KnowledgeEngineType ke_type_;

  // this is needed for legacy mongo interface
  mongo::client::GlobalInstance instance;

  // ROS interface related members
  ros::NodeHandle nh_;

  ros::AsyncSpinner spinner_;

  ros::ServiceServer setContextService_, queryService_, setFlowService_;
  ros::Publisher result_pub_;
  ros::Publisher pc_pub_;
  image_transport::Publisher image_pub_;
  image_transport::ImageTransport it_;

  // needed for sharing query processing's core function

  // action server for query answering
  RSQueryActionServer *actionServer;

  bool wait_for_service_call_;
  bool useVisualizer_;
  bool use_identity_resolution_;
  bool parallel_, pervasive_;

  std::mutex processing_mutex_;

  rs::Visualizer visualizer_;
  std::vector<std::string> lowLvlPipeline_;

  /**
   * @brief RSProcessManager::RSProcessManager constructror: the one and only...for now
   * @param aae_name name of the aggregate we want to initialize
   * @param useVisualizer flag for starting visualization window; If false it runs in headless mode, advertising partial
   * results on a topic
   * @param keType set the knowledge Engine you would like to use. options are knowrob (JSON_PROLOG) or SWI_PROLOG
   */
  RSProcessManager(std::string aae_name, const bool useVisualizer, rs::KnowledgeEngine::KnowledgeEngineType keType);

  /**
    * @brief destructor
    */
  ~RSProcessManager();

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
  bool resetAECallback(robosherlock_msgs::SetRSContext::Request &req, robosherlock_msgs::SetRSContext::Response &res);

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

  /**
   * @brief setWaitForService set the flag for waiting for a service call; if true, process of the AAE is only
   * exectued if a query is sent on the action or service interface
   * @param wait_for_service
   */
  void setWaitForService(bool wait_for_service)
  {
    wait_for_service_call_ = wait_for_service;
  }

  void setParallel(bool parallel)
  {
    parallel_ = parallel;
    engine_->setParallel(parallel_);
  }

  void setPervasive(bool pervasive)
  {
    this->pervasive_ = pervasive;
    if(pervasive)
    {
      visualizer_.setActiveAnnotators(lowLvlPipeline_);
      engine_->setContinuousPipelineOrder(lowLvlPipeline_);
      engine_->setPipelineOrdering(lowLvlPipeline_);
    }
  }

  // draw results on an image
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

#endif  // __RSPROCESS_MANAGER_H__
