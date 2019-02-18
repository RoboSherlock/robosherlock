#include <rs/flowcontrol/RSProcessManager.h>

RSProcessManager::RSProcessManager(const bool useVisualizer, const bool &waitForServiceCall, RSProcessManager::KnowledgeEngineType keType,
                                   ros::NodeHandle n, const std::string &savePath):
  engine_(), nh_(n), it_(nh_),
  waitForServiceCall_(waitForServiceCall),
  useVisualizer_(useVisualizer), useIdentityResolution_(false),
  visualizer_(savePath, !useVisualizer)
{

  signal(SIGINT, RSProcessManager::signalHandler);
  outInfo("Creating resource manager");
  uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");

  switch(OUT_LEVEL)
  {
  case OUT_LEVEL_NOOUT:
  case OUT_LEVEL_ERROR:
    resourceManager.setLoggingLevel(uima::LogStream::EnError);
    break;
  case OUT_LEVEL_INFO:
    resourceManager.setLoggingLevel(uima::LogStream::EnWarning);
    break;
  case OUT_LEVEL_DEBUG:
    resourceManager.setLoggingLevel(uima::LogStream::EnMessage);
    break;
  }

  //ROS interface declarations
  setContextService_ = nh_.advertiseService("set_context", &RSProcessManager::resetAECallback, this);
  visService_ = nh_.advertiseService("vis_command", &RSProcessManager::visControlCallback, this);
  setFlowService_ = nh_.advertiseService("execute_pipeline", &RSProcessManager::executePipelineCallback, this);

  result_pub = nh_.advertise<robosherlock_msgs::RSObjectDescriptions>(std::string("result_advertiser"), 1);
  image_pub_ = it_.advertise("result_image", 1, true);
  pc_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("points", 5);


  if(keType == KnowledgeEngineType::JSON_PROLOG)
  {
    outInfo("Setting KnowRob (through json prolog interface) as the knowledge engine.");
#if WITH_JSON_PROLOG
    if(ros::service::waitForService("json_prolog/simple_query", ros::Duration(60.0)))
      knowledgeEngine_ = std::make_shared<rs::JsonPrologInterface>();
    else
      throw rs::Exception("Json prolog not reachable");
#else
    throw rs::Exception("Json prolog was not found at compile time!");
#endif
  }
  else if(keType == KnowledgeEngineType::SWI_PROLOG)
  {
    knowledgeEngine_ = std::make_shared<rs::SWIPLInterface>();
  }
  else
  {
    outError("This can not be!");
    throw rs::Exception("Wrong initialization param for knowledge engine");
  }

  queryService_ = nh_.advertiseService("query", &RSProcessManager::jsonQueryCallback, this);
  queryInterface = new QueryInterface(knowledgeEngine_);
}

RSProcessManager::~RSProcessManager()
{
  uima::ResourceManager::deleteInstance();
  outInfo("RSControledAnalysisEngine Stoped");
}

void RSProcessManager::init(std::string &engineFile, std::string configFile, bool pervasive, bool parallel)
{
  outInfo("initializing");
  signal(SIGINT, RSProcessManager::signalHandler);

  this->configFile_ = configFile;
  try
  {
    cv::FileStorage fs(cv::String(configFile), cv::FileStorage::READ);

    if(lowLvlPipeline_.empty())   //if not set programatically, load from a config file
    {
      cv::FileNode n = fs["annotators"];
      if(n.type() != cv::FileNode::SEQ)
      {
        outError("Annotators missing from config file");
      }
      cv::FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
      for(; it != it_end; ++it)
      {
        lowLvlPipeline_.push_back(*it);
      }
    }
    fs.release();
  }
  catch(cv::Exception &e)
  {
    outWarn("No low-level pipeline defined. Setting empty!");
  }

  engine_.init(engineFile, parallel, pervasive , lowLvlPipeline_);

  knowledgeEngine_->retractAllAnnotators();
  knowledgeEngine_->assertAnnotators(engine_.getDelegateCapabilities());
  parallel_ = parallel;

  visualizer_.start();
  if(pervasive)
  {
    visualizer_.setActiveAnnotators(lowLvlPipeline_);
  }
  outInfo("done intializing");
}


void RSProcessManager::run()
{
  for(; ros::ok();)
  {
    signal(SIGINT, RSProcessManager::signalHandler);
    {
      std::lock_guard<std::mutex> lock(processing_mutex_);
      if(waitForServiceCall_)
      {
        usleep(100000);
      }
      else
      {
        std::vector<std::string> objDescriptions;
        engine_.process(objDescriptions, "");
        robosherlock_msgs::RSObjectDescriptions objDescr;
        objDescr.obj_descriptions = objDescriptions;
        result_pub.publish(objDescr);
      }
    }
    usleep(100000);
    ros::spinOnce();
  }
}


void RSProcessManager::stop()
{
  visualizer_.stop();
  engine_.resetCas();
  engine_.stop();
}


bool RSProcessManager::visControlCallback(robosherlock_msgs::RSVisControl::Request &req,
    robosherlock_msgs::RSVisControl::Response &res)
{
  std::string command = req.command;
  bool result = true;
  std::string activeAnnotator = "";
  if(command == "next")
  {
    activeAnnotator = visualizer_.nextAnnotator();

  }
  else if(command == "previous")
  {
    activeAnnotator = visualizer_.prevAnnotator();
  }
  else if(command != "")
  {
    activeAnnotator = visualizer_.selectAnnotator(command);
  }
  if(activeAnnotator == "")
    result = false;

  res.success = result;

  res.active_annotator = activeAnnotator;
  return result;
}


bool RSProcessManager::resetAECallback(robosherlock_msgs::SetRSContext::Request &req,
                                       robosherlock_msgs::SetRSContext::Response &res)
{
  std::string newContextName = req.newAe;

  if(resetAE(newContextName))
  {
    return true;
  }
  else
  {
    outError("Contexts need to have a an AE defined");
    outInfo("releasing lock");
    processing_mutex_.unlock();
    return false;
  }
}


bool RSProcessManager::resetAE(std::string newAAEName)
{
  std::string contextAEPath;
  if(rs::common::getAEPaths(newAAEName, contextAEPath))
  {
    outInfo("Setting new context: " << newAAEName);
    cv::FileStorage fs(cv::String(configFile_), cv::FileStorage::READ);

    cv::FileNode n = fs["annotators"];
    if(n.type() != cv::FileNode::SEQ)
    {
      outError("Somethings wrong with pipeline definition");
      return 1;
    }

    std::vector<std::string> lowLvlPipeline;
    cv::FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
    for(; it != it_end; ++it)
    {
      lowLvlPipeline.push_back(*it);
    }

    fs.release();

    {
      std::lock_guard<std::mutex> lock(processing_mutex_);
      this->init(contextAEPath, configFile_, false, parallel_);
    }

    return true;
  }
  else
  {
    return false;
  }
}

bool RSProcessManager::executePipelineCallback(robosherlock_msgs::ExecutePipeline::Request &req,
    robosherlock_msgs::ExecutePipeline::Response &res)
{
  if(req.annotators.empty())
  {
    return false;
  }

  std::vector<std::string> newPipelineOrder = req.annotators;
  outInfo("Setting new pipeline: ");
  for(auto a : newPipelineOrder)
  {
    if(!engine_.isInDelegateList(a))
    {
      outError(a << " was not initialized in current analysis engine");
      return false;
    }
    else
    {
      outInfo(a);
    }
  }

  if(useIdentityResolution_ && std::find(newPipelineOrder.begin(), newPipelineOrder.end(), "ObjectIdentityResolution") == newPipelineOrder.end())
  {
    newPipelineOrder.push_back("ObjectIdentityResolution");
  }

  std::vector<std::string> objDescriptions;
  {
    std::lock_guard<std::mutex> lock(processing_mutex_);
    visualizer_.setActiveAnnotators(newPipelineOrder);
    engine_.setNextPipeline(newPipelineOrder);
    engine_.applyNextPipeline();
    engine_.process(objDescriptions, "");
    engine_.resetPipelineOrdering();
    engine_.setNextPipeline(lowLvlPipeline_);
    res.object_descriptions.obj_descriptions = objDescriptions;
    result_pub.publish(res.object_descriptions);
  }

  return true;
}

bool RSProcessManager::jsonQueryCallback(robosherlock_msgs::RSQueryService::Request &req,
    robosherlock_msgs::RSQueryService::Response &res)
{

  handleQuery(req.query, res.answer);
  return true;
}

bool RSProcessManager::handleQuery(std::string &request, std::vector<std::string> &result)
{
  outInfo("JSON Reuqest: " << request);
  queryInterface->parseQuery(request);
  std::vector<std::string> newPipelineOrder;
  QueryInterface::QueryType queryType = queryInterface->processQuery(newPipelineOrder);

  {
    std::lock_guard<std::mutex> lock(processing_mutex_);
    if(queryType == QueryInterface::QueryType::DETECT)
    {
      std::vector<std::string> resultDesignators;


      outInfo(FG_BLUE << "Executing Pipeline # generated by query");
      visualizer_.setActiveAnnotators(newPipelineOrder);

      engine_.setNextPipeline(newPipelineOrder);
      engine_.applyNextPipeline();
      engine_.process(resultDesignators, request);
      engine_.resetPipelineOrdering();
      engine_.setNextPipeline(lowLvlPipeline_);

      outInfo("Executing pipeline generated by query: done");


      std::vector<std::string> filteredResponse;
      std::vector<bool> desigsToKeep;

      queryInterface->filterResults(resultDesignators, filteredResponse, desigsToKeep);

      cv::Mat resImage;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr dispCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

      if(useIdentityResolution_)
      {
        engine_.drawResulstOnImage<rs::Object>(desigsToKeep, resultDesignators, request, resImage);
        engine_.highlightResultsInCloud<rs::Object>(desigsToKeep, resultDesignators, request, dispCloud);
      }
      else
      {
        engine_.drawResulstOnImage<rs::ObjectHypothesis>(desigsToKeep, resultDesignators, request, resImage);
        engine_.highlightResultsInCloud<rs::ObjectHypothesis>(desigsToKeep, resultDesignators, request, dispCloud);
      }

      cv_bridge::CvImage outImgMsgs;
      //      outImgMsgs.header = cam_info.header;
      outImgMsgs.encoding = sensor_msgs::image_encodings::BGR8;
      outImgMsgs.image = resImage;
      image_pub_.publish(outImgMsgs.toImageMsg());

      pc_pub_.publish(dispCloud);

      result.insert(result.end(), filteredResponse.begin(), filteredResponse.end());
      robosherlock_msgs::RSObjectDescriptions objDescriptions;
      objDescriptions.obj_descriptions = result;
      result_pub.publish(objDescriptions);

      return true;
    }

    else if(queryType == QueryInterface::QueryType::INSPECT)
    {
      outInfo("Inspection is not implemented");
      return true;
    }
  }
  return false;
}
