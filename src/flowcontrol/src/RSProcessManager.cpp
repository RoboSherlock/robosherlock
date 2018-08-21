#include <rs/flowcontrol/RSProcessManager.h>

RSProcessManager::RSProcessManager(const bool useVisualizer, const bool &waitForServiceCall,
                                   ros::NodeHandle n, const std::string &savePath):
  engine_(), inspectionEngine_(), nh_(n), it_(nh_),
  waitForServiceCall_(waitForServiceCall),
  useVisualizer_(useVisualizer), useIdentityResolution_(false),
  pause_(true), inspectFromAR_(false), visualizer_(savePath, !useVisualizer)
{

  outInfo("Creating resource manager");
  uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");

  switch(OUT_LEVEL) {
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
  result_pub = nh_.advertise<robosherlock_msgs::RSObjectDescriptions>(std::string("result_advertiser"), 1);
  setContextService = nh_.advertiseService("set_context", &RSProcessManager::resetAECallback, this);
  visService = nh_.advertiseService("vis_command", &RSProcessManager::visControlCallback, this);
  image_pub_ = it_.advertise("result_image", 1, true);
//  pc_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("points", 5);

#ifdef WITH_JSON_PROLOG
  jsonService = nh_.advertiseService("query", &RSProcessManager::jsonQueryCallback, this);
#endif
}

RSProcessManager::~RSProcessManager()
{
  uima::ResourceManager::deleteInstance();
  outInfo("RSControledAnalysisEngine Stoped");
}

void RSProcessManager::init(std::string &xmlFile, std::string configFile, bool pervasive, bool parallel)
{
  outInfo("initializing");

#ifdef WITH_JSON_PROLOG
  queryInterface = new QueryInterface();
#endif
  this->configFile_ = configFile;

  try {
    cv::FileStorage fs(cv::String(configFile), cv::FileStorage::READ);

    if(lowLvlPipeline_.empty()) { //if not set programatically, load from a config file
      cv::FileNode n = fs["annotators"];
      if(n.type() != cv::FileNode::SEQ) {
        outError("Annotators missing from config file");
      }
      cv::FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
      for(; it != it_end; ++it) {
        lowLvlPipeline_.push_back(*it);
      }
    }
    fs.release();
  }
  catch(cv::Exception &e) {
    outWarn("No low-level pipeline defined. Setting empty!");
  }

  engine_.init(xmlFile, parallel, pervasive , lowLvlPipeline_);

  parallel_ = parallel;

  visualizer_.start();
  if(pervasive) {
    visualizer_.setActiveAnnotators(lowLvlPipeline_);
  }
  outInfo("done intializing");
}


void RSProcessManager::setInspectionAE(std::string inspectionAEPath)
{
  outInfo("initializing inspection AE");
  std::vector<std::string> llvlp;
  llvlp.push_back("CollectionReader");
  inspectionEngine_.init(inspectionAEPath, parallel_,false, llvlp); // set parallel false for now, need discussion for future use of parallel execution
}


void RSProcessManager::run()
{
  for(; ros::ok();) {
    {
      std::lock_guard<std::mutex> lock(processing_mutex_);
      if(waitForServiceCall_ || pause_) {
        usleep(100000);
      }
      else {
        engine_.process();
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
  if(command == "next") {
    activeAnnotator = visualizer_.nextAnnotator();

  }
  else if(command == "previous") {
    activeAnnotator = visualizer_.prevAnnotator();
  }
  else if(command != "") {
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

  if(resetAE(newContextName)) {
    return true;
  }
  else {
    outError("Contexts need to have a an AE defined");
    outInfo("releasing lock");
    processing_mutex_.unlock();
    return false;
  }
}

bool RSProcessManager::resetAE(std::string newContextName)
{
  std::string contextAEPath;
  if(rs::common::getAEPaths(newContextName, contextAEPath)) {
    outInfo("Setting new context: " << newContextName);
    cv::FileStorage fs(cv::String(configFile_), cv::FileStorage::READ);

    cv::FileNode n = fs["annotators"];
    if(n.type() != cv::FileNode::SEQ) {
      outError("Somethings wrong with pipeline definition");
      return 1;
    }

    std::vector<std::string> lowLvlPipeline;
    cv::FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
    for(; it != it_end; ++it) {
      lowLvlPipeline.push_back(*it);
    }

    fs.release();

    {
      std::lock_guard<std::mutex> lock(processing_mutex_);
      this->init(contextAEPath, configFile_, false, parallel_);
    }
    //shouldn't there be an fs.release() here?

    return true;
  }
  else {
    return false;
  }
}
#ifdef WITH_JSON_PROLOG
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
    if(queryType == QueryInterface::QueryType::DETECT) {
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
      if(useIdentityResolution_) {
        engine_.drawResulstOnImage<rs::Object>(desigsToKeep, resultDesignators, request, resImage);
      }
      else {
        engine_.drawResulstOnImage<rs::Cluster>(desigsToKeep, resultDesignators, request, resImage);
      }

      cv_bridge::CvImage outImgMsgs;
      //      outImgMsgs.header = cam_info.header;
      outImgMsgs.encoding = sensor_msgs::image_encodings::BGR8;
      outImgMsgs.image = resImage;
      image_pub_.publish(outImgMsgs.toImageMsg());

      result.insert(result.end(), filteredResponse.begin(), filteredResponse.end());
      robosherlock_msgs::RSObjectDescriptions objDescriptions;
      objDescriptions.obj_descriptions = result;
      result_pub.publish(objDescriptions);

      return true;
    }
    else if(queryType == QueryInterface::QueryType::INSPECT) {
      if(!newPipelineOrder.empty()) {
        outInfo("planned new pipeline: ");
        for(auto s : newPipelineOrder)
          outInfo(s);
        inspectionEngine_.setNextPipeline(newPipelineOrder);
        inspectionEngine_.applyNextPipeline();
        inspectionEngine_.process();
      }
      return true;
    }
  }

  return false;
}

#endif
bool RSProcessManager::renderOffscreen(std::string object)
{
  std::string query;
  std::lock_guard<std::mutex> lock(processing_mutex_);

  //these are hacks,, where we need the
  query = "{\"render\":\"" + object + "\"}";

  std::vector<std::string> newPipelineOrder = {"CollectionReader",
                                               "ImagePreprocessor",
                                               "RegionFilter",
                                               "PlaneAnnotator",
                                               "ObjectIdentityResolution",
                                               "GetRenderedViews"
                                              };

  std::for_each(newPipelineOrder.begin(), newPipelineOrder.end(), [](std::string & p) {
    outInfo(p);
  });
  std::vector<std::string> resultDesignators;
  outInfo(FG_BLUE << "Executing offscreen rendering pipeline");
  engine_.setNextPipeline(newPipelineOrder);
  engine_.process(resultDesignators, query);
  outInfo("Executingoffscreen rendering pipeline: done");
  return true;
}
