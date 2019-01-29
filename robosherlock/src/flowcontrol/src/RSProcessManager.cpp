#include <rs/flowcontrol/RSProcessManager.h>

RSProcessManager::RSProcessManager(const bool useVisualizer, const bool &waitForServiceCall, bool withQnA,
                                   ros::NodeHandle n, const std::string &savePath):
  engine_(), nh_(n), it_(nh_),
  waitForServiceCall_(waitForServiceCall),
  withQA_(withQnA), useVisualizer_(useVisualizer), useIdentityResolution_(false),
  visualizer_(savePath, !useVisualizer)
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
  setContextService_ = nh_.advertiseService("set_context", &RSProcessManager::resetAECallback, this);
  visService_ = nh_.advertiseService("vis_command", &RSProcessManager::visControlCallback, this);
  setFlowService_ = nh_.advertiseService("execute_pipeline", &RSProcessManager::executePipelineCallback, this);

  result_pub = nh_.advertise<robosherlock_msgs::RSObjectDescriptions>(std::string("result_advertiser"), 1);
  image_pub_ = it_.advertise("result_image", 1, true);
  pc_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("points", 5);

#ifdef WITH_JSON_PROLOG
  if(withQA_)
    queryService_ = nh_.advertiseService("query", &RSProcessManager::jsonQueryCallback, this);
#endif
}

RSProcessManager::~RSProcessManager()
{
  uima::ResourceManager::deleteInstance();
  outInfo("RSControledAnalysisEngine Stoped");
}

void RSProcessManager::init(std::string &engineFile, std::string configFile, bool pervasive, bool parallel)
{
  outInfo("initializing");

#ifdef WITH_JSON_PROLOG
  if(withQA_)
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

  engine_.init(engineFile, parallel, pervasive , lowLvlPipeline_);

  parallel_ = parallel;

  visualizer_.start();
  if(pervasive) {
    visualizer_.setActiveAnnotators(lowLvlPipeline_);
  }
  outInfo("done intializing");
}


void RSProcessManager::run()
{
  for(; ros::ok();) {
    {
      std::lock_guard<std::mutex> lock(processing_mutex_);
      if(waitForServiceCall_) {
        usleep(100000);
      }
      else {
        std::vector<std::string> objDescriptions;
        outInfo("Resetting CAS...");
        engine_.resetCas();
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


bool RSProcessManager::resetAE(std::string newAAEName)
{
  std::string contextAEPath;
  if(rs::common::getAEPaths(newAAEName, contextAEPath)) {
    outInfo("Setting new context: " << newAAEName);
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

    return true;
  }
  else {
    return false;
  }
}

bool RSProcessManager::executePipelineCallback(robosherlock_msgs::ExecutePipeline::Request &req,
    robosherlock_msgs::ExecutePipeline::Response &res)
{
  if(req.annotators.empty()) {
    return false;
  }

  std::vector<std::string> newPipelineOrder = req.annotators;
  outInfo("Setting new pipeline: ");
  for(auto a : newPipelineOrder) {
    if(!engine_.isInDelegateList(a)) {
      outError(a << " was not initialized in current analysis engine");
      return false;
    }
    else {
      outInfo(a);
    }
  }


  if(useIdentityResolution_ && std::find(newPipelineOrder.begin(), newPipelineOrder.end(), "ObjectIdentityResolution") == newPipelineOrder.end()) {
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
  std::vector<std::vector<std::string>> newPipelineOrders;
  QueryInterface::QueryType queryType = queryInterface->processQuery(newPipelineOrders);
  if (!newPipelineOrders.empty())
  {
    std::lock_guard<std::mutex> lock(processing_mutex_);
    if(queryType == QueryInterface::QueryType::DETECT) {
      std::vector<std::string> resultDesignators;


      outInfo(FG_BLUE << "Executing Pipeline # generated by query");
      visualizer_.setActiveAnnotators(newPipelineOrders[0]);

      engine_.setNextPipeline(newPipelineOrders[0]);
      engine_.applyNextPipeline();
      outInfo("Resetting CAS...");
      engine_.resetCas();
      engine_.process(resultDesignators, request);
      engine_.resetPipelineOrdering();
      engine_.setNextPipeline(lowLvlPipeline_);

      outInfo("Executing pipeline generated by query: done");


      std::vector<std::string> filteredResponse;
      std::vector<bool> desigsToKeep;

      queryInterface->filterResults(resultDesignators, filteredResponse, desigsToKeep);

      cv::Mat resImage;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr dispCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

      if(useIdentityResolution_) {
        engine_.drawResulstOnImage<rs::Object>(desigsToKeep, resultDesignators, request, resImage);
        engine_.highlightResultsInCloud<rs::Object>(desigsToKeep, resultDesignators, request, dispCloud);
      }
      else {
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

    else if(queryType == QueryInterface::QueryType::INSPECT) {
      outInfo("Inspection is not implemented");
      return true;
    }
    else if(queryType == QueryInterface::QueryType::TRACK)  {

      outInfo("Checking whether to start or stop tracking...");
      if (queryInterface->query.HasMember("command")) {
        outInfo(FG_BLUE << "Found a command member!");
        rapidjson::Value::MemberIterator commandIterator = queryInterface->query.FindMember("command");
        assert(queryInterface->query["command"].IsString());
        if (queryInterface->query["command"] == "stop") { // for some reason, this is never true
          outInfo(FG_LIGHTYELLOW << "COMMAND STOP");
          engine_.reconfigure();
          this->waitForServiceCall_=true;
          result.push_back("Tracking stopped.");
          return true;
        }
      }
      outInfo(FG_GREEN << "START TRACKING");
      std::vector<std::string> resultDesignators;
      outInfo(FG_BLUE << "Executing Pipeline # generated by query");
      visualizer_.setActiveAnnotators(newPipelineOrders[0]);
      engine_.setNextPipeline(newPipelineOrders[0]);
      engine_.applyNextPipeline();
      outInfo("Resetting CAS...");
      engine_.resetCas();
      engine_.process(resultDesignators, request);
      engine_.resetPipelineOrdering();
      // engine_.setNextPipeline(lowLvlPipeline_);

      std::vector<std::string> filteredResponse;
      std::vector<bool> designatorsToKeep;
      queryInterface->filterResults(resultDesignators, filteredResponse, designatorsToKeep);
      int obj_id;
      for(int n = 0; n < designatorsToKeep.size(); n++){
        if(designatorsToKeep[n])
        {
          obj_id = n;
          outInfo("Object " + std::to_string(obj_id) +  " satisfies query designators.");
          break;
        }
      }


      uima::CAS *tcas = engine_.getCas();
      rs::SceneCas sceneCas(*tcas);
      rs::Scene scene = sceneCas.getScene();

      std::vector<rs::ObjectHypothesis> objHyps;
      scene.identifiables.filter(objHyps);
      rs::Size s = rs::create<rs::Size>(*tcas); // This is just a hack to get a simple integer (the object ID) into the cas.
      s.height.set(obj_id);
      outInfo(FG_GREEN << "SETTING OBJ_TO_TRACK");
      sceneCas.setFS("OBJ_ID_TRACK", s);
      outInfo("Found " << objHyps.size() << " objects");

      visualizer_.setActiveAnnotators(newPipelineOrders[1]);

      engine_.setNextPipeline(newPipelineOrders[1]);

      engine_.applyNextPipeline();
      engine_.process(resultDesignators, request);
      newPipelineOrders[1].insert(newPipelineOrders[1].begin(), "CollectionReader");
      //newPipelineOrders[1].insert(newPipelineOrders[1].begin(), "Trigger");
      engine_.changeLowLevelPipeline(newPipelineOrders[1]);
      engine_.resetPipelineOrdering();
      this->waitForServiceCall_=false;

      result.push_back("Tracking started.");
      return true;

      // TODO: Do I need the other stuff in the DETECT case here? What is that for?
    }
  }

  return false;
}
#endif
