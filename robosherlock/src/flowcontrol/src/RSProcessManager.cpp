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
    query_interface = new QueryInterface();
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
  query_interface->parseQuery(request);
  std::vector<std::vector<std::string>> new_pipeline_orders;
  QueryInterface::QueryType queryType = query_interface->processQuery(new_pipeline_orders);
  if (!new_pipeline_orders.empty())
  {
    std::lock_guard<std::mutex> lock(processing_mutex_);
    if(queryType == QueryInterface::QueryType::DETECT)
    {
      std::vector<std::string> resultDesignators;


      outInfo(FG_BLUE << "Executing Pipeline # generated by query");
      visualizer_.setActiveAnnotators(new_pipeline_orders[0]);

      engine_.setNextPipeline(new_pipeline_orders[0]);
      engine_.applyNextPipeline();
      outInfo("Resetting CAS...");
      engine_.resetCas();
      engine_.process(resultDesignators, request);
      engine_.resetPipelineOrdering();
      engine_.setNextPipeline(lowLvlPipeline_);

      outInfo("Executing pipeline generated by query: done");


      std::vector<std::string> filteredResponse;
      std::vector<bool> desigsToKeep;

      query_interface->filterResults(resultDesignators, filteredResponse, desigsToKeep);

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
    else if(queryType == QueryInterface::QueryType::TRACK)
    {
      if (query_interface->query.HasMember("command"))
      {
        rapidjson::Value::MemberIterator command_iterator = query_interface->query.FindMember("command");
        assert(query_interface->query["command"].IsString());
        if (query_interface->query["command"] == "stop")
        {
          outInfo(FG_LIGHTYELLOW << "COMMAND STOP");
          if(waitForServiceCall_)
          {
            result.push_back("Nothing is being tracked currently. Please start tracking before trying to stop it.");
            return true;
          }
          else
          {
          engine_.reconfigure();
          this->waitForServiceCall_=true;
          result.push_back("Tracking stopped.");
          return true;
          }
        }
      }
      outInfo(FG_GREEN << "START TRACKING");
      std::vector<std::string> result_designators;
      outInfo(FG_BLUE << "Executing Pipeline # generated by query");
      visualizer_.setActiveAnnotators(new_pipeline_orders[0]);
      engine_.setNextPipeline(new_pipeline_orders[0]);
      engine_.applyNextPipeline();
      outInfo("Resetting CAS...");
      engine_.resetCas();
      engine_.process(result_designators, request);
      engine_.resetPipelineOrdering();

      std::vector<std::string> filtered_response;
      std::vector<bool> designators_to_keep;
      query_interface->filterResults(result_designators, filtered_response, designators_to_keep);
      int obj_id = -1;
      for(int n = 0; n < designators_to_keep.size(); n++)
      {
        if(designators_to_keep[n])
        {
          obj_id = n;
          outInfo("Object " + std::to_string(obj_id) +  " satisfies query designators.");
          break;
        }
      }

      if(obj_id == -1)
      {
        outError("Can't find any object fulfilling all given constraints. Tracking will not be started.");
        engine_.reconfigure();
        this->waitForServiceCall_=true;
        result.push_back("Can't find any object fulfilling all given constraints. Tracking has not been started.");
        return true;
      }
      else
        {
        uima::CAS *tcas = engine_.getCas();
        rs::SceneCas sceneCas(*tcas);
        rs::Scene scene = sceneCas.getScene();

        std::vector<rs::ObjectHypothesis> obj_hyps;
        scene.identifiables.filter(obj_hyps);
        rs::Size s = rs::create<rs::Size>(*tcas); // This is just a hack to get a simple integer (the object ID) into the cas.
        s.height.set(obj_id);
        outInfo(FG_GREEN << "SETTING OBJ_TO_TRACK");
        sceneCas.setFS("OBJ_ID_TRACK", s);
        outInfo("Found " << obj_hyps.size() << " objects");

        visualizer_.setActiveAnnotators(new_pipeline_orders[1]);

        engine_.setNextPipeline(new_pipeline_orders[1]);

        engine_.applyNextPipeline();
        engine_.process(result_designators, request);
        new_pipeline_orders[1].insert(new_pipeline_orders[1].begin(), "CollectionReader");
        engine_.changeLowLevelPipeline(new_pipeline_orders[1]);
        engine_.resetPipelineOrdering();
        this->waitForServiceCall_=false;

        result.push_back("Tracking started.");
        return true;
      }
    }
  }

  return false;
}
#endif
