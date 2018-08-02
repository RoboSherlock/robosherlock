#include <rs/flowcontrol/RSProcessManager.h>

RSProcessManager::RSProcessManager(const bool useVisualizer, const bool &waitForServiceCall,
                                   ros::NodeHandle n, const std::string &savePath):
  engine_(n), inspectionEngine_(n), nh_(n), waitForServiceCall_(waitForServiceCall),
  useVisualizer_(useVisualizer), useIdentityResolution_(false),
  pause_(true), inspectFromAR_(false), visualizer_(savePath, !useVisualizer)
{

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
  result_pub = nh_.advertise<robosherlock_msgs::RSObjectDescriptions>(std::string("result_advertiser"), 1);

  setContextService = nh_.advertiseService("set_context", &RSProcessManager::resetAECallback, this);

  visService = nh_.advertiseService("vis_command", &RSProcessManager::visControlCallback, this);

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

  try
  {
    cv::FileStorage fs(cv::String(configFile), cv::FileStorage::READ);

    if(lowLvlPipeline_.empty()) //if not set programatically, load from a config file
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

  engine_.init(xmlFile, lowLvlPipeline_, pervasive, parallel);

  parallel_ = parallel;

  visualizer_.start();
  if(pervasive)
  {
    visualizer_.setActiveAnnotators(lowLvlPipeline_);
  }
  outInfo("done intializing");
}


void RSProcessManager::setInspectionAE(std::string inspectionAEPath)
{
  outInfo("initializing inspection AE");
  std::vector<std::string> llvlp;
  llvlp.push_back("CollectionReader");
  inspectionEngine_.init(inspectionAEPath, llvlp, false, parallel_); // set parallel false for now, need discussion for future use of parallel execution
}


void RSProcessManager::run()
{
  for(; ros::ok();)
  {
    {
      std::lock_guard<std::mutex> lock(processing_mutex_);
      if(waitForServiceCall_ || pause_)
      {
        usleep(100000);
      }
      else
      {
        engine_.process(true);
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

bool RSProcessManager::resetAE(std::string newContextName)
{
  std::string contextAEPath;
  if(rs::common::getAEPaths(newContextName, contextAEPath))
  {
    outInfo("Setting new context: " << newContextName);
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
    //shouldn't there be an fs.release() here?

    return true;
  }
  else
  {
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

  processing_mutex_.lock();
  if(queryType == QueryInterface::QueryType::DETECT)
  {
    std::vector<std::string> resultDesignators;


    outInfo(FG_BLUE << "Executing Pipeline # generated by query");
    visualizer_.setActiveAnnotators(newPipelineOrder);
    engine_.process(newPipelineOrder, true, resultDesignators, request);
    outInfo("Executing pipeline generated by query: done");


    std::vector<std::string> filteredResponse;
    std::vector<bool> desigsToKeep;

    queryInterface->filterResults(resultDesignators, filteredResponse, desigsToKeep);


    if(useIdentityResolution_)
    {
      engine_.drawResulstOnImage<rs::Object>(desigsToKeep, resultDesignators, request);
    }
    else
    {
      engine_.drawResulstOnImage<rs::Cluster>(desigsToKeep, resultDesignators, request);
    }

    result.insert(result.end(), filteredResponse.begin(), filteredResponse.end());

    robosherlock_msgs::RSObjectDescriptions objDescriptions;
    objDescriptions.obj_descriptions = result;
    result_pub.publish(objDescriptions);

    processing_mutex_.unlock();
    return true;
  }
  else if(queryType == QueryInterface::QueryType::INSPECT)
  {
    if(!newPipelineOrder.empty())
    {
      outInfo("planned new pipeline: ");
      for(auto s : newPipelineOrder)
      {
        outInfo(s);
      }
      inspectionEngine_.setNextPipeline(newPipelineOrder);
      inspectionEngine_.applyNextPipeline();
      inspectionEngine_.process();
    }

    processing_mutex_.unlock();
    return true;
  }

  processing_mutex_.unlock();
  return false;
}

#endif
bool RSProcessManager::renderOffscreen(std::string object)
{
  std::string query;
  processing_mutex_.lock();

  //these are hacks,, where we need the
  query = "{\"render\":\"" + object + "\"}";

  std::vector<std::string> newPipelineOrder = {"CollectionReader",
                                               "ImagePreprocessor",
                                               "RegionFilter",
                                               "PlaneAnnotator",
                                               "ObjectIdentityResolution",
                                               "GetRenderedViews"
                                              };

  std::for_each(newPipelineOrder.begin(), newPipelineOrder.end(), [](std::string & p)
  {
    outInfo(p);
  });
  std::vector<std::string> resultDesignators;
  outInfo(FG_BLUE << "Executing offscreen rendering pipeline");
  engine_.process(newPipelineOrder, true, resultDesignators, query);
  outInfo("Executingoffscreen rendering pipeline: done");
  processing_mutex_.unlock();
  return true;
}
