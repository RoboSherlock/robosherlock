#include <rs/flowcontrol/RSProcessManager.h>

RSProcessManager::RSProcessManager(const bool useVisualizer, const bool &waitForServiceCall, RSProcessManager::KnowledgeEngineType keType,
                                   ros::NodeHandle n, const std::string &savePath):
  nh_(n), it_(nh_),
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

//  engine_.init(engineFile, parallel, pervasive , lowLvlPipeline_);
  engine_ = rs::createRSAggregateAnalysisEngine(engineFile,parallel, pervasive , lowLvlPipeline_);

  knowledgeEngine_->retractAllAnnotators();
  knowledgeEngine_->assertAnnotators(engine_->getDelegateAnnotatorCapabilities());
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
        engine_->simpleProcess(objDescriptions, "");
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
  engine_->resetCas();
  engine_->collectionProcessComplete();
  engine_->destroy();
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
    if(!engine_->isInDelegateList(a))
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
    engine_->setPipelineOrdering(newPipelineOrder);
    engine_->simpleProcess(objDescriptions, "");
    engine_->resetPipelineOrdering();
    engine_->setNextPipeline(lowLvlPipeline_);
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

//      engine_.setNextPipeline(newPipelineOrder);
      engine_->setPipelineOrdering(newPipelineOrder);
      engine_->simpleProcess(resultDesignators, request);
      engine_->resetPipelineOrdering();
      engine_->setNextPipeline(lowLvlPipeline_);

      outInfo("Executing pipeline generated by query: done");


      std::vector<std::string> filteredResponse;
      std::vector<bool> desigsToKeep;

      queryInterface->filterResults(resultDesignators, filteredResponse, desigsToKeep);

      cv::Mat resImage;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr dispCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

      if(useIdentityResolution_)
      {
          //TODO:Move these to the interface class
        drawResultsOnImage<rs::Object>(desigsToKeep, resultDesignators, request, resImage);
        highlightResultsInCloud<rs::Object>(desigsToKeep, resultDesignators, request, dispCloud);
      }
      else
      {
        drawResultsOnImage<rs::ObjectHypothesis>(desigsToKeep, resultDesignators, request, resImage);
        highlightResultsInCloud<rs::ObjectHypothesis>(desigsToKeep, resultDesignators, request, dispCloud);
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

template <class T>
bool RSProcessManager::drawResultsOnImage(const std::vector<bool> &filter,
    const std::vector<std::string> &resultDesignators, std::string &requestJson,
    cv::Mat &outImg)
{
  if(filter.size() != resultDesignators.size())
  {
    outError("Filter and results descriptions sizes don't match");
    return false;
  }
  rs::SceneCas sceneCas(*engine_->get_cas());
  rs::Scene scene = sceneCas.getScene();
  cv::Mat rgb = cv::Mat::zeros(480, 640, CV_64FC3);

  sensor_msgs::CameraInfo cam_info;

  sceneCas.get(VIEW_COLOR_IMAGE, rgb);
  sceneCas.get(VIEW_CAMERA_INFO, cam_info);

  std::vector<T> clusters;
  if(std::is_same<T, rs::ObjectHypothesis>::value)
  {
    scene.identifiables.filter(clusters);
  }
  else
  {
    sceneCas.get(VIEW_OBJECTS, clusters);
  }

  outInfo("Clusters size: " << clusters.size() << "Designator size: " << resultDesignators.size());
  int colorIdx = 0;
  if(clusters.size() != resultDesignators.size())
  {
    outInfo("Undefined behaviour");
    return false;
  }
  for(int i = 0; i < filter.size(); ++i)
  {
    if(!filter[i])
      continue;

    std::string desigString = resultDesignators[i];
    rapidjson::Document desig;
    desig.Parse(desigString.c_str());

    if(desig.HasMember("id"))
    {
      std::string cID(desig["id"].GetString());

      // Draw cluster on image
      rs::ImageROI roi = clusters[i].rois();
      cv::Rect cvRoi;
      rs::conversion::from(roi.roi(), cvRoi);
      cv::rectangle(rgb, cvRoi, rs::common::cvScalarColors[i % rs::common::numberOfColors], 1.5);
      std::stringstream clusterName;
      clusterName << cID;
      cv::putText(rgb, clusterName.str(), cv::Point(cvRoi.x + 10, cvRoi.y - 10), cv::FONT_HERSHEY_COMPLEX, 0.7,
                  rs::common::cvScalarColors[i % rs::common::numberOfColors]);
      colorIdx++;
    }
  }

  rapidjson::Document request;
  request.Parse(requestJson.c_str());
  if(request.HasMember("obj-part"))
  {
    for(int i = 0; i < clusters.size(); ++i)
    {
      rs::ObjectHypothesis &cluster = clusters[i];
      std::vector<rs::ClusterPart> parts;
      cluster.annotations.filter(parts);
      for(int pIdx = 0; pIdx < parts.size(); ++pIdx)
      {
        rs::ClusterPart &part = parts[pIdx];
        if(part.name() == request["obj-part"] || request["obj-part"] == "")
        {
          pcl::PointIndices indices;
          rs::conversion::from(part.indices(), indices);
          for(int iIdx = 0; iIdx < indices.indices.size(); ++iIdx)
          {
            int idx = indices.indices[iIdx];
            rgb.at<cv::Vec3b>(cv::Point(idx % cam_info.width, idx / cam_info.width)) =
              rs::common::cvVec3bColors[pIdx % rs::common::numberOfColors];
          }
          colorIdx++;
        }
      }
    }
  }
  if(request.HasMember("cad-model"))
  {
    if(sceneCas.has("VIEW_DISPLAY_IMAGE"))
    {
      outInfo("Scene has a display image");
      sceneCas.get("VIEW_DISPLAY_IMAGE", rgb);
    }
  }
  outImg = rgb.clone();
  return true;
}

template <class T>
bool RSProcessManager::highlightResultsInCloud(const std::vector<bool> &filter,
    const std::vector<std::string> &resultDesignators,
    std::string &requestJson, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
  if(filter.size() != resultDesignators.size())
  {
    outError("Filter and results descriptions sizes don't match");
    return false;
  }
  rs::SceneCas sceneCas(*engine_->get_cas());
  rs::Scene scene = sceneCas.getScene();

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr dispCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
  sensor_msgs::CameraInfo cam_info;

  sceneCas.get(VIEW_CAMERA_INFO, cam_info);
  sceneCas.get(VIEW_CLOUD, *dispCloud);

  std::vector<T> clusters;
  std::is_same<T, rs::ObjectHypothesis>::value ? scene.identifiables.filter(clusters) :
  sceneCas.get(VIEW_OBJECTS, clusters);

  outInfo("Clusters size: " << clusters.size() << "Designator size: " << resultDesignators.size());
  int colorIdx = 0;
  if(clusters.size() != resultDesignators.size())
  {
    outInfo("Undefined behaviour");
    return false;
  }

  for(int i = 0; i < filter.size(); ++i)
  {
    if(!filter[i])
      continue;

    std::string desigString = resultDesignators[i];
    rapidjson::Document desig;
    desig.Parse(desigString.c_str());
    if(desig.HasMember("id"))
    {
      pcl::PointIndicesPtr inliers(new pcl::PointIndices());
      if(clusters[i].points.has())
      {
        rs::conversion::from(((rs::ReferenceClusterPoints)clusters[i].points()).indices(), *inliers);
        for(unsigned int idx = 0; idx < inliers->indices.size(); ++idx)
        {
          dispCloud->points[inliers->indices[idx]].rgba = rs::common::colors[colorIdx % rs::common::numberOfColors];
          dispCloud->points[inliers->indices[idx]].a = 255;
        }
      }
      colorIdx++;
    }
  }

  tf::StampedTransform camToWorld;
  camToWorld.setIdentity();
  if(scene.viewPoint.has())
  {
    rs::conversion::from(scene.viewPoint.get(), camToWorld);
  }

  Eigen::Affine3d eigenTransform;
  tf::transformTFToEigen(camToWorld, eigenTransform);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGBA>()),
      dsCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
  pcl::transformPointCloud<pcl::PointXYZRGBA>(*dispCloud, *transformed, eigenTransform);

  pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
  float leaf_size = 0.005;
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.setInputCloud(transformed);
  vg.filter(*dsCloud);

  pcl::copyPointCloud(*dsCloud, *cloud);
  cloud->header.frame_id = camToWorld.child_frame_id_;
  return true;
}

template bool RSProcessManager::drawResultsOnImage<rs::Object>(const std::vector<bool> &filter,
    const std::vector<std::string> &resultDesignators,
    std::string &requestJson, cv::Mat &resImage);

template bool RSProcessManager::drawResultsOnImage<rs::ObjectHypothesis>(
  const std::vector<bool> &filter, const std::vector<std::string> &resultDesignators, std::string &requestJson,
  cv::Mat &resImage);

template bool RSProcessManager::highlightResultsInCloud<rs::Object>(const std::vector<bool> &filter,
    const std::vector<std::string> &resultDesignators,
    std::string &requestJson,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

template bool RSProcessManager::highlightResultsInCloud<rs::ObjectHypothesis>(
  const std::vector<bool> &filter, const std::vector<std::string> &resultDesignators, std::string &requestJson,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
