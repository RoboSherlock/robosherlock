#include <robosherlock/flowcontrol/RSProcessManager.h>
#include <robosherlock/io/MongoDBBridge.h>
// needed for action server
#include <robosherlock/queryanswering/RSQueryActionServer.h>

RSProcessManager::RSProcessManager(std::string engineFile, const bool useVisualizer,
                                   rs::KnowledgeEngine::KnowledgeEngineType keType)
  : nh_("~")
  , spinner_(0)
  , it_(nh_)
  , useVisualizer_(useVisualizer)
  , use_identity_resolution_(false)
  , visualizer_(!useVisualizer, true)
{
  outInfo("Creating resource manager");
  signal(SIGINT, RSProcessManager::signalHandler);
  uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");
  resourceManager.setLoggingLevel(uima::LogStream::EnError);

  if(keType == rs::KnowledgeEngine::KnowledgeEngineType::JSON_PROLOG)
  {
    outInfo("Setting KnowRob (through rosprolog interface) as the knowledge engine.");
#if WITH_ROS_PROLOG
    if(ros::service::waitForService("rosprolog/query", ros::Duration(60.0)))
      knowledge_engine_ = std::make_shared<rs::RosPrologInterface>();
    else
      throw rs::Exception("rosprolog client sercivice not reachable");
#else
    throw rs::Exception("rosprolog was not found at compile time!");
#endif
  }
  else if(keType == rs::KnowledgeEngine::KnowledgeEngineType::SWI_PROLOG)
    knowledge_engine_ = std::make_shared<rs::SWIPLInterface>();
  else
    throw rs::Exception("Wrong initialization param for knowledge engine");

  query_interface_ = std::make_shared<QueryInterface>(knowledge_engine_);

  engine_ = rs::createRSAggregateAnalysisEngine(engineFile);

  knowledge_engine_->retractAllAnnotators();
  knowledge_engine_->assertAnnotators(engine_->getDelegateAnnotatorCapabilities());

  // ROS Service declarations
  setContextService_ = nh_.advertiseService("set_context", &RSProcessManager::resetAECallback, this);
  setFlowService_ = nh_.advertiseService("execute_pipeline", &RSProcessManager::executePipelineCallback, this);
  queryService_ = nh_.advertiseService("query", &RSProcessManager::jsonQueryCallback, this);

  // ROS publisher declarations
  result_pub_ = nh_.advertise<robosherlock_msgs::RSObjectDescriptions>(std::string("result_advertiser"), 1);
  image_pub_ = it_.advertise("result_image", 1, true);
  pc_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("points", 5);

  // ROS action server for query answering
  actionServer = new RSQueryActionServer(nh_, this);


  spinner_.start();
  visualizer_.addVisualizableGroupManager(engine_->getAAEName()); 
  visualizer_.start();

}

RSProcessManager::~RSProcessManager()
{
  visualizer_.stop();
  engine_->resetCas();
  engine_->collectionProcessComplete();
  engine_->destroy();
  uima::ResourceManager::deleteInstance();
  outInfo("RSControledAnalysisEngine Stoped");
}

void RSProcessManager::run()
{
  ros::Rate rate(30.0);
  for(; ros::ok();)
  {
    signal(SIGINT, RSProcessManager::signalHandler);
    {
      std::lock_guard<std::mutex> lock(processing_mutex_);
      if(!wait_for_service_call_)
      {
        std::vector<std::string> objDescriptions;
        engine_->resetCas();
        outInfo("waiting for all cameras to have new data...");
        engine_->processOnce();
        rs::ObjectDesignatorFactory dw(engine_->getCas());
        use_identity_resolution_ ? dw.setMode(rs::ObjectDesignatorFactory::Mode::OBJECT) :
        dw.setMode(rs::ObjectDesignatorFactory::Mode::CLUSTER);
        dw.getObjectDesignators(objDescriptions);
        robosherlock_msgs::RSObjectDescriptions objDescr;
        objDescr.obj_descriptions = objDescriptions;
        result_pub_.publish(objDescr);
      }
    }
    rate.sleep();
  }
}

bool RSProcessManager::visControlCallback(robosherlock_msgs::RSVisControl::Request &req,
    robosherlock_msgs::RSVisControl::Response &res)
{
  std::string command = req.command;
  bool result = true;
  std::string activeAnnotator = "";

  if(command == "next")
    activeAnnotator = visualizer_.nextAnnotator();
  else if(command == "previous")
    activeAnnotator = visualizer_.prevAnnotator();
  else if(command != "")
    activeAnnotator = visualizer_.selectAnnotator(command);
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
    return true;
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
    {
      std::lock_guard<std::mutex> lock(processing_mutex_);
      delete engine_;
      engine_ = rs::createRSAggregateAnalysisEngine(newAAEName);
      knowledge_engine_->retractAllAnnotators();
      knowledge_engine_->assertAnnotators(engine_->getDelegateAnnotatorCapabilities());
    }
    return true;
  }
  else
    return false;
}

bool RSProcessManager::executePipelineCallback(robosherlock_msgs::ExecutePipeline::Request &req,
    robosherlock_msgs::ExecutePipeline::Response &res)
{
  if(req.annotators.empty())
    return false;

  std::vector<std::string> new_pipeline_order = req.annotators;
  outInfo("Setting new pipeline: ");
  for(auto a : new_pipeline_order)
  {
    if(!engine_->isInDelegateList(a))
    {
      outError(a << " was not initialized in current analysis engine");
      return false;
    }
    else
      outInfo(a);
  }

  if(use_identity_resolution_ && std::find(new_pipeline_order.begin(), new_pipeline_order.end(), "ObjectIdentityResolution") == new_pipeline_order.end())
    new_pipeline_order.push_back("ObjectIdentityResolution");
  {
    std::lock_guard<std::mutex> lock(processing_mutex_);
    visualizer_.setActiveAnnotators(new_pipeline_order);
    outInfo("Resetting CAS...");
    engine_->resetCas();
    engine_->setPipelineOrdering(new_pipeline_order);
    engine_->processOnce();
    engine_->resetPipelineOrdering();
    engine_->setPipelineOrdering(lowLvlPipeline_);

    std::vector<std::string> objDescriptions;
    rs::ObjectDesignatorFactory dw(engine_->getCas());
    use_identity_resolution_ ? dw.setMode(rs::ObjectDesignatorFactory::Mode::OBJECT) :
    dw.setMode(rs::ObjectDesignatorFactory::Mode::CLUSTER);
    dw.getObjectDesignators(objDescriptions);
    res.object_descriptions.obj_descriptions = objDescriptions;
    result_pub_.publish(res.object_descriptions);
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
  query_interface_->parseQuery(request);
  std::vector<std::vector<std::string>> new_pipeline_orders;
  QueryInterface::QueryType queryType = query_interface_->processQuery(new_pipeline_orders);
  if(!new_pipeline_orders.empty())
  {
    std::lock_guard<std::mutex> lock(processing_mutex_);
    if(queryType == QueryInterface::QueryType::DETECT)
    {
      outInfo(FG_BLUE << "Executing Pipeline # generated by query");
      visualizer_.setActiveAnnotators(new_pipeline_orders[0]);
      outInfo("Resetting CAS...");

      engine_->resetCas();
      rs::Query query = rs::create<rs::Query>(*engine_->getCas());
      query.query.set(request);
      rs::SceneCas sceneCas(*engine_->getCas());
      sceneCas.setFS("QUERY", query);

      engine_->setPipelineOrdering(new_pipeline_orders[0]);
      engine_->processOnce();
      engine_->resetPipelineOrdering();

      outInfo("Executing pipeline generated by query: done");

      std::vector<std::string> resultDesignators;
      rs::ObjectDesignatorFactory dw(engine_->getCas());
      use_identity_resolution_ ? dw.setMode(rs::ObjectDesignatorFactory::Mode::OBJECT) :
      dw.setMode(rs::ObjectDesignatorFactory::Mode::CLUSTER);
      dw.getObjectDesignators(resultDesignators);

      std::vector<std::string> filteredResponse;
      std::vector<bool> desigsToKeep;

      query_interface_->filterResults(resultDesignators, filteredResponse, desigsToKeep);

      cv::Mat resImage;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr dispCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

      if(use_identity_resolution_)
      {
        // TODO:Move these to the interface class
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
      result_pub_.publish(objDescriptions);

      return true;
    }

    else if(queryType == QueryInterface::QueryType::INSPECT)
    {
      outInfo("Inspection is not implemented");
      return true;
    }
    else if(queryType == QueryInterface::QueryType::TRACK)
    {
      rapidjson::Document &query_document = query_interface_->getQueryDocument();
      if(query_document.HasMember("command"))
      {
        rapidjson::Value::MemberIterator command_iterator = query_document.FindMember("command");
        assert(query_document["command"].IsString());
        if(query_document["command"] == "stop")
        {
          outInfo(FG_LIGHTYELLOW << "COMMAND STOP");
          if(wait_for_service_call_)
          {
            result.push_back("Nothing is being tracked currently. Please start tracking before trying to stop it.");
            return true;
          }
          else
          {
            engine_->reconfigure();
            this->wait_for_service_call_ = true;
            result.push_back("Tracking stopped.");
            return true;
          }
        }
      }
      if(!wait_for_service_call_)
      {
        result.push_back("There's another tracking task being handled right now. Please stop it before starting tracking anew.");
        return true;
      }
      outInfo(FG_GREEN << "START TRACKING");
      if(new_pipeline_orders.size() < 2)
      {
        outError("2 pipelines are needed, but only " + std::to_string(new_pipeline_orders.size()) +
                 " have been built. Aborting...");
        result.push_back("2 pipelines are needed, but only " + std::to_string(new_pipeline_orders.size()) +
                         " have been built. Aborting...");
        return true;
      }
      outInfo(FG_BLUE << "Executing Pipeline # generated by query");
      visualizer_.setActiveAnnotators(new_pipeline_orders[0]);

      outInfo("Resetting CAS...");
      engine_->resetCas();
      rs::Query query = rs::create<rs::Query>(*engine_->getCas());
      query.query.set(request);
      rs::SceneCas sceneCas(*engine_->getCas());
      sceneCas.set("QUERY", query);

      engine_->setPipelineOrdering(new_pipeline_orders[0]);
      engine_->processOnce();
      //outInfo("Resetting CAS...");
      //engine_->resetCas();
      engine_->resetPipelineOrdering();
      engine_->setPipelineOrdering(lowLvlPipeline_);

      std::vector<std::string> result_designators;
      rs::ObjectDesignatorFactory dw(engine_->getCas());
      use_identity_resolution_ ? dw.setMode(rs::ObjectDesignatorFactory::Mode::OBJECT) : dw.setMode(rs::ObjectDesignatorFactory::Mode::CLUSTER);
      dw.getObjectDesignators(result_designators);

      for(int a = 0; a < result_designators.size(); a++)
        outInfo(result_designators[a]);
      outInfo("result_designators has ended.");

      std::vector<std::string> filtered_response;
      std::vector<bool> designators_to_keep;
      query_interface_->filterResults(result_designators, filtered_response, designators_to_keep);
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
        engine_->reconfigure();
        this->wait_for_service_call_ = true;
        result.push_back("Can't find any object fulfilling all given constraints. Tracking has not been started.");
        return true;
      }
      else
      {
        uima::CAS *tcas = engine_->getCas();
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

        // We don't want a CollectionReader in the first iteration of tracking, so trackers initialize on the same frame that hypotheses have been detected on.
        engine_->setPipelineOrdering(new_pipeline_orders[1]);
        engine_->processOnce(result_designators, request);
        new_pipeline_orders[1].insert(new_pipeline_orders[1].begin(), "CollectionReader");
        engine_->setPipelineOrdering(new_pipeline_orders[1]);
        //engine_->changeLowLevelPipeline(new_pipeline_orders[1]);
        this->wait_for_service_call_ = false;
        result.push_back("Tracking started.");
        return true;
      }
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
  rs::SceneCas sceneCas(*engine_->getCas());
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
  for(size_t i = 0; i < filter.size(); ++i)
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
  if(request.IsObject() && request.HasMember("obj-part"))
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
  if(request.IsObject() && request.HasMember("cad-model"))
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
  rs::SceneCas sceneCas(*engine_->getCas());
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
  cloud->header.frame_id = camToWorld.frame_id_;
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
