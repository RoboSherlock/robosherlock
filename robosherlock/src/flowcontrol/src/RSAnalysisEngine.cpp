/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *         Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *         Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <rs/flowcontrol/RSAnalysisEngine.h>


static const std::string GEN_XML_PATH = ".ros/robosherlock_generated_xmls";

RSAnalysisEngine::RSAnalysisEngine() : useIdentityResolution_(false), query_(""), engine_(NULL), cas_(NULL)
{
}

RSAnalysisEngine::~RSAnalysisEngine()
{
  if(cas_) {
    delete cas_;
    cas_ = NULL;
  }
  if(engine_) {
    delete engine_;
    engine_ = NULL;
  }
}

void RSAnalysisEngine::init(const std::string &file, bool parallel, bool pervasive, std::vector<std::string> contPipeline)
{
  size_t pos = file.rfind('/');
  outInfo("Creating analysis engine: " FG_BLUE << (pos == file.npos ? file : file.substr(pos)));
  uima::ErrorInfo errorInfo;

  //Get the processed XML file
  passwd *pw = getpwuid(getuid());
  std::string HOMEPath(pw->pw_dir);
  std::string AEXMLDir(HOMEPath + "/" + GEN_XML_PATH);
  if(!boost::filesystem::exists(AEXMLDir))
    boost::filesystem::create_directory(AEXMLDir);
  //Extract the AE name without the extension
  boost::filesystem::path AEYamlPath(file);
  std::string AEXMLFile(AEXMLDir + "/" + AEYamlPath.stem().string() + ".xml");
  //Generate the xml from the yaml config and then process the XML
  AEYamlToXMLConverter aeConverter(file);
  aeConverter.parseYamlFile();
  std::ofstream xmlOutput;

  xmlOutput.open(AEXMLFile);
  aeConverter.getOutput(xmlOutput);
  xmlOutput.close();
  outInfo("Converted to: " << AEXMLFile);

  //Before creating the analysis engine, we need to find the annotators
  //that belongs to the fixed flow by simply looking for keyword fixedFlow
  //mapping between the name of the annotator to the path of it
  //TODO replace this with a DOM creation;
  std::unordered_map<std::string, std::string> delegateMapping;

  aeConverter.getDelegates(delegates_);
  for(std::string &a : delegates_) {

    std::string genXmlPath = convertYamlToXML(a);
    if(genXmlPath != "")
      delegateMapping[a]  = genXmlPath;
    else {
      outError("Could not generate and XML for: " << a);
      exit(1);
    }
  }
  outInfo("generated XML for annotators");

  engine_ = (RSAggregateAnalysisEngine *) rs::createParallelAnalysisEngine(AEXMLFile.c_str(), delegateMapping, errorInfo);
  if(engine_ == nullptr) {
    outInfo("Could not  create RSAggregateAnalysisEngine. Terminating");
    exit(1);
  }
  if(errorInfo.getErrorId() != UIMA_ERR_NONE) {
    outError("createAnalysisEngine failed.");
    throw std::runtime_error("An error occured during initializations;");
  }

  engine_->setParallel(parallel);
  parallel_ = parallel;
#ifdef WITH_JSON_PROLOG
  if(parallel) {
    engine_->initParallelPipelineManager();
    engine_->parallelPlanner.print();
  }
#endif

  const uima::AnalysisEngineMetaData &data = engine_->getAnalysisEngineMetaData();
  data.getName().toUTF8String(name_);

  // Get a new CAS
  outInfo("Creating a new CAS");
  cas_ = engine_->newCAS();

  if(cas_ == NULL) {
    outError("Creating new CAS failed.");
    engine_->destroy();
    delete engine_;
    engine_ = NULL;
    throw uima::Exception(uima::ErrorMessage(UIMA_ERR_ENGINE_NO_CAS), UIMA_ERR_ENGINE_NO_CAS, uima::ErrorInfo::unrecoverable);
  }

#ifdef WITH_JSON_PROLOG
  if(ros::service::waitForService("json_prolog/simple_query", ros::Duration(2.0))) {
    jsonPrologInterface.retractAllAnnotators();
    jsonPrologInterface.assertAnnotators(delegateCapabilities_);
  }
  else {
    outWarn("Json Prolog is not running! Query answering will not be possible");
  }
#endif

  if(pervasive) {
    //After all annotators have been initialized, pick the default pipeline
    //this stores the pipeline
    changeLowLevelPipeline(contPipeline);
    setNextPipeline(contPipeline);
  }

}

std::string RSAnalysisEngine::convertYamlToXML(std::string annotatorName)
{
  std::string yamlPath = rs::common::getAnnotatorPath(annotatorName);
  if(yamlPath == "") {
    outError("Annotator defined in fixedFlow: " << annotatorName << " can not be found! Exiting!");
    exit(1);
  }
  // If the path is yaml file, we need to convert it to xml
  if(boost::algorithm::ends_with(yamlPath, "yaml")) {

    YamlToXMLConverter converter(yamlPath);
    try {
      converter.parseYamlFile();
      delegateCapabilities_[annotatorName] = converter.getAnnotatorCapabilities();
    }
    catch(YAML::ParserException e) {
      outError("Exception happened when parsing the yaml file: " << yamlPath);
      outError(e.what());
      return "";
    }

    try {
      // To Get $HOME path
      passwd *pw = getpwuid(getuid());
      std::string HOMEPath(pw->pw_dir);
      std::string xmlDir = HOMEPath + "/" + GEN_XML_PATH;
      std::string xmlPath = xmlDir + "/" +  annotatorName + ".xml";

      if(!boost::filesystem::exists(xmlDir))
        boost::filesystem::create_directory(xmlDir);
      std::ofstream of(xmlPath);
      converter.getXml(of);
      of.close();
      return xmlPath;
    }
    catch(std::runtime_error &e) {
      outError("Exception happened when creating the output file: " << e.what());
      return "";
    }
    catch(std::exception &e) {
      outError("Exception happened when creating the output file: " << e.what());
      return "";
    }
  }
  return "";
}

void RSAnalysisEngine::stop()
{
  engine_->collectionProcessComplete();
  engine_->destroy();

  outInfo("Analysis engine stopped: " << name_);
}


void RSAnalysisEngine::process()
{
  std::vector<std::string> desigResponse;
  this->process(desigResponse, query_);
}

void RSAnalysisEngine::process(std::vector<std::string> &designatorResponse,
                               std::string queryString)
{
  outInfo("executing analisys engine: " << name_);
  cas_->reset();

  if(queryString != "" || query_ != "") {
    rs::Query query = rs::create<rs::Query>(*cas_);
    queryString !="" ? query.query.set(queryString):query.query.set(query_);
    rs::SceneCas sceneCas(*cas_);
    sceneCas.set("QUERY", query);
  }
  try {
    UnicodeString ustrInputText;
    ustrInputText.fromUTF8(name_);
    cas_->setDocumentText(uima::UnicodeStringRef(ustrInputText));

    rs::StopWatch clock;
    outInfo("processing CAS");
    try {
#ifdef WITH_JSON_PROLOG
      if(parallel_) {
        if(engine_->querySuccess) {
          engine_->paralleledProcess(*cas_);
        }
        else {
          outWarn("Query annotator dependency for planning failed! Fall back to linear execution!");
          engine_->process(*cas_);
        }
      }
      else {
        engine_->process(*cas_);
      }
#else
      engine_->process(*cas_);
#endif
    }
    catch(const rs::FrameFilterException &) {
      //we could handle image logging here
      //handle extra pipeline here->signal thread that we can start processing
      outError("Got Interrputed with Frame Filter, not time here");
    }

    outInfo("processing finished");
    outInfo(clock.getTime() << " ms." << std::endl << std::endl << FG_YELLOW
            << "********************************************************************************" << std::endl);
  }
  catch(const rs::Exception &e) {
    outError("Exception: " << std::endl << e.what());
  }
  catch(const uima::Exception &e) {
    outError("Exception: " << std::endl << e);
  }
  catch(const std::exception &e) {
    outError("Exception: " << std::endl << e.what());
  }
  catch(...) {
    outError("Unknown exception!");
  }

  rs::DesignatorWrapper dw(cas_);
  useIdentityResolution_ ? dw.setMode(rs::DesignatorWrapper::OBJECT) : dw.setMode(rs::DesignatorWrapper::CLUSTER);
  dw.getObjectDesignators(designatorResponse);
  outInfo("processing finished");
}


template <class T>
bool RSAnalysisEngine::drawResulstOnImage(const std::vector<bool> &filter,
    const std::vector<std::string> &resultDesignators,
    std::string &requestJson, cv::Mat &outImg)
{

  if(filter.size() != resultDesignators.size()) {
    outError("Filter and results descriptions sizes don't match");
    return false;
  }
  rs::SceneCas sceneCas(*cas_);
  rs::Scene scene = sceneCas.getScene();
  cv::Mat rgb = cv::Mat::zeros(480, 640, CV_64FC3);

  sensor_msgs::CameraInfo cam_info;

  sceneCas.get(VIEW_COLOR_IMAGE, rgb);
  sceneCas.get(VIEW_CAMERA_INFO, cam_info);

  std::vector<T> clusters;
  if(std::is_same<T, rs::Cluster>::value) {
    scene.identifiables.filter(clusters);
  }
  else {
    sceneCas.get(VIEW_OBJECTS, clusters);
  }

  outInfo("Clusters size: " << clusters.size() << "Designator size: " << resultDesignators.size());
  int colorIdx = 0;
  if(clusters.size() != resultDesignators.size()) {
    outInfo("Undefined behaviour");
    return false;
  }
  for(int i = 0; i < filter.size(); ++i) {
    if(!filter[i]) continue;

    std::string desigString = resultDesignators[i];
    rapidjson::Document desig;
    desig.Parse(desigString.c_str());
    if(desig.HasMember("id")) {
      std::string cID(desig["id"].GetString());
      int clusterId = std::atoi(cID.c_str());

      //Draw cluster on image
      rs::ImageROI roi = clusters[clusterId].rois();
      cv::Rect cvRoi;
      rs::conversion::from(roi.roi(), cvRoi);
      cv::rectangle(rgb, cvRoi, rs::common::cvScalarColors[clusterId % rs::common::numberOfColors], 1.5);
      std::stringstream clusterName;
      clusterName << "cID_" << clusterId;
      cv::putText(rgb, clusterName.str(), cv::Point(cvRoi.x + 10, cvRoi.y - 10), cv::FONT_HERSHEY_COMPLEX, 0.7, rs::common::cvScalarColors[clusterId % rs::common::numberOfColors]);
      colorIdx++;
    }
  }

  rapidjson::Document request;
  request.Parse(requestJson.c_str());
  if(request.HasMember("obj-part")) {
    for(int i = 0; i < clusters.size(); ++i) {
      rs::Cluster &cluster = clusters[i];
      std::vector<rs::ClusterPart> parts;
      cluster.annotations.filter(parts);
      for(int pIdx = 0; pIdx < parts.size(); ++pIdx) {
        rs::ClusterPart &part = parts[pIdx];
        if(part.name() == request["obj-part"] || request["obj-part"] == "") {
          pcl::PointIndices indices;
          rs::conversion::from(part.indices(), indices);
          for(int iIdx = 0; iIdx < indices.indices.size(); ++iIdx) {
            int idx = indices.indices[iIdx];
            rgb.at<cv::Vec3b>(cv::Point(idx % cam_info.width, idx / cam_info.width)) = rs::common::cvVec3bColors[pIdx % rs::common::numberOfColors];
          }
          colorIdx++;
        }
      }
    }
  }
  if(request.HasMember("cad-model")) {
    if(sceneCas.has("VIEW_DISPLAY_IMAGE")) {
      outInfo("Scene has a display image");
      sceneCas.get("VIEW_DISPLAY_IMAGE", rgb);
    }
  }
  outImg = rgb.clone();
  return true;
}

template <class T>
bool RSAnalysisEngine::highlightResultsInCloud(const std::vector<bool> &filter,
    const std::vector<std::string> &resultDesignators,
    std::string &requestJson, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{

  if(filter.size() != resultDesignators.size()) {
    outError("Filter and results descriptions sizes don't match");
    return false;
  }
  rs::SceneCas sceneCas(*cas_);
  rs::Scene scene = sceneCas.getScene();

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr dispCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
  sensor_msgs::CameraInfo cam_info;

  sceneCas.get(VIEW_CAMERA_INFO, cam_info);
  sceneCas.get(VIEW_CLOUD, *dispCloud);

  std::vector<T> clusters;
  std::is_same<T, rs::Cluster>::value ? scene.identifiables.filter(clusters) : sceneCas.get(VIEW_OBJECTS, clusters);


  outInfo("Clusters size: " << clusters.size() << "Designator size: " << resultDesignators.size());
  int colorIdx = 0;
  if(clusters.size() != resultDesignators.size()) {
    outInfo("Undefined behaviour");
    return false;
  }

  for(int i = 0; i < filter.size(); ++i) {
    if(!filter[i]) continue;

    std::string desigString = resultDesignators[i];
    rapidjson::Document desig;
    desig.Parse(desigString.c_str());
    if(desig.HasMember("id")) {
      std::string cID(desig["id"].GetString());
      int clusterId = std::atoi(cID.c_str());

      pcl::PointIndicesPtr inliers(new pcl::PointIndices());
      if(clusters[clusterId].points.has()) {
        rs::conversion::from(((rs::ReferenceClusterPoints)clusters[clusterId].points()).indices(), *inliers);
        for(unsigned int idx = 0; idx < inliers->indices.size(); ++idx) {
          dispCloud->points[inliers->indices[idx]].rgba = rs::common::colors[colorIdx % rs::common::numberOfColors];
          dispCloud->points[inliers->indices[idx]].a = 255;
        }
      }
      colorIdx++;
    }
  }

  tf::StampedTransform camToWorld;
  camToWorld.setIdentity();
  if(scene.viewPoint.has()) {
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

template bool RSAnalysisEngine::drawResulstOnImage<rs::Object>(const std::vector<bool> &filter,
    const std::vector<std::string> &resultDesignators,
    std::string &requestJson, cv::Mat &resImage);

template bool RSAnalysisEngine::drawResulstOnImage<rs::Cluster>(const std::vector<bool> &filter,
    const std::vector<std::string> &resultDesignators,
    std::string &requestJson, cv::Mat &resImage);

template bool RSAnalysisEngine::highlightResultsInCloud<rs::Object>(const std::vector<bool> &filter,
    const std::vector<std::string> &resultDesignators,
    std::string &requestJson, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

template bool RSAnalysisEngine::highlightResultsInCloud<rs::Cluster>(const std::vector<bool> &filter,
    const std::vector<std::string> &resultDesignators,
    std::string &requestJson, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
