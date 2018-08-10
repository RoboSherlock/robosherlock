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

#include <rs/flowcontrol/RSControledAnalysisEngine.h>


void RSControledAnalysisEngine::init(const std::string &AEFile, const std::vector<std::string> &lowLvlPipeline, bool pervasive, bool parallel)
{
  RSAnalysisEngine::init(AEFile, parallel);

  this->initPipelineManager();

  std::vector<icu::UnicodeString> &non_const_nodes = rspm->getFlowConstraintNodes();
  std::vector<std::string> fixedFlow;
  outInfo("*** Fetch the FlowConstraint nodes. Size is: "  << non_const_nodes.size());
  for(int i = 0; i < non_const_nodes.size(); i++)
  {
    std::string tempString;
    non_const_nodes.at(i).toUTF8String(tempString);
    outInfo(tempString);
    fixedFlow.push_back(tempString);
  }

#ifdef WITH_JSON_PROLOG
  if( ros::service::waitForService("json_prolog/simple_query",ros::Duration(2.0)))
  {jsonPrologInterface.retractAllAnnotators();
  jsonPrologInterface.assertAnnotators(fixedFlow);}
  else{
      outWarn("Json Prolog is not running! Query answering will not be possible");
  }
#endif

  if(pervasive)
  {
    //After all annotators have been initialized, pick the default pipeline
    //this stores the pipeline
    rspm->setDefaultPipelineOrdering(lowLvlPipeline);
    //this applies it
    rspm->setPipelineOrdering(lowLvlPipeline);
  }

  // Get a new CAS
  outInfo("Creating a new CAS");

  outInfo("initialization done: " << name_ << std::endl
          << std::endl << FG_YELLOW << "********************************************************************************" << std::endl);

}

void RSControledAnalysisEngine::process()
{
  std::vector<std::string> desigResponse;
  process(desigResponse, query_);
  setQuery("");
}

void RSControledAnalysisEngine::process(std::vector<std::string> &designatorResponse,
                                        std::string queryString)
{
  outInfo("executing analisys engine: " << name_);
  cas->reset();
  try
  {
    UnicodeString ustrInputText;
    ustrInputText.fromUTF8(name_);
    cas->setDocumentText(uima::UnicodeStringRef(ustrInputText));

    rs::SceneCas sceneCas(*cas);
    rs::Query query = rs::create<rs::Query>(*cas);
    query.asJson.set(queryString);
    sceneCas.set("QUERY", query);

    outInfo("processing CAS");
    try
    {
      rs::StopWatch clock;

#ifdef WITH_JSON_PROLOG
      if(parallel_)
      {
        if(rspm->querySuccess)
        {
          engine->paralleledProcess(*cas);
        }
        else
        {
          outWarn("Query annotator dependency for planning failed! Fall back to linear execution!");
          engine->process(*cas);
        }
      }
      else
      {
        engine->process(*cas);
      }
#else
      engine->process(*cas);
#endif

      counter_++;
      totalTime_ += clock.getTime();
    }
    catch(const rs::FrameFilterException &)
    {
      //we could handle image logging here
      //handle extra pipeline here->signal thread that we can start processing
      outError("Got Interrputed with Frame Filter, not time here");
    }
    if(counter_ > 0)
    {
      outWarn("Avg processing time: " << totalTime_ / counter_);
      outWarn("Frames processed: " << counter_);
    }
  }
  catch(const rs::Exception &e)
  {
    outError("Exception: " << std::endl << e.what());
  }
  catch(const uima::Exception &e)
  {
    outError("Exception: " << std::endl << e);
  }
  catch(const std::exception &e)
  {
    outError("Exception: " << std::endl << e.what());
  }
  catch(...)
  {
    outError("Unknown exception!");
  }

  // Make a designator from the result
  rs::DesignatorWrapper dw(cas);
  if(useIdentityResolution_)
  {
    dw.setMode(rs::DesignatorWrapper::OBJECT);
  }
  else
  {
    dw.setMode(rs::DesignatorWrapper::CLUSTER);
  }
  dw.getObjectDesignators(designatorResponse);
  outInfo("processing finished");
}

// Call process() and decide if the pipeline should be reset or not
void RSControledAnalysisEngine::process(bool reset_pipeline_after_process)
{
  std::vector<std::string> designator_results;
  process(reset_pipeline_after_process, designator_results);
}

// Call process() and decide if the pipeline should be reset or not
void RSControledAnalysisEngine::process(bool reset_pipeline_after_process, std::vector<std::string> &designatorResponse)
{
  process_mutex->lock();
  outInfo(FG_CYAN << "process(bool,desig) - LOCK OBTAINED");
  outInfo("++++++++++++");
  outInfo(query_);
  process(designatorResponse, query_);
  if(reset_pipeline_after_process)
  {
    resetPipelineOrdering();  // reset pipeline to default
  }
  setQuery("");
  process_mutex->unlock();
  outInfo(FG_CYAN << "process(bool,desig) - LOCK RELEASED");
}


// Define a pipeline that should be executed,
// process(reset_pipeline_after_process) everything and
// decide if the pipeline should be reset or not
void RSControledAnalysisEngine::process(std::vector<std::string> annotators,
                                        bool reset_pipeline_after_process,
                                        std::vector<std::string> &designator_response,
                                        std::string queryString)
{
  process_mutex->lock();
  setNextPipeline(annotators);
  applyNextPipeline();
  process(designator_response, queryString);
  if(reset_pipeline_after_process)
    resetPipelineOrdering();  // reset pipeline to default
  process_mutex->unlock();
}

// Define a pipeline that should be executed,
// process(reset_pipeline_after_process) everything and
// decide if the pipeline should be reset or not
void RSControledAnalysisEngine::process(std::vector<std::string> annotators, bool reset_pipeline_after_process)
{
  std::vector<std::string> designator_response;
  process(annotators, reset_pipeline_after_process, designator_response, query_);
}

template <class T>
bool RSControledAnalysisEngine::drawResulstOnImage(const std::vector<bool> &filter,
    const std::vector<std::string> &resultDesignators,
    std::string &requestJson)
{

  rs::SceneCas sceneCas(*cas);
  rs::Scene scene = sceneCas.getScene();
  cv::Mat rgb = cv::Mat::zeros(480, 640, CV_64FC3);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr dispCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
  sensor_msgs::CameraInfo cam_info;

  sceneCas.get(VIEW_COLOR_IMAGE, rgb);
  sceneCas.get(VIEW_CAMERA_INFO, cam_info);
  sceneCas.get(VIEW_CLOUD, *dispCloud);

  uint64_t now = sceneCas.getScene().timestamp();
  std::vector<T> clusters;
  if(std::is_same<T, rs::Cluster>::value)
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
    if(!filter[i]) continue;

    std::string desigString = resultDesignators[i];
    rapidjson::Document desig;
    desig.Parse(desigString.c_str());
    if(desig.HasMember("id"))
    {
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

      //Color points in Point Cloud
      pcl::PointIndicesPtr inliers(new pcl::PointIndices());
      if(clusters[clusterId].points.has())
      {
        rs::conversion::from(((rs::ReferenceClusterPoints)clusters[clusterId].points()).indices(), *inliers);
        for(unsigned int idx = 0; idx < inliers->indices.size(); ++idx)
        {
          dispCloud->points[inliers->indices[idx]].rgba = rs::common::colors[colorIdx % rs::common::numberOfColors];
          dispCloud->points[inliers->indices[idx]].a = 255;
        }
      }
      colorIdx++;
    }
    if(desig.HasMember("handle"))
    {
      std::string handleKvp = desig["handle"].GetString();

      if(handleKvp != NULL)
      {
        //color the pixels of the handle
        std::vector<rs::HandleAnnotation> handles;
        scene.annotations.filter(handles);
        for(int i = 0; i < handles.size(); ++i)
        {
          outInfo("Actual name: " << handles[i].name());
          outInfo("Queried name: " << handleKvp);
          if(handles[i].name() == handleKvp)
          {
            pcl::PointIndices indices;
            rs::conversion::from(handles[i].indices(), indices);
            outInfo("Number of inliers in handle " << i << ": " << indices.indices.size());
            for(int j = 0; j < indices.indices.size(); ++j)
            {
              int idx = indices.indices[j];
              cv::Vec3b new_color;
              new_color[0] = 0;
              new_color[0] = 0;
              new_color[0] = 255;
              rgb.at<cv::Vec3b>(cv::Point(idx % 640, idx / 640)) = new_color;

              dispCloud->points[indices.indices[j]].rgba = rs::common::colors[i % rs::common::numberOfColors];
              dispCloud->points[indices.indices[j]].a = 255;
            }
          }
        }
      }
    }
  }

  rapidjson::Document request;
  request.Parse(requestJson.c_str());
  if(request.HasMember("obj-part"))
  {
    for(int i = 0; i < clusters.size(); ++i)
    {
      rs::Cluster &cluster = clusters[i];
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
            rgb.at<cv::Vec3b>(cv::Point(idx % cam_info.width, idx / cam_info.width)) = rs::common::cvVec3bColors[pIdx % rs::common::numberOfColors];
            dispCloud->points[idx].rgba = rs::common::colors[colorIdx % rs::common::numberOfColors];
            dispCloud->points[idx].a = 255;
          }
          colorIdx++;
        }
      }
    }
  }
  if(request.HasMember("ingredient") || request.HasMember("cad-model"))
  {
    if(sceneCas.has("VIEW_DISPLAY_IMAGE"))
    {
      outInfo("Scene has a display image");
      sceneCas.get("VIEW_DISPLAY_IMAGE", rgb);
    }
  }

  cv_bridge::CvImage outImgMsgs;
  outImgMsgs.header = cam_info.header;
  outImgMsgs.encoding = sensor_msgs::image_encodings::BGR8;
  outImgMsgs.image = rgb;

  std::vector<uchar> imageData;
  std::vector<int> params = {CV_IMWRITE_JPEG_QUALITY, 90, 0};
  cv::imencode(".jpg", rgb, imageData, params);

  std::string encoded = rs::common::base64_encode(&imageData[0], imageData.size());

  std_msgs::String strMsg;
  strMsg.data = "data:image/jpg;base64," + encoded;
  base64ImgPub.publish(strMsg);
  image_pub_.publish(outImgMsgs.toImageMsg());
  image_pub_.publish(outImgMsgs.toImageMsg());
  image_pub_.publish(outImgMsgs.toImageMsg());


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

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudToAdvertise(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*dsCloud, *cloudToAdvertise);
  cloudToAdvertise->header.frame_id = camToWorld.child_frame_id_; //map if localized..head_mount_kinect_rgb_optical_frame otherwise;
  //  dispCloud->header.stamp = ros::Time::now().toNSec();
  pc_pub_.publish(cloudToAdvertise);
  return true;
}

template bool RSControledAnalysisEngine::drawResulstOnImage<rs::Object>(const std::vector<bool> &filter,
    const std::vector<std::string> &resultDesignators,
    std::string &requestJson);

template bool RSControledAnalysisEngine::drawResulstOnImage<rs::Cluster>(const std::vector<bool> &filter,
    const std::vector<std::string> &resultDesignators,
    std::string &requestJson);
