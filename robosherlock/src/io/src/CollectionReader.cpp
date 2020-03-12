/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
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

// UIMA
#include <uima/api.hpp>

// ROS
#include <ros/ros.h>

// RS
#include <robosherlock/io/TFBroadcasterWrapper.hpp>

#include <robosherlock/io/CollectionReader.h>
#include <robosherlock/utils/time.h>
#include <robosherlock/scene_cas.h>

// RapidJson
#include <rapidjson/document.h>
#include <rapidjson/pointer.h>

using namespace uima;

class CollectionReaderAnnotator : public Annotator
{

private:


  std::vector<CollectionReader> cameras_;
  std::vector<std::string> interfaces_;

  std::thread thread_;
  TFBroadcasterWrapper broadCasterObject_;

  uint64_t convertToInt(const std::string &s) const
  {
    return std::stoll(s);
  }

  ~CollectionReaderAnnotator()
  {
    broadCasterObject_.terminate();
    thread_.join();
  }

public:
  TyErrorId initialize(AnnotatorContext &ctx)
  {
    if(!ros::ok())
      ros::init(ros::M_string(), std::string("RS_CollectionReader"));

    outInfo("initialize");
    if(ctx.isParameterDefined("camera_config_files"))
    {
      std::vector<std::string *> configs;
      ctx.extractValue("camera_config_files", configs);
      for(size_t i = 0; i < configs.size(); ++i)
      {
        CollectionReader collRead(*configs[i]);
        interfaces_.push_back(collRead.interface_type_);
        cameras_.push_back(collRead);
      }
    }

    thread_ = std::thread(&TFBroadcasterWrapper::run, &broadCasterObject_);
//    thread_.detach();
    //this needs to be set in order to rewrite parameters
    setAnnotatorContext(ctx);

    return UIMA_ERR_NONE;
  }


  TyErrorId reconfigure()
  {
    outInfo("Reconfiguring");
    AnnotatorContext &ctx = getAnnotatorContext();
    if(ctx.isParameterDefined("camera_config_files"))
    {
      cameras_.clear();
      interfaces_.clear();
      rs::SceneCas::unregisterAllCameraIDs();
      std::vector<std::string *> configs;
      ctx.extractValue("camera_config_files", configs);
      for(size_t i = 0; i < configs.size(); ++i)
      {
        outInfo(*configs[i]);
        CollectionReader collRead(*configs[i]);
        interfaces_.push_back(collRead.interface_type_);
        cameras_.push_back(collRead);
      }
    }
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);

    uint64_t timestamp = std::numeric_limits<uint64_t>::max();

    rs::Query qs = rs::create<rs::Query>(tcas);
    if(cas.getFS("QUERY", qs) && qs.query() != "")
    {
      rapidjson::Document jsonDoc;
      std::string jsonString  = qs.query();
      jsonDoc.Parse(jsonString);

      //TODO Is timestamp nested in something or right under detect?
      rapidjson::Pointer framePointer("/detect/timestamp");
      rapidjson::Value *tsJson = framePointer.Get(jsonDoc);

      std::string newTS;
      if(tsJson && tsJson->IsString())
      {
        newTS = tsJson->GetString();
        newTS = newTS.substr(0, newTS.find_first_of("\""));
        outInfo(newTS);
        if(newTS != "")
        {
          timestamp = atol(newTS.c_str());
        }
      }
    }

    // Check that all camera interface have an ID assigned and that the SceneCAS knows about it
    for(size_t i = 0; i < cameras_.size(); ++i)
    {
      // Check if camera interfaces are already registered in the SceneCAS
      if( !(cameras_[i].cam_interface_->cameraIdAlreadyRegistered()) ){

        cameras_[i].cam_interface_->registerCameraInCAS(cas.getIdentifier());
        outInfo("New Camera Interface in AAE: '" << cas.getIdentifier() << "' discovered. Camera ID: "  << FG_BLUE << cameras_[i].cam_interface_->getCameraId() );
      }
    }

    outInfo("waiting for all cameras to have new data in AE " << cas.getIdentifier() <<" ...");
    double t1 = clock.getTime();
    for(size_t i = 0; i < cameras_.size(); ++i)
    {
      while(!cameras_[i].cam_interface_->newData())
      {
        usleep(100);
        check_ros();
      }
    }
    outDebug("Cameras got new data after waiting " << clock.getTime() - t1 << " ms. Receiving...");



    for(size_t i = 0; i < cameras_.size(); ++i)
    {
      bool ret = cameras_[i].cam_interface_->setData(tcas, timestamp);
      check_expression(ret, "Could not receive data from camera.");
    }

    if(std::find(interfaces_.begin(), interfaces_.end(), "MongoDB") != interfaces_.end() ||
       std::find(interfaces_.begin(), interfaces_.end(), "DataLoader") != interfaces_.end())
    {
      outDebug("Broadcasting TF for cameraPose");
      rs::SceneCas scenecas(tcas);
      rs::Scene scene = scenecas.getScene();
      tf::StampedTransform camToWorld;
      rs::conversion::from(scene.viewPoint(), camToWorld);
      camToWorld.stamp_= ros::Time::now();
      broadCasterObject_.clear();
      broadCasterObject_.addTransform(camToWorld);
    }

    return UIMA_ERR_NONE;
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(CollectionReaderAnnotator)
