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

// STD
#include <sys/stat.h>

// UIMA
#include <uima/api.hpp>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

// RS
#include <rs/io/TFBroadcasterWrapper.hpp>
#include <rs/io/ROSKinectBridge.h>
#include <rs/io/ROSTangoBridge.h>
#include <rs/io/ROSCameraBridge.h>
#include <rs/io/ROSThermalCamBridge.h>
#include <rs/io/MongoDBBridge.h>
#include <rs/io/UnrealVisionBridge.h>
#include <rs/io/UnrealROSIntegrationVisionBridge.h>
#include <rs/io/DataLoaderBridge.h>
#include <rs/utils/time.h>
#include <rs/utils/exception.h>
#include <rs/scene_cas.h>

// RapidJson
#include <rapidjson/document.h>
#include <rapidjson/pointer.h>


using namespace uima;

class CollectionReader : public Annotator
{

private:

  struct SemanticMapItem
  {
    std::string name, type;
    double width, height, depth;
    tf::Transform transform;
  };
  std::vector<SemanticMapItem> semanticMapItems_;
  std::vector<CamInterface *> cameras_;
  std::string interface_;

  std::thread thread_;
  TFBroadcasterWrapper broadCasterObject_;

  uint64_t convertToInt(const std::string &s) const
  {
    return std::stoll(s);
  }

  std::string getFilePath(const std::string &file)
  {
    const std::string configDir = "/config/";
    //generate a vector of possible paths for the analysis engine
    std::vector<std::string> searchPaths;

    //empty path for full path given as argument
    searchPaths.push_back("");
    //add core package path
    searchPaths.push_back(ros::package::getPath("robosherlock") + configDir);

    //look for packages dependent on core and find their full path
    std::vector<std::string> child_packages;
    ros::package::command("depends-on robosherlock", child_packages);
    for(int i = 0; i < child_packages.size(); ++i)
    {
      searchPaths.push_back(ros::package::getPath(child_packages[i]) + configDir);
    }

    struct stat fileStat;
    for(size_t i = 0; i < searchPaths.size(); ++i)
    {
      const std::string filePath = searchPaths[i] + file;

      if(!stat(filePath.c_str(), &fileStat) && S_ISREG(fileStat.st_mode))
      {
        return filePath;
      }
    }
    return "";
  }

  void  readConfig(const std::string &file)
  {
    const std::string &configFile = getFilePath(file);
    if(configFile.empty())
    {
      outError("Camera config file not found: " + file);
      throw std::runtime_error("Camera config file not found: " + file);
    }

    outInfo("Path to config file: " FG_BLUE << configFile);
    boost::property_tree::ptree pt;
    try
    {
      boost::property_tree::ini_parser::read_ini(configFile, pt);

      boost::optional<std::string> semanticMapFile = pt.get_optional<std::string>("tf.semanticMap");
      if(semanticMapFile)
      {
        readSemanticMap(semanticMapFile.get());
      }

      interface_ = pt.get<std::string>("camera.interface");

      if(interface_ == "MongoDB")
      {
        cameras_.push_back(new MongoDBBridge(pt));
      }
      else if(interface_ == "Kinect")
      {
        cameras_.push_back(new ROSKinectBridge(pt));
      }
      else if(interface_ == "Tango")
      {
        cameras_.push_back(new ROSTangoBridge(pt));
      }
      else if(interface_ == "Thermal")
      {
        cameras_.push_back(new ROSThermalCamBridge(pt));
      }
      else if(interface_ == "Camera")
      {
        cameras_.push_back(new ROSCameraBridge(pt));
      }
      else if(interface_ == "UnrealVision")
      {
        cameras_.push_back(new UnrealVisionBridge(pt));
      }
      else if(interface_ == "UnrealROSIntegrationVision")
      {
        cameras_.push_back(new UnrealROSIntegrationVisionBridge(pt));
      }
      else if(interface_ == "DataLoader")
      {
        cameras_.push_back(new DataLoaderBridge(pt));
      }
      else
      {
        throw_exception_message("Unknown camera interface");
      }
    }
    catch(boost::property_tree::ini_parser::ini_parser_error &e)
    {
      throw_exception_message("Error opening config file: " + configFile);
    }
  }

  void readSemanticMap(const std::string &file)
  {
    const std::string &mapFile = getFilePath(file);
    if(mapFile.empty())
    {
      throw_exception_message("Semantic map file not found: " + file);
    }

    outInfo("Path to semantic map file: " FG_BLUE << mapFile);
    cv::FileStorage fs(mapFile, cv::FileStorage::READ);
    std::vector<std::string> names;

    cv::FileNode n = fs["names"];
    for(cv::FileNodeIterator it = n.begin(); it != n.end(); ++it)
    {
      names.push_back((std::string)(*it));
    }

    semanticMapItems_.resize(names.size());
    for(size_t i = 0; i < names.size(); ++i)
    {
      SemanticMapItem &item = semanticMapItems_[i];
      cv::FileNode entry = fs[names[i]];

      item.name = names[i];
      entry["type"] >> item.type;
      entry["width"] >> item.height;
      entry["height"] >> item.depth;
      entry["depth"] >> item.width;

      cv::Mat m;
      entry["transform"] >> m;

      // move frame from center to corner
      /*cv::Mat r = m(cv::Rect(0, 0, 3, 3)).inv();
      cv::Mat t(3, 1, CV_64F);
      t.at<double>(0) = -item.depth / 2;
      t.at<double>(1) = -item.width / 2;
      t.at<double>(2) = -item.height / 2;
      t = r * t;
      m.at<double>(0, 3) += t.at<double>(0);
      m.at<double>(1, 3) += t.at<double>(1);
      m.at<double>(2, 3) += t.at<double>(2);*/

      tf::Matrix3x3 rot;
      tf::Vector3 trans;
      rot.setValue(m.at<double>(0, 0), m.at<double>(0, 1), m.at<double>(0, 2), m.at<double>(1, 0), m.at<double>(1, 1), m.at<double>(1, 2), m.at<double>(2, 0), m.at<double>(2, 1), m.at<double>(2, 2));
      trans.setValue(m.at<double>(0, 3), m.at<double>(1, 3), m.at<double>(2, 3));
      item.transform = tf::Transform(rot, trans);
    }
  }

  ~CollectionReader()
  {
    broadCasterObject_.terminate();
    thread_.join();
  }

public:
  TyErrorId initialize(AnnotatorContext &ctx)
  {
    if(!ros::ok())
    {
      ros::init(ros::M_string(), std::string("RS_CollectionReader"));
    }
    outInfo("initialize");

    if(ctx.isParameterDefined("camera_config_files"))
    {
      std::vector<std::string *> configs;
      ctx.extractValue("camera_config_files", configs);
      for(size_t i = 0; i < configs.size(); ++i)
      {
        readConfig(*configs[i]);
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
    outError("Reconfiguring");
    AnnotatorContext &ctx = getAnnotatorContext();
    if(ctx.isParameterDefined("camera_config_files"))
    {
      for(size_t i = 0; i < cameras_.size(); ++i)
      {
        delete cameras_[i];
      }
      cameras_.clear();

      std::vector<std::string *> configs;
      ctx.extractValue("camera_config_files", configs);
      for(size_t i = 0; i < configs.size(); ++i)
      {
        outError(*configs[i]);
        readConfig(*configs[i]);
      }
    }
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    for(size_t i = 0; i < cameras_.size(); ++i)
    {
      delete cameras_[i];
    }
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);

    mongo::OID oid = mongo::OID::gen();
    std::vector<rs::SemanticMapObject> semanticMap;
    semanticMap.reserve(semanticMapItems_.size());
    for(size_t i = 0; i < semanticMapItems_.size(); ++i)
    {
      SemanticMapItem &item = semanticMapItems_[i];
      rs::SemanticMapObject obj = rs::create<rs::SemanticMapObject>(tcas);
      obj.name(item.name);
      obj.typeName(item.type);
      obj.width(item.width);
      obj.height(item.height);
      obj.depth(item.depth);
      obj.transform(rs::conversion::to(tcas, item.transform));
      semanticMap.push_back(obj);
    }
    cas.set(VIEW_SEMANTIC_MAP, semanticMap);

    uint64_t timestamp = std::numeric_limits<uint64_t>::max();

    rs::Query qs = rs::create<rs::Query>(tcas);
    if(cas.getFS("QUERY", qs) && qs.asJson() != "")
    {
      rapidjson::Document jsonDoc;
      std::string jsonString  = qs.asJson();
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

    outInfo("waiting for all cameras to have new data...");
    double t1 = clock.getTime();
    for(size_t i = 0; i < cameras_.size(); ++i)
    {
      while(!cameras_[i]->newData())
      {
        usleep(100);
        check_ros();
      }
    }
    outInfo("Cameras got new data after waiting " << clock.getTime() - t1 << " ms. Receiving...");

    for(size_t i = 0; i < cameras_.size(); ++i)
    {
      bool ret = cameras_[i]->setData(tcas, timestamp);
      check_expression(ret, "Could not receive data from camera.");
    }

    if(interface_ == "MongoDB")
    {
      outInfo("Broadcasting TF for cameraPose");
      rs::SceneCas scenecas(tcas);
      rs::Scene scene = scenecas.getScene();
      tf::StampedTransform camToWorld;
      rs::conversion::from(scene.viewPoint(), camToWorld);
      broadCasterObject_.clear();
      broadCasterObject_.addTransform(camToWorld);
    }

    return UIMA_ERR_NONE;
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(CollectionReader)
