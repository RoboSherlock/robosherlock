/*
 *
 * Copyright (c) 2011, Nico Blodow <blodow@cs.tum.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// UIMA
#include <uima/api.hpp>

// ROS
#include <ros/ros.h>

// Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

// RS
#include <rs/io/ROSKinectBridge.h>
#include <rs/io/ROSCameraBridge.h>
#include <rs/io/ROSThermalCamBridge.h>
#include <rs/io/MongoDBBridge.h>
#include <rs/utils/time.h>
#include <rs/utils/exception.h>
#include <rs/scene_cas.h>


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
  std::vector<SemanticMapItem> semanticMapItems;

  std::vector<CamInterface *> cameras;

  uint64_t convertToInt(const std::string &s) const
  {
    return std::stoll(s);
  }

  void readConfig(const std::string &file)
  {
    std::string path_to_config_file = CAMERA_CONFIG_PATH + file;
    outInfo("Path to config file: " FG_BLUE << file);
    boost::property_tree::ptree pt;
    try
    {
      boost::property_tree::ini_parser::read_ini(path_to_config_file, pt);

      boost::optional<std::string> semanticMapFile = pt.get_optional<std::string>("tf.semanticMap");
      if(semanticMapFile)
      {
        readSemanticMap(CAMERA_CONFIG_PATH + semanticMapFile.get());
      }

      const std::string &interface = pt.get<std::string>("camera.interface");

      if(interface == "MongoDB")
      {
        cameras.push_back(new MongoDBBridge(pt));
      }
      else if(interface == "Kinect")
      {
        cameras.push_back(new ROSKinectBridge(pt));
      }
      else if(interface == "Thermal")
      {
        cameras.push_back(new ROSThermalCamBridge(pt));
      }
      else if(interface == "Camera")
      {
        cameras.push_back(new ROSCameraBridge(pt));
      }
      else
      {
        throw_exception_message("Unknown camera interface");
      }
    }
    catch(boost::property_tree::ini_parser::ini_parser_error &e)
    {
      throw_exception_message("Error opening config file: " + path_to_config_file);
    }
  }

  void readSemanticMap(const std::string &file)
  {
    cv::FileStorage fs(file, cv::FileStorage::READ);

    std::vector<std::string> names;
    fs["names"] >> names;

    semanticMapItems.resize(names.size());
    for(size_t i = 0; i < names.size(); ++i)
    {
      SemanticMapItem &item = semanticMapItems[i];
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
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    for(size_t i = 0; i < cameras.size(); ++i)
    {
      delete cameras[i];
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

    std::vector<rs::SemanticMapObject> semanticMap;
    semanticMap.reserve(semanticMapItems.size());
    for(size_t i = 0; i < semanticMapItems.size(); ++i)
    {
      SemanticMapItem &item = semanticMapItems[i];
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

    uint64_t timestamp = 0;

    rs::Query qs = rs::create<rs::Query>(tcas);
    if(cas.get("QUERY", qs))
    {
      outWarn("TIMESTAMP SET IN runAE: " << qs.timestamp());
      timestamp = qs.timestamp();
    }



    outInfo("waiting for all cameras to have new data...");
    double t1 = clock.getTime();
    for(size_t i = 0; i < cameras.size(); ++i)
    {
      while(!cameras[i]->newData())
      {
        usleep(100);
        check_ros();
      }
    }
    outInfo("Cameras got new data after waiting " << clock.getTime() - t1 << " ms. Receiving...");

    for(size_t i = 0; i < cameras.size(); ++i)
    {
      bool ret = cameras[i]->setData(tcas, timestamp);
      check_expression(ret, "Could not receive data from camera.");
    }

    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(CollectionReader)

