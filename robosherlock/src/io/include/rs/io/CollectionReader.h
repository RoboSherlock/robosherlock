#ifndef utils_io_H
#define utils_io_H

#include <sys/stat.h>
#include <memory>
#include <string>
#include <vector>

#include <rs/io/CamInterface.h>
#include <rs/io/ROSKinectBridge.h>
#include <rs/io/ROSRealSenseBridge.h>
#include <rs/io/ROSTangoBridge.h>
#include <rs/io/ROSCameraBridge.h>
#include <rs/io/ROSThermalCamBridge.h>
#include <rs/io/MongoDBBridge.h>
#include <rs/io/UnrealVisionBridge.h>
#include <rs/io/UnrealROSIntegrationVisionBridge.h>
#include <rs/io/DataLoaderBridge.h>

#include <rs/utils/exception.h>

// Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/filesystem.hpp>


//ROS
#include <ros/package.h>

class CollectionReader
{

public:

  std::shared_ptr<CamInterface> cam_interface_;
  std::string interface_type_;

  CollectionReader(std::string config_file)
  {
    readConfig(config_file);
  }

  ~CollectionReader(){
  } 

  void readAndSetData(uima::CAS &tcas, uint64_t timestamp = std::numeric_limits<uint64_t>::max())
  {
    rs::StopWatch clock;
    outInfo("waiting for all cameras to have new data...");
    double t1 = clock.getTime();
    while(!cam_interface_->newData())
      usleep(100);
    outInfo("Cameras got new data after waiting " << clock.getTime() - t1 << " ms. Receiving...");
    bool ret = cam_interface_->setData(tcas, timestamp);
    check_expression(ret, "Could not receive data from camera.");
  }

  std::string getConfigFilePath(const std::string &file)
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

  void readConfig(const std::string &file)
  {
    bool found = boost::filesystem::is_regular_file(boost::filesystem::path(file));
    std::string configFile;
    if(found)
      configFile = file;
    else
      configFile = getConfigFilePath(file);

    if(configFile.empty())
    {
      outError("Camera config file not found: " + file);
      throw std::runtime_error("Camera config file not found: " + file);
    }

    outInfo("Path to config file: " FG_BLUE << configFile);
    boost::property_tree::ptree pt;
    std::string interface;
    try
    {
      boost::property_tree::ini_parser::read_ini(configFile, pt);
      interface_type_ = pt.get<std::string>("camera.interface");

      if(interface_type_ == "MongoDB")
        cam_interface_ = std::make_shared<MongoDBBridge>(pt);
      else if(interface_type_ == "Kinect")
        cam_interface_ = std::make_shared<ROSKinectBridge>(pt);
      else if(interface_type_ == "RealSense")
        cam_interface_  = std::make_shared<ROSRealSenseBridge>(pt);
      else if(interface_type_ == "Tango")
        cam_interface_ = std::make_shared<ROSTangoBridge>(pt);
      else if(interface_type_ == "Thermal")
        cam_interface_ = std::make_shared<ROSThermalCamBridge>(pt);
      else if(interface_type_ == "Camera")
        cam_interface_ = std::make_shared<ROSCameraBridge>(pt);
      else if(interface_type_ == "UnrealVision")
        cam_interface_ = std::make_shared<UnrealVisionBridge>(pt);
      else if(interface_type_ == "UnrealROSIntegrationVision")
        cam_interface_ = std::make_shared<UnrealROSIntegrationVisionBridge>(pt);
      else if(interface_type_ == "DataLoader")
        cam_interface_ = std::make_shared<DataLoaderBridge>(pt);
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

};



#endif

