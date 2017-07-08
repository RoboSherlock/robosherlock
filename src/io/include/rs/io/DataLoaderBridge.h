#ifndef DATA_LOADER_BRIDGE_H
#define DATA_LOADER_BRIDGE_H

#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>
#include <rs/io/CamInterface.h>

#include <opencv2/opencv.hpp>

//ROS msg
#include <sensor_msgs/CameraInfo.h>


class DataLoaderBridge : public CamInterface
{
private:
  std::string path_to_cloud;
  std::string path_to_rgb;
  std::string path_to_depth;

  std::vector<std::string> clouds;
  std::vector<std::string> images;
  std::vector<std::string> depths;

  bool isLoop;

  bool isCloudFile;
  bool isRGBFile;
  bool isDepthFile;

  bool haveCloud;
  bool haveRGB;
  bool haveDepth;

  cv::Mat color;
  cv::Mat depth;

  int iterator; // aka frameID
  int data_size;

  void readConfig(const boost::property_tree::ptree &pt);
  void getListFile(std::string& path, std::vector<std::string>& filenames, std::string& pattern, bool& isFile);
  bool checkConsistency();
public:
  DataLoaderBridge(const boost::property_tree::ptree& pt);
  ~DataLoaderBridge();

  bool setData(uima::CAS &tcas, uint64_t ts);
  inline void getColorImage(cv::Mat& c)
  {
    c = this->color.clone();
  }

  inline void getDepthImage(cv::Mat& d)
  {
    d = this->depth.clone();
  }
};


#endif
