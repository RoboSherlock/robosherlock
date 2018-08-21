#ifndef RSCONTROLEDANALYSISENGINE_H
#define RSCONTROLEDANALYSISENGINE_H

#include <rs/utils/common.h>
#include <rs/flowcontrol/RSAnalysisEngine.h>
#include <rs/queryanswering/DesignatorWrapper.h>
#include <rs/scene_cas.h>

#include <std_msgs/String.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <tf_conversions/tf_eigen.h>

#include <fstream>
#include <uima/api.hpp>
#include <rs/queryanswering/JsonPrologInterface.h>


class RSControledAnalysisEngine: public RSAnalysisEngine
{

private:
  std::mutex process_mutex;


#ifdef WITH_JSON_PROLOG
  JsonPrologInterface jsonPrologInterface;
#endif

  int counter_;
  double totalTime_;
  float avgProcessingTime_;

public:

  RSControledAnalysisEngine(ros::NodeHandle nh) : RSAnalysisEngine(),
   counter_(0),totalTime_(0.0),avgProcessingTime_(0.0f)
  {
  }

  ~RSControledAnalysisEngine(){}


  void init(const std::string &file, std::vector<std::string> &lowLvLPipeline, bool pervasive, bool parallel);

  void process();

  void process(std::vector<std::string> &designator_response,
               std::string query);

  //draw results on an image
  template <class T>
  bool drawResulstOnImage(const std::vector<bool> &filter,
                          const std::vector<std::string> &resultDesignators,
                          std::string &requestJson,
                          cv::Mat &resImage);
};
#endif // RSCONTROLEDANALYSISENGINE_H
