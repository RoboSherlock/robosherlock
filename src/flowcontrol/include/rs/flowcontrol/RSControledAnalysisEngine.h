#ifndef RSCONTROLEDANALYSISENGINE_H
#define RSCONTROLEDANALYSISENGINE_H

#include <rs/utils/common.h>
#include <rs/flowcontrol/RSAnalysisEngine.h>
#include <rs/scene_cas.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <tf_conversions/tf_eigen.h>

#include <rs/queryanswering/DesignatorWrapper.h>

#include <fstream>

#include <uima/api.hpp>

#include <rs/queryanswering/JsonPrologInterface.h>


class RSControledAnalysisEngine: public RSAnalysisEngine
{

private:
  std::vector<std::string> next_pipeline_order;
  std::mutex process_mutex;

  std::string query_;

  ros::NodeHandle nh_;
  ros::Publisher base64ImgPub;
  ros::Publisher pc_pub_;
  image_transport::Publisher image_pub_;
  image_transport::ImageTransport it_;

#ifdef WITH_JSON_PROLOG
  JsonPrologInterface jsonPrologInterface;
#endif

  bool useIdentityResolution_;
  int counter_;
  double totalTime_;
  float avgProcessingTime_;

public:

  RSControledAnalysisEngine(ros::NodeHandle nh) : RSAnalysisEngine(),
    query_(""),nh_(nh),it_(nh_),useIdentityResolution_(false),counter_(0),totalTime_(0.0),avgProcessingTime_(0.0f)
  {
    base64ImgPub = nh_.advertise<std_msgs::String>(std::string("image_base64"), 5);
    image_pub_ = it_.advertise("result_image", 1, true);
    pc_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("points", 5 );
  }

  ~RSControledAnalysisEngine(){}

  /*set the next order of AEs to be executed*/
  void setNextPipeline(std::vector<std::string> l)
  {
    next_pipeline_order = l;
  }


  void setQuery(std::string q)
  {
    query_ = q;
  }

  /*get the next order of AEs to be executed*/
  inline std::vector<std::string> &getNextPipeline()
  {
    return next_pipeline_order;
  }

  inline void changeLowLevelPipeline(std::vector<std::string> &pipeline)
  {
    engine->setContinuousPipelineOrder(pipeline);
    engine->setPipelineOrdering(pipeline);
  }

  inline void applyNextPipeline()
  {
    if(engine)
    {
      engine->setPipelineOrdering(next_pipeline_order);
    }
  }

  inline void resetPipelineOrdering()
  {
    if(engine)
    {
      engine->resetPipelineOrdering();
    }
  }

  inline std::string getCurrentAEName()
  {
    return name_;
  }

  bool defaultPipelineEnabled()
  {
    if(engine)
    {
      return engine->use_default_pipeline_;
    }
    return false;
  }

  inline void useIdentityResolution(const bool useIDres)
  {
      useIdentityResolution_=useIDres;
  }


  void init(const std::string &file, std::vector<std::string> &lowLvLPipeline, bool pervasive, bool parallel);

  void process();

  void process(std::vector<std::string> &designator_response,
               std::string query);

  //draw results on an image
  template <class T>
  bool drawResulstOnImage(const std::vector<bool> &filter,
                          const std::vector<std::string> &resultDesignators,
                          std::string &requestJson);
};
#endif // RSCONTROLEDANALYSISENGINE_H
