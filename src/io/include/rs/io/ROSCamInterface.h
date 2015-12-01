#ifndef __ROS_CAM_INTERFACE2_H__
#define __ROS_CAM_INTERFACE2_H__

// UIMA
#include <uima/api.hpp>

// ROS
#include <tf/tf.h>
#include <tf/transform_listener.h>

// RS
#include <rs/io/CamInterface.h>

// STL
#include <mutex>

class ROSCamInterface : public CamInterface
{
private:

protected:
  tf::TransformListener *listener;
  std::string tfFrom, tfTo;
  bool lookUpViewpoint;
  ros::AsyncSpinner spinner;
  ros::NodeHandle nodeHandle;

  std::mutex lock;

public:
  ~ROSCamInterface();

protected:
  ROSCamInterface(const boost::property_tree::ptree &pt);

  void lookupTransform(uima::CAS &tcas, const ros::Time &timestamp);
};

#endif // __ROS_CAM_INTERFACE2_H__
