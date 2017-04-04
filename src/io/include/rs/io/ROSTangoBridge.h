#ifndef __ROS_TANGO_BRIDGE_H__
#define __ROS_TANGO_BRIDGE_H__

//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// RS
#include <rs/io/ROSCamInterface.h>

class ROSTangoBridge : public ROSCamInterface
{
private:
  ros::Subscriber cloud_sub;
  void initSpinner();
  void readConfig(const boost::property_tree::ptree &pt);
  void cb_(const sensor_msgs::PointCloud2 &input);

public:
  ROSTangoBridge(const boost::property_tree::ptree &pt);
  ~ROSTangoBridge();

  bool setData(uima::CAS &tcas, u_int64_t = std::numeric_limits<uint64_t>::max());

};

#endif // __ROS_TANGO_BRIDGE_H__
