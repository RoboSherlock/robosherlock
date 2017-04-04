//ROS
#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>

//RS
#include <rs/io/ROSTangoBridge.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

ROSTangoBridge::ROSTangoBridge(const boost::property_tree::ptree &pt) : ROSCamInterface(pt)
{
  readConfig(pt);
  initSpinner();
}

ROSTangoBridge::~ROSTangoBridge()
{
  spinner.stop();
}

void ROSTangoBridge::initSpinner()
{
spinner.start();
}

void ROSTangoBridge::readConfig(const boost::property_tree::ptree &pt)
{
  std::string cloud_topic = pt.get<std::string>("/tango/point_cloud");

  cloud_sub = nodeHandle.subscribe(cloud_topic, 1000, &ROSTangoBridge::cb_, this);
}

void ROSTangoBridge::cb_(const sensor_msgs::PointCloud2 &input)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_in;
  pcl::fromROSMsg(input, cloud_in);
}

bool ROSTangoBridge::setData(uima::CAS &tcas, uint64_t ts)
{

}
