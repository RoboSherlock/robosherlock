//ROS
#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.h>

//RS
#include <rs/io/ROSTangoBridge.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>

//OpenCV
#include <cv_bridge/cv_bridge.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

ROSTangoBridge::ROSTangoBridge(const boost::property_tree::ptree &pt) : ROSCamInterface(pt), it(nodeHandle)
{
  readConfig(pt);
  initSpinner();
}

ROSTangoBridge::~ROSTangoBridge()
{
  spinner.stop();
  delete colorImageSubscriber;
  delete fisheyeImageSubscriber;
  delete colorInfoSubscriber;
  delete fisheyeInfoSubscriber;
}

void ROSTangoBridge::initSpinner()
{
  spinner.start();
}

void ROSTangoBridge::readConfig(const boost::property_tree::ptree &pt)
{
  std::string cloud_topic = pt.get<std::string>("cloud_topic.cloud");
  std::string color_topic = pt.get<std::string>("camera_topics.color");
  std::string fisheye_topic = pt.get<std::string>("camera_topics.fisheye");
  std::string color_hints = pt.get<std::string>("camera_topics.colorHints", "raw");
  std::string fisheye_hints = pt.get<std::string>("camera_topics.fisheyeHints", "raw");
  std::string color_info_topic = pt.get<std::string>("camera_topics.colorInfo");
  std::string fisheye_info_topic = pt.get<std::string>("camera_topics.fisheyeInfo");

  image_transport::TransportHints hintsColor(color_hints);
  image_transport::TransportHints hintsFisheye(fisheye_hints);

  colorImageSubscriber = new image_transport::SubscriberFilter(it, color_topic, 5, hintsColor);
  fisheyeImageSubscriber = new image_transport::SubscriberFilter(it, fisheye_topic, 5, hintsFisheye);

  colorInfoSubscriber = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nodeHandle, color_info_topic, 5);
  fisheyeInfoSubscriber = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nodeHandle, fisheye_info_topic, 5);

  cloud_sub = nodeHandle.subscribe(cloud_topic, 1000, &ROSTangoBridge::cloudCb_, this);

  outInfo("  Cloud topic: " FG_BLUE << cloud_topic);
  outInfo("  Color topic: " FG_BLUE << color_topic);
  outInfo("  Fisheye topic: " FG_BLUE << fisheye_topic);
  outInfo("  ColorCamInfo topic: " FG_BLUE << color_info_topic);
  outInfo("  FisheyeCamInfo topic: " FG_BLUE << fisheye_info_topic);
  outInfo("  Color Hints: " FG_BLUE << color_hints);
  outInfo("  Fisheye Hints: " FG_BLUE << fisheye_hints);
  //Adding the depthOffset, filterBlurredImages and scale in here
}

void ROSTangoBridge::cb_(const sensor_msgs::Image::ConstPtr color_img_msg,
                         const sensor_msgs::Image::ConstPtr fisheye_img_msg,
                         const sensor_msgs::CameraInfo::ConstPtr color_info_msg,
                         const sensor_msgs::CameraInfo::ConstPtr fisheye_info_msg)
{
  cv::Mat color, fisheye;
  sensor_msgs::CameraInfo colorCameraInfo, fisheyeCameraInfo;

  cv_bridge::CvImageConstPtr orig_color_img;
  orig_color_img = cv_bridge::toCvShare(color_img_msg, sensor_msgs::image_encodings::BGR8);
  colorCameraInfo = sensor_msgs::CameraInfo(*color_info_msg);
  if(!lookupTransform(colorCameraInfo.header.stamp))
  {
    lock.lock();
    _newData = false;
    lock.unlock();
    return;
  }

  cv_bridge::CvImageConstPtr orig_fisheye_img;
  orig_fisheye_img = cv_bridge::toCvShare(fisheye_img_msg, sensor_msgs::image_encodings::BGR8);
  fisheyeCameraInfo = sensor_msgs::CameraInfo(*fisheye_info_msg);
  if(!lookupTransform(fisheyeCameraInfo.header.stamp))
  {
    lock.lock();
    _newData = false;
    lock.unlock();
    return;
  }

  color = orig_color_img->image.clone();
  fisheye = orig_fisheye_img->image.clone();

  lock.lock();
  this->color = color;
  this->fisheye = fisheye;
  this->colorCameraInfo = colorCameraInfo;
  this->fisheyeCameraInfo = fisheyeCameraInfo;
  _newData = true;
  lock.unlock();
}

void ROSTangoBridge::cloudCb_(const sensor_msgs::PointCloud2 cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(cloud_msg, cloud);
  lock.lock();
  this->cloud = cloud;
  _newData = true;
  lock.unlock();
}

bool ROSTangoBridge::setData(uima::CAS &tcas, uint64_t ts)
{
  if(!newData())
  {
    return false;
  }
  MEASURE_TIME;

  cv::Mat color, fisheye;
  sensor_msgs::CameraInfo colorCameraInfo, fisheyeCameraInfo;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  lock.lock();
  color = this->color;
  fisheye = this->fisheye;
  colorCameraInfo = this->colorCameraInfo;
  fisheyeCameraInfo = this->fisheyeCameraInfo;
  cloud = this->cloud;
  _newData = false;
  lock.unlock();

  rs::SceneCas cas(tcas);
  setTransformAndTime(tcas);

  cas.set(VIEW_CLOUD, cloud);
  cas.set(VIEW_COLOR_IMAGE, color);//??????
  cas.set(VIEW_FISHEYE_IMAGE, fisheye);
  cas.set(VIEW_COLOR_CAMERA_INFO, colorCameraInfo);//????ß
  cas.set(VIEW_FISHEYE_CAMERA_INFO, fisheyeCameraInfo);

  return true;
}
