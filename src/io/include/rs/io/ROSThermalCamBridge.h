#ifndef __ROS_THERMAL_CAM_BRIDGE_H__
#define __ROS_THERMAL_CAM_BRIDGE_H__

// ROS
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/image_transport.h>

//OpenCV
#include <opencv2/opencv.hpp>

// RS
#include <rs/io/ROSCamInterface.h>

typedef message_filters::sync_policies::ApproximateTime < sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> ThermalSyncPolicy;

class ROSThermalCamBridge : public ROSCamInterface
{
private:
  void initSpinner();
  void readConfig(const boost::property_tree::ptree &pt);

  message_filters::Synchronizer<ThermalSyncPolicy> *sync;
  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *thermalImageSubscriber;
  image_transport::SubscriberFilter *thermalDepthImageSubscriber;
  image_transport::SubscriberFilter *thermalRGBImageSubscriber;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *cameraInfoSubscriber;

  void cb_(const sensor_msgs::Image::ConstPtr thermal_img_msg,
           const sensor_msgs::Image::ConstPtr thermal_rgb_img_msg,
           const sensor_msgs::Image::ConstPtr thermal_depth_img_msg,
           const sensor_msgs::CameraInfo::ConstPtr camera_info_msg);

  cv::Mat thermalImage;
  cv::Mat thermalRGBImage;
  cv::Mat thermalDepthImage;

  sensor_msgs::CameraInfo cameraInfo;

public:
  ROSThermalCamBridge(const boost::property_tree::ptree &pt);
  ~ROSThermalCamBridge();

  bool setData(uima::CAS &tcas, uint64_t ts = 0);
  void lookupTransform(uima::CAS &tcas, const ros::Time &timestamp);
  void maskImage(cv::Mat &image, int xMin, int yMin, int xMax, int yMax);
};

#endif // __ROS_THERMAL_CAM_BRIDGE_H__
