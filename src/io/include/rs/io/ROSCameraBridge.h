#ifndef __ROS_CAMERA_BRIDGE_H__
#define __ROS_CAMERA_BRIDGE_H__

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
#include <rs/utils/BlurDetector.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> RGBSyncPolicy;

class ROSCameraBridge : public ROSCamInterface
{
private:
  bool filterBlurredImages;
  BlurDetector detector;

  void initSpinner();
  void readConfig(const boost::property_tree::ptree &pt);

  message_filters::Synchronizer<RGBSyncPolicy> *sync;
  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *rgbImageSubscriber;

  message_filters::Subscriber<sensor_msgs::CameraInfo> *cameraInfoSubscriber;

  void cb_(const sensor_msgs::Image::ConstPtr rgb_img_msg,
           const sensor_msgs::CameraInfo::ConstPtr camera_info_msg);

  cv::Mat color;

  sensor_msgs::CameraInfo cameraInfo;

public:
  ROSCameraBridge(const boost::property_tree::ptree &pt);
  ~ROSCameraBridge();

  bool setData(uima::CAS &tcas, u_int64_t = 0);
  void lookupTransform(uima::CAS &tcas, const ros::Time &timestamp);
};

#endif // __ROS_CAMERA_BRIDGE_H__
