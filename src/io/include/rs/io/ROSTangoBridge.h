#ifndef __ROS_TANGO_BRIDGE_H__
#define __ROS_TANGO_BRIDGE_H__

//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/image_transport.h>
#include <message_filters/sync_policies/approximate_time.h>


//OpenCV
#include <opencv2/opencv.hpp>

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// RS
#include <rs/io/ROSCamInterface.h>
#include <rs/utils/BlurDetector.h>

typedef message_filters::sync_policies::ApproximateTime
         <sensor_msgs::Image, sensor_msgs::CameraInfo> RGBSyncPolicy;
typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::Image, sensor_msgs::PointCloud2, sensor_msgs::CameraInfo>
                                                            colorCloudSyncPolicy;

class ROSTangoBridge : public ROSCamInterface
{
private:
  bool filterBlurredImages;
  BlurDetector detector;

  image_transport::ImageTransport it;

  message_filters::Synchronizer<RGBSyncPolicy> *fisheyeSync;
  message_filters::Synchronizer<colorCloudSyncPolicy> *colorCloudSync;

  image_transport::SubscriberFilter *colorImageSubscriber;
  image_transport::SubscriberFilter *fisheyeImageSubscriber;

  message_filters::Subscriber<sensor_msgs::CameraInfo> *colorInfoSubscriber;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *fisheyeInfoSubscriber;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *cloudSubscriber;

  cv::Mat color;
  cv::Mat fisheye;

  //pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZRGBA> cloud_color;

  sensor_msgs::CameraInfo colorCameraInfo;
  sensor_msgs::CameraInfo fisheyeCameraInfo;

  void initSpinner();
  void readConfig(const boost::property_tree::ptree &pt);

  void fisheyeCb_(const sensor_msgs::Image::ConstPtr fisheye_img_msg,
                  const sensor_msgs::CameraInfo::ConstPtr fisheye_info_msg);

  void cb_(const sensor_msgs::Image::ConstPtr color_img_msg,
           const sensor_msgs::PointCloud2::ConstPtr cloud_msg,
           const sensor_msgs::CameraInfo::ConstPtr color_info_msg);

public:
  ROSTangoBridge(const boost::property_tree::ptree &pt);
  ~ROSTangoBridge();

  bool setData(uima::CAS &tcas, u_int64_t = std::numeric_limits<uint64_t>::max());

  inline void getColorImage(cv::Mat& c)
  {
    c = this->color.clone();
  }
  inline void getFisheyeImage(cv::Mat& f)
  {
    f = this->fisheye.clone();
  }
};

#endif // __ROS_TANGO_BRIDGE_H__
