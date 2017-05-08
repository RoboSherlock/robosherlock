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

#include <tf_conversions/tf_eigen.h>
#include <cmath>
#include <Eigen/Core>

#include<iostream>
ROSTangoBridge::ROSTangoBridge(const boost::property_tree::ptree &pt) : ROSCamInterface(pt),
                                                                        it(nodeHandle)
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

  delete cloudSubscriber;

  delete colorCloudSync;
  delete fisheyeSync;
}

void ROSTangoBridge::initSpinner()
{
  fisheyeSync = new message_filters::Synchronizer<RGBSyncPolicy>
                  (RGBSyncPolicy(10), *fisheyeImageSubscriber, *fisheyeInfoSubscriber);
  colorCloudSync = new message_filters::Synchronizer<colorCloudSyncPolicy>
                  (colorCloudSyncPolicy(10), *colorImageSubscriber,
                  *cloudSubscriber, *colorInfoSubscriber);

  fisheyeSync->registerCallback(boost::bind(&ROSTangoBridge::fisheyeCb_, this, _1, _2));
  colorCloudSync->registerCallback(boost::bind(&ROSTangoBridge::cb_, this, _1, _2, _3));

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

  filterBlurredImages = pt.get<bool>("camera.filterBlurredImages", false);
  image_transport::TransportHints hintsColor(color_hints);
  image_transport::TransportHints hintsFisheye(fisheye_hints);

  colorImageSubscriber = new image_transport::SubscriberFilter(it, color_topic,
                                                  5, hintsColor);
  fisheyeImageSubscriber = new image_transport::SubscriberFilter(it, fisheye_topic,
                                                  5, hintsFisheye);

  colorInfoSubscriber = new message_filters::Subscriber<sensor_msgs::CameraInfo>
                                                    (nodeHandle, color_info_topic, 5);
  fisheyeInfoSubscriber = new message_filters::Subscriber<sensor_msgs::CameraInfo>
                                                    (nodeHandle, fisheye_info_topic, 5);

  cloudSubscriber = new message_filters::Subscriber<sensor_msgs::PointCloud2>
                                                    (nodeHandle, cloud_topic, 5);


  outInfo("  Cloud topic: " FG_BLUE << cloud_topic);
  outInfo("  Color topic: " FG_BLUE << color_topic);
  outInfo("  Fisheye topic: " FG_BLUE << fisheye_topic);
  outInfo("  ColorCamInfo topic: " FG_BLUE << color_info_topic);
  outInfo("  FisheyeCamInfo topic: " FG_BLUE << fisheye_info_topic);
  outInfo("  Color Hints: " FG_BLUE << color_hints);
  outInfo("  Fisheye Hints: " FG_BLUE << fisheye_hints);
  outInfo("  Blur filter: " FG_BLUE << (filterBlurredImages ? "ON" : "OFF"));
}

void ROSTangoBridge::cb_(const sensor_msgs::Image::ConstPtr color_img_msg,
                         const sensor_msgs::PointCloud2::ConstPtr cloud_msg,
                         const sensor_msgs::CameraInfo::ConstPtr color_info_msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    cv::Mat color;
    sensor_msgs::CameraInfo colorCameraInfo;

    pcl::PointCloud<pcl::PointXYZRGBA> cloud_color;
    copyPointCloud(cloud, cloud_color);

    cv_bridge::CvImageConstPtr orig_color_img;
    orig_color_img = cv_bridge::toCvShare(color_img_msg,
                                  sensor_msgs::image_encodings::BGR8);
    colorCameraInfo = sensor_msgs::CameraInfo(*color_info_msg);

    if(!lookupTransform(colorCameraInfo.header.stamp))
    {
      lock.lock();
      _newData = false;
      lock.unlock();
      return;
    }

    if(filterBlurredImages && detector.detectBlur(orig_color_img->image))
    {
      lock.lock();
      _newData = false;
      lock.unlock();
      outWarn("Skipping blurred image!");
      return;
    }

    color = orig_color_img->image.clone();
    Eigen::Matrix3d K;
    K << colorCameraInfo.K[0], colorCameraInfo.K[1], colorCameraInfo.K[2],
         colorCameraInfo.K[3], colorCameraInfo.K[4], colorCameraInfo.K[5],
         colorCameraInfo.K[6], colorCameraInfo.K[7], colorCameraInfo.K[8];

    // outInfo("  Intrinsic Matrix fx: " FG_BLUE << colorCameraInfo.K[0]);
    // outInfo("  Intrinsic Matrix fy: " FG_BLUE << colorCameraInfo.K[4]);
    // outInfo("  Intrinsic Matrix cx: " FG_BLUE << colorCameraInfo.K[2]);
    // outInfo("  Intrinsic Matrix cy: " FG_BLUE << colorCameraInfo.K[5]);

    Eigen::Vector3d D;
    D[0]=colorCameraInfo.D[0];
    D[1]=colorCameraInfo.D[1];
    D[2]=colorCameraInfo.D[2];

    // outInfo("  Distorsion value k1: " FG_BLUE << colorCameraInfo.D[0]);
    // outInfo("  Distorsion value k2: " FG_BLUE << colorCameraInfo.D[1]);
    // outInfo("  Distorsion value k3: " FG_BLUE << colorCameraInfo.D[2]);


    for(size_t i = 0; i < cloud_color.points.size(); i++)
    {
      Eigen::Vector2d imageCoords;
      imageCoords[0] = cloud.points[i].x/cloud.points[i].z;
      imageCoords[1] = cloud.points[i].y/cloud.points[i].z;
      float r2 = imageCoords.adjoint()*imageCoords;
      float r4 = r2*r2;
      float r6 = r2*r4;
      imageCoords = imageCoords*(1.0 + D[0]*r2 + D[1]*r4 + D[2]*r6);
      Eigen::Vector3d imageCoords_3;
      imageCoords_3[0]=imageCoords[0];
      imageCoords_3[1]=imageCoords[1];
      imageCoords_3[2]=1;

      Eigen::Vector3d pixelCoords;
      pixelCoords = K*imageCoords_3;

      pixelCoords[0] = static_cast<unsigned int>(pixelCoords[0]);
      pixelCoords[1] = static_cast<unsigned int>(pixelCoords[1]);

      // outInfo("  Pixel Coordinate u: " FG_BLUE << pixelCoords[0]);
      // outInfo("  Pixel Coordinate v: " FG_BLUE << pixelCoords[1]);

      if(pixelCoords[0] <= color.cols && pixelCoords[1] <= color.rows)
      {
          cloud_color.points[i].b = color.at<cv::Vec3b>(cv::Point(pixelCoords[0], pixelCoords[1]))[0];
          cloud_color.points[i].g = color.at<cv::Vec3b>(cv::Point(pixelCoords[0], pixelCoords[1]))[1];
          cloud_color.points[i].r = color.at<cv::Vec3b>(cv::Point(pixelCoords[0], pixelCoords[1]))[2];
          cloud_color.points[i].a = 255;
      }
      else
      {
          outInfo("  The pixel coordinates exceed the size of color image. ");
          outInfo("  Pixel Coordinate u: " FG_BLUE << pixelCoords[0]);
          outInfo("  Pixel Coordinate v: " FG_BLUE << pixelCoords[1]);
      }
    }

    lock.lock();
    this->color = color;
    this->colorCameraInfo = colorCameraInfo;
    this->cloud_color = cloud_color;
    _newData = true;
    lock.unlock();
}

void ROSTangoBridge::fisheyeCb_(const sensor_msgs::Image::ConstPtr fisheye_img_msg,
                                const sensor_msgs::CameraInfo::ConstPtr fisheye_info_msg)
{
cv::Mat fisheye;
sensor_msgs::CameraInfo fisheyeCameraInfo;
cv_bridge::CvImageConstPtr orig_fisheye_img;
orig_fisheye_img = cv_bridge::toCvShare(fisheye_img_msg,
                                        sensor_msgs::image_encodings::BGR8);
fisheyeCameraInfo = sensor_msgs::CameraInfo(*fisheye_info_msg);
if(!lookupTransform(fisheyeCameraInfo.header.stamp))
{
  lock.lock();
  _newData = false;
  lock.unlock();
  return;
}

if(filterBlurredImages && detector.detectBlur(orig_fisheye_img->image))
{
  lock.lock();
  _newData = false;
  lock.unlock();
  outWarn("Skipping blurred image!");
  return;
}

fisheye = orig_fisheye_img->image.clone();
lock.lock();
this->fisheye = fisheye;
this->fisheyeCameraInfo = fisheyeCameraInfo;
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
  pcl::PointCloud<pcl::PointXYZRGBA> cloud_color;
  lock.lock();
  color = this->color;
  fisheye = this->fisheye;
  colorCameraInfo = this->colorCameraInfo;
  fisheyeCameraInfo = this->fisheyeCameraInfo;
  cloud_color = this->cloud_color;
  outInfo("  Fisheye Image Height: " FG_BLUE << fisheye.size().height);
  outInfo("  Fisheye Image Width: " FG_BLUE << fisheye.size().width);
  outInfo("  Color Image Height: " FG_BLUE << color.size().height);
  outInfo("  Color Image Width: " FG_BLUE << color.size().width);

  outInfo("  Cloud Width: " FG_BLUE << cloud_color.width);
  outInfo("  Cloud Height: " FG_BLUE << cloud_color.height);
  outInfo("  Number of Point Cloud: " FG_BLUE << cloud_color.size());
  _newData = false;
  lock.unlock();

  rs::SceneCas cas(tcas);
  setTransformAndTime(tcas);

  cas.set(VIEW_CLOUD, cloud_color);
  cas.set(VIEW_COLOR_IMAGE, color);
  cas.set(VIEW_FISHEYE_IMAGE, fisheye);
  cas.set(VIEW_COLOR_CAMERA_INFO, colorCameraInfo);
  cas.set(VIEW_FISHEYE_CAMERA_INFO, fisheyeCameraInfo);

  return true;
}
