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
  delete colorSync;
  delete fisheyeSync;
//  delete sync;
}

void ROSTangoBridge::initSpinner()
{
  colorSync = new message_filters::Synchronizer<RGBSyncPolicy>
                  (RGBSyncPolicy(10), *colorImageSubscriber, *colorInfoSubscriber);
  fisheyeSync = new message_filters::Synchronizer<RGBSyncPolicy>
                  (RGBSyncPolicy(10), *fisheyeImageSubscriber, *fisheyeInfoSubscriber);

  colorSync->registerCallback(boost::bind(&ROSTangoBridge::colorCb_, this, _1, _2));
  fisheyeSync->registerCallback(boost::bind(&ROSTangoBridge::fisheyeCb_, this, _1, _2));

  spinner.start();
}

void ROSTangoBridge::readConfig(const boost::property_tree::ptree &pt)
{
  std::string cloud_topic = pt.get<std::string>("cloud_topic.cloud");

  std::string color_topic = pt.get<std::string>("camera_topics.color");
  std::string fisheye_topic = pt.get<std::string>("camera_topics.fisheye");

  std::string color_hints = pt.get<std::string>("camera_topics.colorHints", "raw");
  std::string fisheye_hints = pt.get<std::string>("camera_topics.fisheyeHints", "raw");
  //boost::optional<std::string> color_hints = pt.get_optional<std::string>("camera_topics.colorHints");


  std::string color_info_topic = pt.get<std::string>("camera_topics.colorInfo");
  std::string fisheye_info_topic = pt.get<std::string>("camera_topics.fisheyeInfo");

  filterBlurredImages = pt.get<bool>("camera.filterBlurredImages", false);
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
  outInfo("  Blur filter: " FG_BLUE << (filterBlurredImages ? "ON" : "OFF"));
}

void ROSTangoBridge::colorCb_(const sensor_msgs::Image::ConstPtr color_img_msg,
                         const sensor_msgs::CameraInfo::ConstPtr color_info_msg)
{
  cv::Mat color;
  sensor_msgs::CameraInfo colorCameraInfo;

  cv_bridge::CvImageConstPtr orig_color_img;
  outWarn("Check the encoding of color image BGR8 or another type???");
  orig_color_img = cv_bridge::toCvShare(color_img_msg, sensor_msgs::image_encodings::BGR8);
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

  lock.lock();
  this->color = color;
  this->colorCameraInfo = colorCameraInfo;
  _newData = true;
  lock.unlock();
}

void ROSTangoBridge::fisheyeCb_(const sensor_msgs::Image::ConstPtr fisheye_img_msg,
                                const sensor_msgs::CameraInfo::ConstPtr fisheye_info_msg)
{
cv::Mat fisheye;
sensor_msgs::CameraInfo fisheyeCameraInfo;
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
void ROSTangoBridge::cloudCb_(const sensor_msgs::PointCloud2 cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(cloud_msg, cloud);

  pcl::PointCloud<pcl::PointXYZRGBA> cloud_color;
  cloud_color.width = cloud.width;
  cloud_color.height = cloud.height;
  cloud_color.points.resize(cloud_color.width*cloud_color.height);

  float fx = 1042.73999023438f;
  float fy = 1042.96997070313f;
  float cx = 637.273986816406f;
  float cy = 352.928985595703f;
  float k1 = 0.228532999753952f;
  float k2 = -0.663019001483917f;
  float k3 = 0.642908990383148f;
  Eigen::Matrix3f K;
  K << fx, 0, cx,
       0, fy, cy,
       0, 0, 1;

  for(size_t i = 0; i < cloud.points.size(); i++)
  {
    Eigen::Vector2f imageCoords;
    imageCoords[0] = cloud.points[i].x/cloud.points[i].z;
    imageCoords[1] = cloud.points[i].y/cloud.points[i].z;
    float r2 = imageCoords.adjoint()*imageCoords;
    float r4 = r2*r2;
    float r6 = r2*r4;
    imageCoords = imageCoords*(1.0 + k1*r2 + k2*r4 + k3*r6);
    Eigen::Vector3f imageCoords_3;
    imageCoords_3[0]=imageCoords[0];
    imageCoords_3[1]=imageCoords[1];
    imageCoords_3[2]=1;

    Eigen::Vector3f pixelCoords;
    pixelCoords = K*imageCoords_3;

    //Eigen::Vector3i pixelCoords_;//RowVector3i
    pixelCoords[0] = static_cast<unsigned int>(pixelCoords[0]);
    pixelCoords[1] = static_cast<unsigned int>(pixelCoords[1]);

    outInfo("  Pixel Coordinate u: " FG_BLUE << pixelCoords[0]);
    outInfo("  Pixel Coordinate v: " FG_BLUE << pixelCoords[1]);

    cloud_color.points[i].x = pixelCoords[0]*cloud.points[i].z;
    cloud_color.points[i].y = pixelCoords[1]*cloud.points[i].z;
    cloud_color.points[i].z = cloud.points[i].z;

    cv::Vec4b rgba = this->color.at<cv::Vec4b>(pixelCoords[1], pixelCoords[0]);
    cloud_color.points[i].r = rgba[0];
    cloud_color.points[i].g = rgba[1];
    cloud_color.points[i].b = rgba[2];
    cloud_color.points[i].a = rgba[3];
  }

  lock.lock();
  this->cloud_color = cloud_color;
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
