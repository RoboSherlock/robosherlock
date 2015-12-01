// ROS
#include <ros/package.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>

// RS
#include <rs/io/ROSThermalCamBridge.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>

//OpenCV
#include <cv_bridge/cv_bridge.h>

ROSThermalCamBridge::ROSThermalCamBridge(const boost::property_tree::ptree &pt) : ROSCamInterface(pt), it(nodeHandle)
{
  readConfig(pt);
  initSpinner();
}

ROSThermalCamBridge::~ROSThermalCamBridge()
{
  spinner.stop();
  delete sync;
  delete thermalImageSubscriber;
  delete thermalRGBImageSubscriber;
  delete thermalDepthImageSubscriber;
  delete cameraInfoSubscriber;
}

void ROSThermalCamBridge::initSpinner()
{
  sync = new message_filters::Synchronizer<ThermalSyncPolicy>(ThermalSyncPolicy(10), *thermalImageSubscriber, *thermalRGBImageSubscriber, *thermalDepthImageSubscriber, *cameraInfoSubscriber);
  sync->registerCallback(boost::bind(&ROSThermalCamBridge::cb_, this, _1, _2, _3, _4));
  spinner.start();
}

void ROSThermalCamBridge::readConfig(const boost::property_tree::ptree &pt)
{
  std::string thermal_topic = pt.get<std::string>("camera_topics.thermal");
  std::string thermal_depth_topic = pt.get<std::string>("camera_topics.thermal_depth_reg");
  std::string thermal_rgb_topic = pt.get<std::string>("camera_topics.thermal_rgb_reg");
  std::string cam_info_topic = pt.get<std::string>("camera_topics.camInfo");
  boost::optional<std::string> depth_hints = pt.get_optional<std::string>("camera_topics.depthHints");
  boost::optional<std::string> color_hints = pt.get_optional<std::string>("camera_topics.colorHints");
  boost::optional<std::string> thermal_hints = pt.get_optional<std::string>("camera_topics.thermalHints");

  image_transport::TransportHints hintsThermal(thermal_hints ? thermal_hints.get() : "raw");
  image_transport::TransportHints hintsColor(color_hints ? color_hints.get() : "raw");
  image_transport::TransportHints hintsDepth(depth_hints ? depth_hints.get() : "raw");

  thermalImageSubscriber = new image_transport::SubscriberFilter(it, thermal_topic, 0, hintsThermal);
  thermalDepthImageSubscriber = new image_transport::SubscriberFilter(it, thermal_depth_topic, 0, hintsDepth);
  thermalRGBImageSubscriber = new image_transport::SubscriberFilter(it, thermal_rgb_topic, 0, hintsColor);
  cameraInfoSubscriber = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nodeHandle, cam_info_topic, 5);

  outInfo("Thermal  set to: " FG_BLUE << thermal_topic);
  outInfo("ThermalD set to: " FG_BLUE << thermal_depth_topic);
  outInfo("ThermalR set to: " FG_BLUE << thermal_rgb_topic);
  outInfo("CamInfo set to:  " FG_BLUE << cam_info_topic);
}

void ROSThermalCamBridge::cb_(const sensor_msgs::Image::ConstPtr thermal_img_msg,
                              const sensor_msgs::Image::ConstPtr thermal_rgb_img_msg,
                              const sensor_msgs::Image::ConstPtr thermal_depth_img_msg,
                              const sensor_msgs::CameraInfo::ConstPtr camera_info_msg)
{
  cv::Mat thermalImage;
  sensor_msgs::CameraInfo cameraInfo;
  cv::Mat thermalRGBImage;
  cv::Mat thermalDepthImage;
  cv::Mat thermalDepthImageAsInt;

  cv_bridge::CvImageConstPtr orig_thermal_img;
  orig_thermal_img = cv_bridge::toCvShare(thermal_img_msg, sensor_msgs::image_encodings::MONO8);
  orig_thermal_img->image.copyTo(thermalImage);

  //hacked: displaying heat map etc must be disabled in the flir firmware
  if(thermalImage.cols == 320)
  {
    maskImage(thermalImage, 5, 4, 23, 25);
    maskImage(thermalImage, 257, 4, 316, 25);
    maskImage(thermalImage, 304, 27, 316, 212);
    maskImage(thermalImage, 257, 214, 316, 235);
  }
  else if(thermalImage.cols == 640)
  {
    maskImage(thermalImage, 18, 14, 46, 48);
    maskImage(thermalImage, 562, 7, 638, 37);
    maskImage(thermalImage, 616, 47, 640, 424);
    maskImage(thermalImage, 558, 436, 630, 463);
  }
  else
  {
    outError("Unsupported Image size");
    return;
  }

  cameraInfo = sensor_msgs::CameraInfo(*camera_info_msg);

  cv_bridge::CvImageConstPtr orig_thermal_depth_img;
  orig_thermal_depth_img = cv_bridge::toCvShare(thermal_depth_img_msg, thermal_depth_img_msg->encoding);

  if(thermal_depth_img_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    thermalDepthImage = orig_thermal_depth_img->image.clone();
  }
  else if(thermal_depth_img_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    orig_thermal_depth_img->image.convertTo(thermalDepthImage, CV_16U, 0.001);
  }
  else
  {
    outError("Unknown depth image type!");
    return;
  }

  cv_bridge::CvImageConstPtr orig_rgb_img;
  orig_rgb_img = cv_bridge::toCvShare(thermal_rgb_img_msg, sensor_msgs::image_encodings::BGR8);
  orig_rgb_img->image.copyTo(thermalRGBImage);

  lock.lock();
  this->thermalImage = thermalImage;
  this->cameraInfo = cameraInfo;
  this->thermalRGBImage = thermalRGBImage;
  this->thermalDepthImage = thermalDepthImage;
  _newData = true;
  lock.unlock();
}

void ROSThermalCamBridge::maskImage(cv::Mat &image, int xMin, int yMin, int xMax, int yMax)
{
  #pragma omp parallel for
  for(unsigned int y = yMin; y < yMax; ++y)
  {
    uchar *it = image.ptr<uchar>(y);
    unsigned int x = 0;
    while(x < xMin)
    {
      ++it;
      ++x;
    }
    for(; x < xMax; ++x, ++it)
    {
      *it = 0;
    }
  }
}

bool ROSThermalCamBridge::setData(uima::CAS &tcas, uint64_t ts)
{
  if(!newData())
  {
    return false;
  }
  MEASURE_TIME;

  cv::Mat thermalImage, thermalDepthImage, thermalRGBImage, thermalDepthImageAsInt;
  sensor_msgs::CameraInfo cameraInfo;

  lock.lock();
  cameraInfo = this->cameraInfo;
  thermalImage = this->thermalImage;
  thermalRGBImage = this->thermalRGBImage;
  thermalDepthImage = this->thermalDepthImage;
  _newData = false;
  lock.unlock();

  rs::SceneCas cas(tcas);
  lookupTransform(tcas, cameraInfo.header.stamp);

  cas.set(VIEW_THERMAL_IMAGE, thermalImage);
  cas.set(VIEW_THERMAL_DEPTH_IMAGE, thermalDepthImage);
  cas.set(VIEW_THERMAL_COLOR_IMAGE, thermalRGBImage);
  cas.set(VIEW_THERMAL_CAMERA_INFO, cameraInfo);

  return true;
}

void ROSThermalCamBridge::lookupTransform(uima::CAS &tcas, const ros::Time &timestamp)
{
  if(!lookUpViewpoint)
  {
    return;
  }

  tf::StampedTransform transform;
  try
  {
    outInfo("TIME Before lookup:" << ros::Time::now());
    listener->waitForTransform(tfTo, tfFrom, ros::Time(0), ros::Duration(10));
    listener->lookupTransform(tfTo, tfFrom, ros::Time(0), transform);
    rs::Scene scene = rs::SceneCas(tcas).getScene();
    rs::StampedTransform vp(rs::conversion::to(tcas, transform));
    scene.thermalViewPoint.set(vp);
    outInfo("added viewpoint to scene");
  }
  catch(tf::TransformException &ex)
  {
    outError(ex.what());
  }
}
