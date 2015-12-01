// ROS
#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>

// RS
#include <rs/io/ROSKinectBridge.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>

//OpenCV
#include <cv_bridge/cv_bridge.h>

ROSKinectBridge::ROSKinectBridge(const boost::property_tree::ptree &pt) : ROSCamInterface(pt), it(nodeHandle)
{
  readConfig(pt);
  initSpinner();
}

ROSKinectBridge::~ROSKinectBridge()
{
  spinner.stop();
  delete sync;
  delete rgbImageSubscriber;
  delete depthImageSubscriber;
  delete cameraInfoSubscriber;
}

void ROSKinectBridge::initSpinner()
{
  sync = new message_filters::Synchronizer<RGBDSyncPolicy>(RGBDSyncPolicy(5), *rgbImageSubscriber, *depthImageSubscriber, *cameraInfoSubscriber);
  sync->registerCallback(boost::bind(&ROSKinectBridge::cb_, this, _1, _2, _3));
  spinner.start();
}

void ROSKinectBridge::readConfig(const boost::property_tree::ptree &pt)
{
  std::string depth_topic = pt.get<std::string>("camera_topics.depth");
  std::string color_topic = pt.get<std::string>("camera_topics.color");
  boost::optional<std::string> depth_hints = pt.get_optional<std::string>("camera_topics.depthHints");
  boost::optional<std::string> color_hints = pt.get_optional<std::string>("camera_topics.colorHints");
  std::string cam_info_topic = pt.get<std::string>("camera_topics.camInfo");
  filterBlurredImages = pt.get<bool>("camera.filterBlurredImages");

  depthOffset = pt.get<int>("camera.depthOffset");

  image_transport::TransportHints hintsColor(color_hints ? color_hints.get() : "raw");
  image_transport::TransportHints hintsDepth(depth_hints ? depth_hints.get() : "raw");

  depthImageSubscriber = new image_transport::SubscriberFilter(it, depth_topic, 5, hintsDepth);
  rgbImageSubscriber = new image_transport::SubscriberFilter(it, color_topic, 5, hintsColor);
  cameraInfoSubscriber = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nodeHandle, cam_info_topic, 5);

  outInfo("Depth topic:   " FG_BLUE << depth_topic);
  outInfo("Color topic:   " FG_BLUE << color_topic);
  outInfo("CamInfo topic: " FG_BLUE << cam_info_topic);
  if(depth_hints && color_hints)
  {
    outInfo("Depth Hints:   " FG_BLUE << depth_hints.get());
    outInfo("Color Hints:   " FG_BLUE << color_hints.get());
  }
  outInfo("DepthOffset:   " FG_BLUE << depthOffset);
  outInfo("Blur filter:   " FG_BLUE << (filterBlurredImages ? "ON" : "OFF"));
}

void ROSKinectBridge::cb_(const sensor_msgs::Image::ConstPtr rgb_img_msg,
                          const sensor_msgs::Image::ConstPtr depth_img_msg,
                          const sensor_msgs::CameraInfo::ConstPtr camera_info_msg)
{
//  static int frame = 0;
//  outWarn("got image: " << frame++);
  cv::Mat color, depth;
  bool isHDColor;
  sensor_msgs::CameraInfo cameraInfo, cameraInfoHD;

  cv_bridge::CvImageConstPtr orig_rgb_img;
  orig_rgb_img = cv_bridge::toCvShare(rgb_img_msg, sensor_msgs::image_encodings::BGR8);
  cameraInfo = sensor_msgs::CameraInfo(*camera_info_msg);

  if(filterBlurredImages && detector.detectBlur(orig_rgb_img->image))
  {
    outWarn("Skipping blurred image!");
    return;
  }

  cv_bridge::CvImageConstPtr orig_depth_img;
  orig_depth_img = cv_bridge::toCvShare(depth_img_msg, depth_img_msg->encoding);

  if(depth_img_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    depth = orig_depth_img->image.clone();
  }
  else if(depth_img_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    orig_depth_img->image.convertTo(depth, CV_16U, 0.001);
  }
  else
  {
    outError("Unknown depth image type!");
    return;
  }

  color = orig_rgb_img->image.clone();
  if(color.cols == 1280 || color.cols == 1920) // HD or Kinect 2
  {
    isHDColor = true;
    if(color.cols == 1280)
    {
      color = color(cv::Rect(0, depthOffset, 1280, 960));
      cameraInfo.K[5] -= depthOffset;
      cameraInfo.height = 960;
    }

    cameraInfoHD = cameraInfo;
    cameraInfo.height /= 2.0;
    cameraInfo.width /= 2.0;
    cameraInfo.roi.height /= 2.0;
    cameraInfo.roi.width /= 2.0;
    cameraInfo.roi.x_offset /= 2.0;
    cameraInfo.roi.y_offset /= 2.0;
    cameraInfo.K[0] /= 2.0;
    cameraInfo.K[2] /= 2.0;
    cameraInfo.K[4] /= 2.0;
    cameraInfo.K[5] /= 2.0;
    cameraInfo.P[0] /= 2.0;
    cameraInfo.P[2] /= 2.0;
    cameraInfo.P[5] /= 2.0;
    cameraInfo.P[6] /= 2.0;
  }
  else if(color.cols == 640)
  {
    isHDColor = false;
    cameraInfoHD = cameraInfo;
    cameraInfoHD.height *= 2.0;
    cameraInfoHD.width *= 2.0;
    cameraInfoHD.roi.height *= 2.0;
    cameraInfoHD.roi.width *= 2.0;
    cameraInfoHD.roi.x_offset *= 2.0;
    cameraInfoHD.roi.y_offset *= 2.0;
    cameraInfoHD.K[0] *= 2.0;
    cameraInfoHD.K[2] *= 2.0;
    cameraInfoHD.K[4] *= 2.0;
    cameraInfoHD.K[5] *= 2.0;
    cameraInfoHD.P[0] *= 2.0;
    cameraInfoHD.P[2] *= 2.0;
    cameraInfoHD.P[5] *= 2.0;
    cameraInfoHD.P[6] *= 2.0;
  }
  else
  {
    outError("Unknown color image size!");
    return;
  }

  lock.lock();
  this->color = color;
  this->depth = depth;
  this->cameraInfo = cameraInfo;
  this->cameraInfoHD = cameraInfoHD;
//  outWarn("new data");
  _newData = true;
  lock.unlock();
}

bool ROSKinectBridge::setData(uima::CAS &tcas, uint64_t ts)
{
  if(!newData())
  {
    return false;
  }
  MEASURE_TIME;

  cv::Mat color, depth;
  sensor_msgs::CameraInfo cameraInfo, cameraInfoHD;

  lock.lock();
  color = this->color;
  depth = this->depth;
  cameraInfo = this->cameraInfo;
  cameraInfoHD = this->cameraInfoHD;
  _newData = false;
  lock.unlock();

  rs::SceneCas cas(tcas);
  lookupTransform(tcas, cameraInfo.header.stamp);

  if(color.cols >= 1280)
  {
    cas.set(VIEW_COLOR_IMAGE_HD, color);
  }
  else
  {
    cas.set(VIEW_COLOR_IMAGE, color);
  }

  if(depth.cols >= 1280)
  {
    cas.set(VIEW_DEPTH_IMAGE_HD, depth);
  }
  else
  {
    cas.set(VIEW_DEPTH_IMAGE, depth);
  }

  cas.set(VIEW_CAMERA_INFO, cameraInfo);
  cas.set(VIEW_CAMERA_INFO_HD, cameraInfoHD);

  cas.getScene().timestamp.set(cameraInfo.header.stamp.toNSec());

  return true;
}

void ROSKinectBridge::lookupTransform(uima::CAS &tcas, const ros::Time &timestamp)
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
    scene.viewPoint.set(vp);
    outInfo("added viewpoint to scene");
  }
  catch(tf::TransformException &ex)
  {
    outError(ex.what());
  }
}
