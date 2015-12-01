// ROS
#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>

// RS
#include <rs/io/ROSCameraBridge.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>

//OpenCV
#include <cv_bridge/cv_bridge.h>

ROSCameraBridge::ROSCameraBridge(const boost::property_tree::ptree &pt) : ROSCamInterface(pt), it(nodeHandle)
{
  readConfig(pt);
  initSpinner();
}

ROSCameraBridge::~ROSCameraBridge()
{
  spinner.stop();
  delete sync;
  delete rgbImageSubscriber;
  delete cameraInfoSubscriber;
}

void ROSCameraBridge::initSpinner()
{
  sync = new message_filters::Synchronizer<RGBSyncPolicy>(RGBSyncPolicy(10), *rgbImageSubscriber, *cameraInfoSubscriber);
  sync->registerCallback(boost::bind(&ROSCameraBridge::cb_, this, _1, _2));
  spinner.start();
}

void ROSCameraBridge::readConfig(const boost::property_tree::ptree &pt)
{
  std::string color_topic = pt.get<std::string>("camera_topics.color");
  boost::optional<std::string> color_hints = pt.get_optional<std::string>("camera_topics.colorHints");
  std::string cam_info_topic = pt.get<std::string>("camera_topics.camInfo");
  filterBlurredImages = pt.get<bool>("camera.filterBlurredImages");

  image_transport::TransportHints hintsColor(color_hints ? color_hints.get() : "raw");
  rgbImageSubscriber = new image_transport::SubscriberFilter(it, color_topic, 1, hintsColor);

  cameraInfoSubscriber = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nodeHandle, cam_info_topic, 1);

  outInfo("Color topic:   " FG_BLUE << color_topic);
  outInfo("CamInfo topic: " FG_BLUE << cam_info_topic);
  outInfo("Color Hints:   " FG_BLUE << color_hints);
  outInfo("Blur filter:   " FG_BLUE << (filterBlurredImages ? "ON" : "OFF"));
}

void ROSCameraBridge::cb_(const sensor_msgs::Image::ConstPtr rgb_img_msg,
                          const sensor_msgs::CameraInfo::ConstPtr camera_info_msg)
{
  cv::Mat color;
  sensor_msgs::CameraInfo cameraInfo;

  cv_bridge::CvImageConstPtr orig_rgb_img;
  orig_rgb_img = cv_bridge::toCvShare(rgb_img_msg, sensor_msgs::image_encodings::BGR8);
  cameraInfo = sensor_msgs::CameraInfo(*camera_info_msg);

  if(filterBlurredImages && detector.detectBlur(orig_rgb_img->image))
  {
    outWarn("Skipping blurred image!");
    return;
  }

  color = orig_rgb_img->image.clone();

  lock.lock();
  this->color = color;
  this->cameraInfo = cameraInfo;
  _newData = true;
  lock.unlock();
}

bool ROSCameraBridge::setData(uima::CAS &tcas, uint64_t ts)
{
  if(!newData())
  {
    return false;
  }
  MEASURE_TIME;

  cv::Mat color;
  sensor_msgs::CameraInfo cameraInfo;

  lock.lock();
  color = this->color;
  cameraInfo = this->cameraInfo;
  _newData = false;
  lock.unlock();

  rs::SceneCas cas(tcas);
  lookupTransform(tcas, cameraInfo.header.stamp);

  cas.set(VIEW_COLOR_IMAGE_HD, color);
  cas.set(VIEW_COLOR_IMAGE, color);

  cas.set(VIEW_CAMERA_INFO, cameraInfo);
  cas.set(VIEW_CAMERA_INFO_HD, cameraInfo);

  cas.getScene().timestamp.set(cameraInfo.header.stamp.toNSec());

  return true;
}

void ROSCameraBridge::lookupTransform(uima::CAS &tcas, const ros::Time &timestamp)
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
