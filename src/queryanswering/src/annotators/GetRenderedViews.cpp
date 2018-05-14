#include <uima/api.hpp>

// Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

//RS
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/io/UnrealVisionBridge.h>
#include <rs/utils/output.h>
#include <rs/utils/common.h>
#include <rs/io/TFBroadcasterWrapper.hpp>
#include <rs/DrawingAnnotator.h>

//ROS
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <robosherlock_msgs/UpdateObjects.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>

#include <visualization_msgs/MarkerArray.h>
#include <resource_retriever/retriever.h>

//PCL
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

//C/C++
#include <random>
#include <cmath>
#include <chrono>




using namespace uima;

class GetRenderedViews : public DrawingAnnotator
{

private:

  std::map<std::string, std::string> uNameMapping =
  {
    {"cup", "SM_CupEcoOrange_2"},
    {"bowl", "SM_Bowl_8"},
    {"cereal", "SM_KoellnMuesliKnusperHonigNussNew_2"},
    {"spoon", "SM_Spoon_Dessert9_2"}
  };

  ros::ServiceClient client_;
  ros::NodeHandle nh_;

  UnrealVisionBridge *unrealBridge_;

  image_transport::Publisher image_pub_;
  image_transport::ImageTransport it_;
  cv::Mat object_, rgb_;

  std::thread thread_;
  TFBroadcasterWrapper broadCasterObject_;
  ros::Publisher marker_pub_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr sphereCloud_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr viewCloud_;

  bool publishAsMarkers_;

public:
  GetRenderedViews(): DrawingAnnotator(__func__), nh_("~"), it_(nh_),publishAsMarkers_(false)
  {
    client_ = nh_.serviceClient<robosherlock_msgs::UpdateObjects>("/update_objects");
    std::string configFile = ros::package::getPath("robosherlock") + "/config/config_unreal_vision.ini";
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(configFile, pt);
    unrealBridge_ = new UnrealVisionBridge(pt);

    image_pub_ = it_.advertise("rendered_image", 5, false);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 1, true);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    thread_ = std::thread(&TFBroadcasterWrapper::run, &broadCasterObject_);
    sphereCloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    viewCloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    delete unrealBridge_;
    return UIMA_ERR_NONE;
  }

  void generateViewPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const tf::Vector3 centroid, float radius)
  {
    for(float theta = M_PI_2 + M_PI / 4; theta < M_PI - 2 * (M_PI / 18); theta += M_PI / 10)
    {
      for(float phi = -M_PI; phi < M_PI; phi += M_PI_2 / 15)
      {
        pcl::PointXYZ p;
        p.x =  radius / 2 * sin(theta) * cos(phi);
        p.y =  radius / 2 * sin(phi) * sin(theta);
        p.z =  radius * cos(theta) + centroid[2];
        cloud->push_back(p);
      }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
  }

  void publishViewPointsAsMarkers()
  {
    visualization_msgs::MarkerArray markers;
    int idx = 0;
    for(auto p : sphereCloud_->points)
    {
      visualization_msgs::Marker m;
      m.scale.x = 0.01;
      m.scale.y = 0.01;
      m.scale.z = 0.01;

      m.color.a = 1.0;
      m.color.r = 0.0;
      m.color.g = 1.0;
      m.color.b = 0.0;
      m.pose.position.x = p.x;
      m.pose.position.y = p.y;
      m.pose.position.z = p.z;

      m.pose.orientation.w = 1.0;
      m.pose.orientation.x = 0.0;
      m.pose.orientation.x = 0.0;
      m.pose.orientation.x = 0.0;

      m.header.frame_id = "map";
      m.header.stamp = ros::Time::now();
      m.ns = "view_points";
      m.id = idx++;
      m.action = visualization_msgs::Marker::ADD;
      m.type = visualization_msgs::Marker::SPHERE;
      markers.markers.push_back(m);
    }
    marker_pub_.publish(markers);
  }

  bool spawnCameraInUnreal(rs::Scene &scene, tf::Vector3 centroid)
  {
    tf::StampedTransform mapToCam, camToMap;
    rs::conversion::from(scene.viewPoint.get(), mapToCam);

    camToMap  = tf::StampedTransform(mapToCam.inverse(), mapToCam.stamp_, mapToCam.child_frame_id_, mapToCam.frame_id_);

    tf::Vector3 centroidInMap = mapToCam * centroid;
    outInfo("Centroid is: " << centroid[0] << " " << centroid[1] << " " << centroid[2]);
    outInfo("Centroid in map is: " << centroidInMap[0] << " " << centroidInMap[1] << " " << centroidInMap[2]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr sphereCloud(new pcl::PointCloud<pcl::PointXYZ>);
    generateViewPoints(sphereCloud, centroidInMap, centroid.length());
    outInfo("Sphere points size: " << sphereCloud->points.size());

    Eigen::Affine3d eigenTransform;
    tf::transformTFToEigen(mapToCam, eigenTransform);
    pcl::transformPointCloud<pcl::PointXYZ>(*sphereCloud, *sphereCloud_, eigenTransform);

    if(publishAsMarkers_)
    {
      publishViewPointsAsMarkers();
    }

    srand(time(NULL));
    int randomPointIdx = rand() % sphereCloud_->size() + 1;
    outInfo("Random index: " << randomPointIdx);

    pcl::PointXYZ pt = sphereCloud_->points[randomPointIdx];

    tf::StampedTransform mapToPlaneCenter, mapToPoint;
    mapToPlaneCenter.child_frame_id_ = "center_point";
    mapToPlaneCenter.frame_id_ = mapToCam.frame_id_;//map
    mapToPlaneCenter.setOrigin(centroidInMap);
    mapToPlaneCenter.setRotation(tf::Quaternion(0, 0, 0, 1));

    mapToPoint.child_frame_id_ = "view_point";
    mapToPoint.frame_id_ = "map";
    mapToPoint.setOrigin(tf::Vector3(pt.x, pt.y, pt.z));

    tf::Vector3 newZ((tf::Vector3(centroidInMap[0] - pt.x, centroidInMap[1] - pt.y, centroidInMap[2] - pt.z)).normalize());
    tf::Vector3 ZAxes(0.0, 0.0, 1.0);
    tf::Vector3 newX = (newZ.cross(ZAxes)).normalize();
    tf::Vector3 newY = (newZ.cross(newX)).normalize();


    outInfo(newX.dot(newY) << " " << newX.dot(newZ) << " " << newZ.dot(newY));
    tf::Matrix3x3 rotationMatrix(newX.x(), newX.y(), newX.z(),
                                 newY.x(), newY.y(), newY.z(),
                                 newZ.x(), newZ.y(), newZ.z());
    mapToPoint.setBasis(rotationMatrix.transpose());

    broadCasterObject_.clear();
    broadCasterObject_.addTransform(mapToPoint);
    broadCasterObject_.addTransform(mapToPlaneCenter);

    tf::Stamped<tf::Pose> poseStamped;
    poseStamped.setOrigin(mapToPoint.getOrigin());
    poseStamped.setRotation(mapToPoint.getRotation());
    robosherlock_msgs::UpdateObjects updateCameraPose;
    updateCameraPose.request.name = "ARGBDCamera";
    tf::poseStampedTFToMsg(poseStamped, updateCameraPose.request.pose_stamped);
    if(client_.call(updateCameraPose))
    {
      outInfo("Success!");
      return true;
    }
    else
    {
      outInfo("We failed!");
      return false;
    }
  }

  void countObjInliers(cv::Vec3b objectColor, std::vector<cv::Point> &points)
  {
    int totalX = 0,
        totalY = 0;
    #pragma omp parallel
    for(size_t r = 0; r < (size_t)object_.rows; ++r)
    {
      const cv::Vec3b *itC = object_.ptr<cv::Vec3b>(r);
      for(size_t c = 0; c < (size_t)object_.cols; ++c, ++itC)
      {
        if(*itC == objectColor)
        {
          #pragma omp critical
          {
            totalX += c;
            totalY += r;
            points.push_back(cv::Point(c, r));
          }
        }
      }
    }

    if(!points.empty())
    {
      int centerX = totalX / points.size(),
          centerY = totalY / points.size();
      if(centerX < 20 || centerX > object_.cols - 20 ||
         centerY < 20 || centerY > object_.rows - 20)
      {
        points.clear();
      }
      outInfo("Obj too on the side");
    }
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    MEASURE_TIME;
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    std::vector<rs::Plane> planes;
    scene.annotations.filter(planes);

    if(planes.empty())
    {
      return UIMA_ERR_NONE;
    }
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cas.get(VIEW_CLOUD, *cloud);

    rs::Plane &plane  = planes[0];
    std::vector<int> planeInliers = plane.inliers.get();
    Eigen::Vector4f temp;
    pcl::compute3DCentroid(*cloud, planeInliers, temp);
    tf::Vector3 centroid(temp[0], temp[1], temp[2]);

    Eigen::Affine3d eigenTransform;
    tf::StampedTransform mapToCam;
    rs::conversion::from(scene.viewPoint.get(), mapToCam);
    tf::transformTFToEigen(mapToCam, eigenTransform);
    pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud, *viewCloud_, eigenTransform);

    if(!spawnCameraInUnreal(scene, centroid))
    {
      return UIMA_ERR_NONE;
    }

    rs::Query qs = rs::create<rs::Query>(tcas);
    std::string jsonQuery;
    if(cas.getFS("QUERY", qs))
    {
      jsonQuery = qs.asJson();
      outDebug("json query: " << jsonQuery);
    }
    else
    {
      outWarn("No Query, skipping execution");
      return UIMA_ERR_NONE;
    }

    rapidjson::Document doc;
    doc.Parse(jsonQuery.c_str());

    std::string howToHighligh;
    if(!doc.HasMember("render"))
    {
      return UIMA_ERR_NONE;
    }

    howToHighligh = doc["render"].GetString();
    int count = 0; //this is a hack
    while(!unrealBridge_->newData() && count < 5)
    {
      count++;
      outInfo("Waiting for Unreal");
      usleep(100);
    }
    if(count >= 5)
    {
      return UIMA_ERR_NONE;
    }

    unrealBridge_->setData(tcas);


    sensor_msgs::CameraInfo cam_info;
    cas.get(VIEW_OBJECT_IMAGE, object_);
    cas.get(VIEW_COLOR_IMAGE, rgb_);
    cas.get(VIEW_CAMERA_INFO, cam_info);

    std::map<std::string, cv::Vec3b> objectMap;
    cas.get(VIEW_OBJECT_MAP, objectMap);

    cv_bridge::CvImage outImgMsgs;
    outImgMsgs.header = cam_info.header;
    outImgMsgs.encoding = sensor_msgs::image_encodings::BGR8;
    if(howToHighligh == "mask")
    {
      outImgMsgs.image = object_;
    }
    else
    {
      outImgMsgs.image = rgb_;
    }
    image_pub_.publish(outImgMsgs.toImageMsg());
    image_pub_.publish(outImgMsgs.toImageMsg());

    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat disp)
  {
    if(!rgb_.empty())
    {
      disp  = rgb_.clone();
    }
    else
    {
      disp = cv::Mat::ones(cv::Size(640, 480), CV_8UC3);
    }
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {

    const std::string cloudname = this->name;
    const std::string cloudname2 = "cloudInMap";
    double pointSize = 2.0;
    visualizer.addCoordinateSystem(0.5);
    if(firstRun)
    {
      visualizer.addPointCloud(sphereCloud_, cloudname);
      visualizer.addPointCloud(viewCloud_, cloudname2);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname2);
    }
    else
    {
      visualizer.updatePointCloud(sphereCloud_, cloudname);
      visualizer.updatePointCloud(viewCloud_, cloudname2);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname2);
    }
  }


};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(GetRenderedViews)
