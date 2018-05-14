//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>

#include <uima/api.hpp>

// Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/filesystem/path.hpp>

//RS
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/io/UnrealVisionBridge.h>
#include <rs/utils/output.h>
#include <rs/utils/common.h>
#include <rs/io/TFBroadcasterWrapper.hpp>
#include <rs/DrawingAnnotator.h>
#include <rs_queryanswering/KRDefinitions.h>
#include <rs/io/Storage.h>

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

//KnowRob
#include <json_prolog/prolog.h>
#include <json_prolog/prolog_bindings.h>

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

// basic file operations
#include <iostream>
#include <fstream>

// basic file operations
#include <iostream>
#include <fstream>

using namespace uima;

struct stat sb;

class UnrealTableScene : public DrawingAnnotator
{

  using ObjectMap = std::map<std::string, std::string>;
  using ClassLabelToID = std::map<std::string, int>;
  using ObjectAlternatives = std::map<std::string, std::vector<std::string>>;
  using VariationConfig = std::map<int, ObjectMap *>;

private:

  //needed for Yolo
  ClassLabelToID labelToID =
  {
    {"SM_obj_01", 1},
    {"SM_obj_02", 2},
    {"SM_obj_03", 3},
    {"SM_obj_04", 4},
    {"SM_obj_05", 5},
    {"SM_obj_06", 6},
    {"SM_obj_07", 7},
    {"SM_obj_08", 8},
    {"SM_obj_09", 9}
  };

  //vector of floats is the pose: x,y,z,qx,qy,qz,qw
  std::map < std::string, std::map<std::string, std::vector<float> > > sceneConfigs_ =
  {
    {
      "Scene01", {
        {"SM_obj_01", {0.0, -0.08, 0.46, 0.0, 0.0, 0.25, 1.0}},
        {"SM_obj_02", {0.0, 0.02, 0.46, 0.0, 0.0, 0.33, 1.0}},
        {"SM_obj_03", {0.0, 0.1, 0.46, 0.0, 0.0, 0.1, 1.0}},
        {"SM_obj_04", { -0.13, -0.01, 0.46, 0.0, 0.0, 0.7, 1.0}},
        {"SM_obj_05", { -0.11, 0.11, 0.46, 0.0, 0.0, 0.45, 1.0}}
      }
    },
    {
      "Scene02", {
        {"SM_obj_06", {0.0, -0.08, 0.46, 0.0, 0.0, 0.25, 1.0}},
        {"SM_obj_07", {0.0, 0.02, 0.46, 0.0, 0.0, 0.33, 1.0}},
        {"SM_obj_08", {0.0, 0.1, 0.46, 0.0, 0.0, 0.1, 1.0}},
        {"SM_obj_09", { -0.13, -0.01, 0.46, 0.0, 0.0, 0.7, 1.0}},
        {"SM_obj_01", { -0.11, 0.11, 0.46, 0.0, 0.0, 0.45, 1.0}}
      }
    },
    {
      "Scene03", {
        {"SM_obj_02", {0.0, -0.08, 0.46, 0.0, 0.0, 0.25, 1.0}},
        {"SM_obj_04", {0.0, 0.02, 0.46, 0.0, 0.0, 0.33, 1.0}},
        {"SM_obj_06", {0.0, 0.1, 0.46, 0.0, 0.0, 0.1, 1.0}},
        {"SM_obj_08", { -0.13, -0.01, 0.46, 0.0, 0.0, 0.7, 1.0}},
        {"SM_obj_09", { -0.11, 0.11, 0.46, 0.0, 0.0, 0.45, 1.0}}
      }
    },
    {
      "Scene04", {
        {"SM_obj_01", {0.0, -0.08, 0.46, 0.0, 0.0, 0.25, 1.0}},
        {"SM_obj_03", {0.0, 0.02, 0.46, 0.0, 0.0, 0.33, 1.0}},
        {"SM_obj_05", {0.0, 0.1, 0.46, 0.0, 0.0, 0.1, 1.0}},
        {"SM_obj_07", { -0.13, -0.01, 0.46, 0.0, 0.0, 0.7, 1.0}},
        {"SM_obj_09", { -0.11, 0.11, 0.46, 0.0, 0.0, 0.45, 1.0}}
      }
    },
    {
      "Scene05", {
        {"SM_obj_01", {0.0, -0.08, 0.46, 0.0, 0.0, 0.25, 1.0}},
        {"SM_obj_04", {0.0, 0.02, 0.46, 0.0, 0.0, 0.33, 1.0}},
        {"SM_obj_07", {0.0, 0.1, 0.46, 0.0, 0.0, 0.1, 1.0}},
        {"SM_obj_08", { -0.13, -0.01, 0.46, 0.0, 0.0, 0.7, 1.0}},
        {"SM_obj_06", { -0.11, 0.11, 0.46, 0.0, 0.0, 0.45, 1.0}}
      }
    },
    {
      "Scene06", {
        {"SM_obj_02", {0.0, -0.011, 0.46, 0.0, 0.0, 0.0, 1.0}},
        {"SM_obj_05", {0.0, -0.02, 0.46, 0.0, 0.0, 0.0, 1.0}},
        {"SM_obj_08", {0.0, -0.06033, 0.46, 0.0, 0.0, 0.0, 1.0}},
        {"SM_obj_03", {0.0, 0.076, 0.46, 0.0, 0.0, 0.0, 1.0}},
        {"SM_obj_04", {0.0, -0.0912, 0.46, 0.0, 0.0, 0.0, 1.0}}
      }
    },
    {
      "Scene07", {
        {"SM_obj_08", {0.0, -0.053, 0.46, 0.0, 0.0, 0.0, 1.0}},
        {"SM_obj_07", {0.0, -0.0016, 0.46, 0.0, 0.0, 0.0, 1.0}},
        {"SM_obj_06", {0.0, -0.085, 0.46, 0.0, 0.0, 0.0, 1.0}},
        {"SM_obj_05", {0.0, 0.0442, 0.46, 0.0, 0.0, 0.0, 1.0}},
        {"SM_obj_04", {0.0, 0.072, 0.46, 0.0, 0.0, 0.0, 1.0}}
      }
    },
    {
      "Scene08", {
        {"SM_obj_09", {0.0, -0.0744, 0.46, 0.0, 0.0, 0.0, 1.0}},
        {"SM_obj_06", {0.0, -0.0408, 0.46, 0.0, 0.0, 0.0, 1.0}},
        {"SM_obj_05", {0.0, -0.13, 0.46, 0.0, 0.0, 0.0, 1.0}},
        {"SM_obj_03", {0.0, -0.106, 0.46, 0.0, 0.0, 0.0, 1.0}},
        {"SM_obj_02", {0.0, -0.162, 0.46, 0.0, 0.0, 0.0, 1.0}}
      }
    },
    {
      "Scene09", {
        {"SM_obj_01", {0.0, 0.0249, 0.46, 0.0, 0.0, 0.0, 1.0}},
        {"SM_obj_03", {0.0, -0.104, 0.46, 0.0, 0.0, 0.0, 1.0}},
        {"SM_obj_07", {0.0, -0.0218, 0.46, 0.0, 0.0, 0.0, 1.0}},
        {"SM_obj_08", {0.0, 0.0672, 0.46, 0.0, 0.0, 0.0, 1.0}},
        {"SM_obj_09", {0.0, -0.072, 0.46, 0.0, 0.0, 0.0, 1.0}}
      }
    }
  };



  ObjectAlternatives objectAlternatives_;

  VariationConfig variationsConfig_;

  ros::ServiceClient client_;
  ros::NodeHandle nh_;

  UnrealVisionBridge *unrealBridge_;

  image_transport::Publisher image_pub_;
  image_transport::ImageTransport it_;
  cv::Mat object_, rgb_, depth_, disp_;

  std::thread thread_;
  TFBroadcasterWrapper broadCasterObject_;
  ros::Publisher marker_pub_, object_marker_pub_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr sphereCloud_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr viewCloud_;

  bool publishAsMarkers_;

  std::string host;
  std::string db;
  rs::Storage storage;

  bool withStorage;

  int variations;
  bool first;

  ofstream imageListFile_;

public:
  UnrealTableScene(): DrawingAnnotator(__func__), nh_("~"), it_(nh_), publishAsMarkers_(false),
    host("127.0.0.1"), db("THRGeneratedScenes"), withStorage(false), variations(1), first(true)
  {
    client_ = nh_.serviceClient<robosherlock_msgs::UpdateObjects>("/update_objects");
    std::string configFile = ros::package::getPath("robosherlock") + "/config/config_unreal_vision.ini";
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(configFile, pt);
    unrealBridge_ = new UnrealVisionBridge(pt);

    image_pub_ = it_.advertise("rendered_image", 5, false);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("viewpoints", 1, true);

    object_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 1, true);

    imageListFile_.open("images.txt");

  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    if(ctx.isParameterDefined("withStorage"))
    {
      ctx.extractValue("withStorage", withStorage);
    }
    if(ctx.isParameterDefined("variations"))
    {
      ctx.extractValue("variations", variations);
    }

    ros::service::waitForService("/json_prolog/simple_query");

    thread_ = std::thread(&TFBroadcasterWrapper::run, &broadCasterObject_);
    sphereCloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    viewCloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

    if(withStorage)
    {
      storage = rs::Storage(host, db, false);
      storage.enableViewStoring("color_image_hd", true);
      storage.enableViewStoring("depth_image_hd", true);
      storage.enableViewStoring("camera_info", true);
      storage.enableViewStoring("camera_info_hd", true);
      storage.enableViewStoring("scene", true);
    }
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    imageListFile_.close();
    for(VariationConfig::iterator it = variationsConfig_.begin();
        it != variationsConfig_.end(); ++it)
    {
      delete it->second;
    }
    delete unrealBridge_;
    return UIMA_ERR_NONE;
  }

  void generateViewPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const tf::Vector3 centroid, float radius)
  {
    for(float u = 0; u < 1; u += 0.1)
    {
      for(float theta = 0; theta < 6.28; theta += M_PI / 18)
      {
        pcl::PointXYZ p;
        p.x = radius * (sqrt(1 - u * u) * cos(theta)) + centroid[0];
        p.y = radius * (sqrt(1 - u * u) * sin(theta)) + centroid[1] ;
        p.z = radius * u + centroid[2];
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



  bool spawnCameraInUnreal(tf::Vector3 centroid, pcl::PointXYZ pt, double rot)
  {

    tf::StampedTransform mapToPoint;
    //    mapToPlaneCenter.child_frame_id_ = "center_point";
    //    mapToPlaneCenter.frame_id_ = mapToCam.frame_id_;//map
    //    mapToPlaneCenter.setOrigin(centroidInMap);
    //    mapToPlaneCenter.setRotation(tf::Quaternion(0, 0, 0, 1));

    mapToPoint.child_frame_id_ = "view_point";
    mapToPoint.frame_id_ = "map";
    mapToPoint.setOrigin(tf::Vector3(pt.x, pt.y, pt.z));

    tf::Vector3 newZ((tf::Vector3(centroid[0] - pt.x, centroid[1] - pt.y, centroid[2] - pt.z)).normalize());
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
    //    broadCasterObject_.addTransform(mapToPlaneCenter);

    tf::Quaternion hack(0, 0, rot);
    tf::Stamped<tf::Pose> poseStamped;
    poseStamped.setOrigin(mapToPoint.getOrigin());
    poseStamped.setRotation(mapToPoint.getRotation()*hack);

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
        outInfo("Obj too on the side");
      }

    }
  }

  void publishObjects(const std::map<std::string , std::vector<float>> &objects)
  {
    visualization_msgs::MarkerArray markers;
    int idx = 0;
    outInfo(objects.size());
    for(auto obj : objects)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "rs";
      marker.id = idx++;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = obj.second[0];
      marker.pose.position.y = obj.second[1];
      marker.pose.position.z = obj.second[2];
      marker.pose.orientation.x = obj.second[3];
      marker.pose.orientation.y = obj.second[4];
      marker.pose.orientation.z = obj.second[5];
      marker.pose.orientation.w = obj.second[6];

      marker.lifetime = ros::Duration(30, 0);
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      std::string name = obj.first;

      std::string mesh_resource = "package://rs_resources/objects_dataset/cad_models/" + name + "/" + name + ".dae";
      marker.mesh_resource = mesh_resource;
      marker.mesh_use_embedded_materials = true;
      marker.scale.x = 1.0f;
      marker.scale.y = 1.0f;
      marker.scale.z = 1.0f;
      marker.color.a = 1.0;
      markers.markers.push_back(marker);
    }
    outInfo("Publishing " << markers.markers.size() << " markers");
    object_marker_pub_.publish(markers);
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    MEASURE_TIME;
    sphereCloud_->clear();
    //    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    tf::Vector3 centroid(0, 0, 0.46);
    generateViewPoints(sphereCloud_, centroid, 0.6);
    int counter;
    outInfo("Generated " << sphereCloud_->points.size() << "view points");

    for(auto sceneConfig : sceneConfigs_)
    {
      publishObjects(sceneConfig.second);

      if(!boost::filesystem::exists(sceneConfig.first))
      {
        boost::filesystem::create_directory(sceneConfig.first);
      }
      counter = 0;
      for(auto point : sphereCloud_->points)
      {
        std::vector<double> rots = {0.0, M_PI_4, M_PI_2,
                                    M_PI_2 + M_PI_4, M_PI,
                                    M_PI + M_PI_4, M_PI + M_PI_2,
                                    M_PI + M_PI_2 + M_PI_4
                                   };
        //        std::vector<double> rots = {0.0, M_PI_2, M_PI, M_PI + M_PI_2};
        for(auto rot : rots)
        {
          //if we don't reset here it fills the memory
          cas.reset();
          cas.setText("variations");
          spawnCameraInUnreal(centroid, point, rot);
          outInfo("Waiting for Unreal");
          sleep(1);
          while(!unrealBridge_->newData())//&& count < 5)
          {
            usleep(100);
          }
          outInfo("Setting data from unreal");
          unrealBridge_->setData(tcas);

          outInfo("Data form Unreal successfully set to CAS");
          cas.get(VIEW_OBJECT_IMAGE_HD, object_);
          cas.get(VIEW_COLOR_IMAGE_HD, rgb_);

          cv::Mat rgbLowRes;
          cas.get(VIEW_COLOR_IMAGE, rgbLowRes);
          std::stringstream imgFileName;

          imgFileName << sceneConfig.first << "/unreal_color_" << std::setfill('0') << std::setw(3) <<
                      counter << "_" <<     setprecision(3) << rot << "_" << scene.timestamp();
          outWarn(imgFileName.str());
          cv::imwrite(imgFileName.str() + ".png", rgb_);
          imageListFile_ << imgFileName.str() << ".png" << std::endl;
          std::map<std::string, cv::Vec3b> objectMap;
          cas.get(VIEW_OBJECT_MAP, objectMap);
          ofstream gtFileYolo;
          gtFileYolo.open(imgFileName.str() + ".txt");
          scene.identifiables.allocate();

          for(auto obj : labelToID)
          {
            for(auto o : objectMap)
            {
              if(obj.first != "" && o.first.find(obj.first) != std::string::npos)
              {
                std::vector<cv::Point> points;
                countObjInliers(o.second, points);
                outError("Found: " << o.first << ":" << points.size());
                if(points.size() > 100)
                {
                  rs::Cluster uimaCluster = rs::create<rs::Cluster>(tcas);
                  rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
                  pcl::PointIndices indices;
                  cv::Mat mask_full = cv::Mat::zeros(rgb_.rows, rgb_.cols, CV_8U);
                  for(cv::Point &p : points)
                  {
                    indices.indices.push_back(p.y / 2 * rgb_.cols / 2 + p.x / 2);
                    mask_full.at<uint8_t>(p.y, p.x) = 255;
                  }
                  std::set<int> tempSet(indices.indices.begin(), indices.indices.end());
                  indices.indices.assign(tempSet.begin(), tempSet.end());
                  rs::PointIndices uimaIndices = rs::conversion::to(tcas, indices);
                  rcp.indices.set(uimaIndices);

                  cv::Rect roiHires = cv::boundingRect(points),
                           roi = cv::Rect(roiHires.x >> 1, roiHires.y >> 1,
                                          roiHires.width >> 1, roiHires.height >> 1);
                  cv::Mat mask, maskHires;
                  mask_full(roiHires).copyTo(maskHires);
                  //apparently it can happen the that cluster is way too small to downscale it like this
                  try
                  {
                    cv::resize(maskHires, mask, cv::Size(0, 0), 0.5, 0.5, cv::INTER_NEAREST);
                  }
                  catch(cv::Exception &e)
                  {
                    const char *err_msg = e.what();
                    outError("exception caught: " << err_msg);
                    continue;
                  }

                  rs::ImageROI imageRoi = rs::create<rs::ImageROI>(tcas);
                  imageRoi.mask(rs::conversion::to(tcas, mask));
                  imageRoi.mask_hires(rs::conversion::to(tcas, maskHires));
                  imageRoi.roi(rs::conversion::to(tcas, roi));
                  imageRoi.roi_hires(rs::conversion::to(tcas, roiHires));

                  uimaCluster.points.set(rcp);
                  uimaCluster.rois.set(imageRoi);
                  uimaCluster.source.set("UnrealTableSceneGeneration");

                  rs::GroundTruth gt = rs::create<rs::GroundTruth>(tcas);
                  rs::Classification classification = rs::create<rs::Classification>(tcas);
                  classification.classification_type.set("ground_truth");
                  classification.classname.set(obj.first);
                  classification.classifier.set("UnrealEngine");
                  classification.source.set("OffScreenSceneVariation");
                  gt.classificationGT.set(classification);
                  uimaCluster.annotations.append(gt);
                  scene.identifiables.append(uimaCluster);

                  float obj_center_x = (roiHires.x + roiHires.width / 2) / (float)(rgb_.cols);
                  float obj_center_y = (roiHires.y + roiHires.height / 2) / (float)(rgb_.rows);
                  gtFileYolo << obj.second << " " << obj_center_x << " " << obj_center_y << " "
                             << roiHires.width / (float)rgb_.cols << " " << roiHires.height / (float)rgb_.rows << std::endl;
                }
              }
            }
          }
          gtFileYolo.close();
          if(withStorage)
          {
            scene.id.set(::mongo::OID::gen().toString());
            storage.storeScene(*tcas.getBaseCas(), (uint64_t)scene.timestamp());
          }

        }
        counter++;

      }
    }

    exit(0);
    return UIMA_ERR_NONE;
  }
  void drawImageWithLock(cv::Mat &disp)
  {
    if(!rgb_.empty())
      disp  = rgb_.clone();
    else
      disp = cv::Mat::ones(640, 480, CV_8UC3);
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
MAKE_AE(UnrealTableScene)
