//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>

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
#include <rs_queryanswering/KRDefinitions.h>
#include <rs/io/Storage.h>

//ROS
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <iai_robosherlock_msgs/UpdateObjects.h>

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
using namespace std;






using namespace uima;

class OffScreenSceneVariation : public DrawingAnnotator
{

  using ObjectMap = std::map<std::string, std::string>;
  using ClassLabelToID = std::map<std::string, int>;
  using ObjectAlternatives = std::map<std::string, std::vector<std::string>>;
  using VariationConfig = std::map<int, ObjectMap *>;
private:

  ClassLabelToID labelToID =
  {
    {"BluePlasticSpoon", 0}, {"EdekaRedBowl", 1},
    {"JaMilch", 2}, {"KelloggsCornFlakes", 3}, {"KelloggsToppasMini", 4},
    {"KnusperSchokoKeks", 5}, {"KoellnMuesliKnusperHonigNuss", 6},
    {"LargeGreySpoon", 7}, {"LionCerealBox", 8},
    {"NesquikCereal", 9}, {"RedMetalBowlWhiteSpeckles", 10},
    {"RedPlasticSpoon", 11},      {"SojaMilch", 12},
    {"VollMilch", 13},      {"WeideMilchSmall", 14},
    {"WhiteCeramicIkeaBowl", 15},      {"AlbiHimbeerJuice", 16},
    {"BlueCeramicIkeaMug", 17},      {"BlueMetalPlateWhiteSpeckles", 18},
    {"BluePlasticKnife", 19},      {"CupEcoOrange", 20},
    {"JodSalz", 21},      {"LinuxCup", 22},
    {"MarkenSalz", 23},      {"MeerSalz", 24},
    {"PfannerGruneIcetea", 25},      {"PfannerPfirsichIcetea", 26},
    {"RedMetalCupWhiteSpeckles", 27},      {"RedMetalPlateWhiteSpeckles", 28},
    {"RedPlasticKnife", 29},      {"YellowCeramicPlate", 30}
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
  OffScreenSceneVariation(): DrawingAnnotator(__func__), nh_("~"), it_(nh_), publishAsMarkers_(false),
    host("127.0.0.1"), db("UnrealGeneratedScenes"), withStorage(false), variations(1), first(true)
  {
    client_ = nh_.serviceClient<iai_robosherlock_msgs::UpdateObjects>("/update_objects");
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
    cloud->points.push_back(pcl::PointXYZ(0, 0, 0));
    for(float theta = M_PI_2 + M_PI / 4; theta < M_PI - 2 * (M_PI / 18); theta += M_PI / 20)
    {
      for(float phi = -M_PI; phi < M_PI; phi += M_PI_2 / 20)
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

  void findObjectCandidates(std::string objectName, std::vector<std::string> &objectCandidates)
  {
    json_prolog::Prolog pl;
    std::stringstream plQuery;

    plQuery << "owl_direct_subclass_of(kitchen:'" << objectName << "',Superclass),";
    plQuery << "rdf_all_similar(kitchen:'" << objectName << "',Superclass,D)";
    //    outDebug("Asking query:" << plQuery.str());
    json_prolog::PrologQueryProxy bdgs = pl.query(plQuery.str());
    //    outDebug("Found solution: " << (bool)(bdgs.begin() != bdgs.end()));

    for(json_prolog::PrologQueryProxy::iterator it = bdgs.begin();
        it != bdgs.end(); it++)
    {
      json_prolog::PrologBindings bdg = *it;
      json_prolog::PrologValue val = bdg["D"];
      if(val.isList())
      {
        std::vector<json_prolog::PrologValue> values = val.as<std::vector<json_prolog::PrologValue>>();
        for(json_prolog::PrologValue &v : values)
        {
          assert(v.isList());
          std::vector<json_prolog::PrologValue> entries = v.as<std::vector<json_prolog::PrologValue>>();
          assert(!entries.empty());

          plQuery.str(std::string());
          plQuery << "owl_class_properties('" << entries[0].toString() << "',knowrob:'pathToCadModel',_)";
          //          outDebug("Asking query:" << plQuery.str());
          json_prolog::PrologQueryProxy results = pl.query(plQuery.str());
          //          outDebug("Found class property: " << (bool)(results.begin() != results.end()));
          if(results.begin() != results.end())
          {
            std::string candidateObj = entries[0].toString();
            candidateObj = candidateObj.substr(candidateObj.find_last_of("#") + 1, candidateObj.size());
            objectCandidates.push_back(candidateObj);
          }
        }
      }
    }

    //    objectCandidates.erase(std::unique(objectCandidates.begin(),objectCandidates.end()),objectCandidates.end());
    std::set<std::string> set(objectCandidates.begin(), objectCandidates.end());
    objectCandidates.assign(set.begin(), set.end());
    outInfo("Object candidates for " << objectName << " are: ");
    for(auto s : objectCandidates)
      outInfo("     -" << s);
  }

  template <class T>
  void publishObjects(const std::vector<T> &objects, ObjectMap &objReplacement)
  {
    visualization_msgs::MarkerArray markers;
    int idx = 0;
    outInfo(objects.size());
    for(T obj : objects)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "rs";
      marker.id = idx++;
      marker.action = visualization_msgs::Marker::ADD;

      std::vector<rs::Detection> detections;
      obj.annotations.filter(detections);
      outInfo("Detections: " << detections.size());
      std::vector<rs::Geometry> geom;
      obj.annotations.filter(geom);
      if(!geom.empty())
      {
        rs::Geometry &g = geom[0];
        tf::Stamped<tf::Pose> pose;
        rs::conversion::from(g.world(), pose);
        marker.pose.position.x = pose.getOrigin().x();
        marker.pose.position.y = pose.getOrigin().y();
        marker.pose.position.z = pose.getOrigin().z();
        marker.pose.orientation.x = pose.getRotation().x();
        marker.pose.orientation.y = pose.getRotation().y();
        marker.pose.orientation.z = pose.getRotation().z();
        marker.pose.orientation.w = pose.getRotation().w();

        marker.scale.x = g.boundingBox().width();
        marker.scale.y = g.boundingBox().height();
        marker.scale.z = g.boundingBox().depth();
      }
      marker.lifetime = ros::Duration(30, 0);
      if(!detections.empty())
      {
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        std::string name = detections[0].name();
        resource_retriever::Retriever r;
        std::string mesh_resource = "package://rs_resources/objects_dataset/cad_models/" + objReplacement[name] + "/" + objReplacement[name] + ".dae";
        //        std::string mesh_resource = "package://rs_resources/objects_dataset/cad_models/" + name + "/" + name + ".dae";
        try
        {
          r.get(mesh_resource);
          marker.mesh_resource = mesh_resource;
          marker.mesh_use_embedded_materials = true;
          marker.scale.x = 1.0f;
          marker.scale.y = 1.0f;
          marker.scale.z = 1.0f;
          marker.color.a = 1.0;
        }
        catch(resource_retriever::Exception &e)
        {
          outWarn(e.what());
        }

      }
      markers.markers.push_back(marker);
    }
    outInfo("Publishing " << markers.markers.size() << " markers");
    object_marker_pub_.publish(markers);
  }

  bool spawnCameraInUnreal(/*rs::Scene &scene,*/tf::StampedTransform mapToCam, tf::Vector3 centroid)
  {
    tf::StampedTransform /*mapToCam,*/ camToMap;
    //    rs::conversion::from(scene.viewPoint.get(), mapToCam);

    camToMap  = tf::StampedTransform(mapToCam.inverse(), mapToCam.stamp_, mapToCam.child_frame_id_, mapToCam.frame_id_);

    tf::Vector3 centroidInMap = mapToCam * centroid;
    outInfo("Centroid is: " << centroid[0] << " " << centroid[1] << " " << centroid[2]);
    outInfo("Centroid in map is: " << centroidInMap[0] << " " << centroidInMap[1] << " " << centroidInMap[2]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr sphereCloud(new pcl::PointCloud<pcl::PointXYZ>);
    //points are generated in camera frame where center of the sphere is the original location of the
    //camera (0, 0, 0) and then transformed to map frame;
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

    //no random: replace with randomPointIdx to get random view-points
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

    iai_robosherlock_msgs::UpdateObjects updateCameraPose;
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

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    MEASURE_TIME;

    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);
    outInfo("Found " << clusters.size() << " object hypotheses");


    tf::StampedTransform mapToCam;
    for (int counter = 0; counter < variations; counter++)
    {
      ObjectMap *objReplacement;
      if(!first)
        objReplacement = variationsConfig_[counter];
      else
        objReplacement = new ObjectMap();

      outInfo("Variation " << counter << ":");
      for(auto obj : *objReplacement)
      {
        outInfo(obj.first << ": " << obj.second);
      }
      if(counter == 0)
      {
        rs::conversion::from(scene.viewPoint.get(), mapToCam);
      }
      for(auto c : clusters)
      {
        std::vector<rs::Detection> detections;
        c.annotations.filter(detections);
        if(detections.empty()) continue;

        std::string name = detections[0].name();
        //if we don't have any alternative objects yet
        if(objectAlternatives_.find(name) == objectAlternatives_.end())
        {
          std::vector<std::string> objectCandidates;
          findObjectCandidates(name, objectCandidates);
          objectAlternatives_[name]  = objectCandidates;
        }
        //if
        if(objReplacement->find(name) == objReplacement->end())
        {
          srand(time(NULL));
          int randomPointIdx = rand() % (objectAlternatives_[name].size() + 1);
          if(randomPointIdx < objectAlternatives_[name].size())
          {
            outInfo(name << " REPLACED BY " << objectAlternatives_[name].at(randomPointIdx));
            (*objReplacement)[name] = objectAlternatives_[name].at(randomPointIdx);
          }
          else
          {
            (*objReplacement)[name] = "";//if we don't want this object in the scene;
          }
        }
        if(first)
          variationsConfig_[counter] = objReplacement;
      }

      publishObjects(clusters, *variationsConfig_[counter]);
      sleep(2);
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

      if(!spawnCameraInUnreal(mapToCam, centroid))
      {
        return UIMA_ERR_NONE;
      }

      //for visualizations
      Eigen::Affine3d eigenTransform;
      tf::StampedTransform mapToCam;
      //    rs::conversion::from(scene.viewPoint.get(), mapToCam);
      tf::transformTFToEigen(mapToCam, eigenTransform);
      pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud, *viewCloud_, eigenTransform);


      int count = 0; //so we don't block
      outInfo("Waiting for Unreal");
      sleep(2);
      while(!unrealBridge_->newData())//&& count < 5)
      {
        count++;
        usleep(100);
      }

      std::stringstream ss;
      ss << "_" << counter << "_" << scene.timestamp();
      //  cas.get(VIEW_COLOR_IMAGE, rgb_);
      // cv::imwrite("original_color"+ss.str()+".png",rgb_);
      outInfo("Setting data from unreal");
      unrealBridge_->setData(tcas);
      outInfo("Data form Unreal successfully set to CAS");

      cas.get(VIEW_OBJECT_IMAGE_HD, object_);
      cas.get(VIEW_COLOR_IMAGE_HD, rgb_);

      cv::Mat rgbLowRes;
      cas.get(VIEW_COLOR_IMAGE, rgbLowRes);
      std::stringstream imgFileName;
      imgFileName << "unreal_color_" << counter << "_" << scene.timestamp();
      cv::imwrite(imgFileName.str() + ".png", rgbLowRes);
      imageListFile_ << imgFileName.str() <<".png"<< std::endl;

      std::map<std::string, cv::Vec3b> objectMap;
      cas.get(VIEW_OBJECT_MAP, objectMap);
      ofstream gtFileYolo;
      gtFileYolo.open(imgFileName.str() + ".txt");
      scene.identifiables.allocate();
      for(auto obj : *variationsConfig_[counter])
      {
        for(auto o : objectMap)
        {
          if(obj.second != "" && o.first.find(obj.second) != std::string::npos)
          {

            std::vector<cv::Point> points;
            countObjInliers(o.second, points);
            outError("Found: " << o.first << ":" << points.size());
            if(points.size() > 0)
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
              cv::resize(maskHires, mask, cv::Size(0, 0), 0.5, 0.5, cv::INTER_NEAREST);

              rs::ImageROI imageRoi = rs::create<rs::ImageROI>(tcas);
              imageRoi.mask(rs::conversion::to(tcas, mask));
              imageRoi.mask_hires(rs::conversion::to(tcas, maskHires));
              imageRoi.roi(rs::conversion::to(tcas, roi));
              imageRoi.roi_hires(rs::conversion::to(tcas, roiHires));

              uimaCluster.points.set(rcp);
              uimaCluster.rois.set(imageRoi);
              uimaCluster.source.set("EuclideanClustering");

              rs::GroundTruth gt = rs::create<rs::GroundTruth>(tcas);
              rs::Classification classification = rs::create<rs::Classification>(tcas);
              classification.classification_type.set("ground_truth");
              classification.classname.set(obj.second);
              classification.classifier.set("UnrealEngine");
              classification.source.set("OffScreenSceneVariation");
              gt.classificationGT.set(classification);
              uimaCluster.annotations.append(gt);
              scene.identifiables.append(uimaCluster);

              float obj_center_x = (roi.x+roi.width/2)/ (float)(rgbLowRes.cols);
              float obj_center_y = (roi.y+roi.height/2)/(float)(rgbLowRes.rows);
              gtFileYolo<<labelToID[obj.second]<<" "<<obj_center_x<<" "<<obj_center_y<<" "
                                              <<roi.width/(float)rgbLowRes.cols<<" "<<roi.height/(float)rgbLowRes.rows<<std::endl;
            }
          }
        }
      }
      gtFileYolo.close();
      //scene.identifiables.se
      if(withStorage)
      {
        scene.id.set(::mongo::OID::gen().toString());
        storage.storeScene(*tcas.getBaseCas(), (uint64_t)scene.timestamp());
      }
    }
    first = false;
    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    if(!object_.empty())
      disp  = object_.clone();
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
MAKE_AE(OffScreenSceneVariation)
