/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *         Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *         Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <uima/api.hpp>

#include <ctype.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/common/common.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>

#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/utils/common.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/exception.h>
#include <rs/io/TFListenerProxy.h>

#include <rapidjson/document.h>
#include <rapidjson/pointer.h>

#include <yaml-cpp/yaml.h>

using namespace uima;

class RegionFilter : public DrawingAnnotator
{

    /**
   * @brief The SemanticMapItem struct
   * transform describing transformation from reference frame to center of region you are definging
   * Dimensions are defined on each axis. If defining a regins that should have a supporting plane you
   * can optionally add the equation of the plane to the map entry
   */
  struct SemanticMapItem
  {
    tf::Transform transform;
    std::string reference_frame;
    std::string name, type;
    double y_dimention, z_dimension, x_dimention;
    cv::Vec4f plane_eq;
    bool hasPlaneEq = false;
  };
  std::vector<SemanticMapItem> semantic_map_items_;
  std::vector<SemanticMapItem> active_semantic_map_items_;

  typedef pcl::PointXYZRGBA PointT;

  double pointSize;
  float border;

  cv::Mat color, depth;
  pcl::PointCloud<PointT>::Ptr cloud;
  pcl::IndicesPtr indices;

  tf::StampedTransform camToWorld, worldToCam;

  std::vector<std::string> defaultRegions;
  std::vector<std::string> regions_to_look_at_;
  // name mapping for queries
  std::map<std::string, std::string> nameMapping;

  // for change detection
  bool changeDetection, frustumCulling_;
  cv::Mat lastImg, lastMask;
  float threshold, pixelThreshold, depthThreshold;
  std::vector<int> changes, lastIndices;
  size_t frames, filtered;
  ros::Time lastTime;
  uint32_t timeout;

  // for frustum culling
  sensor_msgs::CameraInfo cameraInfo;
  pcl::visualization::Camera camera;

  rs::TFListenerProxy listener_;
  double frustum[24];

public:
  RegionFilter()
    : DrawingAnnotator(__func__)
    , pointSize(1)
    , border(0.05)
    , cloud(new pcl::PointCloud<PointT>())
    , indices(new std::vector<int>())
    , changeDetection(true)
    , frustumCulling_(false)
    , threshold(0.1)
    , pixelThreshold(0.1)
    , depthThreshold(0.01)
    , frames(0)
    , filtered(0)
    , lastTime(ros::Time::now())
    , timeout(120)
  {
  }

  TyErrorId initialize(AnnotatorContext& ctx)
  {
    outInfo("initialize");

    if (ctx.isParameterDefined("border"))
      ctx.extractValue("border", border);

    std::vector<std::string*> temp;
    if (ctx.isParameterDefined("defaultRegions"))
    {
      ctx.extractValue("defaultRegions", temp);
      for (auto s : temp)
      {
        outDebug(*s);
        defaultRegions.push_back(*s);
      }
    }

    if (ctx.isParameterDefined("enable_change_detection"))
      ctx.extractValue("enable_change_detection", changeDetection);
    if (ctx.isParameterDefined("enable_frustum_culling"))
      ctx.extractValue("enable_frustum_culling", frustumCulling_);
    if (ctx.isParameterDefined("pixel_threshold"))
      ctx.extractValue("pixel_threshold", pixelThreshold);
    if (ctx.isParameterDefined("depth_threshold"))
      ctx.extractValue("depth_threshold", depthThreshold);
    if (ctx.isParameterDefined("global_threshold"))
      ctx.extractValue("global_threshold", threshold);
    if (ctx.isParameterDefined("change_timeout"))
    {
      int tmp = 120;
      ctx.extractValue("change_timeout", tmp);
      timeout = tmp;
    }
    if (ctx.isParameterDefined("semantic_map_definition"))
    {
      std::string file;
      ctx.extractValue("semantic_map_definition", file);
      readSemanticMap(file);
    }
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  bool readSemanticMap(const std::string& file)
  {
    std::vector<std::string> search_paths = rs::common::getRSSearchPaths("/config");
    std::string path_to_sem_map;
    for (auto p : search_paths)
      if (boost::filesystem::is_regular_file(boost::filesystem::path(p + "/" + file)))
        path_to_sem_map = p + "/" + file;

    if (path_to_sem_map.empty())
      throw rs::InitAEException(this->name, "Semantic map file not found: " + file);

    outDebug("Path to semantic map file: " FG_BLUE << path_to_sem_map);

    YAML::Node config = YAML::LoadFile(path_to_sem_map);

    if (config["names"])
    {
      auto sem_map_entries = config["names"].as<std::vector<std::string>>();
      semantic_map_items_.resize(sem_map_entries.size());
      for (size_t i = 0; i < sem_map_entries.size(); ++i)
      {
        SemanticMapItem& item = semantic_map_items_[i];
        if (!config[sem_map_entries[i]])
          throw rs::InitAEException(this->name, "Semantic map file is malformed. The region: " + sem_map_entries[i] +
                                                    " has no transformation attached to it!!");
        YAML::Node entry = config[sem_map_entries[i]];

        item.name = sem_map_entries[i];
        item.type = entry["type"].as<std::string>();
        item.x_dimention = entry["depth"].as<double>();  //X-dim
        item.y_dimention = entry["width"].as<double>(); //Y-dim
        item.z_dimension = entry["height"].as<double>(); //Z-dim

        if (entry["reference_frame"])
          item.reference_frame = entry["reference_frame"].as<std::string>();
        else
          item.reference_frame = "map";

        if (entry["plane_eq"])
        {
          item.hasPlaneEq = true;
          std::vector<float> plane_eq = entry["plane_eq"].as<std::vector<float>>();
          item.plane_eq[0] = plane_eq[0];
          item.plane_eq[1] = plane_eq[1];
          item.plane_eq[2] = plane_eq[2];
          item.plane_eq[3] = plane_eq[3];
          outDebug("Found a pre-defined plane equation for " << item.name << " : " << item.plane_eq);
        }

        cv::Mat m;
        YAML::Node transform = entry["transform"];
        int rows = transform["rows"].as<int>();
        int cols = transform["cols"].as<int>();
        std::vector<double> trans_matrix = transform["data"].as<std::vector<double>>();
        if(rows*cols != static_cast<int>(trans_matrix.size()))
          throw rs::InitAEException(this->name, "Semantic map file is malformed. The region: " + sem_map_entries[i] +
                                          " has a bad transformation!! cols*rows != data.size()");

        tf::Matrix3x3 rot;
        tf::Vector3 trans;
        rot.setValue(trans_matrix.at(0),trans_matrix.at(1),trans_matrix.at(2),
                     trans_matrix.at(4),trans_matrix.at(5),trans_matrix.at(6),
                     trans_matrix.at(8),trans_matrix.at(9),trans_matrix.at(10));
        trans.setValue(trans_matrix.at(3),trans_matrix.at(7),trans_matrix.at(11));
        item.transform = tf::Transform(rot, trans);
      }
    }
    else
      throw rs::InitAEException(this->name, "Semantic map file is malformed. Semantic map file needs to have a \"name\""
                                            " tag containing a list of all semantic region names");

  }

private:
  TyErrorId processWithLock(CAS& tcas, ResultSpecification const& res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    std::vector<rs::SemanticMapObject> semanticMap;
    semanticMap.reserve(semantic_map_items_.size());
    for (size_t i = 0; i < semantic_map_items_.size(); ++i)
    {
      SemanticMapItem& item = semantic_map_items_[i];
      rs::SemanticMapObject obj = rs::create<rs::SemanticMapObject>(tcas);
      obj.name(item.name);
      obj.typeName(item.type);
      obj.width(static_cast<double>(item.y_dimention));
      obj.height(static_cast<double>(item.z_dimension));
      obj.depth(static_cast<double>(item.x_dimention));
      obj.transform(rs::conversion::to(tcas, item.transform));
      semanticMap.push_back(obj);
    }
    cas.set(VIEW_SEMANTIC_MAP, semanticMap);

    cas.get(VIEW_CLOUD, *cloud);
    cas.get(VIEW_COLOR_IMAGE, color);
    cas.get(VIEW_DEPTH_IMAGE, depth);
    cas.get(VIEW_CAMERA_INFO, cameraInfo);

    indices->clear();
    indices->reserve(cloud->points.size());

    camToWorld.setIdentity();
    if (scene.viewPoint.has())
    {
      rs::conversion::from(scene.viewPoint.get(), camToWorld);
    }
    else
    {
      outWarn("No camera to world transformation, no further processing!");
      throw rs::FrameFilterException();
    }
    worldToCam =
        tf::StampedTransform(camToWorld.inverse(), camToWorld.stamp_, camToWorld.child_frame_id_, camToWorld.frame_id_);
    computeFrustum();

    // default place to look for objects is counter tops except if we got queried for some different place
    rs::Query qs = rs::create<rs::Query>(tcas);

    regions_to_look_at_.assign(defaultRegions.begin(), defaultRegions.end());

    if (cas.getFS("QUERY", qs) && qs.query() != "")
    {
      rapidjson::Document jsonDoc;
      std::string jsonString = qs.query();
      jsonDoc.Parse(jsonString);
      outWarn("query in CAS : " << jsonString);

      rapidjson::Pointer framePointerIn("/detect/location");
      rapidjson::Value* frameJson = framePointerIn.Get(jsonDoc);
      std::string newLocation;
      if (frameJson && frameJson->IsString())
        newLocation = frameJson->GetString();

      outWarn("location set: " << newLocation);
      if (std::find(defaultRegions.begin(), defaultRegions.end(), newLocation) == std::end(defaultRegions) &&
          newLocation != "")
      {
        outInfo("Location defined in query is not set in the default regions. Setting as active region to look at");
        regions_to_look_at_.clear();
        regions_to_look_at_.push_back(newLocation);
        if (jsonString.find("scan"))
        {
          outInfo("Scanning action defined: filter location set permanently");
          defaultRegions.clear();
          defaultRegions.push_back(newLocation);
        }
      }
    }

    for (size_t i = 0; i < semantic_map_items_.size(); ++i)
    {
      if (frustumCulling(semantic_map_items_[i]) || !frustumCulling_)
      {
        if (std::find(regions_to_look_at_.begin(), regions_to_look_at_.end(), semantic_map_items_[i].name) !=
            regions_to_look_at_.end())
        {
          outDebug("region inside frustum: " << semantic_map_items_[i].name);
          filterRegion(semantic_map_items_[i]);

          if (semantic_map_items_[i].hasPlaneEq)
          {
            Eigen::Matrix4d Trans;  // Your Transformation Matrix
            Trans.setIdentity();    // Set to Identity to make bottom row of Matrix 0,0,0,1
            Trans(0, 0) = worldToCam.getBasis()[0][0];
            Trans(0, 1) = worldToCam.getBasis()[0][1];
            Trans(0, 2) = worldToCam.getBasis()[0][2];

            Trans(1, 0) = worldToCam.getBasis()[1][0];
            Trans(1, 1) = worldToCam.getBasis()[1][1];
            Trans(1, 2) = worldToCam.getBasis()[1][2];

            Trans(2, 0) = worldToCam.getBasis()[2][0];
            Trans(2, 1) = worldToCam.getBasis()[2][1];
            Trans(2, 2) = worldToCam.getBasis()[2][2];

            Trans(0, 3) = worldToCam.getOrigin()[0];
            Trans(1, 3) = worldToCam.getOrigin()[1];
            Trans(2, 3) = worldToCam.getOrigin()[2];

            Eigen::Vector4d plane_eq(semantic_map_items_[i].plane_eq[0], semantic_map_items_[i].plane_eq[1],
                                     semantic_map_items_[i].plane_eq[2], -semantic_map_items_[i].plane_eq[3]);

            Eigen::Vector4d new_plane_eq = Trans.inverse().transpose() * plane_eq;

            rs::Plane supp_plane = rs::create<rs::Plane>(tcas);
            std::vector<float> plane_model_as_std_vect(4);
            plane_model_as_std_vect[0] = new_plane_eq[0];
            plane_model_as_std_vect[1] = new_plane_eq[1];
            plane_model_as_std_vect[2] = new_plane_eq[2];
            plane_model_as_std_vect[3] = new_plane_eq[3];
            supp_plane.model(plane_model_as_std_vect);
            supp_plane.source("RegionFilter");
            scene.annotations.append(supp_plane);
          }
        }
      }
    }

    pcl::ExtractIndices<PointT> ei;
    ei.setKeepOrganized(true);
    ei.setIndices(indices);
    ei.filterDirectly(cloud);

    cas.set(VIEW_CLOUD, *cloud);

    if (changeDetection && !indices->empty())
    {
      ++frames;
      if (lastImg.empty())
      {
        lastMask = cv::Mat::ones(color.rows, color.cols, CV_8U);
        lastImg = cv::Mat::zeros(color.rows, color.cols, CV_32FC4);
      }

      uint32_t secondsPassed = camToWorld.stamp_.sec - lastTime.sec;
      bool change = checkChange() || cas.has("QUERY") || secondsPassed > timeout;

      if (!change)
        ++filtered;
      else
        lastTime = camToWorld.stamp_;

      outDebug("filtered frames: " << filtered << " / " << frames << "(" << (filtered / (float)frames) * 100 << "%)");

      if (!change)
      {
        outInfo("No changes in frame detected, no further processing!");
        throw rs::FrameFilterException();
      }
    }

    return UIMA_ERR_NONE;
  }

  void computeFrustum()
  {
    // compute frustum based on camera info and tf
    const double alphaY = cameraInfo.K[4];
    const double fovY = 2 * atan(cameraInfo.height / (2 * alphaY));
    tf::Vector3 up(0, 1, 0), focal(1, 0, 0);
    up = camToWorld * up;
    focal = camToWorld * focal;

    camera.fovy = fovY;
    camera.clip[0] = 0.001;
    camera.clip[1] = 12.0;
    camera.window_size[0] = cameraInfo.width;
    camera.window_size[1] = cameraInfo.height;
    camera.window_pos[0] = 0;
    camera.window_pos[1] = 0;
    camera.pos[0] = worldToCam.getOrigin().x();
    camera.pos[1] = worldToCam.getOrigin().y();
    camera.pos[2] = worldToCam.getOrigin().z();
    camera.view[0] = up.x();
    camera.view[1] = up.y();
    camera.view[2] = up.z();
    camera.focal[0] = focal.x();
    camera.focal[1] = focal.y();
    camera.focal[2] = focal.z();

    Eigen::Matrix4d viewMatrix;
    Eigen::Matrix4d projectionMatrix;
    Eigen::Matrix4d viewProjectionMatrix;
    camera.computeViewMatrix(viewMatrix);
    camera.computeProjectionMatrix(projectionMatrix);
    viewProjectionMatrix = projectionMatrix * viewMatrix;
    pcl::visualization::getViewFrustum(viewProjectionMatrix, frustum);
  }

  bool frustumCulling(const SemanticMapItem& region)
  {
    double tFrustum[24];

    // transform frustum planes to region
    for (size_t i = 0; i < 6; ++i)
    {
      tf::Vector3 normal(frustum[i * 4 + 0], frustum[i * 4 + 1], frustum[i * 4 + 2]);
      double d = frustum[i * 4 + 3];
      tf::Vector3 point = normal * d;

      normal = region.transform * normal;
      point = region.transform * point;
      d = normal.dot(point);

      tFrustum[i * 4 + 0] = normal.x();
      tFrustum[i * 4 + 1] = normal.y();
      tFrustum[i * 4 + 2] = normal.z();
      tFrustum[i * 4 + 3] = d;
    }

    // check region bounding box
    Eigen::Vector3d bbMin(-(region.y_dimention / 2), -(region.z_dimension / 2), -(region.x_dimention / 2));
    Eigen::Vector3d bbMax((region.y_dimention / 2), (region.z_dimension / 2), 0.5);
    pcl::visualization::FrustumCull res =
        (pcl::visualization::FrustumCull)pcl::visualization::cullFrustum(tFrustum, bbMin, bbMax);
    return res != pcl::visualization::PCL_OUTSIDE_FRUSTUM;
  }

  cv::Vec4f invariantColor(const cv::Vec3b& color, const uint16_t depth) const
  {
    cv::Vec4f out;
    const float sum = color.val[0] + color.val[1] + color.val[2];
    out.val[0] = color.val[0] / sum;
    out.val[1] = color.val[1] / sum;
    out.val[2] = color.val[2] / sum;
    out.val[3] = depth / 1000.0;
    return out;
  }

  bool checkChange(const cv::Vec4f& v1, const int index) const
  {
    if (lastMask.at<uint8_t>(index))
    {
      const cv::Vec4f& v2 = lastImg.at<cv::Vec4f>(index);
      return std::abs(v1.val[3] - v2.val[3]) > depthThreshold ||
             (std::abs(v1.val[0] - v2.val[0]) + std::abs(v1.val[1] - v2.val[1]) + std::abs(v1.val[2] - v2.val[2])) >
                 pixelThreshold;
    }
    return true;
  }

  bool checkChange()
  {
    cv::Mat mask = cv::Mat::zeros(color.rows, color.cols, CV_8U);
    cv::Mat img = cv::Mat::zeros(color.rows, color.cols, CV_32FC4);
    size_t changedPixels = 0;
    size_t size = indices->size();
    changes.clear();

    // Check pixel changes
    for (size_t i = 0; i < indices->size(); ++i)
    {
      const int idx = indices->at(i);

      cv::Vec4f v = invariantColor(color.at<cv::Vec3b>(idx), depth.at<uint16_t>(idx));
      img.at<cv::Vec4f>(idx) = v;
      mask.at<uint8_t>(idx) = 1;

      if (checkChange(v, idx))
      {
        changes.push_back(idx);
        ++changedPixels;
      }
    }
    // Check pixels that are masked out in current but not in last frame
    for (size_t i = 0; i < lastIndices.size(); ++i)
    {
      const int idx = lastIndices.at(i);
      if (!mask.at<uint8_t>(idx))
      {
        changes.push_back(idx);
        ++changedPixels;
        ++size;
      }
    }

    lastImg = img;
    lastMask = mask;
    indices->swap(lastIndices);

    const float diff = changedPixels / (float)size;
    outDebug(changedPixels << " from " << size << " pixels changed (" << diff * 100 << "%)");

    return diff > threshold;
  }

  void filterRegion(const SemanticMapItem& region)
  {
    const float minX = -(region.x_dimention / 2) + border;
    const float maxX = (region.x_dimention/ 2) - border;
    float minY = -(region.y_dimention / 2) + border;
    const float maxY = (region.y_dimention / 2) - border;
    const float minZ = -(region.z_dimension / 2);
    float maxZ = +(region.z_dimension / 2);

    if (region.name == "drawer_sinkblock_upper_open")
    {
      maxZ = 0.08;
    }
    // needed because of crappy sem map

    if (region.type == "CounterTop")
    {
      maxZ = 0.4;
    }
    if (region.name == "kitchen_sink_block_counter_top")
    {
      minY += 1;  // don't get points for the sink
    }
    else if (region.name == "kitchen_island_counter_top")
    {
      minY += 0.6;  // same for the hot plate
    }

    tf::Transform transform;
    outDebug(region.name << " is defined in " << region.reference_frame);
    if (region.reference_frame != "map")
    {
      outDebug("Looking up transfrom from: " << cameraInfo.header.frame_id << "to " << region.reference_frame);
      listener_.listener->waitForTransform(cameraInfo.header.frame_id, region.reference_frame, ros::Time(0),
                                           ros::Duration(2.0));
      listener_.listener->lookupTransform(cameraInfo.header.frame_id, region.reference_frame, ros::Time(0), camToWorld);
    }
    transform = region.transform.inverse() * camToWorld;

    Eigen::Affine3d eigenTransform;
    tf::transformTFToEigen(transform, eigenTransform);

    pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());

    pcl::transformPointCloud<PointT>(*cloud, *transformed, eigenTransform);

    for (size_t i = 0; i < transformed->points.size(); ++i)
    {
      const PointT& p = transformed->points[i];
      if (p.x > minX && p.x < maxX && p.y > minY && p.y < maxY && p.z > minZ && p.z < maxZ)
      {
        indices->push_back(i);
      }
    }
  }

  void drawImageWithLock(cv::Mat& disp)
  {
    disp = cv::Mat::zeros(cloud->height, cloud->width, CV_8UC3);
    const cv::Vec3b white(255, 255, 255);
    const cv::Vec3b red(0, 0, 255);

#pragma omp parallel for
    for (size_t i = 0; i < lastIndices.size(); ++i)
    {
      const int& index = lastIndices.at(i);
      disp.at<cv::Vec3b>(index) = white;
    }
#pragma omp parallel for
    for (size_t i = 0; i < changes.size(); ++i)
    {
      const int& index = changes.at(i);
      disp.at<cv::Vec3b>(index) = red;
    }
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string& cloudname = this->name;

    if (firstRun)
    {
      visualizer.addPointCloud(cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.removeAllShapes();
    }

    visualizer.addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0.2, 0, 0), 1, 0, 0, "X");
    visualizer.addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0.2, 0), 0, 1, 0, "Y");
    visualizer.addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 0.2), 0, 0, 1, "Z");

    tf::Vector3 origin = worldToCam * tf::Vector3(0, 0, 0);
    tf::Vector3 lineX = worldToCam * tf::Vector3(0.2, 0, 0);
    tf::Vector3 lineY = worldToCam * tf::Vector3(0, 0.2, 0);
    tf::Vector3 lineZ = worldToCam * tf::Vector3(0, 0, 0.2);

    pcl::PointXYZ pclOrigin(origin.x(), origin.y(), origin.z());
    pcl::PointXYZ pclLineX(lineX.x(), lineX.y(), lineX.z());
    pcl::PointXYZ pclLineY(lineY.x(), lineY.y(), lineY.z());
    pcl::PointXYZ pclLineZ(lineZ.x(), lineZ.y(), lineZ.z());

    visualizer.addLine(pcl::PointXYZ(0, 0, 0), pclOrigin, 1, 1, 1, "line");
    visualizer.addLine(pclOrigin, pclLineX, 1, 0, 0, "lineX");
    visualizer.addLine(pclOrigin, pclLineY, 0, 1, 0, "lineY");
    visualizer.addLine(pclOrigin, pclLineZ, 0, 0, 1, "lineZ");

    for (int i = 0; i < semantic_map_items_.size(); ++i)
    {
      const SemanticMapItem& region = semantic_map_items_[i];
      if (std::find(regions_to_look_at_.begin(), regions_to_look_at_.end(), region.name) == regions_to_look_at_.end())
        continue;
      tf::Transform transform = worldToCam * region.transform;

      std::ostringstream oss;
      oss << "region_" << i;

      tf::Vector3 originB = transform * tf::Vector3(0, 0, 0);
      tf::Vector3 lineXB = transform * tf::Vector3(0.2, 0, 0);
      tf::Vector3 lineYB = transform * tf::Vector3(0, 0.2, 0);
      tf::Vector3 lineZB = transform * tf::Vector3(0, 0, 0.2);

      pcl::PointXYZ pclOriginB(originB.x(), originB.y(), originB.z());
      pcl::PointXYZ pclLineXB(lineXB.x(), lineXB.y(), lineXB.z());
      pcl::PointXYZ pclLineYB(lineYB.x(), lineYB.y(), lineYB.z());
      pcl::PointXYZ pclLineZB(lineZB.x(), lineZB.y(), lineZB.z());

      visualizer.addLine(pclOrigin, pclOriginB, 1, 1, 1, "line_" + oss.str());
      visualizer.addLine(pclOriginB, pclLineXB, 1, 0, 0, "lineX_" + oss.str());
      visualizer.addLine(pclOriginB, pclLineYB, 0, 1, 0, "lineY_" + oss.str());
      visualizer.addLine(pclOriginB, pclLineZB, 0, 0, 1, "lineZ_" + oss.str());

      Eigen::Vector3d translation;
      Eigen::Quaterniond rotation;

      tf::vectorTFToEigen(transform.getOrigin(), translation);
      tf::quaternionTFToEigen(transform.getRotation(), rotation);

      visualizer.addCube(translation.cast<float>(), rotation.cast<float>(), region.x_dimention, region.y_dimention, region.z_dimension,
                         oss.str());
      visualizer.setRepresentationToWireframeForAllActors();
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(RegionFilter)
