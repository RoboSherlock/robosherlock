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
#include <rs/DrawingAnnotator.h>
#include <rs/utils/exception.h>

#include <rapidjson/document.h>
#include <rapidjson/pointer.h>

#include <rs/io/utils.h>

using namespace uima;

class RegionFilter : public DrawingAnnotator
{
  struct Region
  {
    tf::Transform transform;
    float width, depth, height;
    std::string name;
    std::string type;
  };

  struct SemanticMapItem
  {
    std::string name, type;
    double width, height, depth;
    tf::Transform transform;
  };
  std::vector<SemanticMapItem> semanticMapItems_;


  typedef pcl::PointXYZRGBA PointT;

  double pointSize;
  float border;

  cv::Mat color, depth;
  pcl::PointCloud<PointT>::Ptr cloud;
  pcl::IndicesPtr indices;

  tf::StampedTransform camToWorld, worldToCam;
  std::vector<Region> regions;

  std::vector<std::string> defaultRegions;
  //name mapping for queries
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
  double frustum[24];

public:

  RegionFilter() : DrawingAnnotator(__func__), pointSize(1), border(0.05), cloud(new pcl::PointCloud<PointT>()),
    indices(new std::vector<int>()),
    changeDetection(true), frustumCulling_(false), threshold(0.1), pixelThreshold(0.1), depthThreshold(0.01), frames(0), filtered(0), lastTime(ros::Time::now()), timeout(120)
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    if(ctx.isParameterDefined("border"))
      ctx.extractValue("border", border);

    std::vector<std::string *> temp;
    if(ctx.isParameterDefined("defaultRegions"))
    {
      ctx.extractValue("defaultRegions", temp);
      for(auto s : temp)
      {
        outInfo(*s);
        defaultRegions.push_back(*s);
      }
    }

    if(ctx.isParameterDefined("enable_change_detection"))
      ctx.extractValue("enable_change_detection", changeDetection);
    if(ctx.isParameterDefined("enable_frustum_culling"))
      ctx.extractValue("enable_frustum_culling", frustumCulling_);
    if(ctx.isParameterDefined("pixel_threshold"))
      ctx.extractValue("pixel_threshold", pixelThreshold);
    if(ctx.isParameterDefined("depth_threshold"))
      ctx.extractValue("depth_threshold", depthThreshold);   
    if(ctx.isParameterDefined("global_threshold"))
      ctx.extractValue("global_threshold", threshold);
    if(ctx.isParameterDefined("change_timeout"))
    {
      int tmp = 120;
      ctx.extractValue("change_timeout", tmp);
      timeout = tmp;
    }
    if(ctx.isParameterDefined("semantic_map_definition"))
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

  void readSemanticMap(const std::string &file)
  {
    const std::string &mapFile = getConfigFilePath(file);
    if(mapFile.empty())
    {
      throw_exception_message("Semantic map file not found: " + file);
    }

    outInfo("Path to semantic map file: " FG_BLUE << mapFile);
    cv::FileStorage fs(mapFile, cv::FileStorage::READ);
    std::vector<std::string> names;

    cv::FileNode n = fs["names"];
    for(cv::FileNodeIterator it = n.begin(); it != n.end(); ++it)
    {
      names.push_back((std::string)(*it));
    }

    semanticMapItems_.resize(names.size());
    for(size_t i = 0; i < names.size(); ++i)
    {
      SemanticMapItem &item = semanticMapItems_[i];
      cv::FileNode entry = fs[names[i]];

      item.name = names[i];
      entry["type"] >> item.type;
      entry["width"] >> item.height;
      entry["height"] >> item.depth;
      entry["depth"] >> item.width;

      cv::Mat m;
      entry["transform"] >> m;

      tf::Matrix3x3 rot;
      tf::Vector3 trans;
      rot.setValue(m.at<double>(0, 0), m.at<double>(0, 1), m.at<double>(0, 2), m.at<double>(1, 0), m.at<double>(1, 1), m.at<double>(1, 2), m.at<double>(2, 0), m.at<double>(2, 1), m.at<double>(2, 2));
      trans.setValue(m.at<double>(0, 3), m.at<double>(1, 3), m.at<double>(2, 3));
      item.transform = tf::Transform(rot, trans);
    }
  }

private:
  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    std::vector<rs::SemanticMapObject> semanticMap;
    semanticMap.reserve(semanticMapItems_.size());
    for(size_t i = 0; i < semanticMapItems_.size(); ++i)
    {
      SemanticMapItem &item = semanticMapItems_[i];
      rs::SemanticMapObject obj = rs::create<rs::SemanticMapObject>(tcas);
      obj.name(item.name);
      obj.typeName(item.type);
      obj.width(item.width);
      obj.height(item.height);
      obj.depth(item.depth);
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
    if(scene.viewPoint.has())
    {
      rs::conversion::from(scene.viewPoint.get(), camToWorld);
    }
    else
    {
      outWarn("No camera to world transformation, no further processing!");
      throw rs::FrameFilterException();
    }
    worldToCam = tf::StampedTransform(camToWorld.inverse(), camToWorld.stamp_, camToWorld.child_frame_id_, camToWorld.frame_id_);
    computeFrustum();

    //default place to look for objects is counter tops except if we got queried for some different place
    rs::Query qs = rs::create<rs::Query>(tcas);
    std::vector<std::string> regionsToLookAt;
    regionsToLookAt.assign(defaultRegions.begin(), defaultRegions.end());
    regions.clear();

    if(cas.getFS("QUERY", qs) && qs.query() != "")
    {
      rapidjson::Document jsonDoc;
      std::string jsonString  = qs.query();
      jsonDoc.Parse(jsonString);
      outWarn("query in CAS : " << jsonString);

      //TODO first level of json is currently only detect, needs to be done differently when there are
      //multiple modes (Maybe save query mode in FS?)
      rapidjson::Pointer framePointerIn("/detect/location");
      // rapidjson::Pointer framePointerOn("/detect/location/on");

      rapidjson::Value *frameJson = framePointerIn.Get(jsonDoc);
      //      rapidjson::Value *frameJsonOn = framePointerOn.Get(jsonDoc);
      std::string newLocation;

      if(frameJson && frameJson->IsString())
      {
        newLocation = frameJson->GetString();
      }

      //     if(frameJsonOn && frameJsonOn->IsString())
      //     {
      //       newLocation = frameJsonOn->GetString();
      //     }

      outWarn("location set: " << newLocation);
      if(std::find(defaultRegions.begin(), defaultRegions.end(), newLocation) == std::end(defaultRegions) && newLocation != "")
      {
        outInfo("new location not in default Regions");
        regionsToLookAt.clear();
        regionsToLookAt.push_back(newLocation);
        if(jsonString.find("scan"))
        {
          outInfo("Scanning action defined: filter location set permanently");
          defaultRegions.clear();
          defaultRegions.push_back(newLocation);
        }
      }
    }

    if(regions.empty())
    {
      std::vector<rs::SemanticMapObject> semanticRegions;
      getSemanticMapEntries(cas, regionsToLookAt, semanticRegions);

      regions.resize(semanticRegions.size());
      for(size_t i = 0; i < semanticRegions.size(); ++i)
      {

        Region &region = regions[i];
        region.width = semanticRegions[i].width();
        region.depth = semanticRegions[i].depth();
        region.height = semanticRegions[i].height();
        region.name = semanticRegions[i].name();
        region.type = semanticRegions[i].typeName();
        rs::conversion::from(semanticRegions[i].transform(), region.transform);
      }
    }

    for(size_t i = 0; i < regions.size(); ++i)
    {
      if(frustumCulling(regions[i]) || !frustumCulling_)
      {
        outInfo("region inside frustum: " << regions[i].name);
        filterRegion(regions[i]);
      }
      else
      {
        outInfo("region outside frustum: " << regions[i].name);
      }
    }

    pcl::ExtractIndices<PointT> ei;
    ei.setKeepOrganized(true);
    ei.setIndices(indices);
    ei.filterDirectly(cloud);

    cas.set(VIEW_CLOUD, *cloud);

    if(changeDetection && !indices->empty())
    {
      ++frames;
      if(lastImg.empty())
      {
        lastMask = cv::Mat::ones(color.rows, color.cols, CV_8U);
        lastImg = cv::Mat::zeros(color.rows, color.cols, CV_32FC4);
      }

      uint32_t secondsPassed = camToWorld.stamp_.sec - lastTime.sec;
      bool change = checkChange() || cas.has("QUERY") || secondsPassed > timeout;

      if(!change)
      {
        ++filtered;
      }
      else
      {
        lastTime = camToWorld.stamp_;
      }
      outInfo("filtered frames: " << filtered << " / " << frames << "(" << (filtered / (float)frames) * 100 << "%)");

      if(!change)
      {
        outWarn("no changes in frame detected, no further processing!");
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

  bool frustumCulling(const Region &region)
  {
    double tFrustum[24];

    // transform frustum planes to region
    for(size_t i = 0; i < 6; ++i)
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
    Eigen::Vector3d bbMin(-(region.width / 2), -(region.height / 2), -(region.depth / 2));
    Eigen::Vector3d bbMax((region.width / 2), (region.height / 2), 0.5);
    pcl::visualization::FrustumCull res = (pcl::visualization::FrustumCull)pcl::visualization::cullFrustum(tFrustum, bbMin, bbMax);
    return res != pcl::visualization::PCL_OUTSIDE_FRUSTUM;
  }

  cv::Vec4f invariantColor(const cv::Vec3b &color, const uint16_t depth) const
  {
    cv::Vec4f out;
    const float sum = color.val[0] + color.val[1] + color.val[2];
    out.val[0] = color.val[0] / sum;
    out.val[1] = color.val[1] / sum;
    out.val[2] = color.val[2] / sum;
    out.val[3] = depth / 1000.0;
    return out;
  }

  bool checkChange(const cv::Vec4f &v1, const int index) const
  {
    if(lastMask.at<uint8_t>(index))
    {
      const cv::Vec4f &v2 = lastImg.at<cv::Vec4f>(index);
      return std::abs(v1.val[3] - v2.val[3]) > depthThreshold || (std::abs(v1.val[0] - v2.val[0]) + std::abs(v1.val[1] - v2.val[1]) + std::abs(v1.val[2] - v2.val[2])) > pixelThreshold;
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
    for(size_t i = 0; i < indices->size(); ++i)
    {
      const int idx = indices->at(i);

      cv::Vec4f v = invariantColor(color.at<cv::Vec3b>(idx), depth.at<uint16_t>(idx));
      img.at<cv::Vec4f>(idx) = v;
      mask.at<uint8_t>(idx) = 1;

      if(checkChange(v, idx))
      {
        changes.push_back(idx);
        ++changedPixels;
      }
    }
    // Check pixels that are masked out in current but not in last frame
    for(size_t i = 0; i < lastIndices.size(); ++i)
    {
      const int idx = lastIndices.at(i);
      if(!mask.at<uint8_t>(idx))
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
    outInfo(changedPixels << " from " << size << " pixels changed (" << diff * 100 << "%)");

    return diff > threshold;
  }

  void getSemanticMapEntries(rs::SceneCas &cas, const std::vector<std::string> &name, std::vector<rs::SemanticMapObject> &mapObjects)
  {
    std::vector<rs::SemanticMapObject> objects;
    cas.get(VIEW_SEMANTIC_MAP, objects);
    for(auto n : name)
    {
      for(size_t i = 0; i < objects.size(); ++i)
      {
        if(objects[i].name() == n)
        {
          mapObjects.push_back(objects[i]);
        }
      }
    }
  }

  void filterRegion(const Region &region)
  {
    const float minX = -(region.width / 2) + border;
    const float maxX = (region.width / 2) - border;
    float minY = -(region.height / 2) + border;
    const float maxY = (region.height / 2) - border;
    const float minZ = -(region.depth / 2);
    float maxZ = +(region.depth / 2);

    if(region.name == "drawer_sinkblock_upper_open")
    {
      maxZ = 0.08;
    }
    //needed because of crappy sem map

    if(region.type == "CounterTop")
    {
      maxZ = 0.4;
    }
    if(region.name == "kitchen_sink_block_counter_top")
    {
      minY += 1;//don't get points for the sink
    }
    else if(region.name == "kitchen_island_counter_top")
    {
      minY += 0.6; //same for the hot plate
    }

    tf::Transform transform;
    transform = region.transform.inverse() * camToWorld;

    Eigen::Affine3d eigenTransform;
    tf::transformTFToEigen(transform, eigenTransform);

    pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());

    pcl::transformPointCloud<PointT>(*cloud, *transformed, eigenTransform);

    for(size_t i = 0; i < transformed->points.size(); ++i)
    {
      const PointT &p = transformed->points[i];
      if(p.x > minX && p.x < maxX && p.y > minY && p.y < maxY && p.z > minZ && p.z < maxZ)
      {
        indices->push_back(i);
      }
    }
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = cv::Mat::zeros(cloud->height, cloud->width, CV_8UC3);
    const cv::Vec3b white(255, 255, 255);
    const cv::Vec3b red(0, 0, 255);

    #pragma omp parallel for
    for(size_t i = 0; i < lastIndices.size(); ++i)
    {
      const int &index = lastIndices.at(i);
      disp.at<cv::Vec3b>(index) = white;
    }
    #pragma omp parallel for
    for(size_t i = 0; i < changes.size(); ++i)
    {
      const int &index = changes.at(i);
      disp.at<cv::Vec3b>(index) = red;
    }
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = this->name;

    if(firstRun)
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

    for(int i = 0; i < regions.size(); ++i)
    {
      const Region &region = regions[i];
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

      visualizer.addCube(translation.cast<float>(), rotation.cast<float>(),
                         region.width, region.height, region.depth, oss.str());
      visualizer.setRepresentationToWireframeForAllActors();
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(RegionFilter)
