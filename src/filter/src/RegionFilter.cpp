/*
 * Copyright (c) 2012, Nico Blodow <blodow@cs.tum.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

using namespace uima;

class RegionFilter : public DrawingAnnotator
{
  struct Region
  {
    tf::Transform transform;
    float width, depth, height;
    std::string name;
  };

  typedef pcl::PointXYZRGBA PointT;

  double pointSize;
  float border;

  cv::Mat color;
  pcl::PointCloud<PointT>::Ptr cloud;
  pcl::IndicesPtr indices;

  tf::StampedTransform camToWorld, worldToCam;
  std::vector<Region> regions;

  std::string regionToLookAt;
  //name mapping for queries
  std::map<std::string, std::string> nameMapping;

  // for change detection
  bool changeDetection;
  cv::Mat lastImg, lastMask;
  float threshold, pixelThreshold;
  std::vector<int> changes;
  size_t frames, filtered;
  ros::Time lastTime;
  uint32_t timeout;

  // for frustum culling
  sensor_msgs::CameraInfo cameraInfo;
  pcl::visualization::Camera camera;
  double frustum[24];

public:

  RegionFilter() : DrawingAnnotator(__func__), pointSize(1), border(0.05), cloud(new pcl::PointCloud<PointT>()),
    indices(new std::vector<int>()), regionToLookAt("CounterTop"),
    changeDetection(true), threshold(0.1), pixelThreshold(0.1), frames(0), filtered(0), lastTime(ros::Time::now()), timeout(120)
  {
    nameMapping["drawer"] = "Drawer";
    nameMapping["countertop"] = "CounterTop";
    nameMapping["table"] = "Table";
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    if(ctx.isParameterDefined("border"))
    {
      ctx.extractValue("border", border);
    }

    if(ctx.isParameterDefined("region_to_filter"))
    {
      ctx.extractValue("region_to_filter", regionToLookAt);
      nameMapping[""] = regionToLookAt;
    }

    if(ctx.isParameterDefined("enable_change_detection"))
    {
      ctx.extractValue("enable_change_detection", changeDetection);
    }
    if(ctx.isParameterDefined("pixel_threshold"))
    {
      ctx.extractValue("pixel_threshold", pixelThreshold);
    }
    if(ctx.isParameterDefined("global_threshold"))
    {
      ctx.extractValue("global_threshold", threshold);
    }
    if(ctx.isParameterDefined("change_timeout"))
    {
      int tmp = 120;
      ctx.extractValue("change_timeout", tmp);
      timeout = tmp;
    }
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

private:
  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    cas.get(VIEW_CLOUD, *cloud);
#if USE_HD
    cas.get(VIEW_COLOR_IMAGE_HD, color);
    cas.get(VIEW_CAMERA_INFO_HD, cameraInfo);
#else
    cas.get(VIEW_COLOR_IMAGE, color);
    cas.get(VIEW_CAMERA_INFO, cameraInfo);
#endif

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
    //message comes from desigantor and is not the same as the entries from the semantic map so we need
    //to transform them
    rs::Query qs = rs::create<rs::Query>(tcas);
    if(cas.getFS("QUERY", qs))
    {
      if(regionToLookAt != nameMapping[qs.location()])
      {
        regions.clear();
        outWarn("loaction set in query: " << qs.location());
        regionToLookAt = nameMapping[qs.location()];
      }
    }

    if(regions.empty())
    {
      std::vector<rs::SemanticMapObject> semanticRegions;
      outWarn("Region before filtering: " << regionToLookAt);
      getSemanticMapEntries(cas, regionToLookAt, semanticRegions);

      regions.resize(semanticRegions.size());
      for(size_t i = 0; i < semanticRegions.size(); ++i)
      {
        std::size_t found = semanticRegions[i].name().find("drawer_sinkblock_upper");
        if(regionToLookAt == "Drawer" && found == std::string::npos)
        {
          continue;
        }
        Region &region = regions[i];

        region.width = semanticRegions[i].width();
        region.depth = semanticRegions[i].depth();
        region.height = semanticRegions[i].height();
        region.name = semanticRegions[i].name();
        rs::conversion::from(semanticRegions[i].transform(), region.transform);
      }
    }

    for(size_t i = 0; i < regions.size(); ++i)
    {
      if(frustumCulling(regions[i]))
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
        lastImg = cv::Mat::zeros(color.rows, color.cols, CV_32FC3);
      }

      cv::Mat img, mask;
      uint32_t secondsPassed = camToWorld.stamp_.sec - lastTime.sec;
      bool change = checkChange(img, mask) || cas.has("QUERY") || secondsPassed > timeout;
      lastImg = img;
      lastMask = mask;

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

  cv::Vec3f invariantColor(const cv::Vec3b &color) const
  {
    cv::Vec3f out;
    const float sum = color.val[0] + color.val[1] + color.val[2];
    out.val[0] = color.val[0] / sum;
    out.val[1] = color.val[1] / sum;
    out.val[2] = color.val[2] / sum;
    return out;
  }

  bool checkChange(const cv::Vec3f &v1, const int index) const
  {
    if(lastMask.at<uint8_t>(index))
    {
      const cv::Vec3f &v2 = lastImg.at<cv::Vec3f>(index);
      return (std::abs(v1.val[0] - v2.val[0]) + std::abs(v1.val[1] - v2.val[1]) + std::abs(v1.val[2] - v2.val[2])) > pixelThreshold;
    }
    return false;
  }

  bool checkChange(cv::Mat &img, cv::Mat &mask)
  {
    size_t changedPixels = 0;
    mask = cv::Mat::zeros(color.rows, color.cols, CV_8U);
    img = cv::Mat::zeros(color.rows, color.cols, CV_32FC3);
    changes.clear();

    for(size_t i = 0; i < indices->size(); ++i)
    {
#if USE_HD
      const int i0 = indices->at(i) * 2;
      const int i1 = i0 + 1;
      const int i2 = i0 + color.cols;
      const int i3 = i1 + color.cols;
#else
      const int i0 = indices->at(i);
#endif

      mask.at<uint8_t>(i0) = 1;
#if USE_HD
      mask.at<uint8_t>(i1) = 1;
      mask.at<uint8_t>(i2) = 1;
      mask.at<uint8_t>(i3) = 1;
#endif

      cv::Vec3f v0 = invariantColor(color.at<cv::Vec3b>(i0));
      img.at<cv::Vec3f>(i0) = v0;
#if USE_HD
      cv::Vec3f v1 = invariantColor(color.at<cv::Vec3b>(i1));
      img.at<cv::Vec3f>(i1) = v1;
      cv::Vec3f v2 = invariantColor(color.at<cv::Vec3b>(i2));
      img.at<cv::Vec3f>(i2) = v2;
      cv::Vec3f v3 = invariantColor(color.at<cv::Vec3b>(i3));
      img.at<cv::Vec3f>(i3) = v3;
#endif

      if(checkChange(v0, i0))
      {
        changes.push_back(i0);
        ++changedPixels;
      }
#if USE_HD
      if(checkChange(v1, i1))
      {
        ++changedPixels;
      }
      if(checkChange(v2, i2))
      {
        ++changedPixels;
      }
      if(checkChange(v3, i3))
      {
        ++changedPixels;
      }
#endif
    }

#if USE_HD
    const size_t size = indices->size() * 4;
#else
    const size_t size = indices->size();
#endif
    const float diff = changedPixels / (float)size;
    outInfo(changedPixels << " from " << size << " pixels changed (" << diff * 100 << "%)");
    return diff > threshold;
  }

  void getSemanticMapEntries(rs::SceneCas &cas, const std::string &name, std::vector<rs::SemanticMapObject> &mapObjects)
  {
    std::vector<rs::SemanticMapObject> objects;
    cas.get(VIEW_SEMANTIC_MAP, objects);

    for(size_t i = 0; i < objects.size(); ++i)
    {
      if(objects[i].typeName() == name)
      {
        mapObjects.push_back(objects[i]);
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
    const float maxZ = 0.5;

    //needed because of crappy sem map
    if(region.name == "kitchen_sink_block_counter_top")
    {
      minY += 1;//don't get point for the sink
    }
    else if(region.name == "kitchen_island_counter_top")
    {
      minY += 0.8; //same for the hot plate
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
    for(size_t i = 0; i < indices->size(); ++i)
    {
      const int &index = indices->at(i);
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

      visualizer.addCube(translation.cast<float>(), rotation.cast<float>(), region.width, region.height, region.depth, oss.str());
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(RegionFilter)
