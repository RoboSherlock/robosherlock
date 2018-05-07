/* Copyright (c) 2012, Ferenc Balint-Benczdei<balintbe@cs.uni-bremen.de>
 * Based on the handle extractor from Ross Kidson <ross.kidson@gmail.com>
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

#include <vector>

#include <uima/api.hpp>
#include <uima/fsfilterbuilder.hpp>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>

#include <rs/scene_cas.h>
#include <rs_queryanswering/annotators/HandleExtractor.h>
#include <rs/utils/time.h>

#include <rs/DrawingAnnotator.h>


using namespace uima;

class HandleAnnotator : public DrawingAnnotator
{
  struct Region
  {
    tf::Transform transform;
    float width, depth, height;
    std::string name;
  };

public:
  tf::StampedTransform camToWorld, worldToCam;
  std::vector<Region> regions;


  Type cloud_type;
  rs::HandleExtractor *handle_extractor;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr;
  //drawing
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr display;
  std::vector<pcl::PointIndices> handle_indices;
  pcl::IndicesPtr indices;

  double pointSize;
  float border;

  HandleAnnotator(): DrawingAnnotator(__func__), display(new pcl::PointCloud<pcl::PointXYZRGBA>()),
    indices(new std::vector<int>()), pointSize(1), border(0.00)
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    handle_extractor = new rs::HandleExtractor();
    return UIMA_ERR_NONE;
  }

  TyErrorId typeSystemInit(TypeSystem const &type_system)
  {
    outInfo("typeSystemInit");

    cloud_type = type_system.getType(RS_PCL_POINTCLOUD);

    if(!cloud_type.isValid())
    {
      getAnnotatorContext().getLogger().logError("Error getting Type object for " + std::string(RS_PCL_POINTCLOUD));
      outInfo("typeSystemInit - Error. See logfile");
      return UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");

    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {

    outInfo("process begins");
    rs::StopWatch clock;
    double t = clock.getTime();
    indices->clear();

    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    camToWorld.setIdentity();
    if(scene.viewPoint.has())
    {
      rs::conversion::from(scene.viewPoint.get(), camToWorld);
    }
    else
    {
      outInfo("No camera to world transformation!!!");
    }
    worldToCam = tf::StampedTransform(camToWorld.inverse(), camToWorld.stamp_, camToWorld.child_frame_id_, camToWorld.frame_id_);

    std::vector<rs::SemanticMapObject> drawers, doors;
    getSemanticMapEntries(cas, "Drawer", drawers);
    getSemanticMapEntries(cas, "Refrigerator", doors);


    std::vector<rs::SemanticMapObject> regionsOfInterest;

    for(size_t i = 0; i < drawers.size(); ++i)
    {
      std::string name = drawers[i].name();
      outInfo(name);
      std::size_t found = name.find("sinkblock");
      if(found != std::string::npos)
      {
        regionsOfInterest.push_back(drawers[i]);
      }
    }
    for(size_t i = 0; i < doors.size(); ++i)
    {
      std::string name = doors[i].name();
      outInfo(name);
      std::size_t found_fridge = name.find("fridge");
      if(found_fridge != std::string::npos)
      {
        regionsOfInterest.push_back(doors[i]);
      }
    }
    regions.resize(regionsOfInterest.size());

    for(size_t i = 0; i < regionsOfInterest.size(); ++i)
    {
      Region &region = regions[i];
      region.width = regionsOfInterest[i].width();
      region.depth = regionsOfInterest[i].depth();
      region.height = regionsOfInterest[i].height();
      region.name = regionsOfInterest[i].name();
      rs::conversion::from(regionsOfInterest[i].transform(), region.transform);
      outInfo("Region Name: " << regionsOfInterest[i].name());
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals_ptr(new pcl::PointCloud<pcl::Normal>);
    cloud_ptr = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_CLOUD, *cloud_ptr);
    cas.get(VIEW_NORMALS, *normals_ptr);
    display = cloud_ptr;
    outInfo("regions size " << regions.size());
    std::vector<bool> addedIndices;
    addedIndices.resize(cloud_ptr->points.size(), false);
    outInfo("PointCloud has " << cloud_ptr->points.size() << " points");
    for(size_t i = 0; i < regions.size(); ++i)
    {
      filterRegion(regions[i], addedIndices);
    }
    outInfo(indices->size());
    pcl::ExtractIndices<pcl::PointXYZRGBA> ei;
    ei.setKeepOrganized(true);
    ei.setIndices(indices);
    ei.filterDirectly(cloud_ptr);

    //replace point cloud with the new one..allows ...why did I not finish this comment?:)))
    //        cas.set(VIEW_CLOUD,*cloud_ptr);

    std::vector<pcl::ModelCoefficients> handle_coefficients;
    handle_indices.clear();
    outInfo("Extracting Handles ");
    handle_extractor->extractHandles(cloud_ptr, normals_ptr, handle_indices,
                                     handle_coefficients);

    outInfo("Found " << handle_indices.size() << " handles");
    outInfo("no. clusters: " << handle_indices.size());

    std::vector<rs::SemanticMapObject> handles;
    getSemanticMapEntries(cas, "Handle", handles);
    for(int i = 0; i < handle_indices.size(); ++i)
    {
      outInfo("clusters no : " << i << " size: " << handle_indices[i].indices.size());
      rs::HandleAnnotation handleAnnot = rs::create<rs::HandleAnnotation>(tcas);
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cloud_ptr, handle_indices[i], centroid);
      tf::Stamped<tf::Pose> pose;
      pose.frame_id_ = camToWorld.child_frame_id_;
      pose.stamp_ = camToWorld.stamp_;
      outInfo("FRAME ID: " << pose.frame_id_);
      pose.setOrigin(tf::Vector3(centroid[0], centroid[1], centroid[2]));
      pose.setRotation(tf::Quaternion(0, 0, 0, 1));
      handleAnnot.pose.set(rs::conversion::to(tcas, pose));


      //find nearest handle in sem map amd attach name

      double min_dist = std::numeric_limits<float>::max();
      int min_dist_index = -1;
      for(int i = 0; i < handles.size(); i++)
      {
        rs::SemanticMapObject &semHandle = handles[i];
        std::size_t found = semHandle.name().find("sinkblock");
        std::size_t foundFridge = semHandle.name().find("fridge");
        if(found != std::string::npos || foundFridge != std::string::npos)
        {
          tf::Pose hWorldPose;
          rs::conversion::from(semHandle.transform(), hWorldPose);
          tf::Pose hCamPose = worldToCam * hWorldPose;
          double dist = sqrt(std::pow(centroid[0] - hCamPose.getOrigin().getX() , 2) +
                             std::pow(centroid[1] - hCamPose.getOrigin().getY() , 2) +
                             std::pow(centroid[2] - hCamPose.getOrigin().getZ() , 2));
          if(dist < min_dist)
          {
            min_dist_index = i;
            min_dist = dist;
          }
        }

      }
      outInfo(handles[min_dist_index].name());
      rs::PointIndices uimaIndices = rs::conversion::to(tcas, handle_indices[i]);
      handleAnnot.indices.set(uimaIndices);
      handleAnnot.name.set(handles[min_dist_index].name());
      scene.annotations.append(handleAnnot);
    }

    //for visualization purposes
    drawers.insert(drawers.end(), handles.begin(), handles.end());

    t = clock.getTime();
    outInfo("took: " << clock.getTime() << " ms.");
    outInfo("process ends");
    return UIMA_ERR_NONE;
  }

  void filterRegion(const Region &region, std::vector<bool> &addedIndices)
  {

    float maxY;
    float minX;

    float maxX = (region.width / 2) - border;
    float minY = -(region.height / 2) + border;
    if(region.name == "drawer_fridge_upper")
    {
      maxY = (region.height / 2) + 0.08;
      minX = -(region.width / 2) - 0.05;
    }
    else
    {
      maxY = (region.height / 2) - border;
      minX = -(region.width / 2) + border;
    }
    float minZ = -(region.depth / 2);
    float maxZ = (region.depth / 2);

    //now this is why we need good localization and a good semantic map
    if(region.name == "drawer_sinkblock_upper")
    {
      minX -= 0.25;
      minY += 0.1;
      maxY -= 0.15;
    }

    tf::Transform transform;
    transform = region.transform.inverse() * camToWorld;

    Eigen::Affine3d eigenTransform;
    tf::transformTFToEigen(transform, eigenTransform);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGBA>());

    pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud_ptr, *transformed, eigenTransform);
    for(size_t i = 0; i < transformed->points.size(); ++i)
    {
      const pcl::PointXYZRGBA &p = transformed->points[i];
      if(p.x > minX && p.x < maxX && p.y > minY && p.y < maxY && p.z > minZ && p.z <= maxZ)
      {
        if(addedIndices[i] == false)
        {
          indices->push_back(i);
          addedIndices[i] = true;
        }
      }
    }
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = this->name;
    uint32_t colors[6] = {0xFFFF0000, 0xFF00FF00, 0xFF0000FF, 0xFFFFFF00, 0xFFFF00FF, 0xFF00FFFF};
    //    const pcl::PointCloud<pcl::PointXYZRGBA>::VectorType &origPoints = this->cloud_ptr->points;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output;

    output = display;
    for(size_t i = 0; i < handle_indices.size(); ++i)
    {
      const pcl::PointIndices &indices = handle_indices[i];
      for(size_t j = 0; j < indices.indices.size(); ++j)
      {
        size_t index = indices.indices[j];
        output->points[index].rgba = colors[i % 6];
      }
    }

    if(firstRun)
    {
      visualizer.addPointCloud(output, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(output, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.removeAllShapes();
    }
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

      //      visualizer.addLine(pclOrigin, pclOriginB, 1, 1, 1, "line_" + oss.str());
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

  void getSemanticMapEntries(rs::SceneCas &cas, const std::string &name, std::vector<rs::SemanticMapObject> &mapObjects)
  {
    std::vector<rs::SemanticMapObject> objects;
    cas.get(VIEW_SEMANTIC_MAP, objects);

    for(size_t i = 0; i < objects.size(); ++i)
    {
      if(objects[i].typeName() == name)
        mapObjects.push_back(objects[i]);
    }
  }

};

MAKE_AE(HandleAnnotator)
