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

#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>

#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/DrawingAnnotator.h>

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

public:

  RegionFilter() : DrawingAnnotator(__func__), pointSize(1), border(0.05), cloud(new pcl::PointCloud<PointT>()),
    indices(new std::vector<int>()), regionToLookAt("CounterTop")
  {
    nameMapping["DRAWER"] = "Drawer";
    nameMapping["COUNTERTOP"] = "CounterTop";
    nameMapping["TABLE"] = "Table";


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
    cas.get(VIEW_COLOR_IMAGE_HD, color);

    indices->clear();
    indices->reserve(cloud->points.size());

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
      outWarn("Region before filtering: " <<regionToLookAt);
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
      filterRegion(regions[i]);
    }

    pcl::ExtractIndices<PointT> ei;
    ei.setKeepOrganized(true);
    ei.setIndices(indices);
    ei.filterDirectly(cloud);

    cas.set(VIEW_CLOUD, *cloud);

    return UIMA_ERR_NONE;
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
    cv::Vec3b white;
    white.val[0] = white.val[1] = white.val[2] = 255;

    #pragma omp parallel for
    for(size_t i = 0; i < indices->size(); ++i)
    {
      const size_t &index = indices->at(i);
      disp.at<cv::Vec3b>(index / disp.cols, index % disp.cols) = white;
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
