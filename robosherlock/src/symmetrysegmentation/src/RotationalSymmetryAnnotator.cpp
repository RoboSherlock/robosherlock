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
#include <vector>

//PCL include
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>
#include <rs/types/all_types.h>

#include <rs/visualization/Primitives.hpp>

#include <rs/symmetrysegmentation/RotationalSymmetryExtractor.h>


using namespace uima;

class RotationalSymmetryAnnotator : public DrawingAnnotator
{
private:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

  RotationalSymmetryExtractor extractor;

  //container for final output to CAS
  std::vector<RotationalSymmetry> symmetries;

  float rotSymAnn_min_fit_angle;
  float rotSymAnn_max_fit_angle;

  float rotSymAnn_min_occlusion_dist;
  float rotSymAnn_max_occlusion_dist;

  float rotSymAnn_max_sym_score;
  float rotSymAnn_max_occlusion_score;
  float rotSymAnn_max_perpendicular_score;
  float rotSymAnn_min_coverage_score;

  float dist_map_resolution;

  float boundaryRadiusSearch;
  float boundaryAngleThreshold;

  float max_angle_diff;
  float max_dist_diff;

  double pointSize;

public:
  RotationalSymmetryAnnotator () : DrawingAnnotator(__func__), pointSize(1.0)
  {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    ctx.extractValue("rotSymAnn_min_fit_angle", rotSymAnn_min_fit_angle);
    ctx.extractValue("rotSymAnn_max_fit_angle", rotSymAnn_max_fit_angle);
    ctx.extractValue("rotSymAnn_min_occlusion_dist", rotSymAnn_min_occlusion_dist);
    ctx.extractValue("rotSymAnn_max_occlusion_dist", rotSymAnn_max_occlusion_dist);

    ctx.extractValue("rotSymAnn_max_sym_score", rotSymAnn_max_sym_score);
    ctx.extractValue("rotSymAnn_max_occlusion_score", rotSymAnn_max_occlusion_score);
    ctx.extractValue("rotSymAnn_max_perpendicular_score", rotSymAnn_max_perpendicular_score);
    ctx.extractValue("rotSymAnn_min_coverage_score", rotSymAnn_min_coverage_score);

    ctx.extractValue("dist_map_resolution", dist_map_resolution);

    ctx.extractValue("boundaryRadiusSearch", boundaryRadiusSearch);
    ctx.extractValue("boundaryAngleThreshold", boundaryAngleThreshold);

    ctx.extractValue("max_angle_diff", max_angle_diff);
    ctx.extractValue("max_dist_diff", max_dist_diff);

    extractor.initialize(rotSymAnn_min_fit_angle,
                         rotSymAnn_max_fit_angle,
                         rotSymAnn_min_occlusion_dist,
                         rotSymAnn_max_occlusion_dist,
                         rotSymAnn_max_sym_score,
                         rotSymAnn_max_occlusion_score,
                         rotSymAnn_max_perpendicular_score,
                         rotSymAnn_min_coverage_score,
                         dist_map_resolution,
                         boundaryRadiusSearch,
                         boundaryAngleThreshold,
                         max_angle_diff,
                         max_dist_diff);

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;

    outInfo("process begins");
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    //get RGB objects cloud
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    cas.get(VIEW_CLOUD_OBJECTS, *cloud);
    if(cloud->size() == 0)
    {
      outInfo("Input Object cloud address is empty! Using cleaned cloud");
      rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
      cas.getFS(VIEW_CLOUD_NON_NAN, rcp);
      rs::conversion::from(rcp.cloud(), *cloud);
      rs::conversion::from(rcp.normals(), *normals);
    }
    else
    {
      //get normal cloud
      cas.get(VIEW_NORMALS_OBJECTS, *normals);
    }
    extractor.setInputClouds(cloud, normals);

    //get segments
    std::vector<pcl::PointIndices> segments;
    cas.get(VIEW_SEGMENT_IDS, segments);
    extractor.setInputSegmentIds(segments);

    //get bounding planes
    std::vector<rs::Plane> planes;
    scene.annotations.filter(planes);
    if(!planes.empty())
    {
      extractor.setInputPlanes(planes);
    }

    //main execution
    if(!extractor.extract())
    {
      outError("Extractor has exceptions! Aborting!");
    }

    extractor.getSymmetries(symmetries);

    //convert RotationalSymmetry to CAS Symmetries and push to CAS
    std::vector<rs::RotationalSymmetry> casSymmetries;
    for(size_t it = 0; it < symmetries.size(); it++)
    {
      rs::RotationalSymmetry currSym = rs::create<rs::RotationalSymmetry>(tcas);
      rs::Point3f currOrigin = rs::create<rs::Point3f>(tcas);
      rs::Point3f currOrientation = rs::create<rs::Point3f>(tcas);

      Eigen::Vector3f eigenOrigin = symmetries[it].getOrigin();
      Eigen::Vector3f eigenOrientation = symmetries[it].getOrientation();

      currOrigin.x.set(eigenOrigin[0]);
      currOrigin.y.set(eigenOrigin[1]);
      currOrigin.z.set(eigenOrigin[2]);

      currOrientation.x.set(eigenOrientation[0]);
      currOrientation.y.set(eigenOrientation[1]);
      currOrientation.z.set(eigenOrientation[2]);

      currSym.origin.set(currOrigin);
      currSym.orientation.set(currOrientation);
      casSymmetries.push_back(currSym);
    }

    cas.set(VIEW_ROTATIONAL_SYMMETRIES, casSymmetries);

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";
    const std::string annotatorName = "RotationalSymmetryAnnotator";

    if(firstRun)
    {
      visualizer.addPointCloud(cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      addSymmetryLines(visualizer, symmetries, 0.4f, 0.8f);
    }
    else
    {
      visualizer.updatePointCloud(cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.removeAllShapes();
      addSymmetryLines(visualizer, symmetries, 0.4f, 0.8f);
      visualizer.addText(annotatorName, 2, 20, 12, 1, 1, 1, annotatorName);
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(RotationalSymmetryAnnotator)
