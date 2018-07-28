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

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>
#include <rs/types/all_types.h>

#include <rs/segmentation/BilateralSymmetryExtractor.h>

#include <rs/visualization/Primitives.hpp>

using namespace uima;

class BilateralSymmetryAnnotator : public DrawingAnnotator
{
private:
  //container for inital symmetries usign PCA solver
  std::vector< BilateralSymmetry > finalSymmetries;
  std::vector< int > finalSupportSizeIds;
  std::vector<pcl::PointIndices> segments;

  BilateralSymmetryExtractor extractor;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

  bool bilSymAnn_isDownsampled;
  bool naive_detection;

  float downsample_voxel_size;

  int angle_division;

  float dist_map_resolution;

  float correspondence_search_radius;
  float correspondence_max_normal_fit_error;
  float correspondence_min_sym_dist;
  float correspondence_max_sym_reflected_dist;

  int refine_max_iteration;
  float refine_min_inlier_sym_score;
  float refine_max_inlier_sym_score;

  float bilSymAnn_min_occlusion_dist;
  float bilSymAnn_max_occlusion_dist;

  float bilSymAnn_max_occlusion_score;
  float min_segment_inlier_score;
  float min_corres_inlier_score;

  float sym_angle_diff;
  float sym_dist_diff;

  int numSegments;

  double pointSize;

public:
  BilateralSymmetryAnnotator () : DrawingAnnotator(__func__), pointSize(1.0)
  {
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    ctx.extractValue("bilSymAnn_isDownsampled", bilSymAnn_isDownsampled);
    ctx.extractValue("naive_detection", naive_detection);
    ctx.extractValue("downsample_voxel_size", downsample_voxel_size);

    ctx.extractValue("angle_division", angle_division);

    ctx.extractValue("dist_map_resolution", dist_map_resolution);

    ctx.extractValue("correspondence_search_radius", correspondence_search_radius);
    ctx.extractValue("correspondence_max_normal_fit_error", correspondence_max_normal_fit_error);
    ctx.extractValue("correspondence_min_sym_dist", correspondence_min_sym_dist);
    ctx.extractValue("correspondence_max_sym_reflected_dist", correspondence_max_sym_reflected_dist);

    ctx.extractValue("refine_max_iteration", refine_max_iteration);
    ctx.extractValue("refine_min_inlier_sym_score", refine_min_inlier_sym_score);
    ctx.extractValue("refine_max_inlier_sym_score", refine_max_inlier_sym_score);

    ctx.extractValue("bilSymAnn_min_occlusion_dist", bilSymAnn_min_occlusion_dist);
    ctx.extractValue("bilSymAnn_max_occlusion_dist", bilSymAnn_max_occlusion_dist);

    ctx.extractValue("bilSymAnn_max_occlusion_score", bilSymAnn_max_occlusion_score);
    ctx.extractValue("min_segment_inlier_score", min_segment_inlier_score);
    ctx.extractValue("min_corres_inlier_score", min_corres_inlier_score);

    ctx.extractValue("sym_angle_diff", sym_angle_diff);
    ctx.extractValue("sym_dist_diff", sym_dist_diff);

    extractor.initialize(bilSymAnn_isDownsampled,
                         naive_detection,
                         downsample_voxel_size,
                         angle_division,
                         dist_map_resolution,
                         correspondence_search_radius,
                         correspondence_max_normal_fit_error,
                         correspondence_min_sym_dist,
                         correspondence_max_sym_reflected_dist,
                         refine_max_iteration,
                         refine_min_inlier_sym_score,
                         refine_max_inlier_sym_score,
                         bilSymAnn_min_occlusion_dist,
                         bilSymAnn_max_occlusion_dist,
                         bilSymAnn_max_occlusion_score,
                         min_segment_inlier_score,
                         min_corres_inlier_score,
                         sym_angle_diff,
                         sym_dist_diff);


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

    //get RGB cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    cas.get(VIEW_CLOUD_OBJECTS, *cloud_ptr);
    if(cloud_ptr->size() == 0)
    {
      outInfo("Input Object cloud address is empty! Using cleaned cloud");
      rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
      cas.getFS(VIEW_CLOUD_NON_NAN, rcp);
      rs::conversion::from(rcp.cloud(), *cloud_ptr);
      rs::conversion::from(rcp.normals(), *normals);
    }
    else
    {
      //get normal cloud
      cas.get(VIEW_NORMALS_OBJECTS, *normals);
    }
    cloud = cloud_ptr;

    extractor.setInputClouds(cloud, normals);

    //get segments
    cas.get(VIEW_SEGMENT_IDS, segments);

    extractor.setInputSegmentIds(segments);

    //main execution
    extractor.extract();

    extractor.getSymmetries(finalSymmetries);
    extractor.getSupportIds(finalSupportSizeIds);

    //convert BilateralSymmetry to CAS Symmetries and push to CAS
    std::vector<rs::BilateralSymmetry> casSymmetries;
    for(size_t symId = 0; symId < finalSymmetries.size(); symId++)
    {
      rs::BilateralSymmetry currSym = rs::create<rs::BilateralSymmetry>(tcas);
      rs::Point3f currOrigin = rs::create<rs::Point3f>(tcas);
      rs::Point3f currNormal = rs::create<rs::Point3f>(tcas);

      Eigen::Vector3f eigenOrigin = finalSymmetries[symId].getOrigin();
      Eigen::Vector3f eigenNormal = finalSymmetries[symId].getNormal();

      currOrigin.x.set(eigenOrigin[0]);
      currOrigin.y.set(eigenOrigin[1]);
      currOrigin.z.set(eigenOrigin[2]);

      currNormal.x.set(eigenNormal[0]);
      currNormal.y.set(eigenNormal[1]);
      currNormal.z.set(eigenNormal[2]);

      currSym.origin.set(currOrigin);
      currSym.normal.set(currNormal);
      currSym.support.set(segments[finalSupportSizeIds[symId]].indices);
      casSymmetries.push_back(currSym);
    }

    outInfo("Total detected symmetries: " << casSymmetries.size());

    cas.set(VIEW_BILATERAL_SYMMETRIES, casSymmetries);

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";
    const std::string annotatorName = "BilateralSymmetryAnnotator";

    if(firstRun)
    {
      visualizer.addPointCloud(cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      addSymmetryPlanes(visualizer, finalSymmetries, 0.05f, 0.05f);
    }
    else
    {
      visualizer.updatePointCloud(cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.removeAllShapes();
      addSymmetryPlanes(visualizer, finalSymmetries, 0.05f, 0.05f);
      visualizer.addText(annotatorName, 2, 20, 12, 1, 1, 1, annotatorName);
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(BilateralSymmetryAnnotator)
