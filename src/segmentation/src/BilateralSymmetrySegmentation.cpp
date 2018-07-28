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
#include <omp.h>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>
#include <rs/types/all_types.h>

#include <rs/segmentation/BilateralSymmetrySegmenter.h>

#include <rs/visualization/Primitives.hpp>

// utils
#include <stdlib.h>
#include <time.h>



using namespace uima;


class BilateralSymmetrySegmentation : public DrawingAnnotator
{
private:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sceneCloud;
  pcl::PointCloud<pcl::Normal>::Ptr sceneNormals;

  std::vector< std::vector<int> > segmentIds;

  std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > segments;

  BilateralSymmetrySegmenter segmenter;

  //parameters
  int numSymmetries;
  bool bilSymSeg_isDownsampled;
  float downsample_voxel_size;

  float dist_map_resolution;

  float bilSymSeg_adjacency_radius;
  int bilSymSeg_num_adjacency_neighbors;
  float adjacency_sigma_convex;
  float adjacency_sigma_concave;
  float bilSymSeg_adjacency_weight_factor;

  float bilSymSeg_min_fit_angle;
  float bilSymSeg_max_fit_angle;
  float bilSymSeg_min_occlusion_dist;
  float bilSymSeg_max_occlusion_dist;
  float bilSymSeg_min_perpendicular_angle;
  float bilSymSeg_max_perpendicular_angle;
  float correspondence_max_sym_reflected_dist;

  float symmetric_weight_factor;
  float bilSymSeg_fg_weight_factor;
  float bilSymSeg_bg_weight_factor;

  float bilSymSeg_max_sym_score;
  float bilSymSeg_max_occlusion_score;
  float bilSymSeg_max_cut_score;
  float min_sym_sypport_overlap;
  int min_segment_size;

  float overlap_threshold;

  double pointSize;
  int segVisIt;

  enum
  {
    ALL,
    SEGMENT
  } dispMode;

public:
  BilateralSymmetrySegmentation() : DrawingAnnotator(__func__), pointSize(1.0), segVisIt(0), dispMode(ALL)
  {
    sceneCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    sceneNormals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

    srand(time(NULL));
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    ctx.extractValue("bilSymSeg_isDownsampled", bilSymSeg_isDownsampled);
    ctx.extractValue("downsample_voxel_size", downsample_voxel_size);
    ctx.extractValue("dist_map_resolution", dist_map_resolution);

    ctx.extractValue("bilSymSeg_adjacency_radius", bilSymSeg_adjacency_radius);
    ctx.extractValue("bilSymSeg_num_adjacency_neighbors", bilSymSeg_num_adjacency_neighbors);
    ctx.extractValue("adjacency_sigma_convex", adjacency_sigma_convex);
    ctx.extractValue("adjacency_sigma_concave", adjacency_sigma_concave);
    ctx.extractValue("bilSymSeg_adjacency_weight_factor", bilSymSeg_adjacency_weight_factor);

    ctx.extractValue("bilSymSeg_min_fit_angle", bilSymSeg_min_fit_angle);
    ctx.extractValue("bilSymSeg_max_fit_angle", bilSymSeg_max_fit_angle);
    ctx.extractValue("bilSymSeg_min_occlusion_dist", bilSymSeg_min_occlusion_dist);
    ctx.extractValue("bilSymSeg_max_occlusion_dist", bilSymSeg_max_occlusion_dist);
    ctx.extractValue("bilSymSeg_min_perpendicular_angle", bilSymSeg_min_perpendicular_angle);
    ctx.extractValue("bilSymSeg_max_perpendicular_angle", bilSymSeg_max_perpendicular_angle);
    ctx.extractValue("correspondence_max_sym_reflected_dist", correspondence_max_sym_reflected_dist);

    ctx.extractValue("symmetric_weight_factor", symmetric_weight_factor);
    ctx.extractValue("bilSymSeg_fg_weight_factor", bilSymSeg_fg_weight_factor);
    ctx.extractValue("bilSymSeg_bg_weight_factor", bilSymSeg_bg_weight_factor);

    ctx.extractValue("bilSymSeg_max_sym_score", bilSymSeg_max_sym_score);
    ctx.extractValue("bilSymSeg_max_occlusion_score", bilSymSeg_max_occlusion_score);
    ctx.extractValue("bilSymSeg_max_cut_score", bilSymSeg_max_cut_score);
    ctx.extractValue("min_sym_sypport_overlap", min_sym_sypport_overlap);
    ctx.extractValue("min_segment_size", min_segment_size);

    ctx.extractValue("overlap_threshold", overlap_threshold);

    segmenter.initialize(bilSymSeg_isDownsampled,
                         downsample_voxel_size,
                         dist_map_resolution,
                         bilSymSeg_adjacency_radius,
                         bilSymSeg_num_adjacency_neighbors,
                         adjacency_sigma_convex,
                         adjacency_sigma_concave,
                         bilSymSeg_adjacency_weight_factor,
                         bilSymSeg_min_fit_angle,
                         bilSymSeg_max_fit_angle,
                         bilSymSeg_min_occlusion_dist,
                         bilSymSeg_max_occlusion_dist,
                         bilSymSeg_min_perpendicular_angle,
                         bilSymSeg_max_perpendicular_angle,
                         correspondence_max_sym_reflected_dist,
                         symmetric_weight_factor,
                         bilSymSeg_fg_weight_factor,
                         bilSymSeg_bg_weight_factor,
                         bilSymSeg_max_sym_score,
                         bilSymSeg_max_occlusion_score,
                         bilSymSeg_max_cut_score,
                         min_sym_sypport_overlap,
                         min_segment_size,
                         overlap_threshold);

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
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_ptr(new pcl::PointCloud<pcl::Normal>);
    cas.get(VIEW_CLOUD_OBJECTS, *cloud_ptr);
    if(cloud_ptr->size() == 0)
    {
      outInfo("Input Object cloud address is empty! Using cleaned cloud");
      rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
      cas.getFS(VIEW_CLOUD_NON_NAN, rcp);
      rs::conversion::from(rcp.cloud(), *cloud_ptr);
      rs::conversion::from(rcp.normals(), *normals_ptr);
    }
    else
    {
      //get normal cloud
      cas.get(VIEW_NORMALS_OBJECTS, *normals_ptr);
    }
    sceneCloud = cloud_ptr;
    sceneNormals = normals_ptr;

    segmenter.setInputClouds(sceneCloud, sceneNormals);

    //get Rotational Symmteries
    std::vector<rs::BilateralSymmetry> casSymmetries;
    cas.get(VIEW_BILATERAL_SYMMETRIES, casSymmetries);
    numSymmetries = casSymmetries.size();

    if(numSymmetries < 1)
    {
      outWarn("No input bilateral symmteries! Segmentation abort!");
      return UIMA_ERR_NONE;
    }

    std::vector<BilateralSymmetry> symmetries(numSymmetries);
    std::vector< std::vector<int> > symmetrySupports(numSymmetries);
    #pragma omp parallel for
    for(size_t it = 0; it < casSymmetries.size(); it++)
    {
      Eigen::Vector3f currOrigin(casSymmetries[it].origin().x(), casSymmetries[it].origin().y(), casSymmetries[it].origin().z());
      Eigen::Vector3f currNormal(casSymmetries[it].normal().x(), casSymmetries[it].normal().y(), casSymmetries[it].normal().z());
      symmetries[it] = BilateralSymmetry(currOrigin, currNormal);
      symmetrySupports[it] = casSymmetries[it].support();
    }

    segmenter.setInputSymmetries(symmetries, symmetrySupports);

    //get bounding planes
    std::vector<rs::Plane> planes;
    scene.annotations.filter(planes);

    if(!planes.empty())
    {
      segmenter.setInputPlanes(planes);
    }

    segmenter.segment();

    segmenter.getSegmentIds(segmentIds);

    //extract good segment for visualizer and publish to CAS
    segments.clear();
    if(dispMode == ALL)
    {
      for(size_t pointId = 0; pointId < sceneCloud->size(); pointId++)
      {
        sceneCloud->points[pointId].r = 30;
        sceneCloud->points[pointId].g = 30;
        sceneCloud->points[pointId].b = 30;
      }
    }
    std::vector<pcl::PointIndices> casSegments;
    int finalSize = segmentIds.size();
    for(size_t segmentId = 0; segmentId < segmentIds.size(); segmentId++)
    {
      if(dispMode == SEGMENT)
      {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr currSegment(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::copyPointCloud(*sceneCloud, segmentIds[segmentId], *currSegment);
        segments.push_back(currSegment);
      }
      else if(dispMode == ALL)
      {
        int r = (255 / (finalSize + 2)) * (rand() % (finalSize + 1));
        int g = (255 / (finalSize + 2)) * (rand() % (finalSize + 1));
        int b = (255 / (finalSize + 2)) * (rand() % (finalSize + 1));
        for(size_t pointIdIt = 0; pointIdIt < segmentIds[segmentId].size(); pointIdIt++)
        {
          int pointId = segmentIds[segmentId][pointIdIt];
          sceneCloud->points[pointId].r = r;
          sceneCloud->points[pointId].g = g;
          sceneCloud->points[pointId].b = b;
        }
      }

      pcl::PointIndices currSegmentIds;
      currSegmentIds.indices = segmentIds[segmentId];
      casSegments.push_back(currSegmentIds);
    }

    cas.set(VIEW_BILATERAL_SEGMENTATION_IDS, casSegments);

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";
    const std::string annotatorName = "BilateralSymmetrySegmentation";

    if(numSymmetries > 0)
    {
      if(!firstRun)
      {
        visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      }
      visualizer.removeAllPointClouds();
      visualizer.removeAllShapes();
      if(dispMode == ALL)
      {
        visualizer.addPointCloud(sceneCloud, cloudname);
        visualizer.addText("Total Segment " + std::to_string(segmentIds.size()), 15, 125, 24, 1.0, 1.0, 1.0, "bilat_segments_text");
      }
      else if(dispMode == SEGMENT)
      {
        std::string symname = "sym" + std::to_string(segVisIt + 1);
        visualizer.addPointCloud(segments[segVisIt], cloudname);
        visualizer.addText("Segment " + std::to_string(segVisIt + 1) + " / " + std::to_string(segments.size()), 15, 125, 24, 1.0, 1.0, 1.0);
      }
      visualizer.addText(annotatorName, 2, 20, 12, 1, 1, 1, annotatorName);
    }
  }

  bool callbackKey(const int key, const Source source)
  {
    switch(key)
    {
    case 'a':
      segVisIt--;
      if(segVisIt < 0)
      {
        segVisIt = segments.size() - 1;
      }
      break;
    case 'd':
      segVisIt++;
      if(segVisIt >= segments.size())
      {
        segVisIt = 0;
      }
      break;
    case '1':
      dispMode = ALL;
      break;
    case '2':
      dispMode = SEGMENT;
      break;
    default:
      segVisIt = 0;
      break;
    }
    return true;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(BilateralSymmetrySegmentation)
