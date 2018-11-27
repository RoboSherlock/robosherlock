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

#include <rs/symmetrysegmentation/RotationalSymmetrySegmenter.h>
#include <rs/visualization/Primitives.hpp>

using namespace uima;

class RotationalSymmetrySegmentation : public DrawingAnnotator
{
private:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sceneCloud;
  pcl::PointCloud<pcl::Normal>::Ptr sceneNormals;

  std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > segments;

  RotationalSymmetrySegmenter segmenter;

  std::vector< std::vector<int> > segmentIds;

  //parameters
  int numSymmetries;
  bool rotSymSeg_isDownsampled;
  float downsample_leaf_size;

  float rotSymSeg_dist_map_resolution;

  float rotSymSeg_adjacency_radius;
  int rotSymSeg_num_adjacency_neighbors;

  float rotSymSeg_adjacency_sigma_convex;
  float rotSymSeg_adjacency_sigma_concave;
  float rotSymSeg_adjacency_weight_factor;

  float rotSymSeg_min_fit_angle;
  float rotSymSeg_max_fit_angle;
  float rotSymSeg_min_occlusion_dist;
  float rotSymSeg_max_occlusion_dist;
  float rotSymSeg_max_perpendicular_angle;

  float rotSymSeg_fg_weight_factor;
  float rotSymSeg_bg_weight_factor;

  float rotSymSeg_max_sym_score;
  float rotSymSeg_max_occlusion_score;
  float rotSymSeg_max_cut_score;
  int rotSymSeg_min_segment_size;

  float rotSymSeg_overlap_threshold;

  double pointSize;
  int segVisIt;

  enum
  {
    ALL,
    SEGMENT
  } dispMode;

public:
  RotationalSymmetrySegmentation () : DrawingAnnotator(__func__), pointSize(1.0), segVisIt(0), dispMode(ALL)
  {
    sceneCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    sceneNormals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    ctx.extractValue("rotSymSeg_isDownsampled", rotSymSeg_isDownsampled);
    ctx.extractValue("downsample_leaf_size", downsample_leaf_size);
    ctx.extractValue("rotSymSeg_dist_map_resolution", rotSymSeg_dist_map_resolution);
    ctx.extractValue("rotSymSeg_adjacency_radius", rotSymSeg_adjacency_radius);
    ctx.extractValue("rotSymSeg_num_adjacency_neighbors", rotSymSeg_num_adjacency_neighbors);
    ctx.extractValue("rotSymSeg_adjacency_sigma_convex", rotSymSeg_adjacency_sigma_convex);
    ctx.extractValue("rotSymSeg_adjacency_sigma_concave", rotSymSeg_adjacency_sigma_concave);
    ctx.extractValue("rotSymSeg_adjacency_weight_factor", rotSymSeg_adjacency_weight_factor);
    ctx.extractValue("rotSymSeg_min_fit_angle", rotSymSeg_min_fit_angle);
    ctx.extractValue("rotSymSeg_max_fit_angle", rotSymSeg_max_fit_angle);
    ctx.extractValue("rotSymSeg_min_occlusion_dist", rotSymSeg_min_occlusion_dist);
    ctx.extractValue("rotSymSeg_max_occlusion_dist", rotSymSeg_max_occlusion_dist);
    ctx.extractValue("rotSymSeg_max_perpendicular_angle", rotSymSeg_max_perpendicular_angle);
    ctx.extractValue("rotSymSeg_fg_weight_factor", rotSymSeg_fg_weight_factor);
    ctx.extractValue("rotSymSeg_bg_weight_factor", rotSymSeg_bg_weight_factor);
    ctx.extractValue("rotSymSeg_max_sym_score", rotSymSeg_max_sym_score);
    ctx.extractValue("rotSymSeg_max_occlusion_score", rotSymSeg_max_occlusion_score);
    ctx.extractValue("rotSymSeg_max_cut_score", rotSymSeg_max_cut_score);
    ctx.extractValue("rotSymSeg_min_segment_size", rotSymSeg_min_segment_size);

    ctx.extractValue("rotSymSeg_overlap_threshold", rotSymSeg_overlap_threshold);

    segmenter.initialize(rotSymSeg_isDownsampled,
                         downsample_leaf_size,
                         rotSymSeg_dist_map_resolution,
                         rotSymSeg_adjacency_radius,
                         rotSymSeg_num_adjacency_neighbors,
                         rotSymSeg_adjacency_sigma_convex,
                         rotSymSeg_adjacency_sigma_concave,
                         rotSymSeg_adjacency_weight_factor,
                         rotSymSeg_min_fit_angle,
                         rotSymSeg_max_fit_angle,
                         rotSymSeg_min_occlusion_dist,
                         rotSymSeg_max_occlusion_dist,
                         rotSymSeg_max_perpendicular_angle,
                         rotSymSeg_fg_weight_factor,
                         rotSymSeg_bg_weight_factor,
                         rotSymSeg_max_sym_score,
                         rotSymSeg_max_occlusion_score,
                         rotSymSeg_max_cut_score,
                         rotSymSeg_min_segment_size,
                         rotSymSeg_overlap_threshold);

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
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_ptr (new pcl::PointCloud<pcl::Normal>);
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
    std::vector<RotationalSymmetry> symmetries;
    std::vector<rs::RotationalSymmetry> casSymmetries;
    cas.get(VIEW_ROTATIONAL_SYMMETRIES, casSymmetries);
    numSymmetries = casSymmetries.size();

    if(numSymmetries < 1)
    {
      outWarn("No input rotational symmteries! Segmentation abort!");
      return UIMA_ERR_NONE;
    }
    symmetries.resize(numSymmetries);

    for(size_t it = 0; it < casSymmetries.size(); it++)
    {
      Eigen::Vector3f currOrigin(casSymmetries[it].origin().x(), casSymmetries[it].origin().y(), casSymmetries[it].origin().z());
      Eigen::Vector3f currOrientation(casSymmetries[it].orientation().x(), casSymmetries[it].orientation().y(), casSymmetries[it].orientation().z());
      symmetries[it] = RotationalSymmetry(currOrigin, currOrientation);
    }

    segmenter.setInputSymmetries(symmetries);

    //main execution
    segmenter.segment();

    segmenter.getSegmentIds(segmentIds);

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

    //extract good segment for visualizer and publish to CAS
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

    cas.set(VIEW_ROTATIONAL_SEGMENTATION_IDS, casSegments);

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";
    const std::string annotatorName = "RotationalSymmetrySegmentation";

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
        visualizer.addText("Total Segment " + std::to_string(segmentIds.size()), 15, 125, 24, 1.0, 1.0, 1.0, "rot_segments_text");
      }
      else if (dispMode == SEGMENT)
      {
        std::string symname = "sym" + std::to_string(segVisIt+1);
        visualizer.addPointCloud(segments[segVisIt], cloudname);
        visualizer.addText("Segment " + std::to_string(segVisIt+1) + " / " + std::to_string(segments.size()), 15, 125, 24, 1.0, 1.0, 1.0);
      }
      visualizer.addText(annotatorName, 2, 20, 12, 1, 1, 1, annotatorName);
    }
  }

private:
  bool callbackKey(const int key, const Source source)
  {
    switch(key)
    {
    case 'a':
      segVisIt--;
      if(segVisIt < 0)
        segVisIt = segments.size() - 1;
      break;
    case 'd':
      segVisIt++;
      if(segVisIt >= segments.size())
        segVisIt = 0;
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
MAKE_AE(RotationalSymmetrySegmentation)
