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
#include <mutex>

//PCL include
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/boundary.h>
#include <pcl/common/io.h>
#include <pcl/search/impl/kdtree.hpp>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>
#include <rs/types/all_types.h>

#include <rs/utils/array_utils.hpp>
#include <rs/segmentation/BoundarySegmentation.hpp>
#include <rs/segmentation/RotationalSymmetry.hpp>
#include <rs/segmentation/RotationalSymmetryScoring.hpp>
#include <rs/segmentation/SymmetrySegmentation.hpp>

#include <rs/mapping/DistanceMap.hpp>
#include <rs/mapping/DownsampleMap.hpp>

#include <rs/graph/WeightedGraph.hpp>
#include <rs/graph/Graph.hpp>
#include <rs/graph/GraphAlgorithms.hpp>

#include <rs/visualization/Primitives.hpp>

using namespace uima;


class RotationalSymmetrySegmentation : public DrawingAnnotator
{
private:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sceneCloud;
  pcl::PointCloud<pcl::Normal>::Ptr sceneNormals;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr dsSceneCloud;
  pcl::PointCloud<pcl::Normal>::Ptr dsSceneNormals;

  boost::shared_ptr< DistanceMap<pcl::PointXYZRGBA> > dist_map;
  std::vector<Eigen::Vector4f> boundingPlanes;

  WeightedGraph sceneGraph;

  std::vector<RotationalSymmetry> symmetries;
  std::vector<RotationalSymmetry> finalSymmetries;
  int numSymmetries;

  std::vector< float > symmetryScores;
  std::vector< float > occlusionScores;
  std::vector< float > cutScores;

  std::vector< std::vector< float > > pointSymScores;
  std::vector< std::vector< float > > pointOcclusionScores;
  std::vector< std::vector< float > > pointPerpendicularScores;

  std::vector< std::vector<int> > dsMap;

  std::vector< std::vector< float > > fgWeights;
  std::vector< std::vector< float > > bgWeights;

  std::vector< std::vector<int> > dsSegmentIds;
  std::vector< std::vector<int> > segmentIds;

  std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > segments;
  std::vector<int> filteredSegmentIds;
  std::vector<int> mergedSegmentIds;

  //parameters
  bool rotSymSeg_isDownsampled;
  float downsample_leaf_size;

  float dist_map_resolution;

  float rotSymSeg_adjacency_radius;
  int rotSymSeg_num_adjacency_neighbors;

  float adjacency_sigma_convex;
  float adjacency_sigma_concave;
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
  int min_segment_size;

  float overlap_threshold;

  double pointSize;
  int segVisIt;

  enum
  {
    ALL,
    SEGMENT
  } dispMode;

  std::mutex sym_mutex;

public:
  RotationalSymmetrySegmentation () : DrawingAnnotator(__func__), pointSize(1.0), segVisIt(0), dispMode(ALL)
  {
    sceneCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    sceneNormals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    dsSceneCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    dsSceneNormals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    ctx.extractValue("rotSymSeg_isDownsampled", rotSymSeg_isDownsampled);
    ctx.extractValue("downsample_leaf_size", downsample_leaf_size);
    ctx.extractValue("dist_map_resolution", dist_map_resolution);
    ctx.extractValue("rotSymSeg_adjacency_radius", rotSymSeg_adjacency_radius);
    ctx.extractValue("rotSymSeg_num_adjacency_neighbors", rotSymSeg_num_adjacency_neighbors);
    ctx.extractValue("adjacency_sigma_convex", adjacency_sigma_convex);
    ctx.extractValue("adjacency_sigma_concave", adjacency_sigma_concave);
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
    ctx.extractValue("min_segment_size", min_segment_size);

    ctx.extractValue("overlap_threshold", overlap_threshold);

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

    //clearing previous data
    symmetryScores.clear();
    occlusionScores.clear();
    cutScores.clear();
    pointSymScores.clear();
    pointOcclusionScores.clear();
    pointPerpendicularScores.clear();
    dsMap.clear();
    fgWeights.clear();
    bgWeights.clear();
    symmetries.clear();
    finalSymmetries.clear();
    segmentIds.clear();
    dsSegmentIds.clear();
    segments.clear();
    filteredSegmentIds.clear();
    mergedSegmentIds.clear();
    boundingPlanes.clear();

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

    //get Rotational Symmteries
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

    //allocating containers
    symmetryScores.resize(numSymmetries);
    occlusionScores.resize(numSymmetries);
    cutScores.resize(numSymmetries);
    pointSymScores.resize(numSymmetries);
    pointOcclusionScores.resize(numSymmetries);
    pointPerpendicularScores.resize(numSymmetries);
    fgWeights.resize(numSymmetries);
    bgWeights.resize(numSymmetries);
    segmentIds.resize(numSymmetries);
    dsSegmentIds.resize(numSymmetries);

    //main execution

    //downsample the cloud and normal cloud to speed up segmentation
    if(rotSymSeg_isDownsampled)
    {
      std::vector<int> nearestMap;
      DownsampleMap<pcl::PointXYZRGBA> dc;
      dc.setInputCloud(sceneCloud);
      dc.setLeafSize(downsample_leaf_size);
      dc.filter(*dsSceneCloud);
      dc.getDownsampleMap(dsMap);
      dc.getNearestNeighborMap(nearestMap);

      computeDownsampleNormals(sceneNormals, dsMap, nearestMap, AVERAGE, dsSceneNormals);
    }
    else
    {
      dsSceneCloud = sceneCloud;
      dsSceneNormals = sceneNormals;
    }

    //get bounding planes
    std::vector<rs::Plane> planes;
    scene.annotations.filter(planes);
    boundingPlanes.resize(planes.size());
    if(planes.empty())
    {
      outWarn("Planes are not found! Using default plane z=0");
      boundingPlanes.push_back(Eigen::Vector4f::UnitZ());
    }
    else
    {
      for(size_t planeId = 0; planeId < planes.size(); planeId++)
      {
        boundingPlanes[planeId] = Eigen::Vector4f(planes[planeId].model()[0], planes[planeId].model()[1], planes[planeId].model()[2], planes[planeId].model()[3]);
      }
    }

    //initialize distance map
    dist_map = boost::shared_ptr< DistanceMap< pcl::PointXYZRGBA > >(new DistanceMap <pcl::PointXYZRGBA> (dist_map_resolution));
    dist_map->setBoundingPlanes(boundingPlanes);
    dist_map->setInputCloud(sceneCloud);

    //compute adjacency weigth for smoothness term
    if(!computeCloudAdjacencyWeight<pcl::PointXYZRGBA>(dsSceneCloud, dsSceneNormals, rotSymSeg_adjacency_radius, rotSymSeg_num_adjacency_neighbors, sceneGraph, rotSymSeg_adjacency_weight_factor))
    {
      outWarn("Could not construct adjacency graph!");
      return UIMA_ERR_NONE;
    }


    #pragma omp parallel for
    for(size_t symId = 0; symId < numSymmetries; symId++)
    {
      //compute point scores for each symmetry
      getCloudRotationalSymmetryScore<pcl::PointXYZRGBA>(dsSceneCloud, dsSceneNormals, symmetries[symId], pointSymScores[symId], rotSymSeg_min_fit_angle, rotSymSeg_max_fit_angle);
      getCloudRotationalOcclusionScore<pcl::PointXYZRGBA>(dsSceneCloud, *dist_map, symmetries[symId], pointOcclusionScores[symId], rotSymSeg_min_occlusion_dist, rotSymSeg_max_occlusion_dist);
      getCloudRotationalPerpendicularScore(sceneNormals, symmetries[symId], pointPerpendicularScores[symId], rotSymSeg_max_perpendicular_angle);

      //compute unary weight from scores
      fgWeights[symId].resize(dsSceneCloud->points.size());
      bgWeights[symId].resize(dsSceneCloud->points.size());

      for(size_t pId = 0; pId < dsSceneCloud->points.size(); pId++)
      {
        fgWeights[symId][pId] = (1.0f - pointSymScores[symId][pId]) * (1.0f - pointOcclusionScores[symId][pId]) * (1.0f - pointPerpendicularScores[symId][pId]) * rotSymSeg_fg_weight_factor;

        bgWeights[symId][pId] = (pointSymScores[symId][pId] * (1.0f - pointPerpendicularScores[symId][pId]) + pointOcclusionScores[symId][pId]) * rotSymSeg_bg_weight_factor;
      }

      //sym_mutex.lock();

      std::vector<int> backgroundIds;
      float min_cut_value;
      float max_flow = BoykovMinCut::min_cut(fgWeights[symId], bgWeights[symId], sceneGraph, dsSegmentIds[symId], backgroundIds, min_cut_value);

      if(max_flow < 0.0f)
      {
        outWarn("Could not segment cloud using Boykov min_cut! abort!");
      }

      //compute segment score for filtering
      symmetryScores[symId] = 0.0f;
      occlusionScores[symId] = 0.0f;
      cutScores[symId] = 0.0f;

      if(!dsSegmentIds[symId].empty())
      {
        for(size_t pointIdIt = 0; pointIdIt < dsSegmentIds[symId].size(); pointIdIt++)
        {
          int pointId = dsSegmentIds[symId][pointIdIt];
          occlusionScores[symId] += pointOcclusionScores[symId][pointId];
          symmetryScores[symId] += pointSymScores[symId][pointId];
        }
        occlusionScores[symId] /= static_cast<float>(dsSegmentIds[symId].size());
        symmetryScores[symId] /= static_cast<float>(dsSegmentIds[symId].size());

        if(dsSegmentIds[symId].size() != dsSceneCloud->size())
        {
          cutScores[symId] = min_cut_value / static_cast<float>(dsSegmentIds[symId].size());
        }
      }

      if(rotSymSeg_isDownsampled)
      {
        upsample_cloud(dsSegmentIds[symId], dsMap, segmentIds[symId]);
      }
      else
      {
        segmentIds = dsSegmentIds;
      }

      //sym_mutex.unlock();
    }

    //filtering
    this->filter();
    this->merge();

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
    int finalSize = mergedSegmentIds.size();
    for(size_t segmentIdIt = 0; segmentIdIt < mergedSegmentIds.size(); segmentIdIt++)
    {
      int segmentId = mergedSegmentIds[segmentIdIt];
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
      finalSymmetries.push_back(symmetries[segmentId]);

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
        addSymmetryLines(visualizer, finalSymmetries, 0.4f, 0.8f);
        visualizer.addText("Total Segment " + std::to_string(mergedSegmentIds.size()), 15, 125, 24, 1.0, 1.0, 1.0, "rot_segments_text");
      }
      else if (dispMode == SEGMENT)
      {
        std::string symname = "sym" + std::to_string(segVisIt+1);
        visualizer.addPointCloud(segments[segVisIt], cloudname);
        addSymmetryLine(visualizer, finalSymmetries[segVisIt], symname, 0.4f, 0.8f);
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

  inline void filter(){
    for(size_t symId = 0; symId < numSymmetries; symId++)
    {
      if( symmetryScores[symId] < rotSymSeg_max_sym_score &&
          occlusionScores[symId] < rotSymSeg_max_occlusion_score &&
          cutScores[symId] < rotSymSeg_max_cut_score &&
          segmentIds[symId].size() > min_segment_size)
      {
        filteredSegmentIds.push_back(symId);
      }
    }
  }

  inline void merge()
  {
    Graph similarSegments(filteredSegmentIds.size());

    for(size_t srcSegmentIdIt = 0; srcSegmentIdIt < filteredSegmentIds.size(); srcSegmentIdIt++)
    {
      int srcSegmentId = filteredSegmentIds[srcSegmentIdIt];
      for(size_t tgtSegmentIdIt = srcSegmentIdIt+1; tgtSegmentIdIt < filteredSegmentIds.size(); tgtSegmentIdIt++)
      {
        int tgtSegmentId = filteredSegmentIds[tgtSegmentIdIt];

        int intersectSize = Intersection(segmentIds[srcSegmentId], segmentIds[tgtSegmentId]).size();
        int unionSize = Union(segmentIds[srcSegmentId], segmentIds[tgtSegmentId]).size();
        float ratio = (float) intersectSize / unionSize;
        if(ratio > overlap_threshold)
        {
          similarSegments.addEdge(srcSegmentIdIt, tgtSegmentIdIt);
        }
      }
    }

    std::vector< std::vector<int> > connectedSegments;
    connectedSegments = extractConnectedComponents(similarSegments);

    for(size_t ccId = 0; ccId < connectedSegments.size(); ccId++)
    {
      int bestId = -1;
      int max_segment_size = 0;
      for(size_t segIdIt = 0; segIdIt < connectedSegments[ccId].size(); segIdIt++)
      {
        int segmentId = filteredSegmentIds[connectedSegments[ccId][segIdIt]];
        if(segmentIds[segmentId].size() > max_segment_size)
        {
          bestId = segmentId;
          max_segment_size = segmentIds[segmentId].size();
        }
      }
      if(bestId != -1)
      {
        mergedSegmentIds.push_back(bestId);
      }
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(RotationalSymmetrySegmentation)
