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

#include <rs/segmentation/RotationalSymmetrySegmenter.h>

RotationalSymmetrySegmenter::RotationalSymmetrySegmenter()
{
  this->sceneCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
  this->sceneNormals.reset(new pcl::PointCloud<pcl::Normal>());
}

RotationalSymmetrySegmenter::~RotationalSymmetrySegmenter() {}

void RotationalSymmetrySegmenter::setInputClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                                                 pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
  this->sceneCloud = cloud;
  this->sceneNormals = normals;
}

void RotationalSymmetrySegmenter::setInputSymmetries(std::vector<RotationalSymmetry> &symmetries)
{
  this->symmetries = symmetries;
  this->numSymmetries = symmetries.size();
}

void RotationalSymmetrySegmenter::setInputPlanes(std::vector<rs::Plane> &planes)
{
  this->planes = planes;
}

void RotationalSymmetrySegmenter::initialize(bool rotSymSeg_isDownsampled,
                                             float downsample_leaf_size,
                                             float dist_map_resolution,
                                             float rotSymSeg_adjacency_radius,
                                             int rotSymSeg_num_adjacency_neighbors,
                                             float adjacency_sigma_convex,
                                             float adjacency_sigma_concave,
                                             float rotSymSeg_adjacency_weight_factor,
                                             float rotSymSeg_min_fit_angle,
                                             float rotSymSeg_max_fit_angle,
                                             float rotSymSeg_min_occlusion_dist,
                                             float rotSymSeg_max_occlusion_dist,
                                             float rotSymSeg_max_perpendicular_angle,
                                             float rotSymSeg_fg_weight_factor,
                                             float rotSymSeg_bg_weight_factor,
                                             float rotSymSeg_max_sym_score,
                                             float rotSymSeg_max_occlusion_score,
                                             float rotSymSeg_max_cut_score,
                                             int min_segment_size,
                                             float overlap_threshold)
{
  this->rotSymSeg_isDownsampled = rotSymSeg_isDownsampled;
  this->downsample_leaf_size = downsample_leaf_size;
  this->dist_map_resolution = dist_map_resolution;
  this->rotSymSeg_adjacency_radius = rotSymSeg_adjacency_radius;
  this->rotSymSeg_num_adjacency_neighbors = rotSymSeg_num_adjacency_neighbors;
  this->adjacency_sigma_convex = adjacency_sigma_convex;
  this->adjacency_sigma_concave = adjacency_sigma_concave;
  this->rotSymSeg_adjacency_weight_factor = rotSymSeg_adjacency_weight_factor;
  this->rotSymSeg_min_fit_angle = rotSymSeg_min_fit_angle;
  this->rotSymSeg_max_fit_angle = rotSymSeg_max_fit_angle;
  this->rotSymSeg_min_occlusion_dist = rotSymSeg_min_occlusion_dist;
  this->rotSymSeg_max_occlusion_dist = rotSymSeg_max_occlusion_dist;
  this->rotSymSeg_max_perpendicular_angle = rotSymSeg_max_perpendicular_angle;
  this->rotSymSeg_fg_weight_factor = rotSymSeg_fg_weight_factor;
  this->rotSymSeg_bg_weight_factor = rotSymSeg_bg_weight_factor;
  this->rotSymSeg_max_sym_score = rotSymSeg_max_sym_score;
  this->rotSymSeg_max_occlusion_score = rotSymSeg_max_occlusion_score;
  this->rotSymSeg_max_cut_score = rotSymSeg_max_cut_score;
  this->min_segment_size = min_segment_size;
  this->overlap_threshold = overlap_threshold;
}

bool RotationalSymmetrySegmenter::segment()
{
  if(sceneCloud->empty() || sceneNormals->empty())
  {
    outError("Cloud are empty! Have you inputs cloud yet?");
    return false;
  }

  if(symmetries.empty())
  {
    outError("There is no symmetry to segment! Aborting!");
    return false;
  }

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
  segmentIds.clear();
  dsSegmentIds.clear();
  filteredSegmentIds.clear();
  mergedSegmentIds.clear();
  boundingPlanes.clear();

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

  dsSceneCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
  dsSceneNormals.reset(new pcl::PointCloud<pcl::Normal>());

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

  //generate bounding planes
  if(planes.empty())
  {
    outWarn("Planes are not found! Using default plane z=0");
    boundingPlanes.push_back(Eigen::Vector4f::UnitZ());
  }
  else
  {
    boundingPlanes.resize(planes.size());
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
    outError("Could not construct adjacency graph!");
    return false;
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

    std::vector<int> backgroundIds;
    float min_cut_value;
    float max_flow = BoykovMinCut::min_cut(fgWeights[symId], bgWeights[symId], sceneGraph, dsSegmentIds[symId], backgroundIds, min_cut_value);

    if(max_flow < 0.0f)
    {
      outWarn("Could not segment cloud using Boykov min_cut on symId: " << symId << " Abort!");
      continue;
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
      segmentIds[symId] = dsSegmentIds[symId];
    }
  }

  //filtering
  this->filter();
  this->merge();

  return true;
}

bool RotationalSymmetrySegmenter::getSegmentIds(std::vector< std::vector<int> > &segmentIds)
{
  if(this->segmentIds.empty())
  {
    outError("Segments is unavailable! Have you run segment yet?");
    return false;
  }

  segmentIds.clear();
  for(size_t segmentIdIt = 0; segmentIdIt < mergedSegmentIds.size(); segmentIdIt++)
  {
    segmentIds.push_back(this->segmentIds[mergedSegmentIds[segmentIdIt]]);
  }

  return true;
}

bool RotationalSymmetrySegmenter::filter()
{
  if(symmetryScores.empty())
  {
    outError("Symmetry segment scores are empty! Have you run segment yet?");
    return false;
  }

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

  return true;
}

bool RotationalSymmetrySegmenter::merge()
{
  if(filteredSegmentIds.empty())
  {
    outWarn("Filtered segments is empty! Nothing to merge!");
    return false;
  }

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

  return true;
}
