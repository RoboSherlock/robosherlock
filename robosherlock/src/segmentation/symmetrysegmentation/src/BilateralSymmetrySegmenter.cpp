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

 #include <rs/symmetrysegmentation/BilateralSymmetrySegmenter.h>

 BilateralSymmetrySegmenter::BilateralSymmetrySegmenter()
 {
   this->sceneCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
   this->sceneNormals.reset(new pcl::PointCloud<pcl::Normal>());
   this->dsSceneCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
   this->dsSceneNormals.reset(new pcl::PointCloud<pcl::Normal>());

   this->isSetup = false;
 }

 BilateralSymmetrySegmenter::~BilateralSymmetrySegmenter() {}


 void BilateralSymmetrySegmenter::setInputClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                                                 pcl::PointCloud<pcl::Normal>::Ptr &normals)
 {
   this->sceneCloud = cloud;
   this->sceneNormals = normals;
 }

 void BilateralSymmetrySegmenter::setInputSymmetries(std::vector<BilateralSymmetry> &symmetries,
                                                     std::vector< std::vector<int> > &symmetrySupports)
 {
   if(symmetries.size() == symmetrySupports.size())
   {
     this->symmetries = symmetries;
     this->symmetrySupports = symmetrySupports;
     this->numSymmetries = symmetries.size();
   }
   else
   {
     outWarn("Size of symmetries is not equal to size of symmetry supports! Rejected!");
   }
 }

 void BilateralSymmetrySegmenter::setInputPlanes(std::vector<rs::Plane> &planes)
 {
   this->planes = planes;
 }

 void BilateralSymmetrySegmenter::initialize(bool bilSymSeg_isDownsampled,
                                             float downsample_voxel_size,
                                             float dist_map_resolution,
                                             float bilSymSeg_adjacency_radius,
                                             int bilSymSeg_num_adjacency_neighbors,
                                             float adjacency_sigma_convex,
                                             float adjacency_sigma_concave,
                                             float bilSymSeg_adjacency_weight_factor,
                                             float bilSymSeg_min_fit_angle,
                                             float bilSymSeg_max_fit_angle,
                                             float bilSymSeg_min_occlusion_dist,
                                             float bilSymSeg_max_occlusion_dist,
                                             float bilSymSeg_min_perpendicular_angle,
                                             float bilSymSeg_max_perpendicular_angle,
                                             float correspondence_max_sym_reflected_dist,
                                             float symmetric_weight_factor,
                                             float bilSymSeg_fg_weight_factor,
                                             float bilSymSeg_bg_weight_factor,
                                             float bilSymSeg_max_sym_score,
                                             float bilSymSeg_max_occlusion_score,
                                             float bilSymSeg_max_cut_score,
                                             float min_sym_sypport_overlap,
                                             int min_segment_size,
                                             float overlap_threshold)
{
  this->bilSymSeg_isDownsampled = bilSymSeg_isDownsampled;
  this->downsample_voxel_size = downsample_voxel_size;
  this->dist_map_resolution = dist_map_resolution;
  this->bilSymSeg_adjacency_radius = bilSymSeg_adjacency_radius;
  this->bilSymSeg_num_adjacency_neighbors = bilSymSeg_num_adjacency_neighbors;
  this->adjacency_sigma_convex = adjacency_sigma_convex;
  this->adjacency_sigma_concave = adjacency_sigma_concave;
  this->bilSymSeg_adjacency_weight_factor = bilSymSeg_adjacency_weight_factor;
  this->bilSymSeg_min_fit_angle = bilSymSeg_min_fit_angle;
  this->bilSymSeg_max_fit_angle = bilSymSeg_max_fit_angle;
  this->bilSymSeg_min_occlusion_dist = bilSymSeg_min_occlusion_dist;
  this->bilSymSeg_max_occlusion_dist = bilSymSeg_max_occlusion_dist;
  this->bilSymSeg_min_perpendicular_angle = bilSymSeg_min_perpendicular_angle;
  this->bilSymSeg_max_perpendicular_angle = bilSymSeg_max_perpendicular_angle;
  this->correspondence_max_sym_reflected_dist = correspondence_max_sym_reflected_dist;
  this->symmetric_weight_factor = symmetric_weight_factor;
  this->bilSymSeg_fg_weight_factor = bilSymSeg_fg_weight_factor;
  this->bilSymSeg_bg_weight_factor = bilSymSeg_bg_weight_factor;
  this->bilSymSeg_max_sym_score = bilSymSeg_max_sym_score;
  this->bilSymSeg_max_occlusion_score = bilSymSeg_max_occlusion_score;
  this->bilSymSeg_max_cut_score = bilSymSeg_max_cut_score;
  this->min_sym_sypport_overlap = min_sym_sypport_overlap;
  this->min_segment_size = min_segment_size;
  this->overlap_threshold = overlap_threshold;

  this->isSetup = true;
}

bool BilateralSymmetrySegmenter::segment()
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

  if(!this->isSetup)
  {
    outError("Parameters are not set! Abort.");
    return false;
  }

  correspondences.clear();
  symmetricGraph.clear();
  symmetryScores.clear();
  occlusionScores.clear();
  cutScores.clear();
  pointSymScores.clear();
  pointOcclusionScores.clear();
  pointPerpendicularScores.clear();
  symmetrySupportOverlapScores.clear();
  dsMap.clear();
  reversedMap.clear();
  fgWeights.clear();
  bgWeights.clear();
  segmentIds.clear();
  dsSegmentIds.clear();
  filteredSegmentIds.clear();
  mergedSegmentIds.clear();

  //allocating containers
  correspondences.resize(numSymmetries);
  symmetricGraph.resize(numSymmetries);
  symmetryScores.resize(numSymmetries);
  occlusionScores.resize(numSymmetries);
  cutScores.resize(numSymmetries);
  symmetrySupportOverlapScores.resize(numSymmetries);
  pointSymScores.resize(numSymmetries);
  pointOcclusionScores.resize(numSymmetries);
  pointPerpendicularScores.resize(numSymmetries);
  fgWeights.resize(numSymmetries);
  bgWeights.resize(numSymmetries);
  segmentIds.resize(numSymmetries);
  dsSegmentIds.resize(numSymmetries);

  //downsample the cloud and normal cloud to speed up segmentation
  if(bilSymSeg_isDownsampled)
  {
    std::vector<int> nearestMap;
    DownsampleMap<pcl::PointXYZRGBA> dc;
    dc.setInputCloud(sceneCloud);
    dc.setLeafSize(downsample_voxel_size);
    dc.filter(*dsSceneCloud);
    dc.getDownsampleMap(dsMap);
    dc.getNearestNeighborMap(nearestMap);
    dc.getReversedMap(reversedMap);

    computeDownsampleNormals(sceneNormals, dsMap, nearestMap, AVERAGE, dsSceneNormals);
  }
  else
  {
    dsSceneCloud = sceneCloud;
    dsSceneNormals = sceneNormals;
  }

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

  //main execution
  //compute adjacency weigth for smoothness term
  if(!computeCloudAdjacencyWeight<pcl::PointXYZRGBA>(dsSceneCloud, dsSceneNormals, bilSymSeg_adjacency_radius, bilSymSeg_num_adjacency_neighbors, sceneGraph, bilSymSeg_adjacency_weight_factor))
  {
    outError("Could not construct adjacency graph!");
    return false;
  }

  //initialize search_tree
  search_tree.reset(new pcl::search::KdTree<pcl::PointXYZRGBA>());
  search_tree->setInputCloud(sceneCloud);

  #pragma omp parallel for
  for(size_t symId = 0; symId < numSymmetries; symId++)
  {
    // setup mask for faster interation
    std::vector<bool> supportMask(dsSceneCloud->size(), false);
    for(size_t pointIdIt = 0; pointIdIt < symmetrySupports[symId].size(); pointIdIt++)
    {
      if(bilSymSeg_isDownsampled)
      {
        int dsPointId = reversedMap[symmetrySupports[symId][pointIdIt]];
        supportMask[dsPointId] = true;
      }
      else
      {
        supportMask[symmetrySupports[symId][pointIdIt]] = true;
      }
    }

    //compute score
    getCloudBilateralSymmetryScore<pcl::PointXYZRGBA>(sceneCloud, sceneNormals, dsSceneCloud, dsSceneNormals, search_tree, symmetries[symId], correspondences[symId], pointSymScores[symId], 0.01f, 0.174f, 0.02f, correspondence_max_sym_reflected_dist, bilSymSeg_min_fit_angle, bilSymSeg_max_fit_angle);
    getCloudBilateralOcclusionScore<pcl::PointXYZRGBA>(dsSceneCloud, *dist_map, symmetries[symId], pointOcclusionScores[symId], bilSymSeg_min_occlusion_dist, bilSymSeg_max_occlusion_dist);
    getCloudBilateralPerpendicularScore(dsSceneNormals, symmetries[symId], pointPerpendicularScores[symId], bilSymSeg_min_perpendicular_angle, bilSymSeg_max_perpendicular_angle);

    //compute unary weight from scores
    fgWeights[symId].resize(dsSceneCloud->points.size(), 0.0f);
    bgWeights[symId].resize(dsSceneCloud->points.size());

    for(size_t pId = 0; pId < dsSceneCloud->points.size(); pId++)
    {
      bgWeights[symId][pId] = pointOcclusionScores[symId][pId] * bilSymSeg_bg_weight_factor;
    }

    for(size_t corresId = 0; corresId < correspondences[symId].size(); corresId++)
    {
      int pointId = correspondences[symId][corresId].index_query;
      fgWeights[symId][pointId] = (1.0f - pointSymScores[symId][corresId]) * bilSymSeg_fg_weight_factor;
      if(!supportMask[pointId])
      {
        fgWeights[symId][pointId] *= (1.0f - pointPerpendicularScores[symId][pointId]);
      }
    }

    //compute symmetric weight
    symmetricGraph[symId] = sceneGraph;
    for(size_t corresId = 0; corresId < correspondences[symId].size(); corresId++)
    {
      if(correspondences[symId][corresId].distance < 0.0f || pointSymScores[symId][corresId] >= 1.0f)
      {
        continue;
      }

      int srcPointId, tgtPointId;
      srcPointId = correspondences[symId][corresId].index_query;
      if(bilSymSeg_isDownsampled)
      {
        //convert to downsample ID
        tgtPointId = reversedMap[correspondences[symId][corresId].index_match];
      }
      else
      {
        tgtPointId = correspondences[symId][corresId].index_match;
      }

      if(srcPointId != tgtPointId)
      {
        // need checking: (1.0f - pointSymScores[symId][corresId]) * symmetric_weight_factor or just symmetric_weight_factor;
        symmetricGraph[symId].addEdge(srcPointId, tgtPointId, symmetric_weight_factor * bilSymSeg_adjacency_weight_factor); // * (1.0f - pointSymScores[symId][corresId])
      }
    }

    std::vector<int> backgroundIds;
    float min_cut_value;
    float max_flow = BoykovMinCut::min_cut(fgWeights[symId], bgWeights[symId], symmetricGraph[symId], dsSegmentIds[symId], backgroundIds, min_cut_value);

    if(max_flow < 0.0f)
    {
      outWarn("Could not segment cloud using Boykov min_cut! abort!");
    }

    //compute segment score for filtering
    symmetryScores[symId] = 0.0f;
    occlusionScores[symId] = 0.0f;
    cutScores[symId] = 0.0f;
    symmetrySupportOverlapScores[symId] = 0.0f;

    if(dsSegmentIds[symId].size() > min_segment_size)
    {
      std::vector<bool> dsSegmentMask(dsSceneCloud->size(), false);
      for(size_t pointIdIt = 0; pointIdIt < dsSegmentIds[symId].size(); pointIdIt++)
      {
        dsSegmentMask[dsSegmentIds[symId][pointIdIt]] = true;
      }

      //compute symmetry scores
      int numInlier = 0;
      for(size_t corresId = 0; corresId < correspondences[symId].size(); corresId++)
      {
        int srcPointId, tgtPointId;
        srcPointId = correspondences[symId][corresId].index_query;
        if(bilSymSeg_isDownsampled)
        {
          //convert to downsample ID
          tgtPointId = reversedMap[correspondences[symId][corresId].index_match];
        }
        else
        {
          tgtPointId = correspondences[symId][corresId].index_match;
        }

        if(dsSegmentMask[srcPointId] && dsSegmentMask[tgtPointId])
        {
          numInlier++;
          symmetryScores[symId] += pointSymScores[symId][corresId];
        }
      }

      if(numInlier > 0)
      {
        symmetryScores[symId] /= static_cast<float>(numInlier);
      }
      else
      {
        symmetryScores[symId] = 1.0f;
      }

      //compute occlusion score
      for(size_t pointIdIt = 0; pointIdIt < dsSegmentIds[symId].size(); pointIdIt++)
      {
        int pointId = dsSegmentIds[symId][pointIdIt];
        occlusionScores[symId] += pointOcclusionScores[symId][pointId];
      }
      occlusionScores[symId] /= static_cast<float>(dsSegmentIds[symId].size());

      //compute cut score
      if(dsSegmentIds[symId].size() != dsSceneCloud->size())
      {
        cutScores[symId] = min_cut_value / static_cast<float>(dsSegmentIds[symId].size());
      }

      //compute overlap score
      for(size_t pointIdIt = 0; pointIdIt < dsSegmentIds[symId].size(); pointIdIt++)
      {
        int dsPointId = dsSegmentIds[symId][pointIdIt];
        if(supportMask[dsPointId])
        {
          symmetrySupportOverlapScores[symId] += 1.0f;
        }
      }
      if(bilSymSeg_isDownsampled)
      {
        int denom = 0;
        for(size_t it = 0; it < supportMask.size(); it++)
        {
          denom += static_cast<int>(supportMask[it]);
        }
        if(denom != 0)
        {
          symmetrySupportOverlapScores[symId] /= static_cast<float>(denom);
        }
        else
        {
          symmetrySupportOverlapScores[symId] = 0.0f;
        }
      }
      else
      {
        symmetrySupportOverlapScores[symId] /= static_cast<float>(symmetrySupports[symId].size());
      }
    }

    //if downsampled, upsample the cloud
    if(bilSymSeg_isDownsampled)
    {
      upsample_cloud(dsSegmentIds[symId], dsMap, segmentIds[symId]);
    }
    else
    {
      segmentIds[symId] = dsSegmentIds[symId];
    }
  }

  //filter segments
  this->filter();
  this->merge();

  return true;
}

bool BilateralSymmetrySegmenter::getSegmentIds(std::vector< std::vector<int> > &segmentIds)
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

bool BilateralSymmetrySegmenter::filter()
{
  if(symmetryScores.empty())
  {
    outError("Symmetry segment scores are empty! Have you run segment yet?");
    return false;
  }

  for(size_t symId = 0; symId < numSymmetries; symId++)
  {
    if(symmetryScores[symId] < bilSymSeg_max_sym_score &&
       occlusionScores[symId] < bilSymSeg_max_occlusion_score &&
       cutScores[symId] < bilSymSeg_max_cut_score &&
       segmentIds[symId].size() > min_segment_size &&
       symmetrySupportOverlapScores[symId] > min_sym_sypport_overlap)
    {
      filteredSegmentIds.push_back(symId);
    }
  }

  return true;
}

bool BilateralSymmetrySegmenter::merge()
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
    for(size_t tgtSegmentIdIt = srcSegmentIdIt + 1; tgtSegmentIdIt < filteredSegmentIds.size(); tgtSegmentIdIt++)
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
