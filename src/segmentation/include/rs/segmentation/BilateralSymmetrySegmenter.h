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

#include <vector>
#include <omp.h>

#include <boost/make_shared.hpp>

//PCL include
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/boundary.h>
#include <pcl/common/io.h>
#include <pcl/search/impl/kdtree.hpp>

//RS
#include <rs/types/all_types.h>

#include <rs/utils/array_utils.hpp>
#include <rs/segmentation/BoundarySegmentation.hpp>
#include <rs/segmentation/BilateralSymmetry.hpp>
#include <rs/segmentation/BilateralSymmetryScoring.hpp>
#include <rs/segmentation/SymmetrySegmentation.hpp>

#include <rs/mapping/DistanceMap.hpp>
#include <rs/mapping/DownsampleMap.hpp>

#include <rs/graph/WeightedGraph.h>
#include <rs/graph/Graph.h>
#include <rs/graph/GraphAlgorithms.hpp>

class BilateralSymmetrySegmenter
{
private:
  //inputs
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sceneCloud;
  pcl::PointCloud<pcl::Normal>::Ptr sceneNormals;

  std::vector<BilateralSymmetry> symmetries;
  std::vector< std::vector<int> > symmetrySupports;

  std::vector<rs::Plane> planes;

  //intermediate variables
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr dsSceneCloud;
  pcl::PointCloud<pcl::Normal>::Ptr dsSceneNormals;

  WeightedGraph sceneGraph;
  std::vector<WeightedGraph> symmetricGraph;

  std::vector<pcl::Correspondences> correspondences;

  boost::shared_ptr< DistanceMap<pcl::PointXYZRGBA> > dist_map;
  std::vector<Eigen::Vector4f> boundingPlanes;

  std::vector< std::vector<int> > dsMap;
  std::vector<int> reversedMap;

  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr search_tree;

  std::vector< float > symmetryScores;
  std::vector< float > occlusionScores;
  std::vector< float > cutScores;
  std::vector< float > symmetrySupportOverlapScores;

  std::vector< std::vector< float > > pointSymScores;
  std::vector< std::vector< float > > pointOcclusionScores;
  std::vector< std::vector< float > > pointPerpendicularScores;

  std::vector< std::vector< float > > fgWeights;
  std::vector< std::vector< float > > bgWeights;

  std::vector< std::vector<int> > dsSegmentIds;

  std::vector<int> filteredSegmentIds;
  std::vector<int> mergedSegmentIds;

  //outputs
  std::vector< std::vector<int> > segmentIds;

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

  bool isSetup;

public:
  BilateralSymmetrySegmenter();
  ~BilateralSymmetrySegmenter();

  void setInputClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                      pcl::PointCloud<pcl::Normal>::Ptr &normals);

  void setInputSymmetries(std::vector<BilateralSymmetry> &symmetries,
                          std::vector< std::vector<int> > &symmetrySupports);

  void setInputPlanes(std::vector<rs::Plane> &planes);

  void initialize(bool bilSymSeg_isDownsampled,
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
                  float overlap_threshold);

  bool segment();

  bool getSegmentIds(std::vector< std::vector<int> > &segmentIds);

protected:

  bool filter();

  bool merge();
};
