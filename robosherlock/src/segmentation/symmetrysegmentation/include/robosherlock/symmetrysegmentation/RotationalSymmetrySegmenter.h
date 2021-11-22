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
#include <robosherlock/types/all_types.h>

#include <robosherlock/utils/array_utils.hpp>
#include <robosherlock/symmetrysegmentation/BoundarySegmentation.hpp>
#include <robosherlock/symmetrysegmentation/RotationalSymmetry.hpp>
#include <robosherlock/symmetrysegmentation/RotationalSymmetryScoring.hpp>
#include <robosherlock/symmetrysegmentation/SymmetrySegmentation.hpp>

#include <robosherlock/mapping/DistanceMap.hpp>
#include <robosherlock/mapping/DownsampleMap.hpp>

#include <robosherlock/graph/WeightedGraph.h>
#include <robosherlock/graph/Graph.h>
#include <robosherlock/graph/GraphAlgorithms.hpp>


class RotationalSymmetrySegmenter
{
private:
  //inputs
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sceneCloud;
  pcl::PointCloud<pcl::Normal>::Ptr sceneNormals;

  std::vector<RotationalSymmetry> symmetries;
  std::vector<rs::Plane> planes;

  //intermediate variables
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr dsSceneCloud;
  pcl::PointCloud<pcl::Normal>::Ptr dsSceneNormals;

  boost::shared_ptr< DistanceMap<pcl::PointXYZRGBA> > dist_map;
  std::vector<Eigen::Vector4f> boundingPlanes;

  WeightedGraph sceneGraph;

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

  std::vector<int> filteredSegmentIds;
  std::vector<int> mergedSegmentIds;

  //outputs
  std::vector< std::vector<int> > segmentIds;

  //parameters
  int numSymmetries;

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

  bool isSetup;

public:

  RotationalSymmetrySegmenter();
  ~RotationalSymmetrySegmenter();

  void setInputClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                      pcl::PointCloud<pcl::Normal>::Ptr &normals);

  void setInputSymmetries(std::vector<RotationalSymmetry> &symmetries);

  void setInputPlanes(std::vector<rs::Plane> &planes);

  void initialize(bool rotSymSeg_isDownsampled,
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
                  float overlap_threshold);

  bool segment();

  bool getSegmentIds(std::vector< std::vector<int> > &segmentIds);

protected:

  bool filter();

  bool merge();
};
