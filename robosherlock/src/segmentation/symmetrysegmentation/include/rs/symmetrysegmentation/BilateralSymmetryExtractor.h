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
#include <pcl/common/io.h>
#include <pcl/common/pca.h>

//RS
#include <rs/types/all_types.h>

#include <rs/utils/array_utils.hpp>
#include <rs/symmetrysegmentation/BilateralSymmetry.hpp>
#include <rs/symmetrysegmentation/BilateralSymmetryScoring.hpp>
#include <rs/symmetrysegmentation/BoundarySegmentation.hpp>
#include <rs/mapping/DistanceMap.hpp>
#include <rs/mapping/DownsampleMap.hpp>
#include <rs/NonLinearOptimization/Functor.hpp>
#include <rs/graph/Graph.h>
#include <rs/graph/GraphAlgorithms.hpp>


class BilateralSymmetryExtractor
{
private:
  //inputs
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PointCloud<pcl::Normal>::Ptr normals;

  std::vector<pcl::PointIndices> segments; // over segments
  std::vector<rs::Plane> planes;

  //intermediate variables
  std::vector< std::vector<BilateralSymmetry> > segmentInitialSymmetries;
  std::vector< std::vector<BilateralSymmetry> > segmentRefinedSymmetries;

  std::vector< std::vector<int> > filteredSymmetryIds;
  std::vector< std::vector<float> > symSupportSizes;
  std::vector<Eigen::Vector3f> segment_centroids;

  std::vector< std::vector< std::vector<float> > > pointSymScores;
  std::vector< std::vector< std::vector<float> > > pointOcclusionScores;

  std::vector< std::vector<float> > occlusionScores;
  std::vector< std::vector<float> > segmentInlierScores;
  std::vector< std::vector<float> > corresInlierScores;

  std::vector< std::vector<bool> > validSymmetries;

  boost::shared_ptr< DistanceMap<pcl::PointXYZRGBA> > dist_map;
  std::vector<Eigen::Vector4f> boundingPlanes; // this plane will be extracted from PlaneAnnotator

  //outputs
  std::vector< BilateralSymmetry > finalSymmetries;
  std::vector< int > finalSupportSizeIds;

  //parameters
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

  bool isSetup;

public:

  BilateralSymmetryExtractor();
  ~BilateralSymmetryExtractor();

  void initialize(bool bilSymAnn_isDownsampled,
                  bool naive_detection,
                  float downsample_voxel_size,
                  int angle_division,
                  float dist_map_resolution,
                  float correspondence_search_radius,
                  float correspondence_max_normal_fit_error,
                  float correspondence_min_sym_dist,
                  float correspondence_max_sym_reflected_dist,
                  int refine_max_iteration,
                  float refine_min_inlier_sym_score,
                  float refine_max_inlier_sym_score,
                  float bilSymAnn_min_occlusion_dist,
                  float bilSymAnn_max_occlusion_dist,
                  float bilSymAnn_max_occlusion_score,
                  float min_segment_inlier_score,
                  float min_corres_inlier_score,
                  float sym_angle_diff,
                  float sym_dist_diff);

  void setInputClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                      pcl::PointCloud<pcl::Normal>::Ptr &normals);

  void setInputSegmentIds(std::vector<pcl::PointIndices> &segments);

  void setInputPlanes(std::vector<rs::Plane> &planes);

  bool extract();

  bool getSymmetries(std::vector<BilateralSymmetry> &symmetries);

  bool getSupportIds(std::vector< int > &supportIds);
protected:

  bool detectInitialSymmetries(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &segmentCloud,
                               int segmentId);

  bool refineBilateralSymmetryFitting(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &segmentCloud,
                                      pcl::PointCloud<pcl::Normal>::Ptr &segmentNormals,
                                      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &dsSegmentCloud,
                                      pcl::PointCloud<pcl::Normal>::Ptr &dsSegmentNormals,
                                      pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr &tree,
                                      int segmentId);

  bool filterSymmetries(int segmentId);

  void mergeSymmetries();
};
