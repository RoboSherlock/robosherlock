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
#include <rs/segmentation/RotationalSymmetry.hpp>
#include <rs/segmentation/RotationalSymmetryScoring.hpp>
#include <rs/segmentation/BoundarySegmentation.hpp>
#include <rs/mapping/DistanceMap.hpp>
#include <rs/NonLinearOptimization/Functor.hpp>
#include <rs/graph/Graph.h>
#include <rs/graph/GraphAlgorithms.hpp>

class RotationalSymmetryExtractor
{
private:
  //inputs
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PointCloud<pcl::Normal>::Ptr normals;

  std::vector<rs::Plane> planes;
  std::vector<pcl::PointIndices> segments;

  //intermediate variables
  std::vector< std::vector<RotationalSymmetry> > segmentInitialSymmetries;
  std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > segmentClouds;
  std::vector< pcl::PointCloud<pcl::Normal>::Ptr > segmentNormals;
  std::vector< std::vector<float> > symSupportSizes;
  std::vector<Eigen::Vector3f> segment_centroids;

  //container for refined symmetries
  std::vector< std::vector<RotationalSymmetry> > segmentRefinedSymmetries;

  //container for segment level score
  std::vector< std::vector<float> > segmentSymScores;
  std::vector< std::vector<float> > segmentOcclusionScores;
  std::vector< std::vector<float> > segmentPerpendicularScores;
  std::vector< std::vector<float> > segmentCoverageScores;

  //container for point level score
  std::vector< std::vector< std::vector<float> > > pointSymScores;
  std::vector< std::vector< std::vector<float> > > pointOcclusionScores;
  std::vector< std::vector< std::vector<float> > > pointPerpendicularScores;

  boost::shared_ptr< DistanceMap<pcl::PointXYZRGBA> > dist_map;
  std::vector<Eigen::Vector4f> boundingPlanes; // this plane will be extracted from PlaneAnnotator

  //container for filtered symmetries id
  std::vector< std::vector<int> > filteredSymmetries;
  std::vector< std::vector<int> > bestSymmetries;

  //outputs
  std::vector<RotationalSymmetry> finalSymmetries;

  //parameters
  int numSegments;

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

public:
  RotationalSymmetryExtractor();
  ~RotationalSymmetryExtractor();


  void initialize(float rotSymAnn_min_fit_angle,
                  float rotSymAnn_max_fit_angle,
                  float rotSymAnn_min_occlusion_dist,
                  float rotSymAnn_max_occlusion_dist,
                  float rotSymAnn_max_sym_score,
                  float rotSymAnn_max_occlusion_score,
                  float rotSymAnn_max_perpendicular_score,
                  float rotSymAnn_min_coverage_score,
                  float dist_map_resolution,
                  float boundaryRadiusSearch,
                  float boundaryAngleThreshold,
                  float max_angle_diff,
                  float max_dist_diff);

  void setInputClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                      pcl::PointCloud<pcl::Normal>::Ptr &normals);

  void setInputSegmentIds(std::vector<pcl::PointIndices> &segments);

  void setInputPlanes(std::vector<rs::Plane> &planes);

  bool extract();

  bool getSymmetries(std::vector<RotationalSymmetry> &finalSymmetries);

protected:

  bool detectInitialSymmetries(int segmentId);

  bool refineSymmtries(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &segmentCloud,
                       pcl::PointCloud<pcl::Normal>::Ptr &segmentNormals,
                       int segmentId);

  bool filterSymmetries(int segmentId);

  bool getBestSymmetryID(int segmentId);

  bool mergeSymmetries(std::vector<RotationalSymmetry> &symmetries,
                       std::vector<float> &symSupportSize,
                       std::vector<RotationalSymmetry> &mergedSymmetries);

};
