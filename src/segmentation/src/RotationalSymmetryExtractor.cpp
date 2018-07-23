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


#include <rs/segmentation/RotationalSymmetryExtractor.h>

RotationalSymmetryExtractor::RotationalSymmetryExtractor()
{
  this->cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
  this->normals.reset(new pcl::PointCloud<pcl::Normal>());
}

RotationalSymmetryExtractor::~RotationalSymmetryExtractor() {}

void RotationalSymmetryExtractor::setInputClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                                                 pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
  this->cloud = cloud;
  this->normals = normals;
}

void RotationalSymmetryExtractor::setInputSegmentIds(std::vector<pcl::PointIndices> &segments)
{
  this->segments = segments;
  this->numSegments = segments.size();
}

void RotationalSymmetryExtractor::setInputPlanes(std::vector<rs::Plane> &planes)
{
  this->planes = planes;
}

void RotationalSymmetryExtractor::initialize(float rotSymAnn_min_fit_angle,
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
                                             float max_dist_diff)
{
  this->rotSymAnn_min_fit_angle = rotSymAnn_min_fit_angle;
  this->rotSymAnn_max_fit_angle = rotSymAnn_max_fit_angle;
  this->rotSymAnn_min_occlusion_dist = rotSymAnn_min_occlusion_dist;
  this->rotSymAnn_max_occlusion_dist = rotSymAnn_max_occlusion_dist;
  this->rotSymAnn_max_sym_score = rotSymAnn_max_sym_score;
  this->rotSymAnn_max_occlusion_score = rotSymAnn_max_occlusion_score;
  this->rotSymAnn_max_perpendicular_score = rotSymAnn_max_perpendicular_score;
  this->rotSymAnn_min_coverage_score = rotSymAnn_min_coverage_score;
  this->dist_map_resolution = dist_map_resolution;
  this->boundaryRadiusSearch = boundaryRadiusSearch;
  this->boundaryAngleThreshold = boundaryAngleThreshold;
  this->max_angle_diff = max_angle_diff;
  this->max_dist_diff = max_dist_diff;
}


bool RotationalSymmetryExtractor::extract()
{
  if(segments.empty())
  {
    outError("Over Segments is empty! Extractor will not run!");
    return false;
  }

  if(this->cloud->empty() || this->normals->empty())
  {
    outError("Clouds is empty! Extractor will not run!");
    return false;
  }

  //clearing for consecutive frame
  segmentInitialSymmetries.clear();
  segmentRefinedSymmetries.clear();
  symSupportSizes.clear();
  segment_centroids.clear();
  segmentSymScores.clear();
  segmentOcclusionScores.clear();
  segmentPerpendicularScores.clear();
  segmentCoverageScores.clear();
  pointSymScores.clear();
  pointOcclusionScores.clear();
  pointPerpendicularScores.clear();
  filteredSymmetries.clear();
  bestSymmetries.clear();
  finalSymmetries.clear();
  boundingPlanes.clear();

  //allocating containers
  numSegments = segments.size();
  segmentInitialSymmetries.resize(numSegments);
  segmentRefinedSymmetries.resize(numSegments);
  symSupportSizes.resize(numSegments);
  segment_centroids.resize(numSegments);
  segmentSymScores.resize(numSegments);
  segmentOcclusionScores.resize(numSegments);
  segmentPerpendicularScores.resize(numSegments);
  segmentCoverageScores.resize(numSegments);
  pointSymScores.resize(numSegments);
  pointOcclusionScores.resize(numSegments);
  pointPerpendicularScores.resize(numSegments);
  filteredSymmetries.resize(numSegments);
  bestSymmetries.resize(numSegments);

  //get bounding planes
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
  dist_map->setInputCloud(cloud);

  //main execution
  #pragma omp parallel for
  for(size_t segmentId = 0; segmentId < numSegments; segmentId++)
  {
    //extract cloud segments
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmentCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr segmentNormals(new pcl::PointCloud<pcl::Normal>);

    pcl::copyPointCloud(*cloud, segments[segmentId], *segmentCloud);
    pcl::copyPointCloud(*normals, segments[segmentId], *segmentNormals);

    //extract cloud with no boundary
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr non_boundary_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr non_boundary_normal(new pcl::PointCloud<pcl::Normal>);
    std::vector<int> boundary_indices, non_boundary_indices;
    extractBoundaryCloud<pcl::PointXYZRGBA, pcl::Normal>(segmentCloud, segmentNormals, boundary_indices, non_boundary_indices, boundaryRadiusSearch, boundaryAngleThreshold);
    pcl::copyPointCloud(*segmentCloud, non_boundary_indices, *non_boundary_cloud);
    pcl::copyPointCloud(*segmentNormals, non_boundary_indices, *non_boundary_normal);

    //detect initial symmetries on each cloud segment using PCA, result 3 symmetries on 3 dimension
    this->detectInitialSymmetries(non_boundary_cloud,
                                  segmentId);
    //outInfo("Detect " << segmentInitialSymmetries[segmentId].size() << " symmetries in segmentID " << segmentId);

    //refind symmteries using LevenbergMarquardt algorithm (damped least squared fitting)
    this->refineSymmtries(non_boundary_cloud,
                          non_boundary_normal,
                          segmentId);
    //outInfo("Detect " << segmentRefinedSymmetries[segmentId].size() << " symmetries in segmentID " << segmentId);


    //filter out bad estimated symmetries by bounding scores
    this->filterSymmetries(segmentId);

    //get best representation symmetry of a segment based on occlusion scores
    this->getBestSymmetryID(segmentId);
  }

  //create linear container of symmetries for merging similar symmetries
  std::vector<RotationalSymmetry> unmergedSymmetries;
  std::vector<float> unmergedSupportSizes;
  linearizeSegmentData<RotationalSymmetry>(segmentRefinedSymmetries, unmergedSymmetries, bestSymmetries);
  linearizeSegmentData<float>(symSupportSizes, unmergedSupportSizes, bestSymmetries);

  this->mergeSymmetries(unmergedSymmetries, unmergedSupportSizes, finalSymmetries);

  return true;
}

bool RotationalSymmetryExtractor::getSymmetries(std::vector<RotationalSymmetry> &symmetries)
{
  if(this->finalSymmetries.empty())
  {
    outWarn("Symmetries are not computed! Have you run extract yet?");
    return false;
  }

  symmetries = this->finalSymmetries;
  return true;
}

bool RotationalSymmetryExtractor::detectInitialSymmetries(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &segmentCloud,
                                                          int segmentId)
{
  if(this->cloud->points.size() < 3)
  {
    outError("Segment has under 3 points. Symmetries will not calculated!");
    return false;
  }

  pcl::PCA<pcl::PointXYZRGBA> pca;
  pca.setInputCloud(segmentCloud);
  segment_centroids[segmentId] = pca.getMean().head(3);

  segmentInitialSymmetries[segmentId].resize(3);
  segmentInitialSymmetries[segmentId][0] = RotationalSymmetry(segment_centroids[segmentId], pca.getEigenVectors().col(0));
  segmentInitialSymmetries[segmentId][1] = RotationalSymmetry(segment_centroids[segmentId], pca.getEigenVectors().col(1));
  segmentInitialSymmetries[segmentId][2] = RotationalSymmetry(segment_centroids[segmentId], pca.getEigenVectors().col(2));

  return true;
}

bool RotationalSymmetryExtractor::refineSymmtries(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &segmentCloud,
                                                  pcl::PointCloud<pcl::Normal>::Ptr &segmentNormals,
                                                  int segmentId)
{
  int initialSymSize = segmentInitialSymmetries[segmentId].size();
  if(initialSymSize == 0 || segment_centroids[segmentId] == Eigen::Vector3f::Zero())
  {
    outWarn("Initial symmetries is empty! Have you run detectInitialSymmetries yet?");
    return false;
  }

  segmentRefinedSymmetries[segmentId].resize(initialSymSize);
  symSupportSizes[segmentId].resize(initialSymSize);
  segmentSymScores[segmentId].resize(initialSymSize);
  segmentOcclusionScores[segmentId].resize(initialSymSize);
  segmentPerpendicularScores[segmentId].resize(initialSymSize);
  segmentCoverageScores[segmentId].resize(initialSymSize);
  pointSymScores[segmentId].resize(initialSymSize);
  pointOcclusionScores[segmentId].resize(initialSymSize);
  pointPerpendicularScores[segmentId].resize(initialSymSize);

  for(size_t it = 0; it < initialSymSize; it++)
  {
    RotSymOptimizeFunctorDiff<pcl::PointXYZRGBA> functor;
    functor.cloud = segmentCloud;
    functor.normals = segmentNormals;
    functor.max_fit_angle = rotSymAnn_max_fit_angle;

    Eigen::LevenbergMarquardt<RotSymOptimizeFunctorDiff<pcl::PointXYZRGBA>, float> optimizer(functor);
    optimizer.parameters.ftol = 1e-10;
    optimizer.parameters.maxfev = 800;

    Eigen::VectorXf sym(6);
    sym.head(3) = segmentInitialSymmetries[segmentId][it].getOrigin();
    sym.tail(3) = segmentInitialSymmetries[segmentId][it].getOrientation();
    optimizer.minimize(sym);
    segmentRefinedSymmetries[segmentId][it] = RotationalSymmetry(sym.head(3), sym.tail(3));
    segmentRefinedSymmetries[segmentId][it].setProjectedOrigin(segment_centroids[segmentId]);

    symSupportSizes[segmentId][it] = static_cast<float>(cloud->points.size());

    segmentSymScores[segmentId][it] = getCloudRotationalSymmetryScore<pcl::PointXYZRGBA>(segmentCloud,
                                                                                         segmentNormals,
                                                                                         segmentRefinedSymmetries[segmentId][it],
                                                                                         pointSymScores[segmentId][it],
                                                                                         rotSymAnn_min_fit_angle,
                                                                                         rotSymAnn_max_fit_angle);

    segmentOcclusionScores[segmentId][it] = getCloudRotationalOcclusionScore<pcl::PointXYZRGBA>(segmentCloud,
                                                                                                *dist_map,
                                                                                                segmentRefinedSymmetries[segmentId][it],
                                                                                                pointOcclusionScores[segmentId][it],
                                                                                                rotSymAnn_min_occlusion_dist,
                                                                                                rotSymAnn_max_occlusion_dist);

    segmentPerpendicularScores[segmentId][it] = getCloudRotationalPerpendicularScore(segmentNormals,
                                                                                     segmentRefinedSymmetries[segmentId][it],
                                                                                     pointPerpendicularScores[segmentId][it]);

    segmentCoverageScores[segmentId][it] = getCloudRotationalCoverageScore<pcl::PointXYZRGBA>(cloud,
                                                                                              segmentRefinedSymmetries[segmentId][it]);
  }

  return true;
}

bool RotationalSymmetryExtractor::filterSymmetries(int segmentId)
{
  int symSize = segmentSymScores[segmentId].size();
  if(symSize == 0)
  {
    outWarn("SegmentID: " << segmentId << " has no symmetry!");
    return false;
  }

  for(size_t symId = 0; symId < symSize; symId++)
  {
    if(segmentSymScores[segmentId][symId] < rotSymAnn_max_sym_score &&
       segmentOcclusionScores[segmentId][symId] < rotSymAnn_max_occlusion_score &&
       segmentPerpendicularScores[segmentId][symId] < rotSymAnn_max_perpendicular_score &&
       segmentCoverageScores[segmentId][symId] > rotSymAnn_min_coverage_score)
    {
      filteredSymmetries[segmentId].push_back(symId);
    }
  }

  return true;
}


bool RotationalSymmetryExtractor::getBestSymmetryID(int segmentId)
{
  float bestScore = std::numeric_limits<float>::max(); // a.k.a min occlusionScores (consistent cloud)
  int bestSymId = -1;
  int symSize = filteredSymmetries[segmentId].size();
  /*if(symSize == 0)
  {
    outWarn("Symmetries is empty! Have you run refinedSymmetries yet?");
    return false;
  }*/

  for(size_t it = 0; it < symSize; it++)
  {
    int symId = filteredSymmetries[segmentId][it];
    if(segmentOcclusionScores[segmentId][symId] < bestScore)
    {
      bestSymId = symId;
      bestScore = segmentOcclusionScores[segmentId][symId];
    }
  }

  bestSymmetries[segmentId].push_back(bestSymId);

  return true;
}

bool RotationalSymmetryExtractor::mergeSymmetries(std::vector<RotationalSymmetry> &symmetries,
                                                  std::vector<float> &symSupportSize,
                                                  std::vector<RotationalSymmetry> &mergedSymmetries)
{
  mergedSymmetries.clear();

  Graph symGraph(symmetries.size());

  for(size_t srcId = 0; srcId < symmetries.size(); srcId++)
  {
    RotationalSymmetry &srcSym = symmetries[srcId];

    for(size_t tgtId = srcId+1; tgtId < symmetries.size();tgtId++)
    {
      RotationalSymmetry &tgtSym = symmetries[tgtId];

      float angle, dist;
      srcSym.getRotSymDifference(tgtSym, angle, dist);

      if( (angle < max_angle_diff || (M_PI - angle) < max_angle_diff) && dist < max_dist_diff)
      {
        symGraph.addEdge(srcId, tgtId);
      }
    }
  }

  std::vector< std::vector<int> > symConnectedComponents;
  symConnectedComponents = extractConnectedComponents(symGraph);

  for(size_t clusterId = 0; clusterId < symConnectedComponents.size(); clusterId++)
  {
    float maxSize = -1.0f;
    float bestSym = -1;
    for(size_t symIdIt = 0; symIdIt < symConnectedComponents[clusterId].size(); symIdIt++)
    {
      int symId = symConnectedComponents[clusterId][symIdIt];
      if(symSupportSize[symId] > maxSize)
      {
        maxSize = symSupportSize[symId];
        bestSym = symId;
      }
    }
    if(bestSym == -1)
    {
      outError("Could not merge similar rotational symmetries!");
      return false;
    }

    mergedSymmetries.push_back(symmetries[bestSym]);
  }
  return true;
}
