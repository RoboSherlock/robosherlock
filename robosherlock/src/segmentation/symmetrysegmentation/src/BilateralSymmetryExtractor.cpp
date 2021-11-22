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

#include <robosherlock/symmetrysegmentation/BilateralSymmetryExtractor.h>

BilateralSymmetryExtractor::BilateralSymmetryExtractor()
{
  this->cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
  this->normals.reset(new pcl::PointCloud<pcl::Normal>());

  this->isSetup = false;
}

BilateralSymmetryExtractor::~BilateralSymmetryExtractor() {}

void BilateralSymmetryExtractor::setInputClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                                                 pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
  this->cloud = cloud;
  this->normals = normals;
}

void BilateralSymmetryExtractor::setInputSegmentIds(std::vector<pcl::PointIndices> &segments)
{
  this->segments = segments;
  this->numSegments = segments.size();
}

void BilateralSymmetryExtractor::setInputPlanes(std::vector<rs::Plane> &planes)
{
  this->planes = planes;
}

void BilateralSymmetryExtractor::initialize(bool bilSymAnn_isDownsampled,
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
                                            float sym_dist_diff)
{
  this->bilSymAnn_isDownsampled = bilSymAnn_isDownsampled;
  this->naive_detection = naive_detection;
  this->downsample_voxel_size = downsample_voxel_size;
  this->angle_division = angle_division;
  this->dist_map_resolution = dist_map_resolution;
  this->correspondence_search_radius = correspondence_search_radius;
  this->correspondence_max_normal_fit_error = correspondence_max_normal_fit_error;
  this->correspondence_min_sym_dist = correspondence_min_sym_dist;
  this->correspondence_max_sym_reflected_dist = correspondence_max_sym_reflected_dist;
  this->refine_max_iteration = refine_max_iteration;
  this->refine_min_inlier_sym_score = refine_min_inlier_sym_score;
  this->refine_max_inlier_sym_score = refine_max_inlier_sym_score;
  this->bilSymAnn_min_occlusion_dist = bilSymAnn_min_occlusion_dist;
  this->bilSymAnn_max_occlusion_dist = bilSymAnn_max_occlusion_dist;
  this->bilSymAnn_max_occlusion_score = bilSymAnn_max_occlusion_score;
  this->min_segment_inlier_score = min_segment_inlier_score;
  this->min_corres_inlier_score = min_corres_inlier_score;
  this->sym_angle_diff = sym_angle_diff;
  this->sym_dist_diff = sym_dist_diff;

  this->isSetup = true;
}

bool BilateralSymmetryExtractor::extract()
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

  if(!this->isSetup)
  {
    outError("Parameters are not set! Abort.");
    return false;
  }

  //clearing for consecutive frame
  segmentInitialSymmetries.clear();
  segmentRefinedSymmetries.clear();
  finalSymmetries.clear();
  filteredSymmetryIds.clear();
  finalSupportSizeIds.clear();
  symSupportSizes.clear();
  segment_centroids.clear();
  pointSymScores.clear();
  pointOcclusionScores.clear();
  occlusionScores.clear();
  segmentInlierScores.clear();
  corresInlierScores.clear();
  validSymmetries.clear();

  segmentInitialSymmetries.resize(numSegments);
  segmentRefinedSymmetries.resize(numSegments);
  filteredSymmetryIds.resize(numSegments);
  symSupportSizes.resize(numSegments);
  segment_centroids.resize(numSegments);
  pointSymScores.resize(numSegments);
  pointOcclusionScores.resize(numSegments);
  occlusionScores.resize(numSegments);
  segmentInlierScores.resize(numSegments);
  corresInlierScores.resize(numSegments);
  validSymmetries.resize(numSegments);

  //get bounding planes
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
  dist_map->setInputCloud(cloud);

  #pragma omp parallel for
  for(size_t segmentId = 0; segmentId < numSegments; segmentId++){
    //extract cloud segments
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmentCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr segmentNormals(new pcl::PointCloud<pcl::Normal>);

    pcl::copyPointCloud(*cloud, segments[segmentId], *segmentCloud);
    pcl::copyPointCloud(*normals, segments[segmentId], *segmentNormals);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr dsSegmentCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr dsSegmentNormals(new pcl::PointCloud<pcl::Normal>);

    if(bilSymAnn_isDownsampled)
    {
      std::vector< std::vector<int> > dsMap;
      std::vector<int> nearestMap;
      DownsampleMap<pcl::PointXYZRGBA> ds;
      ds.setInputCloud(segmentCloud);
      ds.setLeafSize(downsample_voxel_size);
      ds.filter(*dsSegmentCloud);
      ds.getDownsampleMap(dsMap);
      ds.getNearestNeighborMap(nearestMap);

      computeDownsampleNormals(segmentNormals, dsMap, nearestMap, AVERAGE, dsSegmentNormals);
    }
    else
    {
      dsSegmentCloud = segmentCloud;
      dsSegmentNormals = segmentNormals;
    }

    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
    tree->setInputCloud(segmentCloud);

    if(!this->detectInitialSymmetries(segmentCloud, segmentId))
    {
      continue;
    }

    if(!refineBilateralSymmetryFitting(segmentCloud,
                                       segmentNormals,
                                       dsSegmentCloud,
                                       dsSegmentNormals,
                                       tree,
                                       segmentId))
    {
      continue;
    }

    //filter symmetries based on score and linearize data to an array
    this->filterSymmetries(segmentId);
  }

  this->mergeSymmetries();

  return true;
}

bool BilateralSymmetryExtractor::getSymmetries(std::vector<BilateralSymmetry> &symmetries)
{
  if(this->finalSymmetries.empty())
  {
    outWarn("Symmetries are not computed! Have you run extract yet?");
    return false;
  }

  symmetries = this->finalSymmetries;
  return true;
}

bool BilateralSymmetryExtractor::getSupportIds(std::vector< int > &supportIds)
{
  if(this->finalSupportSizeIds.empty())
  {
    outWarn("Symmetries are not computed! Have you run extract yet?");
    return false;
  }

  supportIds = finalSupportSizeIds;
  return true;
}

bool BilateralSymmetryExtractor::detectInitialSymmetries(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &segmentCloud,
                                                         int segmentId)
{
  if(segmentId < 0 || segmentId >= numSegments)
  {
    outError("Invalid segmentID: " << segmentId << " out of range!");
    return false;
  }

  if(segmentCloud->size() < 3)
  {
    outWarn("Cloud does not have sufficient points to detect symmetry!");
    return false;
  }

  pcl::PCA<pcl::PointXYZRGBA> pca;
  pca.setInputCloud(segmentCloud);
  segment_centroids[segmentId] = pca.getMean().head(3);
  Eigen::Matrix3f basis = pca.getEigenVectors();

  //ensure axes are right hand coordinate
  if(basis.col(0).cross(basis.col(1)).dot(basis.col(2)) < 0)
  {
    basis.col(2) *= -1.0f;
  }

  if(naive_detection)
  {
    segmentInitialSymmetries[segmentId].push_back(BilateralSymmetry(segment_centroids[segmentId], basis.col(1)));
    segmentInitialSymmetries[segmentId].push_back(BilateralSymmetry(segment_centroids[segmentId], basis.col(2)));
    segmentInitialSymmetries[segmentId].push_back(BilateralSymmetry(segment_centroids[segmentId], basis.col(0)));
  }
  else
  {
    std::vector<Eigen::Vector3f> points;
    generateHemisphere(angle_division, points);

    for(size_t pointId = 0; pointId < points.size(); pointId++)
    {
      segmentInitialSymmetries[segmentId].push_back(BilateralSymmetry(segment_centroids[segmentId], basis * points[pointId]));
    }
  }

  return true;
}

bool BilateralSymmetryExtractor::refineBilateralSymmetryFitting(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &segmentCloud,
                                                                pcl::PointCloud<pcl::Normal>::Ptr &segmentNormals,
                                                                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &dsSegmentCloud,
                                                                pcl::PointCloud<pcl::Normal>::Ptr &dsSegmentNormals,
                                                                pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr &tree,
                                                                int segmentId)
{
  if(segmentId < 0 || segmentId >= numSegments)
  {
    outError("Invalid segmentID: " << segmentId << " out of range!");
    return false;
  }

  if(segmentCloud->size() == 0 || dsSegmentCloud->size() == 0)
  {
    outWarn("No point in cloud! Cloud need at least one point!");
    return false;
  }

  int initialSymSize = segmentInitialSymmetries[segmentId].size();
  if(initialSymSize == 0 || segment_centroids[segmentId] == Eigen::Vector3f::Zero())
  {
    outWarn("Initial symmetries is empty! Have you run detectInitialSymmetries yet?");
    return false;
  }

  segmentRefinedSymmetries[segmentId].resize(initialSymSize);
  pointSymScores[segmentId].resize(initialSymSize);
  pointOcclusionScores[segmentId].resize(initialSymSize);
  occlusionScores[segmentId].resize(initialSymSize);
  segmentInlierScores[segmentId].resize(initialSymSize);
  corresInlierScores[segmentId].resize(initialSymSize);
  validSymmetries[segmentId].resize(initialSymSize, true);

  BilSymOptimizeFunctorDiff<pcl::PointXYZRGBA> functor;
  functor.cloud = segmentCloud;
  functor.normals = segmentNormals;
  functor.dsCloud = dsSegmentCloud;

  pcl::Correspondences correspondences;

  #pragma omp parallel for
  for(size_t symId = 0; symId < initialSymSize; symId++)
  {
    segmentRefinedSymmetries[segmentId][symId] = segmentInitialSymmetries[segmentId][symId];

    BilateralSymmetry last_symmetry;
    for(size_t iteration = 0; iteration < refine_max_iteration; iteration++)
    {
      last_symmetry = segmentRefinedSymmetries[segmentId][symId];

      correspondences.clear();

      //NOTE: first approach, finding correspondences based on error of reflected normal and src normal
      //bool success = findBilateralSymmetryCorrespondences<PointT>(cloud, normals, dsCloud, dsNormals, tree, refined_symmetries[symId], correspondences, NEIGHBOR_RADIUS, correspondence_search_radius, correspondence_max_normal_fit_error, correspondence_min_sym_dist, correspondence_max_sym_reflected_dist);

      //NOTE: second approach
      bool success = true;
      //finding correspondences
      Eigen::Vector3f symOrigin = segmentRefinedSymmetries[segmentId][symId].getOrigin();
      Eigen::Vector3f symNormal = segmentRefinedSymmetries[segmentId][symId].getNormal();

      for(size_t pointId = 0; pointId < dsSegmentCloud->size();pointId++)
      {
        Eigen::Vector3f srcPoint = dsSegmentCloud->points[pointId].getVector3fMap();
        Eigen::Vector3f srcNormal(dsSegmentNormals->points[pointId].normal_x, dsSegmentNormals->points[pointId].normal_y, dsSegmentNormals->points[pointId].normal_z);

        Eigen::Vector3f reflectedSrcPoint = segmentRefinedSymmetries[segmentId][symId].reflectPoint(srcPoint);
        Eigen::Vector3f reflectedSrcNormal = segmentRefinedSymmetries[segmentId][symId].reflectNormal(srcNormal);

        std::vector<float> dists(1);
        std::vector<int> neighbors(1);
        pcl::PointXYZRGBA searchPoint;
        searchPoint.getVector3fMap() = reflectedSrcPoint;
        tree->nearestKSearch(searchPoint, 1, neighbors, dists);

        Eigen::Vector3f tgtPoint = segmentCloud->points[neighbors[0]].getVector3fMap();
        Eigen::Vector3f tgtNormal(segmentNormals->points[neighbors[0]].normal_x, segmentNormals->points[neighbors[0]].normal_y, segmentNormals->points[neighbors[0]].normal_z);

        if(std::abs(segmentRefinedSymmetries[segmentId][symId].pointSignedDist(srcPoint) - segmentRefinedSymmetries[segmentId][symId].pointSignedDist(tgtPoint)) < correspondence_min_sym_dist)
        {
          continue;
        }

        if(dists[0] > correspondence_max_sym_reflected_dist * correspondence_max_sym_reflected_dist)
        {
          continue;
        }

        float normalError = segmentRefinedSymmetries[segmentId][symId].getBilSymNormalFitError(srcNormal, tgtNormal);

        if(normalError > correspondence_max_normal_fit_error)
        {
          continue;
        }

        correspondences.push_back(pcl::Correspondence(pointId, neighbors[0], dists[0]));

      }

      pcl::registration::CorrespondenceRejectorOneToOne correspRejectOneToOne;
      correspRejectOneToOne.getRemainingCorrespondences(correspondences, correspondences);

      if (correspondences.size() == 0)
      {
        success = false;
      }

      if(success)
      {
        Eigen::VectorXf x(6);
        x.head(3) = segmentRefinedSymmetries[segmentId][symId].getOrigin();
        x.tail(3) = segmentRefinedSymmetries[segmentId][symId].getNormal();

        functor.correspondences = correspondences;

        Eigen::LevenbergMarquardt< BilSymOptimizeFunctorDiff<pcl::PointXYZRGBA>, float> optimizer(functor);
        optimizer.minimize(x);

        segmentRefinedSymmetries[segmentId][symId] = BilateralSymmetry(x.head(3), x.tail(3));
        segmentRefinedSymmetries[segmentId][symId].setProjectedOrigin(segment_centroids[segmentId]);

        float angleDiff, distDiff;
        segmentRefinedSymmetries[segmentId][symId].bilateralSymDiff(last_symmetry, angleDiff, distDiff);
        if(angleDiff < 0.05f && distDiff < 0.0001f)
        {
          break;
        }
      }
      else
      {
        validSymmetries[segmentId][symId] = false;
        break;
      }
    }

    //compute symmetry score
    getCloudBilateralSymmetryScore<pcl::PointXYZRGBA>(segmentCloud, segmentNormals, dsSegmentCloud, dsSegmentNormals, tree, segmentRefinedSymmetries[segmentId][symId], correspondences, pointSymScores[segmentId][symId], correspondence_search_radius, correspondence_max_normal_fit_error, correspondence_min_sym_dist, correspondence_max_sym_reflected_dist, refine_min_inlier_sym_score, refine_max_inlier_sym_score);
    getCloudBilateralOcclusionScore<pcl::PointXYZRGBA>(dsSegmentCloud, *dist_map, segmentRefinedSymmetries[segmentId][symId], pointOcclusionScores[segmentId][symId], bilSymAnn_min_occlusion_dist, bilSymAnn_max_occlusion_dist);

    float inlierSum = 0.0f;
    for(size_t corresId = 0; corresId < correspondences.size(); corresId++)
    {
      inlierSum += (1.0f - pointSymScores[segmentId][symId][corresId]);
    }

    occlusionScores[segmentId][symId] = mean(pointOcclusionScores[segmentId][symId]);
    segmentInlierScores[segmentId][symId] = inlierSum / static_cast<float>(dsSegmentCloud->size());
    corresInlierScores[segmentId][symId] = inlierSum / static_cast<float>(correspondences.size());
  }

  return true;
}

bool BilateralSymmetryExtractor::filterSymmetries(int segmentId)
{
  if(segmentId < 0 || segmentId >= numSegments)
  {
    outError("Invalid segmentID: " << segmentId << " out of range!");
    return false;
  }

  if(segmentRefinedSymmetries[segmentId].empty())
  {
    outError("Refined symmetries at segmentID: " << segmentId << " is empty!");
    return false;
  }

  for(size_t symId = 0; symId < segmentRefinedSymmetries[segmentId].size(); symId++)
  {
    if(validSymmetries[segmentId][symId])
    {
      if(occlusionScores[segmentId][symId] < bilSymAnn_max_occlusion_score &&
         segmentInlierScores[segmentId][symId] > min_segment_inlier_score&&
         corresInlierScores[segmentId][symId] > min_corres_inlier_score)
      {
        filteredSymmetryIds[segmentId].push_back(symId);
      }
    }
  }

  return true;
}

void BilateralSymmetryExtractor::mergeSymmetries()
{
  std::vector<float> linear_occlusion_score;
  std::vector<BilateralSymmetry> linear_symmetries;
  std::vector<int> linear_support_size_ids;
  for(size_t segmentId = 0; segmentId < numSegments; segmentId++)
  {
    for(size_t symIdIt = 0; symIdIt < filteredSymmetryIds[segmentId].size(); symIdIt++)
    {
      int symId = filteredSymmetryIds[segmentId][symIdIt];
      linear_occlusion_score.push_back(occlusionScores[segmentId][symId]);
      linear_symmetries.push_back(segmentRefinedSymmetries[segmentId][symId]);
      linear_support_size_ids.push_back(segmentId);
    }
  }

  Graph symGraph(linear_symmetries.size());

  for(size_t srcId = 0; srcId < linear_symmetries.size(); srcId++)
  {
    BilateralSymmetry srcSym = linear_symmetries[srcId];

    for(size_t tgtId = srcId+1; tgtId < linear_symmetries.size(); tgtId++)
    {
      BilateralSymmetry tgtSym = linear_symmetries[tgtId];

      float angleDiff, distDiff;
      srcSym.bilateralSymDiff(tgtSym, angleDiff, distDiff);
      if(angleDiff < sym_angle_diff && distDiff < sym_dist_diff)
      {
        symGraph.addEdge(srcId, tgtId);
      }
    }
  }

  std::vector< std::vector<int> > symConnectedComponents;
  symConnectedComponents = extractConnectedComponents(symGraph);

  for(size_t clusterId = 0; clusterId < symConnectedComponents.size(); clusterId++)
  {
    float minScore = std::numeric_limits<float>::max();
    float bestSym = -1;
    for(size_t symIdIt = 0; symIdIt < symConnectedComponents[clusterId].size(); symIdIt++)
    {
      int symId = symConnectedComponents[clusterId][symIdIt];
      if(linear_occlusion_score[symId] < minScore)
      {
        minScore = linear_occlusion_score[symId];
        bestSym = symId;
      }
    }

    if(bestSym == -1)
    {
      outWarn("Could not merge similar bilateral symmetries!");
    }

    finalSupportSizeIds.push_back(linear_support_size_ids[bestSym]);
    finalSymmetries.push_back(linear_symmetries[bestSym]);
  }
}
