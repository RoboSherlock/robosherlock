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

#ifndef __BILATERAL_SYMMETRY_SCORING_HPP__
#define __BILATERAL_SYMMETRY_SCORING_HPP__

#include <rs/utils/output.h>
#include <rs/mapping/DistanceMap.hpp>
#include <rs/utils/array_utils.hpp>
#include <rs/symmetrysegmentation/BilateralSymmetry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/** \brief Finding bilateral symmetric score of correspondences of original cloud and downsampled cloud.
 *  The symmetric score is computed based on angle difference of normal correspondences. It is then scaled between
 *  max and min inlier angle and clamped to 0.0f-1.0f. The lower score, the better.
 *  \param[in]  cloud                  original cloud
 *  \param[in]  normals                original cloud normals
 *  \param[in]  dsCloud                downsampled cloud
 *  \param[in]  dsNormals              downsampled cloud normals
 *  \param[in]  tree                   search tree of scene cloud
 *  \param[in]  symmetry               input symmetry
 *  \param[out] correspondences        symmetric correspondences
 *  \param[out] point_symmetry_scores  a vector to hold score for each point of correspondences
 *  \param[in]  search_radius          search radius to find neighbors
 *  \param[in]  max_normal_fit_error                      maximum error normal
 *  \param[in]  min_sym_corresspondence_dist              minimum distance between correspondences to be considered
 *  \param[in]  max_sym_corresspondence_reflected_dist    maximum distance between reflected point and original point to be considered
 *  \param[in]  min_inlier_normal_angle
 *  \param[in]  max_inlier_normal_angle
 *  \return false if cloud size or dsCloud size is zero
 */
template<typename PointT>
inline bool getCloudBilateralSymmetryScore(typename pcl::PointCloud<PointT>::Ptr &cloud,
                                           pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                           typename pcl::PointCloud<PointT>::Ptr &dsCloud,
                                           pcl::PointCloud<pcl::Normal>::Ptr &dsNormals,
                                           typename pcl::search::KdTree<PointT>::Ptr &tree,
                                           BilateralSymmetry &symmetry,
                                           pcl::Correspondences &correspondences,
                                           std::vector<float> &point_symmetry_scores,
                                           float search_radius = 0.01f,
                                           float max_normal_fit_error = 0.174f,
                                           float min_sym_corresspondence_dist = 0.02f,
                                           float max_sym_corresspondence_reflected_dist = 0.01f,
                                           float min_inlier_normal_angle = 0.174f,
                                           float max_inlier_normal_angle = 0.262f)
{
  if(cloud->size() == 0 || dsCloud->size() == 0)
  {
    outWarn("No point in cloud! Cloud need at least one point!");
    return false;
  }

  //NOTE: first approach
  /*bool success = true;
  if(correspondences.size() == 0)
  {
    success = findBilateralSymmetryCorrespondences<PointT>(cloud, normals, dsCloud, dsNormals, tree, symmetry, correspondences, NEAREST, search_radius, max_normal_fit_error, min_sym_corresspondence_dist, max_sym_corresspondence_reflected_dist);
  }

  point_symmetry_scores.resize(correspondences.size());
  if(success)
  {
    for(size_t corresId = 0; corresId < correspondences.size(); corresId++)
    {
      int queryId = correspondences[corresId].index_query;
      int matchId = correspondences[corresId].index_match;

      Eigen::Vector3f srcNormal(dsNormals->points[queryId].normal_x, dsNormals->points[queryId].normal_y, dsNormals->points[queryId].normal_z);
      Eigen::Vector3f tgtNormal(normals->points[matchId].normal_x, normals->points[matchId].normal_y, normals->points[matchId].normal_z);

      float score = symmetry.getBilSymNormalFitError(srcNormal, tgtNormal);
      score = (score - min_inlier_normal_angle) / (max_inlier_normal_angle - min_inlier_normal_angle);
      score = clamp(score, 0.0f, 1.0f);
      point_symmetry_scores[corresId] = score;
    }
  }

  return success;*/

  //NOTE: second approach
  point_symmetry_scores.clear();
  correspondences.clear();

  Eigen::Vector3f symOrigin = symmetry.getOrigin();
  Eigen::Vector3f symNormal = symmetry.getNormal();

  for(size_t pointId = 0; pointId < dsCloud->size();pointId++)
  {
    Eigen::Vector3f srcPoint = dsCloud->points[pointId].getVector3fMap();
    Eigen::Vector3f srcNormal(dsNormals->points[pointId].normal_x, dsNormals->points[pointId].normal_y, dsNormals->points[pointId].normal_z);

    Eigen::Vector3f reflectedSrcPoint = symmetry.reflectPoint(srcPoint);
    Eigen::Vector3f reflectedSrcNormal = symmetry.reflectNormal(srcNormal);

    std::vector<float> dists(1);
    std::vector<int> neighbors(1);
    PointT searchPoint;
    searchPoint.getVector3fMap() = reflectedSrcPoint;
    tree->nearestKSearch(searchPoint, 1, neighbors, dists);

    if(dists[0] <= max_sym_corresspondence_reflected_dist * max_sym_corresspondence_reflected_dist)
    {
      Eigen::Vector3f tgtPoint = cloud->points[neighbors[0]].getVector3fMap();
      Eigen::Vector3f tgtNormal(normals->points[neighbors[0]].normal_x, normals->points[neighbors[0]].normal_y, normals->points[neighbors[0]].normal_z);

      float score = symmetry.getBilSymNormalFitError(srcNormal, tgtNormal);

      if(score > M_PI * 3 / 4)
      {
        score = M_PI - score;
      }

      score = (score - min_inlier_normal_angle) / (max_inlier_normal_angle - min_inlier_normal_angle);
      score = clamp(score, 0.0f, 1.0f);
      correspondences.push_back(pcl::Correspondence(pointId, neighbors[0], dists[0]));
      point_symmetry_scores.push_back(score);
    }
  }

  return true;
}

/** \brief Finding bilateral occlusion score of each point of original cloud
 *  The occlusion score is computed based on the nearest occlusion distance of DistanceMap. It is then scaled between
 *  max and min occlusion dist and clamped to 0.0f-1.0f. The lower score, the better.
 *  \param[in]  cloud                   original cloud
 *  \param[in]  dist_map                distance map data structure of scene cloud
 *  \param[in]  symmetry                input symmetry
 *  \param[out] point_occlusion_scores  a vector to hold score for each point of cloud
 *  \param[in]  min_occlusion_dist
 *  \param[in]  max_occlusion_dist
 *  \return false if cloud size is zero
 */
template<typename PointT>
inline bool getCloudBilateralOcclusionScore(typename pcl::PointCloud<PointT>::Ptr &cloud,
                                            DistanceMap<PointT> &distMap,
                                            BilateralSymmetry &symmetry,
                                            std::vector<float> &point_occlusion_scores,
                                            float min_occlusion_dist = 0.01f,
                                            float max_occlusion_dist = 0.05f)
{
  if(cloud->size() == 0)
  {
    outWarn("No point in cloud! Cloud need at least one point!");
    return false;
  }

  point_occlusion_scores.resize(cloud->size());

  for(size_t pointId = 0; pointId < cloud->size(); pointId++)
  {
    Eigen::Vector3f srcPoint = cloud->points[pointId].getVector3fMap();
    Eigen::Vector3f reflectedSrcPoint = symmetry.reflectPoint(srcPoint);

    PointT searchPoint;
    searchPoint.getVector3fMap() = reflectedSrcPoint;

    int index;
    float dist;
    distMap.getNearestOccupiedDistance(searchPoint, index, dist);
    float score = (dist - min_occlusion_dist) / (max_occlusion_dist - min_occlusion_dist);
    score = clamp(score, 0.0f, 1.0f);
    point_occlusion_scores[pointId] = score;
  }

  return true;
}

/** \brief Finding bilateral perpendicular score of each point of original cloud
 *  The perpendicular score is computed based on the angle difference between each point normal and symmetry normal. It is then scaled between
 *  max and min perpendicular angle and clamped to 0.0f-1.0f. The lower score, the better.
 *  \param[in]  normals                     original normals
 *  \param[in]  symmetry                    input symmetry
 *  \param[out] point_perpendicular_scores  a vector to hold score for each point of cloud
 *  \param[in]  min_perpendicular_angle
 *  \param[in]  max_perpendicular_angle
 *  \return false if normals size is zero
 */
inline bool getCloudBilateralPerpendicularScore(pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                                BilateralSymmetry &symmetry,
                                                std::vector<float> &point_perpendicular_scores,
                                                float min_perpendicular_angle = 0.785f,
                                                float max_perpendicular_angle = 1.396f)
{
  if(normals->size() == 0)
  {
    outWarn("No point in cloud! Cloud need at least one point!");
    return false;
  }

  point_perpendicular_scores.resize(normals->size());

  for(size_t pointId = 0; pointId < normals->size(); pointId++)
  {
    Eigen::Vector3f normal(normals->points[pointId].normal_x, normals->points[pointId].normal_y, normals->points[pointId].normal_z);
    float angle = lineLineAngle<float>(normal, symmetry.getNormal());
    angle = (angle - min_perpendicular_angle) / (max_perpendicular_angle - min_perpendicular_angle);
    angle = clamp(angle, 0.0f, 1.0f);
    point_perpendicular_scores[pointId] = angle;
  }
  return true;
}

#endif // __BILATERAL_SYMMETRY_SCORING_HPP__
