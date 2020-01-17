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

#ifndef __ROTATIONAL_SYMMETRY_SCORING_HPP__
#define __ROTATIONAL_SYMMETRY_SCORING_HPP__

#include <robosherlock/utils/output.h>
#include <robosherlock/mapping/DistanceMap.hpp>
#include <robosherlock/utils/array_utils.hpp>
#include <robosherlock/symmetrysegmentation/RotationalSymmetry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/** \brief Finding rotational symmetric score of each point of original cloud.
 *  The symmetric score is computed based on angle difference of point normal and symmetry axis. It is then scaled between
 *  max and min inlier angle and clamped to 0.0f-1.0f. The lower score, the better.
 *  \param[in]  cloud                  original cloud
 *  \param[in]  normals                original cloud normals
 *  \param[in]  symmetry               input symmetry
 *  \param[out] point_symmetry_scores  a vector to hold score for each point of correspondences
 *  \param[in]  min_fit_angle
 *  \param[in]  max_fit_angle
 *  \return mean values of point_symmetry_scores, represents segment symmetric score
 */
template<typename PointT>
inline float getCloudRotationalSymmetryScore(typename pcl::PointCloud<PointT>::Ptr &cloud,
                                   pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                   RotationalSymmetry &symmetry,
                                   std::vector<float> &point_symmetry_scores,
                                   float min_fit_angle = 0.0f,
                                   float max_fit_angle = M_PI / 2)
{
  point_symmetry_scores.resize(cloud->points.size());

  for(size_t it = 0; it < cloud->points.size(); it++)
  {
    Eigen::Vector3f point =  cloud->points[it].getVector3fMap();
    Eigen::Vector3f normal( normals->points[it].normal_x, normals->points[it].normal_y, normals->points[it].normal_z);

    float angle = getRotSymFitError(point, normal, symmetry);
    float score = (angle - min_fit_angle) / (max_fit_angle - min_fit_angle);

    score = clamp(score, 0.0f, 1.0f);
    point_symmetry_scores[it] = score;
  }

  return mean(point_symmetry_scores);
}

/** \brief Finding rotational occlusion score of each point of original cloud
 *  The occlusion score is computed based on the nearest occlusion distance of DistanceMap. It is then scaled between
 *  max and min occlusion dist and clamped to 0.0f-1.0f. The lower score, the better.
 *  \param[in]  cloud                   original cloud
 *  \param[in]  dist_map                distance map data structure of scene cloud
 *  \param[in]  symmetry                input symmetry
 *  \param[out] point_occlusion_scores  a vector to hold score for each point of cloud
 *  \param[in]  min_occlusion_dist
 *  \param[in]  max_occlusion_dist
 *  \param[in]  redundant_factor        factor to generate cloud rotationally
 *  \return mean values of point_occlusion_scores, represents segment occlusion score
 */
template<typename PointT>
inline float getCloudRotationalOcclusionScore(typename pcl::PointCloud<PointT>::Ptr &cloud,
                                    DistanceMap<PointT> &dist_map,
                                    RotationalSymmetry &symmetry,
                                    std::vector<float> &point_occlusion_scores,
                                    float min_occlusion_dist = 0.0f,
                                    float max_occlusion_dist = 1.0f,
                                    int redundant_factor = 12)
{
  point_occlusion_scores.resize(cloud->points.size());

  typename pcl::PointCloud<PointT>::Ptr redundantCloud(new pcl::PointCloud<PointT>);
  symmetry.populateCloud(*cloud, *redundantCloud, 2.0f * M_PI / redundant_factor);

  for(size_t pId = 0; pId < cloud->points.size(); pId++)
  {
    float maxDist = 0.0f;

    for(size_t redundantId = 0; redundantId < redundant_factor; redundantId++)
    {
      int redundantPointId = (cloud->points.size() * redundantId) + pId;

      float currDist;
      int pIndex;
      dist_map.getNearestOccupiedDistance(redundantCloud->points[redundantPointId], pIndex, currDist);
      maxDist = std::max(maxDist, currDist);

      if(maxDist >= max_occlusion_dist)
      {
        break;
      }
    }

    float score = (maxDist - min_occlusion_dist) / (max_occlusion_dist - min_occlusion_dist);
    score = clamp(score, 0.0f, 1.0f);
    point_occlusion_scores[pId] = score;
  }

  return mean(point_occlusion_scores);
}

/** \brief Finding rotational perpendicular score of each point of original cloud
 *  The perpendicular score is computed based on the angle difference between each point normal and symmetry axis. It is then scaled between
 *  max and min perpendicular angle and clamped to 0.0f-1.0f. The lower score, the better.
 *  \param[in]  normals                     original normals
 *  \param[in]  symmetry                    input symmetry
 *  \param[out] point_perpendicular_scores  a vector to hold score for each point of cloud
 *  \param[in]  threshold
 *  \return mean value of point_perpendicular_scores, represents segment perpendicular score
 */
inline float getCloudRotationalPerpendicularScore(pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                        RotationalSymmetry &symmetry,
                                        std::vector<float> &point_perpendicular_scores,
                                        float threshold = M_PI / 2)
{
  if(normals->points.size() == 0)
  {
    outError("Normal cloud is null, this will return score -1!");
    return -1.0f;
  }

  point_perpendicular_scores.resize(normals->points.size());

  for(size_t it = 0; it < normals->points.size(); it++)
  {
    Eigen::Vector3f normal(normals->points[it].normal_x, normals->points[it].normal_y, normals->points[it].normal_z);
    point_perpendicular_scores[it] = getRotSymPerpendicularity(normal, symmetry, threshold);
  }

  return mean(point_perpendicular_scores);
}

/** \brief Finding rotational coverage score of original cloud
 *  The coverage score is computed as max angle between any point in segment cloud, represents coverage of the cloud. It is then scaled between
 *  max and min perpendicular angle and clamped to 0.0f-1.0f. The lower score, the better.
 *  \param[in]  cloud                   original cloud
 *  \param[in]  symmetry                input symmetry
 *  \return max angle between any point in cloud
 */
template<typename PointT>
inline float getCloudRotationalCoverageScore(typename pcl::PointCloud<PointT>::Ptr &cloud,
                                             RotationalSymmetry &symmetry)
{
  if(cloud->points.size() == 0)
  {
    outError("Cloud is null, this will return score -1!");
    return -1.0f;
  }
  else if(cloud->points.size() == 1)
  {
    outInfo("Cloud has only 1 point, return score 360 degree");
    return 0.0f;
  }

  Eigen::Vector3f referenceVec = cloud->points[0].getVector3fMap() - symmetry.projectPoint(cloud->points[0].getVector3fMap());
  std::vector<float> angles(cloud->points.size());
  angles[0] = 0.0f;

  for(size_t it = 1; it < cloud->points.size(); it++)
  {
    Eigen::Vector3f currVec = cloud->points[it].getVector3fMap() - symmetry.projectPoint(cloud->points[it].getVector3fMap());
    angles[it] = vecVecAngleClockwise(referenceVec, currVec, symmetry.getOrientation());
  }

  std::sort(angles.begin(), angles.end());
  std::vector<float> angleDiffs(angles.size());
  for(size_t it = 1; it < angles.size(); it++)
  {
    angleDiffs[it] = angleDifferent(angles[it-1], angles[it]);
  }
  angleDiffs[0] = angleDifferent(angles[angles.size() - 1], angles[0]);

  return 1.0f - *std::max_element(angleDiffs.begin(), angleDiffs.end()) / (2.0f * M_PI);
}

#endif // __ROTATIONAL_SYMMETRY_SCORING_HPP__
