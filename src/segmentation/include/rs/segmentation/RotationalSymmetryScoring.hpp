#ifndef ROTATIONAL_SYMMETRY_SCORING_HPP
#define ROTATIONAL_SYMMETRY_SCORING_HPP

#include <rs/utils/output.h>
#include <rs/occupancy_map/DistanceMap.hpp>
#include <rs/segmentation/array_utils.hpp>
#include <rs/segmentation/RotationalSymmetry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


template<typename PointT>
inline float getCloudSymmetryScore(typename pcl::PointCloud<PointT>::Ptr& cloud,
                                   pcl::PointCloud<pcl::Normal>::Ptr& normals,
                                   RotationalSymmetry& symmetry,
                                   std::vector<float>& point_symmetry_scores,
                                   float min_fit_angle = 0.0f,
                                   float max_fit_angle = M_PI / 2)
{
  point_symmetry_scores.resize(cloud->points.size());

  for(size_t it = 0; it < cloud->points.size(); it++){
    Eigen::Vector3f point =  cloud->points[it].getVector3fMap();
    Eigen::Vector3f normal( normals->points[it].data_c[0], normals->points[it].data_c[1], normals->points[it].data_c[2]);

    float angle = getRotSymFitError(point, normal, symmetry);
    float score = (angle - min_fit_angle) / (max_fit_angle - min_fit_angle);

    score = clamp(score, 0.0f, 1.0f);
    point_symmetry_scores[it] = score;
  }

  return mean(point_symmetry_scores);
}

template<typename PointT>
inline float getCloudOcclusionScore(typename pcl::PointCloud<PointT>::Ptr& cloud,
                                    DistanceMap<PointT>& dist_map,
                                    RotationalSymmetry& symmetry,
                                    std::vector<float>& point_occlusion_scores,
                                    float min_occlusion_dist = 0.0f,
                                    float max_occlusion_dist = 1.0f,
                                    int redundant_factor = 12)
{
  point_occlusion_scores.resize(cloud->points.size());

  typename pcl::PointCloud<PointT>::Ptr redundantCloud(new pcl::PointCloud<PointT>);
  symmetry.populateCloud(*cloud, *redundantCloud, 2.0f * M_PI / redundant_factor);

  for(size_t pId = 0; pId < cloud->points.size(); pId++){
    float maxDist = 0.0f;

    for(size_t redundantId = 0; redundantId < redundant_factor; redundantId++){
      int redundantPointId = (cloud->points.size() * redundantId) + pId;

      float currDist;
      int pIndex;
      dist_map.getNearestOccupiedDistance(redundantCloud->points[redundantPointId], pIndex, currDist);
      maxDist = std::max(maxDist, currDist);

      if(maxDist >= max_occlusion_dist)
        break;
    }

    float score = (maxDist - min_occlusion_dist) / (max_occlusion_dist - min_occlusion_dist);
    score = clamp(score, 0.0f, 1.0f);
    point_occlusion_scores[pId] = score;
  }

  return mean(point_occlusion_scores);
}

inline float getCloudPerpendicularScore(pcl::PointCloud<pcl::Normal>::Ptr& normals,
                                        RotationalSymmetry& symmetry,
                                        std::vector<float>& point_perpendicular_scores,
                                        float threshold = M_PI / 2)
{
  if(normals->points.size() == 0){
    outError("Normal cloud is null, this will return score -1!");
    return -1.0f;
  }

  point_perpendicular_scores.resize(normals->points.size());

  for(size_t it = 0; it < normals->points.size(); it++){
    Eigen::Vector3f normal(normals->points[it].data_c[0], normals->points[it].data_c[1], normals->points[it].data_c[2]);
    point_perpendicular_scores[it] = getRotSymPerpendicularity(normal, symmetry, threshold);
  }

  return mean(point_perpendicular_scores);
}

template<typename PointT>
inline float getCloudCoverageScore(typename pcl::PointCloud<PointT>::Ptr& cloud,
                                   RotationalSymmetry& symmetry
                                  )
{
  if(cloud->points.size() == 0){
    outError("Cloud is null, this will return score -1!");
    return -1.0f;
  }
  else if(cloud->points.size() == 1){
    outInfo("Cloud has only 1 point, return score 360 degree");
    return 0.0f;
  }

  Eigen::Vector3f referenceVec = cloud->points[0].getVector3fMap() - symmetry.projectPoint(cloud->points[0].getVector3fMap());
  std::vector<float> angles(cloud->points.size());
  angles[0] = 0.0f;

  for(size_t it = 1; it < cloud->points.size(); it++){
    Eigen::Vector3f currVec = cloud->points[it].getVector3fMap() - symmetry.projectPoint(cloud->points[it].getVector3fMap());
    angles[it] = vecVecAngleClockwise(referenceVec, currVec, symmetry.getOrientation());
  }

  std::sort(angles.begin(), angles.end());
  std::vector<float> angleDiffs(angles.size());
  for(size_t it = 1; it < angles.size(); it++)
    angleDiffs[it] = angleDifferent(angles[it-1], angles[it]);
  angleDiffs[0] = angleDifferent(angles[angles.size() - 1], angles[0]);

  return 1.0f - *std::max_element(angleDiffs.begin(), angleDiffs.end()) / (2.0f * M_PI);
}


#endif
