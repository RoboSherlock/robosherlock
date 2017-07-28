#ifndef BILATERAL_SYMMETRY_SCORING_HPP
#define BILATERAL_SYMMETRY_SCORING_HPP

#include <rs/utils/output.h>
#include <rs/occupancy_map/DistanceMap.hpp>
#include <rs/segmentation/array_utils.hpp>
#include <rs/segmentation/BilateralSymmetry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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
                                           float max_sym_corresspondence_reflected_dist = 0.005f,
                                           float min_inlier_normal_angle = 0.174f,
                                           float max_inlier_normal_angle = 0.262f)
{
  if(cloud->size() == 0 || dsCloud->size() == 0)
  {
    outWarn("No point in cloud! Cloud need at least one point!");
    return false;
  }

  bool success = true;
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

  return success;
}

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


#endif
