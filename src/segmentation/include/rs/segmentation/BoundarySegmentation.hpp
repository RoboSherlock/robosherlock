#ifndef BOUNDARY_SEGMENTATION_HPP
#define BOUNDARY_SEGMENTATION_HPP

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/boundary.h>
#include <pcl/common/io.h>
#include <pcl/search/impl/kdtree.hpp>

#include <rs/utils/output.h>

template <typename PointT, typename NormalT>
inline bool extractBoundaryCloud(
  typename pcl::PointCloud<PointT>::Ptr &cloud,
  typename pcl::PointCloud<NormalT>::Ptr &normals,
  std::vector<int>& indices,
  std::vector<int>& boundary_indices,
  std::vector<int>& non_boundary_indices,
  float radiusSearch = 0.01f,
  float differentAngleThreshold = 2.356f
)
{
  //initialize necessary instance
  pcl::PointCloud<pcl::Boundary> boundaries;
  pcl::BoundaryEstimation<PointT, NormalT, pcl::Boundary> be;
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());

  //setting up boundary estimator
  be.setInputCloud(cloud);
  be.setInputNormals(normals);
  be.setIndices(boost::make_shared< std::vector<int> >(indices));
  be.setRadiusSearch(radiusSearch);
  be.setAngleThreshold(differentAngleThreshold);
  be.setSearchMethod(tree);
  be.compute(boundaries);

  //extract boundary cloud
  for(size_t it = 0; it < indices.size(); it++){
    int pointId = indices[it];
    if(boundaries.points[pointId].boundary_point == 1)
        boundary_indices.push_back(pointId);
    else
        non_boundary_indices.push_back(pointId);
  }

  if(boundary_indices.size() <= 0){
    outError("Could not extract boundary points!");
    return false;
  }
  return true;
}

template <typename PointT, typename NormalT>
inline bool extractBoundaryCloud(
  typename pcl::PointCloud<PointT>::Ptr &cloud,
  typename pcl::PointCloud<NormalT>::Ptr &normals,
  std::vector<int>& boundary_indices,
  std::vector<int>& non_boundary_indices,
  float radiusSearch = 0.01f,
  float differentAngleThreshold = 2.356f
)
{
  //create fake indices
  std::vector<int> indices(cloud->points.size());
  for(size_t it = 0; it < cloud->points.size();it++){
    indices[it] = it;
  }
  return extractBoundaryCloud<PointT, NormalT>(cloud, normals, indices, boundary_indices, non_boundary_indices, radiusSearch, differentAngleThreshold);
}

template <typename PointT, typename NormalT>
inline bool extractBoundaryCloud(
  typename pcl::PointCloud<PointT>::Ptr &cloud,
  typename pcl::PointCloud<NormalT>::Ptr &normals,
  typename pcl::PointCloud<PointT>::Ptr &boundary_cloud,
  typename pcl::PointCloud<PointT>::Ptr &non_boundary_cloud,
  float radiusSearch = 0.01f,
  float differentAngleThreshold = 2.356f
)
{
  boundary_cloud.reset(new pcl::PointCloud<PointT>);
  non_boundary_cloud.reset(new pcl::PointCloud<PointT>);

  std::vector<int> boundary_indices;
  std::vector<int> non_boundary_indices;
  bool success = extractBoundaryCloud<PointT, NormalT>(cloud, normals, boundary_indices, non_boundary_indices, radiusSearch, differentAngleThreshold);
  if (success){
    pcl::copyPointCloud(*cloud, boundary_indices, *boundary_cloud);
    pcl::copyPointCloud(*cloud, non_boundary_indices, *non_boundary_cloud);
  }

  return success;
}

#endif