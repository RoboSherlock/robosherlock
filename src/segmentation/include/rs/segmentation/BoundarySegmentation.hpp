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
  typename pcl::PointCloud<PointT>::Ptr &boundary_cloud,
  typename pcl::PointCloud<PointT>::Ptr &non_boundary_cloud,
  float radiusSearch,
  float differentAngleThreshold
)
{
  //initialize necessary instance
  pcl::PointCloud<pcl::Boundary> boundaries;
  pcl::BoundaryEstimation<PointT, NormalT, pcl::Boundary> be;
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());

  //setting up boundary estimator
  be.setInputCloud(cloud);
  be.setInputNormals(normals);
  be.setRadiusSearch(radiusSearch);
  be.setAngleThreshold(differentAngleThreshold);
  be.setSearchMethod(tree);
  be.compute(boundaries);

  //extract boundary cloud
  std::vector<int> boundary_indices;
  std::vector<int> non_boundary_indices;

  for(size_t it = 0; it < cloud->points.size(); it++){
    if(boundaries.points[it].boundary_point == 1)
        boundary_indices.push_back(it);
    else
        non_boundary_indices.push_back(it);
  }

  pcl::copyPointCloud(*cloud, boundary_indices, *boundary_cloud);
  pcl::copyPointCloud(*cloud, non_boundary_indices, *non_boundary_cloud);

  if(boundary_cloud->points.size() <= 0){
    outError("Could not extract boundary points!");
    return false;
  }
  else{
    return true;
  }
}

template <typename PointT, typename NormalT>
inline bool extractBoundaryCloud(
  typename pcl::PointCloud<PointT>::Ptr &cloud,
  typename pcl::PointCloud<NormalT>::Ptr &normals,
  std::vector<int>& boundary_indices,
  std::vector<int>& non_boundary_indices,
  float radiusSearch,
  float differentAngleThreshold
)
{
  //initialize necessary instance
  pcl::PointCloud<pcl::Boundary> boundaries;
  pcl::BoundaryEstimation<PointT, NormalT, pcl::Boundary> be;
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());

  //setting up boundary estimator
  be.setInputCloud(cloud);
  be.setInputNormals(normals);
  be.setRadiusSearch(radiusSearch);
  be.setAngleThreshold(differentAngleThreshold);
  be.setSearchMethod(tree);
  be.compute(boundaries);

  //extract boundary cloud
  for(size_t it = 0; it < cloud->points.size(); it++){
    if(boundaries.points[it].boundary_point == 1)
        boundary_indices.push_back(it);
    else
        non_boundary_indices.push_back(it);
  }

  if(boundary_indices.size() <= 0){
    outError("Could not extract boundary points!");
    return false;
  }
  else{
    return true;
  }
}


#endif
