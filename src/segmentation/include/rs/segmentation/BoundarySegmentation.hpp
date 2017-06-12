#ifndef BOUNDARY_SEGMENTATION_HPP
#define BOUNDARY_SEGMENTATION_HPP

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
  boundary_cloud->points.resize(0);
  non_boundary_cloud->points.resize(0);

  for(size_t it = 0; it < cloud->points.size(); it++){
    if(boundaries.points[it].boundary_point == 1)
        boundary_cloud->points.push_back(cloud->points[it]);
    else
        non_boundary_cloud->points.push_back(cloud->points[it]);
  }

  if(boundary_cloud->points.size() <= 0){
    outError("Could not extract boundary points!");
    return false;
  }
  else{
    return true;
  }
}


#endif
