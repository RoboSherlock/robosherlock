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

#ifndef __BOUNDARY_SEGMENTATION_HPP__
#define __BOUNDARY_SEGMENTATION_HPP__

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/boundary.h>
#include <pcl/common/io.h>
#include <pcl/search/impl/kdtree.hpp>

#include <robosherlock/utils/output.h>


/** \brief Extract boundary point of segment cloud
 *  \param[in]  cloud                   original cloud
 *  \param[in]  normals                 original cloud normals
 *  \param[in]  indices                 cloud indices
 *  \param[out] boundary_indices        boundary point indices
 *  \param[out] non_boundary_indices    non boundary point indices
 *  \param[in]  radiusSearch            search radius for finding neighbors
 *  \param[in]  differentAngleThreshold angle threshold to consider as boundary
 *  \return false if cannot extract any boundary point
 */
template <typename PointT, typename NormalT>
inline bool extractBoundaryCloud(
  typename pcl::PointCloud<PointT>::Ptr &cloud,
  typename pcl::PointCloud<NormalT>::Ptr &normals,
  std::vector<int> &indices,
  std::vector<int> &boundary_indices,
  std::vector<int> &non_boundary_indices,
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
  for(size_t it = 0; it < indices.size(); it++)
  {
    int pointId = indices[it];
    if(boundaries.points[pointId].boundary_point == 1)
    {
      boundary_indices.push_back(pointId);
    }
    else
    {
      non_boundary_indices.push_back(pointId);
    }
  }

  if(boundary_indices.size() <= 0)
  {
    outError("Could not extract boundary points!");
    return false;
  }
  return true;
}

/** \brief Extract boundary point of segment cloud
 *  \param[in]  cloud                   original cloud
 *  \param[in]  normals                 original cloud normals
 *  \param[out] boundary_indices        boundary point indices
 *  \param[out] non_boundary_indices    non boundary point indices
 *  \param[in]  radiusSearch            search radius for finding neighbors
 *  \param[in]  differentAngleThreshold angle threshold to consider as boundary
 *  \return false if cannot extract any boundary point
 */
template <typename PointT, typename NormalT>
inline bool extractBoundaryCloud(
  typename pcl::PointCloud<PointT>::Ptr &cloud,
  typename pcl::PointCloud<NormalT>::Ptr &normals,
  std::vector<int> &boundary_indices,
  std::vector<int> &non_boundary_indices,
  float radiusSearch = 0.01f,
  float differentAngleThreshold = 2.356f
)
{
  //create fake indices
  std::vector<int> indices(cloud->points.size());
  for(size_t it = 0; it < cloud->points.size();it++)
  {
    indices[it] = it;
  }

  return extractBoundaryCloud<PointT, NormalT>(cloud, normals, indices, boundary_indices, non_boundary_indices, radiusSearch, differentAngleThreshold);
}

/** \brief Extract boundary point of segment cloud
 *  \param[in]  cloud                   original cloud
 *  \param[in]  normals                 original cloud normals
 *  \param[in]  indices                 cloud indices
 *  \param[out] boundary_cloud          boundary cloud
 *  \param[out] non_boundary_cloud      non boundary cloud
 *  \param[in]  radiusSearch            search radius for finding neighbors
 *  \param[in]  differentAngleThreshold angle threshold to consider as boundary
 *  \return false if cannot extract any boundary point
 */
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
  if (success)
  {
    pcl::copyPointCloud(*cloud, boundary_indices, *boundary_cloud);
    pcl::copyPointCloud(*cloud, non_boundary_indices, *non_boundary_cloud);
  }

  return success;
}

#endif // __BOUNDARY_SEGMENTATION_HPP__
