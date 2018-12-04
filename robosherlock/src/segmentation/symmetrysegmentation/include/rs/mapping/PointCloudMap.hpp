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

#ifndef __POINT_CLOUD_MAP_HPP__
#define __POINT_CLOUD_MAP_HPP__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <rs/utils/output.h>

#include <rs/graph/GraphBase.hpp>
#include <rs/graph/GraphPrimitives.h>

/** \brief Function compute a adjacency graph within radius or nearest neighbors from point cloud.
 *  \param[in]  cloud             input cloud
 *  \param[in]  indices           indices of input cloud
 *  \param[out] graph             undirected adjacency graph
 *  \param[in]  radius            search radius for neighbor points
 *  \param[in]  numNeighbors      number of nearest neighbors
 *  \return false if radius is negative
 */
template<typename PointT, typename VertexT, typename EdgeT>
inline bool computeCloudMap(typename pcl::PointCloud<PointT>::Ptr &cloud,
                            std::vector<int> &indices,
                            GraphBase<VertexT, EdgeT> &graph,
                            const float radius,
                            const int numNeighbors = 0)
{
  if(radius <= 0.0f)
  {
    outError("Radius search must be positive!");
    return false;
  }

  graph.setVertices(cloud->points.size());

  pcl::search::KdTree<PointT> tree;
  tree.setInputCloud(cloud, boost::make_shared< std::vector<int> >(indices));

  for(size_t pointIdIt = 0; pointIdIt < indices.size(); pointIdIt++)
  {
    int pointId = indices[pointIdIt];

    std::vector<int> neighborIndices;
    std::vector<float> dists;

    tree.radiusSearch(pointIdIt, radius, neighborIndices, dists, numNeighbors);

    for(size_t it = 0; it < neighborIndices.size();it++)
    {
      if(pointId != neighborIndices[it])
      {
        graph.addEdge(pointId, neighborIndices[it]);
      }
    }
  }

  if(graph.getNumEdges() < 1)
  {
    outInfo("Point cloud map has no edges!");
    return false;
  }

  return true;
}

/** \brief Function compute a adjacency graph within radius or nearest neighbors from point cloud.
 *  \param[in]  cloud             input cloud
 *  \param[out] graph             undirected adjacency graph
 *  \param[in]  radius            search radius for neighbor points
 *  \param[in]  numNeighbors      number of nearest neighbors
 *  \return false if radius is negative
 */
template<typename PointT, typename VertexT, typename EdgeT>
inline bool computeCloudMap(typename pcl::PointCloud<PointT>::Ptr &cloud,
                            GraphBase<VertexT, EdgeT> &graph,
                            const float radius,
                            const int numNeighbors = 0)
{
  //fake indices
  std::vector<int> indices(cloud->points.size());
  for(size_t it = 0; it < cloud->points.size(); it++)
  {
    indices[it] = it;
  }

  return computeCloudMap<PointT, VertexT, EdgeT>(cloud, indices, graph, radius, numNeighbors);
}

#endif // __POINT_CLOUD_MAP_HPP__
