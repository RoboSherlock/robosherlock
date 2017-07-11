#ifndef POINT_CLOUD_MAP_HPP
#define POINT_CLOUD_MAP_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <rs/utils/output.h>

#include <rs/graph/GraphBase.hpp>
#include <rs/graph/GraphPrimitives.hpp>

template<typename PointT, typename VertexT, typename EdgeT>
inline bool computeCloudMap(typename pcl::PointCloud<PointT>::Ptr& cloud,
                            std::vector<int>& indices,
                            GraphBase<VertexT, EdgeT>& graph,
                            const float radius,
                            const int numNeighbors = 0)
{
  if(radius <= 0.0f){
    outError("Radius search must be positive!");
    return false;
  }

  graph.setVertices(cloud->points.size());

  pcl::search::KdTree<PointT> tree;
  tree.setInputCloud(cloud, boost::make_shared< std::vector<int> >(indices));

  for(size_t pointIdIt = 0; pointIdIt < indices.size(); pointIdIt++){
    int pointId = indices[pointIdIt];

    std::vector<int> neighborIndices;
    std::vector<float> dists;

    tree.radiusSearch(pointIdIt, radius, neighborIndices, dists, numNeighbors);

    for(size_t it = 0; it < neighborIndices.size();it++){
      if(pointId != neighborIndices[it])
        graph.addEdge(pointId, neighborIndices[it]);
    }
  }

  if(graph.getNumEdges() < 1){
    outInfo("Point cloud map has no edges!");
    return false;
  }
  return true;
}

template<typename PointT, typename VertexT, typename EdgeT>
inline bool computeCloudMap(typename pcl::PointCloud<PointT>::Ptr& cloud,
                            GraphBase<VertexT, EdgeT>& graph,
                            const float radius,
                            const int numNeighbors = 0)
{
  //fake indices
  std::vector<int> indices(cloud->points.size());
  for(size_t it = 0; it < cloud->points.size(); it++)
    indices[it] = it;

  return computeCloudMap<PointT, VertexT, EdgeT>(cloud, indices, graph, radius, numNeighbors);
}

#endif
