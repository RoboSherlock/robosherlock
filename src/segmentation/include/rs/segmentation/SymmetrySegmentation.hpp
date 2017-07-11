#ifndef SYMMETRY_SEGMENTATION_HPP
#define SYMMETRY_SEGMENTATION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rs/utils/output.h>

#include <rs/graph/WeightedGraph.hpp>
#include <rs/occupancy_map/PointCloudMap.hpp>
#include <rs/segmentation/Geometry.hpp>

//This function computes weights for smoothness term as in Symmetries Segmentation paper
template<typename PointT>
inline bool computeCloudAdjacencyWeight(typename pcl::PointCloud<PointT>::Ptr& cloud,
                                        pcl::PointCloud<pcl::Normal>::Ptr& normals,
                                        const float radius,
                                        const int numNeighbors,
                                        WeightedGraph& graph,
                                        const float scale_factor = 1.0f,
                                        const float sigmaConvex = 2.0f,
                                        const float sigmaConcave = 0.15f)
{
  graph.clear();

  computeCloudMap<PointT, Vertex, WeightedEdge>(cloud, graph, radius, numNeighbors);

  for(size_t edgeId = 0; edgeId < graph.getNumEdges(); edgeId++){
    float weight;
    int v1_id, v2_id;

    if(!graph.getEdge(edgeId, v1_id, v2_id, weight)){
      outError("EdgeID: " << edgeId << " not found, Cloud map is corrupted!");
      return false;
    }

    Eigen::Vector3f p1 = cloud->points[v1_id].getVector3fMap();
    Eigen::Vector3f p2 = cloud->points[v2_id].getVector3fMap();
    Eigen::Vector3f n1(normals->points[v1_id].normal_x, normals->points[v1_id].normal_y, normals->points[v1_id].normal_z);
    Eigen::Vector3f n2(normals->points[v2_id].normal_x, normals->points[v2_id].normal_y, normals->points[v2_id].normal_z);

    float angle = clamp(n1.dot(n2), -1.0f, 1.0f);
    angle = 1.0f - angle;

    if(n1.dot(p1 - p2) > 0)
      weight = std::exp(-angle / sigmaConvex) * scale_factor;
    else
      weight = std::exp(-angle / sigmaConcave) * scale_factor;

    if(!graph.setEdgeWeight(edgeId, weight)){
      outError("EdgeID: " << edgeId << " not found, Cloud map is corrupted!");
      return false;
    }
  }
  return true;
}

#endif
