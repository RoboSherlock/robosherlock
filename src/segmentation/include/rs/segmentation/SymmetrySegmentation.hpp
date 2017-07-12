#ifndef SYMMETRY_SEGMENTATION_HPP
#define SYMMETRY_SEGMENTATION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rs/utils/output.h>

#include <rs/graph/WeightedGraph.hpp>
#include <rs/occupancy_map/PointCloudMap.hpp>
#include <rs/occupancy_map/DownsampleMap.hpp>

#include <rs/segmentation/Geometry.hpp>


#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>

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

static class BoykovMinCut{
  typedef boost::adjacency_list_traits < boost::vecS, boost::vecS, boost::directedS > Traits;
  typedef boost::adjacency_list < boost::vecS, boost::vecS, boost::directedS,
                                  boost::property < boost::vertex_name_t, std::string,
                                  boost::property < boost::vertex_index_t, long,
                                  boost::property < boost::vertex_color_t, boost::default_color_type,
                                  boost::property < boost::vertex_distance_t, long,
                                  boost::property < boost::vertex_predecessor_t, Traits::edge_descriptor > > > > >,

                                  boost::property < boost::edge_capacity_t, float,
                                  boost::property < boost::edge_residual_capacity_t, float,
                                  boost::property < boost::edge_reverse_t, Traits::edge_descriptor > > > > GraphBoost;
  typedef boost::property_map< GraphBoost, boost::edge_capacity_t >::type CapacityMap;
  typedef boost::property_map< GraphBoost, boost::edge_reverse_t >::type ReverseEdgeMap;
  typedef boost::property_map< GraphBoost, boost::vertex_color_t, boost::default_color_type >::type VertexColorMap;

  static bool boost_add_edge(Traits::vertex_desriptor& v1, Traits::vertex_desriptor& v2, GraphBoost& graph, const float weight, CapacityMap& capacity_map, ReverseEdgeMap& reverse_edge_map){
    Traits::edge_descriptor edge, reverse_edge;
    bool addedEdge, addedReverseEdge;

    boost::tie(edge, addedEdge) = boost::add_edge(v1, v2, graph);
    boost::tie(reverse_edge, addedReverseEdge) = boost::add_edge(v2, v1, graph);
    if( !addedEdge || !addedReverseEdge)
      return false;

    capacity_map[edge] = weight;
    capacity_map[reverse_edge] = weight;
    reverse_edge_map[edge] = reverse_edge;
    reverse_edge_map[reverse_edge] = edge;
  }

  static min_cut(const std::vector<float>& foreground_weights,
                 const std::vector<float>& background_weights,
                 const WeightedGraph& adjacency_weights,
                 std::vector<int>& foreground_ids,
                 std::vector<int>& background_ids)
  {
    if(! (foreground_weights.size() == background_weights.size()) && (foreground_weights.size() == adjacency_weights.getNumVertices()) ){
      outError("Input foreground_weights, background_weights and adjacency_weights is not consistent!");
      return false;
    }

    int numVertices = foreground_weights.size();
    GraphBoost graph;
    std::vector<Traits::vertex_desriptor> vertices;
    Traits::vertex_desriptor source;
    Traits::vertex_desriptor sink;
    CapacityMap capacity = boost::get(boost::edge_capacity, graph);
    ReverseEdgeMap reverse_edge_map = boost::get(boost::edge_reverse, graph);
    VertexColorMap VertexColorMap = boost::get(boost::vertex_color, graph);

    vertices.resize(numVertices + 2); // plus 2 of source and sink vertex
    for(size_t it = 0; it < static_cast<size_t>(numVertices + 2); it++)
      vertices[it] = boost::add_vertex(graph);

    source = vertices[numVertices];
    sink = vertices[numVertices + 1];

    for(size_t it = 0; it < static_cast<size_t>(numVertices); it++){
      boost_add_edge(vertices[it], source, graph, foreground_weights[it], capacity, reverse_edge_map);
      boost_add_edge(vertices[it], sink, graph, background_weights[it], capacity, reverse_edge_map);
    }

    for(size_t it = 0; it < adjacency_weights.getNumEdges(); it++){
      int v1_id, v2_id;
      float weight;

      if(!adjacency_weights.getEdge(edge_id, v1_id, v2_id, weight)){
        outError("Could not get edge from adjacency graph! Abort!");
        return false;
      }

      Traits::vertex_desriptor v1 = vertices[v1_id];
      Traits::vertex_desriptor v2 = vertices[v2_id];
      boost_add_edge(v1, v2, graph, weight, capacity, reverse_edge_map);
    }

    double max_flow = boost::boykov_kolmogorov_max_flow(graph, source, sink);

    foreground_ids.clear();
    background_ids.clear();

    for(size_t it = 0; it < static_cast<size_t>(numVertices); it++){
      if(VertexColorMap(vertices[it]) == 0)
        foreground_ids.push_back(it);
      else
        background_ids.push_back(it);
    }
  }
};

#endif
