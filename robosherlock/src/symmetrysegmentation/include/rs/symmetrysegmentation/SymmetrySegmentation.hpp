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

#ifndef __SYMMETRY_SEGMENTATION_HPP__
#define __SYMMETRY_SEGMENTATION_HPP__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rs/utils/output.h>

#include <rs/graph/WeightedGraph.h>
#include <rs/mapping/PointCloudMap.hpp>
#include <rs/mapping/DownsampleMap.hpp>

#include <rs/symmetrysegmentation/Geometry.hpp>
#include <rs/utils/array_utils.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>

/** \brief This function computes weighted adjacency graph from scene cloud
 *  The method is based on this paper: http://www.umiacs.umd.edu/~aecins/projects/symseg/symmetry_segmentation_ICRA16.pdf
 *  \param[in]  cloud                  original cloud
 *  \param[in]  normals                original cloud normals
 *  \param[in]  radius                 search radius for finding neighbors
 *  \param[in]  numNeighbors           number of nearest neighbors
 *  \param[out] graph                  adjacency graph
 *  \param[in]  scale_factor           scaling weight in graph
 *  \param[in]  sigmaConvex
 *  \param[in]  sigmaConcave
 *  \return false if any edge operation is error
 */
template<typename PointT>
inline bool computeCloudAdjacencyWeight(typename pcl::PointCloud<PointT>::Ptr &cloud,
                                        pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                        const float radius,
                                        const int numNeighbors,
                                        WeightedGraph &graph,
                                        const float scale_factor = 1.0f,
                                        const float sigmaConvex = 2.0f,
                                        const float sigmaConcave = 0.15f)
{
  graph.clear();

  computeCloudMap<PointT, Vertex, WeightedEdge>(cloud, graph, radius, numNeighbors);

  for(size_t edgeId = 0; edgeId < graph.getNumEdges(); edgeId++)
  {
    float weight;
    int v1_id, v2_id;

    if(!graph.getEdge(edgeId, v1_id, v2_id, weight))
    {
      outWarn("EdgeID: " << edgeId << " not found, Cloud map is corrupted!");
      return false;
    }

    Eigen::Vector3f p1 = cloud->points[v1_id].getVector3fMap();
    Eigen::Vector3f p2 = cloud->points[v2_id].getVector3fMap();
    Eigen::Vector3f n1(normals->points[v1_id].normal_x, normals->points[v1_id].normal_y, normals->points[v1_id].normal_z);
    Eigen::Vector3f n2(normals->points[v2_id].normal_x, normals->points[v2_id].normal_y, normals->points[v2_id].normal_z);

    float angle = clamp(n1.dot(n2), -1.0f, 1.0f);
    angle = 1.0f - angle;

    if(n1.dot(p1 - p2) > 0)
    {
      weight = std::exp(-angle / sigmaConvex) * scale_factor;
    }
    else
    {
      weight = std::exp(-angle / sigmaConcave) * scale_factor;
    }

    if(!graph.setEdgeWeight(edgeId, weight))
    {
      outWarn("EdgeID: " << edgeId << " not found, Cloud map is corrupted!");
      return false;
    }
  }

  return true;
}

/** \brief Class defining several boost class type necessary for boost::boykov_kolmogorov_max_flow segmentation
 *  It converts from this system graph type to boost graph type to perform segmentation.
 *  More info: http://www.boost.org/doc/libs/1_54_0/libs/graph/doc/boykov_kolmogorov_max_flow.html
 */
class BoykovMinCut
{
public:
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

  /** \brief Add edge on boost defined graph
   *  \param[in]  v1                  first vertex
   *  \param[in]  v2                  secon vertex
   *  \param[out]  graph              boost defined graph
   *  \param[in]  weight              weight of edge
   *  \param[out] capacity_map
   *  \param[out]  reverse_edge_map
   *  \return false if any edge operation is error
   */
  static bool boost_add_edge(Traits::vertex_descriptor &v1, Traits::vertex_descriptor &v2, GraphBoost &graph, const float weight, CapacityMap &capacity_map, ReverseEdgeMap &reverse_edge_map)
  {
    Traits::edge_descriptor edge, reverse_edge;
    bool addedEdge, addedReverseEdge;

    boost::tie(edge, addedEdge) = boost::add_edge(v1, v2, graph);
    boost::tie(reverse_edge, addedReverseEdge) = boost::add_edge(v2, v1, graph);
    if( !addedEdge || !addedReverseEdge)
    {
      return false;
    }

    capacity_map[edge] = weight;
    capacity_map[reverse_edge] = weight;
    reverse_edge_map[edge] = reverse_edge;
    reverse_edge_map[reverse_edge] = edge;

    return true;
  }

  /** \brief Perform min cut algorithm, this function segments foreground id and background id
   *  from adjacency graph and output min cut and max flow value
   *  \param[in]  foreground_weights
   *  \param[in]  background_weights
   *  \param[in]  adjacency_weights
   *  \param[out] foreground_ids
   *  \param[out] background_ids
   *  \param[out] min_cut_value
   *  \return max flow value
   */
  static float min_cut(const std::vector<float> &foreground_weights,
                 const std::vector<float> &background_weights,
                 WeightedGraph &adjacency_weights,
                 std::vector<int> &foreground_ids,
                 std::vector<int> &background_ids,
                 float &min_cut_value)
  {
    if(! (foreground_weights.size() == background_weights.size()) && (foreground_weights.size() == adjacency_weights.getNumVertices()) )
    {
      outWarn("Input foreground_weights, background_weights and adjacency_weights is not consistent!");
      return false;
    }

    int numVertices = foreground_weights.size();
    GraphBoost graph;
    std::vector<Traits::vertex_descriptor> vertices;
    Traits::vertex_descriptor source;
    Traits::vertex_descriptor sink;
    CapacityMap capacity = boost::get(boost::edge_capacity, graph);
    ReverseEdgeMap reverse_edge_map = boost::get(boost::edge_reverse, graph);
    VertexColorMap vertex_color_map = boost::get(boost::vertex_color, graph);

    vertices.resize(numVertices + 2); // plus 2 of source and sink vertex
    for(size_t it = 0; it < static_cast<size_t>(numVertices + 2); it++)
    {
      vertices[it] = boost::add_vertex(graph);
    }

    source = vertices[numVertices];
    sink = vertices[numVertices + 1];

    for(size_t it = 0; it < static_cast<size_t>(numVertices); it++)
    {
      boost_add_edge(vertices[it], source, graph, foreground_weights[it], capacity, reverse_edge_map);
      boost_add_edge(vertices[it], sink, graph, background_weights[it], capacity, reverse_edge_map);
    }

    for(size_t edge_id = 0; edge_id < adjacency_weights.getNumEdges(); edge_id++)
    {
      int v1_id, v2_id;
      float weight;

      if(!adjacency_weights.getEdge(edge_id, v1_id, v2_id, weight))
      {
        outError("Could not get edge from adjacency graph! Abort!");
        return false;
      }

      Traits::vertex_descriptor v1 = vertices[v1_id];
      Traits::vertex_descriptor v2 = vertices[v2_id];

      boost_add_edge(v1, v2, graph, weight, capacity, reverse_edge_map);
    }

    double max_flow = boost::boykov_kolmogorov_max_flow(graph, source, sink);

    foreground_ids.clear();
    background_ids.clear();

    for(size_t it = 0; it < static_cast<size_t>(numVertices); it++)
    {
      if(vertex_color_map(vertices[it]) == vertex_color_map(source))
      {
        foreground_ids.push_back(it);
      }
      else
      {
        background_ids.push_back(it);
      }
    }

    //get min cut routine
    min_cut_value = 0.0f;
    for(size_t edgeId = 0; edgeId < adjacency_weights.getNumEdges(); edgeId++)
    {
      int v1_id, v2_id;
      float weight;
      if(!adjacency_weights.getEdge(edgeId, v1_id, v2_id, weight))
      {
        outWarn("EdgeID: " << edgeId << " not found, Cloud map is corrupted!");
        return max_flow;
      }

      std::vector<int> foundIndices;
      int v1 = vectorSearch(foreground_ids, v1_id, foundIndices);
      int v2 = vectorSearch(foreground_ids, v2_id, foundIndices);

      if((v1 > 0 && v2 == 0) || (v1 == 0 && v2 > 0))
      {
        min_cut_value += weight;
      }
    }

    return max_flow;
  }
};

#endif // __SYMMETRY_SEGMENTATION_HPP__
