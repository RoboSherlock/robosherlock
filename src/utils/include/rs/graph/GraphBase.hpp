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

#ifndef __GRAPH_BASE_HPP__
#define __GRAPH_BASE_HPP__

#include <vector>
#include <rs/utils/output.h>

/** \brief This class is a data structure representing a undirected acyclic graph. Self
   * loops are not allowed.
   *
   * This class is templated on two parameters:
   * 1. <Vertex> is a struct used to store vertex infromation. It must have
   *    the following members:
   *      "neighbors"       stores the indices of adjacent vertices
   *      "neighbor_edges"  stores the indices of edges corresponding to
   *                        adjacent vertices
   * 2. <Edge> is a struct storing the edge information. It must have members
   * "v1" and "v2" that hold the indices of the two vertices forming
   * an edge.
   *
   * Internally graph information is stored in two vectors: a vector of
   * <Vertex> and a vector of <Edge>. Vertex and edge indices correspond to
   * their positions in the vectors.
   */

template<typename VertexT, typename EdgeT>
class GraphBase
{
public:
  /** \brief Empty constructor. */
  GraphBase() : list_vertex(0), list_edge(0) {}

  /** \brief Constructor that preallocates memory for vertices. */
  GraphBase(const int numVertices) : list_vertex(numVertices), list_edge(0) {}

  /** \brief Destructor. */
  ~GraphBase() {}

  /** \brief Add an edge to the graph if it doesn't exist already, otherwise it does nothing
   *  \param[in]  edge  edge
   *  \return false if graph is not consistent
   */
  inline bool addEdge(EdgeT &edge)
  {
    int v1_id = edge.v1;
    int v2_id = edge.v2;

    int v1_it, v2_it;
    if(!testConsistency(v1_id, v2_id, v1_it, v2_it))
    {
      return false;
    }

    if(v1_id == v2_id)
    {
      outInfo("No loop edge allowed!");
      return false;
    }

    if(v1_it == -1 && v2_it == -1)
    {
      int max_id = std::max(v1_id, v2_id);
      if(getNumVertices() <= max_id)
      {
        list_vertex.resize(max_id + 1);
      }

      list_vertex[v1_id].neighbors.push_back(v2_id);
      list_vertex[v1_id].neighbor_edges.push_back(list_edge.size());
      list_vertex[v2_id].neighbors.push_back(v1_id);
      list_vertex[v2_id].neighbor_edges.push_back(list_edge.size());

      list_edge.push_back(edge);
    }

    return true;
  }

  /** \brief Add an edge to the graph if it doesn't exist already, otherwise it does nothing
   *  \param[in] v1    index of first vertex
   *  \param[in] v2    index of second vertex
   *  \return false if graph is not consistent
   */
  inline bool addEdge(const int v1_id, const int v2_id)
  {
    EdgeT edge;
    edge.v1 = v1_id;
    edge.v2 = v2_id;
    return addEdge(edge);
  }

  /** \brief Remove all vertices and edges from the graph. */
  inline void clear()
  {
    list_vertex.clear();
    list_edge.clear();
  }

  /** \brief Clear graph and preallocate memory for vertices in the
   * adjacency list. Internally this function first clears the graph and
   * then resizes the adjacency list to the required size.
   *  \param[in]  num_vertices  number of vertices in the graph
   *  \note Existing graph data will be erased.
   */
  inline void setVertices(const int numVertices)
  {
    clear();
    list_vertex.resize(numVertices);
  }

  /** \brief Get number of vertices in the graph.  */
  inline int getNumVertices() const
  {
    return list_vertex.size();
  }

  /** \brief Get number of neighbors of a vertex. Return -1 if vertex with
   * requested id does not exist. */
  inline int getNumVertexNeighbors(const int v_id)
  {
    if(!testVertex(v_id))
    {
      outError("Vertex ID out of bound!");
      return -1;
    }
    return list_vertex[v_id].neighbors.size();
  }

  /** \brief Get number of edges in the graph.  */
  inline int getNumEdges() const
  {
    return list_edge.size();
  }

  /** \brief Get vertex at specified index.
   *  \param[in]  v_id    vertex index
   *  \param[out] vertex  vertex
   *  \return FALSE if vertex with requested id doesn't exist in the graph.
   */
  inline bool getVertex(const int v_id, VertexT &v)
  {
    if(!testVertex(v_id))
    {
      outError("Vertex ID out of bound!");
      return false;
    }
    else
    {
      v = list_vertex[v_id];
      return true;
    }
  }

  /** \brief Get index of the neighbor vertex at a specified position in the
   * neighbor list.
   *  \param[in]  v_id       vertex index
   *  \param[out] neighbors  a vector of neighbor of vertex index
   *  \return FALSE if vertex with requested id doesn't exist in the graph
   *          or doesn't have a neighbor at specified position.
   */
  inline bool getVertexNeighbors(const int v_id, std::vector<int> &neighbors)
  {
    VertexT v;
    bool valid = getVertex(v_id, v);

    if(valid)
    {
      neighbors = v.neighbors;
    }

    return valid;
  }

  /** \brief Get index of an edge between specified vertices.
   *  \param[in]  v1_id      index of first vertex
   *  \param[in]  v2_id      index of second vertex
   *  \param[out] edge_id edge index
   *  \return false if edge does not exist
   */
  inline bool getEdgeId(const int v1_id, const int v2_id, int &edge_id)
  {
    edge_id = -1;

    int v1_it, v2_it;
    if(!testConsistency(v1_id, v2_id, v1_it, v2_it))
    {
      return false;
    }

    if(v1_it == -1 && v2_it == -1)
    {
      return false;
    }

    edge_id = list_vertex[v1_id].neighbor_edges[v2_it];
    return true;
  }

  /** \brief Get edge at specified index.
   *  \param[in]  edge_id   edge index
   *  \param[out] edge      edge
   *  \return FALSE if edge with requested id doesn't exist in the graph.
   */
  inline bool getEdge(const int edge_id, EdgeT &edge)
  {
    if(!testEdge(edge_id))
    {
      outError("Edge ID is out of bound!");
      return false;
    }

    edge = list_edge[edge_id];
    return true;
  }

  /** \brief Get vertex indices from edge id
   *  \param[in]  edge_id   edge index
   *  \param[out] vtx1_id   index of first edge
   *  \param[out] vtx2_id   index of first edge
   *  \return FALSE if edge with requested id doesn't exist in the graph.
   */
  inline bool getVertexFromEdge(const int edge_id, int &v1_id, int &v2_id)
  {
    if(!testEdge(edge_id))
    {
      outError("Edge ID is out of bound!");
      return false;
    }
    v1_id = list_edge[edge_id].v1;
    v2_id = list_edge[edge_id].v2;

    return true;
  }

  /** \brief Vertex list. */
  std::vector<VertexT> list_vertex;

  /** \brief Edge list. */
  std::vector<EdgeT> list_edge;

protected:

  /** \brief Test if the input vertex index is not out of bound vector of vertices.
   *  \param[in] v_id  vertex index
   *  \return FALSE if vertex index is out of bound
   */
  inline bool testVertex(const int v_id)
  {
    return v_id >= 0 && v_id < list_vertex.size();
  }

  /** \brief Test if the input edge index is not out of bound vector of edges.
   *  \param[in] edge_id  edge index
   *  \return FALSE if edge index is out of bound
   */
  inline bool testEdge(const int edge_id)
  {
    return edge_id >= 0 && edge_id < list_edge.size();
  }

  /** \brief Test if the input edge (from vertices index) exists
   *  \param[in] v1_id  index of first vertex
   *  \param[in] v2_id  index of second vertex
   *  \return FALSE if edge index does not exist
   */
  inline bool testEdge(const int v1_id, const int v2_id)
  {
    for(size_t it = 0; it < list_edge.size(); it++)
    {
      if((list_edge[it].v1 == v1_id && list_edge[it].v2 == v2_id) || (list_edge[it].v1 == v2_id && list_edge[it].v2 == v1_id))
      {
        return true;
      }
    }
    return false;
  }

  /** \brief Test if it is able to get the two given vertices' positions in each other's neighbor
   * vectors. If the vertex isn't present in the other vertex's neighbor vector
   * it's position is returned as -1.
   * There are three cases:
   * 1. Both positions are non-negative
   *    This means that there is an edge between the vertices.
   * 2. Both positions are -1.
   *    This means that there is no edge between the vertices.
   * 3. Once position is -1 and other one is non-negative
   *    This indicates that the adjacency list storing the grapg is not
   *    consistent.
   *  \param[in]  v1_id  index of the first vertex
   *  \param[in]  v2_id  index of the second vertex
   *  \param[in]  v1_it  position of the first vertex in the second's vertex neigbhor list
   *  \param[in]  v2_it  position of the second vertex in the first's vertex neigbhor list
   *  \return FALSE if adjacency list is not consistent.
   */
  inline bool testConsistency  (const int v1_id, const int v2_id, int &v1_it, int &v2_it)
  {
    // Get vertex neighbor list positions
    v1_it = getVertexNeighborListPosition(v2_id, v1_id);
    v2_it = getVertexNeighborListPosition(v1_id, v2_id);

    // Check adjacency list for consistency (DEBUG)
    if ((v1_it == -1 && v2_it != -1) || (v1_it != -1 && v2_it == -1))
    {
      std::cout << "one vertex has another as it's member, but not the other way around. Adjacency list is not consistent!" << std::endl;
      std::cout << "(v1: " << v1_id << ", v2: " << v2_id << ")" << std::endl;
      return false;
    }

    // Check that both neighbors have the same edge id (DEBUG)
    if (( v1_it != -1 && v2_it != -1) &&
        ( list_vertex[v1_id].neighbor_edges[v2_it] != list_vertex[v2_id].neighbor_edges[v1_it]))
    {
      std::cout << "neighbors belonging to the same edge have different edge ids!" << std::endl;
      std::cout << "v1: " << v1_id << ", edge id: " << list_vertex[v1_id].neighbor_edges[v2_it] << ")" << std::endl;
      std::cout << "v2: " << v2_id << ", edge id: " << list_vertex[v2_id].neighbor_edges[v1_it] << ")" << std::endl;
      return false;
    }
    return true;
  }

  /** \brief Given a source vertex and a target vertex then find the position of
   * the target vertex in the source vertex's neighbor vector. If source
   * vertex does not exists or it's neighbor list does not contain the
   * target vertex - return -1.
   *  \param[in]  src_v_id      index of the first vertex
   *  \param[in]  tgt_v_id      index of the second vertex
   *  \return position of target vertex in source vertex's neighbor list
   */
  inline int getVertexNeighborListPosition(const int src_v_id, const int tgt_v_id)
  {
    int numNeighbors = getNumVertexNeighbors(src_v_id);

    if (numNeighbors == -1)
    {
      return -1;
    }

    for (size_t it = 0; it < numNeighbors; it++)
    {
      if (list_vertex[src_v_id].neighbors[it] == tgt_v_id)
      {
        return it;
      }
    }

    return -1;
  }
};

#endif // __GRAPH_BASE_HPP__
