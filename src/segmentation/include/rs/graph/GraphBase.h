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

#ifndef __GRAPH_BASE_H__
#define __GRAPH_BASE_H__

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

template<typename Vertex, typename Edge>
class GraphBase
{
public:
  /** \brief Empty constructor. */
  GraphBase();

  /** \brief Constructor that preallocates memory for vertices. */
  GraphBase(const int numVertices);

  /** \brief Destructor. */
  ~GraphBase();

  /** \brief Add an edge to the graph if it doesn't exist already, otherwise it does nothing
   *  \param[in]  edge  edge
   *  \return false if graph is not consistent
   */
  inline bool addEdge(Edge &edge);

  /** \brief Add an edge to the graph if it doesn't exist already, otherwise it does nothing
   *  \param[in] v1    index of first vertex
   *  \param[in] v2    index of second vertex
   *  \return false if graph is not consistent
   */
  inline bool addEdge(const int v1_id, const int v2_id);

  /** \brief Remove all vertices and edges from the graph. */
  inline void clear();

  /** \brief Clear graph and preallocate memory for vertices in the
   * adjacency list. Internally this function first clears the graph and
   * then resizes the adjacency list to the required size.
   *  \param[in]  num_vertices  number of vertices in the graph
   *  \note Existing graph data will be erased.
   */
  inline void setVertices(const int numVertices);

  /** \brief Get number of vertices in the graph.  */
  inline int getNumVertices() const;

  /** \brief Get number of neighbors of a vertex. Return -1 if vertex with
   * requested id does not exist. */
  inline int getNumVertexNeighbors(const int v_id);

  /** \brief Get number of edges in the graph.  */
  inline int getNumEdges() const;

  /** \brief Get vertex at specified index.
   *  \param[in]  v_id    vertex index
   *  \param[out] vertex  vertex
   *  \return FALSE if vertex with requested id doesn't exist in the graph.
   */
  inline bool getVertex(const int v_id, Vertex &v);

  /** \brief Get index of the neighbor vertex at a specified position in the
   * neighbor list.
   *  \param[in]  v_id       vertex index
   *  \param[out] neighbors  a vector of neighbor of vertex index
   *  \return FALSE if vertex with requested id doesn't exist in the graph
   *          or doesn't have a neighbor at specified position.
   */
  inline bool getVertexNeighbors(const int v_id, std::vector<int> &neighbors);

  /** \brief Get index of an edge between specified vertices.
   *  \param[in]  v1_id      index of first vertex
   *  \param[in]  v2_id      index of second vertex
   *  \param[out] edge_id edge index
   *  \return false if edge does not exist
   */
  inline bool getEdgeId(const int v1_id, const int v2_id, int &edge_id);

  /** \brief Get edge at specified index.
   *  \param[in]  edge_id   edge index
   *  \param[out] edge      edge
   *  \return FALSE if edge with requested id doesn't exist in the graph.
   */
  inline bool getEdge(const int edge_id, Edge &edge);

  /** \brief Get vertex indices from edge id
   *  \param[in]  edge_id   edge index
   *  \param[out] vtx1_id   index of first edge
   *  \param[out] vtx2_id   index of first edge
   *  \return FALSE if edge with requested id doesn't exist in the graph.
   */
  inline bool getVertexFromEdge(const int edge_id, int &v1_id, int &v2_id);

  /** \brief Vertex list. */
  std::vector<Vertex> list_vertex;

  /** \brief Edge list. */
  std::vector<Edge> list_edge;

protected:

  /** \brief Test if the input vertex index is not out of bound vector of vertices.
   *  \param[in] v_id  vertex index
   *  \return FALSE if vertex index is out of bound
   */
  inline bool testVertex(const int v_id);

  /** \brief Test if the input edge index is not out of bound vector of edges.
   *  \param[in] edge_id  edge index
   *  \return FALSE if edge index is out of bound
   */
  inline bool testEdge(const int edge_id);

  /** \brief Test if the input edge (from vertices index) exists
   *  \param[in] v1_id  index of first vertex
   *  \param[in] v2_id  index of second vertex
   *  \return FALSE if edge index does not exist
   */
  inline bool testEdge(const int v1_id, const int v2_id);

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
  inline bool testConsistency  (const int v1_id, const int v2_id, int &v1_it, int &v2_it);

  /** \brief Given a source vertex and a target vertex then find the position of
   * the target vertex in the source vertex's neighbor vector. If source
   * vertex does not exists or it's neighbor list does not contain the
   * target vertex - return -1.
   *  \param[in]  src_v_id      index of the first vertex
   *  \param[in]  tgt_v_id      index of the second vertex
   *  \return position of target vertex in source vertex's neighbor list
   */
  inline int getVertexNeighborListPosition(const int src_v_id, const int tgt_v_id);
};

#endif // __GRAPH_BASE_H__
