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

#ifndef __WEIGHTED_GRAPH_H__
#define __WEIGHTED_GRAPH_H__

#include <robosherlock/graph/GraphBase.hpp>
#include <robosherlock/graph/GraphPrimitives.h>

/** \brief Data structure represents undirected and weighted graph.
 *  Self loop is not allowed.
 */
class WeightedGraph : public GraphBase<Vertex, WeightedEdge>
{
public:
  /** \brief Empty constructor. */
  WeightedGraph();

  /** \brief Constructor preallocates memory for vertices vector. */
  WeightedGraph(const int numVertices);

  /** \brief Destructor  */
  ~WeightedGraph();

  /** \brief Add an edge to the graph if it doesn't exist already, otherwise it does nothing
   *  \param[in] v1_id     index of first vertex
   *  \param[in] v2_id     index of second vertex
   *  \param[in] weight    edge weight
   *  \return false if graph is not consistent
   */
  bool addEdge(const int v1_id, const int v2_id, const float weight);

  /** \brief Get edge contained its vertex indices and its weight from its index.
   *  \param[in]  edge_id   index of edge
   *  \param[out] v1_id     index of first vertex
   *  \param[out] v2_id     index of second vertex
   *  \param[out] weight    edge weight
   *  \return false if edge is not found.
   */
  bool getEdge(const int edge_id, int& v1_id, int &v2_id, float &weight);

  /** \brief Get edge weight from the edge's vertex indices.
   *  \param[in] v1_id     index of first vertex
   *  \param[in] v2_id     index of second vertex
   *  \param[out] weight    edge weight
   *  \return false if edge is not found.
   */
  bool getEdgeWeight(const int v1_id, const int v2_id, float &weight);

  /** \brief Get edge weight from edge index.
   *  \param[in]  edge_id     edge index
   *  \param[out] weight      edge weight
   *  \return false if edge is not found.
   */
  bool getEdgeWeight(const int edge_id, float &weight);

  /** \brief Set edge weight from the edge's vertex indices.
   *  \param[in] v1_id     index of first vertex
   *  \param[in] v2_id     index of second vertex
   *  \param[in] weight    edge weight
   *  \return false if edge is not found.
   */
  bool setEdgeWeight(const int v1_id, const int v2_id, const float weight);

  /** \brief Set edge weight from edge index.
   *  \param[in]  edge_id     edge index
   *  \param[in] weight    edge weight
   *  \return false if edge is not found.
   */
  bool setEdgeWeight(const int edge_id, const float weight);
};


#endif // __WEIGHTED_GRAPH_H__
