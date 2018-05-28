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

#ifndef __DIRECTED_GRAPH_BASE_H__
#define __DIRECTED_GRAPH_BASE_H__

#include <mutex>
#include <thread>

template<typename VertexT, typename EdgeT>
class DirectedGraphBase
{
public:
  /** \brief Empty constructor. */
  DirectedGraphBase();

  /** \brief Constructor that preallocates memory for vertices. */
  DirectedGraphBase(const int numVertices);

  /** \brief Destructor. */
  ~DirectedGraphBase();

  inline void clear();

  inline void setVertices(const int numVertices);

  inline bool addEdge(EdgeT &edge);

  inline bool addEdge(const int v1_id, const int v2_id);

  inline int getNumVertices() const;

  inline int getNumEdges() const;

  inline int getNumVertexParents(const int v_id);

  inline int getNumVertexChildren(const int v_id);

  inline bool getVertex(const int v_id, VertexT &v);

  inline bool getVertexParents(const int v_id, std::vector<int> &parents);

  inline bool getVertexChildren(const int v_id, std::vector<int> &children);

  inline bool getEdgeId(const int v1_id, const int v2_id, int &edge_id);

  inline bool getEdge(const int edge_id, EdgeT &edge);

  inline bool getVertexFromEdge(const int edge_id, int &v1_id, int &v2_id);

  //for debug purpose
  inline void print();

  /** \brief Vertex list. */
  std::vector<VertexT> list_vertex;

  /** \brief Edge list. */
  std::vector<EdgeT> list_edge;

protected:
  // enable graph to work with multithreading
  std::mutex mutex;

  inline bool testVertex(const int v_id);

  inline bool testEdge(const int edge_id);

  inline bool testEdge(const int v1_id, const int v2_id);

  inline bool testConsistency (const int v1_id, const int v2_id, int &v1_it, int &v2_it);

  inline int getVertexParentListPosition(const int src_v_id, const int tgt_v_id);

  inline int getVertexChildListPosition(const int src_v_id, const int tgt_v_id);
};

#endif // __DIRECTED_GRAPH_BASE_H__
