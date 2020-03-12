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

#ifndef __DIRECTED_GRAPH_BASE_HPP__
#define __DIRECTED_GRAPH_BASE_HPP__

#include <mutex>
#include <thread>
#include <robosherlock/utils/output.h>

#include <algorithm>

template<typename VertexT, typename EdgeT>
class DirectedGraphBase
{
public:
  /** \brief Empty constructor. */
  DirectedGraphBase() : list_vertex(0), list_edge(0) {}

  /** \brief Constructor that preallocates memory for vertices. */
  DirectedGraphBase(const int numVertices) : list_vertex(numVertices), list_edge(0) {}

  /** \brief Destructor. */
  ~DirectedGraphBase() {}

  inline void clear()
  {
    list_vertex.clear();
    list_edge.clear();
  }

  inline void setVertices(const int numVertices)
  {
    std::lock_guard<std::mutex> lock(mutex);

    clear();
    list_vertex.resize(numVertices);
  }

  inline bool addEdge(EdgeT &edge)
  {
    std::lock_guard<std::mutex> lock(mutex);

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

      list_vertex[v1_id].children.push_back(v2_id);
      list_vertex[v1_id].out_edges.push_back(list_edge.size());
      list_vertex[v2_id].parents.push_back(v1_id);
      list_vertex[v2_id].in_edges.push_back(list_edge.size());

      list_edge.push_back(edge);
    }

    return true;
  }

  inline bool addEdge(const int v1_id, const int v2_id)
  {
    EdgeT edge;
    edge.v1 = v1_id;
    edge.v2 = v2_id;
    return addEdge(edge);
  }

  inline int getNumVertices() const
  {
    return list_vertex.size();
  }

  inline int getNumEdges() const
  {
    return list_edge.size();
  }

  inline int getNumVertexParents(const int v_id)
  {
    if(!testVertex(v_id))
    {
      outError("Vertex ID out of bound!");
      return -1;
    }

    return list_vertex[v_id].parents.size();
  }

  inline int getNumVertexChildren(const int v_id)
  {
    if(!testVertex(v_id))
    {
      outError("Vertex ID out of bound!");
      return -1;
    }

    return list_vertex[v_id].children.size();
  }

  inline bool getVertex(const int v_id, VertexT &v)
  {
    if(!testVertex(v_id))
    {
      outError("Vertex ID out of bound!");
      return false;
    }

    v = list_vertex[v_id];

    return true;
  }

  inline bool getVertexParents(const int v_id, std::vector<int> &parents)
  {
    VertexT v;
    bool valid = getVertex(v_id, v);

    if(valid)
    {
      parents = v.parents;
    }

    return valid;
  }

  inline bool getVertexChildren(const int v_id, std::vector<int> &children)
  {
    VertexT v;
    bool valid = getVertex(v_id, v);

    if(valid)
    {
      children = v.children;
    }

    return valid;
  }

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

    edge_id = list_vertex[v1_id].out_edges[v2_it];

    return true;
  }

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

  //for debug purpose
  inline void print()
  {
    std::cout << "\nDependencyGraph\nList Edge: \n";
    for(auto it = list_edge.begin(); it != list_edge.end(); it++)
    {
      it->print();
    }

    std::cout << "List Vertex: \n";
    for(int it = 0; it < list_vertex.size(); it++)
    {
      std::cout << "Vertex " << it << '\n';
      list_vertex[it].print();
    }
  }

  /** \brief Vertex list. */
  std::vector<VertexT> list_vertex;

  /** \brief Edge list. */
  std::vector<EdgeT> list_edge;

protected:
  // enable graph to work with multithreading
  std::mutex mutex;

  inline bool testVertex(const int v_id)
  {
    return v_id >= 0 && v_id < list_vertex.size();
  }

  inline bool testEdge(const int edge_id)
  {
    return edge_id >= 0 && edge_id < list_edge.size();
  }

  inline bool testEdge(const int v1_id, const int v2_id)
  {
    for(size_t it = 0; it < list_edge.size(); it++)
    {
      if(list_edge[it].v1 == v1_id && list_edge[it].v2 == v2_id)
      {
        return true;
      }
    }

    return false;
  }

  //we assume edge v1 --> v2
  inline bool testConsistency (const int v1_id, const int v2_id, int &v1_it, int &v2_it)
  {
    // Get vertex parent and child list positions
    v1_it = getVertexParentListPosition(v2_id, v1_id);
    v2_it = getVertexChildListPosition(v1_id, v2_id);

    // Check dependency list for consistency (DEBUG)
    if ((v1_it == -1 && v2_it != -1) || (v1_it != -1 && v2_it == -1))
    {
      std::cout << "one vertex has the other as child, but the other does not see the vertex as parent. Dependency list is not consistent!" << std::endl;
      std::cout << "(v1: " << v1_id << ", v2: " << v2_id << ")" << std::endl;
      return false;
    }

    // Check that parent children relationship have the same edge id (DEBUG)
    if (( v1_it != -1 && v2_it != -1) &&
        ( list_vertex[v1_id].out_edges[v2_it] != list_vertex[v2_id].in_edges[v1_it]))
    {
      std::cout << "parent children relationship belonging to the same edge have different edge ids!" << std::endl;
      std::cout << "v1: " << v1_id << ", out edge id: " << list_vertex[v1_id].out_edges[v2_it] << ")" << std::endl;
      std::cout << "v2: " << v2_id << ", in edge id: " << list_vertex[v2_id].in_edges[v1_it] << ")" << std::endl;
      return false;
    }
    return true;
  }

  inline int getVertexParentListPosition(const int src_v_id, const int tgt_v_id)
  {
    int numParents = getNumVertexParents(src_v_id);

    if (numParents == -1)
    {
      return -1;
    }

    for (size_t it = 0; it < numParents; it++)
    {
      if (list_vertex[src_v_id].parents[it] == tgt_v_id)
      {
        return it;
      }
    }

    return -1;
  }

  inline int getVertexChildListPosition(const int src_v_id, const int tgt_v_id)
  {
    int numChildren = getNumVertexChildren(src_v_id);

    if (numChildren == -1)
    {
      return -1;
    }

    for (size_t it = 0; it < numChildren; it++)
    {
      if (list_vertex[src_v_id].children[it] == tgt_v_id)
      {
        return it;
      }
    }

    return -1;
  }
};

#endif // __DIRECTED_GRAPH_BASE_HPP__
