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

#ifndef __GRAPH_ALGORITHMS_HPP__
#define __GRAPH_ALGORITHMS_HPP__

#include <rs/graph/GraphAlgorithms.h>
#include <vector>
#include <queue>
#include <utility>
#include <algorithm>

template<typename VertexT, typename EdgeT>
inline std::vector< std::vector<int> > extractConnectedComponents(GraphBase<VertexT, EdgeT> &graph)
{
  std::vector<bool> visited (graph.getNumVertices(), false);
  std::vector< std::vector<int> > connectedComponents;

  for(size_t it = 0; it < graph.getNumVertices(); it++)
  {

    if(visited[it])
    {
      continue;
    }

    std::queue<int> frontier;
    frontier.push(it);
    visited[it] = true;
    std::vector<int> connectedComponent;
    //run breadth first search for each vertex
    while(!frontier.empty())
    {
      int curr_vertex_id = frontier.front();
      frontier.pop();
      connectedComponent.push_back(curr_vertex_id);

      std::vector<int> neighbors;
      graph.getVertexNeighbors(curr_vertex_id, neighbors);
      for(size_t nIt = 0 ; nIt < neighbors.size(); nIt ++)
      {
        if(!visited[neighbors[nIt]])
        {
          frontier.push(neighbors[nIt]);
          visited[neighbors[nIt]] = true;
        }
      }
    }

    connectedComponents.push_back(connectedComponent);
  }

  return connectedComponents;
}

template<typename VertexT, typename EdgeT>
inline bool planDependencyOrderings(DirectedGraphBase<VertexT, EdgeT> &graph, std::vector< std::vector<int> >& result)
{
  int nodeSize = graph.list_vertex.size();
  std::vector<int> nodeInputStates(nodeSize);
  std::vector<int> pathMap(nodeSize);
  std::vector<int> noParentNodes;

  result.clear();

  //NOTE: variable "it" is the nodeID
  //initialize nodeInputStates
  for(int it = 0; it < nodeSize; it++)
  {
    nodeInputStates[it] = graph.list_vertex[it].in_edges.size();
  }

  //insert no parents node to S
  for(int it = 0; it < nodeSize; it++)
  {
    if(nodeInputStates[it] == 0)
    {
      noParentNodes.push_back(it);
    }
  }

  while(true)
  {
    for(auto node_it = noParentNodes.begin(); node_it != noParentNodes.end(); node_it++)
    {
      VertexT& n = graph.list_vertex[*node_it];
      for(auto child_it = n.children.begin(); child_it != n.children.end(); child_it++)
      {
        try
        {
          //remove incoming edge of child node
          nodeInputStates[*child_it]--;
          int newOrder = pathMap[*node_it] + 1; // could replace 1 to weightEdge in the future
          if(pathMap[*child_it] < newOrder)
          {
            pathMap[*child_it] = newOrder;
          }
        }
        catch(std::exception& e)
        {
          outError("Invalid child nodeID of node " << *node_it << ". Graph is broken!");
          return false;
        }
      }
      //pseudo remove node
      nodeInputStates[*node_it] = -1;
    }

    noParentNodes.clear();
    //insert no parents node to S
    for(int it = 0; it < nodeSize; it++)
    {
      if(nodeInputStates[it] == 0)
      {
        noParentNodes.push_back(it);
      }
    }

    if(noParentNodes.empty())
    {
      for(auto it = nodeInputStates.begin(); it != nodeInputStates.end(); it++)
      {
        if(*it >= 0) // if graph still contains nodes
        {
          outError("Dependency graph may contain loop! Dependency orderings generation failed!");
          return false;
        }
      }

      break;
    }
  }

  //extract dependency orderings
  int maxOrder = *std::max_element(pathMap.begin(), pathMap.end()) + 1;
  result.resize(maxOrder);
  for(int it = 0; it < nodeSize; it++)
  {
    result[pathMap[it]].push_back(it);
  }

  return true;
}

#endif // __GRAPH_ALGORITHMS_HPP__
