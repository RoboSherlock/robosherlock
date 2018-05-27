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

#endif // __GRAPH_ALGORITHMS_HPP__
