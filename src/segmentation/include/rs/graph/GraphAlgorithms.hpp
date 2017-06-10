#ifndef GRAPH_ALGORITHMS_HPP
#define GRAPH_ALGORITHMS_HPP

#include <rs/graph/GraphAlgorithms.h>
#include <vector>
#include <queue>

template<typename Vertex, typename Edge>
inline std::vector< std::vector<int> > extractConnectedComponents(GraphBase<Vertex, Edge>& graph){
  std::vector<bool> visited (graph.getNumVertices(), false);
  std::vector< std::vector<int> > connectedComponents;

  for(size_t it = 0; it < graph.getNumVertices(); it++){

    if(visited[it])
      continue;


    std::queue<int> frontier;
    frontier.push(it);
    visited[it] = true;
    std::vector<int> connectedComponent;
    //run breadth first search for each vertex
    while(!frontier.empty()){
      int curr_vertex_id = frontier.front();
      frontier.pop();
      connectedComponent.push_back(curr_vertex_id);

      std::vector<int> neighbors;
      graph.getVertexNeighbors(curr_vertex_id, neighbors);
      for(size_t nIt = 0 ; nIt < neighbors.size(); nIt ++){
        if(!visited[neighbors[nIt]]){
          frontier.push(neighbors[nIt]);
          visited[neighbors[nIt]] = true;
        }
      }
    }

    connectedComponents.push_back(connectedComponent);
  }

  return connectedComponents;
}

#endif
