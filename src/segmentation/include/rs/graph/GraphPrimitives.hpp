#ifndef GRAPH_PRIMITIVES_HPP
#define GRAPH_PRIMITIVES_HPP

#include <rs/graph/GraphPrimitives.h>

#include <iostream>
Vertex::Vertex() : neighbors(0) , neighbor_edges(0) {}

void Vertex::print() const
{
  std::cout << "Neighbors: ";
  for(size_t it = 0 ; it < neighbors.size(); it++)
    std::cout << neighbors[it] << ", ";
  std::cout << '\n';

  std::cout << "Edges: ";
  for(size_t it = 0 ; it < neighbor_edges.size(); it++)
    std::cout << neighbor_edges[it] << ", ";
  std::cout << '\n';
}

Edge::Edge() : v1(0), v2(0) {}
Edge::Edge(const int v1, const int v2){
  this->v1 = v1;
  this->v2 = v2;
}

void Edge::print() const
{
  std::cout << v1 << " <-> " << v2 << '\n';
}

WeightedEdge::WeightedEdge() : Edge(), weight(0.0f) {}
WeightedEdge::WeightedEdge(const int v1, const int v2, const float weight) : Edge(v1, v2){
  this->weight = weight;
}

void WeightedEdge::print() const {
  std::cout << v1 << "<-- " << weight << " -->" << v2 << '\n';
}

#endif
