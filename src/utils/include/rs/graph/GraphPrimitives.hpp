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

#ifndef __GRAPH_PRIMITIVES_HPP__
#define __GRAPH_PRIMITIVES_HPP__

#include <rs/graph/GraphPrimitives.h>

#include <iostream>
Vertex::Vertex() : neighbors(0) , neighbor_edges(0) {}

void Vertex::print() const
{
  std::cout << "Neighbors: ";
  for(size_t it = 0 ; it < neighbors.size(); it++)
  {
    std::cout << neighbors[it] << ", ";
  }
  std::cout << '\n';

  std::cout << "Edges: ";
  for(size_t it = 0 ; it < neighbor_edges.size(); it++)
  {
    std::cout << neighbor_edges[it] << ", ";
  }
  std::cout << '\n';
}

DirectedVertex::DirectedVertex() : parents(0), children(0), in_edges(0), out_edges(0) {}

void DirectedVertex::print() const
{
  std::cout << "Parents: ";
  for(size_t it = 0 ; it < parents.size(); it++)
  {
    std::cout << parents[it] << ", ";
  }
  std::cout << '\n';

  std::cout << "Children: ";
  for(size_t it = 0 ; it < children.size(); it++)
  {
    std::cout << children[it] << ", ";
  }
  std::cout << '\n';

  std::cout << "In Edges: ";
  for(size_t it = 0 ; it < in_edges.size(); it++)
  {
    std::cout << in_edges[it] << ", ";
  }
  std::cout << '\n';

  std::cout << "Out Edges: ";
  for(size_t it = 0 ; it < out_edges.size(); it++)
  {
    std::cout << out_edges[it] << ", ";
  }
  std::cout << '\n';
}

Edge::Edge() : v1(0), v2(0) {}
Edge::Edge(const int v1, const int v2)
{
  this->v1 = v1;
  this->v2 = v2;
}

void Edge::print() const
{
  std::cout << v1 << " <-> " << v2 << '\n';
}

WeightedEdge::WeightedEdge() : Edge(), weight(0.0f) {}
WeightedEdge::WeightedEdge(const int v1, const int v2, const float weight) : Edge(v1, v2)
{
  this->weight = weight;
}

void WeightedEdge::print() const
{
  std::cout << v1 << "<-- " << weight << " -->" << v2 << '\n';
}

DirectedEdge::DirectedEdge() : Edge() {}
DirectedEdge::DirectedEdge(const int v1, const int v2) : Edge(v1, v2) {}

void DirectedEdge::print() const
{
  std::cout << v1 << " --> " << v2 << '\n';
}

#endif // __GRAPH_PRIMITIVES_HPP__
