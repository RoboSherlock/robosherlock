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

#ifndef __GRAPH_PRIMITIVES_H__
#define __GRAPH_PRIMITIVES_H__

#include <vector>

/** \brief Vertex struct, it contains all of its neighbor's indices. */
struct Vertex
{
  std::vector<int> neighbors;
  std::vector<int> neighbor_edges;

  Vertex();
  void print() const;
};

struct DirectedVertex
{
  std::vector<int> parents;
  std::vector<int> children;
  std::vector<int> in_edges;
  std::vector<int> out_edges;

  DirectedVertex();
  void print() const;
};

/** \brief Edge struct, it contains its vertex indices. */
struct Edge
{
    int v1;
    int v2;

    Edge();
    Edge(const int v1, const int v2);
    void print() const;
};

/** \brief WeightedEdge struct, it contains its vertex indices and a weight. */
struct WeightedEdge : public Edge
{
  float weight;

  WeightedEdge();
  WeightedEdge(const int v1, const int v2, const float weight);

  void print() const;
};

struct DirectedEdge : public Edge
{
  DirectedEdge();
  DirectedEdge(const int v1, const int v2);

  void print() const;
};

#endif // __GRAPH_PRIMITIVES_H__
