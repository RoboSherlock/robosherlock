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

#include <rs/graph/WeightedGraph.h>
#include <rs/utils/output.h>

WeightedGraph::WeightedGraph() : GraphBase<Vertex, WeightedEdge>() {}

WeightedGraph::WeightedGraph(const int numVertices) : GraphBase<Vertex, WeightedEdge>(numVertices) {}

WeightedGraph::~WeightedGraph() {}

bool WeightedGraph::addEdge(const int v1_id, const int v2_id, const float weight)
{
  WeightedEdge edge(v1_id, v2_id, weight);
  return GraphBase<Vertex, WeightedEdge>::addEdge(edge);
}

bool WeightedGraph::getEdge(const int edge_id, int &v1_id, int &v2_id, float &weight)
{
  if(edge_id < 0 || edge_id >= list_edge.size())
  {
    return false;
  }

  WeightedEdge edge;

  bool found = GraphBase<Vertex, WeightedEdge>::getEdge(edge_id, edge);

  if(found)
  {
    v1_id = edge.v1;
    v2_id = edge.v2;
    weight = edge.weight;
  }

  return found;
}

bool WeightedGraph::getEdgeWeight(const int edge_id, float& weight)
{
  int v1, v2;
  return this->getEdge(edge_id, v1, v2, weight);
}

bool WeightedGraph::getEdgeWeight(const int v1_id, const int v2_id, float &weight)
{
  int edge_id;
  bool found = GraphBase<Vertex, WeightedEdge>::getEdgeId(v1_id, v2_id, edge_id);
  if(found)
  {
    found = this->getEdgeWeight(edge_id, weight);
  }
  else
  {
    outError("Edge with vertices: " << v1_id << " " << v2_id << " not found!");
  }
  return found;
}

bool WeightedGraph::setEdgeWeight(const int edge_id, const float weight)
{
  if(edge_id < 0 || edge_id >= list_edge.size())
  {
    return false;
  }

  list_edge[edge_id].weight = weight;
  return true;
}

bool WeightedGraph::setEdgeWeight(const int v1_id, const int v2_id, const float weight)
{
  int edge_id;
  bool found = GraphBase<Vertex, WeightedEdge>::getEdgeId(v1_id, v2_id, edge_id);
  if (found)
  {
    list_edge[edge_id].weight = weight;
  }

  return found;
}
