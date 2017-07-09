#ifndef GRAPH_PRIMITIVES_H
#define GRAPH_PRIMITIVES_H

#include <vector>
struct Vertex{
  std::vector<int> neighbors;
  std::vector<int> neighbor_edges;

  Vertex();
  void print() const;
};

struct Edge{
    int v1;
    int v2;

    Edge();
    Edge(const int v1, const int v2);
    void print() const;
};

struct WeightedEdge : public Edge
{
  float weight;

  WeightedEdge();
  WeightedEdge(const int v1, const int v2, const float weight);

  void print() const;
};

#endif
