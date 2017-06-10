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
    void print() const;
};

#endif
