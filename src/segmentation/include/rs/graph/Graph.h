#ifndef GRAPH_H
#define GRAPH_H

#include <rs/graph/GraphBase.hpp>
#include <rs/graph/GraphPrimitives.hpp>

class Graph : public GraphBase<Vertex, Edge>
{
public:
  Graph();
  Graph(const int numVertices);
  ~Graph();
};
#endif
