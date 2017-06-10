#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <rs/graph/Graph.h>

Graph::Graph() : GraphBase<Vertex, Edge>() {}

Graph::Graph(const int numVertices) : GraphBase<Vertex, Edge>(numVertices) {}

Graph::~Graph() {}

#endif
