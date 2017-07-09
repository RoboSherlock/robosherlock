#ifndef WEIGHTED_GRAPH_H
#define WEIGHTED_GRAPH_H

#include <rs/graph/GraphBase.hpp>
#include <rs/graph/GraphPrimitives.hpp>

class WeightedGraph : public GraphBase<Vertex, WeightedEdge>
{
private:
public:
  WeightedGraph();
  WeightedGraph(const int numVertices);
  ~WeightedGraph();

  inline bool addEdge(const int v1_id, const int v2_id, const float weight);
  inline bool getEdge(const int edge_id, int& v1_id, int& v2_id, float& weight);

  inline bool getEdgeWeight(const int v1_id, const int v2_id, float& weight);
  inline bool getEdgeWeight(const int edge_id, float& weight);

  inline bool setEdgeWeight(const int v1_id, const int v2_id, const float weight);
  inline bool setEdgeWeight(const int edge_id, const float weight);
};


#endif
