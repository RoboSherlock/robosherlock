#ifndef GRAPH_BASE_H
#define GRAPH_BASE_H

template<typename Vertex, typename Edge>
class GraphBase
{
public:
  //default constructor
  GraphBase();
  GraphBase(const int numVertices);
  ~GraphBase();

  inline bool addEdge(Edge& edge);
  inline bool addEdge(const int v1_id, const int v2_id);

  inline void clear();
  inline void setVertices(const int numVertices);

  inline int getNumVertices() const;
  inline int getNumVertexNeighbors(const int v_id);
  inline int getNumEdges() const;

  inline bool getVertex(const int v_id, Vertex& v);
  inline bool getVertexNeighbors(const int v_id, std::vector<int>& neighbors);
  inline bool getEdgeId(const int v1_id, const int v2_id, int& edge_id);
  inline bool getEdge(const int edge_id, Edge& edge);
  inline bool getVertexFromEdge(const int edge_id, int& v1_id, int& v2_id);

  std::vector<Vertex> list_vertex;

  std::vector<Edge> list_edge;

protected:

  inline bool testVertex(const int v_id);
  inline bool testEdge(const int edge_id);
  inline bool testEdge(const int v1_id, const int v2_id);
  inline bool testConsistency  (const int v1_id, const int v2_id, int &v1_it, int &v2_it);
  inline int getVertexNeighborListPosition(const int src_v_id, const int tgt_v_id);
};

#endif
