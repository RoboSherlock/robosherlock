#ifndef GRAPH_BASE_HPP
#define GRAPH_BASE_HPP

#include <rs/graph/GraphBase.h>
#include <rs/utils/output.h>

#include <algorithm>

template<typename Vertex, typename Edge>
GraphBase<Vertex, Edge>::GraphBase() : list_vertex(0), list_edge(0) {}

template<typename Vertex, typename Edge>
GraphBase<Vertex, Edge>::GraphBase(const int numVertices) : list_vertex(numVertices), list_edge(0) {}

template<typename Vertex, typename Edge>
GraphBase<Vertex, Edge>::~GraphBase() {}

template<typename Vertex, typename Edge>
inline bool GraphBase<Vertex, Edge>::addEdge(Edge& edge){
  int v1_id = edge.v1;
  int v2_id = edge.v2;

  int v1_it, v2_it;
  if(!testConsistency(v1_id, v2_id, v1_it, v2_it))
    return false;

  if(v1_id == v2_id){
    outInfo("No loop edge allowed!");
    return false;
  }

  if(v1_it == -1 && v2_it == -1){
    int max_id = std::max(v1_id, v2_id);
    if(getNumVertices() <= max_id)
      list_vertex.resize(max_id + 1);

    list_vertex[v1_id].neighbors.push_back(v2_id);
    list_vertex[v1_id].neighbor_edges.push_back(list_edge.size());
    list_vertex[v2_id].neighbors.push_back(v1_id);
    list_vertex[v2_id].neighbor_edges.push_back(list_edge.size());

    list_edge.push_back(edge);
  }

  return true;
}

template<typename Vertex, typename Edge>
inline bool GraphBase<Vertex, Edge>::addEdge(const int v1_id, const int v2_id){
  Edge edge;
  edge.v1 = v1_id;
  edge.v2 = v2_id;
  return addEdge(edge);
}

template<typename Vertex, typename Edge>
inline void GraphBase<Vertex, Edge>::clear(){
  list_vertex.resize(0);
  list_edge.resize(0);
}

template<typename Vertex, typename Edge>
inline void GraphBase<Vertex, Edge>::setVertices(const int numVertices){
  clear();
  list_vertex.resize();
}

template<typename Vertex, typename Edge>
inline int GraphBase<Vertex, Edge>::getNumVertices() const{
  return list_vertex.size();
}

template<typename Vertex, typename Edge>
inline int GraphBase<Vertex, Edge>::getNumVertexNeighbors(const int v_id){
  if(!testVertex(v_id)){
    outError("Vertex ID out of bound!");
    return -1;
  }
  return list_vertex[v_id].neighbors.size();
}

template<typename Vertex, typename Edge>
inline int GraphBase<Vertex, Edge>::getNumEdges() const{
  return list_edge.size();
}

template<typename Vertex, typename Edge>
inline bool GraphBase<Vertex, Edge>::getVertex(const int v_id, Vertex& v)
{
  if(!testVertex(v_id)){
    outError("Vertex ID out of bound!");
    return false;
  }
  else{
    v = list_vertex[v_id];
    return true;
  }
}

template<typename Vertex, typename Edge>
inline bool GraphBase<Vertex, Edge>::getVertexNeighbors(const int v_id, std::vector<int>& neighbors){
  Vertex v;
  bool valid = getVertex(v_id, v);

  if(valid){
    neighbors = v.neighbors;
  }

  return valid;
}

template<typename Vertex, typename Edge>
inline bool GraphBase<Vertex, Edge>::getEdgeId(const int v1_id, const int v2_id, int& edge_id){
  edge_id = -1;

  int v1_it, v2_it;
  if(!testConsistency(v1_id, v2_id, v1_it, v2_it))
    return false;

  if(v1_it == -1 && v2_it == -1)
    return false;

  edge_id = list_vertex[v1_id].neighbor_edges[v2_it];
  return true;
}

template<typename Vertex, typename Edge>
inline bool GraphBase<Vertex, Edge>::getEdge(const int edge_id, Edge& edge){
  if(!testEdge(edge_id)){
    outError("Edge ID is out of bound!");
    return false;
  }

  edge = list_edge[edge_id];
  return true;
}

template<typename Vertex, typename Edge>
inline bool GraphBase<Vertex, Edge>::getVertexFromEdge(const int edge_id, int& v1_id, int& v2_id){
  if(!testEdge(edge_id)){
    outError("Edge ID is out of bound!");
    return false;
  }
  v1_id = list_edge[edge_id].v1;
  v2_id = list_edge[edge_id].v2;
  return true;
}

template<typename Vertex, typename Edge>
inline bool GraphBase<Vertex, Edge>::testVertex(const int v_id){
  return v_id >= 0 && v_id < list_vertex.size();
}

template<typename Vertex, typename Edge>
inline bool GraphBase<Vertex, Edge>::testEdge(const int edge_id){
  return edge_id >= 0 && edge_id < list_edge.size();
}

template<typename Vertex, typename Edge>
inline bool GraphBase<Vertex, Edge>::testEdge(const int v1_id, const int v2_id){
  for(size_t it = 0; it < list_edge.size(); it++){
    if((list_edge[it].v1 == v1_id && list_edge[it].v2 == v2_id) || (list_edge[it].v1 == v2_id && list_edge[it].v2 == v1_id))
      return true;
  }
  return false;
}

template <typename VertexT, typename EdgeT>
inline int GraphBase<VertexT, EdgeT>::getVertexNeighborListPosition(const int src_v_id, const int tgt_v_id)
{
  int numNeighbors = getNumVertexNeighbors(src_v_id);

  if (numNeighbors == -1)
    return -1;

  for (size_t it = 0; it < numNeighbors; it++)
  {
    if (list_vertex[src_v_id].neighbors[it] == tgt_v_id)
      return it;
  }

  return -1;
}

template <typename VertexT, typename EdgeT>
inline bool GraphBase<VertexT, EdgeT>::testConsistency (const int v1_id, const int v2_id, int &v1_it, int &v2_it)
{
  // Get vertex neighbor list positions
  v1_it = getVertexNeighborListPosition(v2_id, v1_id);
  v2_it = getVertexNeighborListPosition(v1_id, v2_id);

  // Check adjacency list for consistency (DEBUG)
  if ((v1_it == -1 && v2_it != -1) || (v1_it != -1 && v2_it == -1))
  {
    std::cout << "one vertex has another as it's member, but not the other way around. Adjacency list is not consistent!" << std::endl;
    std::cout << "(v1: " << v1_id << ", v2: " << v2_id << ")" << std::endl;
    return false;
  }

  // Check that both neighbors have the same edge id (DEBUG)
  if (( v1_it != -1 && v2_it != -1) &&
      ( list_vertex[v1_id].neighbor_edges[v2_it] != list_vertex[v2_id].neighbor_edges[v1_it]))
  {
    std::cout << "neighbors belonging to the same edge have different edge ids!" << std::endl;
    std::cout << "v1: " << v1_id << ", edge id: " << list_vertex[v1_id].neighbor_edges[v2_it] << ")" << std::endl;
    std::cout << "v2: " << v2_id << ", edge id: " << list_vertex[v2_id].neighbor_edges[v1_it] << ")" << std::endl;
    return false;
  }
  return true;
}

#endif
