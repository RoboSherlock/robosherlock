#ifndef GRAPH_ALGORITHMS_H
#define GRAPH_ALGORITHMS_H
#include <rs/graph/GraphBase.h>
#include <rs/graph/GraphPrimitives.h>


template<typename Vertex, typename Edge>
inline std::vector< std::vector<int> > extractConnectedComponents(GraphBase<Vertex, Edge>& graph);

#endif
