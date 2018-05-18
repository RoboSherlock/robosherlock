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

#ifndef __GRAPH_ALGORITHMS_H__
#define __GRAPH_ALGORITHMS_H__

#include <rs/graph/GraphBase.h>
#include <rs/graph/GraphPrimitives.h>

/** \brief Breadth first search algorithm to find connected components in a graph.
 *  \param[in]  graph      GraphBase<Vertex, Edge>
 *  \return a vector of vector of integer represents cluster of connected component indices
 */
template<typename Vertex, typename Edge>
inline std::vector< std::vector<int> > extractConnectedComponents(GraphBase<Vertex, Edge> &graph);

#endif // __GRAPH_ALGORITHMS_H__
