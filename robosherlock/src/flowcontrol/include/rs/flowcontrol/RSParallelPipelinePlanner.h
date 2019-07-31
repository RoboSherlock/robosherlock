/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Patrick Mania <pmania@cs.uni-bremen.de>
 *         Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
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

#ifndef __RSPARALLEL_PIPELINE_PLANNER_H__
#define __RSPARALLEL_PIPELINE_PLANNER_H__

#include <rs/utils/common.h>
#include <rs/scene_cas.h>
#include <rs/graph/DirectedGraph.h>
#include <rs/graph/GraphAlgorithms.hpp>
#include <rs/graph/GraphPrimitives.h>

#include <vector>
#include <map>
#include <utility>
#include <unordered_set>

#include <omp.h>

class RSParallelPipelinePlanner
{
public:
  typedef std::vector< std::vector<std::string> > AnnotatorOrderings;
  typedef std::vector< std::vector<int> >         AnnotatorOrderingIndices;

  DirectedGraph dependencyGraph; // we will assume each node ID correspond to annotatorLists array ID

  RSParallelPipelinePlanner() {}
  RSParallelPipelinePlanner(std::vector<std::string> input_list) : annotatorList(input_list)
  {
    dependencyGraph.setVertices(input_list.size());
  }

  ~RSParallelPipelinePlanner() {}

  void reset();

  bool getAnnotatorList(std::vector<std::string> &list) const;

  void setAnnotatorList(const std::vector<std::string> list);

  bool getPlannedPipeline(AnnotatorOrderings &list) const;

  bool getPlannedPipelineIndices(AnnotatorOrderingIndices &list) const;

  bool planPipelineStructure(std::map<std::string, rs::AnnotatorCapabilities> &dependencies);

  //for debug purpose
  void print();

protected:

  bool refinePlannedPipeline(std::map<std::string, rs::AnnotatorCapabilities> &dependencies);

  bool labelAnnotatorOrder();

  bool buildDependenciesGraph(std::map<std::string, rs::AnnotatorCapabilities> &dependencies);

private:
  std::vector<std::string> annotatorList;
  AnnotatorOrderings annotatorOrderings;
  AnnotatorOrderingIndices annotatorOrderingIndices;

};

#endif // __RSPARALLEL_PIPELINE_PLANNER_H__
