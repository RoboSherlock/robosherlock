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

#ifndef RSPARALLEL_PIPELINE_PLANNER_H
#define RSPARALLEL_PIPELINE_PLANNER_H

#include <rs/utils/common.h>
#include <rs/scene_cas.h>

#include <rs/queryanswering/JsonPrologInterface.h>
#include <rs/graph/DirectedGraph.hpp>

#include <vector>
#include <map>
#include <utility>

class RSParallelPipelinePlanner : public JsonPrologInterface
{
private:
  std::vector<std::string> annotatorList;
  std::vector< std::vector<std::string> > plannedPipeline;
  JsonPrologInterface::AnnotatorDependencies dependencies;

  DirectedGraph dependencyGraph; // we will assume each node ID correspond to annotatorLists array ID

public:

  RSParallelPipelinePlanner() : JsonPrologInterface() {}
  RSParallelPipelinePlanner(std::vector<std::string> input_list) : JsonPrologInterface(), annotatorList(input_list) {}

  ~RSParallelPipelinePlanner() {}

  bool getAnnotatorList(std::vector<std::string> &list) const;

  void setAnnotatorList(std::vector<std::string> list);

  void setAnnotatorDependencies(std::map< std::string,
                                          std::pair< std::vector <std::string>,
                                                     std::vector <std::string> > > &dependencies);

  bool getPlannedPipeline(std::vector< std::vector<std::string> > &list) const;

  bool planPipelineStructure();

protected:

  void refinePlannedPipeline();

  void labelAnnotatorOrder();

  bool buildDependenciesGraph();

};

#endif
