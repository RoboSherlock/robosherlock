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

#include <rs/flowcontrol/RSParallelPipelinePlanner.h>

#ifdef WITH_JSON_PROLOG

bool RSParallelPipelinePlanner::getAnnotatorList(std::vector<std::string> &list) const
{
  if(annotatorList.empty())
  {
    outWarn("Annotator List is not set!");
    return false;
  }

  list = annotatorList;
  return true;
}

bool RSParallelPipelinePlanner::getDependencyGraph(DirectedGraph* graph)
{
  if(dependencyGraph.list_edge.empty())
  {
    outWarn("Dependency Graph is not computed! Please run planPipelineStructure");
    return false;
  }

  graph = &dependencyGraph;
  return true;
}

void RSParallelPipelinePlanner::reset()
{
  dependencyGraph.clear();
  annotatorOrderings.clear();
  annotatorList.clear();
}

void RSParallelPipelinePlanner::setAnnotatorList(const std::vector<std::string> list)
{
  this->reset();
  annotatorList = list;
  dependencyGraph.setVertices(list.size());
}

bool RSParallelPipelinePlanner::getPlannedPipeline(std::vector< std::vector<std::string> > &list) const
{

}

bool RSParallelPipelinePlanner::planPipelineStructure(JsonPrologInterface::AnnotatorDependencies &dependencies)
{
  if(dependencies.empty())
  {
    outError("Dependency data is emptry! Cannot plan parallel pipeline.");
    return false;
  }

  if(!buildDependenciesGraph(dependencies))
  {
    outError("Build dependency graph failed! Cannot plan parallel pipeline.");
    return false;
  }

  //more to come

  return true;
}

bool RSParallelPipelinePlanner::buildDependenciesGraph(JsonPrologInterface::AnnotatorDependencies &dependencies)
{
  if(annotatorList.empty())
  {
    outWarn("Annotator List is not set!");
    return false;
  }

  //actually there are two implementation logic: DEMAND_BASED or SUPPLY_BASED, we will try DEMAND_BASED first
  #pragma omp parallel for
  for(int src_it = 0; src_it < annotatorList.size(); src_it++)
  {
    //for debug purpose
    std::vector<std::string> satisfiedInputs;

    std::unordered_set<std::string> &src_inputs = dependencies[annotatorList[src_it]].first;

    for(auto src_it_in = src_inputs.begin(); src_it_in != src_inputs.end(); src_it_in++)
    {
      for(int tgt_it = 0; tgt_it < annotatorList.size(); tgt_it++)
      {
        if(src_it != tgt_it)
        {
          std::unordered_set<std::string> &tgt_outputs = dependencies[annotatorList[tgt_it]].second;

          // match input requirement of src_it to all other annotators outputs and check if it is statified or not
          auto found = tgt_outputs.find(*src_it_in);
          if(found != tgt_outputs.end())
          {
            dependencyGraph.addEdge(tgt_it, src_it);
            satisfiedInputs.push_back(*src_it_in);
          }
        }
      }
    }

    //check if the src annotator is satisfied
    if(satisfiedInputs.size() != dependencies[annotatorList[src_it]].first.size())
    {
      outWarn("Annotator " << annotatorList[src_it] << " is not satisfied input requirements");
      std::string satisfied = "";
      for(auto it = satisfiedInputs.begin(); it != satisfiedInputs.end(); it++)
      {
        satisfied = satisfied + " ," + *it;
      }

      outWarn("Satisfied inputs are: " << satisfied);
    }
  }

  //check dependency loop

  return true;
}

bool RSParallelPipelinePlanner::refinePlannedPipeline()
{
  return true;
}

void RSParallelPipelinePlanner::labelAnnotatorOrder()
{

}

bool RSParallelPipelinePlanner::checkDependencyLoop()
{
  return true;
}




#endif // WITH_JSON_PROLOG
