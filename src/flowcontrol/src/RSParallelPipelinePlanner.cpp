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

  if(!labelAnnotatorOrder())
  {
    outError("Cannot plan orderings of annotators. Dependency data may contain loops!");
    return false;
  }

  if(!refinePlannedPipeline(dependencies))
  {
    outError("Refine pipeline failed!");
    return false;
  }

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
  //#pragma omp parallel for
  for(int src_it = 0; src_it < annotatorList.size(); src_it++)
  {
    //for debug purpose
    std::unordered_set<std::string> satisfiedInputs;

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
            satisfiedInputs.insert(*src_it_in);
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

bool RSParallelPipelinePlanner::refinePlannedPipeline(JsonPrologInterface::AnnotatorDependencies &dependencies)
{
  if(annotatorOrderings.empty())
  {
    outWarn("Orderings is not planned! Have you run planDependencyOrderings yet?");
    return false;
  }

  //check for standalone annotators at orderings 0
  annotatorOrderings.push_back(std::vector<std::string>());
  for(auto id = annotatorOrderings[0].begin(); id != annotatorOrderings[0].end(); id++)
  {
    if(dependencies[*id].second.empty())
    {
      annotatorOrderings[annotatorOrderings.size() - 1].push_back(*id);
      annotatorOrderings[0].erase(id);
      id--;
    }
  }

  if(annotatorOrderings[annotatorOrderings.size() - 1].empty())
  {
    annotatorOrderings.pop_back();
  }

  return true;
}

bool RSParallelPipelinePlanner::labelAnnotatorOrder()
{
  std::vector< std::vector<int> > orderings;
  if(!planDependencyOrderings<DirectedVertex, DirectedEdge>(dependencyGraph, orderings))
  {
    return false;
  }

  annotatorOrderings.clear();
  annotatorOrderings.resize(orderings.size());
  for(int order = 0; order < orderings.size(); order++)
  {
    for(int id = 0; id < orderings[order].size(); id++)
    {
      annotatorOrderings[order].push_back(annotatorList[orderings[order][id]]);
    }
  }

  return true;
}

void RSParallelPipelinePlanner::print()
{
  if(annotatorList.empty())
  {
    outWarn("Annotator List is not set!");
    return;
  }

  if(annotatorOrderings.empty())
  {
    outWarn("Orderings is not planned! Had you run planDependencyOrderings yet?");
    return;
  }

  for(int it = 0; it < annotatorOrderings.size(); it++)
  {
    std::string annotators = "";
    for(auto order_it = annotatorOrderings[it].begin(); order_it != annotatorOrderings[it].end(); order_it++)
    {
      annotators = annotators + " " + *order_it;
    }
    outInfo("Orderings " << it << ": " << annotators);
  }
}

#endif // WITH_JSON_PROLOG
