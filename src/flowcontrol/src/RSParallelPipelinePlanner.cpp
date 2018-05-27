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

void RSParallelPipelinePlanner::setAnnotatorList(const std::vector<std::string> list)
{
  annotatorList = list;
  dependencyGraph.setVertices(list.size());
}

bool RSParallelPipelinePlanner::getPlannedPipeline(std::vector< std::vector<std::string> > &list) const
{

}

bool RSParallelPipelinePlanner::buildDependenciesGraph(const JsonPrologInterface::AnnotatorDependencies &dependencies)
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

    std::vector<std::string> &src_inputs = dependencies[annotatorList[src_it]].first;
    //std::vector<std::string> &src_outputs = dependencies[annotatorList[src_it]].second;

    for(auto src_it_in = src_inputs.begin(); src_it_in != src_inputs.end(); src_it_in++)
    {
      for(int tgt_it = 0; tgt_it < annotatorList.size(); tgt_it++)
      {
        if(src_it != tgt_it)
        {
          //std::vector<std::string> &tgt_inputs = dependencies[[annotatorList[tgt_it]].first;
          std::vector<std::string> &tgt_outputs = dependencies[[annotatorList[tgt_it]].second;

          // match input requirement of src_it to all other annotators outputs and check if it is statified or not
          for(auto tgt_it_out = tgt_outputs.begin(); tgt_it_out != tgt_outputs.end(); tgt_it_out++)
          {
            if((*tgt_it_out).compare(*src_it_in) == 0)
            {
              dependencyGraph.addEdge(tgt_it, src_it);
              satisfiedInputs.push_back(*src_it_in);
            }
          }

          // match inputs src_it to all other annotators outputs
          /*for(auto src_it_out = src_outputs.begin(); src_it_out != src_outputs.end(); src_it_out++)
          {
            for(auto tgt_it_out = tgt_inputs.begin(); tgt_it_out != tgt_inputs.end(); tgt_inputs++)
            {
              if((*tgt_it_out).compare(*src_it_in))
              {
                dependencyGraph.addEdge(tgt_it, src_it);
              }
            }
          }*/
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


}

bool RSParallelPipelinePlanner::refinePlannedPipeline()
{

}

void RSParallelPipelinePlanner::labelAnnotatorOrder()
{

}

bool RSParallelPipelinePlanner::refinePlannedPipeline()
{

}

bool RSParallelPipelinePlanner::checkDependencyLoop()
{

}




#endif // WITH_JSON_PROLOG
