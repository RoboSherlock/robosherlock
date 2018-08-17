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

#ifndef __RSAGGREGATED_ANALYSIS_ENGINE_H__
#define __RSAGGREGATED_ANALYSIS_ENGINE_H__

#include <rs/utils/common.h>
#include <rs/scene_cas.h>
#include <rs/utils/exception.h>
#include <rs/flowcontrol/RSXMLParser.h>

#ifdef WITH_JSON_PROLOG
#include <rs/flowcontrol/RSParallelPipelinePlanner.h>
#include <rs/queryanswering/JsonPrologInterface.h>
#endif

#include <uima/api.hpp>
#include <uima/internal_aggregate_engine.hpp>
#include <uima/annotator_mgr.hpp>

#include <memory>
#include <vector>
#include <utility>
#include <future>
#include <functional>
#include <mutex>
#include <assert.h>
#include <unordered_map>

class RSAggregatedAnalysisEngine : public uima::internal::AggregateEngine
{
public:
  typedef std::vector< std::vector<std::string> > AnnotatorOrderings;
  typedef std::vector< std::vector<int> >         AnnotatorOrderingIndices;

  RSAggregatedAnalysisEngine(uima::AnnotatorContext &rANC,
                             bool bOwnsANC,
                             bool bOwnsTAESpecififer,
                             uima::internal::CASDefinition &casDefs,
                             bool ownsCasDefs);

  ~RSAggregatedAnalysisEngine();

  uima::TyErrorId annotatorProcess(std::string annotatorName,
                                   uima::CAS &cas,
                                   uima::ResultSpecification &annResultSpec);

  uima::TyErrorId annotatorProcess(int index, // the index of current annotator flow
                                   uima::CAS &cas,
                                   uima::ResultSpecification &annResultSpec);

  uima::TyErrorId paralleledProcess(uima::CAS &cas,
                                    uima::ResultSpecification const &resSpec);

  uima::TyErrorId paralleledProcess(uima::CAS &cas);


  AnnotatorOrderings currentOrderings;
  AnnotatorOrderingIndices currentOrderingIndices;


  ////START NEW STUFF
  std::vector<icu::UnicodeString> &getFlowConstraintNodes()
  {
    uima::FlowConstraints *flow;
    uima::FlowConstraints const *pFlow = this->getAnalysisEngineMetaData().getFlowConstraints();
    flow = CONST_CAST(uima::FlowConstraints *, pFlow);
    std::vector<icu::UnicodeString> const &nodes = flow->getNodes();
    std::vector<icu::UnicodeString> &flow_constraint_nodes = const_cast<std::vector<icu::UnicodeString> &>(nodes);
    return flow_constraint_nodes;
  }


  void resetPipelineOrdering()
  {
    this->iv_annotatorMgr.iv_vecEntries = original_annotators; // Reset to the original pipeline

#ifdef WITH_JSON_PROLOG
    if(parallel_)
    {
      this->currentOrderings = original_annotator_orderings;
      this->currentOrderingIndices = original_annotator_ordering_indices;
    }
#endif

    // Set default pipeline annotators, if set
    if(use_default_pipeline_)
    {
      setPipelineOrdering(default_pipeline_annotators);
    }
  }

  void setDefaultPipelineOrdering(std::vector<std::string> annotators)
  {
    use_default_pipeline_ = true;
    default_pipeline_annotators = annotators;
  }

  int getIndexOfAnnotator(std::string annotator_name)
  {
    icu::UnicodeString icu_annotator_name = icu::UnicodeString::fromUTF8(StringPiece(annotator_name.c_str()));

    std::vector<icu::UnicodeString> &nodes = this->getFlowConstraintNodes();
    auto it = std::find(nodes.begin(), nodes.end(), icu_annotator_name);
    if(it == nodes.end())
    {
      return -1;
    }
    return std::distance(nodes.begin(), it);
  }

  void getCurrentAnnotatorFlow(std::vector<std::string> &annotators)
  {
    annotators.clear();

    for(int i = 0; i < this->iv_annotatorMgr.iv_vecEntries.size(); i++)
    {
      uima::AnalysisEngine *pEngine = this->iv_annotatorMgr.iv_vecEntries[i].iv_pEngine;
      std::string tempNode;
      pEngine->getAnalysisEngineMetaData().getName().toUTF8String(tempNode);
      annotators.push_back(tempNode);
    }
  }

  void setPipelineOrdering(std::vector<std::string> annotators)
  {
    // Create empty list of
    //  typedef std::vector < EngineEntry > TyAnnotatorEntries;
    //  called 'new_annotators'
    // For each given annotator:
    //  1) Look up the index of the desired annotator
    //  2) Add a copy of the respectie EngineEntry from the original_annotators to the new list
    //
    uima::internal::AnnotatorManager::TyAnnotatorEntries new_annotators;
    for(int i = 0; i < annotators.size(); i++)
    {
      //  1) Look up the index of the desired annotator
      int idx = getIndexOfAnnotator(annotators.at(i));
      if(idx >= 0)
      {
        //  2) Add a copy of the respectie EngineEntry from the original_annotators to the new list
        uima::internal::AnnotatorManager::EngineEntry ee = original_annotators.at(idx);
        new_annotators.push_back(ee);
        continue;
      }

      // Right now, we just skip this annotator if it can't be found.
      outWarn(annotators.at(i) <<
              " can't be found in your loaded AnalysisEngine. Maybe it has not been "
              "defined in your given AnalysisEngine XML file? - Skipping it....");
      // return;
    }
    // Pass the new pipeline to uima's annotator manager
    this->iv_annotatorMgr.iv_vecEntries = new_annotators;

    //update parallel orderings
#ifdef WITH_JSON_PROLOG
    if(parallel_)
    {
      std::vector<std::string> currentFlow;
      this->getCurrentAnnotatorFlow(currentFlow);
      querySuccess = this->planParallelPipelineOrderings(currentFlow, this->currentOrderings, this->currentOrderingIndices);

      outInfo("Parallel pipeline after set new pipeline orderings: ");
      this->parallelPlanner.print();
    }
#endif
  }

#ifdef WITH_JSON_PROLOG

  bool planParallelPipelineOrderings(std::vector<std::string> &annotators,
      RSParallelPipelinePlanner::AnnotatorOrderings &orderings,
      RSParallelPipelinePlanner::AnnotatorOrderingIndices &orderingIndices)
  {
    bool success = true;
    if(annotators.empty())
    {
      outWarn("Annotators flow is not set! Parallel orderings will not be planned!");
      return false;
    }

    JsonPrologInterface::AnnotatorDependencies dependencies;
    success = queryInterface->retrieveAnnotatorsInputOutput(annotators, dependencies);

    if(dependencies.empty() || !success)
    {
      outWarn("Querying annotators dependency data is empty! Parallel orderings will not be planned!");
      return false;
    }

    parallelPlanner.setAnnotatorList(annotators);
    parallelPlanner.planPipelineStructure(dependencies);

    parallelPlanner.getPlannedPipeline(orderings);
    parallelPlanner.getPlannedPipelineIndices(orderingIndices);

    return success;
  }

  bool initParallelPipelineManager()
  {
    try
    {
      queryInterface.reset(new JsonPrologInterface());

      std::vector<std::string> currentFlow;
      this->getCurrentAnnotatorFlow(currentFlow);
      querySuccess = this->planParallelPipelineOrderings(currentFlow, this->currentOrderings, this->currentOrderingIndices);

      original_annotator_orderings = this->currentOrderings;
      original_annotator_ordering_indices = this->currentOrderingIndices;
    }
    catch(std::exception &e)
    {
      outError("std c++ error");
      std::cerr << e.what();
      return false;
    }
    catch(...)
    {
      outError("Unknown error has occured! Probaly you have not run json_prolog yet.");
    }

    return querySuccess;
  }


#endif

  void set_original_annotators()
  {
    original_annotators = this->iv_annotatorMgr.iv_vecEntries;
  }
  ///END NEW STUFF

public:
  bool querySuccess; // this variable is for fail safe mechanism to fall back to linear execution if query orderings fail
  bool use_default_pipeline_;

#ifdef WITH_JSON_PROLOG
  RSParallelPipelinePlanner parallelPlanner;
#endif

private:

  std::vector<std::string> default_pipeline_annotators;
  uima::internal::AnnotatorManager::TyAnnotatorEntries original_annotators;

#ifdef WITH_JSON_PROLOG
  bool parallel_;

  std::shared_ptr<JsonPrologInterface> queryInterface;

  RSParallelPipelinePlanner::AnnotatorOrderings original_annotator_orderings;
  RSParallelPipelinePlanner::AnnotatorOrderingIndices original_annotator_ordering_indices;
#endif

protected:
  std::shared_ptr<std::mutex> process_mutex;

};


namespace rs
{
RSAggregatedAnalysisEngine *createParallelAnalysisEngine(icu::UnicodeString const &aeFile,
    uima::ErrorInfo errInfo);

RSAggregatedAnalysisEngine *createParallelAnalysisEngine(uima::AnnotatorContext &rANC,
    uima::internal::CASDefinition &casDefinition,
    uima::ErrorInfo &errInfo);

RSAggregatedAnalysisEngine *createParallelAnalysisEngine(icu::UnicodeString const &aeFile,
    const std::unordered_map<std::string, std::string> &delegateEngines,
    uima::ErrorInfo errInfo);
}


#endif
