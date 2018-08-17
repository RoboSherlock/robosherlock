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

class RSAggregateAnalysisEngine : public uima::internal::AggregateEngine
{
public:
  typedef std::vector< std::vector<std::string> > AnnotatorOrderings;
  typedef std::vector< std::vector<int> >         AnnotatorOrderingIndices;

  RSAggregateAnalysisEngine(uima::AnnotatorContext &rANC,
                             bool bOwnsANC,
                             bool bOwnsTAESpecififer,
                             uima::internal::CASDefinition &casDefs,
                             bool ownsCasDefs);

  ~RSAggregateAnalysisEngine();

  uima::TyErrorId annotatorProcess(std::string annotatorName,
                                   uima::CAS &cas,
                                   uima::ResultSpecification &annResultSpec);

  uima::TyErrorId annotatorProcess(int index, // the index of current annotator flow
                                   uima::CAS &cas,
                                   uima::ResultSpecification &annResultSpec);

  uima::TyErrorId paralleledProcess(uima::CAS &cas,
                                    uima::ResultSpecification const &resSpec);

  uima::TyErrorId paralleledProcess(uima::CAS &cas);


  std::vector<icu::UnicodeString> &getFlowConstraintNodes();


  void resetPipelineOrdering();


  void setContinuousPipelineOrder(std::vector<std::string> annotators);


  int getIndexOfAnnotator(std::string annotator_name);


  void getCurrentAnnotatorFlow(std::vector<std::string> &annotators);


  void setPipelineOrdering(std::vector<std::string> annotators);


#ifdef WITH_JSON_PROLOG
  bool planParallelPipelineOrderings(std::vector<std::string> &annotators,
      RSParallelPipelinePlanner::AnnotatorOrderings &orderings,
      RSParallelPipelinePlanner::AnnotatorOrderingIndices &orderingIndices);

  bool initParallelPipelineManager();
#endif

  void set_original_annotators()
  {
    original_annotators = this->iv_annotatorMgr.iv_vecEntries;
  }

public:
  bool querySuccess; // this variable is for fail safe mechanism to fall back to linear execution if query orderings fail
  bool use_default_pipeline_;

  AnnotatorOrderings currentOrderings;
  AnnotatorOrderingIndices currentOrderingIndices;

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
RSAggregateAnalysisEngine *createParallelAnalysisEngine(icu::UnicodeString const &aeFile,
    uima::ErrorInfo errInfo);

RSAggregateAnalysisEngine *createParallelAnalysisEngine(uima::AnnotatorContext &rANC,
    uima::internal::CASDefinition &casDefinition,
    uima::ErrorInfo &errInfo);

RSAggregateAnalysisEngine *createParallelAnalysisEngine(icu::UnicodeString const &aeFile,
    const std::unordered_map<std::string, std::string> &delegateEngines,
    uima::ErrorInfo errInfo);
}


#endif
