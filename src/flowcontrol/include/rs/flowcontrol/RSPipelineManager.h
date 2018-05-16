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

#ifndef RSPIPELINE_MANAGER_H
#define RSPIPELINE_MANAGER_H

#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include <algorithm>

#include <uima/api.hpp>
#include "uima/internal_aggregate_engine.hpp"
#include "uima/annotator_mgr.hpp"

#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/utils/exception.h>

class RSPipelineManager
{
public:
  RSPipelineManager(uima::AnalysisEngine *engine)
  {
    engine = engine;

    uima::FlowConstraints const *pFlow = engine->getAnalysisEngineMetaData().getFlowConstraints();
    flow = CONST_CAST(uima::FlowConstraints *, pFlow);
    aengine = ((uima::internal::AggregateEngine *)engine);
    original_annotators = aengine->iv_annotatorMgr.iv_vecEntries;
    use_default_pipeline = false;
  }

  void resetPipelineOrdering();

  void setPipelineOrdering(std::vector<std::string> annotators);

  // If you only want to use a restricted set of annotators if you reset the pipeline,
  // pass a list of annotator names to this method.
  void setDefaultPipelineOrdering(std::vector<std::string> annotators);

  std::vector<icu::UnicodeString> &getFlowConstraintNodes();
  // protected:
  // Get the index of the given annotator in this->flow_constraint_nodes.
  int getIndexOfAnnotator(std::string annotator_name);

  // private:
  /* data */
  uima::AnalysisEngine *engine;
  uima::internal::AggregateEngine *aengine;
  uima::FlowConstraints *flow;

  bool use_default_pipeline; // set to false again,if you want to disable the default pipeline order
  std::vector<std::string> default_pipeline_annotators;

  // This attribute will keep a copy of all the initialized annotators that have
  // been loaded on startup.
  uima::internal::AnnotatorManager::TyAnnotatorEntries original_annotators;
};

#endif


