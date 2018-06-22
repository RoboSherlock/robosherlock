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

#ifndef __RSPARALLEL_ANALYSIS_ENGINE_H__
#define __RSPARALLEL_ANALYSIS_ENGINE_H__

#include <rs/utils/common.h>
#include <rs/scene_cas.h>
#include <rs/flowcontrol/RSParallelPipelinePlanner.h>

#include <uima/api.hpp>
#include <uima/internal_aggregate_engine.hpp>
#include <uima/annotator_mgr.hpp>

#include <memory>
#include <vector>
#include <utility>
#include <assert.h> 

class RSParallelAnalysisEngine : public uima::internal::AggregateEngine
{
public:
  RSParallelAnnotatorManager(AnnotatorContext &rANC,
                             bool bOwnsANC,
                             bool bOwnsTAESpecififer,
                             uima::internal::CASDefinition &casDefs,
                             bool ownsCasDefs);

  ~RSParallelAnalysisEngine();

  uima::TyErrorId annotatorProcess(std::string annotatorName,
                                   CAS *cas,
                                   ResultSpecification const &annResultSpec);

  uima::TyErrorId paralleledProcess(CAS *cas,
                                    ResultSpecification const &resultSpec);


  RSParallelPipelinePlanner::AnnotatorOrderings currentOrderings;

private:

protected:

};


namespace rs
{
  uima::AnalysisEngine* createParallelAnalysisEngine(icu::UnicodeString const &aeFile
                                                     uima::ErrorInfo errInfo);

  uima::AnalysisEngine* createParallelAnalysisEngine(uima::AnnotatorContext &rANC,
                                                     bool bOwnsANC,
                                                     bool bOwnsTAESpecifier,
                                                     uima::internal::CASDefinition &casDefinition,
                                                     bool ownsCASDefintion,
                                                     uima::ErrorInfo &errInfo);
}


#endif
