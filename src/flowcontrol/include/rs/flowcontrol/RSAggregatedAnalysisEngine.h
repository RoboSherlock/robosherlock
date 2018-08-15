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

private:

protected:
  std::shared_ptr<std::mutex> process_mutex;

};


namespace rs
{
  uima::AnalysisEngine* createParallelAnalysisEngine(icu::UnicodeString const &aeFile,
                                                     uima::ErrorInfo errInfo);

  uima::AnalysisEngine* createParallelAnalysisEngine(uima::AnnotatorContext &rANC,
                                                     uima::internal::CASDefinition &casDefinition,
                                                     uima::ErrorInfo &errInfo);

  uima::AnalysisEngine* createParallelAnalysisEngine(icu::UnicodeString const &aeFile,
                                                     const std::unordered_map<std::string, std::string>& delegateEngines,
                                                     uima::ErrorInfo errInfo);
}


#endif
