/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *         Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *         Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
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

#ifndef RSANALYSISENGINE_H
#define RSANALYSISENGINE_H

#include <rs/utils/output.h>
#include <rs/utils/time.h>
#include <rs/utils/exception.h>
#include <rs/utils/RSPipelineManager.h>

#include <uima/api.hpp>

class RSAnalysisEngine
{
public:
  std::string name;


protected:
  uima::AnalysisEngine *engine;
  uima::CAS *cas;
  RSPipelineManager *rspm;

public:

  RSAnalysisEngine();

  ~RSAnalysisEngine();

  virtual void init(const std::string &file);

  void initPipelineManager();

  void stop();

  virtual void process();

  inline void resetCas()
  {
    cas->reset();
  }

  uima::CAS* getCas()
  {
    return cas;
  }

  RSPipelineManager* getPipelineManager()
  {
    return rspm;
  }

};
#endif // RSANALYSISENGINE_H
