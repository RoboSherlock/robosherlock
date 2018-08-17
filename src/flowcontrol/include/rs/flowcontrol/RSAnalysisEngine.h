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

#include <rs/utils/common.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>
#include <rs/utils/exception.h>

#include <rs/flowcontrol/YamlToXMLConverter.h>
#include <rs/flowcontrol/RSAggregateAnalysisEngine.h>

#include <uima/api.hpp>
#include <uima/internal_aggregate_engine.hpp>

#include <fstream>
#include <pwd.h>
#include <string>
#include <unordered_map>

#include <ros/package.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>


class RSAnalysisEngine
{

public:    
  std::string name_;
  bool parallel_;

protected:
  RSAggregateAnalysisEngine *engine;
  uima::CAS *cas;

public:

  RSAnalysisEngine();

  ~RSAnalysisEngine();

  void init(const std::string &file, bool parallel=false);

  void initPipelineManager();

  void stop();

  virtual void process();

  uima::TyErrorId parallelProcess(uima::CAS &cas)
  {
    return engine->paralleledProcess(cas);
  }

  inline void resetCas()
  {
    cas->reset();
  }

  uima::CAS* getCas()
  {
    return cas;
  }

  void setPipelineOrdering(std::vector<std::string> order)
  {
      engine->setPipelineOrdering(order);
  }

  void setParallelOrderings(RSAggregateAnalysisEngine::AnnotatorOrderings orderings,
                            RSAggregateAnalysisEngine::AnnotatorOrderingIndices orderingIndices)
  {
      engine->currentOrderings = orderings;
      engine->currentOrderingIndices = orderingIndices;
  }

  uima::CAS* newCAS()
  {
    return engine->newCAS();
  }
  uima::AnnotatorContext& getAnnotatorContext()
  {
    return engine->getAnnotatorContext();
  }
  void reconfigure()
  {
    engine->reconfigure();
  }
  void collectionProcessComplete()
  {
    engine->collectionProcessComplete();
  }
  void destroy()
  {
    engine->destroy();
  }

  template < class T >
  void overwriteParam(const std::string& annotName,const std::string& paramName, T const& param)
  {
    uima::AnnotatorContext &annotContext = getAnnotatorContext();
    UnicodeString ucs_delegate(annotName.c_str());
    uima::AnnotatorContext *cr_context =  annotContext.getDelegate(ucs_delegate);
    cr_context->assignValue(UnicodeString(paramName.c_str()),param);
  }
  template < class T >
  void overwriteParam(const std::string& annotName, const std::string& paramName, const std::vector<T> & param)
  {
   uima::AnnotatorContext &annotContext = getAnnotatorContext();
   UnicodeString ucs_delegate(annotName.c_str());
   uima::AnnotatorContext *cr_context =  annotContext.getDelegate(ucs_delegate);
   cr_context->assignValue(UnicodeString(paramName.c_str()),param); 
  }
  //Ease case for the user
  void overwriteParam(const std::string& annotName,const std::string& paramName, std::string const& param)
  {
    uima::AnnotatorContext &annotContext = getAnnotatorContext();
    UnicodeString ucs_delegate(annotName.c_str());
    uima::AnnotatorContext *cr_context =  annotContext.getDelegate(ucs_delegate);
    cr_context->assignValue(UnicodeString(paramName.c_str()),(UnicodeString) param.c_str());
  }
  void overwriteParam(const std::string& annotName, const std::string& paramName, const std::vector<std::string> & param)
  {
   uima::AnnotatorContext &annotContext = getAnnotatorContext();
   UnicodeString ucs_delegate(annotName.c_str());
   //Convert the std::string vector into UnicodeString and then overwrite with that variable
   std::vector<UnicodeString> conversionString;
   for (std::string i : param)
   {
    conversionString.push_back(UnicodeString(i.c_str()));
   }
   uima::AnnotatorContext *cr_context =  annotContext.getDelegate(ucs_delegate);
   cr_context->assignValue(UnicodeString(paramName.c_str()),conversionString); 
  }

  void getFixedFlow(const std::string filePath,
                    std::vector<std::string>& annotators);
 };
#endif // RSANALYSISENGINE_H
