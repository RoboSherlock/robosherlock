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

#include <rs/queryanswering/JsonPrologInterface.h>
#include <rs/queryanswering/ObjectDesignatorFactory.h>

#include <uima/api.hpp>
#include <uima/internal_aggregate_engine.hpp>

#include <fstream>
#include <pwd.h>
#include <string>
#include <unordered_map>

#include <ros/package.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <tf_conversions/tf_eigen.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <pcl_ros/point_cloud.h>

class RSAnalysisEngine
{

public:

  std::string name_;
  bool parallel_, useIdentityResolution_;
  std::vector<std::string> next_pipeline_order;
  std::string query_;


protected:
  RSAggregateAnalysisEngine *engine_;
  uima::CAS *cas_;

  std::vector<std::string> delegates_;
  std::map<std::string,rs::AnnotatorCapabilities> delegateCapabilities_;

public:

  RSAnalysisEngine();

  ~RSAnalysisEngine();

  std::map<std::string, rs::AnnotatorCapabilities> getDelegateCapabilities();
  void init(const std::string &file, bool parallel = false,
            bool pervasive = false, std::vector<std::string> contPipeline = {});

  std::string convertAnnotatorYamlToXML(std::string);

  void stop();

  virtual void process();

  void process(std::vector<std::string> &designator_response,
               std::string query);


  void getFixedFlow(const std::string filePath, std::vector<std::string> &annotators);

  //draw results on an image
  template <class T>
  bool drawResulstOnImage(const std::vector<bool> &filter, const std::vector<std::string> &resultDesignators,
                          std::string &requestJson, cv::Mat &resImage);

  template <class T>
  bool highlightResultsInCloud(const std::vector<bool> &filter, const std::vector<std::string> &resultDesignators,
                               std::string &requestJson, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);


  uima::TyErrorId parallelProcess(uima::CAS &cas)
  {
    return engine_->parallelProcess(cas);
  }

  inline bool isInDelegateList(std::string d)
  {
    if(std::find(delegates_.begin(), delegates_.end(), d) != std::end(delegates_))
      return true;
    else
      return false;
  }

  inline void resetCas()
  {
    cas_->reset();
  }

  uima::CAS *getCas()
  {
    return cas_;
  }


  void setPipelineOrdering(std::vector<std::string> order)
  {
    engine_->setPipelineOrdering(order);
  }

  void setParallelOrderings(RSAggregateAnalysisEngine::AnnotatorOrderings orderings,
                            RSAggregateAnalysisEngine::AnnotatorOrderingIndices orderingIndices)
  {
    engine_->currentOrderings = orderings;
    engine_->currentOrderingIndices = orderingIndices;
  }

  uima::CAS *newCAS()
  {
    return engine_->newCAS();
  }
  uima::AnnotatorContext &getAnnotatorContext()
  {
    return engine_->getAnnotatorContext();
  }
  void reconfigure()
  {
    engine_->reconfigure();
  }
  void collectionProcessComplete()
  {
    engine_->collectionProcessComplete();
  }
  void destroy()
  {
    engine_->destroy();
  }

  /*set the next order of AEs to be executed*/
  void setNextPipeline(std::vector<std::string> l)
  {
    next_pipeline_order = l;
  }


  void setQuery(std::string q)
  {
    query_ = q;
  }

  /*get the next order of AEs to be executed*/
  inline std::vector<std::string> &getNextPipeline()
  {
    return next_pipeline_order;
  }

  inline void changeLowLevelPipeline(std::vector<std::string> &pipeline)
  {
    engine_->setContinuousPipelineOrder(pipeline);
    engine_->setPipelineOrdering(pipeline);
  }

  inline void applyNextPipeline()
  {
    if(engine_) {
      engine_->setPipelineOrdering(next_pipeline_order);
    }
  }

  inline void resetPipelineOrdering()
  {
    if(engine_) {
      engine_->resetPipelineOrdering();
    }
  }

  inline std::string getCurrentAEName()
  {
    return name_;
  }

  bool defaultPipelineEnabled()
  {
    if(engine_) {
      return engine_->use_default_pipeline_;
    }
    return false;
  }

  inline void useIdentityResolution(const bool useIDres)
  {
    useIdentityResolution_ = useIDres;
  }

  template < class T >
  void overwriteParam(const std::string &annotName, const std::string &paramName, T const &param)
  {
    uima::AnnotatorContext &annotContext = getAnnotatorContext();
    UnicodeString ucs_delegate(annotName.c_str());
    uima::AnnotatorContext *cr_context =  annotContext.getDelegate(ucs_delegate);
    cr_context->assignValue(UnicodeString(paramName.c_str()), param);
  }

  template < class T >
  void overwriteParam(const std::string &annotName, const std::string &paramName, const std::vector<T> &param)
  {
    uima::AnnotatorContext &annotContext = getAnnotatorContext();
    UnicodeString ucs_delegate(annotName.c_str());
    uima::AnnotatorContext *cr_context =  annotContext.getDelegate(ucs_delegate);
    cr_context->assignValue(UnicodeString(paramName.c_str()), param);
  }

  //Ease case for the user
  void overwriteParam(const std::string &annotName, const std::string &paramName, std::string const &param)
  {
    uima::AnnotatorContext &annotContext = getAnnotatorContext();
    UnicodeString ucs_delegate(annotName.c_str());
    uima::AnnotatorContext *cr_context =  annotContext.getDelegate(ucs_delegate);
    cr_context->assignValue(UnicodeString(paramName.c_str()), (UnicodeString) param.c_str());
  }
  void overwriteParam(const std::string &annotName, const std::string &paramName, const std::vector<std::string> &param)
  {
    uima::AnnotatorContext &annotContext = getAnnotatorContext();
    UnicodeString ucs_delegate(annotName.c_str());
    //Convert the std::string vector into UnicodeString and then overwrite with that variable
    std::vector<UnicodeString> conversionString;
    for(std::string i : param) {
      conversionString.push_back(UnicodeString(i.c_str()));
    }
    uima::AnnotatorContext *cr_context =  annotContext.getDelegate(ucs_delegate);
    cr_context->assignValue(UnicodeString(paramName.c_str()), conversionString);
  }
};
#endif // RSANALYSISENGINE_H
