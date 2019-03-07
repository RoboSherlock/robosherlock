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
#include <rs/utils/output.h>
#include <rs/utils/time.h>
#include <rs/utils/exception.h>

#include <rs/scene_cas.h>
#include <rs/flowcontrol/RSXMLParser.h>
#include <rs/flowcontrol/RSParallelPipelinePlanner.h>
#include <rs/flowcontrol/YamlToXMLConverter.h>

#include <rs/queryanswering/ObjectDesignatorFactory.h>

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
#include <pwd.h>
#include <fstream>

#include <boost/algorithm/string.hpp>

/**
 * @brief The RSAggregateAnalysisEngine class enables a flow controller interface for an Aggregate analysis engine;
 * extends the default AAE from uima adding parallel pipeline  execution capability and exchange of the internal fixed flow
 * this overrides default uima behaviour since fixed flow originally was not meant to be changed during runtime;
 */
class RSAggregateAnalysisEngine : public uima::internal::AggregateEngine
{

public:

  /** @brief AnnotatorOrderings struct for holding the parallel orderings*/
  typedef std::vector< std::vector<std::string> > AnnotatorOrderings;

  /** @brief AnnotatorOrderingIndices indidces of annotators for parallel ordering*/
  typedef std::vector< std::vector<int> >AnnotatorOrderingIndices;

  /**
  * @brief RSAggregateAnalysisEngine constructor; mostly taken from the uima::internal::AggregateEngine;
  * calls parent class' constructor and additionally resets mutex
  */
  RSAggregateAnalysisEngine(uima::AnnotatorContext &rANC, bool bOwnsANC, bool bOwnsTAESpecififer,
                            uima::internal::CASDefinition &casDefs, bool ownsCasDefs);

  ~RSAggregateAnalysisEngine();

  /**
   * @brief annotatorProcess process a single annotator
   * @param annotatorName  name of primitive annotator
   * @param cas reference to the CAS
   * @param annResultSpec results specifications
   * @return returns UIMA_ERR_NONE on success
   */
  uima::TyErrorId annotatorProcess(std::string annotatorName,
                                   uima::CAS &cas,
                                   uima::ResultSpecification &annResultSpec);

  /**
   * @brief annotatorProcess same as above but this time with the index of the annotator in the flow
   * @param index index of annotation in the current flow
   * @param cas the CAS
   * @param annResultSpec the result specification
   * @return returns UIMA_ERR_NONE on success
   */
  uima::TyErrorId annotatorProcess(int index, // the index of current annotator flow
                                   uima::CAS &cas,
                                   uima::ResultSpecification &annResultSpec);

  /**
   * @brief parallelProcess run a parallel processing (runs the entire flow)
   * @param[in] cas reference to the CAS
   * @param[in] resSpec result spec
   * @return returns UIMA_ERR_NONE on success
   */
  uima::TyErrorId parallelProcess(uima::CAS &cas,
                                  uima::ResultSpecification const &resSpec);

  /**
   * @brief parallelProcess call from the outside like this; Result specifications are not used in RoboSherlock
   * @param[in] cas reference to the CAS
   * @return UIMA_ERR_NON on succes, other error code otherwise
   */
  uima::TyErrorId parallelProcess(uima::CAS &cas);

  /**
   * @brief getFlowConstraintNodes
   * @return currently set flowconstraints; the list of AEs that are int set as a flow
   */
  std::vector<icu::UnicodeString> &getFlowConstraintNodes();

  /**
   * @brief getCurrentAnnotatorFlow not sure how this is different from the above method;
   * accessing is done in a different way and return type is diff; TODO:check this and merge the two methods if possible
   * @param[out] annotatorslist of annotators in current flow
   */
  void getCurrentAnnotatorFlow(std::vector<std::string> &annotators);

  /**
   * @brief resetPipelineOrdering resets the internal flow to the one originally defined in the config file
   */
  void resetPipelineOrdering();

  /**
   * @brief setContinuousPipelineOrder
   * @param annotators list of (ordered) annotators constituting the continuous pipeline
   */
  void setContinuousPipelineOrder(std::vector<std::string> annotators);

  /**
   * @brief getIndexOfAnnotator get the index of an annotator from the flow;
   * @param annotator_name annotator name
   * @return  index of annotator in original flow
   */
  int getIndexOfAnnotator(std::string annotator_name);

  /**
   * @brief setPipelineOrdering set a new flow of annotators
   * @param annotators (ordered) list of annotators to set
   */
  void setPipelineOrdering(std::vector<std::string> annotators);

  /**
   * @brief planParallelPipelineOrderings plan a new parallel pipeline
   * @param[in] annotators  input sequence of annotators
   * @param[out] orderings output ordering of annotators
   * @param[out] orderingIndices output oredering of annotator indices
   * @return true if planning was successfull
   */
  bool planParallelPipelineOrderings(std::vector<std::string> &annotators,
                                     RSParallelPipelinePlanner::AnnotatorOrderings &orderings,
                                     RSParallelPipelinePlanner::AnnotatorOrderingIndices &orderingIndices);

  /**
   * @brief initParallelPipelineManager initialize the parallel pipeline manager; build dependency graph amongst other things
   * @return true for success
   */
  bool initParallelPipelineManager();


  void processOnce();

  void processOnce(std::vector<std::string> &designator_response, std::string queryString);

  /**
   * @brief setDelegateAnnotatorCapabilities set the capabilities of the AEs
   * @param[in] caps mapping from AE name to capabilities;
   */
  void setDelegateAnnotatorCapabilities(std::map < std::string, rs::AnnotatorCapabilities> caps)
  {
    delegate_annotator_capabilities_ = caps;
  }

  /**
   * @brief getDelegateAnnotatorCapabilities
   * @return map of annotator name to its scapabilities
   */
  std::map < std::string, rs::AnnotatorCapabilities> getDelegateAnnotatorCapabilities()
  {
    return delegate_annotator_capabilities_;
  }

  /**
   * @brief setParallel
   * @param[in] f flag for parallel execution or not
   */
  inline void setParallel(bool f)
  {
    parallel_ = f;
  }

  /**
   * @brief set_original_annotators set the original list of annotators
   */
  void backup_original_annotators()
  {
    delegate_annotators_ = this->iv_annotatorMgr.iv_vecEntries;
  }

  void setUseIdentityResolution(const bool useIDres)
  {
    use_identity_resolution_ = useIDres;
  }

  std::string getAAEName()
  {
    return name_;
  }

  /*set the next order of AEs to be executed*/
  /**
   * @brief setNextPipeline
   * @param l
   */
  void setNextPipeline(std::vector<std::string> next_pipeline_order)
  {
    this->next_pipeline_order = next_pipeline_order;
  }

  inline void resetCas()
  {
    cas_->reset();
  }

  void setParallelOrderings(RSAggregateAnalysisEngine::AnnotatorOrderings orderings,
                            RSAggregateAnalysisEngine::AnnotatorOrderingIndices orderingIndices)
  {
    currentOrderings = orderings;
    currentOrderingIndices = orderingIndices;
  }

  uima::CAS *getCas()
  {
    return cas_;
  }

  inline bool isInDelegateList(std::string d)
  {
    if(std::find(delegates_.begin(), delegates_.end(), d) != std::end(delegates_))
      return true;
    else
      return false;
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
    for(std::string i : param)
    {
      conversionString.push_back(UnicodeString(i.c_str()));
    }
    uima::AnnotatorContext *cr_context =  annotContext.getDelegate(ucs_delegate);
    cr_context->assignValue(UnicodeString(paramName.c_str()), conversionString);
  }

  // this variable is for fail safe mechanism to fall back to linear execution if query orderings fail

  bool querySuccess;
  bool use_default_pipeline_;
  bool use_identity_resolution_;


  std::string query_;

  AnnotatorOrderings currentOrderings;
  AnnotatorOrderingIndices currentOrderingIndices;
  RSParallelPipelinePlanner parallelPlanner;

  uima::CAS *cas_;
  std::map<std::string, rs::AnnotatorCapabilities> delegate_annotator_capabilities_;


private:

  //store different AE orders
  std::vector<std::string> default_pipeline_annotators_;
  std::vector<std::string> delegates_;
  std::vector<std::string> next_pipeline_order;

  uima::internal::AnnotatorManager::TyAnnotatorEntries delegate_annotators_;

  bool parallel_;
  std::string name_;

  //structures used for the parallel execution
  RSParallelPipelinePlanner::AnnotatorOrderings original_annotator_orderings;
  RSParallelPipelinePlanner::AnnotatorOrderingIndices original_annotator_ordering_indices;

protected:
  std::shared_ptr<std::mutex> process_mutex;
};


namespace rs
{

std::string convertAnnotatorYamlToXML(std::string annotatorName, std::map<std::string, rs::AnnotatorCapabilities> &delegate_capabilities);

RSAggregateAnalysisEngine *createRSAggregateAnalysisEngine(const std::string &file, bool parallel = false,
    bool pervasive = false, std::vector<std::string> contPipeline = {});

//RSAggregateAnalysisEngine *createParallelAnalysisEngine(icu::UnicodeString const &aeFile,
//    uima::ErrorInfo errInfo);
RSAggregateAnalysisEngine *createParallelAnalysisEngine(icu::UnicodeString const &aeFile,
    const std::unordered_map<std::string, std::string> &delegateEngines,
    uima::ErrorInfo errInfo);

RSAggregateAnalysisEngine *createParallelAnalysisEngine(uima::AnnotatorContext &rANC,
    uima::internal::CASDefinition &casDefinition,
    uima::ErrorInfo &errInfo);
}


#endif
