#include <rs/flowcontrol/RSAggregatedAnalysisEngine.h>

RSAggregatedAnalysisEngine::RSAggregatedAnalysisEngine(uima::AnnotatorContext &rANC,
    bool bOwnsANC,
    bool bOwnsTAESpecififer,
    uima::internal::CASDefinition &casDefs,
    bool ownsCasDefs) :
  uima::internal::AggregateEngine(rANC, bOwnsANC, bOwnsTAESpecififer, casDefs, ownsCasDefs)
{
  process_mutex.reset(new std::mutex);
}

RSAggregatedAnalysisEngine::~RSAggregatedAnalysisEngine()
{
}

uima::TyErrorId RSAggregatedAnalysisEngine::annotatorProcess(std::string annotatorName,
    uima::CAS &cas,
    uima::ResultSpecification &resultSpec)
{
  //find target PrimitveEngine base on annotator name on current pipeline orderings
  icu::UnicodeString icu_annotator_name = icu::UnicodeString::fromUTF8(StringPiece(annotatorName.c_str()));
  int index = 0;

  for(; index < iv_annotatorMgr.iv_vecEntries.size(); index++)
  {
    if(iv_annotatorMgr.iv_vecEntries[index].iv_pEngine->getAnalysisEngineMetaData().getName() == icu_annotator_name)
    {
      break;
    }
  }

  return this->annotatorProcess(index, cas, resultSpec);
}

uima::TyErrorId RSAggregatedAnalysisEngine::annotatorProcess(int index,
    uima::CAS &cas,
    uima::ResultSpecification &resultSpec)
{
  if(index < 0 || index >= iv_annotatorMgr.iv_vecEntries.size())
  {
    outError("Provided index is invalid in current annotator flow. Index: " << index << ", flow size: " << iv_annotatorMgr.iv_vecEntries.size());
    return UIMA_ERR_ANNOTATOR_COULD_NOT_FIND;
  }

  uima::AnalysisEngine *pEngine = iv_annotatorMgr.iv_vecEntries[index].iv_pEngine;
  uima::internal::CapabilityContainer *pCapContainer = iv_annotatorMgr.iv_vecEntries[index].iv_pCapabilityContainer;

  if(pEngine == NULL || pCapContainer == NULL)
  {
    outError("Annotator index: " << index << " could not be found in current pipeline orderings (iv_vecEntries)");
    return UIMA_ERR_ANNOTATOR_COULD_NOT_FIND;
  }

  uima::TyErrorId utErrorId = UIMA_ERR_NONE;

  //start process Primitive Engine
  try
  {
    std::vector<uima::TypeOrFeature> tofsToBeRemoved;

    //this populates the tofsToBeRemoved vector so always call it
    iv_annotatorMgr.shouldEngineBeCalled(*pCapContainer,
                                         resultSpec,
                                         cas.getDocumentAnnotation().getLanguage(),
                                         tofsToBeRemoved);

    // create ResultSpec for annotator
    // this must be done because an annotator should only be called with the result spec
    // that its XML file indicates.
    uima::ResultSpecification annResSpec;
    for(auto citTOF = tofsToBeRemoved.begin(); citTOF != tofsToBeRemoved.end(); ++citTOF)
    {
      assert((*citTOF).isValid());
      annResSpec.add(*citTOF);
    }

    //always process on baseCAS for now, because robosherlock always specifies input output sofa
    utErrorId = ((uima::AnalysisEngine *) pEngine)->process(cas, annResSpec);

    if(utErrorId != UIMA_ERR_NONE)
    {
      return utErrorId; // return the error immediately, do not remove resSpec on AggregateEngine
    }

    // now remove TOFs from ResultSpec
    {
      std::lock_guard<std::mutex> lock(*process_mutex);
      for(auto citTOF = tofsToBeRemoved.begin(); citTOF != tofsToBeRemoved.end(); ++citTOF)
      {
        assert((*citTOF).isValid());
        resultSpec.remove(*citTOF);
      }
    }
  }
  catch(uima::Exception uimaExc)
  {
    utErrorId = uimaExc.getErrorInfo().getErrorId();
  }
  catch(...)
  {
    std::string annotatorName;
    pEngine->getAnalysisEngineMetaData().getName().toUTF8String(annotatorName);
    outError("Unknown error has occured while process annotator: " << annotatorName);
    return NULL;
  }

  return utErrorId;
}


uima::TyErrorId RSAggregatedAnalysisEngine::paralleledProcess(uima::CAS &cas,
    uima::ResultSpecification const &resSpec)
{
  uima::TyErrorId err = UIMA_ERR_NONE;
  uima::ResultSpecification resultSpec = resSpec;

  assert(iv_annotatorMgr.iv_bIsInitialized);
  assert(!iv_annotatorMgr.iv_vecEntries.empty());
  assert(!currentOrderingIndices.empty());
  assert(currentOrderingIndices.size() == currentOrderings.size());

  for(size_t order = 0; order < currentOrderingIndices.size(); order++)
  {
    outInfo("Start processing orderings: " << order);
    std::vector< std::future<uima::TyErrorId> > primitiveProcessStatus;

    for(size_t i = 0; i < currentOrderingIndices[order].size(); i++)
    {
      outInfo("Start thread: " << currentOrderings[order][i]);
      auto primitiveProcess = [this, order, i, &cas, &resultSpec]()
      {
        return this->annotatorProcess(currentOrderingIndices[order][i],
                                      cas,
                                      resultSpec);
      };
      primitiveProcessStatus.push_back(std::async(std::launch::async, primitiveProcess));
    }

    for(size_t i = 0; i < primitiveProcessStatus.size(); i++)
    {
      err = primitiveProcessStatus[i].get();
      if(err != UIMA_ERR_NONE)
      {
        outError("Exception throws at annotator: " << currentOrderings[order][i] << ". Pipeline will stop at this iteration!");
        return err;
      }
    }
  }

  if(resultSpec.getSize() > 0)
  {
    outWarn("The pipeline still does not satisfy ResultSpecification! Still has: " << resultSpec.getSize());
  }

  return err;
}

uima::TyErrorId RSAggregatedAnalysisEngine::paralleledProcess(uima::CAS &cas)
{
  //generate ResultSpecification from TaeSpecifier
  return this->paralleledProcess(cas, this->getCompleteResultSpecification());
}

#ifdef WITH_JSON_PROLOG
bool RSAggregatedAnalysisEngine::planParallelPipelineOrderings(
  std::vector<std::string> &annotators,
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


bool RSAggregatedAnalysisEngine::initParallelPipelineManager()
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

namespace rs
{
RSAggregatedAnalysisEngine *createParallelAnalysisEngine(icu::UnicodeString const &aeFile,
    uima::ErrorInfo errInfo)
{
  try
  {
    errInfo.reset();
    if(! uima::ResourceManager::hasInstance())
    {
      errInfo.setErrorId(UIMA_ERR_ENGINE_RESMGR_NOT_INITIALIZED);
      return NULL;
    }


    //parsing AEFile routine, using auto_ptr for auto-garbage collection if method failed
    uima::XMLParser builder;
    std::auto_ptr<uima::AnalysisEngineDescription> apTAESpecifier(new uima::AnalysisEngineDescription());
    if(apTAESpecifier.get() == NULL)
    {
      errInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
      return NULL;
    }

    builder.parseAnalysisEngineDescription(*apTAESpecifier.get(), aeFile);

    apTAESpecifier->validate();
    apTAESpecifier->commit();

    std::auto_ptr<uima::AnnotatorContext> apANC(new uima::AnnotatorContext(apTAESpecifier.get()));
    if(apANC.get() == NULL)
    {
      errInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
      return NULL;
    }

    std::auto_ptr<uima::internal::CASDefinition> apCASDef(uima::internal::CASDefinition::createCASDefinition(*apANC.get()));

    // release auto_ptrs here because the createTAE transfers ownership to the engine
    apTAESpecifier.release();
    RSAggregatedAnalysisEngine *pResult = rs::createParallelAnalysisEngine(*apANC.release(),
                                          *apCASDef.release(),
                                          errInfo);

    return pResult;
  }
  catch(uima::Exception &rExc)
  {
    errInfo = rExc.getErrorInfo();
  }

  assert(errInfo.getErrorId() != UIMA_ERR_NONE);
  return NULL;
}

RSAggregatedAnalysisEngine *createParallelAnalysisEngine(icu::UnicodeString const &aeFile,
    const std::unordered_map<std::string, std::string> &delegateEngines,
    uima::ErrorInfo errInfo)
{
  try
  {
    errInfo.reset();
    if(! uima::ResourceManager::hasInstance())
    {
      errInfo.setErrorId(UIMA_ERR_ENGINE_RESMGR_NOT_INITIALIZED);
      return NULL;
    }


    //parsing AEFile routine, using auto_ptr for auto-garbage collection if method failed
    RSXMLParser builder;
    std::auto_ptr<uima::AnalysisEngineDescription> apTAESpecifier(new uima::AnalysisEngineDescription());
    if(apTAESpecifier.get() == NULL)
    {
      errInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
      return NULL;
    }

    builder.parseAnalysisEngineDescription(*(apTAESpecifier.get()), delegateEngines, aeFile);
    // builder.parseAnalysisEngineDescription(*(apTAESpecifier.get()), aeFile);

    apTAESpecifier->validate();
    apTAESpecifier->commit();

    std::auto_ptr<uima::AnnotatorContext> apANC(new uima::AnnotatorContext(apTAESpecifier.get()));
    if(apANC.get() == NULL)
    {
      errInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
      return NULL;
    }

    std::auto_ptr<uima::internal::CASDefinition> apCASDef(uima::internal::CASDefinition::createCASDefinition(*apANC.get()));

    // release auto_ptrs here because the createTAE transfers ownership to the engine
    apTAESpecifier.release();
    RSAggregatedAnalysisEngine *pResult = rs::createParallelAnalysisEngine(*apANC.release(),
                                          *apCASDef.release(),
                                          errInfo);
    return pResult;
  }
  catch(uima::Exception &rExc)
  {
    errInfo = rExc.getErrorInfo();
  }

  assert(errInfo.getErrorId() != UIMA_ERR_NONE);
  return NULL;
}

RSAggregatedAnalysisEngine *createParallelAnalysisEngine(uima::AnnotatorContext &rANC,
    uima::internal::CASDefinition &casDefinition,
    uima::ErrorInfo &errInfo)
{
  RSAggregatedAnalysisEngine *pResult = NULL;
  assert(errInfo.getErrorId() == UIMA_ERR_NONE);
  try
  {
    // create the engine depending on the framework (UIMACPP or JEDII) or if it is primitive or aggregate.
    uima::AnalysisEngineDescription const &crTAESpecifier = rANC.getTaeSpecifier();
    pResult = new RSAggregatedAnalysisEngine(rANC, true, true, casDefinition, true);

    if(pResult == NULL)
    {
      errInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
      return NULL;
    }

    uima::TyErrorId utErrorID = pResult->initialize(crTAESpecifier);

    pResult->set_original_annotators();

    if(utErrorID != UIMA_ERR_NONE)
    {
      uima::ErrorInfo const &crLastError = pResult->getAnnotatorContext().getLogger().getLastError();
      if(crLastError.getErrorId() != UIMA_ERR_NONE)
      {
        errInfo = crLastError;
      }
      // overwrite the error ID
      errInfo.setErrorId(utErrorID);
      delete pResult;
      return NULL;
    }

    return  pResult;
  }
  catch(uima::Exception &rExc)
  {
    errInfo = rExc.getErrorInfo();
  }

  assert(errInfo.getErrorId() != UIMA_ERR_NONE);

  //clean up if failed
  if(pResult != NULL)
  {
    delete pResult;
  }

  return NULL;
}
}
