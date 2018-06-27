#include <rs/flowcontrol/RSParallelAnalysisEngine.h>

RSParallelAnalysisEngine::RSParallelAnalysisEngine(uima::AnnotatorContext &rANC,
                                                   bool bOwnsANC,
                                                   bool bOwnsTAESpecififer,
                                                   uima::internal::CASDefinition & casDefs,
                                                   bool ownsCasDefs) :
                          uima::internal::AggregateEngine(rANC, bOwnsANC, bOwnsTAESpecififer, casDefs, ownsCasDefs)
{
}

RSParallelAnalysisEngine::~RSParallelAnalysisEngine()
{
}

uima::TyErrorId RSParallelAnalysisEngine::annotatorProcess(std::string annotatorName,
                                                           uima::CAS &cas,
                                                           uima::ResultSpecification &resultSpec)
{
  //uima::CAS *tcas;
  uima::TyErrorId utErrorId = UIMA_ERR_NONE;

  //find target PrimitveEngine base on annotator name on current pipeline orderings
  icu::UnicodeString icu_annotator_name = icu::UnicodeString::fromUTF8(StringPiece(annotatorName.c_str()));
  uima::AnalysisEngine *pEngine;
  uima::internal::CapabilityContainer *pCapContainer;

  for(auto it = iv_annotatorMgr.iv_vecEntries.begin(); it != iv_annotatorMgr.iv_vecEntries.end(); it++)
  {
    if(it->iv_pEngine->getAnalysisEngineMetaData().getName() == icu_annotator_name)
    {
      pEngine = it->iv_pEngine;
      pCapContainer = it->iv_pCapabilityContainer;
    }
  }

  if(pEngine == NULL || pCapContainer == NULL)
  {
    outError("Annotator: " << annotatorName << " could not be found in current pipeline orderings (iv_vecEntries)");
    return UIMA_ERR_ANNOTATOR_COULD_NOT_FIND;
  }

  //start process Primitive Engine
  try
  {
    std::vector<uima::TypeOrFeature> tofsToBeRemoved;
    bool callEngine = true;
    bool requiresTCas = true;

    /*if (cas.isBackwardCompatibleCas())
    {
  	  tcas = &cas;
  	}*/

    //this populates the tofsToBeRemoved vector so always call it
    callEngine = iv_annotatorMgr.shouldEngineBeCalled(*pCapContainer,
                                                      resultSpec,
                                                      cas.getDocumentAnnotation().getLanguage(),
                                                      tofsToBeRemoved);

    //check the FlowConstraintType specified in the aggregate engine
    //if CapabilityLanguageFlow whether engine is called is
    //determined by shouldEngineBeCalled()
    /*uima::AnnotatorContext &rANC = this->getAnnotatorContext();
    uima::AnalysisEngineDescription const &crTAESpecifier = rANC.getTaeSpecifier();
    uima::FlowConstraints const *pFlow = crTAESpecifier.getAnalysisEngineMetaData()->getFlowConstraints();
    uima::FlowConstraints *flow = CONST_CAST(FlowConstraints *, pFlow);
    uima::FlowConstraints::EnFlowType flowType = flow->getFlowConstraintsType();

    //if FixedFlow specified all engines are always called so reset callEngine is true
    if (flowType == uima::FlowConstraints::FIXED)
    {
      callEngine = true;
    }*/

    //robosherlock always uses FIXED pipeline anyways, the way robosherlock changes orderings does not affect FlowType
    callEngine = true;

    if ( callEngine )
    {
      // create ResultSpec for annotator
      // this must be done because an annotator should only be called with the result spec
      // that its XML file indicates.
      uima::ResultSpecification annResSpec;
      for (auto citTOF = tofsToBeRemoved.begin(); citTOF != tofsToBeRemoved.end(); ++citTOF)
      {
        assert( (*citTOF).isValid() );
        annResSpec.add(*citTOF);
      }

      // does engine expect a TCas
      //AEs that declare at least one input or output SofA should be sent the base CAS.
      //Otherwise they must be sent a TCAS.
      /*const uima::AnalysisEngineMetaData::TyVecpCapabilities &vecCap = pEngine->getAnalysisEngineMetaData().getCapabilites();
      for (size_t i = 0; i < vecCap.size(); i++)
      {
        uima::Capability * cap = vecCap.at(i);
        uima::Capability::TyVecCapabilitySofas inputSofa = cap->getCapabilitySofas(Capability::INPUTSOFA);
        uima::Capability::TyVecCapabilitySofas outputSofa = cap->getCapabilitySofas(Capability::OUTPUTSOFA);
        if (inputSofa.size() > 0 || outputSofa.size() > 0)
        {
          requiresTCas = false;
          break;
        }
      }

      if (requiresTCas)
      {
  	    uima::SofaFS defSofa = cas.getSofa(pEngine->getAnnotatorContext().mapToSofaID(CAS::NAME_DEFAULT_TEXT_SOFA));
  	    if (!defSofa.isValid())
        {
  	      //TODO: throw exception
  	      cerr << "could not get default text sofa " << endl;
  	      return 99;
  	    }
  	    tcas = cas.getView(defSofa);

        //here call PrimitveEngine
  	    utErrorId = pEngine->process(*tcas, annResSpec);
      }
      else
      {
        utErrorId = ((uima::AnalysisEngine*) pEngine)->process(cas, annResSpec);
      }*/

      //always process on baseCAS for now, because robosherlock always specifies input output sofa
      utErrorId = ((uima::AnalysisEngine*) pEngine)->process(cas, annResSpec);

      if (utErrorId != UIMA_ERR_NONE)
      {
        return utErrorId; // return the error immediately, do not remove resSpec on AggregateEngine
      }

      // now remove TOFs from ResultSpec
      process_mutex.lock();
      for (auto citTOF = tofsToBeRemoved.begin(); citTOF != tofsToBeRemoved.end(); ++citTOF)
      {
        assert( (*citTOF).isValid() );
        resultSpec.remove(*citTOF);
      }
      process_mutex.unlock();

    }
    else
    {
      assert( tofsToBeRemoved.empty() );
      UIMA_TPRINT("----------- engine will *not* be processed");
    }
  }
  catch(uima::Exception uimaExc)
  {
    utErrorId = uimaExc.getErrorInfo().getErrorId();
  }
  catch(...)
  {
    outError("Unknown error has occured while process annotator: " << annotatorName);
    return NULL;
  }

  return utErrorId;
}


uima::TyErrorId RSParallelAnalysisEngine::paralleledProcess(uima::CAS &cas,
                                                            uima::ResultSpecification const &resSpec)
{
  uima::TyErrorId err = UIMA_ERR_NONE;
  uima::ResultSpecification resultSpec = resSpec;

  assert(iv_bIsInitialized);
  assert(!iv_vecEntries.empty());
  assert(!currentOrderings.empty());

  for(size_t order = 0; order < currentOrderings.size(); order++)
  {
    outInfo("Start processing orderings: " << order);
    std::vector< std::future<uima::TyErrorId> > primitiveProcessStatus;

    for(size_t i = 0; i < currentOrderings[order].size(); i++)
    {
      outInfo("Start thread: " << currentOrderings[order][i]);
      primitiveProcessStatus.push_back(std::async(std::launch::async,
                                                  std::bind(&RSParallelAnalysisEngine::annotatorProcess,
                                                            this,
                                                            currentOrderings[order][i],
                                                            std::ref(cas),
                                                            std::ref(resultSpec))));
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

namespace rs
{
  uima::AnalysisEngine* createParallelAnalysisEngine(icu::UnicodeString const &aeFile,
                                                     uima::ErrorInfo errInfo)
  {
    try
    {
      errInfo.reset();
      if (! uima::ResourceManager::hasInstance())
      {
        errInfo.setErrorId(UIMA_ERR_ENGINE_RESMGR_NOT_INITIALIZED);
        return NULL;
      }


      //parsing AEFile routine, using auto_ptr for auto-garbage collection if method failed
  	  uima::XMLParser builder;
      std::auto_ptr<uima::AnalysisEngineDescription> apTAESpecifier( new uima::AnalysisEngineDescription() );
      if (apTAESpecifier.get() == NULL)
      {
        errInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
        return NULL;
      }

      builder.parseAnalysisEngineDescription(*apTAESpecifier.get(), aeFile);

      apTAESpecifier->validate();
      apTAESpecifier->commit();

      std::auto_ptr<uima::AnnotatorContext> apANC( new uima::AnnotatorContext(apTAESpecifier.get()) );
      if (apANC.get() == NULL)
      {
        errInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
        return NULL;
      }

      std::auto_ptr<uima::internal::CASDefinition> apCASDef( uima::internal::CASDefinition::createCASDefinition(*apANC.get()) );

      // release auto_ptrs here because the createTAE transfers ownership to the engine
      apTAESpecifier.release();
      uima::AnalysisEngine *pResult = rs::createParallelAnalysisEngine(*apANC.release(),
                                                                       *apCASDef.release(),
                                                                       errInfo);

      return pResult;
    }
    catch (uima::Exception & rExc)
    {
        errInfo = rExc.getErrorInfo();
    }

    assert( errInfo.getErrorId() != UIMA_ERR_NONE );
    return NULL;
  }

  uima::AnalysisEngine* createParallelAnalysisEngine(uima::AnnotatorContext &rANC,
                                                     uima::internal::CASDefinition &casDefinition,
                                                     uima::ErrorInfo &errInfo)
  {
    uima::AnalysisEngine *pResult = NULL;
    assert( errInfo.getErrorId() == UIMA_ERR_NONE );
    try
    {
      // create the engine depending on the framework (UIMACPP or JEDII) or if it is primitive or aggregate.
      uima::AnalysisEngineDescription const &crTAESpecifier = rANC.getTaeSpecifier();
      pResult = new RSParallelAnalysisEngine( rANC, true, true, casDefinition, true );

      if (pResult == NULL)
      {
        errInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
        return NULL;
      }

      uima::TyErrorId utErrorID = pResult->initialize( crTAESpecifier );
      if (utErrorID != UIMA_ERR_NONE)
      {
        uima::ErrorInfo const &crLastError = pResult->getAnnotatorContext().getLogger().getLastError();
        if (crLastError.getErrorId() != UIMA_ERR_NONE)
        {
          errInfo = crLastError;
        }
          // overwrite the error ID
        errInfo.setErrorId( utErrorID );
        delete pResult;
        return NULL;
      }

      return  pResult;
    }
    catch (uima::Exception & rExc)
    {
      errInfo = rExc.getErrorInfo();
    }

    assert( errInfo.getErrorId() != UIMA_ERR_NONE );

    //clean up if failed
    if (pResult != NULL)
    {
      delete pResult;
    }

    return NULL;
  }
}
