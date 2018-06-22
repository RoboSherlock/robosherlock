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

uima::TyErrorId annotatorProcess(std::string annotatorName,
                                 uima::CAS *cas,
                                 uima::ResultSpecification const &annResultSpec)
{

}


uima::TyErrorId paralleledProcess(uima::CAS *cas,
                                  uima::ResultSpecification const &resultSpec)
{

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
