/** @name annotator.cpp
-----------------------------------------------------------------------------

 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.

-----------------------------------------------------------------------------

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */

//lint -e625

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */

#include <uima/annotator.hpp>
#include <uima/cas.hpp>

#include <uima/msg.h>
#include <uima/annotator_context.hpp>


/* ----------------------------------------------------------------------- */
/*       Implementation dependencies                                       */
/* ----------------------------------------------------------------------- */

//#define DEBUG_VERBOSE
#include <uima/trace.hpp>
#ifndef NDEBUG
#include <uima/envvar.hpp>                              /* for debugIndicateProcessHasBeenPerformed(); */
#include <uima/envvars.h>                               /* for debugIndicateProcessHasBeenPerformed(); */
#endif

#include <uima/comp_ids.h>
#include <uima/resmgr.hpp>

#include <uima/capability.hpp>
#include <uima/log.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */
#define UIMA_ANNOTATOR_DELAYED_INFO_NOT_AVAILABLE _TEXT("Information not available for delayed loaded annotator")

#define UIMA_ANNOTATOR_DEFAULT_UNSPECIFIED_TERRITORY  true
#define UIMA_CONFIG_ANNOTATOR_PROCESS_UNSPECIFIED_TERRITORIES  _TEXT("ProcessUnspecifiedTerritories")


using namespace std;
/***
#ifndef NDEBUG
   #define UIMA_ANNOTATOR_INDICATE_PROCESS_FILE_EXTENSION _TEXT(".ind")
#endif
***/
/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Globals                                                           */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Function declarations                                             */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Macro definitions                                                 */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Private Implementation                                            */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Public implementation                                             */
/* ----------------------------------------------------------------------- */
namespace uima {
  UIMA_EXC_CLASSIMPLEMENT(ExcEnumerationOverflow, ExcIllFormedInputError);


  AnnotatorProxy::AnnotatorProxy(const util::Filename & crclFilename,
                                 const TCHAR * cpszSymbolicName,
                                 bool  bDelayLoad) :
      iv_clFilename(crclFilename),
      iv_strSymbolicName(cpszSymbolicName),
      iv_cpclFile(0),
      iv_bProcessUnspecifiedTerritories(UIMA_ANNOTATOR_DEFAULT_UNSPECIFIED_TERRITORY),
      iv_hUserData(0),
      iv_pAnnotatorContext(NULL)
#ifdef DEBUG_ANNOTATOR_TIMING
      ,iv_clTimerLoad()
      ,iv_clTimerInit()
      ,iv_clTimerDeInit()
      ,iv_clTimerConfig()
      ,iv_clTimerProcessDocument()
      ,iv_clTimerBatchProcessComplete()
      ,iv_clTimerCollectionProcessComplete()
#endif
      /* ----------------------------------------------------------------------- */
  {
    ;
  }

  AnnotatorProxy::~AnnotatorProxy(void)
  /* ----------------------------------------------------------------------- */
  {


    if (!isLoadedDelayed()) {
      unload();
    }
    /* make sure that unload() was performed before call to destructor */
    assert(iv_cpclFile == 0);
    assert(iv_hUserData == (uima::TyHandle) 0);

  }  //lint !e1740: pointer member 'iv_cpclFile' 'iv_hUserData' not directly freed or zero'ed by destructor

  bool AnnotatorProxy::isValid(void) const
  /* ----------------------------------------------------------------------- */
  {
    assert(EXISTS(iv_cpclFile));
    return(iv_cpclFile->isValid());
  }

  const util::Filename & AnnotatorProxy::getFilename(void) const
  /* ----------------------------------------------------------------------- */
  {
    if (EXISTS(iv_cpclFile)) {
      assert(iv_cpclFile->getFilename().matchesBase(iv_clFilename));

      return(iv_cpclFile->getFilename());
    } else {
      return(iv_clFilename);
    }
  }


  TyErrorId AnnotatorProxy::load(void)
  /* ----------------------------------------------------------------------- */
  {
    util::Trace                 clTrace(util::enTraceDetailHigh, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);
    UIMA_ANNOTATOR_TIMING(iv_clTimerLoad.start());

    clTrace.dump(_TEXT("Annotator"), getSymbolicName().c_str());
    iv_cpclFile = ResourceManager::getInstance().requestAnnotatorFile(iv_clFilename);
    assert(EXISTS(iv_cpclFile));

    // (Dropped code that set iv_clFilename to name of annotator file without path & extension
    //  as it is not used ... and the filename is no longer changed during the load.)
    UIMA_ANNOTATOR_TIMING(iv_clTimerLoad.stop());

    if (!(EXISTS(iv_cpclFile)) || iv_cpclFile->getErrorId() !=  UIMA_ERR_NONE) {
      std::string err = "AnnotatorProxy::load() failed to load ";
      err += iv_clFilename.getAsCString();
      ResourceManager::getInstance().getLogger().logError(err);
	}
    return iv_cpclFile->getErrorId();
	
  }

  TyErrorId AnnotatorProxy::loadDelayed(void)
  /* ----------------------------------------------------------------------- */
  {
    iv_bDelayLoad = false;
    return(load());
  }

  void AnnotatorProxy::unload(void)
  /* ----------------------------------------------------------------------- */
  {
    util::Trace                 clTrace(util::enTraceDetailHigh, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);

    if (isLoadedDelayed()) {
      return;
    }
    clTrace.dump(_TEXT("Annotator"), getSymbolicName().c_str());
    clTrace.dump("unload...");

    //assert(EXISTS(iv_cpclFile));
    /* make sure that deInit() was performed before call to unload() */
    //assert(NOTEXISTS(iv_hUserData));
    iv_cpclFile = 0;
  }

  const char* AnnotatorProxy::getErrorMsg() const
  /* ----------------------------------------------------------------------- */
  {
    if (EXISTS(iv_cpclFile))
      return(iv_cpclFile->getErrorMsg());
    else
      return NULL;
  }

  TyErrorId AnnotatorProxy::init(AnnotatorContext & rAnnContext)
  /* ----------------------------------------------------------------------- */
  {
    if (isLoadedDelayed()) {
      return(init_delayed(rAnnContext));
    } else {
      return(init_now(rAnnContext));
    }
  }

  TyErrorId AnnotatorProxy::init_delayed(AnnotatorContext & rAnnContext)
  /* ----------------------------------------------------------------------- */
  {
    assertWithMsg(false, "Never tried");
    return UIMA_ERR_NOT_YET_IMPLEMENTED;

#ifdef DELAYED_LOADING_ENABLED
    assert(isLoadedDelayed());
    iv_pTCAS = & rTCAS;
    iv_pAnnotatorContext = & rAnnContext;
    util::Trace              clTrace(util::enTraceDetailHigh, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);

    // remember address of config control for a later call to init_now
    // Note: we can't remember the rclConfig itself since it will go out of scope
    iv_pclDelayConfigCtrl    = &rclConfig.getConfigCtrl();
    // remember symbolic name for a later call to init_now
    iv_strDelaySectionHeader = rclConfig.getSymbolicName();

    /* Since we can't access the annotator to get enumeration of types/languages
       etc. we have to assume this info has been specified in the ini file.
    */
    TyErrorId               utRetVal = UIMA_ERR_NONE;

    // the ini value was a comma separated list of type names
    iv_vecDelayLoadGeneratedTypes.clear();
    rclConfig.delayLoadGeneratedTypes(iv_vecDelayLoadGeneratedTypes);
    // To allow for easier configuration we assume that each type generated
    // is supported for the "Unspecified-Language"
    // That way the user has less to specify in the SupportedLanguagesForDelayedLoad
    // option
    TyTypenameContainer::iterator itTy1;
    STL_FOR_EACH(itTy1, iv_vecDelayLoadGeneratedTypes) {
      vector< Language > & rvecLanguages = iv_vecDelayLoadSupportedLanguages[(*itTy1)];
      assert(rvecLanguages.size() == 0);
      rvecLanguages.push_back(Language(Language::UNSPECIFIED));
    }

    // the ini value was a comma separated list of type names
    iv_vecDelayLoadRequiredTypes.clear();
    rclConfig.delayLoadRequiredTypes(iv_vecDelayLoadRequiredTypes);

    // the ini value was a comma separated list of type names
    iv_vecDelayLoadRecommendedTypes.clear();
    rclConfig.delayLoadRecommendedTypes(iv_vecDelayLoadRecommendedTypes);

    TyTypenameContainer vecTemp1;
    // the ini value was a comma separated list of <type>:<lang>[:<lang>] expressions
    rclConfig.delayLoadSupportedLanguages(vecTemp1);
    // now look at each string of the form <type>:<lang>[:<lang>] and parse them
    // into type and language-list part
    TyTypenameContainer::const_iterator it1;
    TyTypenameContainer vecTemp2;
    TyTypenameContainer::iterator it2;
    Language clLang(Language::INVALID);
    STL_FOR_EACH(it1, vecTemp1) {
      // parse colon (:) delimited list into separate strings
      vecTemp2.clear();
      delimitedString2Vector(vecTemp2,   // output vector
                             (*it1),     // input string
                             ":",        //lint !e918 delimiters : Prototype coercion of pointers
                             true,       // trim string flag
                             false       // insert empty strings flag
                            );

      it2 = vecTemp2.begin();
      if (it2 == vecTemp2.end()) {
        rAnnContext.getLogger().logError(UIMA_ERR_CONFIG_INVALID_OPTION_VALUE, UIMA_CONFIG_ANNOTATOR_DELAY_LOAD_SUPPORTED_LANGUAGES);
        utRetVal = (TyErrorId)UIMA_ERR_CONFIG_INVALID_OPTION_VALUE;
        break;
      }
      // get a reference to an either an existing entry (created above from
      // delayLoadGeneratedTypes entries) or create a new and empty one.
      vector< Language > & rvecLangList = iv_vecDelayLoadSupportedLanguages[(*it2)];
      // if it existed as default from above throw away default value and
      // overwrite with specified stuff from here
      rvecLangList.clear();
      ++it2;
      for (; it2 != vecTemp2.end(); ++it2) {
        clLang = Language((*it2));
        if (!clLang.isValid()) {
          rAnnContext.getLogger().logError(UIMA_ERR_CONFIG_INVALID_OPTION_VALUE, UIMA_CONFIG_ANNOTATOR_DELAY_LOAD_SUPPORTED_LANGUAGES);
          utRetVal = (TyErrorId)UIMA_ERR_CONFIG_INVALID_OPTION_VALUE;
          break;
        }
        rvecLangList.push_back(clLang);
      }
      if (utRetVal != UIMA_ERR_NONE) {
        break;
      }
    }
    clTrace.dump(_TEXT("rc"), utRetVal);

    return(utRetVal);
#endif
  }

  TyErrorId AnnotatorProxy::init_now(AnnotatorContext & rAnnotatorContext)
  /* ----------------------------------------------------------------------- */
  {
    iv_pAnnotatorContext = & rAnnotatorContext;
    assert(!isLoadedDelayed());
    util::Trace                 clTrace(util::enTraceDetailHigh, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);

    clTrace.dump(_TEXT("Annotator"), getSymbolicName().c_str());
    assert(EXISTS(iv_cpclFile));
    assert(iv_cpclFile->isValid());

    if (rAnnotatorContext.isParameterDefined(UIMA_CONFIG_ANNOTATOR_PROCESS_UNSPECIFIED_TERRITORIES)) {
      /* optionally, check whether AnnotatorProxy should be processed for unspecified territories */
      (void) rAnnotatorContext.extractValue(UIMA_CONFIG_ANNOTATOR_PROCESS_UNSPECIFIED_TERRITORIES, iv_bProcessUnspecifiedTerritories);
    }
    if (iv_bProcessUnspecifiedTerritories) {
      clTrace.dump(UIMA_CONFIG_ANNOTATOR_PROCESS_UNSPECIFIED_TERRITORIES _TEXT(" enabled!"));
    }

    TyAnnotatorSignatureMakeAE procMaker  = (TyAnnotatorSignatureMakeAE) iv_cpclFile->getProcedure("makeAE");
    assert(EXISTS(procMaker));
    if (NOTEXISTS(procMaker)) {
      ResourceManager::getInstance().getLogger().logError("AnnotatorProxy::init_now() Could not get procedure makeAE() ");
      return (UIMA_ERR_ANNOTATOR_MISSING_CREATE);
    }
    /* first we need to trigger the creation of the AnnotatorProxy's object ... */
    clTrace.dump("CreateAnnotator...");
    UIMA_ANNOTATOR_TIMING(iv_clTimerInit.start());
    iv_hUserData= (procMaker) ();
    UIMA_ANNOTATOR_TIMING(iv_clTimerInit.stop());
    clTrace.dumpAdrs(_TEXT("Annotator Handle"), (const void *) iv_hUserData);   //lint !e911
    if (NOTEXISTS(iv_hUserData)) {
      ResourceManager::getInstance().getLogger().logError("AnnotatorProxy::init_now() Could not instantiate annotator ");
      return(UIMA_ERR_ANNOTATOR_COULD_NOT_CREATE);
    }
    /* ... then we initialize the AnnotatorProxy */
    clTrace.dump("InitializeAnnotator...");
    UIMA_ANNOTATOR_TIMING(iv_clTimerInit.start());
    //? AnnotatorContext & rclAnnotatorContext = REINTERPRET_CAST(AnnotatorContext &, rclEngine);
//      AnnotatorContext & rclAnnotatorContext = (AnnotatorContext &) rclEngine;
    ///utRetVal = (procInit)(rAnnotatorContext, iv_hUserData);
    iv_hUserData->setAnnotatorContext(rAnnotatorContext);
    TyErrorId utRetVal = iv_hUserData->initialize(rAnnotatorContext);

    UIMA_ANNOTATOR_TIMING(iv_clTimerInit.stop());
    clTrace.dump(_TEXT("rc"), utRetVal);
    return(utRetVal);
  }

  TyErrorId AnnotatorProxy::deInit(void)
  /* ----------------------------------------------------------------------- */
  {
    util::Trace                 clTrace(util::enTraceDetailHigh, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);

    clTrace.dump(_TEXT("Annotator"), getSymbolicName().c_str());

    if (isLoadedDelayed()) {
      // if we have been delayed until now we have nothing to deinit
      return(TyErrorId)UIMA_ERR_NONE;
    }
    assert(EXISTS(iv_cpclFile));
    assert(iv_cpclFile->isValid());
    assert(EXISTS(iv_hUserData));
    clTrace.dump("DestroyAnnotator...");
    UIMA_ANNOTATOR_TIMING(iv_clTimerDeInit.start());
    TyErrorId utRetVal = iv_hUserData->destroy();

    UIMA_ANNOTATOR_TIMING(iv_clTimerDeInit.stop());
    clTrace.dump(_TEXT("rc"), utRetVal);
    /* at this point, the user data may not be used any more */
	delete iv_hUserData;
    iv_hUserData = 0;
    return(utRetVal);
  }

  TyErrorId AnnotatorProxy::typeSystemInit(uima::TypeSystem const & typeSystem) {
    util::Trace             clTrace(util::enTraceDetailMedium, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);
    TyErrorId               utRetVal = UIMA_ERR_NONE;
    utRetVal = iv_hUserData->typeSystemInit(typeSystem);
    return utRetVal;
  }


  TyErrorId AnnotatorProxy::reConfig()
  /* ----------------------------------------------------------------------- */
  {
    util::Trace             clTrace(util::enTraceDetailMedium, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);
    TyErrorId               utRetVal = UIMA_ERR_NONE;

    // if we are not loaded yet we have nothing to do: the annotator will access
    // modified configuration once/if it is finaly loaded
    if (isLoadedDelayed()) {
      clTrace.dump(_TEXT("rc"), utRetVal);
      return((TyErrorId) utRetVal);
    }
    assert(EXISTS(iv_cpclFile));
    assert(iv_cpclFile->isValid());
    assert( EXISTS(iv_pAnnotatorContext) );
    if (iv_pAnnotatorContext->isParameterDefined(UIMA_CONFIG_ANNOTATOR_PROCESS_UNSPECIFIED_TERRITORIES)) {
      /* optionally, check whether Annotator should be processed for unspecified territories */
      (void) iv_pAnnotatorContext->extractValue(UIMA_CONFIG_ANNOTATOR_PROCESS_UNSPECIFIED_TERRITORIES, iv_bProcessUnspecifiedTerritories);
    }
    clTrace.dump("ReconfigureAnnotator...");
    UIMA_ANNOTATOR_TIMING(iv_clTimerConfig.start());
    utRetVal = iv_hUserData->reconfigure();
    UIMA_ANNOTATOR_TIMING(iv_clTimerConfig.stop());
    clTrace.dump(_TEXT("rc"), utRetVal);
    return(utRetVal);
  }

  TyErrorId AnnotatorProxy::doProcessDocument(CAS & cas, AnnotatorContext & rAnnContext, ResultSpecification const & crResultSpec)
  /* ----------------------------------------------------------------------- */
  {
    util::Trace             clTrace(util::enTraceDetailMedium, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);
    TyErrorId               utErrorId = UIMA_ERR_NONE;

    clTrace.dump(_TEXT("Annotator"), getSymbolicName().c_str());

    if (isLoadedDelayed()) {
      assertWithMsg(false, "Delayed loading could result in errors due to already committed type system!");
#ifdef DELAYED_LOADING_ENABLED
      utErrorId = loadDelayed();
      clTrace.dump(_TEXT("Delayed Load:"), (long)utErrorId);
      /* could be loaded? */
      if (utErrorId != UIMA_ERR_NONE) {
        rAnnContext.getLogger().logError(utErrorId, getFilename().getAsCString());   //lint !e713: Loss of precision (arg. no. 1) (unsigned long to long)
        clTrace.dump(_TEXT("Delayed load error"), (long) utErrorId);
        /****
        #ifndef NDEBUG
                    debugIndicateProcessHasBeenPerformed(rTCAS, rAnnContext, utErrorId);
        #endif
        ***/
        return(utErrorId);
      }
      assert(EXISTS(iv_pclDelayConfigCtrl));
      /* extract annotator info for symbolic name */
      ConfigAnnotator clConfigAnnotator(*iv_pclDelayConfigCtrl, iv_strDelaySectionHeader);
      utErrorId = clConfigAnnotator.init();
      if (utErrorId != UIMA_ERR_NONE) {
        rAnnContext.getLogger().logError(utErrorId, getFilename().getAsCString());   //lint !e713: Loss of precision (arg. no. 1) (unsigned long to long)
        clTrace.dump(_TEXT("Delayed config access error"), (long) utErrorId);
        /***
        #ifndef NDEBUG
                    debugIndicateProcessHasBeenPerformed(rTCAS, rAnnContext, utErrorId);
        #endif
        ***/
        return(utErrorId);
      }

      utErrorId = init_now(rAnnContext, clConfigAnnotator);
      /* could be initialized? */
      if (utErrorId != UIMA_ERR_NONE) {
        rAnnContext.getLogger().logError(utErrorId, getFilename().getAsCString());   //lint !e713: Loss of precision (arg. no. 1) (unsigned long to long)
        clTrace.dump(_TEXT("Delayed init error"), (long) utErrorId);
        /***
        #ifndef NDEBUG
                    debugIndicateProcessHasBeenPerformed(cas, rAnnContext, utErrorId);
        #endif
        ***/
        return(utErrorId);
      }
#endif
    }
    clTrace.dump("ProcessDocument????");

    assert(EXISTS(iv_cpclFile));
    assert(iv_cpclFile->isValid());
    assert(EXISTS(iv_hUserData));
    utErrorId = iv_hUserData->process(cas, crResultSpec);
    clTrace.dump(_TEXT("rc"), utErrorId);
    return(utErrorId);
  }

  TyErrorId AnnotatorProxy::processDocument(CAS & cas, ResultSpecification const & crResultSpec)
  /* ----------------------------------------------------------------------- */
  {
    util::Trace            clTrace(util::enTraceDetailMedium, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);
    TyErrorId utErrorId = UIMA_ERR_NONE;

    UIMA_ANNOTATOR_TIMING(iv_clTimerProcessDocument.start());
    assert( EXISTS(iv_pAnnotatorContext) );
    utErrorId = doProcessDocument(cas, *iv_pAnnotatorContext, crResultSpec);
    UIMA_ANNOTATOR_TIMING(iv_clTimerProcessDocument.stop());
    return utErrorId;
  }



  TyErrorId AnnotatorProxy::batchProcessComplete()
  /* ----------------------------------------------------------------------- */
  {
    util::Trace             clTrace(util::enTraceDetailMedium, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);
    TyErrorId               utRetVal = UIMA_ERR_NONE;

    // if we are not loaded yet we have nothing to do: the annotator will access
    // modified configuration once/if it is finaly loaded
    if (isLoadedDelayed()) {
      clTrace.dump(_TEXT("rc"), utRetVal);
      return((TyErrorId) utRetVal);
    }
    assert(EXISTS(iv_cpclFile));
    assert(iv_cpclFile->isValid());
    assert(EXISTS(iv_hUserData));
    assert( EXISTS(iv_pAnnotatorContext) );
    if (iv_pAnnotatorContext->isParameterDefined(UIMA_CONFIG_ANNOTATOR_PROCESS_UNSPECIFIED_TERRITORIES)) {
      /* optionally, check whether Annotator should be processed for unspecified territories */
      (void) iv_pAnnotatorContext->extractValue(UIMA_CONFIG_ANNOTATOR_PROCESS_UNSPECIFIED_TERRITORIES, iv_bProcessUnspecifiedTerritories);
    }
    /* pass control to the Annotator */
    clTrace.dump("BatchprocessingcompleteAnnotator...");
    UIMA_ANNOTATOR_TIMING(iv_clTimerBatchProcessComplete.start());
    utRetVal = iv_hUserData->batchProcessComplete();
    UIMA_ANNOTATOR_TIMING(iv_clTimerBatchProcessComplete.stop());
    clTrace.dump(_TEXT("rc"), utRetVal);
    return(utRetVal);
  }

  TyErrorId AnnotatorProxy::collectionProcessComplete()
  /* ----------------------------------------------------------------------- */
  {
    util::Trace             clTrace(util::enTraceDetailMedium, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);
    TyErrorId               utRetVal = UIMA_ERR_NONE;

    // if we are not loaded yet we have nothing to do: the annotator will access
    // modified configuration once/if it is finaly loaded
    if (isLoadedDelayed()) {
      clTrace.dump(_TEXT("rc"), utRetVal);
      return((TyErrorId) utRetVal);
    }
    assert(EXISTS(iv_cpclFile));
    assert(iv_cpclFile->isValid());
    assert(EXISTS(iv_hUserData));
    assert( EXISTS(iv_pAnnotatorContext) );
    if (iv_pAnnotatorContext->isParameterDefined(UIMA_CONFIG_ANNOTATOR_PROCESS_UNSPECIFIED_TERRITORIES)) {
      /* optionally, check whether Annotator should be processed for unspecified territories */
      (void) iv_pAnnotatorContext->extractValue(UIMA_CONFIG_ANNOTATOR_PROCESS_UNSPECIFIED_TERRITORIES, iv_bProcessUnspecifiedTerritories);
    }
    clTrace.dump("CollectionprocessingcompleteAnnotator...");
    UIMA_ANNOTATOR_TIMING(iv_clTimerCollectionProcessComplete.start());
    utRetVal = iv_hUserData->collectionProcessComplete();
    UIMA_ANNOTATOR_TIMING(iv_clTimerCollectionProcessComplete.stop());
    clTrace.dump(_TEXT("rc"), utRetVal);
    return(utRetVal);
  }


  bool AnnotatorProxy::hasNext()
  /* ----------------------------------------------------------------------- */
  {
    util::Trace              clTrace(util::enTraceDetailMedium, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);

    // if we are not loaded yet we have nothing to do: the annotator will access
    // modified configuration once/if it is finaly loaded
    if (isLoadedDelayed()) {
      return false;
    }
    assert(EXISTS(iv_cpclFile));
    assert(iv_cpclFile->isValid());
    assert(EXISTS(iv_hUserData));
    assert( EXISTS(iv_pAnnotatorContext) );
    if (iv_pAnnotatorContext->isParameterDefined(UIMA_CONFIG_ANNOTATOR_PROCESS_UNSPECIFIED_TERRITORIES)) {
      /* optionally, check whether Annotator should be processed for unspecified territories */
      (void) iv_pAnnotatorContext->extractValue(UIMA_CONFIG_ANNOTATOR_PROCESS_UNSPECIFIED_TERRITORIES, iv_bProcessUnspecifiedTerritories);
    }
    clTrace.dump("HasnextAnnotator...");
    UIMA_ANNOTATOR_TIMING(iv_clTimerHasNext.start());
    bool hasNext = iv_hUserData->hasNext();
    UIMA_ANNOTATOR_TIMING(iv_clTimerHasNext.stop());

    return(hasNext);
  }


  CAS &  AnnotatorProxy::next()
  /* ----------------------------------------------------------------------- */
  {
    util::Trace              clTrace(util::enTraceDetailMedium, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);

    // if we are not loaded yet we have nothing to do: the annotator will access
    // modified configuration once/if it is finaly loaded
    /** why do this check (bi)
       if (isLoadedDelayed()) {
          return NULL;
       } **/

    assert(EXISTS(iv_cpclFile));
    assert(iv_cpclFile->isValid());
    assert(EXISTS(iv_hUserData));
    assert( EXISTS(iv_pAnnotatorContext) );
    if (iv_pAnnotatorContext->isParameterDefined(UIMA_CONFIG_ANNOTATOR_PROCESS_UNSPECIFIED_TERRITORIES)) {
      /* optionally, check whether Annotator should be processed for unspecified territories */
      (void) iv_pAnnotatorContext->extractValue(UIMA_CONFIG_ANNOTATOR_PROCESS_UNSPECIFIED_TERRITORIES, iv_bProcessUnspecifiedTerritories);
    }
    clTrace.dump("NextAnnotator...");
    UIMA_ANNOTATOR_TIMING(iv_clTimerHasNext.start());
    CAS & cas = iv_hUserData->next();
    UIMA_ANNOTATOR_TIMING(iv_clTimerHasNext.stop());

    return(cas);
  }

  int  AnnotatorProxy::getCasInstancesRequired()
  /* ----------------------------------------------------------------------- */
  {
    util::Trace              clTrace(util::enTraceDetailMedium, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);

    // if we are not loaded yet we have nothing to do: the annotator will access
    // modified configuration once/if it is finaly loaded
    if (isLoadedDelayed()) {

      return 0;
    }
    assert(EXISTS(iv_cpclFile));
    assert(iv_cpclFile->isValid());
    assert(EXISTS(iv_hUserData));
    assert( EXISTS(iv_pAnnotatorContext) );
    if (iv_pAnnotatorContext->isParameterDefined(UIMA_CONFIG_ANNOTATOR_PROCESS_UNSPECIFIED_TERRITORIES)) {
      /* optionally, check whether Annotator should be processed for unspecified territories */
      (void) iv_pAnnotatorContext->extractValue(UIMA_CONFIG_ANNOTATOR_PROCESS_UNSPECIFIED_TERRITORIES, iv_bProcessUnspecifiedTerritories);
    }
    clTrace.dump("GetcasinstancesrequiredAnnotator...");
    UIMA_ANNOTATOR_TIMING(iv_clTimerHasNext.start());
    int max = iv_hUserData->getCasInstancesRequired();
    UIMA_ANNOTATOR_TIMING(iv_clTimerHasNext.stop());

    return(max);
  }

}
/* <EOF> */


