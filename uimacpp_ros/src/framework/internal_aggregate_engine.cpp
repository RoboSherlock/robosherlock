/** @name internal_aggregate_engine.cpp
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

   Description: This file contains class AggregateEngine

-----------------------------------------------------------------------------


   4/27/1999  Initial creation

-------------------------------------------------------------------------- */

#define UIMA_ENGINE_MAIN_CPP


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> //must be included first to disable warnings
#include <uima/annotator_timing.hpp>
#include <uima/engine.hpp>

#include <uima/internal_aggregate_engine.hpp>
#include <uima/internal_fspromoter.hpp>
#include <uima/resmgr.hpp>
#include <uima/comp_ids.h>
#include <uima/err_ids.h>
#include <uima/annotator_mgr.hpp>
#include <uima/internal_casimpl.hpp>

#include <uima/macros.h>
#include <uima/trace.hpp>
#include <uima/consoleui.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */
using namespace std;
namespace uima {

  namespace internal {

    static const char *        gs_cpszReservedPrefix = _TEXT("UIMA");


    /* ----------------------------------------------------------------------- */
    /*       Forward declarations                                              */
    /* ----------------------------------------------------------------------- */

    /* ----------------------------------------------------------------------- */
    /*       Types / Classes                                                   */
    /* ----------------------------------------------------------------------- */

    /* ----------------------------------------------------------------------- */
    /*       Private Implementation                                            */
    /* ----------------------------------------------------------------------- */

    /*
    Most of the implementations required by uima::internal::EngineBase
    directly call the ASB.
    */


    TyErrorId AggregateEngine::init2(util::Trace & rclTrace)
    /* ----------------------------------------------------------------------- */
    {
      TyErrorId               utErrorId = UIMA_ERR_NONE;

      try {
        /* now, after all the general init and config has been done,
           we can init the annotator manager which will init all annotators */
        utErrorId = iv_annotatorMgr.launchInit();
        if (utErrorId != UIMA_ERR_NONE) {
          /* since the init failed, we must deinit all annotators in order
             to release all user allocated stuff */
          (void) iv_annotatorMgr.launchDeInit();   /* ignore this RC */
          return(utErrorId);
        }

        /* at this point, everything is fine and the engine is up and running! */

      }
      /* we need to catch any unexpected exceptions from init */
      catch (Exception & rclException) {
        rclTrace.dump(_TEXT("UIMACPP exception during init"));
        rclTrace.dump(_TEXT("UIMACPP exception id"), rclException.getErrorInfo().getErrorId());
        rclTrace.dump(_TEXT("UIMACPP exception name"), rclException.getName());
        rclTrace.dump(_TEXT("UIMACPP exception message"), rclException.getErrorInfo().getMessage().asString().c_str());
        getAnnotatorContext().getLogger().logError(rclException.getErrorInfo());
        return(logError(rclTrace, rclException.getErrorInfo().getErrorId()));
      }
//#ifdef NDEBUG
//      catch (...) {
//        /* this should never occur!!! */
//        rclTrace.dump(_TEXT("Unexpected unknown exception"));
//        assertWithMsg(false, _TEXT("Unexpected unknown exception in init()"));   //lint !e506: Constant value Boolean
//        return(logError(rclTrace, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION));   /* in case the assert() is no longer here */
//      }
//#endif
      return(utErrorId);
    }


    /* ----------------------------------------------------------------------- */
    /*       Implementation                                                    */
    /* ----------------------------------------------------------------------- */


    AggregateEngine::AggregateEngine(AnnotatorContext & rANC, bool bOwnsANC,
                                     bool bOwnsTAESpecififer,
                                     uima::internal::CASDefinition & casDefs, bool ownsCasDefs) :
        EngineBase(rANC, bOwnsANC, bOwnsTAESpecififer, casDefs, ownsCasDefs),
        iv_annotatorMgr(*this) {
      util::Trace             clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);

      clTrace.dump(_TEXT("UIMACPP Version"), AggregateEngine::getVersionInfo());
      clTrace.dump(_TEXT("UIMACPP Level"), AggregateEngine::getLevelInfo());
      assert(iv_state == EngineState::enEngineState_readyForInit);

#ifndef NDEBUG

      /* taph 18.10.2001: this code can be useful if you want to debug
         UIMACPP in an environment where you can't start the debugger directly
         e.g. if UIMACPP is called from Java.
         The programm will wait in the loop below which gives you time to
         attach the debugger to the (Java) process and start debugging.
      */
      util::Location tempDir;
      util::Filename clLockfileName(tempDir.getAsCString(), "uimacpplock","lck");
      clLockfileName.normalizeAbsolute();
      if (clLockfileName.isExistent()) {
        cout << "In AggregateEngine Ctor: Debug Lockfile detected." << endl;
        cout << "Halting UIMACPP execution until " << clLockfileName.getAsCString() <<  " is deleted/renamed..." << endl;
        do {
          apr_sleep(3000000);             // 3 seconds
        } while (clLockfileName.isExistent());
        cout << "Continuing..." << endl;
      }
#endif

    }



    AggregateEngine::~AggregateEngine(void)
    /* ----------------------------------------------------------------------- */
    {
      util::Trace             clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      destroyIfNeeded();

    }



    TyErrorId AggregateEngine::initializeImpl(AnalysisEngineDescription const & ) {
      util::Trace             clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      TyErrorId               utErrorId = UIMA_ERR_NONE;

      /* continue with the rest of init() */
      utErrorId = init2(clTrace);
      if (utErrorId != UIMA_ERR_NONE) {
        return(logError(clTrace, utErrorId));
      }

      return(logError(clTrace, utErrorId));
    }


    TyErrorId AggregateEngine::reinitTypeSystemImpl() {
      util::Trace             clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      return iv_annotatorMgr.launchReinitTypeSystem();
    }


    TyErrorId AggregateEngine::destroyImpl(void) {
      util::Trace             clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      return iv_annotatorMgr.launchDeInit();
    }


    TyErrorId AggregateEngine::processImpl(CAS & cas, ResultSpecification const & crResultSpec) {
      util::Trace             clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      return iv_annotatorMgr.launchProcessDocument(cas, crResultSpec );
    }


    TyErrorId AggregateEngine::reconfigureImpl(void) {
      util::Trace             clTrace(util::enTraceDetailMedium, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      return iv_annotatorMgr.launchConfig();
    }

    TyErrorId AggregateEngine::batchProcessCompleteImpl(void) {
      util::Trace             clTrace(util::enTraceDetailMedium, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      return iv_annotatorMgr.launchCollectionProcessComplete();
    }

    TyErrorId AggregateEngine::collectionProcessCompleteImpl(void) {
      util::Trace             clTrace(util::enTraceDetailMedium, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      return iv_annotatorMgr.launchCollectionProcessComplete();
    }

    bool AggregateEngine::hasNextImpl() {
      return false;
    }

    CAS & AggregateEngine::nextImpl() {
      UIMA_EXC_THROW_NEW(ExcInvalidRequest,
                         UIMA_ERR_NOT_YET_IMPLEMENTED,
                         UIMA_MSG_ID_EXC_INVALID_CALL_TO_NEXT,
                         UIMA_MSG_ID_EXC_INVALID_CALL_TO_NEXT,
                         ErrorInfo::unrecoverable);
    }

    int AggregateEngine::getCasInstancesRequiredImpl() {
      return 0;
    }


    /* ----------------------------------------------------------------------- */
    /*       Static functions                                                  */
    /* ----------------------------------------------------------------------- */

#ifdef UIMA_DEBUG_ANNOTATOR_TIMING

    size_t
    AggregateEngine::getNbrOfAnnotators( void ) const {
      return iv_annotatorMgr.getNbrOfAnnotators();
    }

    icu::UnicodeString const &
    AggregateEngine::getAnnotatorName( size_t uiAnnotatorIndex  ) const {
      // it is not easy to find the names for the delegate engines
      // given only their position in the flow
      AnalysisEngineMetaData const * pMetaData =
        getAnnotatorContext().getTaeSpecifier().getAnalysisEngineMetaData();
      assert(EXISTS(pMetaData));
      FlowConstraints const * pFlow = pMetaData->getFlowConstraints();
      assert(EXISTS(pFlow));
      return pFlow->getNodes()[uiAnnotatorIndex];
    }

    uima::Timer
    AggregateEngine::getAnnotatorTimer( size_t uAnnotatorIndex,  EnAnnotatorTimer enAnnotatorTimer) const {
      return iv_annotatorMgr.getAnnotatorTimer(uAnnotatorIndex, enAnnotatorTimer);
    }


    void AggregateEngine::displayTimingData(util::ConsoleUI const & rConsole, bool bVerbose) const {
      string                  strTemp;
      double                  dTotalLoadTime            = (double)0.0;
      double                  dTotalInitTime            = (double)0.0;
      double                  dTotalDeInitTime          = (double)0.0;
      double                  dTotalConfigTime          = (double)0.0;
      double                  dTotalProcessDocumentTime = (double)0.0;
      double                  dTotalAnnotatorTime          = (double)0.0;
      size_t                  i;


      // first sum up times to get totals
      for ( i = 0; i < getNbrOfAnnotators(); ++i ) {
        dTotalLoadTime           += getAnnotatorTimer(i, enLoadAnnotatorTime).getAccumulatedTime();
        dTotalInitTime           += getAnnotatorTimer(i, enInitAnnotatorTime).getAccumulatedTime();
        dTotalDeInitTime         += getAnnotatorTimer(i, enDeinitAnnotatorTime).getAccumulatedTime();
        dTotalConfigTime         += getAnnotatorTimer(i, enConfigAnnotatorTime).getAccumulatedTime();
        dTotalProcessDocumentTime+= getAnnotatorTimer(i, enProcessAnnotatorTime).getAccumulatedTime();
      }                                            /* e-o-for */
      dTotalAnnotatorTime = dTotalLoadTime + dTotalInitTime + dTotalDeInitTime + dTotalConfigTime + dTotalProcessDocumentTime;

      rConsole.formatHeader("Overall annotator timing data");
      rConsole.format("Total time spent in annotators",       uima::Timer::timeString(dTotalAnnotatorTime).c_str());
      if (bVerbose) {
        rConsole.format("- library-load time",     uima::Timer::timeString(dTotalLoadTime).c_str());
        rConsole.format("- initialize time",       uima::Timer::timeString(dTotalInitTime).c_str());
        rConsole.format("- reconfigure time",      uima::Timer::timeString(dTotalConfigTime).c_str());
        rConsole.format("- process time",          uima::Timer::timeString(dTotalProcessDocumentTime).c_str());
      }

      // now dump the individual, total  and relative times
      rConsole.header("Individual annotator timing data (% for this annotator)");
      for ( i = 0; i < getNbrOfAnnotators(); ++i ) {
        string str;
        UnicodeStringRef uref(getAnnotatorName(i));
        uref.extract(str);
        rConsole.format(str.c_str(),  getAnnotatorTimer(i, enTotalAnnotatorTime).timeAndPercentString(dTotalAnnotatorTime).c_str());
        if (bVerbose) {
          rConsole.format("- library load time",     getAnnotatorTimer(i, enLoadAnnotatorTime).timeAndPercentString(dTotalLoadTime).c_str());
          rConsole.format("- initialize time",       getAnnotatorTimer(i, enInitAnnotatorTime).timeAndPercentString(dTotalInitTime).c_str());
          rConsole.format("- reconfigure time",      getAnnotatorTimer(i, enConfigAnnotatorTime).timeAndPercentString(dTotalConfigTime).c_str());
          rConsole.format("- process time",          getAnnotatorTimer(i, enProcessAnnotatorTime).timeAndPercentString(dTotalProcessDocumentTime).c_str());
          rConsole.newline();
        }
      }                                            /* e-o-for */
    }

#endif //UIMA_DEBUG_ANNOTATOR_TIMING

  }

} // namespace uima

/* <EOF> */


