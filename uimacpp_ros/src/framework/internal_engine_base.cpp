/** \file internal_engine_base.cpp .
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


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
//#define DEBUG_VERBOSE 1

#include <uima/pragmas.hpp>

#include <uima/macros.h>
#include <uima/internal_engine_base.hpp>
#include <uima/msg.h>
#include <uima/annotator_context.hpp>
#include <uima/log.hpp>
#include <uima/lowlevel_typesystem.hpp>
#include <uima/lowlevel_indexrepository.hpp>
#include <uima/casdefinition.hpp>
#include <uima/internal_casimpl.hpp>
#include <uima/trace.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */
namespace uima {
  namespace internal {

    uima::internal::EngineBase & EngineBase::promoteEngine( uima::AnalysisEngine & engine) {
      return (uima::internal::EngineBase&) engine;
    }

    EngineBase::EngineBase(AnnotatorContext & rANC, bool bOwnsANC,
                           bool bOwnsTAESpecififer,
                           uima::internal::CASDefinition & casDef, bool ownsCasDef)
        :
        iv_pAnnotatorContext( & rANC),
        iv_bOwnsANC(bOwnsANC),
        iv_bOwnsTAESpec(bOwnsTAESpecififer),
        iv_casDefinition( & casDef),
        iv_ownsCASDefinition(ownsCasDef) {
      iv_casHolder = NULL;
    }

    EngineBase::~EngineBase() {
      if (iv_bOwnsTAESpec) {
        delete iv_pAnnotatorContext->getTAESpec();
      }
      if (iv_bOwnsANC) {
        delete iv_pAnnotatorContext;
      }
      if (iv_ownsCASDefinition) {
        delete iv_casDefinition;
      }
      if (iv_casHolder) {
        delete iv_casHolder;
      }
    }

    /**
     * pretty simple CAS compatibility check: Just check if they
     * have the same type system and index definition.
     */
    void EngineBase::checkCASCompatibility(CAS const & cas) const {
      uima::internal::CASImpl const & tcasImpl = uima::internal::CASImpl::promoteCAS(cas);
      if ( (&tcasImpl.getHeap().getTypeSystem()) != (&iv_casDefinition->getTypeSystem())
           || (& tcasImpl.getIndexRepository().getIndexDefinition()) != (& iv_casDefinition->getIndexDefinition()) ) {
        UIMA_EXC_THROW_NEW(CASIncompatibilityException,
                           UIMA_ERR_ENGINE_INCOMPATIBLE_CAS,
                           UIMA_MSG_ID_EXC_INCOMPATIBLE_CAS,
                           UIMA_MSG_ID_EXCON_PROCESSING_CAS,
                           ErrorInfo::recoverable);
      }
    }


    bool EngineBase::checkAndSetCallingSequenceInitialize() {
      if (iv_state.getState() != EngineState::enEngineState_readyForInit) {
        return false;
      }
      iv_state.setToState(EngineState::enEngineState_readyForProcessOrReconfigOrDeinit);
      return true;
    }


    bool EngineBase::checkAndSetCallingSequenceDestroy() {
      if ( (iv_state.getState() != EngineState::enEngineState_readyForProcessOrReconfigOrDeinit) ) {
        return false;
      }

      iv_state.setToState(EngineState::enEngineState_readyForDeletion);
      return true;
    }

    bool EngineBase::checkAndSetCallingSequenceProcess() {
      UIMA_TPRINT("state: " << iv_state.getState());
      if (iv_state.getState() != EngineState::enEngineState_readyForProcessOrReconfigOrDeinit) {
        return false;
      }
      // dont change state
      return true;
    }


    bool EngineBase::checkAndSetCallingSequenceReconfigure() {
      if ( iv_state != EngineState::enEngineState_readyForProcessOrReconfigOrDeinit) {
        return false;
      }
      // dont change state
      return true;
    }

    TyErrorId EngineBase::logError(util::Trace & rclTrace, TyErrorId utErrorId) {
      if (utErrorId != UIMA_ERR_NONE) {
        rclTrace.dump(_TEXT("Error:"), (size_t) utErrorId);
      }
      return(utErrorId);
    }



    bool EngineBase::isInitialized(void) const {
      switch (iv_state.getState()) {
      case EngineState::enEngineState_readyForInit :
      case EngineState::enEngineState_readyForDeletion :
        return false;
      case EngineState::enEngineState_readyForProcessOrReconfigOrDeinit :
        return true;
      }
      assert(false);
      return false;
    }

    CAS * EngineBase::newCAS() const {
      assert( EXISTS(iv_casDefinition) );
      assert( EXISTS(iv_pAnnotatorContext) );
      CAS* tmpCas = uima::internal::CASImpl::createCASImpl(* CONST_CAST(uima::internal::CASDefinition*, iv_casDefinition), *iv_pAnnotatorContext);
      return tmpCas->getInitialView();
    }


    void EngineBase::destroyIfNeeded() {
      util::Trace                 clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);

      try {
        if (iv_state == EngineState::enEngineState_readyForProcessOrReconfigOrDeinit) {
          destroy();
          iv_state.assertMatch(EngineState::enEngineState_readyForDeletion);
        }
      } catch (ExcWinCException & rclException) {
        // windows "exceptions" are really abort signals: we re-throw for app to handle
        throw rclException;
      } catch (Exception & rclException) {
        clTrace.dump("Unexpected UIMACPP exception");
        clTrace.dump(rclException.asString().c_str());
        assertWithMsg(false, _TEXT("Unexpected UIMACPP exception in engine destructor"));   //lint !e506: Constant value Boolean
      } catch (std::exception & rclException) {
        clTrace.dump("ANSI C++ exception");
        clTrace.dump(rclException.what());
        assertWithMsg(false, _TEXT("Unexpected ANSI C++ exception in engine destructor"));   //lint !e506: Constant value Boolean
      }
#ifdef NDEBUG
      catch (...) {
        /* this should never occur!!! */
        clTrace.dump(_TEXT("Unexpected exception"));
        assertWithMsg(false, _TEXT("Unexpected unknown exception in engine destructor"));   //lint !e506: Constant value Boolean
      }
#endif
    }


    /**
     * Gathers all output capabilities from the TAE specifier.
     * This result spec is implicitly assumed when calling process(CAS) without a result spec.
     */
    TyErrorId EngineBase::initializeCompleteResultSpec() {
      iv_completeResultSpec.clear();

      assert( EXISTS(iv_casDefinition) );
      uima::TypeSystem const & rTypeSystem = iv_casDefinition->getTypeSystem();

      AnnotatorContext const & crANC = getAnnotatorContext();

      AnalysisEngineMetaData::TyVecpCapabilities const & crVecCaps = crANC.getTaeSpecifier().getAnalysisEngineMetaData()->getCapabilites();
      AnalysisEngineMetaData::TyVecpCapabilities::const_iterator citCaps;

      for (citCaps = crVecCaps.begin(); citCaps != crVecCaps.end(); ++citCaps) {
        Capability const * cpCap = * citCaps;
        Capability::TyVecCapabilityTofs const & crTypes = cpCap->getCapabilityTypes(Capability::OUTPUT);
        Capability::TyVecCapabilityTofs::const_iterator citTOFs;
        for (citTOFs = crTypes.begin(); citTOFs != crTypes.end(); ++citTOFs) {
          Type t = rTypeSystem.getType( *citTOFs );
          if (! t.isValid() ) {
            ErrorInfo errInf(
              ErrorMessage(UIMA_MSG_ID_UNKNOWN_FEATURE_IN_CAPBILITY_SPEC,(*citTOFs)),
              UIMA_ERR_ANNOTATOR_MGR_CONFIG_INVALID_RESULTSPECIFICATION,
              ErrorInfo::unrecoverable);
            getAnnotatorContext().getLogger().logError(errInf);
            return UIMA_ERR_ANNOTATOR_MGR_CONFIG_INVALID_RESULTSPECIFICATION;
          }
          iv_completeResultSpec.add(t);
        }

        Capability::TyVecCapabilityTofs const & crFeatures = cpCap->getCapabilityFeatures(Capability::OUTPUT);
        for (citTOFs = crFeatures.begin(); citTOFs != crFeatures.end(); ++citTOFs) {
          Feature f = rTypeSystem.getFeatureByFullName(*citTOFs);
          if (! f.isValid() ) {
            ErrorInfo errInf(
              ErrorMessage(UIMA_MSG_ID_UNKNOWN_FEATURE_IN_CAPBILITY_SPEC,(*citTOFs)),
              UIMA_ERR_ANNOTATOR_MGR_CONFIG_INVALID_RESULTSPECIFICATION,
              ErrorInfo::unrecoverable);
            getAnnotatorContext().getLogger().logError(errInf);
            return UIMA_ERR_ANNOTATOR_MGR_CONFIG_INVALID_RESULTSPECIFICATION;
          }
          iv_completeResultSpec.add(f);
        }

      }
      return UIMA_ERR_NONE;
    }


    TyErrorId EngineBase::initialize(AnalysisEngineDescription const & specifier) {
      try {
        util::Trace                clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);

        if (!checkAndSetCallingSequenceInitialize()) {
          return logError(clTrace, UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE);
        }
        assert( (& specifier) == (&iv_pAnnotatorContext->getTaeSpecifier()));
        // call implementation in subclass
        TyErrorId utErrorId = initializeImpl(specifier);
        if (utErrorId != UIMA_ERR_NONE) {
          destroy();
          return utErrorId;
        }

        utErrorId = initializeCompleteResultSpec();
        if (utErrorId != UIMA_ERR_NONE) {
          destroy();
          return utErrorId;
        }

        int numCasRequired =  getCasInstancesRequiredImpl();
        if (numCasRequired > 0) {
          utErrorId = iv_pAnnotatorContext->defineCASPool(numCasRequired);
        }

        return utErrorId;
      } catch (Exception & rclException) {
        getAnnotatorContext().getLogger().logError(rclException.getErrorInfo() );
        return rclException.getErrorInfo().getErrorId();
      }

    }


    TyErrorId EngineBase::reinitTypeSystem() {
      try {
        util::Trace                clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
        TyErrorId utErrorId = reinitTypeSystemImpl();
        if (utErrorId != UIMA_ERR_NONE) {
          destroy();
          return utErrorId;
        }

        // we have to re-initialize the result spec since the types and features
        // may have changed.
        utErrorId = initializeCompleteResultSpec();
        if (utErrorId != UIMA_ERR_NONE) {
          destroy();
          return utErrorId;
        }
        return utErrorId;
      } catch (Exception & rclException) {
        getAnnotatorContext().getLogger().logError(rclException.getErrorInfo() );
        return rclException.getErrorInfo().getErrorId();
      }

    }


    ResultSpecification const & EngineBase::getCompleteResultSpecification() const {
      return iv_completeResultSpec;
    }


    TyErrorId EngineBase::destroy() {
      try {
        util::Trace                clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);

        if (!checkAndSetCallingSequenceDestroy()) {
          return logError(clTrace, UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE);
        }
        return destroyImpl();
      } catch (Exception & rclException) {
        getAnnotatorContext().getLogger().logError(rclException.getErrorInfo() );
        return rclException.getErrorInfo().getErrorId();
      }
    }

    TyErrorId EngineBase::reconfigure() {
      try {
        util::Trace                clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);

        if (!checkAndSetCallingSequenceReconfigure()) {
          return logError(clTrace, UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE);
        }
        return reconfigureImpl();
      } catch (Exception & rclException) {
        getAnnotatorContext().getLogger().logError(rclException.getErrorInfo() );
        return rclException.getErrorInfo().getErrorId();
      }

    }


    TyErrorId EngineBase::process(CAS & cas) {
      try {
        AnalysisEngine * engine = this;
        return engine->process(cas, getCompleteResultSpecification());
      } catch (Exception & rclException) {
        getAnnotatorContext().getLogger().logError(rclException.getErrorInfo() );
        UIMA_EXC_RETHROW(rclException,NULL);
        ///return rclException.getErrorInfo().getErrorId();
      }

    }

    TyErrorId EngineBase::process(CAS & cas, ResultSpecification const & resultSpec) {
      try {
        util::Trace                clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);

        if (!checkAndSetCallingSequenceProcess()) {
          return logError(clTrace, UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE);
        }
        // (ee) WHY DO THIS CHECK???
        //checkCASCompatibility(cas);

        // Removed code that used to skip the processing if the document
        // was empty, since not all Sofas have local text. (bll)

        return processImpl(cas, resultSpec);
      } catch (Exception & rclException) {
        getAnnotatorContext().getLogger().logError(rclException.getErrorInfo() );
        //return rclException.getErrorInfo().getErrorId();
        UIMA_EXC_RETHROW(rclException,NULL);
      }

    }


    TyErrorId EngineBase::batchProcessComplete() {
      try {
        util::Trace                clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);

        if (!checkAndSetCallingSequenceProcess()) {
          return logError(clTrace, UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE);
        }
        return batchProcessCompleteImpl();
      } catch (Exception & rclException) {
        getAnnotatorContext().getLogger().logError(rclException.getErrorInfo() );
        return rclException.getErrorInfo().getErrorId();
      }

    }

    TyErrorId EngineBase::collectionProcessComplete() {
      try {
        util::Trace                clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);

        if (!checkAndSetCallingSequenceProcess()) {
          return logError(clTrace, UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE);
        }
        return collectionProcessCompleteImpl();
      } catch (Exception & rclException) {
        getAnnotatorContext().getLogger().logError(rclException.getErrorInfo() );
        return rclException.getErrorInfo().getErrorId();
      }

    }

    bool EngineBase::hasNext() {
      try {
        util::Trace                clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
        /** why do this ?? (bi)
        if (!checkAndSetCallingSequenceProcess()) {
           logError(clTrace, UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE);
           return false;
        } **/
        return hasNextImpl();
      } catch (Exception & rclException) {
        getAnnotatorContext().getLogger().logError(rclException.getErrorInfo() );
        throw rclException;
      }
    }

    CAS &  EngineBase::next() {
      try {
        util::Trace                clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
        /** why do this (bi)
        if (!checkAndSetCallingSequenceProcess()) {
           logError(clTrace, UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE);
        } **/
        return nextImpl();
      } catch (Exception & rclException) {
        getAnnotatorContext().getLogger().logError(rclException.getErrorInfo() );
        //return rclException.getErrorInfo().getErrorId();
        throw rclException;
      }
    }

    int  EngineBase::getCasInstancesRequired() {
      try {
        util::Trace                clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);

        if (!checkAndSetCallingSequenceProcess()) {
          logError(clTrace, UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE);
          return 0;
        }
        return getCasInstancesRequiredImpl();
      } catch (Exception & rclException) {
        getAnnotatorContext().getLogger().logError(rclException.getErrorInfo() );
        //return rclException.getErrorInfo().getErrorId();
        throw rclException;
      }
    }

    CASIterator EngineBase::processAndOutputNewCASes(CAS & cas) {
      try {
        util::Trace                clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);

        if (!checkAndSetCallingSequenceProcess()) {
          logError(clTrace, UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE);

          UIMA_EXC_THROW_NEW(CASIteratorException,
                             UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE,
                             UIMA_MSG_ID_EXCON_CALLING_ANNOTATOR_FUNCTION,
                             UIMA_MSG_ID_EXCON_CALLING_ANNOTATOR_FUNCTION,
                             ErrorInfo::unrecoverable);
        }
        TyErrorId rc = process(cas);
        if (rc != UIMA_ERR_NONE) {
          UIMA_EXC_THROW_NEW(CASIteratorException,
                             UIMA_ERR_PROCESS_OUTPUT_CAS,
                             UIMA_MSG_ID_EXCON_CALLING_ANNOTATOR_FUNCTION,
                             UIMA_MSG_ID_EXCON_CALLING_ANNOTATOR_FUNCTION,
                             ErrorInfo::unrecoverable);
        }
        return CASIterator(this);
      } catch (Exception & rclException) {
        getAnnotatorContext().getLogger().logError(rclException.getErrorInfo() );
        //return rclException.getErrorInfo().getErrorId();
        throw rclException;
      }
    }

    TyErrorId EngineBase::batchProcessCompleteImpl() {
      return UIMA_ERR_NOT_YET_IMPLEMENTED;
    }

    TyErrorId EngineBase::collectionProcessCompleteImpl() {
      return UIMA_ERR_NOT_YET_IMPLEMENTED;
    }

  }
}

/* ----------------------------------------------------------------------- */



