/** \file internal_primitive_engine.cpp .
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
#include <uima/pragmas.hpp>
#include <uima/macros.h>
#include <uima/internal_primitive_engine.hpp>
#include <uima/capability.hpp>
#include <uima/casdefinition.hpp>
#include <uima/trace.hpp>
#include <uima/internal_capability_container.hpp>
#include <uima/lowlevel_typesystem.hpp>
#include <uima/annotator_mgr.hpp>
#include <uima/log.hpp>
#include <uima/msg.h>
#include <uima/cas.hpp>
#include <uima/filename.hpp>

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
using namespace std;
namespace uima {
  namespace internal {

    PrimitiveEngine::PrimitiveEngine(AnnotatorContext & rAnnContext, bool bOwnsANC,
                                     bool bOwnsTAESpec,
                                     uima::internal::CASDefinition & casdef, bool ownsCasDef,
                                     bool ) :
        EngineBase(rAnnContext, bOwnsANC, bOwnsTAESpec, casdef, ownsCasDef),
        iv_pAnnotator(NULL),
        iv_pCapContainer(NULL) {}

    PrimitiveEngine::~PrimitiveEngine() {
      destroyIfNeeded();
    }


    bool isTypeName(TCHAR const * cpName) {
      icu::UnicodeString target(cpName);
      int iIndex = target.indexOf((UChar)':');
      return iIndex == -1;
    }

    TyErrorId PrimitiveEngine::initializeImpl(AnalysisEngineDescription const & crEngineDescription) {
      util::Trace trace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      TyErrorId utErrorId = UIMA_ERR_NONE;
      UnicodeStringRef refDLLName(crEngineDescription.getAnnotatorImpName());
      string strAnnotatorName;

      refDLLName.extract( strAnnotatorName, CCSID::getDefaultName() );

      UnicodeStringRef refSymbolicName(crEngineDescription.getAnalysisEngineMetaData()->getName() );
      string strSymbolicName;
      refSymbolicName.extract( strSymbolicName, CCSID::getDefaultName() );


      util::Filename dllFileName( strAnnotatorName.c_str() );
      iv_pAnnotator = new AnnotatorProxy(dllFileName, strSymbolicName.c_str() );
      if ( iv_pAnnotator == NULL) {
        return UIMA_ERR_ENGINE_OUT_OF_MEMORY;
      }

      if (! iv_pAnnotator->isLoadedDelayed()) {
        utErrorId = iv_pAnnotator->load();
        if (utErrorId == UIMA_ERR_NONE) {
          utErrorId = iv_pAnnotator->init(getAnnotatorContext());
        }
      } else {
        assertWithMsg(false, "delayed loading not yet implemented");
        return UIMA_ERR_NOT_YET_IMPLEMENTED;
      }
      if (utErrorId != UIMA_ERR_NONE) {
        if (utErrorId == UIMA_ERR_ANNOTATOR_COULD_NOT_FIND ) {
          TyMessageId utErrorMsg = UIMA_MSG_ID_ANNOTATOR_COULD_NOT_FIND;
          ErrorMessage errMsg(utErrorMsg);
          errMsg.addParam(iv_pAnnotator->getFilename());
          ErrorInfo err(errMsg, utErrorId, ErrorInfo::unrecoverable);
          getAnnotatorContext().getLogger().logError(err);
        } else if (utErrorId == UIMA_ERR_ANNOTATOR_MISSING_CREATE) {
          TyMessageId utErrorMsg = UIMA_MSG_ID_ANNOTATOR_COULD_NOT_FIND_MAKEAE;
          ErrorMessage errMsg(utErrorMsg);
          errMsg.addParam(iv_pAnnotator->getFilename());
          if (iv_pAnnotator->getErrorMsg() != NULL) {
            errMsg.addParam(iv_pAnnotator->getErrorMsg());
          }
          ErrorInfo err(errMsg, utErrorId, ErrorInfo::unrecoverable);
          getAnnotatorContext().getLogger().logError(err);
        }  else if (utErrorId == UIMA_ERR_ANNOTATOR_COULD_NOT_LOAD) {
          TyMessageId utErrorMsg = UIMA_MSG_ID_ANNOTATOR_COULD_NOT_LOAD;
          ErrorMessage errMsg(utErrorMsg);
          errMsg.addParam(iv_pAnnotator->getFilename());
          if (iv_pAnnotator->getErrorMsg() != NULL) {
            errMsg.addParam(iv_pAnnotator->getErrorMsg());
          }
          ErrorInfo err(errMsg, utErrorId, ErrorInfo::unrecoverable);
          getAnnotatorContext().getLogger().logError(err);
        }
        // we do not need to log detailed messages for the other errors
        // like "could not init" this has been done already by the annotators
        destroy();
        return utErrorId;
      }

      iv_pCapContainer = new internal::CapabilityContainer();
      assert( EXISTS(iv_pCapContainer) );

      vector<Capability*> const & crCapabilities = getAnnotatorContext().getTaeSpecifier().getAnalysisEngineMetaData()->getCapabilites();

      UIMA_TPRINT("number capabilities: " << crCapabilities.size());
      assert( EXISTS(iv_casDefinition) );
      iv_pCapContainer->init(crCapabilities, iv_casDefinition->getTypeSystem());

      return typeSystemInitImpl();
    }


    TyErrorId PrimitiveEngine::typeSystemInitImpl() {
      util::Trace trace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      try {
        UIMA_TPRINT("Processing annotator: " << iv_pAnnotator->getSymbolicName() );
        assert( EXISTS(iv_pAnnotator) );
        return iv_pAnnotator->typeSystemInit(iv_casDefinition->getTypeSystem() );
      } catch (Exception & rclException) {
        getAnnotatorContext().getLogger().logError(rclException.getErrorInfo() );
        return rclException.getErrorInfo().getErrorId();
      }

    }

    TyErrorId PrimitiveEngine::processImpl(CAS & cas, ResultSpecification const & crResultSpec) {
      util::Trace trace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);

      UIMA_TPRINT("Processing annotator: " << iv_pAnnotator->getSymbolicName() );
      assert( EXISTS(iv_pAnnotator) );
      bool bCallAnnotator = true;
      DocumentFS docfs = cas.getDocumentAnnotation();
      if (isTopLevelEngine()&& docfs.isValid()) {
        vector<TypeOrFeature> tofsToBeRemoved;
        if (! AnnotatorManager::shouldEngineBeCalled(*iv_pCapContainer,
            crResultSpec,
            docfs.getLanguage(),
            tofsToBeRemoved) ) {
          bCallAnnotator = false;
        }
        AnnotatorContext & rANC = getAnnotatorContext();
        AnalysisEngineDescription const & crTAESpecifier = rANC.getTaeSpecifier();
        FlowConstraints const * pFlow = crTAESpecifier.getAnalysisEngineMetaData()->getFlowConstraints();
        if (pFlow == NULL) {
          bCallAnnotator = true;
        } else {
          FlowConstraints * flow = CONST_CAST(FlowConstraints *, pFlow);
          FlowConstraints::EnFlowType flowType = flow->getFlowConstraintsType();
          //if FixedFlow specified all engines are always called so reset callEngine is true
          if (flowType == FlowConstraints::FIXED) {
            bCallAnnotator = true;
          }
        }
      }
      if (bCallAnnotator) {
        // link CAS to current component
        cas.setCurrentComponentInfo(&getAnnotatorContext());
        try {
          iv_pAnnotator->processDocument(cas, crResultSpec);
        } catch (Exception e) {
          UIMA_EXC_RETHROW(e, NULL);
        }
      }
      // if annotator should not be called is not really an error
      return UIMA_ERR_NONE;
    }

    TyErrorId PrimitiveEngine::destroyImpl()  {
      util::Trace trace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);

      TyErrorId utErrorId = UIMA_ERR_NONE;
      if (iv_pAnnotator != NULL ) {
        if (iv_pAnnotator->isInitialized()) {
          assert( iv_pAnnotator->isValid() );
          utErrorId = iv_pAnnotator->deInit();
        }
        delete iv_pAnnotator;
        iv_pAnnotator = NULL;
      }
      if (iv_pCapContainer != NULL) {
        delete iv_pCapContainer;
        iv_pCapContainer = NULL;
      }
      return utErrorId;
    }

    TyErrorId PrimitiveEngine::reconfigureImpl() {
      util::Trace trace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      return iv_pAnnotator->reConfig();
    }

    TyErrorId PrimitiveEngine::batchProcessCompleteImpl() {
      util::Trace trace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      return iv_pAnnotator->batchProcessComplete();
    }

    TyErrorId PrimitiveEngine::collectionProcessCompleteImpl() {
      util::Trace trace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      return iv_pAnnotator->collectionProcessComplete();
    }

    bool PrimitiveEngine::hasNextImpl() {
      util::Trace trace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      return iv_pAnnotator->hasNext();
    }

    CAS & PrimitiveEngine::nextImpl() {
      util::Trace trace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      return iv_pAnnotator->next();
    }

    int PrimitiveEngine::getCasInstancesRequiredImpl() {
      util::Trace trace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      return iv_pAnnotator->getCasInstancesRequired();
    }

#ifdef UIMA_DEBUG_ANNOTATOR_TIMING

    void PrimitiveEngine::displayTimingData(util::ConsoleUI const & crConsole, bool bVerbose) const {
#ifndef NDEBUG
      crConsole.warning("timing data not yet available for primitive engines");
#endif
    }

#endif

  }
}

/* ----------------------------------------------------------------------- */



