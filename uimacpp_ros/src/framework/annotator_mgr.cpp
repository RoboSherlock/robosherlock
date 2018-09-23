/** @name annotator_mgr.cpp
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

   Description: This file contains class AnnotatorManager (ASB in UIMA)

-----------------------------------------------------------------------------



-------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp>

//#define DEBUG_VERBOSE
#include <uima/macros.h>
#include <uima/annotator_mgr.hpp>
#include <uima/strconvert.hpp>
#include <uima/annotator.hpp>
///#include "uima/annotator_file.hpp"
#include <uima/comp_ids.h>
#include <uima/internal_aggregate_engine.hpp>
#include <uima/internal_primitive_engine.hpp>
#include <uima/log.hpp>
#include <uima/err_ids.h>
#include <uima/msg.h>

#include <uima/annotator_context.hpp>
#include <uima/cas.hpp>

#include <uima/trace.hpp>
#include <uima/assertmsg.h>

#include <uima/engine.hpp>
#include <uima/result_specification.hpp>
#include <uima/internal_capability_container.hpp>
#include <uima/casdefinition.hpp>
#include <uima/lowlevel_typesystem.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

#define UIMA_ANNOTATOR_MGR_DEFAULT_PROCESS_UNSUPPORTED_LANGUAGES   false
#define UIMA_ANNOTATOR_MGR_DEFAULT_IGNORE_PROCESSDOCUMENT_ERRORS   false
#define UIMA_ANNOTATOR_MGR_DEFAULT_PREVENT_DUPLICATE_TARGET_TYPES  true

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Private Implementation                                            */
/* ----------------------------------------------------------------------- */
using namespace std;
namespace uima {

  namespace internal {

    /* ----------------------------------------------------------------------- */
    /*       Implementation                                                    */
    /* ----------------------------------------------------------------------- */

    AnnotatorManager::AnnotatorManager(internal::AggregateEngine & rEngine) :
        iv_pEngine(& rEngine),
        iv_vecEntries(),
        iv_bIsInitialized(false),
        iv_uiNbrOfDocsProcessed(0)
        /* ----------------------------------------------------------------------- */{
      ;
    }

    AnnotatorManager::~AnnotatorManager(void)
    /* ----------------------------------------------------------------------- */{
      util::Trace                 clTrace(util::enTraceDetailHigh, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);

      if (! iv_vecEntries.empty() ) {
        launchDeInit();
      }
      assert( iv_vecEntries.empty() );

    }


#ifdef DEBUG_VERBOSE
    void printCapabilityContainer(ostream & os, uima::Language const & crLanguage, uima::internal::CapabilityContainer const & crCapContainer) {
      os << " Language: " << crLanguage.asString() << endl;
      os << " CapContainer: "  << endl;

      internal::CapabilityContainer & nonConstCAP = (internal::CapabilityContainer &) crCapContainer;
      internal::CapabilityContainer::TySetTypeOrFeatures const & outTOFs = nonConstCAP.getOutputTypeOrFeatures( crLanguage );
      internal::CapabilityContainer::TySetTypeOrFeatures::const_iterator citTOFS;
      for (citTOFS = outTOFs.begin(); citTOFS != outTOFs.end(); ++citTOFS) {
        if ( (*citTOFS).isType() ) {
          os << "  type: " ;
        } else {
          os << " feat: ";
        }
        os << (*citTOFS).getName() << endl;
      }

    }

#endif

    bool containsTOF(TypeOrFeature const & crTOF, uima::Language const & crLanguage, uima::internal::CapabilityContainer const & crCapContainer) {
      internal::CapabilityContainer & nonConstCAP = (internal::CapabilityContainer &) crCapContainer;
      internal::CapabilityContainer::TySetTypeOrFeatures const & outTOFs = nonConstCAP.getOutputTypeOrFeatures( crLanguage );
      internal::CapabilityContainer::TySetTypeOrFeatures::const_iterator citTOFS;
      for (citTOFS = outTOFs.begin(); citTOFS != outTOFs.end(); ++citTOFS) {
        if (crTOF == (*citTOFS) ) {
          return true;
        }
      }
      return false;
    }



    TyErrorId AnnotatorManager::launchInit()
    /* ----------------------------------------------------------------------- */{
      util::Trace                 clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);
      UIMA_ANNOTATOR_TIMING(iv_clTimerLaunchInit.start());
      TyErrorId               utErrorId = UIMA_ERR_NONE;

      assert( !iv_bIsInitialized );
      assert(iv_vecEntries.empty());

      AnnotatorContext & rANC = iv_pEngine->getAnnotatorContext();

      AnalysisEngineDescription const & crTAESpecifier = rANC.getTaeSpecifier(); // this method must be added

      assert( ! crTAESpecifier.isPrimitive() );
      //BSIvector < icu::UnicodeString > const & crVecEngineNames = crTAESpecifier.getAnalysisEngineMetaData()->getFixedFlow()->getNodes();

      vector < icu::UnicodeString > const & crVecEngineNames = crTAESpecifier.getAnalysisEngineMetaData()->getFlowConstraints()->getNodes();

      // for all engines in the flow
      size_t ui;
      for (ui=0; ui<crVecEngineNames.size(); ++ui) {

        icu::UnicodeString const & crEngineName = crVecEngineNames[ui];
//         clTrace.dump(_TEXT("Delegate name"), crEngineName);

        AnnotatorContext * pDelegateANC = rANC.getDelegate(crEngineName);
        if (pDelegateANC == NULL)
          cerr << "Cannot find engine: " << crEngineName << " in the given text analysis engine file." << endl;
        ErrorInfo errInfo;
        // create a delegate TAE which owns neither the ANC, nor the TAESpecifier, nor the CAS definition
        AnalysisEngine * pEngine = Framework::createAnalysisEngine(*pDelegateANC,
                                       false,
                                       false,
                                       iv_pEngine->getCASDefinition(),
                                       false,
                                       errInfo);
        utErrorId = errInfo.getErrorId();
        if (utErrorId != UIMA_ERR_NONE) {
          assert( pEngine == NULL );
          iv_pEngine->getAnnotatorContext().getLogger().logError(errInfo);
          return utErrorId;
        }
        assert( EXISTS(pEngine) );

        EngineEntry entry;
        entry.iv_pEngine = pEngine;

        // create a capability container for each engine (the cap. container is used for faster lookup)
        uima::internal::CapabilityContainer * pCapContainer = new internal::CapabilityContainer();
        assert( EXISTS(pCapContainer) );
        entry.iv_pCapabilityContainer = pCapContainer;

        vector<Capability*> const & crCapabilities = pDelegateANC->getTaeSpecifier().getAnalysisEngineMetaData()->getCapabilites();

        UIMA_TPRINT("-------------------------------------------------");
        UIMA_TPRINT("Engine name: " << crEngineName);
        UIMA_TPRINT("number of capabilities: " << crCapabilities.size());
        pCapContainer->init(crCapabilities, iv_pEngine->getCASDefinition().getTypeSystem() );

#ifdef DEBUG_VERBOSE
        /*
                 UIMA_TPRINT("capabilities:");
                 vector<Capability*>::const_iterator cit;
                 for (cit = crCapabilities.begin(); cit != crCapabilities.end(); ++cit) {
                    Capability* cap = (*cit);
                    vector<icu::UnicodeString> const & outCapT = cap->getCapabilityTypes(Capability::OUTPUT);
                    size_t i;
                    for (i=0; i<outCapT.size(); ++i) {
                       UIMA_TPRINT("  type: " << outCapT[i]);
                    }

                    vector<icu::UnicodeString> const & outCapF = cap->getCapabilityFeatures(Capability::OUTPUT);
                    for (i=0; i<outCapF.size(); ++i) {
                       UIMA_TPRINT("  feat: " << outCapF[i]);
                    }

                    vector<icu::UnicodeString> const  & suppLangs = cap->getSupportedLanguages();
                    for (i=0; i<suppLangs.size(); ++i) {
                       UIMA_TPRINT("  lang: " << suppLangs[i]);
                    }

                 }
        */
        UIMA_TPRINT("cap container for fr");
        printCapabilityContainer(cout, uima::Language("fr"), *pCapContainer);
        UIMA_TPRINT("  is like dump ? " << pCapContainer->hasEmptyOutputTypeOrFeatures(uima::Language("en") ));
#endif

        /* add to set of delegate TAEs as managed by the AnnotatorManager */
        iv_vecEntries.insert(iv_vecEntries.end(), entry);
      }

      UIMA_ANNOTATOR_TIMING(iv_clTimerLaunchInit.stop());

      iv_bIsInitialized = true;
      return(utErrorId);
    }


    TyErrorId AnnotatorManager::launchDeInit()
    /* ----------------------------------------------------------------------- */{
      util::Trace                 clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);
      TyAnnotatorEntries::reverse_iterator it;           /* deinit must be performed in reverse order! */
      TyErrorId               utErrorId = UIMA_ERR_NONE;
      TyErrorId               utRetVal = UIMA_ERR_NONE;

#ifdef UIMA_DEBUG_ANNOTATOR_TIMING
      if (clTrace.isEnabled() && iv_uiNbrOfDocsProcessed > 0) {
        dumpTimingData();
      }
#endif

      /* iterate through ALL Annotators in reverse order */
      for (it = iv_vecEntries.rbegin(); !(it == iv_vecEntries.rend()); ++it) {
        EngineEntry & rEntry =  (*it);
        AnalysisEngine * pEngine = rEntry.iv_pEngine;
        assert(EXISTS(pEngine));
        if (pEngine->isInitialized()) {
          /* use the annotator */
          utErrorId = pEngine->destroy();
          if (utErrorId != UIMA_ERR_NONE) {
            clTrace.dump(_TEXT("Error"), (long) utErrorId);
            utRetVal = utErrorId;                     /* I know, this overwrites a previous error */
          }
          /* we do not return in the case of an error as we want to be sure that
             ALL annotators will be deinitialized */
        }
      } /* e-o-for */
      for (it = iv_vecEntries.rbegin(); !(it == iv_vecEntries.rend()); ++it) {
        EngineEntry & rEntry =  (*it);
        AnalysisEngine * pEngine = rEntry.iv_pEngine;
        assert(EXISTS(pEngine));
        internal::CapabilityContainer * pCapContainer = rEntry.iv_pCapabilityContainer;
        assert(EXISTS(pCapContainer));
        delete pEngine;
        delete pCapContainer;
      }

      iv_vecEntries.clear();
      return(utRetVal);
    }


    TyErrorId AnnotatorManager::launchReinitTypeSystem() {
      util::Trace                 clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);
      size_t ui;
      TyErrorId               utErrorId = UIMA_ERR_NONE;
      TyErrorId               utRetVal = UIMA_ERR_NONE;

      for (ui=0; ui<iv_vecEntries.size(); ++ui) {
        uima::internal::EngineBase & engine = uima::internal::EngineBase::promoteEngine( * iv_vecEntries[ui].iv_pEngine );
        utErrorId = engine.reinitTypeSystem();
        if (utErrorId != UIMA_ERR_NONE) {
          clTrace.dump(_TEXT("Error in launchReinitTypeSystem()"), (long) utErrorId);
          utRetVal = utErrorId;                        /* I know, this overwrites a previous error */
        }
        // create new capability containers
        assert( EXISTS(iv_vecEntries[ui].iv_pCapabilityContainer) );
        delete iv_vecEntries[ui].iv_pCapabilityContainer;

        uima::internal::CapabilityContainer * capContainer = new uima::internal::CapabilityContainer();
        assert( EXISTS(capContainer) );
        capContainer->init( engine.getAnalysisEngineMetaData().getCapabilites(),
                            engine.getCASDefinition().getTypeSystem() );

        iv_vecEntries[ui].iv_pCapabilityContainer = capContainer;

      }
      return utRetVal;
    }

    TyErrorId AnnotatorManager::launchConfig()
    /* ----------------------------------------------------------------------- */{
      util::Trace                 clTrace(util::enTraceDetailMedium, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);
      TyAnnotatorEntries::iterator it;
      TyErrorId               utErrorId = UIMA_ERR_NONE;
      TyErrorId               utRetVal = UIMA_ERR_NONE;

      assert(iv_bIsInitialized);
      /* iterate through ALL Annotators */
      assert(!iv_vecEntries.empty());
      for (it = iv_vecEntries.begin(); it != iv_vecEntries.end(); ++it) {
        EngineEntry & rEntry =  (*it);
        AnalysisEngine * pEngine = rEntry.iv_pEngine;
        assert(EXISTS(pEngine));

//         clTrace.dump(_TEXT("Annotator"), pEngine->getSymbolicName().c_str());
        /* use the annotator */
        utErrorId = pEngine->reconfigure();
        if (utErrorId != UIMA_ERR_NONE) {
          clTrace.dump(_TEXT("Error"), (long) utErrorId);
          utRetVal = utErrorId;                        /* I know, this overwrites a previous error */
        }
        /* eben if an error occurs, we give all others a chance to reconfig */
      } /* e-o-for */
      return(utRetVal);
    }



    TyErrorId AnnotatorManager::launchBatchProcessComplete() {
      util::Trace                  clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);
      size_t ui;
      TyErrorId               utErrorId = UIMA_ERR_NONE;
      TyErrorId               utRetVal = UIMA_ERR_NONE;

      for (ui=0; ui<iv_vecEntries.size(); ++ui) {
        uima::internal::EngineBase & engine = uima::internal::EngineBase::promoteEngine( * iv_vecEntries[ui].iv_pEngine );
        utErrorId = engine.batchProcessComplete();
        if (utErrorId != UIMA_ERR_NONE) {
          clTrace.dump(_TEXT("Error in launchBatchProcessComplete()"), (long) utErrorId);
          utRetVal = utErrorId;                        /* I know, this overwrites a previous error */
        }
      }
      return utRetVal;
    }


    TyErrorId AnnotatorManager::launchCollectionProcessComplete() {
      util::Trace                  clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);
      size_t ui;
      TyErrorId               utErrorId = UIMA_ERR_NONE;
      TyErrorId               utRetVal = UIMA_ERR_NONE;

      for (ui=0; ui<iv_vecEntries.size(); ++ui) {
        uima::internal::EngineBase & engine = uima::internal::EngineBase::promoteEngine( * iv_vecEntries[ui].iv_pEngine );
        utErrorId = engine.collectionProcessComplete();
        if (utErrorId != UIMA_ERR_NONE) {
          clTrace.dump(_TEXT("Error in launchCollectionProcessComplete()"), (long) utErrorId);
          utRetVal = utErrorId;                        /* I know, this overwrites a previous error */
        }
      }
      return utRetVal;
    }





    bool AnnotatorManager::shouldEngineBeCalled(uima::internal::CapabilityContainer const & crCapContainer,
        ResultSpecification const & rResultSpec,
        Language const & crLanguage,
        vector<TypeOrFeature> & rTOFSToBeRemoved) {
      util::Trace                 clTrace(util::enTraceDetailHigh, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);

#ifdef DEBUG_VERBOSE
      UIMA_TPRINT("CapContainer:");
      printCapabilityContainer(cout, crLanguage, crCapContainer);
      rResultSpec.print(cout);

      /*
      uima::Language unspecLang(Language::UNSPECIFIED);

      internal::CapabilityContainer & nonConstCAP = (internal::CapabilityContainer &) crCapContainer;
      internal::CapabilityContainer::TySetTypeOrFeatures const & outTOFs = nonConstCAP.getOutputTypeOrFeatures( unspecLang );
      internal::CapabilityContainer::TySetTypeOrFeatures::const_iterator citTOFS;
      UIMA_TPRINT("Capabilities for unspecified language");
      for (citTOFS = outTOFs.begin(); citTOFS != outTOFs.end(); ++citTOFS) {
         UIMA_TPRINT("  " << (*citTOFS).getName() );
      }
      */
#endif

      // treat dump-like annotators for this language specially
      if (crCapContainer.hasEmptyOutputTypeOrFeatures( crLanguage )) {
        return true;
      }

      ResultSpecification::TyTypeOrFeatureSTLSet const & crTOFSet = rResultSpec.getTypeOrFeatureSTLSet();
      bool bHasTOF = false;
      ResultSpecification::TyTypeOrFeatureSTLSet::const_iterator cit;
      for (cit = crTOFSet.begin(); cit != crTOFSet.end(); ++cit) {
        TypeOrFeature const & crTOF = (*cit);
        assert( (*cit).isValid() );
        assert( crTOF.isValid() );
        assert( rResultSpec.contains( crTOF ) );

        UIMA_TPRINT("  TOF Name: " << crTOF.getName());

        if (crCapContainer.hasOutputTypeOrFeature(crTOF, crLanguage)) {
          assert( containsTOF(crTOF, crLanguage, crCapContainer) );
          UIMA_TPRINT( "    in capability" );
          bHasTOF = true;
          rTOFSToBeRemoved.push_back(crTOF);
        } else {
          assert( ! containsTOF(crTOF, crLanguage, crCapContainer) );
          UIMA_TPRINT("     not in capability");
        }
      }
      return bHasTOF;
    }





    TyErrorId AnnotatorManager::launchProcessDocument(CAS & cas, ResultSpecification const & crResultSpec) {
      /*
      This works as follows:
      The passes result spec is copied and for each delegate AE, it is determined
      (via shouldEngineBeCalled()) which TOFs of the result spec should be passed to the
      TAE (if it should be called at all). After the call to the delegate, those TOFs are
      removed from the copied result spec. At the end, this result spec is then empty (ideally).

      Note that for this to work, you have to add all needed intermediate results to the process()
      call on the TAE even if those are not the results the application is interested in.
      E.g., if I have a tokenizer and a summarizer and my aplication is only interested in summaries,
      it nonetheless must sepcify that it needs tokens, sentences, and paragraphs because this is what
      the summarizer needs as input.
      */
      util::Trace                 clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ANNOTATOR_MGR);
      UIMA_ANNOTATOR_TIMING(iv_clTimerLaunchProcess.start());
      TyAnnotatorEntries::iterator it;
      TyErrorId               utErrorId = UIMA_ERR_NONE;
      TyErrorId               utRetVal = UIMA_ERR_NONE;
      assert( EXISTS(iv_pEngine) );
      size_t                     uiNbrOfSkippedAnnotators = 0;
      CAS * tcas=NULL;

      // copy the result spec
      ResultSpecification resSpec = crResultSpec;

      ++iv_uiNbrOfDocsProcessed;
      assert(iv_bIsInitialized);

      assert(!iv_vecEntries.empty());
      for (it = iv_vecEntries.begin(); it != iv_vecEntries.end(); ++it) {
        EngineEntry & rEntry =  (*it);
        AnalysisEngine * pEngine = rEntry.iv_pEngine;
        uima::internal::CapabilityContainer * pCapContainer = rEntry.iv_pCapabilityContainer;
        assert(EXISTS(pEngine));
        assert(EXISTS(pCapContainer));

#ifdef DEBUG_VERBOSE
        UIMA_TPRINT("=========================================");
        UIMA_TPRINT("----------- ResultSpec before process():");
        resSpec.print(cout);
#endif

        UIMA_TRACE_STREAM_ARG(clTrace, "ASB checks engine", pEngine->getAnalysisEngineMetaData().getName() );

        UIMA_TPRINT("--------- Checking annotator: " << pEngine->getAnalysisEngineMetaData().getName());
        vector<TypeOrFeature> tofsToBeRemoved;
        bool callEngine=true;
        bool requiresTCas=true;

        if (cas.isBackwardCompatibleCas()) {
	  tcas = &cas;
		}
          //this populates the tofsToBeRemoved vector so always call it
          callEngine = shouldEngineBeCalled(*pCapContainer,
                                            resSpec,
                                            cas.getDocumentAnnotation().getLanguage(),
                                            tofsToBeRemoved);
          //check the FlowConstraintType specified in the aggregate engine
          //if CapabilityLanguageFlow whether engine is called is
          //determined by shouldEngineBeCalled()
          AnnotatorContext & rANC = iv_pEngine->getAnnotatorContext();
          AnalysisEngineDescription const & crTAESpecifier = rANC.getTaeSpecifier();
          FlowConstraints const * pFlow = crTAESpecifier.getAnalysisEngineMetaData()->getFlowConstraints();
          FlowConstraints * flow = CONST_CAST(FlowConstraints *, pFlow);
          FlowConstraints::EnFlowType flowType = flow->getFlowConstraintsType();

          //if FixedFlow specified all engines are always called so reset callEngine is true
          if (flowType == FlowConstraints::FIXED) {
            callEngine=true;
          }
        

        if ( callEngine ) {

          UIMA_TPRINT("----------- engine will be processed");
          UIMA_TRACE_STREAM(clTrace, "Engine will be called");

          // create ResultSpec for annotator
          // this must be done because an annotator should only be called with the result spec
          // that its XML file indicates.
          ResultSpecification annResSpec;
          vector<TypeOrFeature>::const_iterator citTOF;
          for (citTOF = tofsToBeRemoved.begin(); citTOF != tofsToBeRemoved.end(); ++citTOF) {
            assert( (*citTOF).isValid() );
            annResSpec.add(*citTOF);
            UIMA_TRACE_STREAM_ARG(clTrace, "    engine is called with result spec", (*citTOF).getName() );
          }

          /// does engine expect a TCas
          //AEs that declare at least one input or output SofA should be sent the base CAS.
          //Otherwise they must be sent a TCAS.
          const AnalysisEngineMetaData::TyVecpCapabilities & vecCap = pEngine->getAnalysisEngineMetaData().getCapabilites();
          AnalysisEngineMetaData::TyVecpCapabilities::const_iterator itCap;
          for (size_t i=0; i < vecCap.size(); i++) {
            Capability * cap = vecCap.at(i);
            Capability::TyVecCapabilitySofas inputSofa = cap->getCapabilitySofas(Capability::INPUTSOFA);
            Capability::TyVecCapabilitySofas outputSofa = cap->getCapabilitySofas(Capability::OUTPUTSOFA);
            if (inputSofa.size() > 0 || outputSofa.size() > 0) {
              requiresTCas = false;
              break;
            }
          }

          if (requiresTCas) {
	    SofaFS defSofa = cas.getSofa(pEngine->getAnnotatorContext().mapToSofaID(CAS::NAME_DEFAULT_TEXT_SOFA));
	    if (!defSofa.isValid()) {
	      //TODO: throw exception
	      cerr << "could not get default text sofa " << endl;
	      return 99;
	    }
	    tcas = cas.getView(defSofa);
	    utErrorId = pEngine->process(*tcas, annResSpec);
          } else {
            utErrorId = ((AnalysisEngine*) pEngine)->process(cas, annResSpec);
          }

          if (utErrorId != UIMA_ERR_NONE) {
            clTrace.dump(_TEXT("Error"), (long) utErrorId);
            utRetVal = utErrorId;                  /* I know, this overwrites a previous error */
          } else {
            // now remove TOFs from ResultSpec
            vector<TypeOrFeature>::const_iterator citTOF;
            for (citTOF = tofsToBeRemoved.begin(); citTOF != tofsToBeRemoved.end(); ++citTOF) {
              assert( (*citTOF).isValid() );
              resSpec.remove(*citTOF);
            }
          }
        } else {
          assert( tofsToBeRemoved.empty() );
          UIMA_TPRINT("----------- engine will *not* be processed");
          ++uiNbrOfSkippedAnnotators;
        }
      }                                               /* e-o-for */
      /* in case there was no error but not any annotator which generates a target type
         has been caled for process, we have an error */
      UIMA_TPRINT("Annotators skipped due to unsupport lang: " << uiNbrOfSkippedAnnotators);
      UIMA_TPRINT("Overall number of annotators: " << iv_vecEntries.size() );

      if (   (utRetVal == UIMA_ERR_NONE)
             && (uiNbrOfSkippedAnnotators > 0)
             && (uiNbrOfSkippedAnnotators == iv_vecEntries.size())
             && (crResultSpec.getSize() > 0) ) {
        // utRetVal = UIMA_ERR_ANNOTATOR_MGR_LANG_NOT_SUPPORTED_FOR_ANNOTATOR;
        iv_pEngine->getAnnotatorContext().getLogger().logWarning("All annotators skipped (maybe unsupported language)");
      }

      UIMA_ANNOTATOR_TIMING(iv_clTimerLaunchProcess.stop());
      return(utRetVal);
    }

#ifdef UIMA_DEBUG_ANNOTATOR_TIMING
    void AnnotatorManager::dumpTimingData( void ) const
      /* ----------------------------------------------------------------------- */ {
      util::Trace                 clTrace(util::enTraceDetailMedium, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_TIMING);

      if (clTrace.isEnabled() && iv_uiNbrOfDocsProcessed > 0) {
        TyAnnotatorEntries::const_iterator it;
        string                 strTemp;
        double                  dTotalLoadTime            = (double)0.0;
        double                  dTotalInitTime            = (double)0.0;
        double                  dTotalDeInitTime          = (double)0.0;
        double                  dTotalConfigTime          = (double)0.0;
        double                  dTotalProcessDocumentTime = (double)0.0;
        double                  dTotalAnnotatorTime          = (double)0.0;

        assert(iv_bIsInitialized);
        assert(!iv_vecEntries.empty());
        // first sum up times to get totals
        for (it = iv_vecEntries.begin(); it != iv_vecEntries.end(); ++it) {
          EngineEntry const & rEntry = (*it);
          AnalysisEngine * pEngine = rEntry.iv_pEngine;

          dTotalLoadTime           += pEngine->getLoadTimer().getAccumulatedTime();
          dTotalInitTime           += pEngine->getInitTimer().getAccumulatedTime();
          dTotalDeInitTime         += pEngine->getDeInitTimer().getAccumulatedTime();
          dTotalConfigTime         += pEngine->getConfigTimer().getAccumulatedTime();
          dTotalProcessDocumentTime+= pEngine->getProcessDocumentTimer().getAccumulatedTime();
        }                                            /* e-o-for */
        dTotalAnnotatorTime = dTotalLoadTime + dTotalInitTime + dTotalDeInitTime + dTotalConfigTime + dTotalProcessDocumentTime;

        clTrace.dump(_TEXT("Total timing data"));
        long2String((long)iv_uiNbrOfDocsProcessed, strTemp);
        clTrace.dump(_TEXT("  Documents processed "), strTemp.c_str());
        clTrace.dump(_TEXT("  Total annotator time   "), Timer::timeString(dTotalAnnotatorTime).c_str());
        assert(iv_uiNbrOfDocsProcessed > 0);
        assert(dTotalAnnotatorTime > 0);
        // we have to make sure our values stay in range for very quick processing times
        strTemp = " > FLT_MAX ";
        if (dTotalAnnotatorTime > FLT_MIN) {                                            // use FLT_MIN instead of DBL_MIN to have some buffer
          double               dThroughput = iv_uiNbrOfDocsProcessed / dTotalAnnotatorTime;

          if (dThroughput < FLT_MAX) {
            double2String(dThroughput, (size_t)1, strTemp);
          }
        }
        clTrace.dump(_TEXT("  Total throughput     "), (strTemp + " docs/sec").c_str());
        clTrace.dump(_TEXT("  PlgMgr init time     "), iv_clTimerLaunchInit.timeString().c_str());
        clTrace.dump(_TEXT("  PlgMgr process time  "), iv_clTimerLaunchProcess.timeString().c_str());
        clTrace.dump(_TEXT("- Load Time            "), Timer::timeString(dTotalLoadTime).c_str());
        clTrace.dump(_TEXT("- Init Time            "), Timer::timeString(dTotalInitTime).c_str());
        clTrace.dump(_TEXT("- Config Time          "), Timer::timeString(dTotalConfigTime).c_str());
        clTrace.dump(_TEXT("- ProcessDocument Time "), Timer::timeString(dTotalProcessDocumentTime).c_str());
        clTrace.dump(_TEXT("- DeInit Time          "), Timer::timeString(dTotalDeInitTime).c_str());

        // now dump the individual, total  and relative times
        for (it = iv_vecEntries.begin(); it != iv_vecEntries.end(); ++it) {
          EngineEntry const & rEntry =  (*it);
          AnalysisEngine * pEngine = rEntry.iv_pEngine;

          assert(EXISTS(pEngine));
          clTrace.dump(_TEXT("Timing data for engine "));
          clTrace.dump(_TEXT("- Load Time            "), pEngine->getLoadTimer().timeAndPercentString(dTotalLoadTime).c_str());
          clTrace.dump(_TEXT("- Init Time            "), pEngine->getInitTimer().timeAndPercentString(dTotalInitTime).c_str());
          clTrace.dump(_TEXT("- Config Time          "), pEngine->getConfigTimer().timeAndPercentString(dTotalConfigTime).c_str());
          clTrace.dump(_TEXT("- ProcessDocument Time "), pEngine->getProcessDocumentTimer().timeAndPercentString(dTotalProcessDocumentTime).c_str());
          clTrace.dump(_TEXT("- DeInit Time          "), pEngine->getDeInitTimer().timeAndPercentString(dTotalDeInitTime).c_str());
        }                                            /* e-o-for */
      }
    }

    size_t
    AnnotatorManager::getNbrOfAnnotators( void ) const {
      return iv_vecEntries.size();
    }

    Timer
    AnnotatorManager::getAnnotatorTimer( size_t uiAnnotatorIndex,  size_t uiAnnotatorTimer) const {
      if (uiAnnotatorIndex >= iv_vecEntries.size()) {
        return iv_clTimerLaunchProcess;
      }
      assert(uiAnnotatorTimer < internal::AggregateEngine::enNumberOfAnnotatorTimerEnums);
      internal::AggregateEngine::EnAnnotatorTimer enAnnotatorTimer = (internal::AggregateEngine::EnAnnotatorTimer)uiAnnotatorTimer;
      EngineEntry const & rEntry = iv_vecEntries[uiAnnotatorIndex];
      AnalysisEngine * pEngine = rEntry.iv_pEngine;
      switch (enAnnotatorTimer) {
      case internal::AggregateEngine::enLoadAnnotatorTime    :
        return pEngine->getLoadTimer();
      case internal::AggregateEngine::enInitAnnotatorTime    :
        return pEngine->getInitTimer();
      case internal::AggregateEngine::enConfigAnnotatorTime  :
        return pEngine->getConfigTimer();
      case internal::AggregateEngine::enProcessAnnotatorTime :
        return pEngine->getProcessDocumentTimer();
      case internal::AggregateEngine::enDeinitAnnotatorTime  :
        return pEngine->getDeInitTimer();
      case internal::AggregateEngine::enTotalAnnotatorTime   :
        return(pEngine->getLoadTimer() +
               pEngine->getInitTimer() +
               pEngine->getConfigTimer() +
               pEngine->getProcessDocumentTimer() +
               pEngine->getDeInitTimer());
      default:
        assert(false);
      }
      assert(false);
      return iv_clTimerLaunchProcess; // to satisfy compiler: never reached
    }


#endif //UIMA_DEBUG_ANNOTATOR_TIMING


  }

}

/* <EOF> */




