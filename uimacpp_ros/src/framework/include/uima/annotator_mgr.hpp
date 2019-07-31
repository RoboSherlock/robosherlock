/** \file annotator_mgr.hpp .
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

    \brief  Contains AnnotatorManager

   Description:

-----------------------------------------------------------------------------


   4/26/1999   Initial creation
   1/17/2000   Autom. priorisation of annotators added

-------------------------------------------------------------------------- */

#ifndef UIMA_ANNOTATOR_MGR_HPP
#define UIMA_ANNOTATOR_MGR_HPP

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> //must be included first to disable warnings
#include <vector>

#include <uima/annotator_timing.hpp>
#include <uima/exceptions.hpp>
#include <uima/timedatetools.hpp>

#include <uima/result_specification.hpp>
//#include "uima/internal_capability_container.hpp"

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
  class Annotator;
  class Language;
  class AnnotatorManager;
  class AnnotatorContext;
  class ResultSpecification;
  class AnalysisEngine;
  class FlowConstraints;
  namespace internal {
    class CapabilityContainer;
    class AggregateEngine;
    class PrimitiveEngine;
  }
  namespace util {
    class ConsoleUI;  // forward needed only for friend statements below
  }
}

class EnAnnotatorTimer;           // forward declaration
/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */


namespace uima {

  namespace internal {

/// @if internal

    /**
     * The class <TT>AnnotatorManager</TT> dispatches calls from the engine
     * to annotators.
     * This is really what is called "Annotation Structure Broker" (ASB) in UIMA.
     * (The name is still here only for historical reasons).
     */
    class UIMA_LINK_IMPORTSPEC AnnotatorManager {
    public:
      /** @name Constructors */
      /*@{*/
      /** Create a new instance of the AnnotatorManager. */
      AnnotatorManager(internal::AggregateEngine & );
      /*@}*/
      ~AnnotatorManager(void);

      /**
       * calls initialize() on all sub-engines
       */
      TyErrorId               launchInit();

      /**
       * calls reinitTypeSystem() on all sub-engines
       */
      TyErrorId launchReinitTypeSystem();

      /**
       * calls destroy() on all sub-engines
       */
      TyErrorId               launchDeInit();

      /**
       * calls reconfigure() on all sub-engines
       */
      TyErrorId               launchConfig();

      /**
       * calls process() on all sub-engines
       */
      TyErrorId               launchProcessDocument(CAS & cas, ResultSpecification const &);

      /**
             * calls batchProcessComplete on all sub-engines
             */
      TyErrorId               launchBatchProcessComplete();

      /**
             * calls collectionProcessComplete on all sub-engines
             */
      TyErrorId               launchCollectionProcessComplete();
      /*@}*/


#ifdef UIMA_DEBUG_ANNOTATOR_TIMING

      /** Support for timers (only compiled if DEBUG_TIMING is defined.
         */
      /*@{*/
      /**
       * Retrieves the number of annotators.
       *
       * @return the number of annotators loaded by the annotator manager
       *
       * @see getAnnotatorTimer
       * @see Engine::EnAnnotatorTimer
       * @see Engine::getAnnotatorTimer
       * @see Engine::getNbrOfAnnotators
       */
      size_t
      getNbrOfAnnotators( void ) const;

      /**
       * For all annotators: get timers for all major annotator
       * API functions.
       *
       * @param uiAnnotatorIndex
       *               The index specifying the annotator for which to get
       *               timing information.
       *               Must be between 0 and getNbrOfAnnotators()
       *
       * @param uiAnnotatorTimer
       *               An enum (Engine::EnAnnotatorTimers) specifying which
       *               timing information to return for annotator number uiAnnotatorIndex
       *
       * @return The timer recording the time spent in the function
       *         number uiAnnotatorTimer
       *         of
       *         annotator number uiAnnotatorIndex
       *
       * @see getNbrOfAnnotators
       * @see Engine::EnAnnotatorTimer
       * @see Engine::getAnnotatorTimer
       * @see Engine::getNbrOfAnnotators
       */
      Timer
      getAnnotatorTimer( size_t uiAnnotatorIndex,  size_t uiAnnotatorTimer ) const;

      /**
       * Return a descriptive name for the annotator number uiAnnotatorIndex
       *
       * @param uiAnnotatorIndex
       *               The index specifying the annotator for which to get
       *               timing information.
       *               Must be between 0 and getNbrOfAnnotators()
       *
       * @return Name of the annotator if 0 <= uiAnnotatorIndex < getNbrOfAnnotators(),
       *         NULL otherwise
       *
       * @see getNbrOfAnnotators
       */
//      const char * getAnnotatorName( size_t uiAnnotatorIndex ) const;
      /*@}*/
#endif //UIMA_DEBUG_ANNOTATOR_TIMING
    // protected:
      /* --- functions --- */
    // private:
    public:
#ifdef UIMA_COMP_REQ_PUBLIC_TYPES
    public:
#endif
      typedef struct _EngineEntry {
        AnalysisEngine * iv_pEngine;
        internal::CapabilityContainer * iv_pCapabilityContainer;
      }
      EngineEntry;
      /* --- types --- */
      typedef std::vector < EngineEntry > TyAnnotatorEntries;
    // private:
    public:
      friend class uima::internal::PrimitiveEngine;
      // the engine whic howns this annotator manager
      internal::AggregateEngine * iv_pEngine;
      /* --- variables --- */
      TyAnnotatorEntries            iv_vecEntries;
      bool                       iv_bIsInitialized;
      size_t                     iv_uiNbrOfDocsProcessed; // for timing statistics

      /* --- functions --- */
#ifdef UIMA_DEBUG_ANNOTATOR_TIMING
      Timer                      iv_clTimerLaunchInit;
      Timer                      iv_clTimerLaunchProcess;
      /** Dumps the timing data for each annotator to the trace.
          Called in destructor. */
      void                       dumpTimingData(void) const;
#endif
      /**
       * helper function which, given a capability container (i.e.
       * capaiblities of an engine), a result spec, and a language,
       * determines whether this an engine should be called (return value).
       * If yes, the last output parameter contains the TOFs to be passed
       * to the engine.
       */
      static bool shouldEngineBeCalled(internal::CapabilityContainer const &,
                                       ResultSpecification const & rResultSpec,
                                       Language const &,
                                       std::vector<TypeOrFeature>&) ;

      /* COPY CONSTRUCTOR NOT SUPPORTED */
      AnnotatorManager(const AnnotatorManager & ); //lint !e1704
      /* ASSIGNMENT OPERATOR NOT SUPPORTED */
      AnnotatorManager & operator=(const AnnotatorManager & crclObject);


    }
    ; /* AnnotatorManager */

/// @endif internal
  }
}
/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
#endif /* UIMA_ANNOTATOR_MGR_HPP */

/* <EOF> */

