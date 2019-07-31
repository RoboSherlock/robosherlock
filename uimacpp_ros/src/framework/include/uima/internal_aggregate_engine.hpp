#ifndef UIMA_INTERNAL_AGGREGATE_ENGINE_HPP
#define UIMA_INTERNAL_AGGREGATE_ENGINE_HPP
/** \file internal_aggregate_engine.hpp .
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

#include <uima/engine.hpp>
#include <uima/log.hpp>
#include <uima/language.hpp>
#include <uima/timedatetools.hpp>
#include <uima/annotator_timing.hpp> // for define UIMA_DEBUG_ANNOTATOR_TIMING
#include <uima/annotator_mgr.hpp>
#include <uima/annotator_context.hpp>
#include <uima/internal_engine_base.hpp>
#include <uima/consoleui.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  namespace internal {
    class CASDefinition;
  }
}
/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace internal {




    /**
     * The class <TT>AggregateEngine</TT> is the main UIMACPP engine which processes
     * document data, evaluates annotations, and other document-related data.
     * All results are stored the the uima::TCAS object.
     * @nosubgrouping
     * @see uima::TCAS
     */
    class UIMA_LINK_IMPORTSPEC AggregateEngine : public EngineBase {
    public:
      virtual char const * getInfoName() const {
        assert(false);
        return NULL;
      }

      virtual bool isPrimitive() const {
        return false;
      }


      AggregateEngine(AnnotatorContext &,
                      bool bOwnsANC,
                      bool bOwnsTAESpecififer,
                      uima::internal::CASDefinition & casDefinition,
                      bool ownsCASDefinition
                     );

      ~AggregateEngine(void);

      virtual TyErrorId initializeImpl(AnalysisEngineDescription const & crDescription);
      virtual TyErrorId reinitTypeSystemImpl();
      virtual TyErrorId processImpl(CAS & cas, ResultSpecification const &);
      virtual TyErrorId destroyImpl();
      virtual TyErrorId reconfigureImpl();
      virtual TyErrorId batchProcessCompleteImpl();
      virtual TyErrorId collectionProcessCompleteImpl();
      virtual bool      hasNextImpl();
      virtual CAS &     nextImpl();
      virtual int       getCasInstancesRequiredImpl();

#ifdef UIMA_DEBUG_ANNOTATOR_TIMING

      /**
       * Enum listing all annotator API functions for which timing information can
       * be retrieved using function getAnnotatorTimer
       */
      typedef enum {
        enLoadAnnotatorTime,              /// timer for function load annotator
        enInitAnnotatorTime,              /// timer for function init annotator
        enConfigAnnotatorTime,            /// timer for function config annotator
        enProcessAnnotatorTime,           /// timer for function process annotator
        enDeinitAnnotatorTime,            /// timer for function deinit annotator
        enTotalAnnotatorTime,             /// timer for sum of all annotator functions
        enNumberOfAnnotatorTimerEnums     /// must be last in enum
      }
      EnAnnotatorTimer;

      /** @name Support for timers.
          This is only compiled if UIMA_DEBUG_ANNOTATOR_TIMING is defined
          (currently active). */
      /*@{*/
      /**
       * Retrieves the number of loaded annotators.
       *
       * @return the number of annotators loaded by the annotator manager of the engine
       *
       * @see getAnnotatorTimer
       */
      virtual size_t          getNbrOfAnnotators( void ) const;
      /**
       * Retrieves the name of a given loaded annotator.
       *
       * @param uAnnotatorIndex
       *               The index specifying the annotator for which to get
       *               name information.
       *               Must be between 0 and getNbrOfAnnotators()
       *
       */
      virtual icu::UnicodeString const &  getAnnotatorName( size_t uiAnnotatorIndex ) const;
      /**
       * For all annotators: get timers for all major annotator
       * API functions.
       *
       * @param uAnnotatorIndex
       *               The index specifying the annotator for which to get
       *               timing information.
       *               Must be between 0 and getNbrOfAnnotators()
       *
       * @param enAnnotatorTimer
       *               An enum (Engine::EnAnnotatorTimers) specifying which
       *               timing information to return for annotator number uAnnotatorIndex
       *
       * @return The timer recording the time spent in the function
       *         number uAnnotatorTimer of annotator number enAnnotatorIndex
       *
       * @see getNbrOfAnnotators
       */
      virtual uima::Timer      getAnnotatorTimer( size_t uiAnnotatorIndex,  EnAnnotatorTimer enAnnotatorTimer ) const;

      Timer iv_timer;

      /** Return the time spent in load(). */
      virtual const Timer &              getLoadTimer(void) const {
        return iv_timer;
      }
      /** Return the time spent in init(). */
      virtual const Timer &              getInitTimer(void) const {
        return iv_timer;
      }
      /** Return the time spent in deInit(). */
      virtual const Timer &              getDeInitTimer(void) const {
        return iv_timer;
      }
      /** Return the time spent in config(). */
      virtual const Timer &              getConfigTimer(void) const {
        return iv_timer;
      }
      /** Return the time spent in processDocument(). */
      virtual const Timer &              getProcessDocumentTimer(void) const {
        return iv_timer;
      }

      virtual void displayTimingData(util::ConsoleUI const &, bool bVerbose = false) const;
      /*@}*/


      /** @} */
#endif // UIMA_DEBUG_ANNOTATOR_TIMING

      AnnotatorManager        iv_annotatorMgr;
    protected:
      /* --- functions --- */
    private:

      // AnnotatorManager        iv_annotatorMgr;

      /* --- functions --- */


      TyErrorId               init2(util::Trace & rclTrace);
      /* COPY CONSTRUCTOR NOT SUPPORTED */
      AggregateEngine(const AggregateEngine & ); //lint !e1704
      /* ASSIGNMENT OPERATOR NOT SUPPORTED */
      AggregateEngine & operator=(const AggregateEngine & crclObject);
    }
    ;                                                 /* AggregateEngine */


  }
}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif

