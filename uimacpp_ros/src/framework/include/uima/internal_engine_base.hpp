#ifndef UIMA_INTERNAL_ENGINE_BASE_HPP
#define UIMA_INTERNAL_ENGINE_BASE_HPP
/** \file internal_engine_base.hpp .
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
#include <uima/result_specification.hpp>
#include <uima/engine_state.hpp>
#include <uima/internal_casimpl.hpp>
#include <uima/casiterator.hpp>

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
  namespace util {
    class Trace;
  }
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */
namespace uima {
  namespace internal {

    /**
     * EngineBase shares common code for all concrete analysis engines
     * (i.e. primitive, aggregate, and JEDII)
     */
    class UIMA_LINK_IMPORTSPEC EngineBase : public AnalysisEngine {
    private:
      TyErrorId initializeCompleteResultSpec();

      bool checkAndSetCallingSequenceInitialize();
//         bool checkAndSetCallingSequenceTypeSystemInit();
      bool checkAndSetCallingSequenceDestroy();
      bool checkAndSetCallingSequenceProcess();
      bool checkAndSetCallingSequenceReconfigure();

      // checks if a CAS is compatible to the type system and index definitions
      // of this engine, throws an exception if not
      void checkCASCompatibility(CAS const &) const;
    protected:
      AnnotatorContext *        iv_pAnnotatorContext;
      bool iv_bOwnsANC;
      bool iv_bOwnsTAESpec;

      uima::internal::CASDefinition * iv_casDefinition;
      bool iv_ownsCASDefinition;

      // hold CAS for old-style TCAS applications which now get a TCAS view
      CAS* iv_casHolder;

      ResultSpecification iv_completeResultSpec;
      EngineState             iv_state;

      EngineBase(AnnotatorContext & rANC, bool bOwnsANC,
                 bool bOwnsTAESpecififer,
                 uima::internal::CASDefinition & casDefinition, bool ownsCASDefinition);

      ~EngineBase();

      // helper function
      bool isTopLevelEngine() const {
        return iv_bOwnsANC;
      }

      // subclasses must implement the following methods.
      // it can be assumed that the calling sequence is correct. Error checking need not be done.
      virtual TyErrorId initializeImpl(AnalysisEngineDescription const &) = 0;
      virtual TyErrorId reinitTypeSystemImpl() = 0;
      virtual TyErrorId destroyImpl() = 0;
      virtual TyErrorId processImpl(CAS &, ResultSpecification const &) = 0;
      virtual TyErrorId reconfigureImpl() = 0;

      virtual TyErrorId batchProcessCompleteImpl()=0;
      virtual TyErrorId collectionProcessCompleteImpl()=0;

      virtual bool hasNextImpl()=0;
      virtual CAS & nextImpl()=0;
      virtual int getCasInstancesRequiredImpl()=0;


      // called by subclass in destructor, we cannot put this in the destructor
      // since it possibly operates on data which is already destroyed in the
      // destructor of EngineBase
      void destroyIfNeeded();
    public:
      static uima::internal::EngineBase & promoteEngine( uima::AnalysisEngine &);

      uima::internal::CASDefinition const & getCASDefinition() const {
        assert( EXISTS(iv_casDefinition) );
        return *iv_casDefinition;
      }

      uima::internal::CASDefinition & getCASDefinition() {
        assert( EXISTS(iv_casDefinition) );
        return *iv_casDefinition;
      }

      bool isInitialized(void) const;

      TyErrorId reinitTypeSystem();

      EngineState const &     getState(void) const {
        return(iv_state);
      }

      virtual AnnotatorContext & getAnnotatorContext() {
        assert( EXISTS(iv_pAnnotatorContext) );
        return *iv_pAnnotatorContext;
      }

      virtual AnnotatorContext const & getAnnotatorContext() const {
        assert( EXISTS(iv_pAnnotatorContext) );
        return *iv_pAnnotatorContext;
      }

      CAS * newTCAS();
      CAS * newCAS() const;

      ResultSpecification const & getCompleteResultSpecification() const;

      TyErrorId logError(util::Trace & rclTrace, TyErrorId utErrorId);

      virtual TyErrorId initialize(AnalysisEngineDescription const &);
      virtual TyErrorId destroy();
      virtual TyErrorId reconfigure();
      virtual TyErrorId batchProcessComplete();
      virtual TyErrorId collectionProcessComplete();
      virtual bool hasNext();
      virtual CAS &  next();
      virtual int  getCasInstancesRequired();
      CASIterator processAndOutputNewCASes(CAS &);
      virtual TyErrorId process(CAS & cas, ResultSpecification const &);
      virtual TyErrorId process(CAS & cas);
    };
  }
}
/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif

