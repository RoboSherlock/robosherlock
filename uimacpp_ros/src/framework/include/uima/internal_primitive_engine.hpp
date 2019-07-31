#ifndef UIMA_INTERNAL_PRIMITIVE_ENGINE_HPP
#define UIMA_INTERNAL_PRIMITIVE_ENGINE_HPP
/** \file internal_primitive_engine.hpp .
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
#include <uima/engine.hpp>
#include <uima/annotator.hpp>
#include <uima/result_specification.hpp>
#include <uima/internal_engine_base.hpp>
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
  class AnnotatorContext;
  namespace internal {
    class TCASImpl;
    class CapabilityContainer;
  }
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace internal {

    /**
     * Implementation of a primitive engine.
     * Basically consists of the annotator plus its capability description.
     */
    class UIMA_LINK_IMPORTSPEC PrimitiveEngine : public EngineBase {
    private:
      AnnotatorProxy * iv_pAnnotator;
      CapabilityContainer * iv_pCapContainer;
    public:
      PrimitiveEngine(AnnotatorContext &,
                      bool bOwnsANC,
                      bool bOwnsTAESpec,
                      uima::internal::CASDefinition &,
                      bool ownsCAsDef,
                      bool bDelayLoad = false);

      ~PrimitiveEngine();

      virtual bool isPrimitive() const {
        return true;
      }

      TyErrorId initializeImpl(AnalysisEngineDescription const & crDescription);

      TyErrorId typeSystemInitImpl();

      TyErrorId processImpl(CAS & cas, ResultSpecification const & crResultSpecification);

      TyErrorId destroyImpl();

      virtual const std::string & getSymbolicName(void) const {
        assert( EXISTS(iv_pAnnotator) );
        return iv_pAnnotator->getSymbolicName();
      }

      virtual TyErrorId reinitTypeSystemImpl() {
        return typeSystemInitImpl();
      }

#ifdef UIMA_DEBUG_ANNOTATOR_TIMING

      virtual const Timer & getLoadTimer(void) const {
        assert( EXISTS(iv_pAnnotator) );
        return iv_pAnnotator->getLoadTimer();
      }

      virtual const Timer &              getInitTimer(void) const {
        assert( EXISTS(iv_pAnnotator) );
        return iv_pAnnotator->getInitTimer();
      }

      virtual const Timer &              getDeInitTimer(void) const {
        assert( EXISTS(iv_pAnnotator) );
        return iv_pAnnotator->getDeInitTimer();
      }

      virtual const Timer &              getConfigTimer(void) const {
        assert( EXISTS(iv_pAnnotator) );
        return iv_pAnnotator->getConfigTimer();
      }

      virtual const Timer &              getProcessDocumentTimer(void) const {
        assert( EXISTS(iv_pAnnotator) );
        return iv_pAnnotator->getProcessDocumentTimer();
      }

      virtual void displayTimingData(util::ConsoleUI const & crConsole, bool bVerbose = false) const;
#endif

      virtual TyErrorId reconfigureImpl();
      virtual TyErrorId batchProcessCompleteImpl();
      virtual TyErrorId collectionProcessCompleteImpl();

      virtual bool hasNextImpl();
      virtual CAS & nextImpl();
      virtual int getCasInstancesRequiredImpl();

    };



  }
}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */

#endif

