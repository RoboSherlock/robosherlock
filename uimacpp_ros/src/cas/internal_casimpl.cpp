/** \file internal_casimpl.cpp .
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

//#define DEBUG_VERBOSE
/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp>
#include <uima/internal_casimpl.hpp>
#include <uima/annotator_context.hpp>
#include <uima/resmgr.hpp>
#include <uima/msg.h>
#include <uima/taespecifierbuilder.hpp>
#include "xercesc/framework/MemBufInputSource.hpp"
#include "xercesc/parsers/XercesDOMParser.hpp"
#include <uima/xmlerror_handler.hpp>
#include <uima/internal_engine_base.hpp>
#include <uima/casdefinition.hpp>
#include <uima/lowlevel_indexdefinition.hpp>

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

    CASImpl & CASImpl::promoteCAS(CAS & rCAS) {
      assert(sizeof(CASImpl) == sizeof(CAS) );
      return (CASImpl &) rCAS;
    }

    CASImpl const & CASImpl::promoteCAS(CAS const & crCAS) {
      assert(sizeof(CASImpl) == sizeof(CAS) );
      return (CASImpl const &) crCAS;
    }

    CASImpl::CASImpl(uima::internal::CASDefinition & casdefs,
                     size_t uiFSHeapPageSize,
                     size_t uiStringHeapPageSize,
                     size_t uiStringRefHeapPageSize)
        : CAS( casdefs,
               uiFSHeapPageSize,
               uiStringHeapPageSize,
               uiStringRefHeapPageSize) {
      // This assertion ensures that downcasting a CAS to a CASImpl
      // does no harm
      assert(sizeof(CASImpl) == sizeof(uima::CAS) );
    }

    CASImpl::CASImpl(uima::internal::CASDefinition & casdefs,
                     bool ownsCasDef,
                     size_t uiFSHeapPageSize,
                     size_t uiStringHeapPageSize,
                     size_t uiStringRefHeapPageSize)
        : CAS( casdefs,
               ownsCasDef,
               uiFSHeapPageSize,
               uiStringHeapPageSize,
               uiStringRefHeapPageSize) {
      // This assertion ensures that downcasting a CAS to a CASImpl
      // does no harm
      assert(sizeof(CASImpl) == sizeof(uima::CAS) );
    }




    CASImpl::CASImpl(CAS* baseCAS, SofaFS aSofa) : CAS(baseCAS, aSofa) {
      assert(sizeof(CASImpl) == sizeof(uima::CAS) );
    }


    CASImpl * CASImpl::createCASImpl(CASDefinition & casdefs, AnnotatorContext const & crANC) {
      int     iFSHeapPageSize = UIMA_ENGINE_CONFIG_DEFAULT_FSHEAP_PAGESIZE;
      int     iStringHeapPageSize = UIMA_ENGINE_CONFIG_DEFAULT_STRINGHEAP_PAGESIZE;
      int     iStringRefHeapPageSize = UIMA_ENGINE_CONFIG_DEFAULT_STRINGREFHEAP_PAGESIZE;

      if (crANC.isParameterDefined(UIMA_ENGINE_CONFIG_OPTION_FSHEAP_PAGESIZE)) {
        (void) crANC.extractValue(UIMA_ENGINE_CONFIG_OPTION_FSHEAP_PAGESIZE, iFSHeapPageSize);
      }
      if (crANC.isParameterDefined(UIMA_ENGINE_CONFIG_OPTION_STRINGHEAP_PAGESIZE)) {
        (void) crANC.extractValue(UIMA_ENGINE_CONFIG_OPTION_STRINGHEAP_PAGESIZE, iStringHeapPageSize);
      }
      if (crANC.isParameterDefined(UIMA_ENGINE_CONFIG_OPTION_STRINGREFHEAP_PAGESIZE)) {
        (void) crANC.extractValue(UIMA_ENGINE_CONFIG_OPTION_STRINGREFHEAP_PAGESIZE, iStringRefHeapPageSize);
      }

      CASImpl * pResult = new CASImpl(casdefs,
                                      iFSHeapPageSize,
                                      iStringHeapPageSize,
                                      iStringRefHeapPageSize);
      return pResult;
    }


    //BSI
    CASImpl * CASImpl::createCASImpl(CASDefinition & casdefs, bool ownsCasDef) {
      int     iFSHeapPageSize = UIMA_ENGINE_CONFIG_DEFAULT_FSHEAP_PAGESIZE;
      int     iStringHeapPageSize = UIMA_ENGINE_CONFIG_DEFAULT_STRINGHEAP_PAGESIZE;
      int     iStringRefHeapPageSize = UIMA_ENGINE_CONFIG_DEFAULT_STRINGREFHEAP_PAGESIZE;


      CASImpl * pResult = new CASImpl(casdefs,
                                      ownsCasDef,
                                      iFSHeapPageSize,
                                      iStringHeapPageSize,
                                      iStringRefHeapPageSize);
      return pResult;
    }


    CASImpl::~CASImpl() {}

    uima::lowlevel::FSHeap & CASImpl::getHeap() {
      assert( EXISTS(iv_heap) );
      return *iv_heap;
    }

    uima::lowlevel::FSHeap const & CASImpl::getHeap() const {
      assert( EXISTS(iv_heap) );
      return *iv_heap;
    }

    uima::lowlevel::IndexRepository & CASImpl::getIndexRepository() {
      assert( EXISTS(iv_indexRepository));
      return *iv_indexRepository;
    }

    uima::lowlevel::IndexRepository const & CASImpl::getIndexRepository() const {
      assert( EXISTS(iv_indexRepository));
      return *iv_indexRepository;
    }

    uima::internal::CASDefinition const & CASImpl::getCASDefinition() const {
      assert(EXISTS(iv_casDefinition));
      return *iv_casDefinition;
    }

    uima::internal::CASDefinition & CASImpl::getCASDefinition() {
      assert(EXISTS(iv_casDefinition));
      return *iv_casDefinition;
    }


  } //namespace internal
} //namespace uima

/* ----------------------------------------------------------------------- */

