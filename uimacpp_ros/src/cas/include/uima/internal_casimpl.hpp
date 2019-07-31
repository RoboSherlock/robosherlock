#ifndef UIMA_INTERNAL_CASIMPL_HPP
#define UIMA_INTERNAL_CASIMPL_HPP
/** \file internal_casimpl.hpp .
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
#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <uima/cas.hpp>
#include <uima/lowlevel_fsheap.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  class AnnotatorContext;
  namespace internal {
    class FSSystem;
  }
}
/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace internal {

    /**
     * A concrete subclass of the CAS.
     * In fact, all CAS objects created in UIMACPP are instances of this
     * class, so calling the static promoteCAS() methods here should be safe.
     */
    class UIMA_LINK_IMPORTSPEC CASImpl : public uima::CAS {
    public:
      static CASImpl & promoteCAS(CAS &);
      static CASImpl const & promoteCAS(CAS const &);

      static CASImpl * createCASImpl(CASDefinition &, AnnotatorContext const &);

      static CASImpl * createCASImpl(CASDefinition &, bool ownsCasDef);

      CASImpl(CASDefinition &,
              size_t uiFSHeapPageSize,
              size_t uiStringHeapPageSize,
              size_t uiStringRefHeapPageSize);
      CASImpl(CASDefinition &,
              bool ownsCasDef,
              size_t uiFSHeapPageSize,
              size_t uiStringHeapPageSize,
              size_t uiStringRefHeapPageSize);


      CASImpl(CAS* baseCas, SofaFS aSofa = SofaFS());

      ~CASImpl();

      uima::lowlevel::FSHeap & getHeap();
      uima::lowlevel::FSHeap const & getHeap() const;

      uima::lowlevel::IndexRepository & getIndexRepository();
      uima::lowlevel::IndexRepository const & getIndexRepository() const;

      uima::internal::CASDefinition const & getCASDefinition() const;
      uima::internal::CASDefinition & getCASDefinition();

//       virtual bool isTCAS() const {
//         return false;
//       }


      /**
       * Resets the CAS state. In particular, all data structures meant only to
       * live throughout a document are deleted.
       */
      TyErrorId reset(void) {
        return CAS::reset();
      }

    }
    ;  // class CASImpl
  } // namespace internal
} // namespace uima

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif

