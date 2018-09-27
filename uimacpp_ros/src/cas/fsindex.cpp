/** \file fsindex.cpp .
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

#include <uima/fsindex.hpp>
#include <uima/msg.h>
#include <uima/internal_fspromoter.hpp>
#include <uima/lowlevel_fsfilter.hpp>
#include <uima/lowlevel_index.hpp>
#include <uima/lowlevel_indexrepository.hpp>
#include <uima/fsfilterbuilder.hpp>
using namespace std;
using namespace uima::internal;
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

/*
Those implementations are thin wrappers on top of the lowlevel API.
*/

namespace uima {

  UIMA_EXC_CLASSIMPLEMENT(InvalidIndexObjectException, CASException);
  UIMA_EXC_CLASSIMPLEMENT(WrongFSTypeForIndexException, CASException);

  void FSIndex::checkValidity() const {
    if (!isValid()) {
      UIMA_EXC_THROW_NEW(InvalidIndexObjectException,
                         UIMA_ERR_INVALID_INDEX_OBJECT,
                         UIMA_MSG_ID_EXC_INVALID_INDEX_OBJECT,
                         ErrorMessage(UIMA_MSG_ID_EXCON_UNKNOWN_CONTEXT),
                         ErrorInfo::recoverable
                        );
    }
  }

  FSIndex::FSIndex(lowlevel::IndexABase const * anIndex, lowlevel::IndexRepository const & rFSSystem)
      : iv_pIndex(anIndex),
      iv_indexRepository(& CONST_CAST(uima::lowlevel::IndexRepository &, rFSSystem) ) {}

  FSIndex::FSIndex(lowlevel::IndexABase const * anIndex, lowlevel::IndexRepository & rFSSystem)
      : iv_pIndex(anIndex),
      iv_indexRepository(& rFSSystem) {}

  FSIndex::FSIndex()
      : iv_pIndex(NULL),
      iv_indexRepository(NULL) {}

  bool FSIndex::isValid() const {
    return(iv_pIndex != NULL) && (iv_indexRepository != NULL);
  }

  size_t FSIndex::getSize() const {
    checkValidity();
    return iv_pIndex->getSize();
  }

  FeatureStructure FSIndex::find(FeatureStructure const & anFS) const {
    checkValidity();
    if (!anFS.isValid()) {
      UIMA_EXC_THROW_NEW(InvalidFSObjectException,
                         UIMA_ERR_INVALID_FS_OBJECT,
                         UIMA_MSG_ID_EXC_INVALID_FS_OBJECT,
                         ErrorMessage(UIMA_MSG_ID_EXCON_FINDING_FS_IN_INDEX),
                         ErrorInfo::recoverable
                        );
    }
    lowlevel::TyFS result = iv_pIndex->find( internal::FSPromoter::demoteFS( anFS ) );
    return internal::FSPromoter::promoteFS( result, iv_indexRepository->getCas() );
  }

  FSIterator FSIndex::iterator() const {
    checkValidity();
    lowlevel::IndexIterator* it = iv_pIndex->createIterator();
    return FSIterator( it, &iv_indexRepository->getCas() );
  }


  FSIterator FSIndex::typeSetIterator(set<uima::Type> const & crTypes) const {
    checkValidity();
    uima::lowlevel::TyFSType tyIndexType = iv_pIndex->getType();
    set<uima::lowlevel::TyFSType> setLowlevelTypes;
    set<uima::Type>::const_iterator cit;
    for (cit = crTypes.begin(); cit != crTypes.end(); ++cit) {
      uima::lowlevel::TyFSType tyType = uima::internal::FSPromoter::demoteType(*cit);
      if ( ! iv_indexRepository->getFSHeap().getTypeSystem().subsumes(tyIndexType, tyType) ) {
        UIMA_EXC_THROW_NEW(WrongFSTypeForIndexException,
                           UIMA_ERR_WRONG_FSTYPE_FOR_INDEX,
                           UIMA_MSG_ID_EXC_WRONG_FSTYPE_FOR_INDEX,
                           ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_ITERATOR),
                           ErrorInfo::recoverable
                          );

      }
      setLowlevelTypes.insert(tyType);
    }

    uima::lowlevel::IndexIterator * it = iv_pIndex->createTypeSetIterator( setLowlevelTypes );
    return FSIterator( it, &iv_indexRepository->getCas() );
  }



///////////////////////////////////////////////////////////////

  /**
   * This class wraps an OO API filter to obey the lowlevel::FSFilter interface.
   */
  namespace internal {
    class FSFilterWrapper : public lowlevel::FSFilter {
    private:
      uima::FSFilter const * iv_cpFilter;
      uima::CAS const * iv_cas;
    public:
      FSFilterWrapper(uima::FSFilter const * cpFilter, uima::CAS const * cas)
          : iv_cpFilter(cpFilter),
          iv_cas(cas) {
        assert(EXISTS(iv_cpFilter));
        assert(EXISTS(iv_cas));
      }

      bool isFiltered(lowlevel::TyFS anFS) const {
        assert(EXISTS(iv_cpFilter));
        assert(EXISTS(iv_cas));
        // const cast is OK here since anFS will not be changed
        //  since it is passed as const parameters to isFiltered
        FeatureStructure fs = internal::FSPromoter::promoteFS(anFS, *iv_cas);
        return iv_cpFilter->isFiltered(fs);
      }
    };
  }


  FSIterator FSIndex::filteredIterator(uima::FSFilter const * cpFilter) const {
    checkValidity();
    internal::FSFilterWrapper* pFilterWrapper = new internal::FSFilterWrapper( cpFilter, & iv_indexRepository->getCas() );
    lowlevel::IndexIterator* it = iv_pIndex->createFilteredIterator(pFilterWrapper);
    return FSIterator(it, & iv_indexRepository->getCas() );
  }

}


