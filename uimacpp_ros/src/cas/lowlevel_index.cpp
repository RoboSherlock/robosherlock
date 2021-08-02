/** \file lowlevel_index.cpp .
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

#include <uima/lowlevel_index.hpp>
#include <uima/lowlevel_fsheap.hpp>
#include <uima/lowlevel_fsfilter.hpp>
#include <uima/lowlevel_indexiterator.hpp>
#include <uima/lowlevel_indexcomparator.hpp>
#include <uima/lowlevel_indexrepository.hpp>
#include <uima/msg.h>

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
  namespace lowlevel {

    IndexABase::IndexABase(IndexRepository const & crIndexRepository,
                           uima::lowlevel::TyFSType tyType)
        : iv_crIndexRepository(crIndexRepository),
        iv_crFSHeap(iv_crIndexRepository.getFSHeap() ),
        iv_crTypeSystem(iv_crFSHeap.getTypeSystem()),
        iv_tyFSType(tyType) {
      assert( iv_crTypeSystem.isValidType(iv_tyFSType) );
    }


    /**
     * generic implementation of find().
     */
    TyFS IndexABase::find(TyFS fs) const {
      unique_ptr<IndexIterator> it(createIterator());
      assert( EXISTS(it.get()) );
      for (it->moveToFirst(); it->isValid(); it->moveToNext() ) {
        TyFS nextFS = it->get();
        if (nextFS == fs) {
          return fs;
        }
      }
      return uima::lowlevel::FSHeap::INVALID_FS;
    }


    /**
     * An iterator on top of another (decorator) which does simple filtering.
     */
    class FilterIndexIterator : public IndexIterator {
    private:
      FSFilter const * iv_cpFilter;
      IndexIterator* iv_pIterator;

      void moveToNextUnfiltered() {
        while (iv_pIterator->isValid()) {
          bool bIsFiltered = iv_cpFilter->isFiltered(iv_pIterator->get());
          if (!bIsFiltered) {
            return;
          } else {
            iv_pIterator->moveToNext();
          }
        }
      }

      void moveToPreviousUnfiltered() {
        while (iv_pIterator->isValid()) {
          bool bIsFiltered = iv_cpFilter->isFiltered(iv_pIterator->get());
          if (!bIsFiltered) {
            return;
          } else {
            iv_pIterator->moveToPrevious();
          }
        }
      }

    public:
      FilterIndexIterator(FSFilter const * cpFilter, IndexIterator* pIterator)
          : iv_cpFilter(cpFilter),
          iv_pIterator(pIterator) {}

      virtual ~FilterIndexIterator() {
        assert( EXISTS(iv_pIterator) );
        delete iv_pIterator;
      }

      void moveToFirst() {
        assert( EXISTS(iv_pIterator) );
        iv_pIterator->moveToFirst();
        moveToNextUnfiltered();
      }

      void moveToLast() {
        assert( EXISTS(iv_pIterator) );
        iv_pIterator->moveToLast();
        moveToPreviousUnfiltered();
      }

      void moveToPrevious() {
        assert(isValid());
        iv_pIterator->moveToPrevious();
        moveToPreviousUnfiltered();
      }

      void moveToNext() {
        assert(isValid());
        iv_pIterator->moveToNext();
        moveToNextUnfiltered();
      }

      TyFS get() const {
          assert(isValid());
          return iv_pIterator->get();
        }

      TyFSType getTyFSType() const {
        assert( isValid() );
        return iv_pIterator->getTyFSType();
      }

      bool isValid() const {
        assert( EXISTS(iv_pIterator) );
        return iv_pIterator->isValid();
      }

      IndexIterator* clone() const {
        return new FilterIndexIterator(iv_cpFilter, iv_pIterator->clone());
      }

      bool moveTo(TyFS fs) {
        assert( EXISTS(iv_pIterator) );
        return iv_pIterator->moveTo(fs);
      }

    };


    IndexIterator* IndexABase::createFilteredIterator(FSFilter const * cpFilter) const {
      IndexIterator* pit = createIterator();
      assert( EXISTS(pit) );
      IndexIterator* result = new FilterIndexIterator(cpFilter, pit);
      assert( EXISTS(result) );
      return result;
    }

  }
}



/* ----------------------------------------------------------------------- */



