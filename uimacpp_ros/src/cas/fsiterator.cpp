/** \file fsiterator.cpp .
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
#include <uima/fsiterator.hpp>
#include <uima/lowlevel_indexiterator.hpp>
#include <uima/internal_fspromoter.hpp>
#include <uima/lowlevel_fsheap.hpp>

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

  FSIterator::FSIterator(uima::lowlevel::IndexIterator* pIt,
                         uima::CAS * pFSSystem)
      :  iv_pIterator( pIt),
      iv_cas( pFSSystem ) {
    assert(EXISTS(iv_pIterator));
    iv_pIterator->moveToFirst();
  }

  FSIterator::FSIterator()
      :  iv_pIterator(NULL),
      iv_cas(NULL) {}

  FSIterator & FSIterator::operator=(FSIterator const & crOther) {
    if (this == &crOther) {
      return *this;
    }
    if (iv_pIterator != NULL) {
      delete iv_pIterator;
    }
    iv_pIterator = NULL;
    iv_cas = crOther.iv_cas;
    iv_pIterator = crOther.iv_pIterator->clone();
    return *this;
  }

  FSIterator::FSIterator(FSIterator const & crOther)
      : iv_pIterator(NULL),
      iv_cas(NULL) {
    *this = crOther;
  }

  FSIterator::~FSIterator() {
    if (iv_pIterator != NULL) {
      delete iv_pIterator;
    }
  }


  bool FSIterator::isValid() const {
    if (iv_pIterator == NULL) {
      return false;
    }
    assert( EXISTS(iv_pIterator) );
    return iv_pIterator->isValid();
  }

  void FSIterator::moveTo(FeatureStructure fs) {
    assert( EXISTS(iv_pIterator) );
    iv_pIterator->moveTo(uima::internal::FSPromoter::demoteFS(fs) );
  }

  void FSIterator::moveToFirst() {
    assert( EXISTS(iv_pIterator) );
    iv_pIterator->moveToFirst();
  }

  void FSIterator::moveToLast() {
    assert( EXISTS(iv_pIterator) );
    iv_pIterator->moveToLast();
  }

  void FSIterator::moveToNext() {
    assert( EXISTS(iv_pIterator) );
    iv_pIterator->moveToNext();
  }

  void FSIterator::moveToPrevious() {
    assert( EXISTS(iv_pIterator) );
    iv_pIterator->moveToPrevious();
  }

  FeatureStructure FSIterator::get() const {
      assert( EXISTS(iv_pIterator) );
      assert( EXISTS(iv_cas) );
      return uima::internal::FSPromoter::promoteFS( iv_pIterator->get(), *iv_cas );
    }


  FeatureStructure FSIterator::peekNext() const {
    FeatureStructure result;
    iv_pIterator->moveToNext();
    if (iv_pIterator->isValid()) {
      result = uima::internal::FSPromoter::promoteFS( iv_pIterator->get(), *iv_cas );
      iv_pIterator->moveToPrevious();
    } else {
      iv_pIterator->moveToLast();
    }
    return result;
  }

  FeatureStructure FSIterator::peekPrevious() const {
    FeatureStructure result;
    iv_pIterator->moveToPrevious();
    if (iv_pIterator->isValid()) {
      result = uima::internal::FSPromoter::promoteFS( iv_pIterator->get(), *iv_cas );
      iv_pIterator->moveToNext();
    } else {
      iv_pIterator->moveToFirst();
    }
    return result;
  }


}



/* ----------------------------------------------------------------------- */



