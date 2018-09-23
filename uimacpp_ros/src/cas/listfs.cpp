/** \file listfs.cpp .
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
#include <uima/listfs.hpp>
#include <uima/featurestructure.hpp>
#include <uima/lowlevel_fsheap.hpp>
#include <uima/msg.h>
#include <uima/internal_fspromoter.hpp>
#include <uima/internal_typeshortcuts.hpp>
#include <uima/internal_fsvalue_accessors.hpp>

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
  /* ----------------------------------------------------------------------- */
  /*       Exceptions Implementation                                         */
  /* ----------------------------------------------------------------------- */
  UIMA_EXC_CLASSIMPLEMENT(FSIsNotListException, CASException);
  UIMA_EXC_CLASSIMPLEMENT(ListIsEmptyException, CASException);
  UIMA_EXC_CLASSIMPLEMENT(ListIsCircularException, CASException);

  /* ----------------------------------------------------------------------- */
  /*       Tool Functions Implementation                                     */
  /* ----------------------------------------------------------------------- */

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  void
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  checkList(lowlevel::TyFS tyFS, TyMessageId tyContext) const {
    assert(EXISTS(iv_cas));
    lowlevel::TyFSType tyType = iv_cas->getHeap()->getType( tyFS );
    if ( ! iv_cas->getHeap()->getTypeSystem().subsumes( LIST_TYPE, tyType) ) {
      UIMA_EXC_THROW_NEW(FSIsNotListException,
                         UIMA_ERR_FS_IS_NOT_LIST,
                         UIMA_MSG_ID_EXC_FS_IS_NOT_LIST,
                         ErrorMessage(tyContext),
                         ErrorInfo::recoverable
                        );
    }
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  void
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  checkNEList(lowlevel::TyFS tyFS, TyMessageId tyContext) const {
    assert(EXISTS(iv_cas));
    lowlevel::FSHeap const & crHeap = *iv_cas->getHeap();
    lowlevel::TyFSType tyType = crHeap.getType( tyFS );
    if (tyType != NELIST_TYPE) {
      UIMA_EXC_THROW_NEW(ListIsEmptyException,
                         UIMA_ERR_LIST_IS_EMPTY,
                         UIMA_MSG_ID_EXC_LIST_IS_EMPTY,
                         ErrorMessage(tyContext),
                         ErrorInfo::recoverable);
    }
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  void
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  checkCircularity(lowlevel::TyFS tyFS1, lowlevel::TyFS tyFS2, TyMessageId tyContext) const {
    assert(EXISTS(iv_cas));
    if ( tyFS1 == tyFS2 ) {
      lowlevel::FSHeap const & crHeap = *iv_cas->getHeap();
      lowlevel::TyFSType tyType1 = crHeap.getType( tyFS1 );
      lowlevel::TyFSType tyType2 = crHeap.getType( tyFS2 );
      // if both lists are empty we don't have any problem
      if (   tyType1 == ELIST_TYPE
             && tyType2 == ELIST_TYPE) {
        return;
      }
      UIMA_EXC_THROW_NEW(ListIsCircularException,
                         UIMA_ERR_LIST_IS_CIRCULAR,
                         UIMA_MSG_ID_EXC_LIST_IS_CIRCULAR,
                         ErrorMessage(tyContext),
                         ErrorInfo::recoverable
                        );
    }
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  lowlevel::TyFS
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  getLastListElement(lowlevel::TyFS tyListFS, size_t & rOutSize) const {
    assert(EXISTS(iv_cas));
    rOutSize = 0;
    lowlevel::FSHeap & rHeap = *iv_cas->getHeap();
    assert( rHeap.getType(tyListFS) == NELIST_TYPE );
    lowlevel::TyFS tyCurrentList = tyListFS;
    while (true) {
      assert( rHeap.getType(tyCurrentList) == NELIST_TYPE );
      lowlevel::TyFS tyNextList = rHeap.getFSValue(tyCurrentList, TAIL_FEATURE);
      if ( rHeap.getType(tyNextList) != NELIST_TYPE ) {
        assert( rHeap.getType(tyNextList) == ELIST_TYPE );
        break;
      }
      tyCurrentList = tyNextList;
      ++rOutSize;
    }
    return tyCurrentList;
  }


  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  lowlevel::TyFS
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  addLastLowlevel(lowlevel::TyFS tyListFS, T tyNewElement) {
    assert(EXISTS(iv_cas));
    lowlevel::FSHeap & rHeap = *iv_cas->getHeap();
    assert( rHeap.getType(tyListFS) == NELIST_TYPE );
    size_t ui = 0;
    lowlevel::TyFS tyLastListElement = getLastListElement(tyListFS, ui);

    lowlevel::TyFS tyEmptyList = rHeap.getFSValue( tyLastListElement, TAIL_FEATURE);
    assert( ! rHeap.isUntouchedFSValue(tyLastListElement, TAIL_FEATURE) );
    assert( rHeap.getType( tyEmptyList ) == ELIST_TYPE );

    lowlevel::TyFS tyNewListElement = rHeap.createFS(NELIST_TYPE);
    internal::setFSValueTempl((uima::CAS *)iv_cas, tyNewListElement, HEAD_FEATURE, tyNewElement);
    rHeap.setFSValue( tyNewListElement, TAIL_FEATURE, tyEmptyList );
    rHeap.setFSValue( tyLastListElement, TAIL_FEATURE, tyNewListElement );

    return tyNewListElement;
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  void
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  appendLowlevel(lowlevel::TyFS tyListFS1, lowlevel::TyFS tyListFS2) {
    assert(EXISTS(iv_cas));
    lowlevel::FSHeap & rHeap = *iv_cas->getHeap();
    assert( rHeap.getType(tyListFS1) == NELIST_TYPE );
    size_t ui = 0;
    lowlevel::TyFS tyLastListElement = getLastListElement(tyListFS1, ui);

#ifndef NDEBUG
    lowlevel::TyFS tyEmptyList = rHeap.getFSValue( tyLastListElement, TAIL_FEATURE);
    assert( ! rHeap.isUntouchedFSValue(tyLastListElement, TAIL_FEATURE) );
    assert( rHeap.getType( tyEmptyList ) == ELIST_TYPE );
#endif /* debug mode only */

    rHeap.setFSValue( tyLastListElement, TAIL_FEATURE, tyListFS2);
  }
  /* ----------------------------------------------------------------------- */
  /*       BasicListFS Implementation                                             */
  /* ----------------------------------------------------------------------- */

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  BasicListFS(lowlevel::TyFS anFS, uima::CAS & cas, bool bDoChecks) :
      FeatureStructure(anFS, cas) {
    if (bDoChecks) {
      checkValidity(UIMA_MSG_ID_EXCON_CREATING_LISTFS);
      checkList(iv_tyFS, UIMA_MSG_ID_EXCON_CREATING_LISTFS);
    }
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  BasicListFS() :
      FeatureStructure() {}

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  BasicListFS( FeatureStructure const & fs) :
      FeatureStructure(fs) {
    if (isValid()) {
      checkList(iv_tyFS, UIMA_MSG_ID_EXCON_CREATING_LISTFS);
    }
    // we should not have any additional members
    assert(sizeof(BasicListFS) == sizeof(FeatureStructure));
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  bool
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  isEmpty() const {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_LIST_ISEMPTY);
    assert(EXISTS(iv_cas));
    lowlevel::FSHeap const & crHeap = *iv_cas->getHeap();
    lowlevel::TyFSType tyType = crHeap.getType( iv_tyFS );
    return(tyType != NELIST_TYPE);
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  size_t
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  getLength() const {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_LIST_LENGTH);
    assert(EXISTS(iv_cas));
    lowlevel::FSHeap & rFSHeap = *iv_cas->getHeap();
    if ( rFSHeap.getType( iv_tyFS ) != NELIST_TYPE ) {
      return 0;
    }
    size_t uiResult = 0;
    (void) getLastListElement(iv_tyFS, uiResult);
    return uiResult+1;
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  T
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  getHead() const {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_LIST_HEAD);
    checkNEList(iv_tyFS, UIMA_MSG_ID_EXCON_GETTING_LIST_HEAD);
    assert(EXISTS(iv_cas));
    lowlevel::FSHeap const & crHeap = *iv_cas->getHeap();
    assert( crHeap.getType(iv_tyFS) == NELIST_TYPE );
    T result;
    internal::getFSValueTempl((uima::CAS *)iv_cas, iv_tyFS, HEAD_FEATURE, result);
    return result;
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  void
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  setHead( T const & fs ) {
    checkValidity(UIMA_MSG_ID_EXCON_SETTING_LIST_HEAD);
    checkNEList(iv_tyFS, UIMA_MSG_ID_EXCON_SETTING_LIST_HEAD);
    assert(EXISTS(iv_cas));
    lowlevel::FSHeap & rHeap = *iv_cas->getHeap();
    assert( rHeap.getType(iv_tyFS) == NELIST_TYPE );
    internal::setFSValueTempl((uima::CAS *)iv_cas, iv_tyFS, HEAD_FEATURE, fs);
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  getTail() const {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_LIST_TAIL);
    checkNEList(iv_tyFS, UIMA_MSG_ID_EXCON_GETTING_LIST_TAIL);
    assert(EXISTS(iv_cas));
    lowlevel::FSHeap const & crHeap = *iv_cas->getHeap();
    assert( crHeap.getType(iv_tyFS) == NELIST_TYPE );
    lowlevel::TyFS tyNextList = crHeap.getFSValue(iv_tyFS, TAIL_FEATURE);
    return BasicListFS(tyNextList, *iv_cas, false);
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  void
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  setTail( BasicListFS fs ) {
    checkValidity(UIMA_MSG_ID_EXCON_SETTING_LIST_TAIL);
    fs.checkValidity(UIMA_MSG_ID_EXCON_SETTING_LIST_TAIL);
    checkNEList(iv_tyFS, UIMA_MSG_ID_EXCON_SETTING_LIST_TAIL);
    assert(EXISTS(iv_cas));
    lowlevel::FSHeap & rHeap = *iv_cas->getHeap();
    assert( rHeap.getType(iv_tyFS) == NELIST_TYPE );
    rHeap.setFSValue(iv_tyFS, TAIL_FEATURE, fs.iv_tyFS);
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  addFirst( T const & fs ) {
    checkValidity(UIMA_MSG_ID_EXCON_ADDING_LIST_VALUE);
    lowlevel::FSHeap & rHeap = *iv_cas->getHeap();
    lowlevel::TyFS tyNEFS = rHeap.createFS( NELIST_TYPE );
    rHeap.setFSValue(tyNEFS, TAIL_FEATURE, iv_tyFS);
    internal::setFSValueTempl((uima::CAS *)iv_cas, tyNEFS, HEAD_FEATURE, fs);
    iv_tyFS = tyNEFS;
    return(*this);
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  addLast( T const & fs ) {
    checkValidity(UIMA_MSG_ID_EXCON_ADDING_LIST_VALUE);
    lowlevel::FSHeap & rHeap = *iv_cas->getHeap();
    lowlevel::TyFS tyNewEnd;
    if (rHeap.getType( iv_tyFS ) == NELIST_TYPE) {
      // if this list is not empty we search for the end and put fs there
      tyNewEnd = addLastLowlevel(iv_tyFS, fs);
    } else {
      // if this list is empty we create an inital terminated list
      assert(rHeap.getType( iv_tyFS ) == ELIST_TYPE);
      lowlevel::TyFS tyEFS   = iv_tyFS;
      iv_tyFS = rHeap.createFS( NELIST_TYPE );
      internal::setFSValueTempl((uima::CAS *)iv_cas, iv_tyFS, HEAD_FEATURE, fs);
      rHeap.setFSValue(iv_tyFS, TAIL_FEATURE, tyEFS);
      tyNewEnd = iv_tyFS;
    }
    return BasicListFS(tyNewEnd, *iv_cas, false);
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  append( BasicListFS fs ) {
    checkValidity(UIMA_MSG_ID_EXCON_APPENDING_TO_LIST);
    fs.checkValidity(UIMA_MSG_ID_EXCON_APPENDING_TO_LIST);
    lowlevel::FSHeap & rHeap = *iv_cas->getHeap();
    checkCircularity(iv_tyFS, fs.iv_tyFS, UIMA_MSG_ID_EXCON_APPENDING_TO_LIST);
    if (rHeap.getType( iv_tyFS ) == NELIST_TYPE) {
      // if this list is not empty we do a full append
      appendLowlevel(iv_tyFS, fs.iv_tyFS);
    } else {
      // if this list is empty we just replace it with with fs
      iv_tyFS = fs.iv_tyFS;
    }
    return(*this);
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  prepend( BasicListFS fs ) {
    checkValidity(UIMA_MSG_ID_EXCON_PREPENDING_TO_LIST);
    fs.checkValidity(UIMA_MSG_ID_EXCON_PREPENDING_TO_LIST);
    lowlevel::FSHeap & rHeap = *iv_cas->getHeap();
    checkCircularity(iv_tyFS, fs.iv_tyFS, UIMA_MSG_ID_EXCON_PREPENDING_TO_LIST);
    // only do something if fs does have some elements
    if (rHeap.getType( fs.iv_tyFS ) == NELIST_TYPE) {
      appendLowlevel(fs.iv_tyFS, iv_tyFS);
      iv_tyFS = fs.iv_tyFS;
    }
    return(*this);
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  void
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  moveToNext() {
    checkValidity(UIMA_MSG_ID_EXCON_MOVING_LIST_TO_NEXT);
    checkNEList(iv_tyFS, UIMA_MSG_ID_EXCON_MOVING_LIST_TO_NEXT);
    assert(EXISTS(iv_cas));
    lowlevel::FSHeap const & crHeap = *iv_cas->getHeap();
    assert( crHeap.getType(iv_tyFS) == NELIST_TYPE );
    lowlevel::TyFS tyNextList = crHeap.getFSValue(iv_tyFS, TAIL_FEATURE);
    iv_tyFS = tyNextList;
#ifndef NDEBUG
    checkList(iv_tyFS, UIMA_MSG_ID_EXCON_MOVING_LIST_TO_NEXT);
#endif
  }


  template < class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  void terminateList(BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE> & aList,
                     uima::lowlevel::FSHeap & heap) {
    uima::lowlevel::TyFS emptyList = heap.createFS(ELIST_TYPE);
    uima::lowlevel::TyFS tailElement = uima::internal::FSPromoter::demoteFS( aList );
    heap.setFSValue( tailElement, TAIL_FEATURE, emptyList );
  }


  template < class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>
  deleteElementFromList(BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE> & aList,
                        uima::lowlevel::FSHeap & heap,
                        T const & element) {
    assert(! aList.isEmpty() );
    BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE> tail = aList.getTail();
    if (! tail.isEmpty() ) {
      if (tail.getHead() == element) {
        aList.setTail(tail.getTail());
        terminateList(tail, heap);
        return tail;
      } else {
        return deleteElementFromList(tail, heap, element);
      }
    }
    return BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>();
  }


  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::removeElement( T const & element ) {
    checkValidity(UIMA_MSG_ID_EXCON_UNKNOWN_CONTEXT);
    if (! isEmpty() ) {
      // if the first element of list is to be deleted
      if (getHead() == element) {
        BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE> result = *this;
        *this = getTail();
        terminateList(result, *iv_cas->getHeap() );
        return result;
      } else {
        BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE> result = deleteElementFromList(*this, *iv_cas->getHeap(), element);
        return result;
      }
    }
    return BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>();
  }


  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  /*static*/ bool
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  hasListElements(FeatureStructure fs, Feature const & f) {
    // there are several ways a feature can have zero values
    uima::lowlevel::FSHeap const & crHeap = *internal::FSPromoter::getFSHeap(fs);
    lowlevel::TyFSFeature tyFeat =  internal::FSPromoter::demoteFeature( f );
    lowlevel::TyFS        tyFS   =  internal::FSPromoter::demoteFS( fs );
    // 1: it is untouched. So we don't touch it either and return false
    if ( crHeap.isUntouchedFSValue(tyFS, tyFeat) ) {
      return false;
    }
    lowlevel::TyFS tyFSResult = internal::FSPromoter::getFSHeap(fs)->getFSValue(tyFS, tyFeat);
    // the value can be either
    //    1: a generic list type (should not happen but is a valid possiblity)
    //    2: an empty list (no elements in the list)
    //    3: a non-empty list (some elements in the list)
    assert(    crHeap.getType( tyFSResult ) == LIST_TYPE
               || crHeap.getType( tyFSResult ) == ELIST_TYPE
               || crHeap.getType( tyFSResult ) == NELIST_TYPE);
    // Return true, if the value is of type non-empty list
    return(crHeap.getType( tyFSResult ) == NELIST_TYPE);
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  /*static*/ BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  getListFSValue( FeatureStructure const & fs, Feature const & f ) {
    assert(fs.isValid());
    uima::CAS & rCas = *internal::FSPromoter::getFSCas(fs);
    uima::lowlevel::FSHeap & rHeap = *internal::FSPromoter::getFSHeap(fs);
    lowlevel::TyFSFeature tyF  =  internal::FSPromoter::demoteFeature( f );
    lowlevel::TyFS        tyFS =  internal::FSPromoter::demoteFS( fs );
    // check range type
    lowlevel::TyFSType tyRangeType = rHeap.getTypeSystem().getRangeType(tyF);
    if ( ! rHeap.getTypeSystem().subsumes( LIST_TYPE, tyRangeType) ) {
      UIMA_EXC_THROW_NEW(FSIsNotListException,
                         UIMA_ERR_FS_IS_NOT_LIST,
                         UIMA_MSG_ID_EXC_FS_IS_NOT_LIST,
                         ErrorMessage(UIMA_MSG_ID_EXCON_GETTING_FIRST_LIST_ELEMENT),
                         ErrorInfo::recoverable
                        );
    }

    // it is untouched. We have to create a proper empty terminated list
    if ( rHeap.isUntouchedFSValue(tyFS, tyF) ) {
      lowlevel::TyFS tyFSEList = rHeap.createFS(ELIST_TYPE);
      rHeap.setFSValue(tyFS, tyF, tyFSEList);
      return BasicListFS(tyFSEList, rCas);
    }
    return BasicListFS(rHeap.getFSValue(tyFS, tyF), rCas, false);
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  /*static*/ BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  createListFS( CAS & cas, bool bIsPermanent ) {
    lowlevel::FSHeap & rHeap = *uima::internal::FSPromoter::getFSHeap(cas);
    lowlevel::TyFS tyFS = rHeap.createFS(ELIST_TYPE);
    return BasicListFS(tyFS, cas, false);
  }

  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  /*static*/ BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>
  BasicListFS<T, LIST_TYPE, ELIST_TYPE, NELIST_TYPE, HEAD_FEATURE, TAIL_FEATURE>::
  createListFS( CAS & cas, T const & head, bool bIsPermanent ) {
    lowlevel::FSHeap & rHeap = *uima::internal::FSPromoter::getFSHeap(cas);
    lowlevel::TyFS tyFS  = rHeap.createFS( NELIST_TYPE );
    lowlevel::TyFS tyEFS = rHeap.createFS( ELIST_TYPE );
    internal::setFSValueTempl(&cas, tyFS, HEAD_FEATURE, head);
    rHeap.setFSValue(tyFS, TAIL_FEATURE, tyEFS);
    return BasicListFS(tyFS, cas, false);
  }

  // explicit instantiation
  template class BasicListFS< FeatureStructure, internal::gs_tyFSListType, internal::gs_tyEListType, internal::gs_tyNEListType, internal::gs_tyHeadFeature, internal::gs_tyTailFeature >;
  // explicit instantiation
  template class BasicListFS< float, internal::gs_tyFloatListType, internal::gs_tyEFloatListType, internal::gs_tyNEFloatListType, internal::gs_tyFloatHeadFeature, internal::gs_tyFloatTailFeature >;
  // explicit instantiation
  template class BasicListFS< int, internal::gs_tyIntListType, internal::gs_tyEIntListType, internal::gs_tyNEIntListType, internal::gs_tyIntHeadFeature, internal::gs_tyIntTailFeature >;
  // explicit instantiation
  template class BasicListFS< UnicodeStringRef, internal::gs_tyStringListType, internal::gs_tyEStringListType, internal::gs_tyNEStringListType, internal::gs_tyStringHeadFeature, internal::gs_tyStringTailFeature >;

} // namespace uima

/* ----------------------------------------------------------------------- */
/* <EOF> */

