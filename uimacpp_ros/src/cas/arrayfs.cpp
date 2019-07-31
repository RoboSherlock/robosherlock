/** \file arrayfs.cpp .
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
#include <uima/arrayfs.hpp>
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
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */
namespace uima {

  /* ----------------------------------------------------------------------- */
  /*       Exceptions Implementation                                         */
  /* ----------------------------------------------------------------------- */

  UIMA_EXC_CLASSIMPLEMENT(FSIsNotArrayException, CASException);
  UIMA_EXC_CLASSIMPLEMENT(FSArrayOutOfBoundsException, CASException);

  /* ----------------------------------------------------------------------- */
  /*       Tool Functions Implementation                                     */
  /* ----------------------------------------------------------------------- */

  void checkArray(const uima::lowlevel::TyFSType ARRAY_TYPE, lowlevel::TyFS tyFS, uima::lowlevel::FSHeap * pFSSystem, TyMessageId tyContext) {
    assert(EXISTS(pFSSystem));
    if (pFSSystem->getType( tyFS) != ARRAY_TYPE ) {
      UIMA_EXC_THROW_NEW(FSIsNotArrayException,
                         UIMA_ERR_FS_IS_NOT_ARRAY,
                         UIMA_MSG_ID_EXC_FS_IS_NOT_ARRAY,
                         ErrorMessage(tyContext),
                         ErrorInfo::recoverable
                        );
    }
  }

  void checkArraySize(lowlevel::TyFS tyFS, uima::lowlevel::FSHeap * pFSSystem, size_t n, TyMessageId tyContext) {
    size_t uiSize = pFSSystem->getArraySize(tyFS);
    if (n >= uiSize) {
      ErrorMessage err(UIMA_MSG_ID_EXC_ARRAY_OUT_OF_BOUNDS);
      err.addParam(n);
      err.addParam(uiSize);
      UIMA_EXC_THROW_NEW(FSArrayOutOfBoundsException,
                         UIMA_ERR_FS_ARRAY_OUT_OF_BOUNDS,
                         err,
                         ErrorMessage(tyContext),
                         ErrorInfo::recoverable
                        );
    }
  }

  /* ----------------------------------------------------------------------- */
  /*       BasicArrayFS                                                      */
  /* ----------------------------------------------------------------------- */
  template< class T, const uima::lowlevel::TyFSType ARRAY_TYPE >
  BasicArrayFS<T, ARRAY_TYPE>::BasicArrayFS(lowlevel::TyFS anFS, uima::CAS & rFSSystem, bool bDoChecks) :
      FeatureStructure(anFS, rFSSystem) {
    if (bDoChecks) {
      checkValidity(UIMA_MSG_ID_EXCON_CREATING_ARRAYFS);
      checkArray(ARRAY_TYPE, iv_tyFS, iv_cas->getHeap(), UIMA_MSG_ID_EXCON_CREATING_ARRAYFS);
    }
  }

  template< class T, const uima::lowlevel::TyFSType ARRAY_TYPE >
  BasicArrayFS<T, ARRAY_TYPE>::BasicArrayFS() :
      FeatureStructure() {}

  template< class T, const uima::lowlevel::TyFSType ARRAY_TYPE >
  BasicArrayFS<T, ARRAY_TYPE>::BasicArrayFS( FeatureStructure const & fs) :
      FeatureStructure(fs) {
    // don't do checks here, exceptions should only be thrown
    // when accessing an invalid object
    if (isValid()) {
      checkArray(ARRAY_TYPE, iv_tyFS, iv_cas->getHeap(), UIMA_MSG_ID_EXCON_CREATING_ARRAYFS);
    }
    // we should not have any additional members
    assert(sizeof(BasicArrayFS) == sizeof(FeatureStructure));
  }

  template< class T, const uima::lowlevel::TyFSType ARRAY_TYPE >
  T BasicArrayFS<T, ARRAY_TYPE>::get(size_t n) const {
      checkValidity(UIMA_MSG_ID_EXCON_GETTING_FS_FROM_ARRAY);
      checkArraySize(iv_tyFS, iv_cas->getHeap(), n, UIMA_MSG_ID_EXCON_GETTING_FS_FROM_ARRAY);
      uima::lowlevel::TyFSType typecode = iv_cas->getHeap()->getType(iv_tyFS);
      T result;
      if (typecode == uima::internal::gs_tyIntArrayType ||
          typecode == uima::internal::gs_tyFloatArrayType  ||
          typecode == uima::internal::gs_tyStringArrayType ||
          typecode == uima::internal::gs_tyFSArrayType) {
        lowlevel::TyHeapCell* pArray = iv_cas->getHeap()->getCArrayFromFS(iv_tyFS);
        uima::internal::fromHeapCellTempl(pArray[n], *iv_cas, result );
      } else {
        size_t pos = iv_cas->getHeap()->getArrayOffset(iv_tyFS) + n;
        uima::internal::fromHeapCellTempl(pos, *iv_cas, result );
      }
      return result;
    }

  template< class T, const uima::lowlevel::TyFSType ARRAY_TYPE >
  void BasicArrayFS<T, ARRAY_TYPE>::set(size_t n, T const & val) {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_FS_FROM_ARRAY);
    checkArraySize(iv_tyFS, iv_cas->getHeap(), n, UIMA_MSG_ID_EXCON_GETTING_FS_FROM_ARRAY);
    uima::lowlevel::TyFSType typecode = iv_cas->getHeap()->getType(iv_tyFS);
    if (typecode == uima::internal::gs_tyIntArrayType ||
        typecode == uima::internal::gs_tyFloatArrayType  ||
        typecode == uima::internal::gs_tyStringArrayType ||
        typecode == uima::internal::gs_tyFSArrayType) {
      lowlevel::TyHeapCell* pArray = iv_cas->getHeap()->getCArrayFromFS(iv_tyFS);
      uima::lowlevel::TyHeapCell result = uima::internal::toHeapCellTempl(val, *iv_cas->getHeap(),0);
      pArray[n] = result;
    } else {
      size_t pos = iv_cas->getHeap()->getArrayOffset(iv_tyFS) + n;
      uima::lowlevel::TyHeapCell result = uima::internal::toHeapCellTempl(val, *iv_cas->getHeap(),pos);
    }

  }

  template< class T, const uima::lowlevel::TyFSType ARRAY_TYPE >
  size_t BasicArrayFS<T, ARRAY_TYPE>::size() const {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_ARRAYSIZE_FROM_FS);
    return iv_cas->getHeap()->getArraySize(iv_tyFS);
  }

  template< class T, const uima::lowlevel::TyFSType ARRAY_TYPE >
  void BasicArrayFS<T, ARRAY_TYPE>::copyToArray(
    size_t uiStart,
    size_t uiEnd,
    T* destArray,
    size_t uiDestOffset) const {
      checkValidity(UIMA_MSG_ID_EXCON_GETTING_FS_FROM_ARRAY);
      checkArraySize(iv_tyFS, iv_cas->getHeap(), uiEnd - uiStart - 1 , UIMA_MSG_ID_EXCON_GETTING_FS_FROM_ARRAY);
      uima::lowlevel::TyFSType typecode = iv_cas->getHeap()->getType(iv_tyFS);

	  size_t srcOffset = uiStart;
	  size_t numelements = uiEnd-uiStart;
	  size_t destOffset = uiDestOffset;

	  if (typecode== uima::internal::gs_tyIntArrayType || 
		  typecode== uima::internal::gs_tyFloatArrayType   ) {
			  iv_cas->getHeap()->copyToArray( srcOffset,iv_tyFS,(uima::lowlevel::TyHeapCell*) destArray,destOffset,numelements);
	  } else if(typecode== uima::internal::gs_tyByteArrayType || 
		        typecode== uima::internal::gs_tyBooleanArrayType) {
		  iv_cas->getHeap()->copyToArray( srcOffset,iv_tyFS,(char*) destArray,destOffset,numelements);
	  } else if(typecode== uima::internal::gs_tyShortArrayType ) {
		  iv_cas->getHeap()->copyToArray( srcOffset,iv_tyFS,(short*) destArray,destOffset,numelements);
      } else if(typecode== uima::internal::gs_tyLongArrayType || 
		        typecode== uima::internal::gs_tyDoubleArrayType) {
		  iv_cas->getHeap()->copyToArray( srcOffset,iv_tyFS,(INT64*) destArray,destOffset,numelements);
      } else {
      assertWithMsg(false, "Not yet implemented");
      UIMA_EXC_THROW_NEW(NotYetImplementedException,
                 UIMA_ERR_NOT_YET_IMPLEMENTED,
                 UIMA_MSG_ID_EXC_NOT_YET_IMPLEMENTED,
                 ErrorMessage(UIMA_MSG_ID_EXCON_UNKNOWN_CONTEXT),
                 ErrorInfo::unrecoverable
                );
      }
  }
  template< class T, const uima::lowlevel::TyFSType ARRAY_TYPE >
  void BasicArrayFS<T, ARRAY_TYPE>::copyFromArray(
  T const * sourceArray,
    size_t uiStart,
    size_t uiEnd,
    size_t uiOffset) {
      checkValidity(UIMA_MSG_ID_EXCON_GETTING_FS_FROM_ARRAY);
      checkArraySize(iv_tyFS, iv_cas->getHeap(), uiEnd - uiStart - 1, UIMA_MSG_ID_EXCON_GETTING_FS_FROM_ARRAY);
      
	  size_t srcOffset = uiStart;
	  size_t numelements = uiEnd-uiStart;
	  size_t destOffset = uiOffset;
	  checkArraySize(iv_tyFS, iv_cas->getHeap(), destOffset+numelements-1, UIMA_MSG_ID_EXCON_GETTING_FS_FROM_ARRAY);
      uima::lowlevel::TyFSType typecode = iv_cas->getHeap()->getType(iv_tyFS);

	  if (typecode== uima::internal::gs_tyIntArrayType || 
		  typecode== uima::internal::gs_tyFloatArrayType   ) {
	    iv_cas->getHeap()->copyFromArray(  (uima::lowlevel::TyHeapCell *) sourceArray, srcOffset,  iv_tyFS, destOffset, numelements);
      }
      else if(typecode== uima::internal::gs_tyByteArrayType || 
		  typecode== uima::internal::gs_tyBooleanArrayType ) {
		iv_cas->getHeap()->copyFromArray(  (char *) sourceArray, srcOffset,  iv_tyFS, destOffset, numelements);
       
      }
	  else if(typecode== uima::internal::gs_tyShortArrayType) {
       	iv_cas->getHeap()->copyFromArray(  (short *) sourceArray, srcOffset,  iv_tyFS, destOffset, numelements);
      }
	  else if(typecode== uima::internal::gs_tyLongArrayType ||
		  typecode == uima::internal::gs_tyDoubleArrayType ) {
       	iv_cas->getHeap()->copyFromArray(  (INT64 *) sourceArray, srcOffset,  iv_tyFS, destOffset, numelements);
      }
      else {
        assertWithMsg(false, "Not yet implemented");
        UIMA_EXC_THROW_NEW(NotYetImplementedException,
                         UIMA_ERR_NOT_YET_IMPLEMENTED,
                         UIMA_MSG_ID_EXC_NOT_YET_IMPLEMENTED,
                         ErrorMessage(UIMA_MSG_ID_EXCON_UNKNOWN_CONTEXT),
                         ErrorInfo::unrecoverable
                        );

    }
  }

  template< class T, const uima::lowlevel::TyFSType ARRAY_TYPE >
  void BasicArrayFS<T, ARRAY_TYPE>::copyToArray(
    size_t srcOffset, T * destArray, size_t destOffset, size_t length)
    const {
		copyToArray(srcOffset, srcOffset + length, destArray, destOffset ); 
  }

  // template< class T, const uima::lowlevel::TyFSType ARRAY_TYPE >
  //void BasicArrayFS<T, ARRAY_TYPE>::copyFromArray(
	 // T const * srcArray, size_t srcOffset, size_t destOffset, size_t length) {
		//copyFromArray(srcArray, srcOffset, srcOffset + length, destOffset);
  //}

  template< class T, const uima::lowlevel::TyFSType ARRAY_TYPE >
  /*static*/ BasicArrayFS<T, ARRAY_TYPE> BasicArrayFS<T, ARRAY_TYPE>::createArrayFS( CAS & cas, size_t uiSize, bool bIsPermanent) {
    assertWithMsg( sizeof(FeatureStructure::TyArrayElement) == sizeof(lowlevel::TyHeapCell), "Port required");
    uima::lowlevel::FSHeap & heap =  *uima::internal::FSPromoter::getFSHeap(cas);
    lowlevel::TyFS tyFS = heap.createArrayFS(ARRAY_TYPE, uiSize);
    return BasicArrayFS(internal::FSPromoter::promoteFS( tyFS, cas ));
  }

  // explicit instantiation
  template class BasicArrayFS< FeatureStructure, internal::gs_tyFSArrayType >;
  // explicit instantiation
  template class BasicArrayFS< float, internal::gs_tyFloatArrayType >;
  // explicit instantiation
  template class BasicArrayFS< int, internal::gs_tyIntArrayType >;
  // explicit instantiation
  template class BasicArrayFS< UnicodeStringRef, internal::gs_tyStringArrayType >;


  // explicit instantiation
  template class BasicArrayFS< bool, internal::gs_tyBooleanArrayType >;
  template class BasicArrayFS< char, internal::gs_tyByteArrayType >;
  template class BasicArrayFS< short, internal::gs_tyShortArrayType >;
  template class BasicArrayFS< INT64, internal::gs_tyLongArrayType >;
  template class BasicArrayFS< double, internal::gs_tyDoubleArrayType >;



} // namespace uima

/* ----------------------------------------------------------------------- */
/* <EOF> */

