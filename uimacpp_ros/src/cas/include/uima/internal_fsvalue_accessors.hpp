#ifndef UIMA_INTERNAL_FS_ACCESSORS_HPP
#define UIMA_INTERNAL_FS_ACCESSORS_HPP
/** \file internal_fs_accessors.hpp .
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
#include <uima/lowlevel_typedefs.hpp>
#include <uima/featurestructure.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  class CAS;

  namespace internal {

    inline void getFSValueTempl(uima::CAS * cas, lowlevel::TyFS tyFS, lowlevel::TyFSFeature tyFeat, lowlevel::TyFS & rResult) {
      rResult = cas->getHeap()->getFSValue( tyFS, tyFeat );
    }

    inline void getFSValueTempl(uima::CAS * cas, lowlevel::TyFS tyFS, lowlevel::TyFSFeature tyFeat, FeatureStructure & rResult) {
      lowlevel::TyFS tyFSValue = cas->getHeap()->getFSValue( tyFS, tyFeat );
      rResult = internal::FSPromoter::promoteFS(tyFSValue, *cas);
    }

    inline void getFSValueTempl(uima::CAS * cas, lowlevel::TyFS tyFS, lowlevel::TyFSFeature tyFeat, float & rResult) {
      rResult = cas->getHeap()->getFloatValue( tyFS, tyFeat );
    }

    inline void getFSValueTempl(uima::CAS *cas, lowlevel::TyFS tyFS, lowlevel::TyFSFeature tyFeat, int & rResult) {
      rResult = cas->getHeap()->getIntValue( tyFS, tyFeat );
    }

    inline void getFSValueTempl(uima::CAS * cas, lowlevel::TyFS tyFS, lowlevel::TyFSFeature tyFeat, UnicodeStringRef & rResult) {
      lowlevel::TyFS tyStringFS = cas->getHeap()->getFSValue( tyFS, tyFeat);
      rResult = cas->getHeap()->getFSAsString( tyStringFS );
    }


    inline void setFSValueTempl(uima::CAS * cas, lowlevel::TyFS tyFS, lowlevel::TyFSFeature tyFeat, lowlevel::TyFS val) {
      cas->getHeap()->setFSValue(tyFS, tyFeat, val);
    }
    inline void setFSValueTempl(uima::CAS * cas, lowlevel::TyFS tyFS, lowlevel::TyFSFeature tyFeat, FeatureStructure const & val) {
      cas->getHeap()->setFSValue(tyFS, tyFeat, internal::FSPromoter::demoteFS(val));
    }
    inline void setFSValueTempl(uima::CAS * cas, lowlevel::TyFS tyFS, lowlevel::TyFSFeature tyFeat, float val) {
      cas->getHeap()->setFloatValue(tyFS, tyFeat, val);
    }
    inline void setFSValueTempl(uima::CAS * cas, lowlevel::TyFS tyFS, lowlevel::TyFSFeature tyFeat, int val) {
      cas->getHeap()->setIntValue(tyFS, tyFeat, val);
    }
    inline void setFSValueTempl(uima::CAS * cas, lowlevel::TyFS tyFS, lowlevel::TyFSFeature tyFeat, UnicodeStringRef const & val) {
      int ulsHeap = cas->getHeap()->addString(val);
      cas->getHeap()->setStringValue(tyFS, tyFeat, ulsHeap );
      assert( cas->getHeap()->getStringValue(tyFS, tyFeat) == val );
    }


    inline lowlevel::TyHeapCell toHeapCellTempl( FeatureStructure const & fs,
        uima::lowlevel::FSHeap & heap,
        lowlevel::TyHeapCell offset) {
      return (lowlevel::TyHeapCell)uima::internal::FSPromoter::demoteFS(fs);
    }

    inline lowlevel::TyHeapCell toHeapCellTempl(float f,
        uima::lowlevel::FSHeap & heap,
        lowlevel::TyHeapCell offset) {
      return uima::lowlevel::FSHeap::getAsFS(f);
    }

    inline lowlevel::TyHeapCell toHeapCellTempl(int f,
        uima::lowlevel::FSHeap & heap,
        lowlevel::TyHeapCell offset) {
      return uima::lowlevel::FSHeap::getAsFS(f);
    }

    inline lowlevel::TyHeapCell toHeapCellTempl( UnicodeStringRef const & s,
        uima::lowlevel::FSHeap & heap,
        lowlevel::TyHeapCell offset) {
      int ulsHeap = heap.addString(s);
      lowlevel::TyFS tyStringFS = heap.getStringAsFS( ulsHeap );
      return (lowlevel::TyHeapCell)tyStringFS;
      //  return (lowlevel::TyHeapCell) ulsHeap;
    }

    //new
    inline lowlevel::TyHeapCell toHeapCellTempl(bool f,
        uima::lowlevel::FSHeap & heap,
        lowlevel::TyHeapCell offset) {
      heap.setArrayElement(f,offset);
      return offset;
    }

    inline lowlevel::TyHeapCell toHeapCellTempl(char f,
        uima::lowlevel::FSHeap & heap,
        lowlevel::TyHeapCell offset) {
      heap.setArrayElement(f,offset);
      return offset;
    }

    inline lowlevel::TyHeapCell toHeapCellTempl(short f,
        uima::lowlevel::FSHeap & heap,
        lowlevel::TyHeapCell offset) {
      heap.setArrayElement(f,offset);
      return offset;
    }

    inline lowlevel::TyHeapCell toHeapCellTempl(INT64 f,
        uima::lowlevel::FSHeap & heap,
        lowlevel::TyHeapCell offset) {
      heap.setArrayElement(f,offset);
      return offset;
    }

    inline lowlevel::TyHeapCell toHeapCellTempl(double f,
        uima::lowlevel::FSHeap & heap,
        lowlevel::TyHeapCell offset) {
      heap.setArrayElement(f,offset);
      return offset;
    }


    inline void fromHeapCellTempl( lowlevel::TyHeapCell tyCell, uima::CAS & cas, FeatureStructure & rResult) {
      // if tyCell is an annotation, might create FS from different CAS
      CAS & other = cas.getCasForTyFS(tyCell);
      rResult = uima::internal::FSPromoter::promoteFS((lowlevel::TyFS)tyCell, other);
    }

    inline void fromHeapCellTempl( lowlevel::TyHeapCell tyCell, uima::CAS &, float & rResult ) {
      rResult = uima::lowlevel::FSHeap::getFSAsFloat( (lowlevel::TyFS) tyCell);
    }

    inline void fromHeapCellTempl( lowlevel::TyHeapCell tyCell, uima::CAS &, int & iResult ) {
      iResult = uima::lowlevel::FSHeap::getFSAsInt( (lowlevel::TyFS) tyCell);
    }

    inline void fromHeapCellTempl( lowlevel::TyHeapCell tyCell, uima::CAS & cas, UnicodeStringRef & rResult) {
      rResult = cas.getHeap()->getFSAsString((lowlevel::TyFS)tyCell);
    }

    //new types
    inline void fromHeapCellTempl( lowlevel::TyHeapCell tyCell, uima::CAS & cas,bool & rResult) {
      rResult = cas.getHeap()->getBoolean(tyCell);
    }

    inline void fromHeapCellTempl( lowlevel::TyHeapCell tyCell, uima::CAS & cas, char & rResult) {
      rResult = cas.getHeap()->getByte(tyCell);
    }

    inline void fromHeapCellTempl( lowlevel::TyHeapCell tyCell, uima::CAS & cas, short & rResult) {
      rResult = cas.getHeap()->getShort(tyCell);
    }

    inline void fromHeapCellTempl( lowlevel::TyHeapCell tyCell, uima::CAS & cas, INT64 & rResult) {
      rResult = cas.getHeap()->getLong(tyCell);
    }

    inline void fromHeapCellTempl( lowlevel::TyHeapCell tyCell, uima::CAS & cas, double & rResult) {
      rResult = cas.getHeap()->getDouble(tyCell);
    }




  } // namespace internal
} // namespace uima

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */


#endif

