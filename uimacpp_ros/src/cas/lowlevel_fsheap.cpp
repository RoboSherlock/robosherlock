/** \file lowlevel_fsheap.cpp .
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

//#define DEBUG_VERBOSE
#include <uima/macros.h>

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
using namespace std;
namespace uima {

  namespace lowlevel {

    TyFS const FSHeap::INVALID_FS = 0;

    FSHeap::FSHeap(TypeSystem const & rclTypeSystem,
                   size_t uiFSHeapPageSize,
                   size_t uiStringHeapPageSize,
                   size_t uiStringRefHeapPageSize) :
        iv_clTemporaryHeap(uiFSHeapPageSize, 0),
        iv_clTemporaryStringHeap(uiStringHeapPageSize, 0),
        iv_clTemporaryStringRefHeap(uiStringRefHeapPageSize, 0),
        iv_clTemporary8BitHeap(1, 0),
        iv_clTemporary16BitHeap(1, 0),
        iv_clTemporary64BitHeap(1, 0),
        iv_rclTypeSystem(rclTypeSystem) {
      UIMA_TPRINT("heap constructed");
      assert( INVALID_FS == 0 );
    }

    FSHeap::FSHeap(TypeSystem const & rclTypeSystem,
                   size_t uiFSHeapPageSize,
                   size_t uiStringHeapPageSize,
                   size_t uiStringRefHeapPageSize,
                   size_t uiMinHeapPageSize) :
        iv_clTemporaryHeap(uiFSHeapPageSize, 0),
        iv_clTemporaryStringHeap(uiStringHeapPageSize, 0),
        iv_clTemporaryStringRefHeap(uiStringRefHeapPageSize, 0),
        iv_clTemporary8BitHeap(uiMinHeapPageSize, 0),
        iv_clTemporary16BitHeap(uiMinHeapPageSize, 0),
        iv_clTemporary64BitHeap(uiMinHeapPageSize, 0),
        iv_rclTypeSystem(rclTypeSystem) {
      UIMA_TPRINT("heap constructed");
      assert( INVALID_FS == 0 );
    }


    FSHeap::~FSHeap() {}

    void FSHeap::reset() {
      assert( debugIsConsistent() );
      TyFSHeap & rHeapToBeResetted = getHeap();
      rHeapToBeResetted.reset();
      TyStringHeap & rStringHeapToBeResetted = getStringHeap();
      rStringHeapToBeResetted.reset();
      TyStringRefHeap & rStringRefHeapToBeResetted = getStringRefHeap();
      rStringRefHeapToBeResetted.reset();

      Ty8BitHeap & r8BitHeapToBeResetted = get8BitHeap();
      r8BitHeapToBeResetted.reset();
      Ty16BitHeap & r16BitHeapToBeResetted = get16BitHeap();
      r16BitHeapToBeResetted.reset();
      Ty64BitHeap & r64BitHeapToBeResetted = get64BitHeap();
      r64BitHeapToBeResetted.reset();
    }


    void FSHeap::setStringValue(TyFS tyFs, TyFSFeature tyFeature, int strRef) {
      assert( iv_rclTypeSystem.isValidFeature(tyFeature) );
      assert( iv_rclTypeSystem.subsumes(uima::internal::gs_tyStringType, iv_rclTypeSystem.getRangeType(tyFeature)));

#ifndef NDEBUG
      TyFSType rangeType = iv_rclTypeSystem.getRangeType(tyFeature);
      // if we have an enumeration subtype of string
      if (rangeType != uima::internal::gs_tyStringType ) {
        vector<icu::UnicodeString> const & crStringSet = iv_rclTypeSystem.getStringsForStringSubtype(rangeType);
        vector<icu::UnicodeString>::const_iterator cit;
        bool bStringFound = false;
        UnicodeStringRef crStringRef(iv_clTemporaryStringHeap.getHeapStart()+strRef);
        for (cit = crStringSet.begin(); cit != crStringSet.end(); ++cit) {
          UnicodeStringRef ref(*cit);
          if ( ref == crStringRef ) {
            bStringFound = true;
            break;
          }
        }
        assert (bStringFound);
      }
#endif

      TyFS tyCurrentString = getFeatureInternal( tyFs, tyFeature );
      if (tyCurrentString == 0) {
        // add entry to StringRefHeap and set values there
        tyCurrentString = getStringAsFS(strRef);
      } else {
        // reuse entry in StringRefHeap
        setStringRef(tyCurrentString, strRef);
      }

      setFeatureInternal(tyFs, tyFeature, tyCurrentString);
    }

    void FSHeap::copyFeatures(TyFS tyTarget, TyFS tySource, size_t featNum) {
      uima::lowlevel::TyFSType type = getType(tyTarget);
      if (featNum == 0) {
        assert( type == getType(tySource) );
        featNum = iv_rclTypeSystem.getFeatureNumber( getType(tyTarget));
      }
      size_t i;
      for (i=1; i<=featNum; ++i) {
        uima::lowlevel::TyFSFeature feat = iv_rclTypeSystem.getFeatureFromOffset(type, i);
        if ( iv_rclTypeSystem.subsumes( uima::internal::gs_tyStringType, iv_rclTypeSystem.getRangeType(feat) ) ) {
          // get offset of stringRef in stringRefHeap
          int strRef = iv_clTemporaryHeap.getHeapValue(tySource+i);
          // if string value set, create duplicate reference to string in refHeap
          if (strRef != 0) {
            int ref = iv_clTemporaryStringRefHeap.getHeapValue(1 + 2*(strRef-1));
            iv_clTemporaryHeap.setHeapValue(tyTarget+i, getStringAsFS(ref));
          }
        } else {
          iv_clTemporaryHeap.setHeapValue(tyTarget+i, iv_clTemporaryHeap.getHeapValue(tySource+i));
        }
      }

    }



// cout << of a 64bit type causes a warning (conversion to int, loss of data)
// since we can't help the implementation of cout we surpress the warning
#if defined( _MSC_VER )
#pragma warning( push )
#pragma warning( disable : 4244 )
#endif
    void FSHeap::printFS(ostream& os, TyFS fs) const {
      TyFSType type = getType( fs );
      icu::UnicodeString typeName = iv_rclTypeSystem.getTypeName(type);
      int n=0;
      os << n++ << ":  | " << typeName << endl;
      unsigned int i;
      for (i=1; i<iv_rclTypeSystem.getFeatureNumber(type) + 1; ++i) {
        os << n++ << ":  | " << (ptrdiff_t) getFeatureWithOffset(fs, i) << endl;
      }
    }


    void FSHeap::print(ostream& os) const {
      os << "=============================================" << endl;
#ifdef BYEBYEPTRS
      void * h;
      size_t i;
      size_t lastHeapCell = iv_clTemporaryHeap.getLastHeapCell();
      size_t topOfHeap = iv_clTemporaryHeap.getTopOfHeap();

      if (true) {
        os << "Plain HEAP:" << endl;
        h=iv_clTemporaryHeap.getHeapStart();
        i=0;
        for (h=1; h< lastHeapCell; ++h) {
          os << i++ << " (" << (ptrdiff_t) h << ", " << h << ") :  | " << ((ptrdiff_t) (*h));
          if (h == topOfHeap) {
            os << "   <---" << endl;
            break;
          }

          os << endl;
        }
      }
      os << "-----------------------------" << endl;


      UChar* uc = iv_clTemporaryStringHeap.getHeapStart();
      size_t topOfStringHeap = iv_clTemporaryStringHeap.getTopOfHeap();

      if (false) {
        os << "PLAIN STRING TABLE" << endl;
        while ((uc-iv_clTemporaryStringHeap.getHeapStart())<topOfStringHeap) {
          os << (ptrdiff_t) uc << ": " << *uc << "(" << (ptrdiff_t) (*uc) << ")" << endl;
          ++uc;
        }
        os << "-----------------------------" << endl;
        uc = iv_clTemporaryStringHeap.getHeapStart();
        topOfStringHeap = iv_clTemporaryStringHeap.getTopOfHeap();
      }
      os << "String table: " << endl;
      while ( (uc-iv_clTemporaryStringHeap.getHeapStart()) < topOfStringHeap) {
        UnicodeStringRef uls(uc);
        os << " " << (ptrdiff_t) uc << ": " << uls << endl;
        uc += uls.length() + 1;
      }

      os << "-----------------------------" << endl;
      os << "HEAP:" << endl;
      h = iv_clTemporaryHeap.getHeapStart() + 1;
      int n=1;
      while (h<lastHeapCell) {
        if ( (*h) == 0 ) {
          return;
        }
        TyFS currentFS = h;
        TyFSType type = getType( h );
        icu::UnicodeString typeName = iv_rclTypeSystem.getTypeName(type);
        os << n++ << ":  (" << (ptrdiff_t) h << ")| " << typeName << endl;
        ++h;
        int i;
        for (i=0; i< (int) iv_rclTypeSystem.getFeatureNumber(type); ++i) {
          os << n++ << ":  (" << (ptrdiff_t) h << ")| " << ((ptrdiff_t) (*h)) << endl;
          ++h;
        }
        if (iv_rclTypeSystem.subsumes( uima::internal::gs_tyArrayBaseType, type) ) {
          ++h; // for the array size field
          int iArraySize = getArraySize(currentFS);
          for (i=0; i<iArraySize; ++i) {
            os << n++ << ":  (" << (ptrdiff_t) h << ")| " << ((ptrdiff_t) (*h)) << endl;
            ++h;
          }
        }
      }
#endif
    }
#if defined( _MSC_VER )
#pragma warning( pop )
#endif

#ifdef ASSERT_OR_RETURN_FALSE
#error ASSERT_OR_RETURN_FALSE already defined
#endif
#define ASSERT_OR_RETURN_FALSE(x) if (!(x)) return false
//#define ASSERT_OR_RETURN_FALSE(x) assert(x)


    bool FSHeap::debugIsConsistent() const {
#ifdef BYEBYEPTRS
      if (iv_clTemporaryHeap.getSegmentNumber() >= 1) {
        return true;
      }
//         print(cout);
      static int debugCheck = 0;
      //   cerr << " debugCheck: " << debugCheck << endl;
      ++debugCheck;
      if ( (debugCheck % 1) != 0) {
//            return true;
      }
      UIMA_TPRINT("  doing check... (debugCheck: " << debugCheck << ")");
      TyFSType integerType = iv_rclTypeSystem.getTypeByName("uima.cas.Integer");
      TyFSType floatType = iv_rclTypeSystem.getTypeByName("uima.cas.Float");
      TyFSType stringType = iv_rclTypeSystem.getTypeByName("uima.cas.String");

      size_t lastHeapCell = iv_clTemporaryHeap.getLastHeapCell();
      size_t h = 1;
      while (h<lastHeapCell) {
        UIMA_TPRINT("Checking heap cell " << ((lastHeapCell-h)) );
        ASSERT_OR_RETURN_FALSE( EXISTS(h) );
        ASSERT_OR_RETURN_FALSE( debugIsValidHeapCell(h) );

        if ( (*h) == 0) {
          do {
            ASSERT_OR_RETURN_FALSE( (*h) == 0 );
            ++h;
          } while (h<lastHeapCell);
          UIMA_TPRINT("   check done");
          return true;
        }
        ASSERT_OR_RETURN_FALSE( *h != 0 );

        TyFSType type = getType( h );
        ASSERT_OR_RETURN_FALSE( type != 0 );
        vector<TyFSFeature> features;
        iv_rclTypeSystem.getAppropriateFeatures(type, features);
        ASSERT_OR_RETURN_FALSE( features.size() == iv_rclTypeSystem.getFeatureNumber(type) );
        size_t i;
        for (i=0; i<features.size(); ++i) {
          TyFSFeature f = features[i];
          TyFSType rangeType = iv_rclTypeSystem.getRangeType( f );
          if ( (rangeType == integerType)
               || (rangeType == floatType)
             || iv_rclTypeSystem.subsumes(stringType, rangeType) ) {}
          else {
            if (!isUntouchedFSValue(h,f)) {
              // type is complex FS
              TyFS fs = getFSValue(h, f);
              ASSERT_OR_RETURN_FALSE(debugIsValidHeapCell(fs));
              TyFSType fsType = getType(fs);
              ASSERT_OR_RETURN_FALSE( iv_rclTypeSystem.subsumes(rangeType, fsType) );
            }
          }
        }
        h += features.size() + 1;
      }
#endif
      UIMA_TPRINT("   check done");
      return true;
    }


    bool FSHeap::debugIsValidHeapCell(TyHeapCell cell) const {
      return iv_clTemporaryHeap.debugIsValidHeapCell(cell);
    }

  }
}
/* ----------------------------------------------------------------------- */





