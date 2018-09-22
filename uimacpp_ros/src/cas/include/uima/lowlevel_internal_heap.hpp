#ifndef UIMA_LOWLEVEL_HEAP_HPP
#define UIMA_LOWLEVEL_HEAP_HPP
/** \file lowlevel_heap.hpp .
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

   Description: Base Heap design. 32-bit heaps used for interop

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp>

#include <uima/lowlevel_typedefs.hpp>

#include <uima/macros.h>

#include <vector>

#if defined( _MSC_VER )
#pragma warning( push )
#pragma warning( disable : 4311 )
#pragma warning( disable : 4312 )
#endif

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  namespace internal {
    class CASSerializer;
    class CASDeserializer;
  }
}
/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace lowlevel {
    namespace internal {

      /**
       * Generic implementation of a possibly segmented heap of cells of type T.
       * Used in uima::lowlevel::FSHeap for the FS heap, the StringRef Heap,
       * and the String heap. T must be an integral type.
       */
      template <class T>
      class UIMA_LINK_IMPORTSPEC Heap {
        friend class uima::internal::CASSerializer;
        friend class uima::internal::CASDeserializer;
#if defined(__GNUC__) && (__GNUC__ < 3)
        //Older GNU compiler can't handle friend properly so we make public
      public:
#else
      private:
#endif
        // start of the current heap segment
        T* iv_pTHeap;
        // current top of heap
        T* iv_pTTopOfHeap;
        // end of current allocated heap segment
        T* iv_pTLastHeapCell;
        // all starts of all used segments so far (most recently allocated first)
	std::vector<T*> iv_vecUsedSegments;
        // all ends of all used segments so far (most recently allocated first)
	std::vector<T*> iv_vecUsedSegmentEnds;
        // all tops of all used segments so far (most recently allocated first)
	std::vector<T*> iv_vecUsedSegmentTops;

        // next available cell on heap
        WORD32 iv_topOfHeap;
        // segment size
        WORD32 iv_uiCellNumber;
        // default value of the heap cells
        int iv_iDefaultValue;

#ifdef BYEBYEPTRS
        /**
         * returns -1 if t lies in the current heap, an index >=0 into
         * usedSegments, if t resides there, or -2 if t could not be found at all.
         */
        int searchIndex(T* t) const {
          if ( (t >= iv_pTHeap) && (t < iv_pTTopOfHeap) ) {
            return -1;
          }
          WORD32 i;
          for (i=0; i<iv_vecUsedSegments.size(); ++i) {
            if ( (t>=iv_vecUsedSegments[i]) && (t< iv_vecUsedSegmentEnds[i] ) ) {
              return(int) i;
            }
          }
          return -2;
        }
#endif

      public:
        Heap(WORD32 cells, int value)
            : iv_pTHeap(NULL),
            iv_pTTopOfHeap(NULL),
            iv_pTLastHeapCell(NULL),
            iv_uiCellNumber(cells),
            iv_iDefaultValue(value) {
          iv_pTHeap = new T[iv_uiCellNumber];
          iv_topOfHeap = 1; // leave 0th cell empty
          memset(iv_pTHeap, iv_iDefaultValue, (iv_uiCellNumber * sizeof(T)) );
        }


        ~Heap() {
          if (iv_pTHeap != NULL) {
            assert( EXISTS(iv_pTHeap) );
            delete[] iv_pTHeap;
          }
          iv_pTHeap = NULL;
        }


        bool debugIsValidHeapCell(TyFS cell) const {
          if ( cell < 1 || cell >= iv_topOfHeap ) {
            return false;
          }
          return true;
        }

        T* getHeapStart() const {
          return iv_pTHeap;
        }
        WORD32 getTopOfHeap() const {
          return iv_topOfHeap;
        }
        WORD32 getLastHeapCell() const {
          return iv_uiCellNumber;
        }

        T getHeapValue(TyFS tyFS) const {
          return iv_pTHeap[tyFS];
        }

        void setHeapValue(TyFS tyFS, T t) {
          iv_pTHeap[tyFS] = t;
        }


        /**
         * reserve n more heap cells; if necessary, increase heap size
         */
        int increaseHeap(WORD32 n) {
          //assert( n > 0 );
          int result = iv_topOfHeap;
          iv_topOfHeap += n;

          // usual case, just return
          if ( iv_topOfHeap < iv_uiCellNumber ) {
            return result;
          }

          // calculate new heap size to be at least double current size
          if ( n > iv_uiCellNumber ) {
            iv_uiCellNumber += n;
          } else {
            iv_uiCellNumber *= 2;
          }
          T* iv_pTHeapNew = new T[iv_uiCellNumber];
          assert( iv_pTHeapNew != NULL );
          memcpy(iv_pTHeapNew, iv_pTHeap, result*sizeof(T));
          memset(iv_pTHeapNew + result, iv_iDefaultValue, (iv_uiCellNumber - result)*sizeof(T));
          delete[] iv_pTHeap;
          iv_pTHeap = iv_pTHeapNew;
          return result;
        }

        /**
         * reset this heap. Paint over used cells.
         */
        void reset() {
          UIMA_TPRINT("resetting heap");
          if ( iv_topOfHeap == 1 ) {
            // nothing to be done
            return;
          }
          // !TODO don't need to fill all heaps with default value?????
          memset(iv_pTHeap, iv_iDefaultValue, iv_topOfHeap * sizeof(T) );
          iv_topOfHeap = 1;
          UIMA_TPRINT("  heap reset'd");
        }

        bool resides(TyFS t) const {
          return debugIsValidHeapCell(t);
        }

#ifdef BYEBYEPTRS
        WORD32 getPageSize() const {
          return iv_uiCellNumber;
        }

        WORD32 getSegmentNumber() const {
          assert( iv_vecUsedSegments.size() == iv_vecUsedSegmentEnds.size() );
          return iv_vecUsedSegmentEnds.size() + 1;
        }

        /**
         * method used during serialization.
         * sets the output parameter to a vector of pairs of pointers to
         * the start and end, respectively, of the used heap segments
         * in the order they were allocated.
         */
        void getHeapSegments( vector<pair<T*, T*> >& rvecResult ) const {
          assert( iv_vecUsedSegments.size() == iv_vecUsedSegmentEnds.size() );
          WORD32 uiSegmentNum = iv_vecUsedSegmentEnds.size() + 1;
          rvecResult.resize(uiSegmentNum);
          if (uiSegmentNum == 1) {
            rvecResult[0] = pair<T*, T*>(iv_pTHeap, iv_pTTopOfHeap);
            return;
          }
          // fill result from back to front
          rvecResult[uiSegmentNum-1] = pair<T*, T*>(iv_pTHeap, iv_pTTopOfHeap);
          int i;
          for (i=uiSegmentNum-2; i>=1; --i) {
            rvecResult[i] = pair<T*, T*>(iv_vecUsedSegments[i], iv_vecUsedSegmentEnds[i]);
          }
          assert( i == 0 );
          rvecResult[0] = pair<T*, T*>(iv_vecUsedSegments[0], iv_vecUsedSegmentEnds[0]);
        }

        /**
         * method used during DEserialization.
         * sets the output parameter to a vector of pairs of pointers to
         * the start and TOP, respectively, of the used heap segments
         * in the order they were allocated.
         */
        void getHeapSegmentBounds( vector<pair<T*, T*> >& rvecResult ) const {
          assert( iv_vecUsedSegments.size() == iv_vecUsedSegmentEnds.size() );
          WORD32 uiSegmentNum = iv_vecUsedSegmentEnds.size() + 1;
          rvecResult.resize(uiSegmentNum);
          if (uiSegmentNum == 1) {
            rvecResult[0] = pair<T*, T*>(iv_pTHeap, iv_pTLastHeapCell);
            return;
          }
          // fill result from back to front
          rvecResult[uiSegmentNum-1] = pair<T*, T*>(iv_pTHeap, iv_pTLastHeapCell);
          int i;
          for (i=uiSegmentNum-2; i>=1; --i) {
            rvecResult[i] = pair<T*, T*>(iv_vecUsedSegments[i], iv_vecUsedSegmentTops[i]);
          }
          assert( i == 0 );
          rvecResult[0] = pair<T*, T*>(iv_vecUsedSegments[0], iv_vecUsedSegmentTops[0]);
        }

        /**
         * get a unique ID for a pointer into the heap.
         */
        ptrdiff_t getUniqueID(T const * t) const {
          if (iv_vecUsedSegmentEnds.size() == 0) {
            if ( (t > iv_pTHeap) && (t < iv_pTTopOfHeap) ) {
              return(t - iv_pTHeap);
            }
          } else {
            // if in the first allocated segment
            if ( (t>iv_vecUsedSegments[0]) && (t< iv_vecUsedSegmentEnds[0] ) ) {
              return(t - iv_vecUsedSegments[0]);
            }
            // keep absolute number of heap cells
            WORD32 uiAbsoluteHeapCellNum = iv_vecUsedSegmentEnds[0] - iv_vecUsedSegments[0];
            WORD32 i;
            WORD32 len = iv_vecUsedSegments.size();
            // for each allocated heap segment
            for (i=1; i<len; ++i) {
              T* segmentStart = iv_vecUsedSegments[i];
              T* segmentEnd = iv_vecUsedSegmentEnds[i];
              if ( (t>segmentStart) && (t< segmentEnd ) ) {
                return uiAbsoluteHeapCellNum - 1 + (t - segmentStart);
              }
              uiAbsoluteHeapCellNum += (segmentEnd - segmentStart) - 1;
            }
            // if in the current (most recently allocated) heap segment
            if ( (t > iv_pTHeap) && (t < iv_pTTopOfHeap) ) {
              return uiAbsoluteHeapCellNum + (t - iv_pTHeap) - 1;
            }
          }
          // if we arrived here, something went wrong
          assert(false);
          return -1;
        }
#endif
      };

    }
  }
}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */

#if defined( _MSC_VER )
#pragma warning( pop )
#endif

#endif

