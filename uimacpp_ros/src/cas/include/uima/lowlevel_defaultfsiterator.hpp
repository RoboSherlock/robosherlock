#ifndef UIMA_DEFAULTFSITERATOR_HPP
#define UIMA_DEFAULTFSITERATOR_HPP

/** \file lowlevel_internal_indexes.hpp .
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

#include <uima/lowlevel_index.hpp>
#include <uima/lowlevel_indexcomparator.hpp>
#include <uima/lowlevel_indexiterator.hpp>
#include <uima/lowlevel_fsheap.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {

  namespace lowlevel {
    class IndexIterator;
    class FSHeap;
  }
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/**
 * This file containts all different index types, i.e., any implementation
 * of single and composite indexes for the falvors "ordered", "set", and "FIFO".
 */
namespace uima {
  namespace lowlevel {
    /**
     * An Iterator over featurestructures in the FSHeap
     * Can only move forward!
     * 
     */
    class UIMA_LINK_IMPORTSPEC DefaultFSIterator : public IndexIterator {
    private:
      uima::lowlevel::TypeSystem const & crTypeSystem;
      uima::lowlevel::FSHeap::TyFSHeap const &  tyTempHeap;  //the fs array segments

      TyFS pEnd;                           // top of heap
      TyFS iv_it;                          // current position in heap, should be start of an FS
      uima::lowlevel::TyFSType tyType;      // heap value at iv_it == FS type

      uima::lowlevel::IndexComparator const * iv_cpComparator;
      uima::lowlevel::FSHeap const & iv_heap;

    public:
      // DefaultFSIterator( IndexComparator const * cpComparator,
      //                   uima::lowlevel::FSHeap const & heap)

      DefaultFSIterator(uima::lowlevel::FSHeap const & heap)
          :  //iv_cpComparator(cpComparator),
          iv_heap(heap),
          crTypeSystem(heap.getTypeSystem()),
          tyTempHeap(heap.iv_clTemporaryHeap) {
        pEnd = tyTempHeap.getTopOfHeap();
        iv_it = 1;
        tyType = tyTempHeap.getHeapValue(iv_it);
      }

      void moveToFirst() {
        iv_it = 1;
        tyType = tyTempHeap.getHeapValue(iv_it);
        assert( crTypeSystem.isValidType(tyType) );
        UIMA_TPRINT("  fs type: " << crTypeSystem.getTypeName(tyType) );
      }

      /**
       *  same as moveToFirst 
       */
      void moveToLast() {
        moveToFirst();
      }

      /**
       *  same as move to first
       */
      void moveToPrevious() {
        moveToNext();
      }


      bool isValid() const {
        if (iv_it >= pEnd) {
          return false;
        }
        // is valid type?
        UIMA_TPRINT("  fs type: " << crTypeSystem.getTypeName(tyType) );
        return crTypeSystem.isValidType(tyType);
      }

      TyFS get() const {
          assert(isValid());
          return iv_it;
        }

      TyFS getTyFSType() const {
        assert(isValid());
        return tyType;
      }

      void moveToNext() {
        if ( !isValid() )
          return;
        if (crTypeSystem.isArrayType(tyType)) {
          // get array size
          iv_it += 2 + tyTempHeap.getHeapValue(iv_it+1);
        } else {
          //look up number of features
          uima::lowlevel::TyFeatureOffset tySize = crTypeSystem.getFeatureNumber(tyType);
          iv_it += tySize+1;
        }

        if (!isValid())
          return;
        tyType = tyTempHeap.getHeapValue(iv_it);
        assert( crTypeSystem.isValidType(tyType) );
        UIMA_TPRINT("  fs type: " << crTypeSystem.getTypeName(tyType) );
      }

      IndexIterator* clone() const {
        return new DefaultFSIterator(iv_heap);
      }

      bool moveTo(TyFS fs) {
        return false;
      }

    };


  } //namespace lowlevel
} //namespace uima

#endif
