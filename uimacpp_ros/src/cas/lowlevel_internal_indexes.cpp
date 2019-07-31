/** \file lowlevel_internal_indexes.cpp .
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

//#define DEBUG_VERBOSE
/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp>

#include <uima/macros.h>

#include <uima/lowlevel_internal_indexes.hpp>
#include <uima/lowlevel_indexcomparator.hpp>
#include <uima/lowlevel_indexiterator.hpp>
#include <uima/lowlevel_indexrepository.hpp>

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
    namespace internal {

      /**
       * A class which is a model of Compare STL interface for
       * IndexComparators.
       */
      class FSIndexCompare {
      private:
        uima::lowlevel::IndexComparator const * iv_cpComparator;
        uima::lowlevel::FSHeap const & iv_heap;
      public:
        FSIndexCompare(uima::lowlevel::IndexComparator const * cpComparator,
                       uima::lowlevel::FSHeap const & heap)
            : iv_cpComparator(cpComparator),
            iv_heap(heap) {}

        bool operator()(uima::lowlevel::TyFS fs1, uima::lowlevel::TyFS fs2) const {
          return iv_cpComparator->compare(iv_heap, fs1, fs2) > 0;
        }
      };


      /**
       * A generic implementation of an IndexIterator over STL containers.
       * The parameter T can be a vector or a set.
       */
      template <class T>
      class STLSingleIndexIterator : public IndexIterator {
      private:
        T const & iv_crStructures;
        typename T::const_iterator iv_it;
        uima::lowlevel::IndexComparator const * iv_cpComparator;
        uima::lowlevel::FSHeap const & iv_heap;
      public:
        STLSingleIndexIterator(T const & crStructures,
                               IndexComparator const * cpComparator,
                               uima::lowlevel::FSHeap const & heap)
            : iv_crStructures(crStructures),
            iv_it(iv_crStructures.end()),
            iv_cpComparator(cpComparator),
            iv_heap(heap) {}

        void moveToFirst() {
          iv_it = iv_crStructures.begin();
        }

        void moveToLast() {
          iv_it =  iv_crStructures.begin();
          if (iv_it == iv_crStructures.end() ) {
            return;
          }

          /*
                         typename T::const_iterator itNext = iv_it;
                         ++itNext;
                         while (itNext != iv_crStructures.end()) {
                            ++iv_it;
                            assert( iv_it == itNext );
                            assert( iv_it != iv_crStructures.end() );
                            ++itNext;
                         }
          */
          iv_it = iv_crStructures.end();
          --iv_it;
        }

        void moveToPrevious() {
          assert( isValid() );
          if (iv_it == iv_crStructures.begin() ) {
            iv_it = iv_crStructures.end();
          } else {
            --iv_it;
          }
        }

        bool isValid() const {
          return iv_it != iv_crStructures.end();
        }

        TyFS get() const {
            assert(isValid());
            return *iv_it;
          }

        TyFS getTyFSType() const {
          assert(isValid());
          return iv_heap.getType(*iv_it);
        }

        void moveToNext() {
          assert( isValid() );
          ++iv_it;
        }

        IndexIterator* clone() const {
          return new STLSingleIndexIterator<T>(*this);
        }

        bool moveTo(TyFS fs) {
          typename T::const_iterator cit;

          if (iv_cpComparator == NULL) {
            cit = find(iv_crStructures.begin(), iv_crStructures.end(), fs);
            if (cit != iv_crStructures.end()) {
              ++cit;
            }
          } else {
            // first try to find the same fs
            cit = find(iv_crStructures.begin(), iv_crStructures.end(), fs);
            // if not found, find an equivalent one
            if (cit == iv_crStructures.end()) {
              FSIndexCompare comp( iv_cpComparator, iv_heap );
              cit = lower_bound(iv_crStructures.begin(), iv_crStructures.end(), fs, comp);
            }
          }

          bool bCanBeSet = ( cit != iv_crStructures.end() );
          if (bCanBeSet) {
            iv_it = cit;
            return true;
          } else {
            return false;
          }
        }

      };



      CompositeIndex::CompositeIndex(IndexRepository const & crIndexRepository,
                                     uima::lowlevel::TyFSType tyType)
          : IndexABase(crIndexRepository, tyType) {}

      void CompositeIndex::getComponentsForTypes(set<uima::lowlevel::TyFSType> const & crTypes, TyComponents & crResult) const {
        set<uima::lowlevel::TyFSType>::const_iterator cit;
#ifndef NDEBUG
        // check that all the types are subsumed by the type of this index
        for (cit = crTypes.begin(); cit != crTypes.end(); ++cit) {
          assert( iv_crTypeSystem.subsumes( iv_tyFSType, *cit ) ) ;
        }
#endif

        crResult.clear();
        for (cit = crTypes.begin(); cit != crTypes.end(); ++cit) {
          TyComponents::const_iterator compit;
          for (compit = iv_tyComponents.begin(); compit != iv_tyComponents.end(); ++compit) {
            if ( (*compit)->getType() == (*cit) ) {
              crResult.push_back( *compit );
              break;
            }
          }
        }
        assert( crResult.size() == crTypes.size() );
      }



      size_t CompositeIndex::getSize() const {
        TyComponents::const_iterator cit;
        size_t result = 0;
        // get sizes of all components
        for (cit = iv_tyComponents.begin(); cit != iv_tyComponents.end(); ++cit) {
          result += (*cit)->getSize();
        }
        return result;
      }



      TyFS CompositeIndex::find(TyFS fs) const {
        // find fs in any component
        TyComponents::const_iterator it;
        for (it = iv_tyComponents.begin(); it != iv_tyComponents.end(); ++it) {
          TyFS foundFS = (*it)->find(fs);
          if (foundFS != uima::lowlevel::FSHeap::INVALID_FS) {
            return foundFS;
          }
        }
        return  uima::lowlevel::FSHeap::INVALID_FS;
      }

      template <class T>
      IndexIterator* CachedCompositeIndex<T>::createIterator() const {
        if (iv_crIndexRepository.isDirtyForIndex( this )) {
          CachedCompositeIndex<T> * nonConstThis = CONST_CAST( CachedCompositeIndex<T> *, this);
          nonConstThis->clearAndFillCache();
          uima::lowlevel::IndexRepository & nonConstIR = CONST_CAST( IndexRepository &, iv_crIndexRepository);
          nonConstIR.clearDirtyFlagForIndex( this);
        }
        return new STLSingleIndexIterator<T>(iv_cache, iv_comparator, iv_crFSHeap );
      }


      // an iterator of type sets built on top of another IndexIterator
      class TypeSetIterator : public IndexIterator {
      private:
        set<uima::lowlevel::TyFSType> iv_types;
        IndexIterator * iv_it;
        void goForward() {
          if (! iv_it->isValid()) {
            return;
          }
          // (*iv_it->get()) is the type!!
          while ( (iv_it->isValid()) && ( iv_types.end() == iv_types.find( (TyFSType) (iv_it->getTyFSType()))) ) {
            iv_it->moveToNext();
          }
        }

        void goBackward() {
          if (! iv_it->isValid()) {
            return;
          }
          // (*iv_it->get()) is the type!!
          while ( (iv_it->isValid()) && ( iv_types.end() == iv_types.find( (TyFSType) (iv_it->getTyFSType()))) ) {
            iv_it->moveToPrevious();
          }
        }

        TypeSetIterator(TypeSetIterator const & other)
            : iv_it( other.iv_it->clone() ),
            iv_types( other.iv_types) {}

        TypeSetIterator & operator=(TypeSetIterator const & other);
      public:
        TypeSetIterator(IndexIterator * it, set<uima::lowlevel::TyFSType> const & crTypes)
              : iv_it(it),
              iv_types(crTypes) {
        }

        ~TypeSetIterator() {
          delete iv_it;
        }

        void moveToFirst() {
          iv_it->moveToFirst();
          goForward();
        }

        void moveToLast() {
          iv_it->moveToLast();
          goBackward();
        }

        TyFS get() const {
            return iv_it->get();
          }

        TyFS getTyFSType() const {
          return iv_it->getTyFSType();
        }

        bool isValid() const {
          return iv_it->isValid();
        }

        void moveToNext() {
          iv_it->moveToNext();
          goForward();
        }

        void moveToPrevious() {
          iv_it->moveToPrevious();
          goBackward();
        }

        IndexIterator * clone() const {
          return new TypeSetIterator(*this);
        }

        bool moveTo(TyFS fs) {
          bool b = iv_it->moveTo(fs);
          if (!b) {
            return false;
          }
          goForward();
          return iv_it->isValid();
        }

      };

      template <class T>
      IndexIterator* CachedCompositeIndex<T>::createTypeSetIterator(set<uima::lowlevel::TyFSType> const & crTypes) const {
        if (iv_crIndexRepository.isDirtyForIndex( this )) {
          CachedCompositeIndex<T> * nonConstThis = CONST_CAST( CachedCompositeIndex<T> *, this);
          nonConstThis->clearAndFillCache();
          uima::lowlevel::IndexRepository & nonConstIR = CONST_CAST( IndexRepository &, iv_crIndexRepository);
          nonConstIR.clearDirtyFlagForIndex( this );
        }
        return new TypeSetIterator( new STLSingleIndexIterator<T>(iv_cache, iv_comparator, iv_crFSHeap), crTypes);
      }



      ////////////////////////////////////////////////////////////////////////


      /**
       * Use this function in connection with iterators over ordered indexes
       * of a non-leaf type.
       * The problem is as follows: two annotations over the same begin and end position
       * and the same type are considered
       * equal by the comparator. It might
       * then happen that the order of a forward and backward iteration is
       * equal except the order of annotations over different types with
       * the same span. To resolve this, this function only returns 0 if
       * the two compared FSs are the same. Otherwise, the address (heap cell number)
       * of the FSs is used as a comparison criterion.
       */
      int compareWithoutEquality(IndexComparator const * cpComparator,
                                 uima::lowlevel::FSHeap const & heap,
                                 TyFS fs1,
                                 TyFS fs2) {
        int iComp = cpComparator->compare(heap, fs1, fs2 );
        if ( (iComp == 0) && (fs1 != fs2) ) {
          if (fs1 < fs2) {
            return 1;
          }
          if (fs1 > fs2) {
            return -1;
          }
          assert( fs1 == fs2 );
        }
        return iComp;
      }

      // A model of the STL compare to compare two iterators.
      // Two iterators are compared as the FSs they point to.
      class IndexIteratorCompare {
      private:
        IndexComparator const * iv_cpComparator;
        uima::lowlevel::FSHeap const & iv_heap;
      public:
        IndexIteratorCompare( IndexComparator const * cpComparator,
                              uima::lowlevel::FSHeap const & heap) :
            iv_cpComparator(cpComparator),
            iv_heap(heap) {
          assert( EXISTS(iv_cpComparator) );
        }

        bool operator()(IndexIterator const * const & crcpIt1, IndexIterator const * const & crcpIt2) const {
          bool bVal1 = crcpIt1->isValid();
          bool bVal2 = crcpIt2->isValid();

          if (bVal1 && !bVal2) {
            return true;
          }

          if (!bVal1 && bVal2) {
            return false;
          }

          if ( bVal1 && bVal2 ) {
            return compareWithoutEquality(iv_cpComparator, iv_heap, crcpIt1->get(), crcpIt2->get()) > 0;
          }
          assert (!bVal1 && !bVal2);
          return false;
        }
      };


      OrderedCompositeIndex::OrderedCompositeIndex(IndexRepository const & crIndexRepository,
          uima::lowlevel::TyFSType tyType,
          IndexComparator const * aComparator)
          : CachedCompositeIndex<vector<TyFS> >(crIndexRepository, tyType, aComparator) {}

      /* taph 27.08.2003: this can not be inline otherwise xlc6 will
         complain about undefined class FSIndexCompare */
      OrderedCompositeIndex::~OrderedCompositeIndex() {}


      // a simple bubble sort of iterators. Sorts the first argument.
      // Since those vectors get too long (approx. 2-15), bubble sort is quicker
      // than a quicksort or the like.
      void bubbleSortIterators(vector<IndexIterator*> & iterators,
                               IndexComparator const * comparator,
                               uima::lowlevel::FSHeap const & heap) {
        IndexIteratorCompare compare(comparator, heap);
        size_t i;

        size_t n = iterators.size();
        if (n==0 || n==1) {
          return;
        }
        bool bWasSwapped = true;
        while (bWasSwapped) {
          bWasSwapped = false;

          for (i=1; i<n; ++i) {
            uima::lowlevel::IndexIterator * previous = iterators[i-1];
            uima::lowlevel::IndexIterator * current = iterators[i];
            if (compare(current, previous)) {
              bWasSwapped = true;
              iterators[i-1] = current;
              iterators[i] = previous;
            }
          }
        }
      }


      void printIterators(char const * c,ostream & os, vector<IndexIterator*> const & iterators) {
        os << c << endl;
        size_t i;
        for (i=0; i<iterators.size(); ++i) {
          os << " IT: " << i << ": ";
          if (iterators[i]->isValid()) {
            os << (int) iterators[i]->get() << ", type: " << (int) (iterators[i]->getTyFSType()) << endl;
          } else {
            os << "INVALID";
          }
          os << endl;
        }
      }

      void printCache(ostream & os,vector<TyFS> const & cache) {
        size_t i;
        for (i=0; i<cache.size(); ++i) {
//!TODO Problem here               os << i << ": " << (int) *(cache[i]) << endl;
          os << i << ": " << (int) (cache[i]) << endl;
        }
      }


      void OrderedCompositeIndex::clearAndFillCache() {
        vector<IndexIterator*> iterators;

        // initialize iterators
        assert( iv_tyComponents.size() > 0 );
        size_t uiSize = 0;
        TyComponents::const_iterator cit;
        for (cit = iv_tyComponents.begin(); cit != iv_tyComponents.end(); ++cit) {
          assert( EXISTS( *cit ) );
          uiSize += (*cit)->getSize();
          uima::lowlevel::IndexIterator * pit = (*cit)->createIterator();
          assert( EXISTS(pit) );
          pit->moveToFirst();
          if (pit->isValid()) {
            iterators.push_back(pit);
          } else {
            delete pit;
          }
        }
        // now all iterators are valid
        iv_cache.resize( uiSize );

        size_t i=0;

        if ( iterators.size() == 0) {
          return;
        }
        if (iterators.size() == 1) {
          while (iterators[0]->isValid()) {
            iv_cache[i] = iterators[0]->get();
            ++i;
            iterators[0]->moveToNext();
          }
          assert(i == uiSize);
        } else {
          // fill the cache
          bubbleSortIterators(iterators, iv_comparator, iv_crFSHeap);
          while (i<uiSize) {
            assert(iterators[0]->isValid());
            if (iterators[1]->isValid()) {
              while (iterators[0]->isValid() && (compareWithoutEquality(iv_comparator, iv_crFSHeap, iterators[0]->get(), iterators[1]->get()) >0 ) ) {
                iv_cache[i] = iterators[0]->get();
                ++i;
                iterators[0]->moveToNext();
              }
              bubbleSortIterators(iterators, iv_comparator, iv_crFSHeap);
            } else {
              while (iterators[0]->isValid()) {
                iv_cache[i] = iterators[0]->get();
                ++i;
                iterators[0]->moveToNext();
              }
              assert(i == uiSize);
            }
          }
        }

        // delete all iterators
        for (i=0; i<iterators.size(); ++i) {
          assert(EXISTS(iterators[i]));
          delete iterators[i];
        }
      }


///////////////////////////////////////////////////////////////////

      SetCompositeIndex::SetCompositeIndex(IndexRepository const & crIndexRepository,
                                           uima::lowlevel::TyFSType tyType,
                                           IndexComparator const * aComparator)
          : CachedCompositeIndex<set<TyFS> >(crIndexRepository, tyType, aComparator) {}



///////////////////////////////////////////////////////////////////

      SingleIndex::SingleIndex(IndexRepository const & crIndexRepository,
                               TyFSType tyType)
          : IndexABase(crIndexRepository, tyType) {}

      // generic implementation here, may be optimized for subclasses
      bool SingleIndex::contains(TyFS tyFS) const {
        auto_ptr<IndexIterator> it( createIterator() );
        for (it->moveToFirst(); it->isValid(); it->moveToNext()) {
          if ( it->get() == tyFS ) {
            return true;
          }
        }
        return false;
      }


////////////////////////////////////////////////////////////////

      ComparatorSingleIndex::ComparatorSingleIndex(IndexRepository const & crIndexRepository,
          uima::lowlevel::TyFSType tyType,
          IndexComparator const * cpComparator)
          : SingleIndex(crIndexRepository, tyType),
          iv_cpComparator(cpComparator) {
        assert( EXISTS(cpComparator) );
      }

      TyFS ComparatorSingleIndex::find(TyFS fs) const {
        assert( EXISTS(iv_cpComparator) );
        auto_ptr<IndexIterator> it(createIterator());
        assert( EXISTS(it.get()) );
        TyFS result = 0;
        for (it->moveToFirst(); it->isValid(); it->moveToNext() ) {
          TyFS nextFS = it->get();
          if (iv_cpComparator->compare(iv_crFSHeap, fs, nextFS) == 0) {
            result = nextFS;
            break;
          }
        }
        return result;
      }

/////////////////////////////////////////////////////////////////////
      /* taph 25.04.2003: Number of elements to pre-allocated in each
         ordered single index.
         This is just a performance test to reduce allocation times.
         We need to determine if we really need this and
         what a good default value would be.
         We could even make this configurable.
         (maybe even per index because we most likely have more tokens
         than paragraphs).
       */
      static size_t const uiORDERED_SINGLE_INDEX_RESERVE_SIZE = 10;

      OrderedSingleIndex::OrderedSingleIndex(IndexRepository const & crIndexRepository,
                                             TyFSType aType,
                                             IndexComparator const * aComparator)
          :  ComparatorSingleIndex(crIndexRepository, aType, aComparator) {
        iv_tyStructures.reserve(uiORDERED_SINGLE_INDEX_RESERVE_SIZE);
      }


      IndexIterator* OrderedSingleIndex::createIterator() const {
        return new STLSingleIndexIterator<TyStructures>(iv_tyStructures, iv_cpComparator, iv_crFSHeap);
      }


      void OrderedSingleIndex::add(TyFS fs) {
        if (iv_tyStructures.size() > 0) {
          TyFS lastFS = iv_tyStructures.back();
          UIMA_TPRINT(" Adding fs of type " << iv_crFSHeap.getType(fs) << " in index of type " << iv_tyFSType);
          assert( iv_crFSHeap.getType(fs) == iv_tyFSType );
          int comp = iv_cpComparator->compare(iv_crFSHeap, fs, lastFS);
          if (comp > 0) {
            UIMA_TPRINT(" insert!");
            IndexComparatorLess comp(iv_cpComparator, iv_crFSHeap);
            TyStructures::iterator it = lower_bound( iv_tyStructures.begin(), iv_tyStructures.end(), fs, comp);
            assert( it != iv_tyStructures.end() );
            iv_tyStructures.insert(it, fs);
            return;
          }
        }
        // fs is pushed back if
        //  - structures is empty
        //  - fs can be inserted at the end
        iv_tyStructures.push_back(fs);
      }

      void OrderedSingleIndex::remove(TyFS fs) {
        IndexComparatorLess comp(iv_cpComparator, iv_crFSHeap);
        TyStructures::iterator it = lower_bound( iv_tyStructures.begin(), iv_tyStructures.end(), fs, comp);
        if ( it != iv_tyStructures.end() ) {
          iv_tyStructures.erase(it);
        }
      }

      TyFS OrderedSingleIndex::find(TyFS fs) const {
        TyStructures::const_iterator it;
        TyFS result = 0;
        for (it = iv_tyStructures.begin(); it != iv_tyStructures.end(); ++it) {
          TyFS nextFS = *it;
          if (iv_cpComparator->compare(iv_crFSHeap, fs, nextFS) == 0) {
            result = nextFS;
            break;
          }
        }
        return result;
      }

      void OrderedSingleIndex::reset() {
        iv_tyStructures.clear();
        iv_tyStructures.reserve(uiORDERED_SINGLE_INDEX_RESERVE_SIZE);
      }

#ifndef NDEBUG
#ifndef ASSERT_OR_RETURN_FALSE
#define ASSERT_OR_RETURN_FALSE(x) assert(x)
#endif
      bool OrderedSingleIndex::debugIsConsistent() const {
        static int debugCheck = 0;

        ++debugCheck;
        if ( (debugCheck % 1) != 0) {
          return true;
        }
        size_t i;
        for (i=0; i< (iv_tyStructures.size() - 1); ++i) {
          ASSERT_OR_RETURN_FALSE( iv_cpComparator->compare(iv_crFSHeap, iv_tyStructures[i], iv_tyStructures[i+1]) >= 0 );
        }
        if (iv_tyStructures.size() >=2) {
          i = iv_tyStructures.size() - 2;
          ASSERT_OR_RETURN_FALSE( iv_cpComparator->compare(iv_crFSHeap, iv_tyStructures[i], iv_tyStructures[i+1]) >= 0 );
        }
        return true;
      }
#endif


///////////////////////////////////////////////////////////////////////////

      SetSingleIndex::SetSingleIndex(IndexRepository const & crIndexRepository,
                                     TyFSType aType,
                                     IndexComparator const * aComparator)
          :
          ComparatorSingleIndex(crIndexRepository, aType, aComparator) {
        iv_pStructures = new TyStructures(
                           IndexComparatorLess(iv_cpComparator, iv_crFSHeap) );
        assert(EXISTS(iv_pStructures) );
      }

      SetSingleIndex::~SetSingleIndex() {
        assert(EXISTS(iv_pStructures) );
        delete iv_pStructures;
      }

      IndexIterator* SetSingleIndex::createIterator() const {
        return new STLSingleIndexIterator<TyStructures>(*iv_pStructures, iv_cpComparator, iv_crFSHeap);
      }

      void SetSingleIndex::reset() {
        iv_pStructures->clear();
        assert(EXISTS(iv_pStructures) );


      }

///////////////////////////////////////////////////////////////////////

      IndexIterator* FIFOSingleIndex::createIterator() const {
        return new STLSingleIndexIterator<TyStructures>(iv_tyFIFO, NULL, iv_crFSHeap);
      }

      /* taph 27.08.2003: this can not be inline otherwise xlc6 will
         complain about undefined class FSIndexCompare */
      FIFOCompositeIndex::FIFOCompositeIndex(
        IndexRepository const & crIndexRepository,
        uima::lowlevel::TyFSType tyType):
          CachedCompositeIndex<vector<TyFS> >(crIndexRepository, tyType, NULL) {}

    }
  }
}
/* ----------------------------------------------------------------------- */



