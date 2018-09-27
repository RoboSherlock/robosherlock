#ifndef UIMA_LOWLEVEL_INTERNAL_INDEXES_HPP
#define UIMA_LOWLEVEL_INTERNAL_INDEXES_HPP
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
#include <algorithm>
#include <memory>
#include <set>

#include <uima/lowlevel_index.hpp>
#include <uima/lowlevel_indexcomparator.hpp>
#include <uima/lowlevel_indexiterator.hpp>
#include <uima/types.h>
#include <uima/macros.h>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  namespace lowlevel {
    class IndexIterator;
    class SingleOrderedIndexIterator;
    class IndexRepository;
    class KeyFeatureLexicographicLess;
    class SingleSetIndex;
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
    namespace internal {

      /**
       * The base class for all single indexes.
       */
      class UIMA_LINK_IMPORTSPEC SingleIndex : public IndexABase {
      protected:
        SingleIndex(IndexRepository const & aIndexRepository,
                    TyFSType tyType);
      public:
        /**
         * check if a feature structure is contained in the index.
         * This is different from find() since it always tests for
         * identity rather equivalence.
         */
        virtual bool contains(TyFS) const;

        virtual IndexIterator* createTypeSetIterator(std::set<uima::lowlevel::TyFSType> const & crType) const {
          assert( crType.size() <= 1 );
          if (crType.size() == 1) {
            assert( *(crType.begin()) == getType() );
          }
          return createIterator();
        }
      };


      /**
       * A single index with a comparator.
       */
      class UIMA_LINK_IMPORTSPEC ComparatorSingleIndex : public SingleIndex {
      protected:
        IndexComparator const * iv_cpComparator;

        ComparatorSingleIndex(IndexRepository const & crIndexRepository,
                              uima::lowlevel::TyFSType tyType,
                              IndexComparator const * cpComparator);
      public:
        virtual TyFS find(TyFS) const;
        IndexComparator const * getComparator() const {
          return iv_cpComparator;
        }
      };


      /**
       * A single ordered index. Inserting works fast if the elements
       * are inserted already in the correct order. Slow otherwise...
       */
      class UIMA_LINK_IMPORTSPEC OrderedSingleIndex : public ComparatorSingleIndex {
      public:
        typedef std::vector<TyFS> TyStructures;
      private:
        TyStructures iv_tyStructures;
      protected:
        void add(TyFS fs);
        void remove(TyFS fs);
        void reset();
      public:
        OrderedSingleIndex(IndexRepository const & aIndexRepository,
                           TyFSType aType,
                           IndexComparator const * );

        size_t getSize() const {
          return iv_tyStructures.size();
        }

        IndexIterator* createIterator() const;
        TyFS find(TyFS fs) const;

	std::vector<TyFS> const * getVector() const {
          return & iv_tyStructures;
        }
#ifndef NDEBUG
        bool debugIsConsistent() const;
#endif
      };


      /**
       * This class is a simply adapter to make a binary function out of a
       * comparator.
       * @see SetSingleIndex
       */
      class UIMA_LINK_IMPORTSPEC IndexComparatorLess : public std::binary_function<TyFS, TyFS, bool> {
      private:
        IndexComparator const * iv_cpclComparator;
        FSHeap const * iv_heap;

        IndexComparatorLess()
            : iv_cpclComparator(NULL),
            iv_heap(NULL) {
          assertWithMsg(false, "Default constructor for STL compliance only!");
        };
      public:
        IndexComparatorLess(IndexComparator const * aComparator,
                            FSHeap const & heap)
            : iv_cpclComparator(aComparator),
            iv_heap(&heap) {
          assert( EXISTS(iv_cpclComparator) );
        }

        bool operator()(TyFS fs1, TyFS fs2) const {
          assert( EXISTS(iv_cpclComparator) );
          assert( EXISTS(iv_heap) );
          UIMA_TPRINT("Comparing fs " << (long) fs1 << " with " << (long) fs2 << ". result: " << iv_cpclComparator->compare(*iv_heap, fs1, fs2) );
          return iv_cpclComparator->compare(*iv_heap, fs1, fs2) > 0;
        }
      };


      /**
       * A single set index.
       */
      class UIMA_LINK_IMPORTSPEC SetSingleIndex : public ComparatorSingleIndex {
      public:
        typedef std::set<TyFS, IndexComparatorLess> TyStructures;
      private:
        // the set is allocated on the heap due to template problems with the Solaris compiler
        TyStructures *          iv_pStructures;

      protected:
        void add(TyFS fs) {
          assert( EXISTS(iv_pStructures) );
          (void) iv_pStructures->insert(fs);
        }
        void remove(TyFS fs) {
          assert( EXISTS(iv_pStructures) );
          (void) iv_pStructures->erase(fs);
        }

        void reset();
      public:
        SetSingleIndex(IndexRepository const & aIndexRepository,
                       TyFSType aType,
                       IndexComparator const * aComparator);
        ~SetSingleIndex();

        size_t getSize() const {
          assert( EXISTS(iv_pStructures) );
          return iv_pStructures->size();
        }

        IndexIterator* createIterator() const;

        TyFS find(TyFS fs) const {
          assert( EXISTS(iv_pStructures) );
          TyStructures::const_iterator it = iv_pStructures->find(fs);
          if (it == iv_pStructures->end()) {
            return 0;
          }
          return *it;
        }

      };


      /**
       * A single index behaving like a FIFO (a vector).
       * A comparator is not needed here.
       */
      class UIMA_LINK_IMPORTSPEC FIFOSingleIndex : public SingleIndex {
      public:
        typedef std::vector<TyFS> TyStructures;
      private:
        TyStructures iv_tyFIFO;
      protected:
        void add(TyFS fs) {
          iv_tyFIFO.push_back(fs);
        }
        void reset() {
          iv_tyFIFO.clear();
        }

        void remove(TyFS tyFS) {
          TyStructures::iterator it = std::find(iv_tyFIFO.begin(), iv_tyFIFO.end(), tyFS);
          if (it == iv_tyFIFO.end() ) {
            return;
          }
          iv_tyFIFO.erase(it);
        }
      public:
        FIFOSingleIndex(IndexRepository const & aIndexRepository,
                        TyFSType aType)
            : SingleIndex(aIndexRepository, aType) {}

        size_t getSize() const {
          return iv_tyFIFO.size();
        }

        TyFS find(TyFS fs) const {
          TyStructures::const_iterator it = std::find(iv_tyFIFO.begin(), iv_tyFIFO.end(), fs);
          if (it == iv_tyFIFO.end()) {
            return 0;
          }
          return *it;
        }

        IndexIterator* createIterator() const;

	std::vector<TyFS> const * getVector() const {
          return & iv_tyFIFO;
        }

      };



      ////////////////////////////////////////////////////////////////////////
      // Composite indexes

      /**
       * The base class for all composite indexes. All methods are implemented
       * but may be overwritten by subclasses.
       */
      class UIMA_LINK_IMPORTSPEC CompositeIndex : public IndexABase {
      public:
        typedef std::vector<SingleIndex*> TyComponents;
      protected:
        TyComponents iv_tyComponents;
        void add(TyFS /*fs*/) {
          assertWithMsg(false, "Add operation not supported on CompositeIndex!");
        }

        void remove(TyFS /*fs*/) {
          assertWithMsg(false, "Remove operation not supported on CompositeIndex!");
        }

        void reset() {
          assertWithMsg(false, "Reset operation not supported on CompositeIndex!");
        }

        CompositeIndex(IndexRepository const & aIndexRepository,
                       uima::lowlevel::TyFSType tyType);

        /**
         * get all the components for the respective types.
         */
        void getComponentsForTypes(std::set<uima::lowlevel::TyFSType> const & crTypes, TyComponents & crResult) const;
      public:
        void addComponent(SingleIndex* pclIndex) {
          assert( iv_crTypeSystem.subsumes( getType(), pclIndex->getType() ) );
          iv_tyComponents.push_back(pclIndex);
        }
        virtual size_t getSize() const;
        virtual TyFS find(TyFS fs) const;

      };


      /**
       * generic implementation of a composite index with a cache of all feature structures.
       * This cache maybe used for fast iterating over all feature structures.
       * The cache type T can either be an STL vector or set.
       */
      template <class T>
      class CachedCompositeIndex : public CompositeIndex {
      protected:
        T iv_cache;
        uima::lowlevel::IndexComparator const * iv_comparator;

        CachedCompositeIndex(IndexRepository const & aIndexRepository,
                             uima::lowlevel::TyFSType tyType,
                             IndexComparator const *);

        // generic implementation of clearAndFillCache, maybe overwritten by subclasses
        // this method is here because if it is in the cpp file, we get linking problems on Solaris.
        virtual void clearAndFillCache() {
          iv_cache.clear();
          TyComponents::const_iterator cit;
          for (cit = iv_tyComponents.begin(); cit != iv_tyComponents.end(); ++cit) {
            SingleIndex const * index = (*cit);
	    std::auto_ptr<IndexIterator> apit( index->createIterator() );
            for (apit->moveToFirst(); apit->isValid(); apit->moveToNext()) {
              iv_cache.insert(iv_cache.end(), apit->get() );
            }
          }
        }
      public:
        IndexIterator* createIterator() const;
        IndexIterator* createTypeSetIterator(std::set<uima::lowlevel::TyFSType> const & crTypes) const;

        IndexComparator const * getComparator() const {
          return iv_comparator;
        }
      };



      /**
       * A composite ordered index. The iterator is different than the one
       * of the superclass in that a different comparison scheme is used.
       */
      class OrderedCompositeIndex : public CachedCompositeIndex<std::vector<TyFS> > {
      protected:
        void clearAndFillCache();
      public:

        OrderedCompositeIndex(IndexRepository const & aIndexRepository,
                              uima::lowlevel::TyFSType tyType,
                              IndexComparator const * aComparator);
        virtual ~OrderedCompositeIndex();

      };


      /**
       * A composite set index.
       */
      class SetCompositeIndex : public CachedCompositeIndex<std::set<TyFS> > {
      protected:
      public:
        SetCompositeIndex(IndexRepository const & aIndexRepository,
                          uima::lowlevel::TyFSType tyType,
                          IndexComparator const * aComparator);
      };


      class FIFOCompositeIndex : public CachedCompositeIndex<std::vector<TyFS> > {
      protected:
      public:
        FIFOCompositeIndex(IndexRepository const & crIndexRepository,
                           uima::lowlevel::TyFSType tyType);
      };


    } //namespace internal
  } //namespace lowlevel
} //namespace uima

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace lowlevel {
    namespace internal {

      template<class T>
      CachedCompositeIndex<T>::CachedCompositeIndex(IndexRepository const & aIndexRepository,
          uima::lowlevel::TyFSType tyType,
          IndexComparator const * comparator)
          : CompositeIndex( aIndexRepository, tyType),
          iv_comparator(comparator) {
      }


    } //namespace internal
  } //namespace lowlevel
} //namespace uima
/* ----------------------------------------------------------------------- */


#endif

