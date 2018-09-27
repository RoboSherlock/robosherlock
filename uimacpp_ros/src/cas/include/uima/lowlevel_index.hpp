#ifndef UIMA_LOWLEVEL_INDEX_HPP
#define UIMA_LOWLEVEL_INDEX_HPP
/** \file lowlevel_index.hpp .
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
#include <vector>
#include <algorithm>
#include <set>
#include <uima/types.h>
#include <uima/lowlevel_typedefs.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  namespace lowlevel {
    class IndexComparator;
    class TypeSystem;
    class FSHeap;
    class IndexIterator;
    class IndexRepository;
    class FSFilter;
    /*
    namespace internal {
       class CompositeIndex;
    }
    */
  }
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace lowlevel {

    /**
     * The base class for all indexes.
     */
    class UIMA_LINK_IMPORTSPEC IndexABase {
      friend class IndexRepository; // for access to add()
//         friend class internal::CompositeIndex; // for access of reset()
    protected:
      IndexRepository const & iv_crIndexRepository;
      FSHeap const & iv_crFSHeap;
      TypeSystem const & iv_crTypeSystem;
      uima::lowlevel::TyFSType iv_tyFSType;

      IndexABase(IndexRepository const &, uima::lowlevel::TyFSType);

      virtual ~IndexABase() {}

      virtual void add(TyFS fs) = 0;
      virtual void remove(TyFS fs) = 0;
      virtual void reset() = 0;
    public:
      /**
       * get the size of this index, i.e., how many feature structures it contains.
       */
      virtual size_t getSize() const = 0;

      /**
       * Find a feature structure within this index which matches <code>tyFS</code>
       * with respect to the comparator of this index (i.e. where the <code>compare</code>
       * method of <code>IndexComparator</code> returns 0). If no such feature structure
       * is found, <code>INVALID_FS</code> is returned.
       */
      virtual TyFS find(TyFS tyFS) const;

      /**
       * create an iterator over this index.
       */
      virtual IndexIterator* createIterator() const = 0;

      /**
       * create an iterator over this index which contains only structures
       * of the types indicated in <code>crTypes</code>.
       * Precondition: All types must be subsumed by the type of this index
       * (i.e. <code>getType()</code>.
       */
      virtual IndexIterator* createTypeSetIterator(std::set<uima::lowlevel::TyFSType> const & crTypes) const = 0;

      /**
       * create a filtered iterator over this index which filter
       * <code>cpFilter</code>. All feature structures where the
       * <code>isFiltered</code> method of the filter object returns true
       * are skipped by the iterator.
       */
      IndexIterator* createFilteredIterator(FSFilter const * cpFilter) const;

      /**
       * Advanced use only:
       * Returns a const pointer to an internal vector of feature structures.
       * A value != NULL is returned _only_ if the implementing class actually
       * uses a vector internally to store the FSs.
       */
      virtual std::vector<uima::lowlevel::TyFS> const * getVector() const {
        return NULL;
      }

      uima::lowlevel::TyFSType getType() const {
        return iv_tyFSType;
      }

      FSHeap const & getFSHeap() const {
        return iv_crFSHeap;
      }

      IndexRepository const & getIndexRepository() const {
        return iv_crIndexRepository;
      }

    };

  }
}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */


#endif

