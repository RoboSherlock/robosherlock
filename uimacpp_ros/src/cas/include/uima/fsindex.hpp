#ifndef UIMA_FS_INDEX_HPP
#define UIMA_FS_INDEX_HPP
/** \file fsindex.hpp .
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

    \brief Contains classes uima::FSFilter and uima::FSIndex

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <set>

#include <uima/featurestructure.hpp>
#include <uima/fsiterator.hpp>
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
  class FSIndexRepository;
  class FSFilter;
  namespace lowlevel {
    class IndexABase;
    class IndexRepository;
  }
  namespace internal {
    class FSSystem;
  }


}


/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {

  UIMA_EXC_CLASSDECLARE(InvalidIndexObjectException, CASException);
  UIMA_EXC_CLASSDECLARE(WrongFSTypeForIndexException, CASException);

  /**
   * This class represents a single index over feature structures of some type.
   */
  class UIMA_LINK_IMPORTSPEC FSIndex {
    friend class uima::FSIndexRepository;
  protected:
    lowlevel::IndexABase const * iv_pIndex;
    uima::lowlevel::IndexRepository * iv_indexRepository;

    FSIndex(lowlevel::IndexABase const * cpIndex, uima::lowlevel::IndexRepository &);
    FSIndex(lowlevel::IndexABase const * cpIndex, uima::lowlevel::IndexRepository const &);
    void checkValidity() const;
  public:
    /**
     * Default constructor. Creates an invalid FSIndex object.
     */
    FSIndex();

    /**
     * @return true if this index object is valid.
     */
    bool isValid() const;

    /**
     * get the number of feature structures currently in this index.
     * @throws InvalidIndexObjectException
     */
    size_t getSize() const;

    /**
     * find the feature structure in this index which is equivalent to <code>anFS</code>
     * according to the comparator for this index. If such an FS cannot be found
     * an invalid feature structure is returned.
     * @throws InvalidIndexObjectException
     */
    FeatureStructure find(FeatureStructure const & anFS) const;

    /**
     * create an iterator over all the feature structures in this index.
     * @throws InvalidIndexObjectException
     */
    FSIterator iterator() const;

    /**
     * create an iterator over all the feature structures in this index
     * with one of the specified types.
     * @throws InvalidIndexObjectException
     * @throws InvalidFSTypeObjectException
     * @throws IncompatibleTypeForIndexException
     */
    FSIterator typeSetIterator(std::set<uima::Type> const & crTypes) const;

    /**
     * create an iterator over this index with the filter <code>cpFilter</code>,
     * @see FSFilter
     * @throws InvalidIndexObjectException
     */
    FSIterator filteredIterator(FSFilter const * cpFilter) const;
  };


} // namespace uima

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif

