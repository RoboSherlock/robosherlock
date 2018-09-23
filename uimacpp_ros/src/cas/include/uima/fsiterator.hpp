#ifndef UIMA_FSITERATOR_HPP
#define UIMA_FSITERATOR_HPP
/** \file fsiterator.hpp .
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

    \brief Contains class FSIterator

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <uima/featurestructure.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
  class CAS;
  namespace lowlevel {
    class IndexIterator;
    class FSHeap;
  }
  namespace internal {
    class FSPromoter;
  }
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {

  /**
   * Iterator over FeatureStructure objects in a CAS Index.
   * Iterators are created by calling FSIndex.iterator()
   * @see FSIndex
   */
  class UIMA_LINK_IMPORTSPEC FSIterator {
    friend class FSIndex;
    friend class uima::internal::FSPromoter;
    friend class uima::CAS;
  protected:
    uima::lowlevel::IndexIterator* iv_pIterator;
    CAS * iv_cas;

    bool operator==(FSIterator const &) const;

    FSIterator(uima::lowlevel::IndexIterator*, CAS * cas);
  public:
    /// true iff get() can be called sucessfully
    bool isValid() const;
    /** retrieve the current element in the index for this iterator.
     */
    FeatureStructure get() const;
    /** make iterator point to the next element in the index for this iterator.
     * isValid() is false after the call if no such element exists.
     */
    void moveToNext();
    /** make iterator point to the previous element in the index for this iterator.
     * isValid() is false after the call if no such element exists.
     */
    void moveToPrevious();
    /** make iterator point to the first element in the index for this iterator.
     */
    void moveToFirst();
    /** make iterator point to the last element in the index for this iterator.
     */
    void moveToLast();
    /** make iterator point to the first feature structure fs2 in the index
     * for this iterator where fs >= fs2.
     * fs must be of a type compatible with the index for this iterator.
     */
    void moveTo(FeatureStructure fs);

    /**
     * returns the next feature structure of the iterator without
     * moving it. If there is no such feature structure
     * (i.e. the iterator points to the first element) an invalid FS is returned.
     */
    FeatureStructure peekNext() const;

    /**
     * returns the previous feature structure of the iterator without
     * moving it. If there is no such feature structure
     * (i.e. the iterator points to the last element) an invalid FS is returned.
     */
    FeatureStructure peekPrevious() const;

    /// Default CTOR
    FSIterator();
    /// copy CTOR
    FSIterator(FSIterator const &);
    /// assignment operator
    FSIterator & operator=(FSIterator const &);
    ~FSIterator();
  }
  ; // class FSIterator

} // namespace uima

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif /* UIMA_FSITERATOR_HPP */

