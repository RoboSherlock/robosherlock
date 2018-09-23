#ifndef UIMA_FSFILTERBUILDER_HPP
#define UIMA_FSFILTERBUILDER_HPP
/** \file fsfilterbuilder.hpp
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

    \brief Contains class FSFilterBuilder

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp>

#include <vector>
#include <uima/typesystem.hpp>

#include "unicode/utf.h"

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  class CAS;
  class FeatureStructure;
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */




namespace uima {


  /**
   * The base class of all feature structure filters.
   * Pass this object to Index::createFilteredIterator().
   * @see FSIndex
   */
  class UIMA_LINK_IMPORTSPEC FSFilter {
  public:
    virtual ~FSFilter() { }

    /**
     * override this function to explictly free resources.
     * default does nothing.
     * @see uima::FSFilterBuilder
     */
    virtual void deInit() { };

    /**
     * Implement this method in your subclass.
     * @return true if <code>crFS</code> should be filtered (i.e. not returned
     * via an iterator).
     */
    virtual bool isFiltered(FeatureStructure const & crFS) const = 0;
  };

  /**
   * This class enables creation of filters over feature structures, which can
   * be used for creating filtered iterators. You can access an FSFilterBuilder
   * only through the getFilterBuilder() method on the CAS, you cannot create
   * such an object yourself.
   *
   * Memory ownership of the created FSFilter objects
   * stays with the caller of the builder functions.
   * If you use a "compound" filter (like AND, OR, or NOT filter), you
   * can delete all the filters of the compound object by explictly
   * calling deInit(). See the following sample code:
   *   FSFilterBuilder const & builder = cas.getFilterBuilder();
   *   FSFilter * f1 = builder.createTypeFilter(...);
   *   FSFilter * f2 = builder.createIntFeatureFilter(...);
   *   FSFilter * f = cas.getFilterBuilder().createANDFilter(f1, f2);
   *   f->deInit();    // f1 and f2 are deleted in this call
   *   delete f;       // only delete f (not the embedded ones)
   *
   * @see FSFilter
   * @see CAS
   */
  class UIMA_LINK_IMPORTSPEC FSFilterBuilder {
    friend class uima::CAS;
  protected:
    FSFilterBuilder();
    ~FSFilterBuilder();
  public:
    /**
     * Comparison operators for ints and floats.
     */
    typedef enum {
      EQUALS,
      GREATER,
      LESS
    } EnComparisonOperator;

    /**
     * create a conjunction.
     * The isFiltered() method of the result returns true iff pFilter1->isFiltered()
     * AND pFilter2->isFiltered() return true.
     */
    FSFilter * createANDFilter(FSFilter * pFilter1, FSFilter * pFilter2) const;
    /**
     * create a disjunction.
     * The isFiltered() method of the result returns true iff pFilter1->isFiltered()
     * OR pFilter2->isFiltered() return true.
     */
    FSFilter * createORFilter(FSFilter * pFilter1, FSFilter * pFilter2) const;
    /**
     * create a negation.
     */
    FSFilter * createNOTFilter(FSFilter * pFilter) const;
    /**
     * create a filter which filters all feature structures where the structure
     * under the feature path <code>crFeaturePath</code> satisfies <code>pFilter</code>,
     */
    FSFilter * createMatchFilter(std::vector<Feature> const & crFeaturePath, FSFilter * pFilter) const;
    /**
     * create a filter which filters a feature structure if the int value found
     * under the feature path <code>crFeaturePath</code> is equal, greater, or less than <code>iVal</code>.
     */
    FSFilter * createIntFeatureFilter(std::vector<Feature> const & crFeaturePath, EnComparisonOperator enOp, int iVal) const;
    /**
     * create a filter which filters a feature structure if the float value found
     * under the feature path <code>crFeaturePath</code> is equal, greater, or less than <code>fVal</code>.
     */
    FSFilter * createFloatFeatureFilter(std::vector<Feature> const & crFeaturePath, EnComparisonOperator enOp, float fVal) const;

    /**
     * create a filter which filters a feature structure if the string value found
     * under the feature path <code>crFeaturePath</code> is equal, greater, or less than
     * the string stored under <code>cpUTFBuffer</code> with length <code>uiLength</code>.
     * The string is copied so the buffer may be invalidated after this call.
     */
    FSFilter * createStringFeatureFilter(std::vector<Feature> const & crFeaturePath, UChar const * cpUTFBuffer, size_t uiLength) const;

    /**
     * create a filter which filters all feature structures whose type
     * equals <code>crType</code> (if <code>bSubsumptionOn</code> is true) or
     * subsumes <code>crType</code> (if <code>bSubsumptionOn</code> is false).
     */
    FSFilter * createTypeFilter(Type const & crType, bool bSubsumptionOn) const;
  };

}


/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif

