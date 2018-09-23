/** \file result_specification.hpp .
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

    \brief  Contains uima::ResultSpecification

   Description:

-----------------------------------------------------------------------------


   02/05/2003  Initial creation

-------------------------------------------------------------------------- */

#ifndef UIMA_RESULT_SPECIFICATION_HPP
#define UIMA_RESULT_SPECIFICATION_HPP

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <set>
#include "unicode/unistr.h"
#include <uima/unistrref.hpp>

#include <uima/typesystem.hpp>
#include <uima/internal_fspromoter.hpp>
#include <uima/type_or_feature.hpp>

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


  /**
   * An engine/annotator gets a set like this passed in its process() function
   * and is required to check this set to see which kind of output types and
   * features it is supposed to create.
   * An annotator which produces only one kind of output can skip this check since
   * is made sure that the set only contains types/features which
   * the annotator is capable of producing and would never call an annotator that is
   * not required at all.
   * So this set is most relevant for multi-purpose annotators to determine which
   * of their various functions are required for this specific document.
   */

  class UIMA_LINK_IMPORTSPEC ResultSpecification {
  public:
    typedef std::set< TypeOrFeature > TyTypeOrFeatureSTLSet;
  private:
    TyTypeOrFeatureSTLSet iv_targetSet;

    /// check if a type or feature name is contained in this set
    bool isSubsumedBySomeElement(TypeOrFeature const & crElem) const {
      std::set<TypeOrFeature>::const_iterator it;
      for (it = iv_targetSet.begin(); it != iv_targetSet.end(); ++it) {
        if ( (*it).subsumes(crElem) ) {
          return true;
        }
      }
      return false;
    }

  public:

    /**
     * An annotator can call this method to determine whether it should
     * produce the specified TypeOrFeature.
     */
    bool shouldBeCreatedByAnnotator(TypeOrFeature const & crTypeOrFeature) const {
      assert( crTypeOrFeature.isValid() );
      return contains(crTypeOrFeature);
    }

    TyTypeOrFeatureSTLSet const & getTypeOrFeatureSTLSet() const {
      return iv_targetSet;
    }

    void clear() {
      iv_targetSet.clear();
    }

    size_t getSize() const {
      return iv_targetSet.size();
    }

    void add(TypeOrFeature const & crTypeOrFeature) {
      assert( crTypeOrFeature.isValid() );
      iv_targetSet.insert(crTypeOrFeature);
    }

    void remove(TypeOrFeature const & crTypeOrFeature) {
      assert( crTypeOrFeature.isValid() );
      iv_targetSet.erase(crTypeOrFeature);
    }

    bool contains(TypeOrFeature const & crTypeOrFeature) const {
      assert( crTypeOrFeature.isValid() );
      return iv_targetSet.find(crTypeOrFeature) != iv_targetSet.end();
    }


    void print(std::ostream & os) const {
      os << "ResultSpecification: " << std::endl;
      ResultSpecification::TyTypeOrFeatureSTLSet const & crTOFSet = getTypeOrFeatureSTLSet();
      ResultSpecification::TyTypeOrFeatureSTLSet::const_iterator cit;
      for (cit = crTOFSet.begin(); cit != crTOFSet.end(); ++cit) {
        TypeOrFeature const & crTOF = (*cit);
//            os << "Check TOF: " << i++ << endl;
        assert( (*cit).isValid() );
        assert( crTOF.isValid() );
        assert( contains( crTOF ) );

        os << "  TOF Name: " << crTOF.getName() << std::endl;
      }

    }

  }
  ; /* class ResultSpecification */



} // namespace uima

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


#endif /* UIMA_RESULT_SPECIFICATION_HPP */

/* <EOF> */

