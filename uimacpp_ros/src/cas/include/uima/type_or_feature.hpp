/** \file type_or_feature.hpp .
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

#ifndef UIMA_TYPE_OR_FEATURE_HPP
#define UIMA_TYPE_OR_FEATURE_HPP

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <set>
#include "unicode/unistr.h"
#include <uima/unistrref.hpp>

#include <uima/typesystem.hpp>
#include <uima/internal_fspromoter.hpp>

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
   * Wrapper class to encapsulate either a uima::Type <em>or</em> a
   * uima::Feature object since both types and features can be used as
   * parts of a result specification
   *
   * @see uima::ResultSpecification
   */
  class UIMA_LINK_IMPORTSPEC TypeOrFeature {
  private:
    Type    iv_type;
    Feature iv_feature;
  public:
    /// Default constructor
    TypeOrFeature() {
      assert( ! iv_type.isValid() );
      assert( ! iv_feature.isValid() );
    }
    /// Constructor from a uima::Type object
    TypeOrFeature(Type const & crType)
        : iv_type( crType ),
        iv_feature(  ) {
      assert( iv_type.isValid() );
      assert( ! iv_feature.isValid() );
    }

    TypeOrFeature(TypeOrFeature const & crTOF) {
      *this = crTOF;
    }

    TypeOrFeature & operator=(TypeOrFeature const & crTOF) {
      iv_type = crTOF.iv_type;
      iv_feature = crTOF.iv_feature;
      assert( isValid() );
      return *this;
    }

    /// Constructor from a uima::Feature object
    TypeOrFeature(Feature const & crFeature)
        : iv_type(  ),
        iv_feature( crFeature ) {
      assert( ! iv_type.isValid() );
      assert( iv_feature.isValid() );
    }

    bool isType() const {
      return iv_type.isValid();
    }

    bool isValid() const {
      if (isType()) {
        return iv_type.isValid();
      }
      return iv_feature.isValid();
    }

    Type getType() const {
      return iv_type;
    }

    Feature getFeature() const {
      return iv_feature;
    }

    bool subsumes(TypeOrFeature const & crTypeOrFeature) const {
      if (isType()) {
        if (crTypeOrFeature.isType()) {
          return iv_type.subsumes( crTypeOrFeature.iv_type );
        }
        return false;
      }
      if (crTypeOrFeature.isType()) {
        return false;
      }
      return iv_feature == crTypeOrFeature.iv_feature;
    }

    UnicodeStringRef getName() const {
      if (isType()) {
        return iv_type.getName();
      }
      return iv_feature.getName();
    }

    bool operator==(TypeOrFeature const & crOther) const {
      uima::lowlevel::TyFSFeature tyFeature = uima::internal::FSPromoter::demoteFeature(iv_feature);
      uima::lowlevel::TyFSFeature tyOtherFeature = uima::internal::FSPromoter::demoteFeature(crOther.iv_feature);
      if (tyFeature != tyOtherFeature) {
        return false;
      }

      uima::lowlevel::TyFSType tyType = uima::internal::FSPromoter::demoteType(iv_type);
      uima::lowlevel::TyFSType tyOtherType = uima::internal::FSPromoter::demoteType(crOther.iv_type);
      if ( tyType != tyOtherType) {
        return false;
      }

      return true;
    }

    bool operator<(TypeOrFeature const & crOther) const {
      if ( (*this) == crOther ) {
        return false;
      }
      uima::lowlevel::TyFSType tyType = uima::internal::FSPromoter::demoteType(iv_type);
      uima::lowlevel::TyFSType tyOtherType = uima::internal::FSPromoter::demoteType(crOther.iv_type);
      if (tyType == tyOtherType) {
        uima::lowlevel::TyFSFeature tyFeature = uima::internal::FSPromoter::demoteFeature(iv_feature);
        uima::lowlevel::TyFSFeature tyOtherFeature = uima::internal::FSPromoter::demoteFeature(crOther.iv_feature);
        assert( tyFeature != tyOtherFeature );
        return tyFeature < tyOtherFeature;
      }

      return tyType < tyOtherType;
    }


  }
  ; /* class TypeOrFeature */



} // namespace uima

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


#endif /* UIMA_TYPE_OR_FEATURE_HPP */

/* <EOF> */

