/** \file typesystem.cpp .
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
#include <uima/typesystem.hpp>
#include <uima/lowlevel_typesystem.hpp>
#include <uima/msg.h>
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

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

using namespace std;
namespace uima {
  UIMA_EXC_CLASSIMPLEMENT(InvalidFSTypeObjectException, CASException);
  UIMA_EXC_CLASSIMPLEMENT(InvalidFSFeatureObjectException, CASException);
  UIMA_EXC_CLASSIMPLEMENT(TypeSystemAlreadyCommittedException, CASException);

  void Feature::checkValidity() const {
    if (!isValid()) {
      UIMA_EXC_THROW_NEW(InvalidFSFeatureObjectException,
                         UIMA_ERR_INVALID_FSFEATURE_OBJECT,
                         UIMA_MSG_ID_EXC_INVALID_FSFEATURE_OBJECT,
                         ErrorMessage(UIMA_MSG_ID_EXCON_UNKNOWN_CONTEXT),
                         ErrorInfo::recoverable
                        );
    }
  }

  void Feature::getRangeType(Type& result) const {
    checkValidity();
    lowlevel::TyFSType type = iv_typeSystem->getRangeType(iv_tyFeature);
    Type res = internal::FSPromoter::promoteType(type, *iv_typeSystem);
    result = res;
  }

  Type Feature::getRangeType() const {
    checkValidity();
    lowlevel::TyFSType type = iv_typeSystem->getRangeType(iv_tyFeature);
    return internal::FSPromoter::promoteType(type, *iv_typeSystem);
  }

  Type Feature::getIntroType() const {
    checkValidity();
    lowlevel::TyFSType type = iv_typeSystem->getIntroType(iv_tyFeature);
    return internal::FSPromoter::promoteType(type, *iv_typeSystem);
  }

  Feature::Feature(lowlevel::TyFSFeature aFeature, uima::lowlevel::TypeSystem & typeSystem)
      : iv_tyFeature(aFeature),
      iv_typeSystem( & typeSystem) {}

  Feature::Feature()
      : iv_tyFeature(0),
      iv_typeSystem( NULL ) {}

  bool Feature::isValid() const {
    return(iv_tyFeature != 0) && (EXISTS(iv_typeSystem));
  }

  UnicodeStringRef Feature::getName() const {
    checkValidity();
    return UnicodeStringRef(iv_typeSystem->getFeatureBaseName(iv_tyFeature) );
  }

  UnicodeStringRef Feature::getCreatorID() const {
    checkValidity();
    return UnicodeStringRef( iv_typeSystem->getFeatureCreatorID(iv_tyFeature) );
  }


  bool Feature::operator==(Feature const & crOther) const {
//      checkValidity();
//      crOther.checkValidity();
    return( (iv_tyFeature == crOther.iv_tyFeature) && (iv_typeSystem == crOther.iv_typeSystem) );
  }

  /* taph 07.06.2002: we need an explicit != because of template deduction problems */
  bool Feature::operator!=(Feature const & crOther) const {
    return( !( (*this) == crOther) );
  }

  void Feature::getIntroType(Type & rResult) const {
    checkValidity();
    uima::lowlevel::TyFSType tyIntroType = iv_typeSystem->getIntroType(iv_tyFeature);
    rResult = uima::internal::FSPromoter::promoteType(tyIntroType, *iv_typeSystem);
  }

  uima::TypeSystem const & Feature::getTypeSystem() const {
    checkValidity();
    return *iv_typeSystem;
  }

  bool Feature::isMultipleReferencesAllowed() const {
    checkValidity();
    return iv_typeSystem->isMultipleReferencesAllowed(iv_tyFeature);
  }


/////////////////////////////////////////////////////
// Type

  void Type::checkValidity() const {
    if (!isValid()) {
      UIMA_EXC_THROW_NEW(InvalidFSTypeObjectException,
                         UIMA_ERR_INVALID_FSTYPE_OBJECT,
                         UIMA_MSG_ID_EXC_INVALID_FSTYPE_OBJECT,
                         ErrorMessage(UIMA_MSG_ID_EXCON_UNKNOWN_CONTEXT),
                         ErrorInfo::recoverable
                        );
    }
  }

  Type::Type(lowlevel::TyFSType aType, uima::lowlevel::TypeSystem & typeSystem)
      : iv_tyType(aType),
      iv_typeSystem( & typeSystem) {}

  Type::Type()
      : iv_tyType(lowlevel::TypeSystem::INVALID_TYPE),
      iv_typeSystem(NULL) {}

  bool Type::isValid() const {
    return(iv_tyType != 0) && (EXISTS(iv_typeSystem));
  }

  UnicodeStringRef Type::getName() const {
    checkValidity();
    return UnicodeStringRef( iv_typeSystem->getTypeName(iv_tyType) );
  }

  UnicodeStringRef Type::getCreatorID() const {
    checkValidity();
    return UnicodeStringRef( iv_typeSystem->getTypeCreatorID(iv_tyType) );
  }

  void Type::getAppropriateFeatures(vector<uima::Feature> & rResult) const {
    checkValidity();
    rResult.clear();
    vector<uima::lowlevel::TyFSFeature> lowlevelResult;
    iv_typeSystem->getAppropriateFeatures( iv_tyType, lowlevelResult );
    vector<uima::lowlevel::TyFSFeature>::const_iterator cit;
    for (cit = lowlevelResult.begin(); cit != lowlevelResult.end(); ++cit) {
      rResult.push_back( uima::internal::FSPromoter::promoteFeature( *cit, *iv_typeSystem ) );
    }
  }

  bool Type::isAppropriateFeature(Feature const & f) const {
    checkValidity();
    return iv_typeSystem->isAppropriateFeature( iv_tyType, uima::internal::FSPromoter::demoteFeature(f) );
  }

  void Type::getDirectSubTypes(vector<uima::Type> & rResult) const {
    checkValidity();
    rResult.clear();
    vector<uima::lowlevel::TyFSType> lowlevelResult;
    iv_typeSystem->getDirectSubTypes(iv_tyType, lowlevelResult);
    vector<uima::lowlevel::TyFSType>::const_iterator cit;
    for (cit = lowlevelResult.begin(); cit != lowlevelResult.end(); ++cit) {
      rResult.push_back( uima::internal::FSPromoter::promoteType( *cit, *iv_typeSystem ) );
    }
  }

  void Type::getSubTypes(std::vector<Type> & rResult) const {
    checkValidity();
    rResult.clear();
    vector<uima::lowlevel::TyFSType> lowlevelResult;
    iv_typeSystem->getSubsumedTypes(iv_tyType, lowlevelResult);
    vector<uima::lowlevel::TyFSType>::const_iterator cit;
    for (cit = lowlevelResult.begin(); cit != lowlevelResult.end(); ++cit) {
      rResult.push_back( uima::internal::FSPromoter::promoteType( *cit, *iv_typeSystem ) );
    }
  }

  Feature Type::getFeatureByBaseName(icu::UnicodeString const & crBaseName) const {
    checkValidity();
    lowlevel::TyFSFeature tyFeature = iv_typeSystem->getFeatureByBaseName(iv_tyType, crBaseName);
    return uima::internal::FSPromoter::promoteFeature(tyFeature, *iv_typeSystem);
  }


  bool Type::operator==(Type const & crOther) const {
//      checkValidity();
//      crOther.checkValidity();
    return( (iv_tyType == crOther.iv_tyType) && (iv_typeSystem == crOther.iv_typeSystem) );
  }

  /* taph 07.06.2002: we need an explicit != because of template deduction problems */
  bool Type::operator!=(Type const & crOther) const {
    return( !( (*this) == crOther ) );
  }

  bool Type::operator<(Type const & other) const {
    return iv_tyType < other.iv_tyType;
  }

  bool Type::subsumes(Type const & crType) const {
    checkValidity();
    crType.checkValidity();
    return( iv_typeSystem->subsumes( iv_tyType, crType.iv_tyType) );
  }

  uima::TypeSystem const & Type::getTypeSystem() const {
    checkValidity();
    return *iv_typeSystem;
  }

  bool Type::isStringSubType() const {
    return iv_typeSystem->isStringSubType(iv_tyType);
  }

  ////////////////////////////////////////////////////////////
  // TypeSystem

  const char TypeSystem::FEATURE_SEPARATOR = UIMA_TYPE_FEATURE_SEPARATOR_CHAR;
  const char TypeSystem::NAMESPACE_SEPARATOR = UIMA_NAMESPACE_SEPARATOR_CHAR;

  TypeSystem::TypeSystem() {}

  TypeSystem::~TypeSystem() {}

  Type TypeSystem::getTopType() const {
    return uima::internal::FSPromoter::promoteType( getLowlevelTypeSystem().getTopType(), getLowlevelTypeSystem() );
  }


  Type TypeSystem::getType(icu::UnicodeString const & crTypeName) const {
    return internal::FSPromoter::promoteType( getLowlevelTypeSystem().getTypeByName( crTypeName ), getLowlevelTypeSystem() );
  }


  Feature TypeSystem::getFeatureByFullName(icu::UnicodeString const & crFeatureName) const {
    return internal::FSPromoter::promoteFeature( getLowlevelTypeSystem().getFeatureByFullName( crFeatureName ), getLowlevelTypeSystem() );
  }

  void TypeSystem::getAllTypes(vector<Type> & rResult) const {
    vector<lowlevel::TyFSType> vecLOLTypes;
    getLowlevelTypeSystem().getAllTypes(vecLOLTypes);
    rResult.clear();
    size_t i;
    for (i=0; i<vecLOLTypes.size(); ++i) {
      rResult.push_back( uima::internal::FSPromoter::promoteType(vecLOLTypes[i], getLowlevelTypeSystem()));
    }
  }

  void TypeSystem::getAllFeatures(vector<Feature> & rResult) const {
    vector<lowlevel::TyFSFeature> vecLOLFeatures;
    getLowlevelTypeSystem().getAllFeatures(vecLOLFeatures);
    rResult.clear();
    size_t i;
    for (i=0; i<vecLOLFeatures.size(); ++i) {
      rResult.push_back( uima::internal::FSPromoter::promoteFeature(vecLOLFeatures[i], getLowlevelTypeSystem()));
    }
  }

  bool TypeSystem::isPrimitive(lowlevel::TyFSType tyType) const {

    //lowlevel::TyFSType tyType = uima::internal::FSPromoter::demoteType(type);
    return (tyType == uima::internal::gs_tyIntegerType)
           ||   (tyType == uima::internal::gs_tyFloatType)
           ||   (tyType == uima::internal::gs_tyStringType)
           ||   ( getLowlevelTypeSystem().subsumes(uima::internal::gs_tyStringType, tyType) )
           ||   (tyType == uima::internal::gs_tyByteType)
           ||   (tyType == uima::internal::gs_tyBooleanType)
           ||   (tyType == uima::internal::gs_tyShortType)
           ||   (tyType == uima::internal::gs_tyLongType)
           ||   (tyType == uima::internal::gs_tyDoubleType)
           ;
  }

  bool TypeSystem::isStringSubType(lowlevel::TyFSType tyType) const {
    //lowlevel::TyFSType tyType = uima::internal::FSPromoter::demoteType(type);
    return  ( getLowlevelTypeSystem().getParentType(tyType) == uima::internal::gs_tyStringType);
  }

  /*------------------------------ Static Methods -------------------------------*/

  // Release contents of vector container allocated by some of the "get" methods
  void TypeSystem::release(std::vector<uima::Type> & rResult) {
    rResult.clear();
  }

  void TypeSystem::release(std::vector<uima::Feature> & rResult) {
    rResult.clear();
  }
}
/* ----------------------------------------------------------------------- */





