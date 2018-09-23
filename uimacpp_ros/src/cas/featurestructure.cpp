/** \file featurestructure.cpp .
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
#include <uima/featurestructure.hpp>
#include <uima/lowlevel_fsheap.hpp>
#include <uima/msg.h>
#include <uima/internal_fspromoter.hpp>
#include <uima/internal_typeshortcuts.hpp>
#include <uima/listfs.hpp>
#include <uima/arrayfs.hpp>
#include <uima/cas.hpp>
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

  /*
  Most of the methods in here are thin wrappers over then lowlevel API
  but also include error checking.
  */

  /* ----------------------------------------------------------------------- */
  /*       Exceptions Implementation                                         */
  /* ----------------------------------------------------------------------- */

  UIMA_EXC_CLASSIMPLEMENT(InvalidFSObjectException, CASException);
  UIMA_EXC_CLASSIMPLEMENT(FeatureNotAppropriateException, CASException);
  UIMA_EXC_CLASSIMPLEMENT(IncompatibleValueTypeException, CASException);
  UIMA_EXC_CLASSIMPLEMENT(FSIsNotStringException, CASException);
  UIMA_EXC_CLASSIMPLEMENT(WrongStringValueException, CASException);

  /* ----------------------------------------------------------------------- */
  /*       Tool Functions Implementation                                     */
  /* ----------------------------------------------------------------------- */

  /**
   * Checks if this FS is valid and throws an appropriate exeption
   * with the passed context if not.
   */
  void FeatureStructure::checkValidity(TyMessageId tyContext) const {
    if (!isValid()) {
      UIMA_EXC_THROW_NEW(InvalidFSObjectException,
                         UIMA_ERR_INVALID_FS_OBJECT,
                         UIMA_MSG_ID_EXC_INVALID_FS_OBJECT,
                         ErrorMessage(tyContext),
                         ErrorInfo::recoverable
                        );
    }
  }

  /**
   * Checks whether the passed feature is defined on the type of this FS.
   * Throws an appropriate if not.
   */
  void FeatureStructure::checkFeature(Feature const & f, TyMessageId tyContext) const {
    if (!f.isValid()) {
      UIMA_EXC_THROW_NEW(InvalidFSFeatureObjectException,
                         UIMA_ERR_INVALID_FSFEATURE_OBJECT,
                         UIMA_MSG_ID_EXC_INVALID_FSFEATURE_OBJECT,
                         ErrorMessage(tyContext),
                         ErrorInfo::recoverable
                        );
    }
    assert(EXISTS(iv_cas));
    if (! iv_cas->getHeap()->getTypeSystem()
        .isAppropriateFeature( iv_cas->getHeap()->getType(iv_tyFS),
                               internal::FSPromoter::demoteFeature(f)) ) {
      ErrorMessage msg(UIMA_MSG_ID_EXC_FEATURE_NOT_APPROPRIATE);
      UnicodeStringRef ref = f.getName();
      icu::UnicodeString usFeatureName( ref.getBuffer(), ref.length() );
      msg.addParam( usFeatureName );
      msg.addParam( getType().getName() );

      UIMA_EXC_THROW_NEW(FeatureNotAppropriateException,
                         UIMA_ERR_FEATURE_NOT_APPROPRIATE,
                         msg,
                         ErrorMessage(tyContext),
                         ErrorInfo::recoverable
                        );
    }
  }

  /**
   * checks if the range type of the feature is not built-in.
   */
  void FeatureStructure::checkNonBuiltinFeature(Feature const & crFeature, TyMessageId tyContext) const {
    assert(EXISTS(iv_cas));
    lowlevel::TypeSystem const & rTS = iv_cas->getHeap()->getTypeSystem();
    uima::lowlevel::TyFSType rangeType = rTS.getRangeType(internal::FSPromoter::demoteFeature(crFeature));
    if ( rangeType == uima::internal::gs_tyIntegerType
         || rangeType == uima::internal::gs_tyFloatType
         || rangeType == uima::internal::gs_tyStringType
         || rTS.getParentType(rangeType) == uima::internal::gs_tyStringType ) {
      ErrorMessage msg( UIMA_MSG_ID_EXC_INCOMPATIBLE_RANGE_TYPE);
      msg.addParam( rTS.getFeatureName( internal::FSPromoter::demoteFeature(crFeature) ) );
      Type t;
      crFeature.getRangeType(t);
      msg.addParam( t.getName()  );
      UIMA_EXC_THROW_NEW(FeatureNotAppropriateException,
                         UIMA_ERR_INCOMPATIBLE_RANGE_TYPE,
                         msg,
                         ErrorMessage(tyContext),
                         ErrorInfo::recoverable
                        );
    }
  }

  /**
   * checks if the range type of the feature subsumes the aType.
   * In other words, checks if an FS of aType is an appropriate value for f.
   */
  void FeatureStructure::checkAppropFeature(Feature const & f, lowlevel::TyFSType aType, TyMessageId tyContext) const {
    checkFeature(f, tyContext);
    assert(EXISTS(iv_cas));
    uima::lowlevel::TypeSystem const & ts = iv_cas->getHeap()->getTypeSystem();

    //This to deal with FS array type with <elementType> tag
    //which results in a feature range type that is a subtype of
    //FS array.  FSs of type FSArray are allowed values.
    if (aType == uima::internal::gs_tyFSArrayType) {
      Type featType;
      f.getRangeType(featType);
      lowlevel::TyFSType featTypeCode = internal::FSPromoter::demoteType(featType);
      if (aType != featTypeCode && !ts.subsumes(aType,featTypeCode) ) {
        ErrorMessage msg( UIMA_MSG_ID_EXC_INCOMPATIBLE_RANGE_TYPE);
        msg.addParam( ts.getFeatureName(internal::FSPromoter::demoteFeature(f)) );
        msg.addParam( ts.getTypeName(aType) );
        UIMA_EXC_THROW_NEW(IncompatibleValueTypeException,
                           UIMA_ERR_INCOMPATIBLE_RANGE_TYPE,
                           msg,
                           ErrorMessage(tyContext),
                           ErrorInfo::recoverable
                          );
      }
    } else if ( !ts.subsumes(ts.getRangeType(internal::FSPromoter::demoteFeature(f)), aType ) ) {
      ErrorMessage msg( UIMA_MSG_ID_EXC_INCOMPATIBLE_RANGE_TYPE);
      msg.addParam( ts.getFeatureName(internal::FSPromoter::demoteFeature(f)) );
      msg.addParam( ts.getTypeName(aType) );
      UIMA_EXC_THROW_NEW(IncompatibleValueTypeException,
                         UIMA_ERR_INCOMPATIBLE_RANGE_TYPE,
                         msg,
                         ErrorMessage(tyContext),
                         ErrorInfo::recoverable
                        );
    }
  }

  /**
   * checks if anFS is an appropriate value for f.
   */
  void FeatureStructure::checkAppropFeature(Feature const & f, FeatureStructure const & anFS, TyMessageId tyContext) const {
    if ( (&iv_cas->getHeap()->getTypeSystem()) != (&anFS.iv_cas->getHeap()->getTypeSystem()) ) {
      UIMA_EXC_THROW_NEW(InvalidFSObjectException,
                         UIMA_ERR_INVALID_FS_OBJECT,
                         UIMA_MSG_ID_EXC_INVALID_FS_OBJECT,
                         ErrorMessage(tyContext),
                         ErrorInfo::recoverable
                        );
    }
    checkAppropFeature(f, iv_cas->getHeap()->getType(anFS.iv_tyFS), tyContext );
  }

  /* ----------------------------------------------------------------------- */
  /*       General FeatureStructure Implementation                           */
  /* ----------------------------------------------------------------------- */

  FeatureStructure::FeatureStructure(lowlevel::TyFS anFS, CAS & cas)
      : iv_tyFS(anFS),
      iv_cas(&cas) {}


  FeatureStructure::FeatureStructure()
      : iv_tyFS(0),
      iv_cas(NULL) {}


  CAS & FeatureStructure::getCAS() {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_FSTYPE);
    assert(EXISTS(iv_cas));
    return *iv_cas;
  }

  CAS const & FeatureStructure::getCAS() const {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_FSTYPE);
    assert(EXISTS(iv_cas));
    return *iv_cas;
  }

  bool FeatureStructure::isValid() const {
    if (! EXISTS(iv_cas) ) {
      return false;
    }
    return iv_tyFS != 0 && ( iv_cas->getHeap()->isValid(iv_tyFS));
  }

  Type FeatureStructure::getType() const {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_FSTYPE);
    return uima::internal::FSPromoter::promoteType(iv_cas->getHeap()->getType(iv_tyFS), iv_cas->getHeap()->getTypeSystem() );
  }

  FeatureStructure FeatureStructure::clone() {
    checkValidity(UIMA_MSG_ID_EXCON_CREATING_FS);
    FeatureStructure newFS = getCAS().createFS(getType());
    iv_cas->getHeap()->copyFeatures(newFS.iv_tyFS, iv_tyFS);
    return newFS;
  }

  FeatureStructure FeatureStructure::clone(Type const & t) {
    checkValidity(UIMA_MSG_ID_EXCON_CREATING_FS);
    uima::lowlevel::TypeSystem const & ts = iv_cas->getHeap()->getTypeSystem();
    FeatureStructure newFS = getCAS().createFS(t);
    lowlevel::TyFSType mcst = ts.getMostSpecificCommonSupertype( uima::internal::FSPromoter::demoteType(getType()),
                              uima::internal::FSPromoter::demoteType(t) );
    lowlevel::TyFeatureOffset featNum = ts.getFeatureNumber( mcst );
    iv_cas->getHeap()->copyFeatures(newFS.iv_tyFS, iv_tyFS, featNum);
    return newFS;
  }

  FeatureStructure FeatureStructure::getFeatureValue(Feature const & f) const {
    return getFSValue(f);
  }

  FeatureStructure FeatureStructure::getFSValue(Feature const & f) const {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    checkFeature(f, UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    checkNonBuiltinFeature(f, UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    lowlevel::TyFSFeature tyFeat =  internal::FSPromoter::demoteFeature( f );
    lowlevel::TyFS resultAddr = iv_cas->getHeap()->getFSValue(iv_tyFS, tyFeat);
    if (resultAddr != uima::lowlevel::FSHeap::INVALID_FS &&
        iv_cas->getHeap()->getTypeSystem().subsumes(internal::gs_tyAnnotationType,
            internal::FSPromoter::demoteType(internal::FSPromoter::promoteFS(resultAddr, *iv_cas).getType()))) {
      // an annotation. Check that sofaNum agrees with current CAS
      lowlevel::TyFS annSofaAddr = iv_cas->getHeap()->getFSValue(resultAddr, internal::gs_tySofaRefFeature);
      if ( annSofaAddr != internal::FSPromoter::demoteFS(iv_cas->getSofa())) {
        // does not agree. Get appropriate View for annotation Sofa
        int annSofaNum = iv_cas->getHeap()->getIntValue(annSofaAddr, internal::gs_tySofaNumFeature);
        CAS* annTcas = iv_cas->getViewBySofaNum(annSofaNum);
        return FeatureStructure(resultAddr, (CAS &)*annTcas);
      }
    }
    return FeatureStructure(resultAddr, *iv_cas);
  }

  bool FeatureStructure::isUntouchedFSValue(Feature const & crFeature) const {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    checkFeature(crFeature, UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    return iv_cas->getHeap()->isUntouchedFSValue(iv_tyFS, internal::FSPromoter::demoteFeature(crFeature));
  }

  int FeatureStructure::getIntValue(Feature const & f) const {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    checkAppropFeature(f, uima::internal::gs_tyIntegerType, UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    return iv_cas->getHeap()->getIntValue(iv_tyFS, internal::FSPromoter::demoteFeature(f));
  }

  void FeatureStructure::setFSValue(Feature const & f, FeatureStructure const & anFS) {
    checkValidity(UIMA_MSG_ID_EXCON_SETTING_FEATURE_VALUE);
    checkAppropFeature(f, anFS, UIMA_MSG_ID_EXCON_SETTING_FEATURE_VALUE);
    iv_cas->getHeap()->setFSValue(iv_tyFS, internal::FSPromoter::demoteFeature(f), anFS.iv_tyFS);
  }

  void FeatureStructure::setFeatureValue(Feature const & f, FeatureStructure const & anFS) {
    setFSValue(f, anFS);
  }

  void FeatureStructure::setIntValue(Feature const & f, int i) {
    checkValidity(UIMA_MSG_ID_EXCON_SETTING_FEATURE_VALUE);
    checkAppropFeature(f, uima::internal::gs_tyIntegerType, UIMA_MSG_ID_EXCON_SETTING_FEATURE_VALUE);
    iv_cas->getHeap()->setIntValue(iv_tyFS, internal::FSPromoter::demoteFeature(f), i);
  }

  float FeatureStructure::getFloatValue(Feature const & crFeature) const {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    checkAppropFeature(crFeature, uima::internal::gs_tyFloatType, UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    return iv_cas->getHeap()->getFloatValue(iv_tyFS, internal::FSPromoter::demoteFeature(crFeature));
  }

  void FeatureStructure::setFloatValue(Feature const & crFeature, float f) {
    checkValidity(UIMA_MSG_ID_EXCON_SETTING_FEATURE_VALUE);
    checkAppropFeature(crFeature, uima::internal::gs_tyFloatType, UIMA_MSG_ID_EXCON_SETTING_FEATURE_VALUE);
    iv_cas->getHeap()->setFloatValue(iv_tyFS, internal::FSPromoter::demoteFeature(crFeature), f);
  }

  bool FeatureStructure::getBooleanValue(Feature const & crFeature) const {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    checkAppropFeature(crFeature, uima::internal::gs_tyBooleanType, UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    return iv_cas->getHeap()->getBooleanValue(iv_tyFS, internal::FSPromoter::demoteFeature(crFeature));
  }

  void FeatureStructure::setBooleanValue(Feature const & crFeature, bool b ) {
    checkValidity(UIMA_MSG_ID_EXCON_SETTING_FEATURE_VALUE);
    checkAppropFeature(crFeature, uima::internal::gs_tyBooleanType, UIMA_MSG_ID_EXCON_SETTING_FEATURE_VALUE);

    //add byte to 8bit heap
    lowlevel::TyFSFeature tyFeat =  internal::FSPromoter::demoteFeature( crFeature );
    iv_cas->getHeap()->setBooleanValue( iv_tyFS, tyFeat, b);
  }


  char FeatureStructure::getByteValue(Feature const & crFeature) const {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    checkAppropFeature(crFeature, uima::internal::gs_tyByteType, UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    return iv_cas->getHeap()->getByteValue(iv_tyFS, internal::FSPromoter::demoteFeature(crFeature));
  }

  void FeatureStructure::setByteValue(Feature const & crFeature, char b ) {
    checkValidity(UIMA_MSG_ID_EXCON_SETTING_FEATURE_VALUE);
    checkAppropFeature(crFeature, uima::internal::gs_tyByteType, UIMA_MSG_ID_EXCON_SETTING_FEATURE_VALUE);

    //add byte to 8bit heap
    lowlevel::TyFSFeature tyFeat =  internal::FSPromoter::demoteFeature( crFeature );
    iv_cas->getHeap()->setByteValue( iv_tyFS, tyFeat, b );
  }

  short FeatureStructure::getShortValue(Feature const & crFeature) const {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    checkAppropFeature(crFeature, uima::internal::gs_tyShortType, UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    return iv_cas->getHeap()->getShortValue(iv_tyFS, internal::FSPromoter::demoteFeature(crFeature));
  }

  void FeatureStructure::setShortValue(Feature const & crFeature, short s ) {
    checkValidity(UIMA_MSG_ID_EXCON_SETTING_FEATURE_VALUE);
    checkAppropFeature(crFeature, uima::internal::gs_tyShortType, UIMA_MSG_ID_EXCON_SETTING_FEATURE_VALUE);

    //add byte to 8bit heap
    lowlevel::TyFSFeature tyFeat =  internal::FSPromoter::demoteFeature( crFeature );
    iv_cas->getHeap()->setShortValue( iv_tyFS, tyFeat, s );
  }


  INT64 FeatureStructure::getLongValue(Feature const & crFeature) const {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    checkAppropFeature(crFeature, uima::internal::gs_tyLongType, UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    return iv_cas->getHeap()->getLongValue(iv_tyFS, internal::FSPromoter::demoteFeature(crFeature));
  }

  void FeatureStructure::setLongValue(Feature const & crFeature, INT64 l ) {
    checkValidity(UIMA_MSG_ID_EXCON_SETTING_FEATURE_VALUE);
    checkAppropFeature(crFeature, uima::internal::gs_tyLongType, UIMA_MSG_ID_EXCON_SETTING_FEATURE_VALUE);

    //add byte to 8bit heap
    //int aRef = iv_cas->getHeap()->addLong(l);

    //set the 8bit heap offset in main heap
    lowlevel::TyFSFeature tyFeat =  internal::FSPromoter::demoteFeature( crFeature );
    iv_cas->getHeap()->setLongValue( iv_tyFS, tyFeat, l );
  }


  double FeatureStructure::getDoubleValue(Feature const & crFeature) const {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    checkAppropFeature(crFeature, uima::internal::gs_tyDoubleType, UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE);
    return iv_cas->getHeap()->getDoubleValue(iv_tyFS, internal::FSPromoter::demoteFeature(crFeature));
  }

  void FeatureStructure::setDoubleValue(Feature const & crFeature, double d ) {
    checkValidity(UIMA_MSG_ID_EXCON_SETTING_FEATURE_VALUE);
    checkAppropFeature(crFeature, uima::internal::gs_tyDoubleType, UIMA_MSG_ID_EXCON_SETTING_FEATURE_VALUE);

    //add byte to 8bit heap
    //int aRef = iv_cas->getHeap()->addDouble(d);

    //set the 8bit heap offset in main heap
    lowlevel::TyFSFeature tyFeat =  internal::FSPromoter::demoteFeature( crFeature );
    iv_cas->getHeap()->setDoubleValue( iv_tyFS, tyFeat, d );
  }





  /* ----------------------------------------------------------------------- */
  /*       String-oriented FeatureStructure Implementation                   */
  /* ----------------------------------------------------------------------- */

  void FeatureStructure::checkRangeIsString(Feature const & f, TyMessageId tyContext) const {
    assert(EXISTS(iv_cas));
    uima::lowlevel::FSHeap const & crHeap = *iv_cas->getHeap();
    uima::lowlevel::TyFSType tyType = crHeap.getTypeSystem().getRangeType(internal::FSPromoter::demoteFeature( f ));
    if ( ! crHeap.getTypeSystem().subsumes( uima::internal::gs_tyStringType, tyType) ) {
      UIMA_EXC_THROW_NEW(FSIsNotStringException,
                         UIMA_ERR_FS_IS_NOT_STRING,
                         UIMA_MSG_ID_EXC_FS_IS_NOT_STRING,
                         ErrorMessage(tyContext),
                         ErrorInfo::recoverable

                        );
    }
  }

  UnicodeStringRef  FeatureStructure::getStringValue(Feature const & f) const {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_STRINGVALUE_FROM_FS);
    checkRangeIsString(f, UIMA_MSG_ID_EXCON_GETTING_STRINGVALUE_FROM_FS);
    lowlevel::TyFSFeature tyFeat =  internal::FSPromoter::demoteFeature( f );
    lowlevel::TyFS tyStringFS = iv_cas->getHeap()->getFSValue(iv_tyFS, tyFeat);
    if (tyStringFS == uima::lowlevel::FSHeap::INVALID_FS) {
      return UnicodeStringRef();
    }
    return iv_cas->getHeap()->getFSAsString( tyStringFS );
  }

  void FeatureStructure::setStringValue(Feature const & f, UnicodeStringRef uls) {
    setStringValueExternal( f, uls );
  }

  void FeatureStructure::setStringValueExternal(Feature const & f, UnicodeStringRef uls) {
    checkValidity(UIMA_MSG_ID_EXCON_SETTING_STRINGVALUE_IN_FS);
    checkRangeIsString(f, UIMA_MSG_ID_EXCON_SETTING_STRINGVALUE_IN_FS);

    //!!! Not supporting External at this time. All strings added to StringHeap.
    int ulsRef = iv_cas->getHeap()->addString(uls);
    lowlevel::TyFSFeature tyFeat =  internal::FSPromoter::demoteFeature( f );

    uima::lowlevel::TypeSystem const & crTypeSystem = iv_cas->getHeap()->getTypeSystem();
    // if string sub type
    uima::lowlevel::TyFSType tyRangeType = crTypeSystem.getRangeType(tyFeat);
    if ( tyRangeType != uima::internal::gs_tyStringType) {
      vector<icu::UnicodeString> const & crStringSet = crTypeSystem.getStringsForStringSubtype(tyRangeType);
      vector<icu::UnicodeString>::const_iterator cit;
      bool bStringFound = false;
      for (cit = crStringSet.begin(); cit != crStringSet.end(); ++cit) {
        UnicodeStringRef ref(*cit);
        if ( ref == uls ) {
          bStringFound = true;
          break;
        }
      }
      if (! bStringFound) {
        UnicodeString us(uls.getBuffer(), uls.length() );
        ErrorMessage errMess(UIMA_MSG_ID_EXC_WRONG_STRING_VALUE);
        errMess.addParam(us);
        errMess.addParam(crTypeSystem.getTypeName(tyRangeType) );
        UIMA_EXC_THROW_NEW(WrongStringValueException,
                           UIMA_ERR_WRONG_STRING_VALUE,
                           errMess,
                           ErrorMessage(UIMA_MSG_ID_EXCON_SETTING_STRINGVALUE_IN_FS),
                           ErrorInfo::recoverable);
      }
    }



    iv_cas->getHeap()->setStringValue( iv_tyFS, tyFeat, ulsRef );
  }

  /* ----------------------------------------------------------------------- */
  /*       Array-oriented FeatureStructure Implementation                    */
  /* ----------------------------------------------------------------------- */

  ArrayFS FeatureStructure::getArrayFSValue( Feature const & fArray ) const {
    return ArrayFS(getFSValue(fArray));
  }

  FloatArrayFS FeatureStructure::getFloatArrayFSValue( Feature const & fArray ) const {
    return FloatArrayFS(getFSValue(fArray));
  }

  IntArrayFS FeatureStructure::getIntArrayFSValue( Feature const & fArray ) const {
    return IntArrayFS(getFSValue(fArray));
  }

  StringArrayFS FeatureStructure::getStringArrayFSValue( Feature const & fArray ) const {
    return StringArrayFS(getFSValue(fArray));
  }

  BooleanArrayFS FeatureStructure::getBooleanArrayFSValue( Feature const & fArray ) const {
    return BooleanArrayFS(getFSValue(fArray));
  }

  ByteArrayFS FeatureStructure::getByteArrayFSValue( Feature const & fArray ) const {
    return ByteArrayFS(getFSValue(fArray));
  }

  ShortArrayFS FeatureStructure::getShortArrayFSValue( Feature const & fArray ) const {
    return ShortArrayFS(getFSValue(fArray));
  }

  LongArrayFS FeatureStructure::getLongArrayFSValue( Feature const & fArray ) const {
    return LongArrayFS(getFSValue(fArray));
  }
  DoubleArrayFS FeatureStructure::getDoubleArrayFSValue( Feature const & fArray ) const {
    return DoubleArrayFS(getFSValue(fArray));
  }


  /* ----------------------------------------------------------------------- */
  /*       List-oriented FeatureStructure Implementation                     */
  /* ----------------------------------------------------------------------- */

  bool FeatureStructure::hasListElements(Feature const & f) const {
    Type tRangeType;
    f.getRangeType(tRangeType);
    lowlevel::TyFSFeature tyRangeType = internal::FSPromoter::demoteType(tRangeType);
    switch (tyRangeType) {
    case uima::internal::gs_tyFSListType    :
    case uima::internal::gs_tyEListType     :
    case uima::internal::gs_tyNEListType    :
      return ListFS::hasListElements(*this, f);
    case uima::internal::gs_tyFloatListType   :
    case uima::internal::gs_tyEFloatListType  :
    case uima::internal::gs_tyNEFloatListType :
      return FloatListFS::hasListElements(*this, f);
    case uima::internal::gs_tyIntListType     :
    case uima::internal::gs_tyEIntListType    :
    case uima::internal::gs_tyNEIntListType   :
      return IntListFS::hasListElements(*this, f);
    case uima::internal::gs_tyStringListType  :
    case uima::internal::gs_tyEStringListType :
    case uima::internal::gs_tyNEStringListType:
      return StringListFS::hasListElements(*this, f);
    }
    UIMA_EXC_THROW_NEW(FSIsNotListException,
                       UIMA_ERR_FS_IS_NOT_LIST,
                       UIMA_MSG_ID_EXC_FS_IS_NOT_LIST,
                       ErrorMessage(UIMA_MSG_ID_EXCON_GETTING_LIST_ISEMPTY),
                       ErrorInfo::recoverable
                      );
    return false;
  }

  ListFS FeatureStructure::getListFSValue( Feature const & f ) const {
    return ListFS::getListFSValue(*this, f);
  }

  FloatListFS FeatureStructure::getFloatListFSValue( Feature const & f ) const {
    return FloatListFS::getListFSValue(*this, f);
  }

  IntListFS FeatureStructure::getIntListFSValue( Feature const & f ) const {
    return IntListFS::getListFSValue(*this, f);
  }

  StringListFS FeatureStructure::getStringListFSValue( Feature const & f ) const {
    return StringListFS::getListFSValue(*this, f);
  }


  /* ----------------------------------------------------------------------- */
  /*       Operator FeatureStructure Implementation                          */
  /* ----------------------------------------------------------------------- */

  bool FeatureStructure::operator==(FeatureStructure const & crFS) const {
    return (iv_cas == crFS.iv_cas) && (iv_tyFS == crFS.iv_tyFS);
  }

  bool FeatureStructure::operator<(FeatureStructure const & crFS) const {
    return (iv_cas == crFS.iv_cas) && (iv_tyFS < crFS.iv_tyFS);
  }

} // namespace uima

/* ----------------------------------------------------------------------- */
/* <EOF> */

