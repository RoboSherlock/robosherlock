#ifndef UIMA_FEATURESTRUCTURE_HPP
#define UIMA_FEATURESTRUCTURE_HPP
/** \file featurestructure.hpp .
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

    \brief  Contains class uima::FeatureStructure

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp>

#include <uima/lowlevel_typedefs.hpp>
#include <uima/typesystem.hpp>
#include <uima/casexception.hpp>
#include <uima/unistrref.hpp>

#include "unicode/utf.h"
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */


namespace uima {
  class ListFS;
  class FloatListFS;
  class IntListFS;
  class StringListFS;
  class ArrayFS;
  class FloatArrayFS;
  class IntArrayFS;
  class StringArrayFS;
  class StringFilter;
  class ByteArrayFS;
  class BooleanArrayFS;
  class ShortArrayFS;
  class LongArrayFS;
  class DoubleArrayFS;

  namespace internal {
    class IndexCompWrapper;
    class FSFilterWrapper;
    class FSPromoter;
  }
  namespace lowlevel {
    class FSHeap;
  }
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */



namespace uima {
  // exceptions
  UIMA_EXC_CLASSDECLARE(InvalidFSObjectException, CASException);
  UIMA_EXC_CLASSDECLARE(FeatureNotAppropriateException, CASException);
  UIMA_EXC_CLASSDECLARE(IncompatibleValueTypeException, CASException);
  UIMA_EXC_CLASSDECLARE(FSIsNotStringException, CASException);
  UIMA_EXC_CLASSDECLARE(WrongStringValueException, CASException);


  /**
   * Our feature structure.
   * All methods may throw an <code>InvalidFSObjectException</code> if the
   * feature structure object is not valid.
   */
  class UIMA_LINK_IMPORTSPEC FeatureStructure {
    friend class uima::internal::FSPromoter;
    friend class uima::ListFS;
    friend class uima::ArrayFS;
    friend class uima::StringFilter;
    friend class uima::CAS;
    friend class RemoteSofaDataStream;
    friend class LocalSofaDataStream;
    friend class XCASDeserializerHandler;
	friend class XmiDeserializerHandler;

  protected:
    lowlevel::TyFS      iv_tyFS;
    CAS * iv_cas;

    //FeatureStructure(lowlevel::TyFS anFS, uima::lowlevel::FSHeap &);
    FeatureStructure(lowlevel::TyFS anFS, uima::CAS &);

    void checkValidity(TyMessageId) const;
    void checkFeature(Feature const & f, TyMessageId) const;
    void checkNonBuiltinFeature(Feature const & f, TyMessageId) const;
    void checkAppropFeature(Feature const & f, lowlevel::TyFSType aType, TyMessageId) const;
    void checkAppropFeature(Feature const & f, FeatureStructure const & anFS, TyMessageId) const;

    void checkRangeIsString(Feature const & fList, TyMessageId) const;

  public:
    typedef lowlevel::TyHeapCell TyArrayElement;

    /**
     * Default constructor: Creates an invalid FS.
     */
    FeatureStructure();

    /**
     * Returns the CAS object in which this feature structure lives.
     */
    CAS & getCAS();
    CAS const & getCAS() const;

    /**
     * Check if this FS object is valid, i.e., properly initialized.
     */
    bool isValid() const;

    /**
     * @return the type of this FS.
     * @throws InvalidFSObjectException
     */
    Type getType() const;


    /**
     * Creates a copy of this feature structure.
     * The returned feature structure is a new and separate object but
     * all features of the feature structure which are not of builtin
     * types (integer, float, string) will be shared between the clone
     * and it's source FS.
     * @return the cloned copy of this object.
     * @throws InvalidFSObjectException
     */
    FeatureStructure clone();

    /**
     * creates a feature structure of type t and copies all
     * "common" features of t and this->getType().
     * A feature is common to two types t1 and t2 if it is defined on the 
     * most specific common supertype of t1 and t2.
     */
    FeatureStructure clone(Type const & t);

    /**
      * Check if the value of <code>crFeature</code> was already used via
      * a call to <code>getFSValue()</code> or <code>setFSValue()</code>.
      * @throws InvalidFSObjectException
      * @throws FeatureNotAppropriateException
      */
    bool isUntouchedFSValue(Feature const & crFeature) const;

    /**
     * Get the value of feature <code>crFeature</code> of this feature structure
     * (must not be a builtin type).
     * @throws InvalidFSObjectException
     * @throws FeatureNotAppropriateException
     */
    FeatureStructure getFSValue(Feature const & crFeature) const;

    /**
     * Get the value of feature <code>crFeature</code> of this feature structure
     * (must not be a builtin type).
     * @throws InvalidFSObjectException
     * @throws FeatureNotAppropriateException
     */
    FeatureStructure getFeatureValue(Feature const & crFeature) const;

    /**
     * set the value of feature <code>crFeature</code> of this feature structure
     * (must not be a builtin type).
     * @throws InvalidFSObjectException
     * @throws FeatureNotAppropriateException
     * @throws IncompatibleValueTypeException
     */
    void setFSValue(Feature const & crFeature, FeatureStructure const & anFS);

    /**
     * set the value of feature <code>crFeature</code> of this feature structure
     * (must not be a builtin type).
     * @throws InvalidFSObjectException
     * @throws FeatureNotAppropriateException
     * @throws IncompatibleValueTypeException
     */
    void setFeatureValue(Feature const & crFeature, FeatureStructure const & anFS);

    /**
     * Get the value of feature <code>crFeature</code>, must be builtin type integer.
     * @throws InvalidFSObjectException
     * @throws FeatureNotAppropriateException
     * @throws IncompatibleValueTypeException
     */
    int getIntValue(Feature const & crFeature) const;

    /**
     * Set the value of feature <code>crFeature</code>, must be builtin type integer.
     * @throws InvalidFSObjectException
     * @throws FeatureNotAppropriateException
     * @throws IncompatibleValueTypeException
     */
    void setIntValue(Feature const & crFeature, int i);

    /**
     * Get the value of feature <code>crFeature</code>, must be builtin type float.
     * @throws InvalidFSObjectException
     * @throws FeatureNotAppropriateException
     * @throws IncompatibleValueTypeException
     */
    float getFloatValue(Feature const & crFeature) const;

    /**
     * Set the value of feature <code>crFeature</code>, must be builtin type float.
     * @throws InvalidFSObjectException
     * @throws FeatureNotAppropriateException
     * @throws IncompatibleValueTypeException
     */
    void setFloatValue(Feature const & crFeature, float );

    /**
     * Get the value of feature <code>crFeature</code>, must be of type string.
     * @throws InvalidFSObjectException
     * @throws FeatureNotAppropriateException
     * @throws FSIsNotAStringException
     */
    UnicodeStringRef getStringValue(Feature const & crFeature) const;

    /**
     * sets the value of feature <code>crFeature</code> of this feature structure
     * to a new string on the heap.
     * Precondition: getFSValue(crFeature) must be of type string.
     * The string is copied to the heap and the string value will point
     * to the copy on the heap and not to <code>cuStr</code>.
     * @throws InvalidFSObjectException
     * @throws FeatureNotAppropriateException
     * @throws FSIsNotAStringException
     */
    void setStringValue(Feature const & crFeature, UnicodeStringRef cuStr);
    void setStringValue(Feature const & crFeature, UChar const * cuStr, size_t uiLen );
    void setStringValue(Feature const & crFeature, icu::UnicodeString const & crustr );

    /**
     * sets the value of feature <code>crFeature</code> of this feature structure
     * to a new string outside of the heap.
     * Precondition: FS must be of type string.
     * The string is <strong>not</strong> copied to the heap,
     * so the string value will point to <code>cuStr</code>.
     * The caller must make sure that the characters in <code>cuStr</code>
     * have an appropriate live time.
     * For persistency of the CAS to be consistent and complete the caller
     * must make sure that all the string data lives somewhere on the heap.
     *
     * @throws InvalidFSObjectException
     * @throws FSIsNotAStringException
     */
    void setStringValueExternal(Feature const & crFeature,  UnicodeStringRef cuStr);
    void setStringValueExternal(Feature const & crFeature,  UChar const * cuStr, size_t uiLen );
    void setStringValueExternal(Feature const & crFeature,  icu::UnicodeString const & crustr );


    /**
     * Return feature structure of type array stored at feature fArray.
     * The return value is guaranteed to be a properly terminated array.
     * Each element in the array is a FeatureStructure.
     *
     * @param fArray  The feature referencing the array to return.
     *                fArray must be valid.
     *                fArray must be appropriate for this feature strucutre.
     *                fArray must be subsumed by type array.
     *
     * @throws InvalidFSObjectException
     * @throws InvalidFSFeatureObjectException
     * @throws FeatureNotAppropriateException
     * @throws FSIsNotArrayException
     */
    ArrayFS getArrayFSValue(Feature const & fArray) const;

    /**
     * Return feature structure of type array stored at feature fArray.
     * The return value is guaranteed to be a properly terminated array.
     * Each element in the array is a float.
     *
     * @param fArray  The feature referencing the array to return.
     *                fArray must be valid.
     *                fArray must be appropriate for this feature strucutre.
     *                fArray must be subsumed by type array.
     *
     * @throws InvalidFSObjectException
     * @throws InvalidFSFeatureObjectException
     * @throws FeatureNotAppropriateException
     * @throws FSIsNotArrayException
     */
    FloatArrayFS getFloatArrayFSValue( Feature const & fArray ) const;

    /**
     * Return feature structure of type array stored at feature fArray.
     * The return value is guaranteed to be a properly terminated array.
     * Each element in the array is a int.
     *
     * @param fArray  The feature referencing the array to return.
     *                fArray must be valid.
     *                fArray must be appropriate for this feature strucutre.
     *                fArray must be subsumed by type array.
     *
     * @throws InvalidFSObjectException
     * @throws InvalidFSFeatureObjectException
     * @throws FeatureNotAppropriateException
     * @throws FSIsNotArrayException
     */
    IntArrayFS getIntArrayFSValue( Feature const & fArray ) const;

    /**
     * Return feature structure of type array stored at feature fArray.
     * The return value is guaranteed to be a properly terminated array.
     * Each element in the array is a string.
     *
     * @param fArray  The feature referencing the array to return.
     *                fArray must be valid.
     *                fArray must be appropriate for this feature strucutre.
     *                fArray must be subsumed by type array.
     *
     * @throws InvalidFSObjectException
     * @throws InvalidFSFeatureObjectException
     * @throws FeatureNotAppropriateException
     * @throws FSIsNotArrayException
     */
    StringArrayFS getStringArrayFSValue( Feature const & fArray ) const;

    /**
     * Returns true if the list stored at feature fList has elements.
     *
     * @param fList   The feature to check.
     *                fList must be of type list
     *
     * @throws InvalidFSObjectException
     * @throws InvalidFSFeatureObjectException
     * @throws FeatureNotAppropriateException
     * @throws FSIsNotListException
     */
    bool hasListElements(Feature const & fList) const;

    /**
     * Return feature structure of type list stored at feature fList.
     * The return value is guaranteed to be a properly terminated list.
     * Each element of the list is a feature structure.
     *
     * @param fList   The feature referencing the list to return.
     *                fList must be valid.
     *                fList must be appropriate for this feature strucutre.
     *                fList must be subsumed by type list.
     *
     * @throws InvalidFSObjectException
     * @throws InvalidFSFeatureObjectException
     * @throws FeatureNotAppropriateException
     * @throws FSIsNotListException
     */
    ListFS getListFSValue(Feature const & fList) const;

    /**
     * Return feature structure of type list stored at feature fList.
     * The return value is guaranteed to be a properly terminated list.
     * Each element of the list is a float.
     *
     * @param f The feature referencing the list to return.
     *          f must be valid.
     *          f must be appropriate for this feature strucutre.
     *          f must be subsumed by type list.
     *
     * @throws InvalidFSObjectException
     * @throws InvalidFSFeatureObjectException
     * @throws FeatureNotAppropriateException
     * @throws FSIsNotListException
     */
    FloatListFS getFloatListFSValue( Feature const & f ) const;

    /**
     * Return feature structure of type list stored at feature fList.
     * The return value is guaranteed to be a properly terminated list.
     * Each element of the list is a int.
     *
     * @param f The feature referencing the list to return.
     *          f must be valid.
     *          f must be appropriate for this feature strucutre.
     *          f must be subsumed by type list.
     *
     * @throws InvalidFSObjectException
     * @throws InvalidFSFeatureObjectException
     * @throws FeatureNotAppropriateException
     * @throws FSIsNotListException
     */
    IntListFS getIntListFSValue( Feature const & f ) const;

    /**
     * Return feature structure of type list stored at feature fList.
     * The return value is guaranteed to be a properly terminated list.
     * Each element of the list is a string.
     *
     * @param f The feature referencing the list to return.
     *          f must be valid.
     *          f must be appropriate for this feature strucutre.
     *          f must be subsumed by type list.
     *
     * @throws InvalidFSObjectException
     * @throws InvalidFSFeatureObjectException
     * @throws FeatureNotAppropriateException
     * @throws FSIsNotListException
     */
    StringListFS getStringListFSValue( Feature const & f ) const;


    /**
     * Get the value of feature <code>crFeature</code>, must be builtin type byte.
     * @throws InvalidFSObjectException
     * @throws FeatureNotAppropriateException
     * @throws IncompatibleValueTypeException
     */
    bool getBooleanValue(Feature const & crFeature) const;

    /**
     * Set the value of feature <code>crFeature</code>, must be builtin type byte.
     * @throws InvalidFSObjectException
     * @throws FeatureNotAppropriateException
     * @throws IncompatibleValueTypeException
     */
    void setBooleanValue(Feature const & crFeature, bool );

    /**
     * Get the value of feature <code>crFeature</code>, must be builtin type byte.
     * @throws InvalidFSObjectException
     * @throws FeatureNotAppropriateException
     * @throws IncompatibleValueTypeException
     */
    char getByteValue(Feature const & crFeature) const;

    /**
     * Set the value of feature <code>crFeature</code>, must be builtin type byte.
     * @throws InvalidFSObjectException
     * @throws FeatureNotAppropriateException
     * @throws IncompatibleValueTypeException
     */
    void setByteValue(Feature const & crFeature, char );

    /**
    * Get the value of feature <code>crFeature</code>, must be builtin type short.
    * @throws InvalidFSObjectException
    * @throws FeatureNotAppropriateException
    * @throws IncompatibleValueTypeException
    */
    short getShortValue(Feature const & crFeature) const;

    /**
     * Set the value of feature <code>crFeature</code>, must be builtin type short.
     * @throws InvalidFSObjectException
     * @throws FeatureNotAppropriateException
     * @throws IncompatibleValueTypeException
     */
    void setShortValue(Feature const & crFeature, short );


    /**
    * Get the value of feature <code>crFeature</code>, must be builtin type short.
    * @throws InvalidFSObjectException
    * @throws FeatureNotAppropriateException
    * @throws IncompatibleValueTypeException
    */
    INT64 getLongValue(Feature const & crFeature) const;

    /**
    * Set the value of feature <code>crFeature</code>, must be builtin type short.
    * @throws InvalidFSObjectException
    * @throws FeatureNotAppropriateException
    * @throws IncompatibleValueTypeException
    */
    void setLongValue(Feature const & crFeature, INT64 );

    /**
     * Set the value of feature <code>crFeature</code>, must be builtin type short.
     * @throws InvalidFSObjectException
     * @throws FeatureNotAppropriateException
     * @throws IncompatibleValueTypeException
     */
    void setDoubleValue(Feature const & crFeature, double );

    /**
     * Get the value of feature <code>crFeature</code>, must be builtin type short.
     * @throws InvalidFSObjectException
     * @throws FeatureNotAppropriateException
     * @throws IncompatibleValueTypeException
     */
    double getDoubleValue(Feature const & crFeature) const;





    /** Return feature structure of type array stored at feature fArray.
    * The return value is guaranteed to be a properly terminated array.
    * Each element in the array is a byte.
    *
    * @param fArray  The feature referencing the array to return.
    *                fArray must be valid.
    *                fArray must be appropriate for this feature strucutre.
    *                fArray must be subsumed by type array.
    *
    * @throws InvalidFSObjectException
    * @throws InvalidFSFeatureObjectException
    * @throws FeatureNotAppropriateException
    * @throws FSIsNotArrayException
    */
    BooleanArrayFS getBooleanArrayFSValue( Feature const & fArray ) const;

    /** Return feature structure of type array stored at feature fArray.
     * The return value is guaranteed to be a properly terminated array.
     * Each element in the array is a byte.
     *
     * @param fArray  The feature referencing the array to return.
     *                fArray must be valid.
     *                fArray must be appropriate for this feature strucutre.
     *                fArray must be subsumed by type array.
     *
     * @throws InvalidFSObjectException
     * @throws InvalidFSFeatureObjectException
     * @throws FeatureNotAppropriateException
     * @throws FSIsNotArrayException
     */
    ByteArrayFS getByteArrayFSValue( Feature const & fArray ) const;


    /** Return feature structure of type array stored at feature fArray.
     * The return value is guaranteed to be a properly terminated array.
     * Each element in the array is a short.
     *
     * @param fArray  The feature referencing the array to return.
     *                fArray must be valid.
     *                fArray must be appropriate for this feature strucutre.
     *                fArray must be subsumed by type array.
     *
     * @throws InvalidFSObjectException
     * @throws InvalidFSFeatureObjectException
     * @throws FeatureNotAppropriateException
     * @throws FSIsNotArrayException
     */
    ShortArrayFS getShortArrayFSValue( Feature const & fArray ) const;

    /** Return feature structure of type array stored at feature fArray.
     * The return value is guaranteed to be a properly terminated array.
     * Each element in the array is a lomg (INT64).
     *
     * @param fArray  The feature referencing the array to return.
     *                fArray must be valid.
     *                fArray must be appropriate for this feature strucutre.
     *                fArray must be subsumed by type array.
     *
     * @throws InvalidFSObjectException
     * @throws InvalidFSFeatureObjectException
     * @throws FeatureNotAppropriateException
     * @throws FSIsNotArrayException
     */
    LongArrayFS getLongArrayFSValue( Feature const & fArray ) const;

    /** Return feature structure of type array stored at feature fArray.
     * The return value is guaranteed to be a properly terminated array.
     * Each element in the array is a double (INT64).
     *
     * @param fArray  The feature referencing the array to return.
     *                fArray must be valid.
     *                fArray must be appropriate for this feature strucutre.
     *                fArray must be subsumed by type array.
     *
     * @throws InvalidFSObjectException
     * @throws InvalidFSFeatureObjectException
     * @throws FeatureNotAppropriateException
     * @throws FSIsNotArrayException
     */
    DoubleArrayFS getDoubleArrayFSValue( Feature const & fArray ) const;

    bool operator==(FeatureStructure const & ) const;
    bool operator!=(FeatureStructure const & crFS) const;
    bool operator<(FeatureStructure const & crFS) const;
  };



} // namespace uima

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

namespace uima {
  inline bool FeatureStructure::operator!=(FeatureStructure const & crFS) const {
    return ! (*this == crFS);
  }
  inline void FeatureStructure::setStringValue(Feature const & crFeature, UChar const * cpuc, size_t uiLen) {
    setStringValue(crFeature, UnicodeStringRef(cpuc, uiLen));
  }
  inline void FeatureStructure::setStringValue(Feature const & crFeature, icu::UnicodeString const & crustr ) {
    setStringValue(crFeature, UnicodeStringRef(crustr.getBuffer(), crustr.length()));
  }
  inline void FeatureStructure::setStringValueExternal(Feature const & crFeature, UChar const * cpuc, size_t uiLen) {
    setStringValueExternal(crFeature, UnicodeStringRef(cpuc, uiLen));
  }
  inline void FeatureStructure::setStringValueExternal(Feature const & crFeature,  icu::UnicodeString const & crustr ) {
    setStringValueExternal(crFeature, UnicodeStringRef(crustr.getBuffer(), crustr.length()));
  }
}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */


#endif
/* <EOF> */


