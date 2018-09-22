#ifndef UIMA_ARRAYFS_HPP
#define UIMA_ARRAYFS_HPP
/** \file arrayfs.hpp .
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

    \brief Declares all ArrayFS classes (IntArrayFS, FloatArrayFS etc)

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
#include <uima/featurestructure.hpp>
#include <uima/internal_typeshortcuts.hpp>

#include "unicode/utf.h"
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */


namespace uima {
  class CAS;
  namespace lowlevel {
    class FSHeap;
  }
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  // exceptions
  UIMA_EXC_CLASSDECLARE(FSIsNotArrayException, CASException);
  UIMA_EXC_CLASSDECLARE(FSArrayOutOfBoundsException, CASException);
} // namespace uima

/* ----------------------------------------------------------------------- */
/*       ArrayFS                                                            */
/* ----------------------------------------------------------------------- */

namespace uima {
  /// @if internal
  template< class T, const uima::lowlevel::TyFSType ARRAY_TYPE >
  /// @endif internal
  /**
   * A on object representing an array of CAS elements.
   * An element can be either a FeatureStructure or a float or an int or a
   * string. For each such element type there is a sub-class of BasicListFS
   * implementing this template interface with type specific methods:
   * ArrayFS, IntArrayFS, FloatArrayFS, StringArrayFS.
   *
   * All get methods may throw an <code>InvalidFSObjectException</code> if the
   * feature structure object is not valid.
   * All access methods using array indexes may throw an
   * <code>FSArrayOutOfBoundsException</code> if the index is >= the array size.
   *
   * Creating an array can be done like this:
     @code
     // store elements of a vector of strings in an array feature
     FeatureStructure fsNewString;           // re-used for each new string
     const size_t uiARRAY_SIZE = vecStrings.size();
     StringArrayFS fsNewArray = tcas.createStringArrayFS(uiARRAY_SIZE); // create an array
     // build up a fs array from our vector
     for (size_t i = 0; i < vecStrings.size(); ++i) {
        fsNewArray.set(i, vecStrings[i]);
     }
     // now that the array is complete, set the feature value to the array
     fsWithArrayFeature.setFSValue(fArrayFeature, fsNewArray);
     @endcode
   * Accessing a array can be done like this:
     @code
     // get array
     StringArrayFS fsArray = fsWithArrayFeature.getStringArrayFSValue(fArrayFeature);
     // iterate array
     for (size_t i = 0; i < fsArray.size(); ++i) {
        cout << fsArray.get(i) << " ";
     }
     @endcode
   *
   * The example uses StringArrayFS to show the usage of the interfaces with
   * strings. Usage of FeatureStructure, int or float values is analogous.
   *
   * Arrays can not be resized after creation. If more (or less) elements are
   * required in the array a new array must be created.
   * Elements from the old array may be copied to then new array.
   * Finally the new array must be assigned to the array feature holding the
   * old array.
   *
   * @see ArrayFS
   * @see IntArrayFS
   * @see FloatArrayFS
   * @see StringArrayFS
   */
  class UIMA_LINK_IMPORTSPEC BasicArrayFS  : public FeatureStructure {
    /// @if internal
    friend class CAS;
    friend class FeatureStructure;
  protected:
    BasicArrayFS(lowlevel::TyFS anFS, uima::CAS&, bool bDoChecks = true);
    /// @endif internal
  public:

    /**
     * Default CTOR: Creates an invalid BasicArrayFS (use CAS::createArrayFS() instead)
     */
    BasicArrayFS();

    /**
     * Upgrade/Conversion CTOR: Creates an ArrayFS from an existing FeatureStructure.
     * fs must be of type array.
     *
     * @throws FSIsNotArrayException
     */
    explicit BasicArrayFS( FeatureStructure const & fs );

    /**
     * get the <code>n</code>th element of an array.
     * @throws InvalidFSObjectException
     * @throws FSArrayOutOfBoundsException
     */
    T get(size_t n) const;

    /**
     * set the <code>n</code>th element of an array.
     * @throws InvalidFSObjectException
     * @throws FSArrayOutOfBoundsException
     */
    void set(size_t n, T const & val);

    /**
     * get the size of the array
     * @throws InvalidFSObjectException
     */
    size_t size() const;

    /**
     * Copy the contents of the array from <code>start</code> to <code>end</code>
     * to the destination <code>destArray</code> with destination offset
     * <code>destOffset</code>.
     *
     * @param uiStart The index of the first element to copy.
     * @param numelements number of element, the index after the last element to copy.
     * @param destArray The array to copy to.
     * @param uiDestOffset Where to start copying into <code>destArray</code>.
     *
     * @throws InvalidFSObjectException
     * @throws FSArrayOutOfBoundsException If <code>start &lt; 0</code>
     * or <code>end > size()</code> or
     * <code>destOffset + (end - start) > destArray.length</code>.
     */
    void copyToArray(
      size_t uiStart,
      size_t uiEnd,
      T* destArray,
      size_t uiDestOffset) const;

    void copyFromArray(
      T const * sourceArray,
      size_t uiStart,
      size_t uiEnd,
      size_t uiOffset);

 /**
   * Copy the contents of the array from <code>start</code> for <code>numelements</code> to the
   * destination <code>destArray</code> with destination offset <code>destOffset</code>.
   * 
   * @param srcOffset	  The index of the first element to copy.
   * @param dest          The array to copy to.
   * @param destOffset    Where to start copying into.
   * @param numelements   The number of elements to copy.
   * @throws InvalidFSObjectException
   * @throws FSArrayOutOfBoundsException
   *        If <code>srcOffset &lt; 0</code> or <code>length > size()</code> or
   *        <code>destOffset + length > destArray.length</code>.
   */
  void copyToArray(size_t srcOffset, T * destArray, size_t destOffset, size_t numelements) const ;
      

 /**
   * Copy the contents of an external array into this array.
   * 
   * @param src          The source array.
   * @param srcOffset    Where to start copying in the source array.
   * @param destOffset   Where to start copying to in the destination array.
   * @param length       The number of elements to copy.
   * @throws InvalidFSObjectException
   * @throws FSArrayOutOfBoundsException If <code>srcOffset or destOffset &lt; 0</code>
   * or <code>length > size()</code> or
   * <code>destOffset + length > size()</code>.
   *
   * NOTE: replace copyFromArray with this API to be consistent with the Java API.
   */
   
//  void copyFromArray(T const * src, size_t srcOffset, size_t destOffset, size_t length) ;
       
    /// @if internal
    /**
     * create a feature structure of type empty list (list length is zero)
     * @param bIsPermanent indicate if the data should be permanent,
     *                     i.e., has a lifetime longer than the document
     */
    static BasicArrayFS createArrayFS( CAS & cas, size_t uiSize, bool bIsPermanent = false );
    /// @endif internal

  }
  ; // class ArrayFS

  typedef BasicArrayFS< FeatureStructure, internal::gs_tyFSArrayType > BasicFSArrayFS;
  /**
   * A ArrayFS object implements an array of FeatureStructure values.
   * It is derived from the BasicArrayFS template interface so all
   * interesting member functions are derived from BasicArrayFS.
   * @see BasicArrayFS
   * @see IntArrayFS
   * @see StringArrayFS
   * @see FloatArrayFS
   */
  class UIMA_LINK_IMPORTSPEC ArrayFS : public BasicFSArrayFS {
    /// @if internal
    friend class CAS;
    friend class FeatureStructure;
  protected:
    ArrayFS(lowlevel::TyFS anFS, uima::CAS& cas, bool bDoChecks = true) :
        BasicFSArrayFS(anFS, cas, bDoChecks) {}
  public:
    ArrayFS() :
        BasicFSArrayFS() {}

    explicit ArrayFS( FeatureStructure const & fs ) :
        BasicFSArrayFS(fs) {}

    ArrayFS( BasicFSArrayFS const & fs ) :
        BasicFSArrayFS(fs) {}
    /// @endif internal
  };

  typedef BasicArrayFS< float, internal::gs_tyFloatArrayType > BasicFloatArrayFS;
  /**
   * A FloatArrayFS object implements an array of float values.
   * It is derived from the BasicArrayFS template interface so all
   * interesting member functions are derived from BasicArrayFS.
   * @see BasicArrayFS
   * @see ArrayFS
   * @see IntArrayFS
   * @see StringArrayFS
   */
  class UIMA_LINK_IMPORTSPEC FloatArrayFS : public BasicFloatArrayFS {
    /// @if internal
    friend class CAS;
    friend class FeatureStructure;
  protected:
    FloatArrayFS(lowlevel::TyFS anFS, uima::CAS & cas, bool bDoChecks = true) :
        BasicFloatArrayFS(anFS, cas, bDoChecks) {}
  public:
    FloatArrayFS() :
        BasicFloatArrayFS() {}

    explicit FloatArrayFS( FeatureStructure const & fs ) :
        BasicFloatArrayFS(fs) {}

    FloatArrayFS( BasicFloatArrayFS const & fs ) :
        BasicFloatArrayFS(fs) {}
    /// @endif internal
  };

  typedef BasicArrayFS< int, internal::gs_tyIntArrayType > BasicIntArrayFS;
  /**
   * A IntArrayFS object implements an array of int values.
   * It is derived from the BasicArrayFS template interface so all
   * interesting member functions are derived from BasicArrayFS.
   * @see BasicArrayFS
   * @see ArrayFS
   * @see StringArrayFS
   * @see FloatArrayFS
   */
  class UIMA_LINK_IMPORTSPEC IntArrayFS : public BasicIntArrayFS {
    /// @if internal
    friend class CAS;
    friend class FeatureStructure;
  protected:
    IntArrayFS(lowlevel::TyFS anFS, uima::CAS & cas, bool bDoChecks = true) :
        BasicIntArrayFS(anFS, cas, bDoChecks) {}
  public:
    IntArrayFS() :
        BasicIntArrayFS() {}

    explicit IntArrayFS( FeatureStructure const & fs ) :
        BasicIntArrayFS(fs) {}

    IntArrayFS( BasicIntArrayFS const & fs ) :
        BasicIntArrayFS(fs) {}
    /// @endif internal
  };

  typedef BasicArrayFS< UnicodeStringRef, internal::gs_tyStringArrayType > BasicStringArrayFS;
  /**
   * A StringArrayFS object implements an array of string values.
   * It is derived from the BasicArrayFS template interface so all
   * interesting member functions are derived from BasicArrayFS.
   * @see BasicArrayFS
   * @see ArrayFS
   * @see IntArrayFS
   * @see FloatArrayFS
   */
  class UIMA_LINK_IMPORTSPEC StringArrayFS : public BasicStringArrayFS {
    /// @if internal
    friend class CAS;
    friend class FeatureStructure;
  protected:
    StringArrayFS(lowlevel::TyFS anFS, uima::CAS & cas, bool bDoChecks = true) :
        BasicStringArrayFS(anFS, cas, bDoChecks) {}
  public:
    StringArrayFS() :
        BasicStringArrayFS() {}

    explicit StringArrayFS( FeatureStructure const & fs ) :
        BasicStringArrayFS(fs) {}

    StringArrayFS( BasicStringArrayFS const & fs ) :
        BasicStringArrayFS(fs) {}
    /// @endif internal
  };

  typedef BasicArrayFS< bool, internal::gs_tyBooleanArrayType > BasicBooleanArrayFS;
  /**
   * A ByteArrayFS object implements an array of byte values.
   * It is derived from the BasicArrayFS template interface so all
   * interesting member functions are derived from BasicArrayFS.
   * @see BasicArrayFS
   * @see ArrayFS
   * @see StringArrayFS
   * @see FloatArrayFS
   */
  class UIMA_LINK_IMPORTSPEC BooleanArrayFS : public BasicBooleanArrayFS {
    /// @if internal
    friend class CAS;
    friend class FeatureStructure;
  protected:
    BooleanArrayFS(lowlevel::TyFS anFS, uima::CAS & cas, bool bDoChecks = true) :
        BasicBooleanArrayFS(anFS, cas, bDoChecks) {}
  public:
    BooleanArrayFS() :
        BasicBooleanArrayFS() {}

    explicit BooleanArrayFS( FeatureStructure const & fs ) :
        BasicBooleanArrayFS(fs) {}

    BooleanArrayFS( BasicBooleanArrayFS const & fs ) :
        BasicBooleanArrayFS(fs) {}
    /// @endif internal
  };



  typedef BasicArrayFS< char, internal::gs_tyByteArrayType > BasicByteArrayFS;
  /**
   * A ByteArrayFS object implements an array of byte values.
   * It is derived from the BasicArrayFS template interface so all
   * interesting member functions are derived from BasicArrayFS.
   * @see BasicArrayFS
   * @see ArrayFS
   * @see StringArrayFS
   * @see FloatArrayFS
   */
  class UIMA_LINK_IMPORTSPEC ByteArrayFS : public BasicByteArrayFS {
    /// @if internal
    friend class CAS;
    friend class FeatureStructure;
  protected:
    ByteArrayFS(lowlevel::TyFS anFS, uima::CAS & cas, bool bDoChecks = true) :
        BasicByteArrayFS(anFS, cas, bDoChecks) {}
  public:
    ByteArrayFS() :
        BasicByteArrayFS() {}

    explicit ByteArrayFS( FeatureStructure const & fs ) :
        BasicByteArrayFS(fs) {}

    ByteArrayFS( BasicByteArrayFS const & fs ) :
        BasicByteArrayFS(fs) {}
    /// @endif internal
  };

  typedef BasicArrayFS<short, internal::gs_tyShortArrayType > BasicShortArrayFS;
  /**
   * A ShortArrayFS object implements an array of short values.
   * It is derived from the BasicArrayFS template interface so all
   * interesting member functions are derived from BasicArrayFS.
   * @see BasicArrayFS
   * @see ArrayFS
   * @see StringArrayFS
   * @see FloatArrayFS
   */
  class UIMA_LINK_IMPORTSPEC ShortArrayFS : public BasicShortArrayFS {
    /// @if internal
    friend class CAS;
    friend class FeatureStructure;
  protected:
    ShortArrayFS(lowlevel::TyFS anFS, uima::CAS & cas, bool bDoChecks = true) :
        BasicShortArrayFS(anFS, cas, bDoChecks) {}
  public:
    ShortArrayFS() :
        BasicShortArrayFS() {}

    explicit ShortArrayFS( FeatureStructure const & fs ) :
        BasicShortArrayFS(fs) {}

    ShortArrayFS( BasicShortArrayFS const & fs ) :
        BasicShortArrayFS(fs) {}
    /// @endif internal
  };

  typedef BasicArrayFS< INT64, internal::gs_tyLongArrayType > BasicLongArrayFS;
  /**
   * A ShortArrayFS object implements an array of short values.
   * It is derived from the BasicArrayFS template interface so all
   * interesting member functions are derived from BasicArrayFS.
   * @see BasicArrayFS
   * @see ArrayFS
   * @see StringArrayFS
   * @see FloatArrayFS
   */
  class UIMA_LINK_IMPORTSPEC LongArrayFS : public BasicLongArrayFS {
    /// @if internal
    friend class CAS;
    friend class FeatureStructure;
  protected:
    LongArrayFS(lowlevel::TyFS anFS, uima::CAS & cas, bool bDoChecks = true) :
        BasicLongArrayFS(anFS, cas, bDoChecks) {}
  public:
    LongArrayFS() :
        BasicLongArrayFS() {}

    explicit LongArrayFS( FeatureStructure const & fs ) :
        BasicLongArrayFS(fs) {}

    LongArrayFS( BasicLongArrayFS const & fs ) :
        BasicLongArrayFS(fs) {}
    /// @endif internal
  };


  typedef BasicArrayFS<double, internal::gs_tyDoubleArrayType > BasicDoubleArrayFS;
  /**
   * A StringArrayFS object implements an array of string values.
   * It is derived from the BasicArrayFS template interface so all
   * interesting member functions are derived from BasicArrayFS.
   * @see BasicArrayFS
   * @see ArrayFS
   * @see IntArrayFS
   * @see FloatArrayFS
   */
  class UIMA_LINK_IMPORTSPEC DoubleArrayFS : public BasicDoubleArrayFS {
    /// @if internal
    friend class CAS;
    friend class FeatureStructure;
  protected:
    DoubleArrayFS(lowlevel::TyFS anFS, uima::CAS & cas, bool bDoChecks = true) :
        BasicDoubleArrayFS(anFS, cas, bDoChecks) {}
  public:
    DoubleArrayFS() :
        BasicDoubleArrayFS() {}

    explicit DoubleArrayFS( FeatureStructure const & fs ) :
        BasicDoubleArrayFS(fs) {}

    DoubleArrayFS( BasicDoubleArrayFS const & fs ) :
        BasicDoubleArrayFS(fs) {}
    /// @endif internal
  };



} // namespace uima

#endif
/* <EOF> */


