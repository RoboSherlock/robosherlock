#ifndef UIMA_LISTFS_HPP
#define UIMA_LISTFS_HPP
/** \file listfs.hpp .
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

    \brief Contains the family of ListFS classes (IntListFS, FloatListFS etc.)

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp>

#include <uima/lowlevel_typedefs.hpp>
#include <uima/typesystem.hpp>
#include <uima/exceptions.hpp>
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
  namespace internal {
    class FSSystem;
  }
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  // exceptions
  UIMA_EXC_CLASSDECLARE(FSIsNotListException, CASException);
  UIMA_EXC_CLASSDECLARE(ListIsEmptyException, CASException);
  UIMA_EXC_CLASSDECLARE(ListIsCircularException, CASException);
} // namespace uima

/* ----------------------------------------------------------------------- */
/*       BasicListFS                                                            */
/* ----------------------------------------------------------------------- */

namespace uima {
  /// @if internal
  template< class T,
  const uima::lowlevel::TyFSType LIST_TYPE,
  const uima::lowlevel::TyFSType ELIST_TYPE,
  const uima::lowlevel::TyFSType NELIST_TYPE,
  const uima::lowlevel::TyFSFeature HEAD_FEATURE,
  const uima::lowlevel::TyFSFeature TAIL_FEATURE >
  /// @endif internal
  /**
   * A on object representing a list of elements from the CAS.
   * (e.g. they are of type empty list or of type non-empty list).
   * An element can be either a FeatureStructure or a float or an int or a
   * string. For each such element type there is a sub-class of BasicListFS
   * implementing this template interface with type specific methods.
   * ListFS, IntListFS, FloatListFS, StringListFS.
   *
   * All get methods may throw an <code>InvalidFSObjectException</code> if the
   * feature structure object is not valid.
   *
   * Creating a list can be done like this:
     @code
     // store elements of a vector of strings in a list feature
     FeatureStructure fsNewString;           // re-used for each new string
     StringListFS fsNewList = tcas.createListFS(); // create an empty list
     // build up a fs list from our vector
     for (size_t i = 0; i < vecStrings.size(); ++i) {
        // create a string feature structure
        fsNewList.addLast(vecStrings[i]);
     }
     // now that the list is complete, set the feature value to the list
     fsWithListFeature.setFSValue(fListFeature, fsNewList);
     @endcode
   * Accessing a list can be done like this:
     @code
     // check if we need to touch our element
     if (fsWithListFeature.hasListElements(fListFeature) ) {
        // get list
        StringListFS fsList = fsWithListFeature.getStringListFSValue(fListFeature);
        // iterate list
        while (!fsList.isEmpty()) {
           cout << fsList.getHead() << " "; // print the string
           fsList.moveToNext();
        }
     }
     @endcode
   *
   * The example uses StringListFS to show the usage of the interfaces with
   * strings. Usage of FeatureStructure, int or float values is analogous.
   *
   * @see ListFS
   * @see IntListFS
   * @see FloatListFS
   * @see StringListFS
   */
  class UIMA_LINK_IMPORTSPEC BasicListFS : public FeatureStructure {
    friend class CAS;
    friend class FeatureStructure;
  protected:
    /// @if internal
    void checkList(lowlevel::TyFS tyFS, TyMessageId tyContext) const;
    void checkNEList(lowlevel::TyFS tyFS, TyMessageId tyContext) const;
    void checkCircularity(lowlevel::TyFS tyFS1, lowlevel::TyFS tyFS2, TyMessageId tyContext) const;
    lowlevel::TyFS getLastListElement(lowlevel::TyFS tyListFS, size_t & rOutSize) const;
    lowlevel::TyFS addLastLowlevel(lowlevel::TyFS tyListFS, T tyNewElement);
    void appendLowlevel(lowlevel::TyFS tyListFS1, lowlevel::TyFS tyListFS2);

    BasicListFS(lowlevel::TyFS anFS, uima::CAS&, bool bDoChecks = true);
    /// @endif internal
  public:
    typedef T HeadType;

    /**
     * Default CTOR: Creates an invalid BasicListFS (use CAS::createListFS() instead)
     */
    BasicListFS();

    /**
     * Upgrade/Conversion CTOR: Creates an BasicListFS from an existing FeatureStructure.
     * fs must be of type List.
     *
     * @throws FSIsNotListException
     */
    explicit BasicListFS( FeatureStructure const & fs );

    /**
     * Return true if this element is not valid or of type empty list.
     * @throws InvalidFSObjectException
     */
    bool isEmpty() const;

    /**
     * Get the value of the head feature of this list element.
     * @throws InvalidFSObjectException
     * @throws ListIsEmptyException
     */
    T getHead() const;

    /**
     * Set the value of the head feature of this list element.
     * @throws InvalidFSObjectException
     * @throws ListIsEmptyException
     */
    void setHead( T const & fs );

    /**
     * Get the value of the tail feature of this list element.
     * @throws InvalidFSObjectException
     * @throws ListIsEmptyException
     */
    BasicListFS getTail() const;

    /**
     * Set the value of the tail feature of this list element.
     * @throws InvalidFSObjectException
     * @throws ListIsEmptyException
     */
    void setTail( BasicListFS fs );

    /**
     * Get the length of this list.
     * @throws InvalidFSObjectException
     */
    size_t getLength() const;

    /**
     * Adds the value fs to the begin of this list.
     * addFirst takes constant time with the length of this list
     *
     * @return  The list starting with the newly added element.
     *          (i.e. the first element of the full list)
     *
     * @throws InvalidFSObjectException
     */
    BasicListFS addFirst( T const & fs );

    /**
     * Adds the value fs to the end of this list.
     * addLast takes linear time with the length of this list
     *
     * @return  The list starting with the newly added element
     *          (i.e. the last element of the full list)
     *
     * @throws InvalidFSObjectException
     */
    BasicListFS addLast( T const & fs );

    /**
     * Appends the list l to the end of this list.
     * append takes linear time with the length of this list
     *
     * @return  The new list.
     *
     * @throws InvalidFSObjectException
     */
    BasicListFS append( BasicListFS l );

    /**
     * Appends the list l to the begin of this list.
     * prepend takes linear time with the length of list <code>l</code>
     *
     * @return  The new list.
     *
     * @throws InvalidFSObjectException
     */
    BasicListFS prepend( BasicListFS l );

    /**
     * Advance this list element to the next element in the list.
     * Equivalent to <code>this = this.getTailFS();</code>
     * Assumes that the list is not empty.
     *
     * @throws InvalidFSObjectException
     * @throws ListIsEmptyException
     */
    void moveToNext();

    /**
     * Removes the first occurrence of element from this list.
     *
     * @return  The singleton list containing the removed element.
     *
     * @throws InvalidFSObjectException
     */
    BasicListFS removeElement( T const & element);


    /// @if internal
    /**
     * Check if the list-value for the feature f on feature structure fs
     * has any elements.
     * Conceptually this is equivalent to
     * <code>!fs.getListFSValue(f).isEmpty()</code>
     * But unlike getListFSValue() hasListElements() will not touch the
     * feature value for f.
     *
     * @param fs  The feature structure on which to check the value for f
     * @param f   The feature to check (must be of type list)
     *
     * @throws InvalidFSObjectException
     * @throws InvalidFSFeatureObjectException
     * @throws FeatureNotAppropriateException
     * @throws FSIsNotListException
     */
    static bool hasListElements(FeatureStructure fs, Feature const & f);

    /**
     * Return feature structure of type list stored at feature fList.
     * The return value is guaranteed to be a properly terminated list.
     * If the feature value for f is untouched a properly terminated list
     * will be created on the fly.
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
    static BasicListFS getListFSValue(FeatureStructure const & fs, Feature const & fList);

    /**
     * create a feature structure of type empty list (list length is zero)
     * @param bIsPermanent indicate if the data should be permanent,
     *                     i.e., has a lifetime longer than the document
     */
    static BasicListFS createListFS( CAS & cas, bool bIsPermanent = false );

    /**
     * create a feature structure of type non-empty list (list length is 1)
     * @param fsHead        the feature structure to be made the head of the new list.
     *                      cas.createListFS(f).getHead() == f
     * @param bIsPermanent  indicate if the data should be permanent,
     *                      i.e., has a lifetime longer than the document
     */
    static BasicListFS createListFS( CAS & cas, T const & head, bool bIsPermanent = false );

    /// @endif internal
  }
  ; // class BasicListFS

  typedef BasicListFS< FeatureStructure, internal::gs_tyFSListType, internal::gs_tyEListType, internal::gs_tyNEListType, internal::gs_tyHeadFeature, internal::gs_tyTailFeature > BasicFSListFS;
  /**
   * A ListFS object implements a list of FeatureStructure objects
   * It is derived from the BasicListFS template interface so all
   * interesting member functions are derived from BasicListFS.
   * @see BasicListFS
   * @see IntListFS
   * @see FloatListFS
   * @see StringListFS
   */
  class UIMA_LINK_IMPORTSPEC ListFS : public BasicFSListFS {
    /// @if internal
    friend class CAS;
    friend class FeatureStructure;
  protected:
    ListFS(lowlevel::TyFS anFS, uima::CAS & cas, bool bDoChecks = true) :
        BasicFSListFS(anFS, cas, bDoChecks) {}
  public:
    /**
     * Default CTOR: Creates an invalid ListFS (use CAS::createListFS() instead)
     */
    ListFS() :
        BasicFSListFS() {}

    /**
     * Upgrade/Conversion CTOR: Creates an ListFS from an existing FeatureStructure.
     * fs must be of type List.
     *
     * @throws FSIsNotListException
     */
    explicit ListFS( FeatureStructure const & fs ) :
        BasicFSListFS(fs) {}

    /**
     * Upgrade/Conversion CTOR: Creates an ListFS from an existing BasicListFS.
     */
    ListFS( BasicFSListFS const & fs ) :
        BasicFSListFS(fs) {}
    /// @endif internal
  };


  typedef BasicListFS< float, internal::gs_tyFloatListType, internal::gs_tyEFloatListType, internal::gs_tyNEFloatListType, internal::gs_tyFloatHeadFeature, internal::gs_tyFloatTailFeature > BasicFloatListFS;
  /**
   * A FloatListFS object implements a list of float values.
   * It is derived from the BasicListFS template interface so all
   * interesting member functions are derived from BasicListFS.
   * @see BasicListFS
   * @see ListFS
   * @see IntListFS
   * @see StringListFS
   */
  class UIMA_LINK_IMPORTSPEC FloatListFS : public BasicFloatListFS {
    /// @if internal
    friend class CAS;
    friend class FeatureStructure;
  protected:
    FloatListFS(lowlevel::TyFS anFS, uima::CAS & cas, bool bDoChecks = true) :
        BasicFloatListFS(anFS, cas, bDoChecks) {}
  public:
    /**
     * Default CTOR: Creates an invalid ListFS (use CAS::createListFS() instead)
     */
    FloatListFS() :
        BasicFloatListFS() {}

    /**
     * Upgrade/Conversion CTOR: Creates an ListFS from an existing FeatureStructure.
     * fs must be of type List.
     *
     * @throws FSIsNotListException
     */
    explicit FloatListFS( FeatureStructure const & fs ) :
        BasicFloatListFS(fs) {}

    /**
     * Upgrade/Conversion CTOR: Creates an ListFS from an existing BasicListFS.
     */
    FloatListFS( BasicFloatListFS const & fs ) :
        BasicFloatListFS(fs) {}
    /// @endif internal
  };

  typedef BasicListFS< int, internal::gs_tyIntListType, internal::gs_tyEIntListType, internal::gs_tyNEIntListType, internal::gs_tyIntHeadFeature, internal::gs_tyIntTailFeature > BasicIntListFS;
  /**
   * A IntListFS object implements a list of int values.
   * It is derived from the BasicListFS template interface so all
   * interesting member functions are derived from BasicListFS.
   * @see BasicListFS
   * @see ListFS
   * @see FloatListFS
   * @see StringListFS
   */
  class UIMA_LINK_IMPORTSPEC IntListFS : public BasicIntListFS {
    /// @if internal
    friend class CAS;
    friend class FeatureStructure;
  protected:
    IntListFS(lowlevel::TyFS anFS, uima::CAS & cas, bool bDoChecks = true) :
        BasicIntListFS(anFS, cas, bDoChecks) {}
  public:
    /**
     * Default CTOR: Creates an invalid ListFS (use CAS::createListFS() instead)
     */
    IntListFS() :
        BasicIntListFS() {}

    /**
     * Upgrade/Conversion CTOR: Creates an ListFS from an existing FeatureStructure.
     * fs must be of type List.
     *
     * @throws FSIsNotListException
     */
    explicit IntListFS( FeatureStructure const & fs ) :
        BasicIntListFS(fs) {}

    /**
     * Upgrade/Conversion CTOR: Creates an ListFS from an existing BasicListFS.
     */
    IntListFS( BasicIntListFS const & fs ) :
        BasicIntListFS(fs) {}
    /// @endif internal
  };


  typedef BasicListFS< UnicodeStringRef, internal::gs_tyStringListType, internal::gs_tyEStringListType, internal::gs_tyNEStringListType, internal::gs_tyStringHeadFeature, internal::gs_tyStringTailFeature > BasicStringListFS;
  /**
   * A StringListFS object implements a list of string values.
   * It is derived from the BasicListFS template interface so all
   * interesting member functions are derived from BasicListFS.
   * @see BasicListFS
   * @see ListFS
   * @see IntListFS
   * @see FloatListFS
   */
  class UIMA_LINK_IMPORTSPEC StringListFS : public BasicStringListFS {
    /// @if internal
    friend class CAS;
    friend class FeatureStructure;
  protected:
    StringListFS(lowlevel::TyFS anFS, uima::CAS & cas, bool bDoChecks = true) :
        BasicStringListFS(anFS, cas, bDoChecks) {}
  public:
    /**
     * Default CTOR: Creates an invalid ListFS (use CAS::createListFS() instead)
     */
    StringListFS() :
        BasicStringListFS() {}

    /**
     * Upgrade/Conversion CTOR: Creates an ListFS from an existing FeatureStructure.
     * fs must be of type List.
     *
     * @throws FSIsNotListException
     */
    explicit StringListFS( FeatureStructure const & fs ) :
        BasicStringListFS(fs) {}

    /**
     * Upgrade/Conversion CTOR: Creates an ListFS from an existing BasicListFS.
     */
    StringListFS( BasicStringListFS const & fs ) :
        BasicStringListFS(fs) {}
    /// @endif internal
  };

} // namespace uima

#endif
/* <EOF> */


