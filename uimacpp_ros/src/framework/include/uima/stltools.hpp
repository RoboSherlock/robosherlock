#ifndef UIMA_STLTOOLS_HPP
#define UIMA_STLTOOLS_HPP
/** \file stltools.hpp .
-------------------------------------------------------------------------------


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

-------------------------------------------------------------------------------

    \brief  Contains STL utility functions

-------------------------------------------------------------------------------
*/

#include <uima/pragmas.hpp>
#include <uima/configure.h>
#include <uima/types.h>


//lint -e1905 : implicit default constructor generated (too often used in STL)

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */


#include <iostream>
#include <vector>
#include <algorithm>
#include <map>
#include <string>
#include <utility>
#if defined( UIMA_NO_HASH_CONTAINERS_SUPPORTED ) || defined( UIMA_DONT_USE_HASH_CONTAINERS )
// we include the appropriate headers
#  include <set>
#  include <map>
#else
#  if defined(__GNUC__)
#define GCC_VERSION (__GNUC__ * 10000 \
                     + __GNUC_MINOR__ * 100 \
                     + __GNUC_PATCHLEVEL__)
#     if (__GNUC__ >= 3)
#        include <ext/hash_set>
#        include <ext/hash_map>
#        if (__GNUC_MINOR__ > 0)
#        endif
#     else   // gcc 2.9
#        include <hash_set>
#        include <hash_map>
#     endif
#  elif defined(__INTEL_COMPILER)
#     define _HAS_TRADITIONAL_STL 1
#     include <hash_set>
#     include <hash_map>
#  else
#     include <hashset>
#     include <hashmap>
#  endif
#endif

namespace uima {

  /* ----------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------- */

  /**@name Hash container defines
     Defines for hash container which may not be supported by some
     compilers as they are not ANSI.

     Depending on the define <TT>UIMA_NO_HASH_CONTAINERS_SUPPORTED</TT>,
     <TT>taf_stltools.hpp</TT> includes the appropriate headers
     and establishes defines which can be used for hash and non-hash containers
     (hash_set <=> set and hash_map <=> map).
     The define ignores the arguments neccesary for hash
     containers, but unnecessary for non-hash containers (hash-function).
     Note: the arguments to the macros must be single tokens, so you cannot write the following:

     \code
     typedef UIMA_TY_HASH_SET( ULString,
                              StringHashing1< ULString, icu::UChar >)
             TyTokenStringSet;
     \endcode

     but instead write:

     \code
     typedef StringHashing1< ULString, icu::UChar > TyULStringHashFunc;
     typedef UIMA_TY_HASH_SET( ULString,
                              TyULStringHashFunc)
             TyTokenStringSet;
     \endcode

     See the include file <TT>taf_strhashfuncts.hpp</TT> for some useful string hash
     functions.
  */
//@{

// Depending on this define...
#if defined( UIMA_NO_HASH_CONTAINERS_SUPPORTED ) || defined( UIMA_DONT_USE_HASH_CONTAINERS )
/// Define for a hash_set/set depending on <TT>UIMA_NO_HASH_CONTAINERS_SUPPORTED</TT>
#  define UIMA_TY_HASH_SET( ClKeyType,  ClElementType, ClHashFunction) \
            set< ClElementType, less< ClElementType > >
/// Define for a hash_multiset/multiset depending on <TT>UIMA_NO_HASH_CONTAINERS_SUPPORTED</TT>
#  define UIMA_TY_HASH_MULTISET( ClKeyType,  ClElementType, ClHashFunction) \
            multiset< ClElementType, less< ClElementType > >
/// Define for a hash_map/map depending on <TT>UIMA_NO_HASH_CONTAINERS_SUPPORTED</TT>
#  define UIMA_TY_HASH_MAP( ClKeyType,  ClElementType, ClHashFunction) \
            map< ClKeyType, ClElementType, less< ClKeyType > >
/// Define for a hash_multimap/multimap depending on <TT>UIMA_NO_HASH_CONTAINERS_SUPPORTED</TT>
#  define UIMA_TY_HASH_MULTIMAP( ClKeyType,  ClElementType, ClHashFunction) \
            multimap< ClKeyType, ClElementType, less< ClKeyType > >
#else
#  define UIMA_TY_HASH_SET( ClElementType, ClHashFunction) \
            hash_set< ClElementType, ClHashFunction, equal_to< ClElementType > >
#  define UIMA_TY_HASH_MULTISET( ClElementType, ClHashFunction) \
            hash_multiset< ClElementType, ClHashFunction, equal_to< ClElementType > >
#  define UIMA_TY_HASH_MAP( ClKeyType,  ClElementType, ClHashFunction) \
            hash_map< ClKeyType, ClElementType, ClHashFunction, equal_to< ClKeyType > >
#  define UIMA_TY_HASH_MULTIMAP( ClKeyType,  ClElementType, ClHashFunction) \
            hash_multimap< ClKeyType, ClElementType, ClHashFunction, equal_to< ClKeyType > >
#endif
//@}

  /* ----------------------------------------------------------------------- */
  /*      Types / Classes                                                    */
  /* ----------------------------------------------------------------------- */

  /**
     Define to iterate all elements of an STL container.
  */
#define STL_FOR_EACH( it, container ) \
      for ( it = container.begin(); it != container.end(); ++it )


  /**
     Define to iterate in parallel all elements of two STL containers.
  */
#define STL_FOR_EACH2( it1, it2, container1, container2 ) \
      for ( it1 = container1.begin(), it2 = container2.begin(); it1 != container1.end() && it2 != container2.end(); ++it1 , ++it2 )

  /**
   Defines a shorthand for "named" pairs, where in addition to accessing the
   parts of the pair as first and second accessor functions, more
   descriptive names are defined.
   For example:
   STL_NAMED_PAIR(KcRulePair, KcTestCategory, category, KcTestConditions, conditions);
   This defines a class KcRulePair as a pair of KcTestCategory and KcTestConditions with accessor
   function category() and conditions()
  */
#define STL_NAMED_PAIR(name, TypeFirst, nameFirst, TypeSecond, nameSecond) \
      class UIMA_LINK_IMPORTSPEC name : public pair<TypeFirst, TypeSecond> { \
         public:  \
         name(const TypeFirst& thefirst = TypeFirst(), const TypeSecond& thesecond = TypeSecond()) : \
            pair<TypeFirst, TypeSecond>(thefirst, thesecond) {} \
         TypeFirst&  nameFirst() {return first;} \
         TypeSecond& nameSecond() {return second;} \
         const TypeFirst&  nameFirst() const {return first;} \
         const TypeSecond& nameSecond() const {return second;} \
      }

  /**
     Define to return the address of an object avaiable via an STL iterator.
  */
#define STL_ITER2PTR(iterator) \
      (&(*iterator))


  /**
     Define to search through a complete STL container.
  */
#define STL_FIND(container, element) \
   find(container.begin(), container.end(), element)

  /**
     Define to search through a complete STL container with predicate.
  */
#define STL_FIND_IF(container, predicate) \
   find_if(container.begin(), container.end(), predicate)

  /**
     Function for a binary search through a range in a < sorted STL container.
     In this case, use the lower_bound to do the actual binary_search.
  */
  template < class FwdIt, class Value >
  FwdIt
  taf_stl_find_sorted( FwdIt itFirst, FwdIt itLast, const Value & crclValue) {
    FwdIt it = lower_bound( itFirst, itLast, crclValue );
    // Since lower_bound always return a value, we must check if we hit a matching one
    if (it != itLast && !( crclValue < *it )) { // don't use == only <
      return it;
    }
    return itLast;
  }

  /**
     Function for a binary search through a range in a Compare sorted STL container.
     In this case, use the lower_bound to do the actual binary_search.
  */
  template < class FwdIt, class Value, class Compare >
  FwdIt
  taf_stl_find_sorted( FwdIt itFirst, FwdIt itLast, const Value & crclValue, Compare clCompare) {
    FwdIt it = lower_bound( itFirst, itLast, crclValue, clCompare );
    // Since lower_bound always return a value, we must check if we hit a matching one
    if (it != itLast && !clCompare( crclValue , *it )) { // don't use == only clCompare
      return it;
    }
    return itLast;
  }  //lint !e1746: parameter 'clCompare' could be made const reference

  /**
     Define to binary search through a complete sorted STL container.
  */
#define STL_FIND_SORTED(it, container, element) \
   taf_stl_find_sorted(container.begin(), container.end(), element)

  /**
     Define to search through a STL container,
     which is completely sorted by a given compare function.
  */
#define STL_FIND_SORTED_COMPARE(container, element, compare) \
   taf_stl_find_sorted(container.begin(), container.end(), element, compare)



  /**
    STL tool function
  */
  template <class T1, class T2>
  struct first_equal_to : public std::binary_function<T1, T2, bool> {
    bool operator()(const T1& x, const T2& y) const {
      return x.first == y;
    }
  };

  /**
    STL tool function
  */
  template <class T1, class T2>
  struct second_equal_to : public std::binary_function<T1, T2, bool> {
    bool operator()(const T1& x, const T2& y) const {
      return x.second == y;
    }
  };

  /**
    STL tool function
  */
  template <class T1, class T2>
  struct it_first_equal_to : public std::binary_function<T1, T2, bool> {
    bool operator()(const T1& x, const T2& y) const {
      return (*x).first == y;
    }
  };

  /**
    STL tool function
  */
  template <class T1, class T2>
  struct it_second_equal_to : public std::binary_function<T1, T2, bool> {
    bool operator()(const T1& x, const T2& y) const {
      return (*x).second == y;
    }
  };

  /**
    STL tool function
  */
  template <class T>
  struct first_less : public std::binary_function<T, T, bool> {
    bool operator()(const T& x, const T& y) const {
      return x.first < y.first;
    }
  };

  /**
    STL tool function
  */
  template <class T>
  struct second_less : public std::binary_function<T, T, bool> {
    bool operator()(const T& x, const T& y) const {
      return x.second < y.second;
    }
  };

  /**
    STL tool function
  */
  template <class T>
  struct it_first_less : public std::binary_function<T, T, bool> {
    bool operator()(const T x, const T y) const {
      return (*x).first < (*y).first;
    }
  };

  /**
    STL tool function
  */
  template <class T>
  struct it_second_less : public std::binary_function<T, T, bool> {
    bool operator()(const T x, const T y) const {
      return (*x).second < (*y).second;
    }
  };

  /**
    STL tool function
  */
  template <class T>
  struct first_greater : public std::binary_function<T, T, bool> {
    bool operator()(const T& x, const T& y) const {
      return x.first > y.first;
    }
  };

  /**
    STL tool function
  */
  template <class T>
  struct second_greater : public std::binary_function<T, T, bool> {
    bool operator()(const T& x, const T& y) const {
      return x.second > y.second;
    }
  };

  /**
    STL tool function
  */
  template <class T>
  struct it_first_greater : public std::binary_function<T, T, bool> {
    bool operator()(const T x, const T y) const {
      return (*x).first > (*y).first;
    }
  };

  /**
    STL tool function
  */
  template <class T>
  struct it_second_greater : public std::binary_function<T, T, bool> {
    bool operator()(const T x, const T y) const {
      return (*x).second > (*y).second;
    }
  };


  /**
    STL tool class
  */
  template <class Element>
  struct CountContainer : public std::map<Element, size_t, std::less< Element > > {
public:
//   typedef vector<value_type*> SortedByCountContainer;
    typedef std::map<Element, size_t, std::less< Element > > ParentType;

#if defined(__xlC__)
    // this is redundant information, but xlC5 needs it
    typedef typename ParentType::iterator iterator;
#endif

    typedef std::vector<typename ParentType::const_iterator> SortedByCountContainer;
private:
    SortedByCountContainer m_sortByCountMap;
    bool   m_sortMapDirty;
    long   m_sum;
    bool   m_sumDirty;

public:
    //add an element with count 0 to the count list if not already there
    //(if element is already there, it's count is unchanged)
    bool create(const Element& element) {
      std::pair<typename CountContainer::iterator, bool> pairItBool;
      pairItBool = insert(typename CountContainer::value_type(element, 0));
      if (pairItBool.second) { //if not yet there
        m_sortByCountMap.push_back(pairItBool.first);
      }
      m_sortMapDirty = true;
      m_sumDirty     = true;
      return pairItBool.second;
    }
    //increase the count of element by 1
    //(if element is not there already, it is created with a count of 1)
    bool increase_count(const Element& element) {
      std::pair<typename CountContainer::iterator, bool> pairItBool;
      pairItBool = insert(typename CountContainer::value_type(element, 1));
      if (pairItBool.second) { //if not yet there
        m_sortByCountMap.push_back(pairItBool.first);
      } else {
        ++(*pairItBool.first).second;
      }
      m_sortMapDirty = true;
      m_sumDirty     = true;
      return pairItBool.second;
    }
    void eraseAll() {
      (*this).clear();
      m_sortByCountMap.clear();
    }
    size_t getCount(const Element& element) const {
      typename CountContainer::const_iterator it(find(element));
      return (it == this->end() ? 0 : getCount(*it));
    }
    size_t getCount(typename std::map<Element, size_t, std::less< Element > >::const_iterator it) const {
      return (*it).second;
    }
    size_t getCount(const typename std::map<Element, size_t, std::less< Element > >::value_type&  value) const {
      return value.second;
    }
    float getPercentage(const Element& element) {
      typename CountContainer::const_iterator it(find(element));
      return (it == this->end() ? 0.0 : getPercentage(*it));
    }
    long getSum() {
      if (m_sumDirty) {
        m_sum = 0;
        typename CountContainer::const_iterator it;
        STL_FOR_EACH(it, (*this)) {
          m_sum += getCount(it);
        }
      }
      return m_sum;
    }
    float getPercentage(typename std::map<Element, size_t, std::less< Element > >::const_iterator it)  {
      return (getSum() == 0 ? (float)0 : (float)(*it).second / (float)getSum()) * 100.0;
    }
    float getPercentage(const typename std::map<Element, size_t, std::less< Element > >::value_type& value) {
      return (getSum() == 0 ? (float)0 : (float)value.second / (float)getSum()) * 100.0;
    }
    const Element& getElement(typename std::map<Element, size_t, std::less< Element > >::const_iterator it) const {
      return (*it).first;
    }
    const SortedByCountContainer& getSortedByCountContainer() {
      if (m_sortMapDirty) {
        sort(m_sortByCountMap.begin(), m_sortByCountMap.end(), it_second_greater<typename SortedByCountContainer::value_type>());
      }
      return m_sortByCountMap;
    }
  }
  ;  //lint !e1509: base class destructor for class 'map' is not virtual

  /**
   * STL Tool class with semantics of "less"
   * applied to elements which are pointers and require dereferencing
   */
  template< class T >
  class UIMA_LINK_IMPORTSPEC less_derefptr : public std::binary_function< T, T, bool > {
  public:
    bool operator()( const T& x, const T& y ) const {
      return (bool)((*x) < (*y));
    }
  }
  ;  //lint !e1905: implicit default constructor generated for class 'less_derefptr'


  /**
    Prints all elements in a STL container to stream.
  */
  template <class Container>
  inline void
  taf_STLStreamOut(
    const Container &        rclContainer,
    std::ostream &           rclOut,
    const char*              cpszDelimiter
  ) {
//?    ostream_iterator<typename Container::value_type> itOut(rclOut, cpszDelimiter);
    std::ostream_iterator<
// typename must not be used for some GNU versions
#if ( ((GCC_VERSION < 30000) || (GCC_VERSION > 30203)) || defined(__xlC__) )
    typename
#endif
    Container::value_type > itOut(rclOut, cpszDelimiter);
    copy(rclContainer.begin(), rclContainer.end(), itOut);
  }

  /**
    Prints all elements in an associative STL container to a stream.
  */
  template <class Container>
  inline void
  taf_STLStreamOutAssoc(
    const Container &        rclContainer,
    std::ostream &           rclOut,
    const char*              cpszDelimiter,
    const char*              cpszMapString
  ) {
    typename Container::const_iterator itOut;
    STL_FOR_EACH(itOut, rclContainer) {  //lint !e1703 !e1023: Call operator!=is ambiguous
      rclOut << (*itOut).first << cpszMapString << (*itOut).second << cpszDelimiter;
    }
  }

  /**
    Prints elements in a range in a STL container to stream.
    Note: this does not work on HP CFRONT 10.36
  */
  template <class Container>
  inline void
  taf_STLStreamOut(
    const Container &         rclContainer,
    typename Container::const_iterator itBeg,
    typename Container::const_iterator itEnd,
    std::ostream&             rclOut,
    const char*               cpszDelimiter
  ) {
    std::ostream_iterator<
// typename must not be used for some GNU versions
#if ( ((GCC_VERSION < 30000) || (GCC_VERSION > 30203)) || defined(__xlC__) )
    typename
#endif

    Container::value_type> itOut(rclOut, cpszDelimiter);
    copy(itBeg, itEnd, itOut);
  }

  /**
     Create a pair with none of the elements being constant.
  */
  template <class T1, class T2>
  inline std::pair<T1, T2> make_non_const_pair(T1& x, T2& y) {
    return std::pair<T1, T2>(x, y);
  }

  /**
     Creates a pair with the first element being constant.
  */
  template <class T1, class T2>
  inline std::pair<const T1, T2> make_const_pair(const T1& x, T2& y) {
    return std::pair<const T1, T2>(x, y);
  }

  /**
     Creates a pair with the both elements being constant.
  */
  template <class T1, class T2>
  inline std::pair<const T1, T2> make_double_const_pair(const T1& x, const T2& y) {
    return std::pair<const T1, const T2>(x, y);
  }


  /**
     A simple implementation of a triple. Usage analogous to pair.
  */
  template<class A, class B, class C>
  class UIMA_LINK_IMPORTSPEC triple {
  public:
    A first;  //lint !e1925: Symbol 'first' is a public data member
    B second; //lint !e1925: Symbol 'second' is a public data member
    C third;  //lint !e1925: Symbol 'third' is a public data member

    triple() { }

    ~triple() { }

    triple(const A& a, const B& b, const C& c) {
      first = a;
      second = b;
      third = c;
    }

    triple(const triple<A, B, C>& t) {
      first = t.first;
      second = t.second;
      third = t.third;
    }

    triple<A,B,C>& operator=(const triple<A,B,C>& t) {
      first = t.first;
      second = t.second;
      third = t.third;
      return *this;
    }

    bool operator==(const triple<A, B, C>& t) const {
      return ( (first == t.first)
               && (second == t.second)
               && (third == t.third) );
    }

    bool operator<(const triple<A,B,C>& t) const {
      // lexicographical order
      if (first != t.first) {
        return first < t.first;
      } else { // first == t.first
        if (second != t.second) {
          return second < t.second;
        } else { // second == t.second
          return third < t.third;
        }
      }
    }
  };

// SUN WSPro 6 does not recognize these operators defined in <utility>
//   (because they are embedded in the namespace std::rel_op ??)
#ifdef UIMA_RELOPS_TEMPLATE_BUG
  template <class T>
  inline bool operator!=(const T& x, const T& y) {
    return !(x == y);
  }

  template <class T>
  inline bool operator>(const T& x, const T& y) {
    return y < x;
  }

  template <class T>
  inline bool operator<=(const T& x, const T& y) {
    return !(y < x);
  }

  template <class T>
  inline bool operator>=(const T& x, const T& y) {
    return !(x < y);
  }

#endif

/////////////////////////////////////////////


// find the index of the value in vec
// if it is nout foun, -1 is returned
  template <class T>
  int findIndex(std::vector<T> const & vec, T const & value) {
    size_t i;
    for (i=0; i<vec.size(); ++i) {
      if (vec[i] == value) {
        return (int)i;
      }
    }
    return -1;
  }

// the equivalent to auto_ptr for single objects for arrays
  template<class T>
  class UIMA_LINK_IMPORTSPEC auto_array {
  private:
    T* iv_ar;
    bool iv_ownsArray;

    // copy construtcor and assignment are hidden here!!

    auto_array(auto_array<T> & other) {
      *this = other;
    }

    auto_array<T> & operator=(auto_array<T> & other ) {
      iv_ar = other.iv_ar;
      if (other.iv_ownsArray) {
        iv_ownsArray = true;
        other.iv_ownsArray = false;
      } else {
        iv_ownsArray = false;
      }
      return *this;
    }

  public:
    auto_array(T* ar)
        : iv_ar(ar),
        iv_ownsArray(true) {}


    ~auto_array() {
      if (iv_ownsArray) {
        delete[] iv_ar;
      }
    }

    T& operator[](size_t i) {
      return iv_ar[i];
    }

    T const & operator[](size_t i) const {
      return iv_ar[i];
    }

    T* release() {
      iv_ownsArray = false;
      return iv_ar;
    }

    T* get() {
      return iv_ar;
    }

    T const * get() const {
        return iv_ar;
      }

  };


} //namespace uima

#endif /* UIMA_STLTOOLS_HPP */
/* <EOF> */

