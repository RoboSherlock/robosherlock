#ifndef UIMA_STRTOOLS_HPP
#define UIMA_STRTOOLS_HPP
/** \file strtools.hpp .
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

    \brief  String tools functions.

-------------------------------------------------------------------------------
*/

/* ----------------------------------------------------------------------- */
/*      Include dependencies                                               */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> //must be included first to disable warnings
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <functional>
#include <uima/types.h>
#include "unicode/uchar.h"

/* ----------------------------------------------------------------------- */
/*      Implementation dependencies                                        */
/* ----------------------------------------------------------------------- */

namespace uima {

  /* ----------------------------------------------------------------------- */
  /*      Constants                                                          */
  /* ----------------------------------------------------------------------- */

#if defined(AE_STL)
  /** Macro for the new ANSI string constant <TT>string::npos</TT>.
     To make it work with adaption efforts <TT>bstring.h</TT>
  */
#  define STRING_NPOS NPOS
#else
#  define STRING_NPOS std::string::npos
#endif

//defines for defining basic string template functions with OS/AE STl implementations
#ifdef OS_STL           //Object Space STL implementation
#  define BASIC_STRING_NAME                  os_basic_string
#  define BS_TEMPLATE_DEFINITION_ARGS        OS_BS_TEMPLATE_DEFINITION_ARGS
#  define BS_TEMPLATE_ARGS                   OS_BS_TEMPLATE_ARGS
#  define SINGLE_ARG_BASIC_STRING( CharT )   os_basic_string<CharT, os_char_traits< CharT >, Allocator< CharT > >
#elif defined(AE_STL)   //Adaption Effort STL implementation (public domain)
#  define BASIC_STRING_NAME                  basic_string
#  define BS_TEMPLATE_DEFINITION_ARGS        <class CharT CLASS_TRAITS_ALLOC>
#  define BS_TEMPLATE_ARGS                   <CharT CLASS_TRAITS_ALLOC>
#  define SINGLE_ARG_BASIC_STRING( CharT )   std::basic_string<CharT, char_traits< CharT >, allocator< CharT > >
#else                   //ANSI compliant compiler specific STL (assumes compiler can handle template default args)
#  define BASIC_STRING_NAME                  std::basic_string
#  if defined(__OS_DYNIXPTX__)
  // ptxC++ supports default template arguments, but definition of
  // class basic_string doesn't exploit them :-(
#    define BS_TEMPLATE_DEFINITION_ARGS        <class CharT, class traits, class Allocator >
#    define BS_TEMPLATE_ARGS                   < CharT, traits, Allocator >
#    define SINGLE_ARG_BASIC_STRING( CharT ) std::basic_string< CharT, char_traits< CharT >, Allocator< CharT > >
#  else
#  define BS_TEMPLATE_DEFINITION_ARGS        <class CharT >
#  define BS_TEMPLATE_ARGS                   < CharT >
#  define SINGLE_ARG_BASIC_STRING( CharT )   std::basic_string< CharT >
#endif
#endif

#define BASIC_STRING_TEMPLATE                BASIC_STRING_NAME BS_TEMPLATE_ARGS

#define CHAR_EOS  '\0'
  /* ----------------------------------------------------------------------- */
  /** @name Character utility functions
      Character functions are handled by providing specializations
      for each character type. In practice, a template is used for the
      default case single character and one specialization for UCS2.
      Provided functions:
      <TT>tolower_templ()</TT>
      <TT>toupper_templ()</TT>
      <TT>isspace_templ()</TT>
      <TT>isalnum_templ()</TT>
      Note: Automatic documentation does not seem to work for some of them!
    ---------------------------------------------------------------------------*/
  /*@{*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template < class CharT >
  CharT
  tolower_templ(
    const CharT & c
  );
#endif

  /** Template version of <TT>tolower()</TT>.
     This is designed for single byte and UCS2 characters only.
  */
  template < class CharT >
  CharT
  tolower_templ(
    const CharT & c
  ) {
    assertWithMsg(sizeof(CharT) == sizeof(char),"Missing explicit specialization for character type.");//lint !e506: Constant value Boolean
    return (CharT)::tolower(c);  //lint !e50: Attempted to take the address of a non-lvalue
  }

  inline UChar
  tolower_templ(
    const UChar & c
  ) {
    return (UChar)u_tolower(c);
  }  //lint !e1746: parameter 'c' in function 'tolower_templ(UChar)' could be made const reference

  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template < class CharT >
  CharT
  toupper_templ(
    const CharT & c
  );
#endif

  /** Template version of <TT>toupper()</TT>.
     This is designed for single byte and UCS2 characters only.
  */
  template < class CharT >
  CharT
  toupper_templ(
    const CharT & c
  ) { //lint !e1053 !e18: Symbol 'toupper_templ(const char)' redeclared
    assertWithMsg(sizeof(CharT) == sizeof(char),"Missing explicit specialization for character type.");//lint !e506: Constant value Boolean
    return (CharT)toupper(c);
  }

  inline UChar
  toupper_templ(
    const UChar & c
  ) {
    return (UChar)u_toupper(c);
  }  //lint !e1746: parameter 'c' in function 'toupper_templ(UChar)' could be made const reference

  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template < class CharT >
  bool
  isspace_templ(
    const CharT & c
  );
#endif

  /** Template version of <TT>isspace()</TT>.
     This is designed for single byte and UCS2 characters only.
  */
  template < class CharT >
  bool
  isspace_templ(
    const CharT & c
  ) {
    assertWithMsg(sizeof(CharT) == sizeof(char),"Missing explicit specializatio for character type.");//lint !e506: Constant value Boolean
    return (bool)isspace(c);
  }

  inline bool
  isspace_templ(
    const UChar & c
  ) {
    return (bool)u_isspace(c);
  }  //lint !e1746: parameter 'c' in function 'isspace_templ(UChar)' could be made const reference


  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template < class CharT >
  bool
  isalnum_templ(
    const CharT & c
  );
#endif

  /** Template version of <TT>isalnum()</TT>.
     This is designed for single byte and UCS2 characters only.
  */
  template < class CharT >
  bool
  isalnum_templ(
    const CharT & c
  ) {
    assertWithMsg(sizeof(CharT) == sizeof(char),"Missing explicit specializatio for character type.");//lint !e506: Constant value Boolean
    return (bool)isalnum(c);
  }

  inline bool
  isalnum_templ(
    const UChar & c
  ) {
    return (bool)u_isalnum(c);
  }  //lint !e1746: parameter 'c' in function 'isalnum_templ(UChar)' could be made const reference

  /*@}*/

  /* ----------------------------------------------------------------------- */
  /** @name Template version of common C string functions                    */
  /* ----------------------------------------------------------------------- */
  /*@{*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template < class CharT >
  const CharT *
  strchr_templ(
    const CharT * cpszString,
    const CharT & c
  );
#endif

  /** Template version of <TT>strchr()</TT>.
      This is case sensitive.
  */
  template < class CharT >
  const CharT *
  strchr_templ(
    const CharT * cpszString,
    const CharT & c
  ) {
    assert(EXISTS(cpszString));
    while (*cpszString != c && *cpszString != (CharT)CHAR_EOS) {  //lint !e1912: Implicit call of conversion function from class 'UChar' to type 'unsigned short'
      ++cpszString;
    }
    if (*cpszString == (CharT)CHAR_EOS) {
      return NULL;
    }
    return (cpszString);
  }

  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template < class CharT >
  size_t
  strlen_templ(
    const CharT * cpszString
  );
#endif

  /** Template version of <TT>strlen()</TT>.
      This is case sensitive.
  */
  template < class CharT >
  size_t
  strlen_templ(
    const CharT * cpszString
  ) {
    assert(EXISTS(cpszString));
    size_t uiLen = 0;
    while (*cpszString != (CharT)CHAR_EOS) {  //lint !e1912: Implicit call of conversion function from class 'UChar' to type 'unsigned short'
      ++uiLen;
      ++cpszString;
    }
    return (uiLen);
  }

  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template < class CharT >
  inline CharT *
  strncpy_templ(
    CharT *       pszTarget,
    const CharT * cpszSource,
    size_t        uiSourceLen
  );
#endif

  /** Template version of <TT>strncpy()</TT>.
  */
  template < class CharT >
  inline CharT *
  strncpy_templ(
    CharT *       pszTarget,
    const CharT * cpszSource,
    size_t        uiSourceLen
  ) {
    assert(EXISTS(cpszSource));
    assert(uiSourceLen == 0 || EXISTS(pszTarget));
    memcpy(pszTarget, cpszSource, uiSourceLen*sizeof(CharT));

    return pszTarget;
  }



  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template < class CharT >
  int
  strcmp_templ(
    const CharT * cpszString1,
    const CharT * cpszString2
  );
#endif

  /** Template version of <TT>strcmp()</TT>.
      This is case sensitive.
  */
  template < class CharT >
  int
  strcmp_templ(
    const CharT * cpszString1,
    const CharT * cpszString2
  ) {
    if (cpszString1 == NULL) {
      if (cpszString2 == NULL) {
        return 0;
      }
      return  -1;
    }
    if (cpszString2 == NULL) {
      return +1;
    }
    assert(EXISTS(cpszString1));
    assert(EXISTS(cpszString2));
    while (*cpszString1 == *cpszString2) {
      if (*cpszString1 == (CharT)CHAR_EOS)
        return(0);
      ++cpszString1;
      ++cpszString2;
    }
    return ( (int) *cpszString1 - (int) *cpszString2);
  }

  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template < class CharT >
  int
  stricmp_templ(
    const CharT * cpszString1,
    const CharT * cpszString2
  );
#endif

  /** Template version of <TT>stricmp()</TT>.
      This is case sensitive
  */
  template < class CharT >
  int
  stricmp_templ(
    const CharT * cpszString1,
    const CharT * cpszString2
  ) {
    if (cpszString1 == NULL) {
      if (cpszString2 == NULL) {
        return 0;
      }
      return  -1;
    }
    if (cpszString2 == NULL) {
      return +1;
    }
    assert(EXISTS(cpszString1));
    assert(EXISTS(cpszString2));
    while (toupper_templ(*cpszString1) == toupper_templ(*cpszString2)) {
      if (*cpszString1 == (CharT)CHAR_EOS)
        return(0);
      ++cpszString1;
      ++cpszString2;
    }
    return ((int) toupper_templ(*cpszString1) - (int) toupper_templ(*cpszString2));
  }

  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template < class CharT >
  int
  strncmp_templ(
    const CharT * cpszString1,
    const CharT * cpszString2,
    size_t        uiLen
  );
#endif

  /** Template version of <TT>strncmp()</TT>.
      Compare string up to specified length (this is case sensitive).
  */
  template < class CharT >
  int
  strncmp_templ(
    const CharT * cpszString1,
    const CharT * cpszString2,
    size_t        uiLen
  ) {
    assert(EXISTS(cpszString1));
    assert(EXISTS(cpszString2));

    if (!(bool)uiLen) {
      return(0);
    }
    while ((bool)--uiLen && (bool)*cpszString1 && *cpszString1 == *cpszString2) //lint !e1912: Implicit call of conversion function from class 'const UChar' to type 'const UChar'
    {
      cpszString1++;
      cpszString2++;
    }
    return((int) *cpszString1 - (int) *cpszString2);  //lint !e794: Conceivable use of null pointer 'cpszString2' in argument to operator 'unary *'
  }

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template < class CharT >
  int
  strncmp_templ(
    const CharT * cpszString1,
    const CharT * cpszString2,
    int           iLen
  );
#endif

  /*  Int arg version to get template arguments matched.
      of the template version of <TT>strncmp()</TT>.
  */
  template < class CharT >
  int
  strncmp_templ(
    const CharT * cpszString1,
    const CharT * cpszString2,
    int           iLen
  ) {
    assert(iLen >= 0);
    return strncmp_templ(cpszString1, cpszString2, (size_t)iLen);
  }

  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template < class CharT >
  int
  strnicmp_templ(
    const CharT * cpszString1,
    const CharT * cpszString2,
    size_t        uiLen
  );
#endif

  /** Template version of <TT>strnicmp()</TT>.
      Compare string up to specified length (this is case insensitive).
  */
  template < class CharT >
  int
  strnicmp_templ(
    const CharT * cpszString1,
    const CharT * cpszString2,
    size_t        uiLen
  ) {
    assert(EXISTS(cpszString1));
    assert(EXISTS(cpszString2));
    while (uiLen && (toupper_templ(*cpszString1) == toupper_templ(*cpszString2))) {
      if (*cpszString1 == (CharT)CHAR_EOS)
        return(0);
      ++cpszString1;
      ++cpszString2;
      --uiLen;
    }
    if (uiLen)
      return ((int) toupper_templ(*cpszString1) - (int) toupper_templ(*cpszString2));
    return(0);
  }

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template < class CharT >
  int
  strnicmp_templ(
    const CharT * cpszString1,
    const CharT * cpszString2,
    int           iLen
  );
#endif

  /*  Int arg version to get template arguments matched.
      of the template version of <TT>strnicmp()</TT>.
  */
  template < class CharT >
  int
  strnicmp_templ(
    const CharT * cpszString1,
    const CharT * cpszString2,
    int           iLen
  ) {
    assert(iLen >= 0);
    return strnicmp_templ(cpszString1, cpszString2, (size_t)iLen);
  }
  /*@}*/

  /**
     Template class for case insensitive string comparison.
     Class T must be a derived from template class basic_string.
     The class currently supports only single character-based comparision.
  */
  template< class T >
  class UIMA_LINK_IMPORTSPEC StringLessI : public std::binary_function< T, T, bool > {
  public:
    bool
    operator()(const T& x, const T& y) const {
      if (x.length() == 0) {
        if (y.length() == 0) {
          return false;
        }
        return  true;
      }
      if (y.length() == 0) {
        return false;
      }
      assert(x.length() > 0);
      assert(y.length() > 0);
      return (stricmp_templ(x.c_str(), y.c_str()) < 0);
    }
  }
  ;  //lint !e1509 !e1905: implicit default constructor generated for class 'StringLessI' : base class destructor for class 'binary_function' is not virtual



  /**
    Tool class to encapsulate string pointers.
    Useful in templates where string* is not possible
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  struct StringPtrObj {
    const BASIC_STRING_TEMPLATE *
    m_pStr;
  public:
    ///(Default) Constructor
    StringPtrObj(
      const BASIC_STRING_TEMPLATE * pStr = NULL
    ) :
        m_pStr(pStr) {}

    ///String Constructor
    StringPtrObj(
      const BASIC_STRING_TEMPLATE & str
    ) :
        m_pStr(&str) {}

    ///Copy Constructor
    StringPtrObj(
      const StringPtrObj & spo
    ) :
        m_pStr(spo.m_pStr) {}

    ///Sssignment operator
    const StringPtrObj&
    operator = (
      const StringPtrObj & spo
    ) {
      m_pStr = spo.m_pStr;
      return *this;
    }

    ///less operator
    bool operator < (
      const StringPtrObj & spo
    ) const
    {
      return (m_pStr < spo.m_pStr);
    }

    ///equality operator
    bool operator == (
      const StringPtrObj & spo
    ) const
    {
      return m_pStr == spo.m_pStr;
    }

    ///conversion to BASIC_STRING_TEMPLATE *
    operator const BASIC_STRING_TEMPLATE * () const
    {
      return m_pStr;
    }

    ///* operator
    const BASIC_STRING_TEMPLATE
    operator * () {
      return *m_pStr;
    }
  };


  /* ----------------------------------------------------------------------- */
  /** @name Reading from files up to delimiters                              */
  /* ----------------------------------------------------------------------- */
  /*@{*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  bool
  strFileToStrVector(
    const BASIC_STRING_TEMPLATE &  filename,
    vector<BASIC_STRING_TEMPLATE>& v,
    bool                           toLower,
    bool                           trim
  );
#endif

  /**
     Reads in a text file and puts the content in a vector of
     strings.
     One vector element per line.

     @param filename The filename and vector to store the results.
     @param v        (Output) The vector of strings to store the result in.
     @param toLower  A Flag, if the strings are to be lowercase.

     @return true if file could be successfully opened, otherwise false.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  bool
  strFileToStrVector(
    const BASIC_STRING_TEMPLATE     &  filename,
    std::vector< BASIC_STRING_TEMPLATE > & v,
    bool                              toLower,
    bool                              trim
  ) {
    std::ifstream inStream(filename);
    if (inStream.fail()) {
      return false;
    }
    STLEraseAll(v);
    BASIC_STRING_TEMPLATE s;
    while (!inStream.eof()) {
      getline(inStream, s, "\n");
      if (toLower) {
        strlower(s);
      }
      if (trim) {
        s = strtrim(s); // fixed, tg Fri Nov  6 19:04:24 EST 1998
      }
      v.push_back(s);
    }
    return true;
  }



  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  CharT
  getline(
    istream& i,
    BASIC_STRING_TEMPLATE & s,
    const CharT* delimiters
  );
#endif


  /**
     Reads in a string s from a stream up to a delimiter from
     a set of delimiters.
     The delimiter set is specified as a CharT*
     Note: Derived from getline for strings in bstring.h

     @param i          The input stream to read from.
     @param s          (Output) The string to return the line read in.
     @param delimiters The (line) delimiters (interpreted as a SET of characters).

     @return The actual delimiter found.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  CharT
  getline(
    std::istream          & i,
    BASIC_STRING_TEMPLATE & s,
    const CharT           * delimiters
  ) {
    assert(EXISTS(delimiters));
    s.erase();
    //s.clear();
    CharT  value;
    while (true) {  //lint !e716: while(1) ...
      i >> value;
      if (!i)//.operator void*())
        break;
      if (!strchr_templ(delimiters, value)) {
        s += value;
        while (true) {  //lint !e716: while(1) ...
          i >> value;
          if (!i)//).operator void*())
            break;
          if (!strchr_templ(delimiters, value)) {
            s += value;
          } else
            break;
        }
        break;
      }
    }
    return value;
  }



  /**
     Reads in a string s from a stream up to a delimiter out of
     a set of delimiters and converts into an int or long.
     The delimiter set is specified as a CharT*

     @param i          The input stream to read from.
     @param l          (Output) The number to store the resulting number in.
     @param delimiters The (line) delimiters (interpreted as a SET of characters)

     @return The actual delimiter found
  */
  template <class NumType, class CharT >
  CharT
  getlineAsNum(
    std::istream& i,
    NumType     & l,
    const CharT * delimiters
  ) {
    SINGLE_ARG_BASIC_STRING( CharT ) s;
    CharT retChar = getline(i, s, delimiters);
    l = string2Long(s);
    return retChar;
  }


  /**
     Reads in a string s from a stream up to a delimiter out of
     a set of delimiters and converts into a double, float or similar.
     The delimiter set is specified as a CharT*

     @param i          The input stream to read from.
     @param d          (Output) The number to store the resulting number in.
     @param delimiters The (line) delimiters (interpreted as a SET of chars)

     @return The actual delimiter found
  */
  template <class RealType, class CharT >
  CharT
  getlineAsReal(
    std::istream& i,
    RealType    & d,
    const CharT * delimiters
  ) {
    SINGLE_ARG_BASIC_STRING( CharT ) s;
    CharT retChar = getline(i, s, delimiters);
    d = string2Double(s);
    return retChar;
  }

  /*@}*/

  /* ----------------------------------------------------------------------- */
  /** @name Padding (right and left)                                         */
  /* ----------------------------------------------------------------------- */
  /*@{*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  strpad_r(
    BASIC_STRING_TEMPLATE & s,
    size_t               newLen,
    const CharT          padChar
  );
#endif

  /**
     Pads a string from the right up to a specified length with a specified pad
     character.
     Note: Padding will not shorten the string if <TT>newLen</TT> is less then <TT>s.length()</TT>,
     a mutating version.

     @param s       (Output) The string to pad.
     @param newLen  The new length of the string after padding.
     @param padChar The character to use for padding.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  strpad_r(
    BASIC_STRING_TEMPLATE & s,
    size_t               newLen,
    const CharT          padChar
  ) {
    for (size_t i = s.length(); i < newLen; i++) {
      s += padChar;
    }
  }


  /*---------------------------------------------------------------------------*/
#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  strpad_r(
    BASIC_STRING_TEMPLATE & s,
    size_t               newLen
  );
#endif

  /**
     Pads a string from the right up to a specified length with a specified pad
     character.
     Note: Padding will not shorten the string if <TT>newLen</TT> is less then <TT>s.length()</TT>,
     a mutating version.

     @param s       (Output) The string to pad.
     @param newLen  The new length of the string after padding.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  strpad_r(
    BASIC_STRING_TEMPLATE & s,
    size_t               newLen
  ) {
    strpad_r(s, newLen, ' ');
  }


  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strpad_r_copy(
    const BASIC_STRING_TEMPLATE & s,
    size_t                     newLen,
    const CharT                padChar
  );
#endif

  /**
     Pads a string from the right up to a specified length with a specified pad
     character.
     Note: Padding will not shorten the string if <TT>newLen</TT> is less then <TT>s.length()</TT>,
     a mutating version.

     @param s       (Output) The string to pad.
     @param newLen  The new length of the string after padding.
     @param padChar The character to use for padding.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strpad_r_copy(
    const BASIC_STRING_TEMPLATE & s,
    size_t                     newLen,
    const CharT                padChar
  ) {
    BASIC_STRING_TEMPLATE s1(s);
    strpad_r(s1, newLen, padChar);
    return s1;
  }

  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strpad_r_copy(
    const BASIC_STRING_TEMPLATE & s,
    size_t                     newLen
  );
#endif

  /**
     Pads a string from the right up to a specified length with a specified pad
     character.
     Note: Padding will not shorten the string if <TT>newLen</TT> is less then <TT>s.length()</TT>,
     a mutating version.

     @param s       (Output) The string to pad.
     @param newLen  The new length of the string after padding.
     @param padChar The character to use for padding.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strpad_r_copy(
    const BASIC_STRING_TEMPLATE & s,
    size_t                     newLen
  ) {
    return strpad_r_copy(s, newLen, ' ');
  }


  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  strpad_l(
    BASIC_STRING_TEMPLATE & s,
    size_t                  newLen,
    const CharT             padChar
  );
#endif

  /**
     Pads a string from the left up to a specified length with a specified pad
     character.
     Note: Padding will not shorten the string if <TT>newLen</TT> is less then <TT>s.length()</TT>,
     a mutating version.

     @param s       (Output) The string to pad.
     @param newLen  The new length of the string after padding.
     @param padChar The character to use for padding.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  strpad_l(
    BASIC_STRING_TEMPLATE & s,
    size_t                  newLen,
    const CharT             padChar
  ) {
    if (newLen > s.length()) {
      s.insert((size_t)0, (size_t)newLen-s.length(), padChar);
    }
  }

  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  strpad_l(
    BASIC_STRING_TEMPLATE & s,
    size_t                  newLen
  );
#endif

  /**
     Pads a string from the left up to a specified length with a specified pad
     character.
     Note: Padding will not shorten the string if <TT>newLen</TT> is less then <TT>s.length()</TT>,
     a mutating version.

     @param s       (Output) The string to pad.
     @param newLen  The new length of the string after padding.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  strpad_l(
    BASIC_STRING_TEMPLATE & s,
    size_t                  newLen
  ) {
    strpad_l(s, newLen, ' ');
  }


  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strpad_l_copy(
    const BASIC_STRING_TEMPLATE & s,
    size_t                        newLen,
    const CharT                   padChar
  );
#endif


  /**
     Pads a string from the left up to a specified length with a specified pad
     character.
     Note: Padding will not shorten the string if <TT>newLen</TT> is less then <TT>s.length()</TT>,
     a copy version.

     @param s       The string to pad.
     @param newLen  The new length of the string after padding.
     @param padChar The character to use for padding.

     @return The padded string.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strpad_l_copy(
    const BASIC_STRING_TEMPLATE & s,
    size_t                        newLen,
    const CharT                   padChar
  ) {
    BASIC_STRING_TEMPLATE s1(s);
    strpad_l(s1, newLen, padChar);
    return s1;
  }

  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strpad_l_copy(
    const BASIC_STRING_TEMPLATE & s,
    size_t                        newLen
  );
#endif


  /**
     Pads a string from the left up to a specified length with a specified pad
     character.
     Note: Padding will not shorten the string if <TT>newLen</TT> is less then <TT>s.length()</TT>,
     a copy version.

     @param s       The string to pad.
     @param newLen  The new length of the string after padding.

     @return The padded string.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strpad_l_copy(
    const BASIC_STRING_TEMPLATE & s,
    size_t                        newLen
  ) {
    return strpad_l_copy(s, newLen, ' ');
  }

  /*@}*/

  /* ----------------------------------------------------------------------- */
  /** @name String upper/lower casing                                        */
  /* ----------------------------------------------------------------------- */
  /*@{*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  strupper(
    BASIC_STRING_TEMPLATE & s
  );
#endif

  /** Converts a string to uppercase, a mutating version.
     Template function.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  strupper(
    BASIC_STRING_TEMPLATE & s
  ) {
    for (size_t i =0; i < s.length(); ++i) {
      s[i] = (CharT)toupper_templ(s[i]);
    }
  }



  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strupper_copy(
    const BASIC_STRING_TEMPLATE & s
  );
#endif

  /** Converts a string to uppercase, a copy version.
     Template function.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strupper_copy(
    const BASIC_STRING_TEMPLATE & s
  ) {
    BASIC_STRING_TEMPLATE s1(s);
    strupper(s1);
    return s1;
  }


  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  strlower(
    BASIC_STRING_TEMPLATE & s
  );
#endif

  /** Converts a string to lowercase, a mutating version.
     Template function.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  strlower(
    BASIC_STRING_TEMPLATE & s
  ) {
    for (size_t i = 0; i < s.length(); ++i) {
      s[i] = (CharT)tolower_templ(s[i]);
    }
  }


  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strlower_copy(
    const BASIC_STRING_TEMPLATE & s
  );
#endif

  /** Converts a string to lowercase, a copy version.
     Template function.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strlower_copy(
    const BASIC_STRING_TEMPLATE & s
  ) {
    BASIC_STRING_TEMPLATE s1(s);
    strlower(s1);
    return s1;
  }

  /*@}*/

  /* ----------------------------------------------------------------------- */
  /** @name Trimming (white space removal)                                   */
  /* ----------------------------------------------------------------------- */
  /*@{*/


#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strtrim(
    const BASIC_STRING_TEMPLATE & s
  );
#endif

  /**
     Removes whitespace from both ends of a string.
     Template function using <TT>isspace_templ()</TT>.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strtrim(
    const BASIC_STRING_TEMPLATE & s
  ) {
    if (s.length() == 0) {
      return s;
    }
    size_t beg = 0;
    size_t end = s.length()-1;
    while (end >= beg && isspace_templ(s[end]) ) {
      --end;
    }
    while (beg < end && isspace_templ(s[beg]) ) {
      ++beg;
    }
    return s.substr(beg, end-beg+1);
  }

  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strtrimr(
    const BASIC_STRING_TEMPLATE & s
  );
#endif

  /**
     Removes whitespace from the end of a string.
     Template function using <TT>isspace_templ()</TT>.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strtrimr(
    const BASIC_STRING_TEMPLATE & s
  ) {
    long end = s.length()-1;
    while (end >= 0 && isspace_templ(s[end])) {
      end--;
    }
    return s.substr(0, end+1);
  }

  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strtriml(
    const BASIC_STRING_TEMPLATE & s
  );
#endif

  /**
     Removes whitespace from the beginning of a string.
     Template function using <TT>isspace_templ()</TT>.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strtriml(
    const BASIC_STRING_TEMPLATE & s
  ) {
    long beg = 0;
    long end = s.length()-1;
    while (beg != end && isspace_templ(s[beg]) ) {
      beg++;
    }
    return s.substr(beg, end-beg);
  }


  /*@}*/

  /* ----------------------------------------------------------------------- */
  /** @name Search and Replace                                               */
  /* ----------------------------------------------------------------------- */
  /*@{*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template <class CharT>
  size_t
  str_find_first(
    const CharT * cpacPattern, // pattern to search for
    size_t        uiPatternLen,// length of pattern
    const CharT * cpacSearch,  // string to search in
    size_t        uiSearchLen  // length of string to search in
  );
#endif

  /**
     Finds the first occurence of key string <TT>cpacPattern</TT> (with length
     <TT>uiPatternLen</TT>), in the searched string <TT>cpacSearch</TT> (with length
     <TT>uiSearchLen</TT>).
  */
  template <class CharT>
  size_t
  str_find_first(
    const CharT * cpacPattern, // pattern to search for
    size_t        uiPatternLen,// length of pattern
    const CharT * cpacSearch,  // string to search in
    size_t        uiSearchLen  // length of string to search in
  ) {
    // The key won't even fit in what we are searching.
    if ( uiPatternLen > uiSearchLen )
      return STRING_NPOS;
    // The empty string occurs at every position: we return the first
    if ( uiPatternLen == 0 )
      return 0;


    /* taph 2/22/2000: insert your Boyer-Moore search here:
        Should be much faster for longer patterns, but the skip table
        gets quite large for Unicode CharT
    */

    assert(EXISTS(cpacSearch));
    assert(EXISTS(cpacPattern));
    // The highest position it could possibly be found at:
    const CharT * cpacEndPos = cpacSearch + (uiSearchLen - uiPatternLen);
    for ( const CharT * cpacLoop = cpacSearch; cpacLoop <= cpacEndPos; ++cpacLoop ) {
      /* taph 2/22/2000: this could also use traits< CharT >::compare but for now
         we have decided not to make those tool functions traits aware.
       */
      if ( memcmp( cpacLoop, cpacPattern, (uiPatternLen*sizeof(CharT)) ) == 0 )
        return (cpacLoop - cpacSearch);
    }
    return STRING_NPOS; // Not found.
  }


#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template <class CharT>
  size_t
  str_find_first(
    const CharT   cPattern,   // pattern character to search for
    const CharT * cpacSearch, // string to search in
    size_t        uiSearchLen // length of string to search in
  );
#endif

  /**
     Finds the first occurence of character <TT>cPattern</TT>
     in the searched string <TT>cpacSearch</TT> (with length <TT>uiSearchLen</TT>).
  */
  template <class CharT>
  size_t
  str_find_first(
    const CharT   cPattern,   // pattern character to search for
    const CharT * cpacSearch, // string to search in
    size_t        uiSearchLen // length of string to search in
  ) {
    assert(EXISTS(cpacSearch));
    // The highest position it could possibly be found at:
    const CharT * cpacEndPos = cpacSearch + uiSearchLen;
    for ( const CharT * cpacLoop = cpacSearch; cpacLoop < cpacEndPos; ++cpacLoop ) {
      /* taph 2/22/2000: this could also use traits< CharT >::compare but for now
         we have decided not to make those tool functions traits aware.
       */
      if ( (*cpacLoop) == cPattern )
        return (cpacLoop - cpacSearch);
    }
    return STRING_NPOS; // Not found.
  }  //lint !e1746: parameter 'cPattern' could be made const reference

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template <class CharT>
  size_t
  str_find_first_of(
    const CharT * cpacPatternCharSet, // set of chars to search for
    size_t        uiPatternCharSetLen,// length of pattern
    const CharT * cpacSearch,         // string to search in
    size_t        uiSearchLen         // length of string to search in
  );
#endif

  /**
     Finds the first occurence of key string <TT>cpacPattern</TT> (with length
     <TT>uiPatternLen</TT>), in the searched string <TT>cpacSearch</TT> (with length
     <TT>uiSearchLen</TT>).
  */
  template <class CharT>
  size_t
  str_find_first_of(
    const CharT * cpacPatternCharSet, // set of chars to search for
    size_t        uiPatternCharSetLen,// length of pattern
    const CharT * cpacSearch,         // string to search in
    size_t        uiSearchLen         // length of string to search in
  ) {
    assert(uiPatternCharSetLen > 0);
    assert(EXISTS(cpacPatternCharSet));

    // The highest position it could possibly be found at:
    const CharT * cpacEndPos = cpacSearch + uiSearchLen;
    for ( const CharT * cpacLoop = cpacSearch; cpacLoop < cpacEndPos; ++cpacLoop ) {
      if ( str_find_first((*cpacLoop), cpacPatternCharSet, uiPatternCharSetLen ) != STRING_NPOS)
        return (cpacLoop - cpacSearch);
    }
    return STRING_NPOS; // Not found.
  }

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template <class CharT>
  size_t
  str_find_first_not_of(
    const CharT * cpacPatternCharSet, // set of chars to search for
    size_t        uiPatternCharSetLen,// length of pattern
    const CharT * cpacSearch,         // string to search in
    size_t        uiSearchLen         // length of string to search in
  );
#endif

  /**
     Finds the first occurence of key string <TT>cpacPattern</TT> (with length
     <TT>uiPatternLen</TT>), in the searched string <TT>cpacSearch</TT> (with length
     <TT>uiSearchLen</TT>).
  */
  template <class CharT>
  size_t
  str_find_first_not_of(
    const CharT * cpacPatternCharSet, // set of chars to search for
    size_t        uiPatternCharSetLen,// length of pattern
    const CharT * cpacSearch,         // string to search in
    size_t        uiSearchLen         // length of string to search in
  ) {
    assert(uiPatternCharSetLen > 0);
    assert(EXISTS(cpacPatternCharSet));

    // The highest position it could possibly be found at:
    const CharT * cpacEndPos = cpacSearch + uiSearchLen;
    for ( const CharT * cpacLoop = cpacSearch; cpacLoop < cpacEndPos; ++cpacLoop ) {
      if ( str_find_first((*cpacLoop), cpacPatternCharSet, uiPatternCharSetLen ) == STRING_NPOS)
        return (cpacLoop - cpacSearch);
    }
    return STRING_NPOS; // Not found.
  }

  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template <class CharT>
  void
  strreplaceallchar(
    CharT* s,
    CharT  searchchar,
    CharT  replacechar
  );
#endif

  /**
     Replaces one CharT in a CharT* with another CharT.
     (CharT* target with CharT search arguments)

     @param s           The CharT* where characters are replaced.
     @param searchchar  The CharT to search for.
     @param replacechar The CharT to replace the <TT>searchchar</TT> with.
  */
  template <class CharT>
  void
  strreplaceallchar(
    CharT* s,
    CharT  searchchar,
    CharT  replacechar
  ) {
    if (s == NULL) {
      return;
    }
    size_t pos;
    for (pos = 0; s[pos] != CharT(); pos++) {
      if (s[pos] == searchchar) {
        s[pos] = replacechar;
      }
    }
  }


  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  strreplaceallchar(
    BASIC_STRING_TEMPLATE & s,
    const CharT             searchchar,
    const CharT             replacechar
  );
#endif

  /**
     Replaces one CharT in a string with another CharT.
     (string target with CharT search arguments)

     @param s           (Output) The string where characters are replaced.
     @param searchchar  The CharT to search for.
     @param replacechar The CharT to replace the <TT>searchchar</TT> with.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  strreplaceallchar(
    BASIC_STRING_TEMPLATE & s,
    const CharT             searchchar,
    const CharT             replacechar
  ) {
    size_t pos;
    for (pos = 0; pos < s.length(); pos++) {
      if (s[pos] == searchchar) {
        s[pos] = replacechar;
      }
    }
  }


  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  strreplaceall(
    BASIC_STRING_TEMPLATE       & s,
    const BASIC_STRING_TEMPLATE & searchstr,
    const BASIC_STRING_TEMPLATE & replacestr
  );
#endif

  /**
     Replaces one string in a string with another string.
     (string target with string search arguments)

     @param s          (Output) The string where characters are replaced.
     @param searchstr  The string to search for.
     @param replacestr The string to replace the <TT>searchstring</TT> with.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  strreplaceall(
    BASIC_STRING_TEMPLATE       & s,
    const BASIC_STRING_TEMPLATE & searchstr,
    const BASIC_STRING_TEMPLATE & replacestr
  ) {
    size_t pos(0);
    while (((pos = s.find(searchstr, pos)) != STRING_NPOS)) {
      s.replace(pos, searchstr.length(), replacestr);
      pos += replacestr.length();
    }
  }


  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  strreplacall_env(
    BASIC_STRING_TEMPLATE & s
  );
#endif

  /**
     Replaces all occurences of "$ENV(<TT>varname</TT>)" in a string with the value of
     the OS environment variable <TT>varname</TT>, a mutating version.

     @param s          (Output) The string where env-strings are replaced.
     @param searchstr  The position in s to start searching.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  strreplacall_env(
    BASIC_STRING_TEMPLATE & s
  ) {
    _strreplacall_env(s, (size_t)0);
  }


  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strreplacall_env_copy(
    const BASIC_STRING_TEMPLATE & s
  );
#endif

  /**
     Replaces all occurences of "$ENV(<TT>varname</TT>)" in a string with the value of
     the OS environment variable <TT>varname</TT>,
     a copy version.

     @param s          The string where env-strings are replaced.
     @param searchstr  The position in s to start searching.

     @return The string with the replaces vars.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  strreplacall_env_copy(
    const BASIC_STRING_TEMPLATE & s
  ) {
    BASIC_STRING_TEMPLATE s1(s);
    _strreplacall_env(s1, (size_t)0);
    return s1;
  }


  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  _strreplacall_env(
    BASIC_STRING_TEMPLATE & s,
    size_t                  pos
  );
#endif

  /*
     Replaces all occurences of "$ENV(<TT>varname</TT>)" in a string with the value of
     the OS environment variable <TT>varname</TT>,
     a mutating version.
     The function starts to search from a specified start position.
     Note: This function is used in the above <TT>strreplacall</TT> functions.

     @param s          (Output) The string where env-strings are replaced.
     @param searchstr  The position in s to start searching.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  _strreplacall_env(
    BASIC_STRING_TEMPLATE & s,
    size_t                  pos
  ) {
    const BASIC_STRING_TEMPLATE ENV_VAR = "$ENV(";
    unsigned int envBegPos = s.find(ENV_VAR, pos);
    unsigned int envEndPos;
    if (envBegPos != STRING_NPOS) {
      envEndPos = s.find(')', envBegPos+1);
      if (envEndPos != STRING_NPOS) {
        BASIC_STRING_TEMPLATE envVarName(s.substr(envBegPos+strlen_templ(ENV_VAR), envEndPos-envBegPos-strlen_templ(ENV_VAR)));
        assertWithMsg(sizeof(CharT) == sizeof(char), "Double check the use of getenv() with wchar.");//lint !e506: Constant value Boolean
        CharT   pEnv = getenv(envVarName.c_str());
        BASIC_STRING_TEMPLATE envVarValue("");
        if (pEnv != NULL) {
          envVarValue = pEnv;
        }
        _strreplacall_env(s, (size_t)envEndPos + 1);
        s = s.substr(0, envBegPos) +
        envVarValue +
        s.substr(envEndPos + 1);
      }
    }
  }

  /*@}*/

  /* ----------------------------------------------------------------------- */
  /** @name Sanitizing (special char removal)                                */
  /* ----------------------------------------------------------------------- */
  /*@{*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  sanitzeString(
    BASIC_STRING_TEMPLATE & s
  );
#endif

  /**
     Replaces all occurences of CharT not in [A-Za-z0-9] in a string with '_'.
     This is useful in creating valid filenames from strings,
     a mutating version.

     @param s          (Output) The string to replace chars in.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  void
  sanitzeString(
    BASIC_STRING_TEMPLATE & s
  ) {
    for (unsigned int i = 0; i < s.length(); i++) {
      if ( ! isalnum_templ(s[i]) ) {
        s[i] = '_';
      }
    }
  }



  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  sanitzeString_copy(
    const BASIC_STRING_TEMPLATE & s
  );
#endif

  /**
     Replaces all occurences of CharT not in [A-Za-z0-9] in a string with '_'.
     This is useful in creating valid filenames from strings,
     a copy version.

     @param s          The string to replace chars in.

     @return The sanitized string.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  sanitzeString_copy(
    const BASIC_STRING_TEMPLATE & s
  ) {
    BASIC_STRING_TEMPLATE s1(s);
    sanitzeString(s1);
    return s1;
  }

  /*@}*/

  /* ----------------------------------------------------------------------- */
  /** @name vector to/from delimited string conversion routines              */
  /* ----------------------------------------------------------------------- */
  /*@{*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  size_t
  delimitedString2Vector(
    vector< BASIC_STRING_TEMPLATE > & vecStrOutput,
    const BASIC_STRING_TEMPLATE     & crstrInput,
    const CharT*                      cpszDelimiters,
    bool                              bTrimString,
    bool                              bInsertEmptyStrings
  );
#endif


  /**
     Splits a delimited string into pieces and stores the results in a vector
     of strings. Delimiters are passed as a zero-terminated string.

     @param rvecstrOutput       (Output) The vector where the results are stored.
     @param crstrInput          The delimited string to split.
     @param cpszDelimiters      The delimiters (CharT* interpreted as a set of delimiters).
     @param bTrimString         Flag: If true, all pieces will be trimmed before storing in <TT>storeVar</TT>
     @param bInsertEmptyStrings Flag: If false, pieces that have length 0 will not be stored in  <TT>storeVar</TT>

     @return The number of strings added to <TT>rvecstrOutput</TT>
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  size_t
  delimitedString2Vector(
    std::vector< BASIC_STRING_TEMPLATE > & rvecstrOutput,
    const BASIC_STRING_TEMPLATE     & crstrInput,
    const CharT*                      cpszDelimiters,
    bool                              bTrimString,
    bool                              bInsertEmptyStrings
  ) {
    size_t uiBegin = 0;
    size_t uiEnd;
    size_t uiNumFound = 0;

    if (crstrInput.length() == 0) {
      return 0;
    }
    BASIC_STRING_TEMPLATE _s;

    while (uiBegin != STRING_NPOS) {
      //      uiBegin--;
      uiEnd   = crstrInput.find_first_of(cpszDelimiters, uiBegin+1);
      if (uiEnd != STRING_NPOS) {
        uiEnd   = crstrInput.find_first_not_of(cpszDelimiters, uiEnd+1);
      }
      if (uiEnd == STRING_NPOS) {
        uiEnd = crstrInput.length()+1;
      }
      _s = crstrInput.substr(uiBegin, uiEnd-uiBegin-1);
      if (bTrimString) {
        _s = strtrim(_s);
      }
      if (bInsertEmptyStrings || _s.length() > 0) {
        rvecstrOutput.push_back(_s);
        uiNumFound++;
      }
      uiBegin = crstrInput.find_first_not_of(cpszDelimiters, uiEnd);
    }
    return uiNumFound;
  }


#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  size_t
  delimitedStr2Vectorx(
    vector< BASIC_STRING_TEMPLATE > & rvecstrOutput,
    const BASIC_STRING_TEMPLATE     & crstrInput,
    const BASIC_STRING_TEMPLATE     & crstrDelimiters,
    bool                              bTrimString,
    bool                              bInsertEmptyStrings
  );
#endif


  /**
     Splits a delimited string into pieces and stores the results in a vector
     of strings. Delimiters are passed as string object.

     @param rvecstrOutput       (Output) The vector where the results are stored.
     @param crstrInput          The delimited string to split.
     @param cpszDelimiters      The delimiters (CharT* interpreted as a set of delimiters)
     @param bTrimString         Flag: If true, all pieces will be trimmed before storing in <TT>storeVar</TT>
     @param bInsertEmptyStrings Flag: If false, pieces that have length 0 will not be stored in  <TT>storeVar</TT>

     @return The number of strings added to <TT>rvecstrOutput</TT>
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  size_t
  delimitedStr2Vectorx(
    std::vector< BASIC_STRING_TEMPLATE > & rvecstrOutput,
    const BASIC_STRING_TEMPLATE     & crstrInput,
    const BASIC_STRING_TEMPLATE     & crstrDelimiters,
    bool                              bTrimString,
    bool                              bInsertEmptyStrings
  ) {
    return delimitedStr2Vector(rvecstrOutput, crstrInput, crstrDelimiters.c_str(), bTrimString, bInsertEmptyStrings);
  }


  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  vector2delimitedStr(
    const vector<BASIC_STRING_TEMPLATE>& v,
    const BASIC_STRING_TEMPLATE &        delimiters
  );
#endif

  /**
     Concatenates the strings in a vector of strings into one long string
     using a specified delimiter.

     @param v          The vector of strings to concatenate.
     @param delimiters The delimiter CharT(s) to insert between members of v.

     @return The concatenated string.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  vector2delimitedStr(
    const std::vector< BASIC_STRING_TEMPLATE >& v,
    const BASIC_STRING_TEMPLATE          & delimiters
  ) {
    BASIC_STRING_TEMPLATE s;
    typename std::vector< BASIC_STRING_TEMPLATE >::const_iterator it;

    for (it = v.begin(); it != v.end(); ++it) {
      s += *it;
      if (it != v.end() -1) {
        s += delimiters;
      }
    }
    return s;
  }


  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  int
  bracketedStr2Vector(
    vector<BASIC_STRING_TEMPLATE>& storeVar,
    const BASIC_STRING_TEMPLATE &  s,
    const CharT*                   openBracket,
    const CharT*                   closeBracket
  );
#endif

  /**
     Splits a string delimited by brackets into pieces and stores the results in
     a vector of strings.
     This is useful for LISP like structures such as <TT>(adf)(eds)</TT>

     @param storeVar           (Output) The vector where the results are stored.
     @param str                The delimited string to split.
     @param openBracket        The opening delimiter. CharT* is interpreted as a string.
     @param closeBracket       The closing delimiter. CharT* interpreted as a string.

     @return The number of strings added to <TT>storeVar</TT>
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  int
  bracketedStr2Vector(
    std::vector< BASIC_STRING_TEMPLATE > & storeVar,
    const BASIC_STRING_TEMPLATE     & s,
    const CharT*                      openBracket,
    const CharT*                      closeBracket
  ) {
    assert(EXISTS(openBracket));
    assert(EXISTS(closeBracket));
    if (s.length() == 0) {
      return 0;
    }
    int begin;
    int end;
    int openLen  = (int)strlen_templ(openBracket);  //lint !e668 Possibly passing a null pointer to function
    int closeLen = (int)strlen_templ(closeBracket); //lint !e668 Possibly passing a null pointer to function
    int numFound = 0;
    begin = (int)s.find(openBracket);
    if (begin == (int)STRING_NPOS) {
      begin = -openLen+1;
    }

    while (begin != (int)STRING_NPOS) {
      begin+=openLen;
      end   = (int)s.find(closeBracket, (size_t)(begin+openLen));
      if (end == (int)STRING_NPOS) {
        end = (int)s.length()+1;
      }
      storeVar.push_back(trimString(s.substr((size_t)begin, (size_t)(end-begin))));
      numFound++;
      begin = (int)s.find(openBracket, (size_t)(end+closeLen));
    }
    return numFound;
  }
  /*@}*/

} //namespace uima

#endif /* UIMA_STRTOOLS_HPP */
/* <EOF> */

