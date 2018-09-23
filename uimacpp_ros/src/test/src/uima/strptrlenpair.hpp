#ifndef UIMA_STRPTRLENPAIR_HPP
#define UIMA_STRPTRLENPAIR_HPP
/** \file strptrlenpair.hpp .
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

   \brief  Shallow string object consisting of a pair of
           string pointer and a length

-----------------------------------------------------------------------------
*/

#include "uima/pragmas.hpp" //must be included first to disable warnings

#include <vector>
#include <utility>
#include <string>
#include <iostream>

#include "uima/assertmsg.h"
//#include "uima/ccsid.hpp"
//#include "uima/u2cpcnvrt.hpp"
#include "unicode/uchar.h"

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {

  /**
     The class <TT>BasicStrPtrLenPair</TT> provides support for non zero-terminated strings
     that are presented as pointers to string arrays with an associated length.
     As this type of string is used only as string reference into read-only buffers,
     the string pointer is constant.
     The member functions are names in an ANSI basic_string.
     This enables a limited use of basic-l-strings in template functions that
     are designed for basic_strings (the hash functions will work, for example).
     Note: This is why previous function <TT>set()</TT> has been renamed
     <TT>assign()</TT>
  */
  template < class CharT >
  class BasicStrPtrLenPair : public std::pair< CharT const *, size_t > {
  public:
    ///(Default) Constructor
    BasicStrPtrLenPair( void ) :
        std::pair< CharT const * , size_t >(NULL, 0) {}

    ///Constructor from zero terminated string
    BasicStrPtrLenPair(
      const CharT * cpacString
    ) :
        std::pair< CharT const * , size_t >(cpacString, strlen_templ(cpacString)) {
      assert(   (EXISTS(first) )
                || ((first == NULL       ) && (second == 0)) );
    }

    ///Constructor from string and length
    BasicStrPtrLenPair(
      const CharT * cpacString,
      size_t        uiLength
    ) :
        std::pair< CharT const * , size_t >(cpacString, uiLength) {
      assert(   (EXISTS(first) )
                || ((first == NULL       ) && (second == 0)) );
    }

    /// Constructor from a two pointers (begin/end). Note: end points to the first char <em>behind</em> the string.
    BasicStrPtrLenPair(
      const CharT * paucStringBegin,
      const CharT * paucStringEnd
    ) :
        std::pair< CharT const * , size_t >(paucStringBegin, paucStringEnd - paucStringBegin )  //lint !e613: Possible use of null pointer 'paucStringEnd' in left argument to operator 'ptr-ptr'
    {
      assert(EXISTS(paucStringBegin));
      assert(EXISTS(paucStringEnd));
      assert(paucStringEnd >= paucStringBegin);
      assert(   (EXISTS(first) )
                || ((first == NULL       ) && (second == 0)) );
    }

    ///Constructor from basic_string<CharT>
    BasicStrPtrLenPair(
      const std::basic_string< CharT > & crclBasicString
    ) :
        std::pair< CharT const * , size_t >(crclBasicString.data(), crclBasicString.length()) {
      assert(   (EXISTS(first) )
                || ((first == NULL       ) && (second == 0)) );
    }

    ///Constructor from pair
    BasicStrPtrLenPair(
      std::pair< CharT const * , size_t > const & crclPair
    ) :  //lint !e1724: Argument to copy constructor for class 'uima::BasicStrPtrLenPair<<1>>' should be a const reference
        std::pair< CharT const * , size_t >(crclPair.first, crclPair.second) {
      assert(   (EXISTS(first) )
                || ((first == NULL       ) && (second == 0)) );
    }

    ///Accessor for the string length in logical characters
    size_t
    length( void ) const {
      return second;
    }

    ///Accessor for the string length in bytes
    size_t
    getSizeInBytes( void ) const {
      return (second * sizeof(CharT));
    }

    ///CONST Accessor for the string content (NOT ZERO DELIMITED!).
    const CharT *
    data( void ) const {
      return first;
    }

    ///CONST Accessor to the begin of string content (NOT ZERO DELIMITED!).
    const CharT *
    begin( void ) const {
      return (first);
    }

    ///Accessor to position AFTER the end of string content.
    const CharT *
    end( void ) const {
      return (first == NULL ? NULL : first + second);
    }

    /**
       Finds the first occurence of key character <TT>cPattern</TT> in the string.
    */
    size_t
    find( CharT cPattern ) const {
      return str_find_first( cPattern, first, second);
    }  //lint !e1746: parameter 'cPattern' in function 'BasicStrPtrLenPair<UChar>::find(UChar) const' could be made const reference

    /**
       Finds the first occurence of key string <TT>cpacPattern</TT> (with length
       <TT>uiPatternLen</TT>) in the string
    */
    size_t
    find(
      const BasicStrPtrLenPair< CharT > & crlstrPattern // pattern to search for
    ) const {
      return str_find_first( crlstrPattern.first, crlstrPattern.second,
                             first, second);
    }

    /**
       Finds the first occurence of key string <TT>cpacPattern</TT> (with length
       <TT>uiPatternLen</TT>) in the string
    */
    size_t
    find(
      const CharT * cpacPattern,   // pattern to search for
      size_t        uiPatternLen   // length of pattern
    ) const {
      return str_find_first( cpacPattern, uiPatternLen, first, second);
    }

    /**
       Finds the first occurence of key string <TT>cpacPattern</TT> (with length
       <TT>uiPatternLen</TT>), in the substring from <TT>uiStartPos</TT> to
       <TT>uiStartPos+uiStartLength</TT>
    */
    size_t
    find(
      const CharT * cpacPattern,   // pattern to search for
      size_t        uiPatternLen,  // length of pattern
      size_t        uiStartPos,    // from this pos
      size_t        /*uiStartLength*/  // up to uiStartPos+uiStartLength
    ) const {
      if ( uiStartPos >= second ) {
        return STRING_NPOS; // If search starts past end of str, indicate "not found".
      }

      assert(EXISTS(cpacPattern));
      assert(EXISTS(first));
      return str_find_first( cpacPattern, uiPatternLen,
                             (first+uiStartPos), second);
    }

    /** Return a sub-string of this string starting from position <TT>uiStartPos</TT>
        and including the following <TT>uiLength</TT> characters.
    */
    BasicStrPtrLenPair< CharT >
    sub_str(
      size_t uiStartPos,
      size_t uiLength
    ) {
      assert(uiStartPos < second);
      assert(uiStartPos + uiLength < second);
      assert(EXISTS(first));
      return BasicStrPtrLenPair< CharT >(first+uiStartPos, uiLength);
    }

    ///Set the string to new value. (used to be named <TT>set()</TT>)
    BasicStrPtrLenPair< CharT > &
    assign(
      const CharT * cpacString,
      size_t        uiLength
    ) {
      first = cpacString;
      second   = uiLength;
      assert(   (EXISTS(first) )
                || ((first == NULL       ) && (second == 0)) );

      return (*this);
    }

    ///Set the string to new value. (used to be names set)
    BasicStrPtrLenPair< CharT > &
    assign(
      const std::basic_string< CharT > & crclBasicString
    ) {
      first = crclBasicString.data();
      second   = crclBasicString.length();
      assert(   (EXISTS(first) )
                || ((first == NULL       ) && (second == 0)) );

      return (*this);
    }

    ///Assignment operator
    BasicStrPtrLenPair< CharT > &
    assign( const std::pair< CharT const *, size_t > & crclPair ) {
      first = crclPair.first;
      second = crclPair.second;
      return (*this);
    }

    /** Accessor for the string content (CharT dependant string return type).
    */
    std::basic_string< CharT >
    copyToBasicString(
      void
    ) const {
      return basic_string< CharT >(first, second);
    }

#ifdef NEVER
    /// convert to single byte string. crclCCSID specifies the target encoding)
    std::string
    prv_asSingleByteString(
      const uima::CCSID & crclCCSID
    ) const {
      if (sizeof(CharT) == 1) {  //lint !e774: Boolean within 'if' always evaluates to True
        // single byte lstrings
        return string((char*)data(), length());
      }
      if (length() == 0) {
        return string();
      }
      assert(sizeof(CharT) == 2); // unicode lstrings
      assert(EXISTS(data()));  //lint !e527 !e666: Expression with side effects passed to repeated parameter 1 in macro EXISTS
      // Small values are copied in a stack based buffer, larger are allocated
      // Max string length to handle stack based
      const size_t STACK_BUFF_LIMIT = 64;
      // Our stack buffer
      char   acStackBuff [STACK_BUFF_LIMIT];
      // A pointer to either the stack buffer of dynamic storage
      char * pcCharBuff;

      Unicode2CodePageConverter clConverter(crclCCSID);
      size_t uiMaxNewLength = clConverter.getMaximumSizeForLength(length());

      if (uiMaxNewLength < STACK_BUFF_LIMIT) {
        pcCharBuff = acStackBuff;     //use stack buffer
      } else {
        pcCharBuff = new char [uiMaxNewLength];  //allocate
      }
      // Now convert UChar into char array
      size_t uiCharsWritten = clConverter.convertCharacters(pcCharBuff, uiMaxNewLength, (const UChar*)data(), length());

      // Construct our string
      string strRetVal(pcCharBuff, uiCharsWritten);


      if (uiMaxNewLength >= STACK_BUFF_LIMIT) { // if allocated ...
        delete [] pcCharBuff; //lint !e673 Possibly inappropriate deallocation (delete[]) for 'auto' data
      }
      return strRetVal;
    }
#endif

    ///CONST Array Index Access operator
    const CharT &
    operator[]( size_t uiIndex ) const {
      assert(uiIndex < second);
      assert(EXISTS(first));
      return first[uiIndex];  //lint !e613: Possible use of null pointer 'BasicStrPtrLenPair<wchar_t>::first' in left argument to operator '['
    }


    ///Equality operator
    int
    operator==( const BasicStrPtrLenPair< CharT > & crclRHS ) const {
      if (second != crclRHS.second) {
        return false;
      }
      return strncmp_templ(first, crclRHS.first, second) == 0;
    }

    ///Assignment operator
    BasicStrPtrLenPair< CharT > &
    operator=( BasicStrPtrLenPair< CharT > const & crclRHS ) {
      first = crclRHS.first;
      second = crclRHS.second;
      return (*this);
    }

    ///Assignment operator
    BasicStrPtrLenPair< CharT > &
    operator=( std::pair< CharT const *, size_t > const & crclPair ) { //lint !e1520 !e1720 :multiple assignment ops  assignment operator for class 'uima::BasicStrPtrLenPair<<1>>' has non-const parameter
      first = crclPair.first;
      second = crclPair.second;
      return (*this);
    }

    ///less operator
    bool operator <( BasicStrPtrLenPair< CharT > const & crclRHS ) const {
      size_t uiLen1 = length();
      size_t uiLen2 = crclRHS.length();
      if (!(bool)uiLen2) {
        return(false);
      }
      if (!(bool)uiLen1) {
        return(true);
      }
      const CharT * cpszString1 = data();
      const CharT * cpszString2 = crclRHS.data();
      while ((bool)uiLen1 && (bool)uiLen2 && *cpszString1 == *cpszString2) {
        ++cpszString1;
        ++cpszString2;
        --uiLen1;
        --uiLen2;
      }
      if (!(bool)uiLen2) {
        return(false);
      }
      if (!(bool)uiLen1) {
        return(true);
      }
      return (*cpszString1 < *cpszString2);
    }

  };

///This defines the standard LString class with single byte character.
  typedef BasicStrPtrLenPair< char >    StrPtrLenPair;

///This defines the standard LString class with wide character.
  typedef BasicStrPtrLenPair< wchar_t > WStrPtrLenPair;

///This defines the wide LString class with wide/double byte character.
  typedef BasicStrPtrLenPair< UChar >   UStrPtrLenPair;

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template < class CharT >
  std::ostream &
  operator << (
    std::ostream &                           rclOStream,
    const BasicStrPtrLenPair< CharT > & crclLString
  );
#endif

#ifdef NEVER
///Output stream support for BasicStrPtrLenPair
  template < class CharT >
  inline std::ostream &
  operator << (
    std::ostream &                      rclOStream,
    const BasicStrPtrLenPair< CharT > & crclLString
  ) {
    if (rclOStream == cout || rclOStream == cerr) {  //lint !e1912: Implicit call of conversion function from class 'basic_ostream' to type 'void *'
      rclOStream << crclLString.prv_asSingleByteString(CosClCCSID::getConsoleCCSID()).c_str();
    } else {
      rclOStream << crclLString.prv_asSingleByteString(CosClCCSID::CosEnCCSID_UTF8).c_str();
    }
    return rclOStream;
  }


///Output stream support for pointer length pairs
  inline std::ostream &
  operator << (
    std::ostream &                             rclOStream,
    const std::pair< UChar const *, size_t > & crclPair
  ) {
    BasicStrPtrLenPair< UChar > const lString(crclPair);
    if (rclOStream == std::cout || rclOStream == std::cerr) {  //lint !e1912: Implicit call of conversion function from class 'basic_ostream' to type 'void *'
      rclOStream << lString.prv_asSingleByteString(CosClCCSID::getConsoleCCSID()).c_str();
    } else {
      rclOStream << lString.prv_asSingleByteString(CosClCCSID::CosEnCCSID_UTF8).c_str();
    }
    return rclOStream;
  }

#endif


  /* ----------------------------------------------------------------------- */
  /** @name vector to/from delimited string conversion routines              */
  /* ----------------------------------------------------------------------- */
  /*@{*/

  /**
     Removes whitespace from both ends of a string.
     Template function using <TT>isspace_templ()</TT>.
  */
  template < class CharT >
  inline BasicStrPtrLenPair< CharT >
  strtrim(
    const BasicStrPtrLenPair< CharT > & s
  ) {
    if (s.length() == 0) {
      return s;
    }
    const CharT * beg = s.data();
    const CharT * end = s.data()+s.length()-1;
    while (end >= beg && isspace_templ(*end) ) {
      --end;
    }
    while (beg < end && isspace_templ(*beg) ) {
      ++beg;
    }
    return BasicStrPtrLenPair< CharT >(beg, end-beg+1);
  }

  /**
     Splits a delimited string into pieces and stores the results in a vector
     of strings. Delimiters are passed as a zero terminated string.

     @param rveclstrOutput      (Output) The vector where the results are stored
     @param pcInput             The delimited string to split.
     @param uiInputLength       The number of chars in pcInput
     @param cpszDelimiters      The delimiters. CharT* are interpreted as a set of delimiters.
     @param bTrimString         Flag: If true, all pieces will be trimmed before storing in <TT>storeVar</TT>
     @param bInsertEmptyStrings Flag: If false, pieces that have length 0 will not be stored in  <TT>storeVar</TT>

     @return The number of strings added to <TT>rvecstrOutput</TT>
  */
  template < class CharT >
  inline size_t
  delimitedStrPtrLenPair2Vector(
    std::vector< uima::BasicStrPtrLenPair< CharT > > & rveclstrOutput,
    const CharT                           * pcInput,
    size_t                                  uiInputLength,
    const CharT                           * cpszDelimiters,
    bool                                    bTrimString,
    bool                                    bInsertEmptyStrings
  ) {
    const CharT * pcBegin = pcInput;
    size_t uiEnd;
    const CharT * pcEnd = pcBegin;
    size_t uiNumFound = 0;
    size_t uiDelimitersLen = strlen_templ(cpszDelimiters);

    if (uiInputLength == 0) {
      return 0;
    }
    const CharT * pcInputEnd = pcInput + uiInputLength;
    BasicStrPtrLenPair< CharT > _s;

    while (pcBegin < pcInputEnd) {
      //      uiBegin--;
      uiEnd   = str_find_first_of(cpszDelimiters, uiDelimitersLen, pcBegin, (size_t)(pcInputEnd-pcBegin));
      pcEnd = pcBegin+uiEnd;
      if (uiEnd != STRING_NPOS) {
        ++pcEnd;
      }
      if (uiEnd == STRING_NPOS) {
        uiEnd = uiInputLength+1;
        pcEnd = pcInputEnd+1;
      }
      assert(pcEnd > pcBegin);
      _s.assign(pcBegin, pcEnd-pcBegin-1);
      if (bTrimString) {
        _s = strtrim(_s);
      }
      if (bInsertEmptyStrings || _s.length() > 0) {
        rveclstrOutput.push_back(_s);
        uiNumFound++;
      }
      pcBegin = pcEnd;
    }
    return uiNumFound;
  }

  template < class CharT >
  inline size_t
  delimitedStrPtrLenPair2Vector(
    std::vector< BasicStrPtrLenPair< CharT > > & veclstrOutput,
    const CharT                           * pcInput,
    const CharT                           * cpszDelimiters,
    bool                                    bTrimString,
    bool                                    bInsertEmptyStrings
  ) {
    return delimitedStrPtrLenPair2Vector(veclstrOutput, pcInput, strlen_templ(pcInput), cpszDelimiters, bTrimString, bInsertEmptyStrings);
  }

//@}

} // namespace uima

#endif /* UIMA_STRPTRLENPAIR_HPP */

/* <EOF> */

