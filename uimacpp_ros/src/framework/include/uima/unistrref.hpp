#ifndef UIMA_UNICODESTRINGREF_HPP
#define UIMA_UNICODESTRINGREF_HPP
/** \file unistrref.hpp .
-----------------------------------------------------------------------------



           string interface of uima::UnicodeStringRef

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


    \brief  Shallow string object consisting of a pair of string pointer and a length

-----------------------------------------------------------------------------
*/

#include <uima/pragmas.hpp> //must be included first to disable warnings

#include <vector>
#include <string>
#include <iostream>

#include <uima/types.h>
#include <uima/assertmsg.h>
#include <uima/ccsid.hpp>
#include "unicode/unistr.h"
#include "unicode/ustring.h"
#include "unicode/uchar.h"
#include <uima/strtools.hpp>

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {

  /**
   * The class <TT>UnicodeStringRef</TT> provides support for non zero-terminated
   * strings that are presented as pointers to Unicode character arrays
   * with an associated length.
   * As this type of string is supposed to be used only as string reference into
   * read-only buffers, the string pointer is constant.
   * The member functions are named to implement the icu::UnicodeString interface
   * but only providing const member functions
   * This class is a quick ,light-weight, shallow string
   * (internally it consists only of a pointer and a length)
   * which can be copied by value without performance penalty.
   * It allows references into other string buffers to be treated like real
   * string objects.
   * Since it does not own it's string memory care must be taken to make sure
   * the lifetime of an UnicodeStringRef object does not exceed the lifetime
   * of the Unicode character buffer it references.
   */
  class UIMA_LINK_IMPORTSPEC UnicodeStringRef {
  public:
    /**
     * Default Constructor
     */
    UnicodeStringRef( void );

    /**
     * Constructor from icu::UnicodeString
     */
    UnicodeStringRef( const icu::UnicodeString & crUniString );

    /**
     * Constructor from zero terminated string
     */
    explicit UnicodeStringRef( UChar const * cpacString );

    /**
     * Constructor from string and length
     */
    UnicodeStringRef( UChar const * cpacString, int32_t uiLength );

    /**
     * Constructor from a two pointers (begin/end).
     * Note: end points to the first char <em>behind</em> the string.
     * @deprecated Replace with UnicodeStringRef(paucStringBegin,paucStringEnd-paucStringBegin).
     */
    UnicodeStringRef( UChar const * paucStringBegin, UChar const * paucStringEnd );

    ///Accessor for the number of bytes occupied by this string
    int32_t getSizeInBytes( void ) const;

    ///CONST Accessor for the string content (NOT ZERO DELIMITED!).
    UChar const * getBuffer( void ) const;

    ///Assignment operator
    UnicodeStringRef & operator=( UnicodeStringRef const & crclRHS );

    ///Equality operator
    int operator==( const UnicodeStringRef & crclRHS ) const;
    ///Inequality operator
    int operator!=( const UnicodeStringRef & crclRHS ) const;
    ///less operator
    bool operator< ( UnicodeStringRef const & text ) const;
    ///less equal operator
    bool operator<=( UnicodeStringRef const & text ) const;
    ///greater operator
    bool operator> ( UnicodeStringRef const & text ) const;
    ///greater equal operator
    bool operator>=( UnicodeStringRef const & text ) const;

    /**
     * Compare the characters bitwise in this UnicodeStringRef to
     * the characters in <TT>text</TT>.
     * @param text The UnicodeStringRef to compare to this one.
     * @return The result of bitwise character comparison: 0 if <TT>text</TT>
     * contains the same characters as this, -1 if the characters in
     * <TT>text</TT> are bitwise less than the characters in this, +1 if the
     * characters in <TT>text</TT> are bitwise greater than the characters
     * in this.
     * @stable
     */
    inline int8_t compare(const UnicodeStringRef& text) const;

    /**
     * Compare the characters bitwise in this UnicodeStringRef to
     * the characters in <TT>text</TT>.
     * @param text The icu::UnicodeString to compare to this one.
     * @return The result of bitwise character comparison: 0 if <TT>text</TT>
     * contains the same characters as this, -1 if the characters in
     * <TT>text</TT> are bitwise less than the characters in this, +1 if the
     * characters in <TT>text</TT> are bitwise greater than the characters
     * in this.
     * @stable
     */
    inline int8_t compare(const icu::UnicodeString& text) const;

    /**
     * Compare the characters bitwise in the range
     * [<TT>start</TT>, <TT>start + length</TT>) with the characters
     * in <TT>srcText</TT>
     * @param start the offset at which the compare operation begins
     * @param length the number of characters of text to compare.
     * @param srcText the text to be compared
     * @return The result of bitwise character comparison: 0 if <TT>text</TT>
     * contains the same characters as this, -1 if the characters in
     * <TT>text</TT> are bitwise less than the characters in this, +1 if the
     * characters in <TT>text</TT> are bitwise greater than the characters
     * in this.
     * @stable
     */
    inline int8_t compare(int32_t start,
                          int32_t length,
                          const UnicodeStringRef& srcText) const;

    /**
     * Compare the characters bitwise in the range
     * [<TT>start</TT>, <TT>start + length</TT>) with the characters
     * in <TT>srcText</TT> in the range
     * [<TT>srcStart</TT>, <TT>srcStart + srcLength</TT>).
     * @param start the offset at which the compare operation begins
     * @param length the number of characters in this to compare.
     * @param srcText the text to be compared
     * @param srcStart the offset into <TT>srcText</TT> to start comparison
     * @param srcLength the number of characters in <TT>src</TT> to compare
     * @return The result of bitwise character comparison: 0 if <TT>text</TT>
     * contains the same characters as this, -1 if the characters in
     * <TT>text</TT> are bitwise less than the characters in this, +1 if the
     * characters in <TT>text</TT> are bitwise greater than the characters
     * in this.
     * @stable
     */
    inline int8_t compare(int32_t start,
                          int32_t length,
                          const UnicodeStringRef& srcText,
                          int32_t srcStart,
                          int32_t srcLength) const;

    /**
     * Compare the characters bitwise in this UnicodeStringRef with the first
     * <TT>srcLength</TT> characters in <TT>srcChars</TT>.
     * @param srcChars The characters to compare to this UnicodeStringRef.
     * @param srcLength the number of characters in <TT>srcChars</TT> to compare
     * @return The result of bitwise character comparison: 0 if <TT>text</TT>
     * contains the same characters as this, -1 if the characters in
     * <TT>text</TT> are bitwise less than the characters in this, +1 if the
     * characters in <TT>text</TT> are bitwise greater than the characters
     * in this.
     * @stable
     */
    inline int8_t compare(UChar const *srcChars,
                          int32_t srcLength) const;

    /**
     * Compare the characters bitwise in the range
     * [<TT>start</TT>, <TT>start + length</TT>) with the first
     * <TT>length</TT> characters in <TT>srcChars</TT>
     * @param start the offset at which the compare operation begins
     * @param length the number of characters to compare.
     * @param srcChars the characters to be compared
     * @return The result of bitwise character comparison: 0 if <TT>text</TT>
     * contains the same characters as this, -1 if the characters in
     * <TT>text</TT> are bitwise less than the characters in this, +1 if the
     * characters in <TT>text</TT> are bitwise greater than the characters
     * in this.
     * @stable
     */
    inline int8_t compare(int32_t start,
                          int32_t length,
                          UChar const *srcChars) const;

    /**
     * Compare the characters bitwise in the range
     * [<TT>start</TT>, <TT>start + length</TT>) with the characters
     * in <TT>srcChars</TT> in the range
     * [<TT>srcStart</TT>, <TT>srcStart + srcLength</TT>).
     * @param start the offset at which the compare operation begins
     * @param length the number of characters in this to compare
     * @param srcChars the characters to be compared
     * @param srcStart the offset into <TT>srcChars</TT> to start comparison
     * @param srcLength the number of characters in <TT>srcChars</TT> to compare
     * @return The result of bitwise character comparison: 0 if <TT>text</TT>
     * contains the same characters as this, -1 if the characters in
     * <TT>text</TT> are bitwise less than the characters in this, +1 if the
     * characters in <TT>text</TT> are bitwise greater than the characters
     * in this.
     * @stable
     */
    inline int8_t compare(int32_t start,
                          int32_t length,
                          UChar const *srcChars,
                          int32_t srcStart,
                          int32_t srcLength) const;

    /**
     * Compare the characters bitwise in the range
     * [<TT>start</TT>, <TT>limit</TT>) with the characters
     * in <TT>srcText</TT> in the range
     * [<TT>srcStart</TT>, <TT>srcLimit</TT>).
     * @param start the offset at which the compare operation begins
     * @param limit the offset immediately following the compare operation
     * @param srcText the text to be compared
     * @param srcStart the offset into <TT>srcText</TT> to start comparison
     * @param srcLimit the offset into <TT>srcText</TT> to limit comparison
     * @return The result of bitwise character comparison: 0 if <TT>text</TT>
     * contains the same characters as this, -1 if the characters in
     * <TT>text</TT> are bitwise less than the characters in this, +1 if the
     * characters in <TT>text</TT> are bitwise greater than the characters
     * in this.
     * @stable
     */
    inline int8_t compareBetween(int32_t start,
                                 int32_t limit,
                                 const UnicodeStringRef& srcText,
                                 int32_t srcStart,
                                 int32_t srcLimit) const;

    /**
     * Compare two Unicode strings in code point order.
     * This is different in UTF-16 from how compare(), operator==, startsWith() etc. work
     * if supplementary characters are present:
     *
     * In UTF-16, supplementary characters (with code points U+10000 and above) are
     * stored with pairs of surrogate code units. These have values from 0xd800 to 0xdfff,
     * which means that they compare as less than some other BMP characters like U+feff.
     * This function compares Unicode strings in code point order.
     * If either of the UTF-16 strings is malformed (i.e., it contains unpaired surrogates), then the result is not defined.
     *
     * @param text Another string to compare this one to.
     * @return a negative/zero/positive integer corresponding to whether
     * this string is less than/equal to/greater than the second one
     * in code point order
     */
    inline int8_t compareCodePointOrder(const UnicodeStringRef& text) const;

    /**
     * Compare two Unicode strings in code point order.
     * This is different in UTF-16 from how compare(), operator==, startsWith() etc. work
     * if supplementary characters are present:
     *
     * In UTF-16, supplementary characters (with code points U+10000 and above) are
     * stored with pairs of surrogate code units. These have values from 0xd800 to 0xdfff,
     * which means that they compare as less than some other BMP characters like U+feff.
     * This function compares Unicode strings in code point order.
     * If either of the UTF-16 strings is malformed (i.e., it contains unpaired surrogates), then the result is not defined.
     *
     * @param start The start offset in this string at which the compare operation begins.
     * @param length The number of code units from this string to compare.
     * @param srcText Another string to compare this one to.
     * @return a negative/zero/positive integer corresponding to whether
     * this string is less than/equal to/greater than the second one
     * in code point order
     */
    inline int8_t compareCodePointOrder(int32_t start,
                                        int32_t length,
                                        const UnicodeStringRef& srcText) const;

    /**
     * Compare two Unicode strings in code point order.
     * This is different in UTF-16 from how compare(), operator==, startsWith() etc. work
     * if supplementary characters are present:
     *
     * In UTF-16, supplementary characters (with code points U+10000 and above) are
     * stored with pairs of surrogate code units. These have values from 0xd800 to 0xdfff,
     * which means that they compare as less than some other BMP characters like U+feff.
     * This function compares Unicode strings in code point order.
     * If either of the UTF-16 strings is malformed (i.e., it contains unpaired surrogates), then the result is not defined.
     *
     * @param start The start offset in this string at which the compare operation begins.
     * @param length The number of code units from this string to compare.
     * @param srcText Another string to compare this one to.
     * @param srcStart The start offset in that string at which the compare operation begins.
     * @param srcLength The number of code units from that string to compare.
     * @return a negative/zero/positive integer corresponding to whether
     * this string is less than/equal to/greater than the second one
     * in code point order
     */
    inline int8_t compareCodePointOrder(int32_t start,
                                        int32_t length,
                                        const UnicodeStringRef& srcText,
                                        int32_t srcStart,
                                        int32_t srcLength) const;

    /**
     * Compare two Unicode strings in code point order.
     * This is different in UTF-16 from how compare(), operator==, startsWith() etc. work
     * if supplementary characters are present:
     *
     * In UTF-16, supplementary characters (with code points U+10000 and above) are
     * stored with pairs of surrogate code units. These have values from 0xd800 to 0xdfff,
     * which means that they compare as less than some other BMP characters like U+feff.
     * This function compares Unicode strings in code point order.
     * If either of the UTF-16 strings is malformed (i.e., it contains unpaired surrogates), then the result is not defined.
     *
     * @param srcChars A pointer to another string to compare this one to.
     * @param srcLength The number of code units from that string to compare.
     * @return a negative/zero/positive integer corresponding to whether
     * this string is less than/equal to/greater than the second one
     * in code point order
     */
    inline int8_t compareCodePointOrder(UChar const *srcChars,
                                        int32_t srcLength) const;

    /**
     * Compare two Unicode strings in code point order.
     * This is different in UTF-16 from how compare(), operator==, startsWith() etc. work
     * if supplementary characters are present:
     *
     * In UTF-16, supplementary characters (with code points U+10000 and above) are
     * stored with pairs of surrogate code units. These have values from 0xd800 to 0xdfff,
     * which means that they compare as less than some other BMP characters like U+feff.
     * This function compares Unicode strings in code point order.
     * If either of the UTF-16 strings is malformed (i.e., it contains unpaired surrogates), then the result is not defined.
     *
     * @param start The start offset in this string at which the compare operation begins.
     * @param length The number of code units from this string to compare.
     * @param srcChars A pointer to another string to compare this one to.
     * @return a negative/zero/positive integer corresponding to whether
     * this string is less than/equal to/greater than the second one
     * in code point order
     */
    inline int8_t compareCodePointOrder(int32_t start,
                                        int32_t length,
                                        UChar const *srcChars) const;

    /**
     * Compare two Unicode strings in code point order.
     * This is different in UTF-16 from how compare(), operator==, startsWith() etc. work
     * if supplementary characters are present:
     *
     * In UTF-16, supplementary characters (with code points U+10000 and above) are
     * stored with pairs of surrogate code units. These have values from 0xd800 to 0xdfff,
     * which means that they compare as less than some other BMP characters like U+feff.
     * This function compares Unicode strings in code point order.
     * If either of the UTF-16 strings is malformed (i.e., it contains unpaired surrogates), then the result is not defined.
     *
     * @param start The start offset in this string at which the compare operation begins.
     * @param length The number of code units from this string to compare.
     * @param srcChars A pointer to another string to compare this one to.
     * @param srcStart The start offset in that string at which the compare operation begins.
     * @param srcLength The number of code units from that string to compare.
     * @return a negative/zero/positive integer corresponding to whether
     * this string is less than/equal to/greater than the second one
     * in code point order
     */
    inline int8_t compareCodePointOrder(int32_t start,
                                        int32_t length,
                                        UChar const *srcChars,
                                        int32_t srcStart,
                                        int32_t srcLength) const;

    /**
     * Compare two Unicode strings in code point order.
     * This is different in UTF-16 from how compare(), operator==, startsWith() etc. work
     * if supplementary characters are present:
     *
     * In UTF-16, supplementary characters (with code points U+10000 and above) are
     * stored with pairs of surrogate code units. These have values from 0xd800 to 0xdfff,
     * which means that they compare as less than some other BMP characters like U+feff.
     * This function compares Unicode strings in code point order.
     * If either of the UTF-16 strings is malformed (i.e., it contains unpaired surrogates), then the result is not defined.
     *
     * @param start The start offset in this string at which the compare operation begins.
     * @param limit The offset after the last code unit from this string to compare.
     * @param srcText Another string to compare this one to.
     * @param srcStart The start offset in that string at which the compare operation begins.
     * @param srcLimit The offset after the last code unit from that string to compare.
     * @return a negative/zero/positive integer corresponding to whether
     * this string is less than/equal to/greater than the second one
     * in code point order
     */
    inline int8_t compareCodePointOrderBetween(int32_t start,
        int32_t limit,
        const UnicodeStringRef& srcText,
        int32_t srcStart,
        int32_t srcLimit) const;

    /**
     * Compare two strings case-insensitively using full case folding.
     * This is equivalent to this->foldCase(options).compare(text.foldCase(options)).
     *
     * @param text Another string to compare this one to.
     * @param options Either U_FOLD_CASE_DEFAULT or U_FOLD_CASE_EXCLUDE_SPECIAL_I
     * @return A negative, zero, or positive integer indicating the comparison result.
     */
    inline int8_t caseCompare(const UnicodeStringRef& text, uint32_t options) const;

    /**
     * Compare two strings case-insensitively using full case folding.
     * This is equivalent to this->foldCase(options).compare(srcText.foldCase(options)).
     *
     * @param start The start offset in this string at which the compare operation begins.
     * @param length The number of code units from this string to compare.
     * @param srcText Another string to compare this one to.
     * @param options Either U_FOLD_CASE_DEFAULT or U_FOLD_CASE_EXCLUDE_SPECIAL_I
     * @return A negative, zero, or positive integer indicating the comparison result.
     */
    inline int8_t caseCompare(int32_t start,
                              int32_t length,
                              const UnicodeStringRef& srcText,
                              uint32_t options) const;

    /**
     * Compare two strings case-insensitively using full case folding.
     * This is equivalent to this->foldCase(options).compare(srcText.foldCase(options)).
     *
     * @param start The start offset in this string at which the compare operation begins.
     * @param length The number of code units from this string to compare.
     * @param srcText Another string to compare this one to.
     * @param srcStart The start offset in that string at which the compare operation begins.
     * @param srcLength The number of code units from that string to compare.
     * @param options Either U_FOLD_CASE_DEFAULT or U_FOLD_CASE_EXCLUDE_SPECIAL_I
     * @return A negative, zero, or positive integer indicating the comparison result.
     */
    inline int8_t caseCompare(int32_t start,
                              int32_t length,
                              const UnicodeStringRef& srcText,
                              int32_t srcStart,
                              int32_t srcLength,
                              uint32_t options) const;

    /**
     * Compare two strings case-insensitively using full case folding.
     * This is equivalent to this->foldCase(options).compare(srcChars.foldCase(options)).
     *
     * @param srcChars A pointer to another string to compare this one to.
     * @param srcLength The number of code units from that string to compare.
     * @param options Either U_FOLD_CASE_DEFAULT or U_FOLD_CASE_EXCLUDE_SPECIAL_I
     * @return A negative, zero, or positive integer indicating the comparison result.
     */
    inline int8_t caseCompare(UChar const *srcChars,
                              int32_t srcLength,
                              uint32_t options) const;

    /**
     * Compare two strings case-insensitively using full case folding.
     * This is equivalent to this->foldCase(options).compare(srcChars.foldCase(options)).
     *
     * @param start The start offset in this string at which the compare operation begins.
     * @param length The number of code units from this string to compare.
     * @param srcChars A pointer to another string to compare this one to.
     * @param options Either U_FOLD_CASE_DEFAULT or U_FOLD_CASE_EXCLUDE_SPECIAL_I
     * @return A negative, zero, or positive integer indicating the comparison result.
     */
    inline int8_t caseCompare(int32_t start,
                              int32_t length,
                              UChar const *srcChars,
                              uint32_t options) const;

    /**
     * Compare two strings case-insensitively using full case folding.
     * This is equivalent to this->foldCase(options).compare(srcChars.foldCase(options)).
     *
     * @param start The start offset in this string at which the compare operation begins.
     * @param length The number of code units from this string to compare.
     * @param srcChars A pointer to another string to compare this one to.
     * @param srcStart The start offset in that string at which the compare operation begins.
     * @param srcLength The number of code units from that string to compare.
     * @param options Either U_FOLD_CASE_DEFAULT or U_FOLD_CASE_EXCLUDE_SPECIAL_I
     * @return A negative, zero, or positive integer indicating the comparison result.
     */
    inline int8_t caseCompare(int32_t start,
                              int32_t length,
                              UChar const *srcChars,
                              int32_t srcStart,
                              int32_t srcLength,
                              uint32_t options) const;

    /**
     * Compare two strings case-insensitively using full case folding.
     * This is equivalent to this->foldCase(options).compareBetween(text.foldCase(options)).
     *
     * @param start The start offset in this string at which the compare operation begins.
     * @param limit The offset after the last code unit from this string to compare.
     * @param srcText Another string to compare this one to.
     * @param srcStart The start offset in that string at which the compare operation begins.
     * @param srcLimit The offset after the last code unit from that string to compare.
     * @param options Either U_FOLD_CASE_DEFAULT or U_FOLD_CASE_EXCLUDE_SPECIAL_I
     * @return A negative, zero, or positive integer indicating the comparison result.
     */
    inline int8_t caseCompareBetween(int32_t start,
                                     int32_t limit,
                                     const UnicodeStringRef& srcText,
                                     int32_t srcStart,
                                     int32_t srcLimit,
                                     uint32_t options) const;

    /**
     * Determine if this starts with the characters in <TT>text</TT>
     * @param text The text to match.
     * @return TRUE if this starts with the characters in <TT>text</TT>,
     * FALSE otherwise
     * @stable
     */
    inline bool startsWith(const UnicodeStringRef& text) const;

    /**
     * Determine if this starts with the characters in <TT>srcText</TT>
     * in the range [<TT>srcStart</TT>, <TT>srcStart + srcLength</TT>).
     * @param srcText The text to match.
     * @param srcStart the offset into <TT>srcText</TT> to start matching
     * @param srcLength the number of characters in <TT>srcText</TT> to match
     * @return TRUE if this starts with the characters in <TT>text</TT>,
     * FALSE otherwise
     * @stable
     */
    inline bool startsWith(const UnicodeStringRef& srcText,
                           int32_t srcStart,
                           int32_t srcLength) const;

    /**
     * Determine if this starts with the characters in <TT>srcChars</TT>
     * @param srcChars The characters to match.
     * @param srcLength the number of characters in <TT>srcChars</TT>
     * @return TRUE if this starts with the characters in <TT>srcChars</TT>,
     * FALSE otherwise
     * @stable
     */
    inline bool startsWith(UChar const *srcChars,
                           int32_t srcLength) const;

    /**
     * Determine if this starts with the characters in <TT>srcChars</TT>
     * in the range [<TT>srcStart</TT>, <TT>srcStart + srcLength</TT>).
     * @param srcChars The characters to match.
     * @param srcStart the offset into <TT>srcText</TT> to start matching
     * @param srcLength the number of characters in <TT>srcChars</TT> to match
     * @return TRUE if this starts with the characters in <TT>srcChars</TT>,
     * FALSE otherwise
     * @stable
     */
    inline bool startsWith(UChar const *srcChars,
                           int32_t srcStart,
                           int32_t srcLength) const;

    /**
     * Determine if this ends with the characters in <TT>text</TT>
     * @param text The text to match.
     * @return TRUE if this ends with the characters in <TT>text</TT>,
     * FALSE otherwise
     * @stable
     */
    inline bool endsWith(const UnicodeStringRef& text) const;

    /**
     * Determine if this ends with the characters in <TT>srcText</TT>
     * in the range [<TT>srcStart</TT>, <TT>srcStart + srcLength</TT>).
     * @param srcText The text to match.
     * @param srcStart the offset into <TT>srcText</TT> to start matching
     * @param srcLength the number of characters in <TT>srcText</TT> to match
     * @return TRUE if this ends with the characters in <TT>text</TT>,
     * FALSE otherwise
     * @stable
     */
    inline bool endsWith(const UnicodeStringRef& srcText,
                         int32_t srcStart,
                         int32_t srcLength) const;

    /**
     * Determine if this ends with the characters in <TT>srcChars</TT>
     * @param srcChars The characters to match.
     * @param srcLength the number of characters in <TT>srcChars</TT>
     * @return TRUE if this ends with the characters in <TT>srcChars</TT>,
     * FALSE otherwise
     * @stable
     */
    inline bool endsWith(UChar const *srcChars,
                         int32_t srcLength) const;

    /**
     * Determine if this ends with the characters in <TT>srcChars</TT>
     * in the range [<TT>srcStart</TT>, <TT>srcStart + srcLength</TT>).
     * @param srcChars The characters to match.
     * @param srcStart the offset into <TT>srcText</TT> to start matching
     * @param srcLength the number of characters in <TT>srcChars</TT> to match
     * @return TRUE if this ends with the characters in <TT>srcChars</TT>,
     * FALSE otherwise
     * @stable
     */
    inline bool endsWith(UChar const *srcChars,
                         int32_t srcStart,
                         int32_t srcLength) const;


    /* Searching - bitwise only */

    /**
     * Locate in this the first occurrence of the characters in <TT>text</TT>,
     * using bitwise comparison.
     * @param text The text to search for.
     * @return The offset into this of the start of <TT>text</TT>,
     * or -1 if not found.
     * @stable
     */
    inline int32_t indexOf(const UnicodeStringRef& text) const;

    /**
     * Locate in this the first occurrence of the characters in <TT>text</TT>
     * starting at offset <TT>start</TT>, using bitwise comparison.
     * @param text The text to search for.
     * @param start The offset at which searching will start.
     * @return The offset into this of the start of <TT>text</TT>,
     * or -1 if not found.
     * @stable
     */
    inline int32_t indexOf(const UnicodeStringRef& text,
                           int32_t start) const;

    /**
     * Locate in this the first occurrence in the range
     * [<TT>start</TT>, <TT>start + length</TT>) of the characters
     * in <TT>text</TT>, using bitwise comparison.
     * @param text The text to search for.
     * @param start The offset at which searching will start.
     * @param length The number of characters to search
     * @return The offset into this of the start of <TT>text</TT>,
     * or -1 if not found.
     * @stable
     */
    inline int32_t indexOf(const UnicodeStringRef& text,
                           int32_t start,
                           int32_t length) const;

    /**
     * Locate in this the first occurrence in the range
     * [<TT>start</TT>, <TT>start + length</TT>) of the characters
     *  in <TT>srcText</TT> in the range
     * [<TT>srcStart</TT>, <TT>srcStart + srcLength</TT>),
     * using bitwise comparison.
     * @param srcText The text to search for.
     * @param srcStart the offset into <TT>srcText</TT> at which
     * to start matching
     * @param srcLength the number of characters in <TT>srcText</TT> to match
     * @param start the offset into this at which to start matching
     * @param length the number of characters in this to search
     * @return The offset into this of the start of <TT>text</TT>,
     * or -1 if not found.
     * @stable
     */
    inline int32_t indexOf(const UnicodeStringRef& srcText,
                           int32_t srcStart,
                           int32_t srcLength,
                           int32_t start,
                           int32_t length) const;

    /**
     * Locate in this the first occurrence of the characters in
     * <TT>srcChars</TT>
     * starting at offset <TT>start</TT>, using bitwise comparison.
     * @param srcChars The text to search for.
     * @param srcLength the number of characters in <TT>srcChars</TT> to match
     * @param start the offset into this at which to start matching
     * @return The offset into this of the start of <TT>text</TT>,
     * or -1 if not found.
     * @stable
     */
    inline int32_t indexOf(UChar const *srcChars,
                           int32_t srcLength,
                           int32_t start) const;

    /**
     * Locate in this the first occurrence in the range
     * [<TT>start</TT>, <TT>start + length</TT>) of the characters
     * in <TT>srcChars</TT>, using bitwise comparison.
     * @param srcChars The text to search for.
     * @param srcLength the number of characters in <TT>srcChars</TT>
     * @param start The offset at which searching will start.
     * @param length The number of characters to search
     * @return The offset into this of the start of <TT>srcChars</TT>,
     * or -1 if not found.
     * @stable
     */
    inline int32_t indexOf(UChar const *srcChars,
                           int32_t srcLength,
                           int32_t start,
                           int32_t length) const;

    /**
     * Locate in this the first occurrence in the range
     * [<TT>start</TT>, <TT>start + length</TT>) of the characters
     * in <TT>srcChars</TT> in the range
     * [<TT>srcStart</TT>, <TT>srcStart + srcLength</TT>),
     * using bitwise comparison.
     * @param srcChars The text to search for.
     * @param srcStart the offset into <TT>srcChars</TT> at which
     * to start matching
     * @param srcLength the number of characters in <TT>srcChars</TT> to match
     * @param start the offset into this at which to start matching
     * @param length the number of characters in this to search
     * @return The offset into this of the start of <TT>text</TT>,
     * or -1 if not found.
     * @stable
     */
    int32_t indexOf(UChar const *srcChars,
                    int32_t srcStart,
                    int32_t srcLength,
                    int32_t start,
                    int32_t length) const;

    /**
     * Locate in this the first occurrence of the code unit <TT>c</TT>,
     * using bitwise comparison.
     * @param c The code unit to search for.
     * @return The offset into this of <TT>c</TT>, or -1 if not found.
     * @stable
     */
    inline int32_t indexOf(UChar c) const;

    /**
     * Locate in this the first occurrence of the code point <TT>c</TT>,
     * using bitwise comparison.
     * @param c The code point to search for.
     * @return The offset into this of <TT>c</TT>, or -1 if not found.
     * @stable
     */
    inline int32_t indexOf(UChar32 c) const;

    /**
     * Locate in this the first occurrence of the code unit <TT>c</TT>
     * starting at offset <TT>start</TT>, using bitwise comparison.
     * @param c The code unit to search for.
     * @param start The offset at which searching will start.
     * @return The offset into this of <TT>c</TT>, or -1 if not found.
     * @stable
     */
    inline int32_t indexOf(UChar c,
                           int32_t start) const;

    /**
     * Locate in this the first occurrence of the code point <TT>c</TT>
     * starting at offset <TT>start</TT>, using bitwise comparison.
     * @param c The code point to search for.
     * @param start The offset at which searching will start.
     * @return The offset into this of <TT>c</TT>, or -1 if not found.
     * @stable
     */
    inline int32_t indexOf(UChar32 c,
                           int32_t start) const;

    /**
     * Locate in this the first occurrence of the code unit <TT>c</TT>
     * in the range [<TT>start</TT>, <TT>start + length</TT>),
     * using bitwise comparison.
     * @param c The code unit to search for.
     * @param start the offset into this at which to start matching
     * @param length the number of characters in this to search
     * @return The offset into this of <TT>c</TT>, or -1 if not found.
     * @stable
     */
    inline int32_t indexOf(UChar c,
                           int32_t start,
                           int32_t length) const;

    /**
     * Locate in this the first occurrence of the code point <TT>c</TT>
     * in the range [<TT>start</TT>, <TT>start + length</TT>),
     * using bitwise comparison.
     * @param c The code point to search for.
     * @param start the offset into this at which to start matching
     * @param length the number of characters in this to search
     * @return The offset into this of <TT>c</TT>, or -1 if not found.
     * @stable
     */
    inline int32_t indexOf(UChar32 c,
                           int32_t start,
                           int32_t length) const;

    /**
     * Locate in this the last occurrence of the characters in <TT>text</TT>,
     * using bitwise comparison.
     * @param text The text to search for.
     * @return The offset into this of the start of <TT>text</TT>,
     * or -1 if not found.
     * @stable
     */
    inline int32_t lastIndexOf(const UnicodeStringRef& text) const;

    /**
     * Locate in this the last occurrence of the characters in <TT>text</TT>
     * starting at offset <TT>start</TT>, using bitwise comparison.
     * @param text The text to search for.
     * @param start The offset at which searching will start.
     * @return The offset into this of the start of <TT>text</TT>,
     * or -1 if not found.
     * @stable
     */
    inline int32_t lastIndexOf(const UnicodeStringRef& text,
                               int32_t start) const;

    /**
     * Locate in this the last occurrence in the range
     * [<TT>start</TT>, <TT>start + length</TT>) of the characters
     * in <TT>text</TT>, using bitwise comparison.
     * @param text The text to search for.
     * @param start The offset at which searching will start.
     * @param length The number of characters to search
     * @return The offset into this of the start of <TT>text</TT>,
     * or -1 if not found.
     * @stable
     */
    inline int32_t lastIndexOf(const UnicodeStringRef& text,
                               int32_t start,
                               int32_t length) const;

    /**
     * Locate in this the last occurrence in the range
     * [<TT>start</TT>, <TT>start + length</TT>) of the characters
     * in <TT>srcText</TT> in the range
     * [<TT>srcStart</TT>, <TT>srcStart + srcLength</TT>),
     * using bitwise comparison.
     * @param srcText The text to search for.
     * @param srcStart the offset into <TT>srcText</TT> at which
     * to start matching
     * @param srcLength the number of characters in <TT>srcText</TT> to match
     * @param start the offset into this at which to start matching
     * @param length the number of characters in this to search
     * @return The offset into this of the start of <TT>text</TT>,
     * or -1 if not found.
     * @stable
     */
    inline int32_t lastIndexOf(const UnicodeStringRef& srcText,
                               int32_t srcStart,
                               int32_t srcLength,
                               int32_t start,
                               int32_t length) const;

    /**
     * Locate in this the last occurrence of the characters in <TT>srcChars</TT>
     * starting at offset <TT>start</TT>, using bitwise comparison.
     * @param srcChars The text to search for.
     * @param srcLength the number of characters in <TT>srcChars</TT> to match
     * @param start the offset into this at which to start matching
     * @return The offset into this of the start of <TT>text</TT>,
     * or -1 if not found.
     * @stable
     */
    inline int32_t lastIndexOf(UChar const *srcChars,
                               int32_t srcLength,
                               int32_t start) const;

    /**
     * Locate in this the last occurrence in the range
     * [<TT>start</TT>, <TT>start + length</TT>) of the characters
     * in <TT>srcChars</TT>, using bitwise comparison.
     * @param srcChars The text to search for.
     * @param srcLength the number of characters in <TT>srcChars</TT>
     * @param start The offset at which searching will start.
     * @param length The number of characters to search
     * @return The offset into this of the start of <TT>srcChars</TT>,
     * or -1 if not found.
     * @stable
     */
    inline int32_t lastIndexOf(UChar const *srcChars,
                               int32_t srcLength,
                               int32_t start,
                               int32_t length) const;

    /**
     * Locate in this the last occurrence in the range
     * [<TT>start</TT>, <TT>start + length</TT>) of the characters
     * in <TT>srcChars</TT> in the range
     * [<TT>srcStart</TT>, <TT>srcStart + srcLength</TT>),
     * using bitwise comparison.
     * @param srcChars The text to search for.
     * @param srcStart the offset into <TT>srcChars</TT> at which
     * to start matching
     * @param srcLength the number of characters in <TT>srcChars</TT> to match
     * @param start the offset into this at which to start matching
     * @param length the number of characters in this to search
     * @return The offset into this of the start of <TT>text</TT>,
     * or -1 if not found.
     * @stable
     */
    int32_t lastIndexOf(UChar const *srcChars,
                        int32_t srcStart,
                        int32_t srcLength,
                        int32_t start,
                        int32_t length) const;

    /**
     * Locate in this the last occurrence of the code unit <TT>c</TT>,
     * using bitwise comparison.
     * @param c The code unit to search for.
     * @return The offset into this of <TT>c</TT>, or -1 if not found.
     * @stable
     */
    inline int32_t lastIndexOf(UChar c) const;

    /**
     * Locate in this the last occurrence of the code point <TT>c</TT>,
     * using bitwise comparison.
     * @param c The code point to search for.
     * @return The offset into this of <TT>c</TT>, or -1 if not found.
     * @stable
     */
    inline int32_t lastIndexOf(UChar32 c) const;

    /**
     * Locate in this the last occurrence of the code unit <TT>c</TT>
     * starting at offset <TT>start</TT>, using bitwise comparison.
     * @param c The code unit to search for.
     * @param start The offset at which searching will start.
     * @return The offset into this of <TT>c</TT>, or -1 if not found.
     * @stable
     */
    inline int32_t lastIndexOf(UChar c,
                               int32_t start) const;

    /**
     * Locate in this the last occurrence of the code point <TT>c</TT>
     * starting at offset <TT>start</TT>, using bitwise comparison.
     * @param c The code point to search for.
     * @param start The offset at which searching will start.
     * @return The offset into this of <TT>c</TT>, or -1 if not found.
     * @stable
     */
    inline int32_t lastIndexOf(UChar32 c,
                               int32_t start) const;

    /**
     * Locate in this the last occurrence of the code unit <TT>c</TT>
     * in the range [<TT>start</TT>, <TT>start + length</TT>),
     * using bitwise comparison.
     * @param c The code unit to search for.
     * @param start the offset into this at which to start matching
     * @param length the number of characters in this to search
     * @return The offset into this of <TT>c</TT>, or -1 if not found.
     * @stable
     */
    inline int32_t lastIndexOf(UChar c,
                               int32_t start,
                               int32_t length) const;

    /**
     * Locate in this the last occurrence of the code point <TT>c</TT>
     * in the range [<TT>start</TT>, <TT>start + length</TT>),
     * using bitwise comparison.
     * @param c The code point to search for.
     * @param start the offset into this at which to start matching
     * @param length the number of characters in this to search
     * @return The offset into this of <TT>c</TT>, or -1 if not found.
     * @stable
     */
    inline int32_t lastIndexOf(UChar32 c,
                               int32_t start,
                               int32_t length) const;


    /* Character access */

    /**
     * Return the code unit at offset <tt>offset</tt>.
     * @param offset a valid offset into the text
     * @returns the code unit at offset <tt>offset</tt>
     * @stable
     */
    inline UChar charAt(int32_t offset) const;

    /**
     * Return the code unit at offset <tt>offset</tt>.
     * @param offset a valid offset into the text
     * @returns the code unit at offset <tt>offset</tt>
     * @stable
     */
    inline UChar operator [] (int32_t offset) const;

    /**
     * Return the code point that contains the code unit
     * at offset <tt>offset</tt>.
     * @param offset a valid offset into the text
     * that indicates the text offset of any of the code units
     * that will be assembled into a code point (21-bit value) and returned
     * @returns the code point of text at <tt>offset</tt>
     * @stable
     */
    inline UChar32 char32At(int32_t offset) const;

    /**
     * Adjust a random-access offset so that
     * it points to the beginning of a Unicode character.
     * The offset that is passed in points to
     * any code unit of a code point,
     * while the returned offset will point to the first code unit
     * of the same code point.
     * In UTF-16, if the input offset points to a iv_uiLength surrogate
     * of a surrogate pair, then the returned offset will point
     * to the first surrogate.
     * @param offset a valid offset into one code point of the text
     * @return offset of the first code unit of the same code point
     */
    inline int32_t getChar32Start(int32_t offset) const;

    /**
     * Adjust a random-access offset so that
     * it points behind a Unicode character.
     * The offset that is passed in points behind
     * any code unit of a code point,
     * while the returned offset will point behind the last code unit
     * of the same code point.
     * In UTF-16, if the input offset points behind the first surrogate
     * (i.e., to the iv_uiLength surrogate)
     * of a surrogate pair, then the returned offset will point
     * behind the iv_uiLength surrogate (i.e., to the first surrogate).
     * @param offset a valid offset after any code unit of a code point of the text
     * @return offset of the first code unit after the same code point
     */
    inline int32_t getChar32Limit(int32_t offset) const;

    /**
     * Move the code unit index along the string by delta code points.
     * Interpret the input index as a code unit-based offset into the string,
     * move the index forward or backward by delta code points, and
     * return the resulting index.
     * The input index should point to the first code unit of a code point,
     * if there is more than one.
     *
     * Both input and output indexes are code unit-based as for all
     * string indexes/offsets in ICU (and other libraries, like MBCS char*).
     * If delta<0 then the index is moved backward (toward the start of the string).
     * If delta>0 then the index is moved forward (toward the end of the string).
     *
     * This behaves like CharacterIterator::move32(delta, kCurrent).
     *
     * Examples:
     * <code>
     * // s has code points 'a' U+10000 'b' U+10ffff U+2029
     * UnicodeStringRef s=UNICODE_STRING("a\\U00010000b\\U0010ffff\\u2029", 31).unescape();
     *
     * // initial index: position of U+10000
     * int32_t index=1;
     *
     * // the following examples will all result in index==4, position of U+10ffff
     *
     * // skip 2 code points from some position in the string
     * index=s.moveIndex32(index, 2); // skips U+10000 and 'b'
     *
     * // go to the 3rd code point from the start of s (0-based)
     * index=s.moveIndex32(0, 3); // skips 'a', U+10000, and 'b'
     *
     * // go to the next-to-last code point of s
     *
     * index=s.moveIndex32(s.length(), -2); // backward-skips U+2029 and U+10ffff
     * </code>
     *
     * @param index input code unit index
     * @param delta (signed) code point count to move the index forward or backward
     *        in the string
     * @return the resulting code unit index
     */
    int32_t moveIndex32(int32_t index, int32_t delta) const;

    /* Substring extraction without conversion */

    /**
     * Copy the characters in the range
     * [<tt>start</tt>, <tt>start + length</tt>) into the array <tt>dst</tt>,
     * beginning at <tt>dstStart</tt>.
     * If the string aliases to <code>dst</code> itself as an external buffer,
     * then extract() will not copy the contents.
     *
     * @param start offset of first character which will be copied into the array
     * @param length the number of characters to extract
     * @param dst array in which to copy characters.  The length of <tt>dst</tt>
     * must be at least (<tt>dstStart + length</tt>).
     * @param dstStart the offset in <TT>dst</TT> where the first character
     * will be extracted
     * @stable
     */
    inline void extract(int32_t start,
                        int32_t length,
                        UChar *dst,
                        int32_t dstStart = 0) const;

    /**
     * Copy the characters in the range [<tt>start</tt>, <tt>limit</tt>)
     * into the array <tt>dst</tt>, beginning at <tt>dstStart</tt>.
     * @param start offset of first character which will be copied into the array
     * @param limit offset immediately following the last character to be copied
     * @param dst array in which to copy characters.  The length of <tt>dst</tt>
     * must be at least (<tt>dstStart + (limit - start)</tt>).
     * @param dstStart the offset in <TT>dst</TT> where the first character
     * will be extracted
     * @stable
     */
    inline void extractBetween(int32_t start,
                               int32_t limit,
                               UChar *dst,
                               int32_t dstStart = 0) const;

    /**
     * Copy the contents of the string into dst.
     * This is a convenience function that
     * checks if there is enough space in dst,
     * extracts the entire string if possible,
     * and NUL-terminates dst if possible.
     *
     * If the string fits into dst but cannot be NUL-terminated
     * (length()==dstCapacity) then the error code is set to U_STRING_NOT_TERMINATED_WARNING.
     * If the string itself does not fit into dst
     * (length()>dstCapacity) then the error code is set to U_BUFFER_OVERFLOW_ERROR.
     *
     * If the string aliases to <code>dst</code> itself as an external buffer,
     * then extract() will not copy the contents.
     *
     * @param dst Destination string buffer.
     * @param dstCapacity Number of UChars available at dst.
     * @param errorCode ICU error code.
     * @return length()
     */
    int32_t
    extract(UChar *dst, int32_t dstCapacity,
            UErrorCode &errorCode) const;

    /**
     * Copy the characters in the range
     * [<tt>start</tt>, <tt>start + length</tt>) into the  UnicodeString
     * <tt>dst</tt>.
     * @param start offset of first character which will be copied
     * @param length the number of characters to extract
     * @param dst icu::UnicodeString into which to copy characters.
     * @return A reference to <TT>dst</TT>
     * @stable
     */
    inline void extract(int32_t start,
                        int32_t length,
                        icu::UnicodeString& dst) const;

    /**
     * Copy the characters in the range [<tt>start</tt>, <tt>limit</tt>)
     * into the icu::UnicodeString <tt>dst</tt>.
     * @param start offset of first character which will be copied
     * @param limit offset immediately following the last character to be copied
     * @param dst icu::UnicodeString into which to copy characters.
     * @return A reference to <TT>dst</TT>
     * @stable
     */
    inline void extractBetween(int32_t start,
                               int32_t limit,
                               icu::UnicodeString& dst) const;

    /* Substring extraction with conversion */

    /**
     * Copy the characters in the range
     * [<tt>start</TT>, <tt>start + length</TT>) into an array of characters
     * in a specified codepage.
     * The output string is NUL-terminated.
     *
     * @param start offset of first character which will be copied
     * @param startLength the number of characters to extract
     * @param target the target buffer for extraction
     * @param codepage the desired codepage for the characters.  0 has
     * the special meaning of the default codepage
     * If <code>codepage</code> is an empty string (<code>""</code>),
     * then a simple conversion is performed on the codepage-invariant
     * subset ("invariant characters") of the platform encoding. See utypes.h.
     * If <TT>target</TT> is NULL, then the number of bytes required for
     * <TT>target</TT> is returned.
     * NOTE: It is assumed that the target is big enough to fit all of the characters.
     * @return the output string length, not including the terminating NUL
     * @stable
     */
    inline int32_t extract(int32_t start,
                           int32_t startLength,
                           char *target,
                           const char *codepage = 0) const;

    /**
     * Copy the characters in the range
     * [<tt>start</TT>, <tt>start + length</TT>) into an array of characters
     * in a specified codepage.
     * This function does not write any more than <code>targetLength</code>
     * characters but returns the length of the entire output string
     * so that one can allocate a larger buffer and call the function again
     * if necessary.
     * The output string is NUL-terminated if possible.
     *
     * @param start offset of first character which will be copied
     * @param startLength the number of characters to extract
     * @param target the target buffer for extraction
     * @param targetLength the length of the target buffer
     * @param codepage the desired codepage for the characters.  0 has
     * the special meaning of the default codepage
     * If <code>codepage</code> is an empty string (<code>""</code>),
     * then a simple conversion is performed on the codepage-invariant
     * subset ("invariant characters") of the platform encoding. See utypes.h.
     * If <TT>target</TT> is NULL, then the number of bytes required for
     * <TT>target</TT> is returned.
     * @return the output string length, not including the terminating NUL
     * @stable
     */
    int32_t extract(int32_t start,
                    int32_t startLength,
                    char *target,
                    uint32_t targetLength,
                    const char *codepage = 0) const;

    /**
     * Convert the UnicodeStringRef into a codepage string using an existing UConverter.
     * The output string is NUL-terminated if possible.
     *
     * This function avoids the overhead of opening and closing a converter if
     * multiple strings are extracted.
     *
     * @param target destination string buffer, can be NULL if targetCapacity==0
     * @param targetCapacity the number of chars available at target
     * @param cnv the converter object to be used (ucnv_resetFromUnicode() will be called),
     *        or NULL for the default converter
     * @param errorCode normal ICU error code
     * @return the length of the output string, not counting the terminating NUL;
     *         if the length is greater than targetCapacity, then the string will not fit
     *         and a buffer of the indicated length would need to be passed in
     * @stable
     */
    int32_t extract(char *target, int32_t targetCapacity,
                    UConverter *cnv,
                    UErrorCode &errorCode) const;

    /**
     * Copy the characters in the range
     * [<tt>start</TT>, <tt>start + length</TT>) into a std::string object
     * in a specified codepage.
     * The output string is NUL-terminated.
     *
     * @param start offset of first character which will be copied
     * @param startLength the number of characters to extract
     * @param target the target string for extraction
     * @param codepage the desired codepage for the characters.  0 has
     * the special meaning of the default codepage.
     * If <code>codepage</code> is an empty string (<code>""</code>),
     * then a simple conversion is performed on the codepage-invariant
     * subset ("invariant characters") of the platform encoding. See utypes.h.
     * @return the output string length, not including the terminating NUL
     * @stable
     */
    int32_t extract(int32_t start,
                    int32_t startLength,
                    std::string & target,
                    const char *codepage = 0) const;

    /**
     * Copy all the characters in the string into an std::string object
     * in a specified codepage.  Equivalent to 
     * extract(0, length(), target, codepage)
     *
     * @param target the target string for extraction
     * @param codepage the desired codepage for the characters.
     * @return the output string length, not including the terminating NUL
     * @stable
     */
    inline int32_t extract(std::string & target,
                           const char *codepage = 0) const;

    /**
     * Copy all the characters in the string into an std::string object
     * in UTF-8.  Slightly more efficient than asUTF8() as avoids
     * one copy.
     *
     * @param target the target string for extraction
     * @return the output string length, not including the terminating NUL
     */
    int32_t extractUTF8(std::string & target) const;

    /**
     * Convert to a UTF8 string
     * @return a std::string
     */
    inline std::string asUTF8(void) const;

    /**
     * Release contents of string container allocated by extract methods
     * Useful when caller and callee use different heaps, 
     * e.g. when debug code uses a release library.
     * Is static so can be called on the <TT>UnicodeStringRef</TT> class directly.
     */
    static void release(std::string & target);

    /* Length operations */

    /**
     * Return the length of the UnicodeStringRef object.
     * The length is the number of characters in the text.
     * @returns the length of the UnicodeStringRef object
     * @stable
     */
    inline int32_t  length(void) const;

    /**
     * Count Unicode code points in the length UChar code units of the string.
     * A code point may occupy either one or two UChar code units.
     * Counting code points involves reading all code units.
     *
     * This functions is basically the inverse of moveIndex32().
     *
     * @param start the index of the first code unit to check
     * @param length the number of UChar code units to check
     * @return the number of code points in the specified code units
     */
    int32_t
    countChar32(int32_t start=0, int32_t length=0x7fffffff) const;

    /**
     * Determine if this string is empty.
     * @return TRUE if this string contains 0 characters, FALSE otherwise.
     */
    inline bool isEmpty(void) const;

    /**
     * Set the text in the icu::UnicodeString object to the characters in
     * <TT>srcText</TT>.
     * <TT>srcText</TT> is not modified.
     * @param srcText the source for the new characters
     * @return a reference to this
     * @stable
     */
    inline UnicodeStringRef& setTo(const UnicodeStringRef& srcText);

    /**
     * Set the text in the icu::UnicodeString object to the characters in
     * <TT>srcText</TT>.
     * <TT>srcText</TT> is not modified.
     * @param srcText the source for the new characters
     * @return a reference to this
     * @stable
     */
    inline UnicodeStringRef& setTo(const icu::UnicodeString& srcText);

    /**
     * Set the characters in the icu::UnicodeString object to the characters
     * in <TT>srcChars</TT>. <TT>srcChars</TT> is not modified.
     * @param srcChars the source for the new characters
     * @param srcLength the number of Unicode characters in srcChars.
     * @return a reference to this
     * @stable
     */
    inline UnicodeStringRef& setTo(const UChar *srcChars,
                                   int32_t srcLength);
    /**
     * Print a single byte version to outStream.
     * The encoding is UTF-8 if outStream is directed to disk,
     * if outStream is cout our cerr the encoding is a Console-CCSID
     * that will allow most character to be readable in a shell/command window.
     */
    void toSingleByteStream(std::ostream & outStream) const;



  private:
    /* --- functions -------------------------------------------------------- */

    inline int8_t
    doCompare( int32_t start,
               int32_t length,
               const UnicodeStringRef& srcText,
               int32_t srcStart,
               int32_t srcLength) const;

    int8_t
    doCompare( int32_t start,
               int32_t length,
               const UChar *srcChars,
               int32_t srcStart,
               int32_t srcLength) const;
    inline int8_t
    doCompareCodePointOrder(int32_t start,
                            int32_t length,
                            const UnicodeStringRef& srcText,
                            int32_t srcStart,
                            int32_t srcLength) const;
    int8_t
    doCompareCodePointOrder(int32_t start,
                            int32_t length,
                            const UChar *srcChars,
                            int32_t srcStart,
                            int32_t srcLength) const;
    inline int8_t
    doCaseCompare(int32_t start,
                  int32_t length,
                  const UnicodeStringRef& srcText,
                  int32_t srcStart,
                  int32_t srcLength,
                  uint32_t options) const;

    int8_t
    doCaseCompare(int32_t start,
                  int32_t length,
                  const UChar *srcChars,
                  int32_t srcStart,
                  int32_t srcLength,
                  uint32_t options) const;
    int32_t doIndexOf(UChar c,
                      int32_t start,
                      int32_t length) const;
    int32_t doLastIndexOf(UChar c,
                          int32_t start,
                          int32_t length) const;

    inline void doExtract(int32_t start,
                          int32_t length,
                          UChar *dst,
                          int32_t dstStart) const;
    inline void doExtract(int32_t start,
                          int32_t length,
                          icu::UnicodeString& dst) const;

    inline void
    pinIndices(int32_t& start,
               int32_t& length) const;
    // constants
    enum {
      kInvalidUChar=0xffff // invalid UChar index
    };
    /* --- variables -------------------------------------------------------- */
    UChar const * iv_pUChars;
    int32_t        iv_uiLength;
  }
  ;  // class UnicodeStringRef

  ///Output stream support for UnicodeStringRef (Note: inside namespace)
  UIMA_LINK_IMPORTSPEC std::ostream &
  operator << (
    std::ostream                & outStream,
    const uima::UnicodeStringRef & crUStrRef
  );
} // namespace uima


/* ----------------------------------------------------------------------- */
/*  Implementation UnicodeStringRef                                        */
/* ----------------------------------------------------------------------- */

namespace uima {

  inline
  UnicodeStringRef::UnicodeStringRef( void ) :
      iv_pUChars(NULL),
      iv_uiLength(0) {}

  inline UnicodeStringRef::UnicodeStringRef(
    const icu::UnicodeString & crUniString
  ) :
      iv_pUChars(crUniString.getBuffer()),
      iv_uiLength(crUniString.length()) {}

  inline
  UnicodeStringRef::UnicodeStringRef(
    UChar const * cpacString
  ) :
      iv_pUChars(cpacString),
      iv_uiLength(cpacString==NULL ? 0 : u_strlen(cpacString)) {
    assert(   (EXISTS(iv_pUChars) )
              || ((iv_pUChars == NULL       ) && (iv_uiLength == 0)) );
  }

  inline
  UnicodeStringRef::UnicodeStringRef(
    UChar const * cpacString,
    int32_t        uiLength
  ) :
  iv_pUChars(cpacString),
  iv_uiLength(uiLength) {
    assert(   (EXISTS(iv_pUChars) )
              || ((iv_pUChars == NULL       ) && (iv_uiLength == 0)) );
  }

  inline
  UnicodeStringRef::UnicodeStringRef(
    UChar const * paucStringBegin,
    UChar const * paucStringEnd
  ) :
      iv_pUChars(paucStringBegin),
      iv_uiLength(paucStringEnd - paucStringBegin) {
    assert(EXISTS(paucStringBegin));
    assert(EXISTS(paucStringEnd));
    assert(paucStringEnd >= paucStringBegin);
    assert(   (EXISTS(iv_pUChars) )
              || ((iv_pUChars == NULL) && (iv_uiLength == 0)) );
  }

  inline int32_t
  UnicodeStringRef::length( void ) const {
    return iv_uiLength;
  }

  inline int32_t
  UnicodeStringRef::getSizeInBytes( void ) const {
    return (iv_uiLength * sizeof(UChar));
  }

  inline UChar
  UnicodeStringRef::operator[]( int32_t uiIndex ) const {
    assert(uiIndex < iv_uiLength);
    assert(EXISTS(iv_pUChars));
    return iv_pUChars[uiIndex];  //lint !e613: Possible use of null pointer 'UnicodeStringRef<wchar_t>::iv_pUChars' in left argument to operator '['
  }

  inline int
  UnicodeStringRef::operator==( const UnicodeStringRef & crclRHS ) const {
    if (iv_uiLength != crclRHS.iv_uiLength) {
      return false;
    }
    return u_strncmp(iv_pUChars, crclRHS.iv_pUChars, iv_uiLength) == 0;
  }

  inline int
  UnicodeStringRef::operator!=( const UnicodeStringRef & crclRHS ) const {
    return !((*this)==crclRHS);
  }

  inline UnicodeStringRef &
  UnicodeStringRef::operator=( UnicodeStringRef const & crclRHS ) {
    iv_pUChars = crclRHS.iv_pUChars;
    iv_uiLength = crclRHS.iv_uiLength;
    return (*this);
  }

//========================================
// Read-only alias methods
//========================================
  inline void
  UnicodeStringRef::pinIndices(int32_t& start,
                               int32_t& length) const {
    // pin indices
    if (start > iv_uiLength) {
      start = iv_uiLength;
    }
    if (length > (iv_uiLength - start)) {
      length = (iv_uiLength - start);
    }
  }

  inline bool
  UnicodeStringRef::operator> (const UnicodeStringRef& text) const {
    return doCompare(0, iv_uiLength, text, 0, text.iv_uiLength) == 1;
  }

  inline bool
  UnicodeStringRef::operator< (const UnicodeStringRef& text) const {
    return doCompare(0, iv_uiLength, text, 0, text.iv_uiLength) == -1;
  }

  inline bool
  UnicodeStringRef::operator>= (const UnicodeStringRef& text) const {
    return doCompare(0, iv_uiLength, text, 0, text.iv_uiLength) != -1;
  }

  inline bool
  UnicodeStringRef::operator<= (const UnicodeStringRef& text) const {
    return doCompare(0, iv_uiLength, text, 0, text.iv_uiLength) != 1;
  }

  inline int8_t
  UnicodeStringRef::compare(const UnicodeStringRef& text) const {
    return doCompare(0, iv_uiLength, text, 0, text.iv_uiLength);
  }

  inline int8_t
  UnicodeStringRef::compare(int32_t start,
                            int32_t length,
                            const UnicodeStringRef& srcText) const {
    return doCompare(start, length, srcText, 0, srcText.iv_uiLength);
  }

  inline int8_t
  UnicodeStringRef::compare(const UChar *srcChars,
                            int32_t srcLength) const {
    return doCompare(0, iv_uiLength, srcChars, 0, srcLength);
  }


  inline int8_t
  UnicodeStringRef::compare(icu::UnicodeString const  &src ) const {
    return doCompare(0, iv_uiLength, src.getBuffer(), 0, src.length());
  }


  inline int8_t
  UnicodeStringRef::compare(int32_t start,
                            int32_t length,
                            const UChar *srcChars) const {
    return doCompare(start, length, srcChars, 0, length);
  }

  inline int8_t
  UnicodeStringRef::compare(int32_t start,
                            int32_t length,
                            const UChar *srcChars,
                            int32_t srcStart,
                            int32_t srcLength) const {
    return doCompare(start, length, srcChars, srcStart, srcLength);
  }

  inline int8_t
  UnicodeStringRef::compare(int32_t start,
                            int32_t length,
                            const UnicodeStringRef& srcText,
                            int32_t srcStart,
                            int32_t srcLength) const {
    return doCompare(start, length, srcText, srcStart, srcLength);
  }

  inline int8_t
  UnicodeStringRef::compareBetween(int32_t start,
                                   int32_t limit,
                                   const UnicodeStringRef& srcText,
                                   int32_t srcStart,
                                   int32_t srcLimit) const {
    return doCompare(start, limit - start,
                     srcText, srcStart, srcLimit - srcStart);
  }

  inline int8_t
  UnicodeStringRef::doCompare(int32_t start,
                              int32_t length,
                              const UnicodeStringRef& srcText,
                              int32_t srcStart,
                              int32_t srcLength) const {
    const UChar *srcChars = srcText.getBuffer();
    return doCompare(start, length, srcChars, srcStart, srcLength);
  }

  inline int8_t
  UnicodeStringRef::compareCodePointOrder(const UnicodeStringRef& text) const {
    return doCompareCodePointOrder(0, iv_uiLength, text, 0, text.iv_uiLength);
  }

  inline int8_t
  UnicodeStringRef::compareCodePointOrder(int32_t start,
                                          int32_t length,
                                          const UnicodeStringRef& srcText) const {
    return doCompareCodePointOrder(start, length, srcText, 0, srcText.iv_uiLength);
  }

  inline int8_t
  UnicodeStringRef::compareCodePointOrder(const UChar *srcChars,
                                          int32_t srcLength) const {
    return doCompareCodePointOrder(0, iv_uiLength, srcChars, 0, srcLength);
  }

  inline int8_t
  UnicodeStringRef::compareCodePointOrder(int32_t start,
                                          int32_t length,
                                          const UnicodeStringRef& srcText,
                                          int32_t srcStart,
                                          int32_t srcLength) const {
    return doCompareCodePointOrder(start, length, srcText, srcStart, srcLength);
  }

  inline int8_t
  UnicodeStringRef::compareCodePointOrder(int32_t start,
                                          int32_t length,
                                          const UChar *srcChars) const {
    return doCompareCodePointOrder(start, length, srcChars, 0, length);
  }

  inline int8_t
  UnicodeStringRef::compareCodePointOrder(int32_t start,
                                          int32_t length,
                                          const UChar *srcChars,
                                          int32_t srcStart,
                                          int32_t srcLength) const {
    return doCompareCodePointOrder(start, length, srcChars, srcStart, srcLength);
  }

  inline int8_t
  UnicodeStringRef::compareCodePointOrderBetween(int32_t start,
      int32_t limit,
      const UnicodeStringRef& srcText,
      int32_t srcStart,
      int32_t srcLimit) const {
    return doCompareCodePointOrder(start, limit - start,
                                   srcText, srcStart, srcLimit - srcStart);
  }

  inline int8_t
  UnicodeStringRef::doCompareCodePointOrder(int32_t start,
      int32_t length,
      const UnicodeStringRef& srcText,
      int32_t srcStart,
      int32_t srcLength) const {
    const UChar *srcChars = srcText.getBuffer();
    return doCompareCodePointOrder(start, length, srcChars, srcStart, srcLength);
  }

  inline int8_t
  UnicodeStringRef::caseCompare(const UnicodeStringRef &text, uint32_t options) const {
    return doCaseCompare(0, iv_uiLength, text, 0, text.iv_uiLength, options);
  }

  inline int8_t
  UnicodeStringRef::caseCompare(int32_t start,
                                int32_t length,
                                const UnicodeStringRef &srcText,
                                uint32_t options) const {
    return doCaseCompare(start, length, srcText, 0, srcText.iv_uiLength, options);
  }

  inline int8_t
  UnicodeStringRef::caseCompare(const UChar *srcChars,
                                int32_t srcLength,
                                uint32_t options) const {
    return doCaseCompare(0, iv_uiLength, srcChars, 0, srcLength, options);
  }

  inline int8_t
  UnicodeStringRef::caseCompare(int32_t start,
                                int32_t length,
                                const UnicodeStringRef &srcText,
                                int32_t srcStart,
                                int32_t srcLength,
                                uint32_t options) const {
    return doCaseCompare(start, length, srcText, srcStart, srcLength, options);
  }

  inline int8_t
  UnicodeStringRef::caseCompare(int32_t start,
                                int32_t length,
                                const UChar *srcChars,
                                uint32_t options) const {
    return doCaseCompare(start, length, srcChars, 0, length, options);
  }

  inline int8_t
  UnicodeStringRef::caseCompare(int32_t start,
                                int32_t length,
                                const UChar *srcChars,
                                int32_t srcStart,
                                int32_t srcLength,
                                uint32_t options) const {
    return doCaseCompare(start, length, srcChars, srcStart, srcLength, options);
  }

  inline int8_t
  UnicodeStringRef::caseCompareBetween(int32_t start,
                                       int32_t limit,
                                       const UnicodeStringRef &srcText,
                                       int32_t srcStart,
                                       int32_t srcLimit,
                                       uint32_t options) const {
    return doCaseCompare(start, limit - start, srcText, srcStart, srcLimit - srcStart, options);
  }

  inline int8_t
  UnicodeStringRef::doCaseCompare(int32_t start,
                                  int32_t length,
                                  const UnicodeStringRef &srcText,
                                  int32_t srcStart,
                                  int32_t srcLength,
                                  uint32_t options) const {
    const UChar *srcChars = srcText.getBuffer();
    return doCaseCompare(start, length, srcChars, srcStart, srcLength, options);
  }

  inline int32_t
  UnicodeStringRef::indexOf(const UnicodeStringRef& text) const {
    return indexOf(text, 0, text.iv_uiLength, 0, iv_uiLength);
  }

  inline int32_t
  UnicodeStringRef::indexOf(const UnicodeStringRef& text,
                            int32_t start) const {
    return indexOf(text, 0, text.iv_uiLength, start, iv_uiLength - start);
  }

  inline int32_t
  UnicodeStringRef::indexOf(const UnicodeStringRef& text,
                            int32_t start,
                            int32_t length) const {
    return indexOf(text, 0, text.iv_uiLength, start, length);
  }

  inline int32_t
  UnicodeStringRef::indexOf(const UnicodeStringRef& srcText,
                            int32_t srcStart,
                            int32_t srcLength,
                            int32_t start,
                            int32_t length) const {
    return indexOf(srcText.getBuffer(), srcStart, srcLength, start, length);
  }

  inline int32_t
  UnicodeStringRef::indexOf(const UChar *srcChars,
                            int32_t srcLength,
                            int32_t start) const {
    return indexOf(srcChars, 0, srcLength, start, iv_uiLength - start);
  }

  inline int32_t
  UnicodeStringRef::indexOf(const UChar *srcChars,
                            int32_t srcLength,
                            int32_t start,
                            int32_t length) const {
    return indexOf(srcChars, 0, srcLength, start, length);
  }

  inline int32_t
  UnicodeStringRef::indexOf(UChar c) const {
    return doIndexOf(c, 0, iv_uiLength);
  }

  inline int32_t
  UnicodeStringRef::indexOf(UChar32 c) const {
    if (!UTF_NEED_MULTIPLE_UCHAR(c)) {
      return doIndexOf((UChar)c, 0, iv_uiLength);
    } else {
      UChar buffer[UTF_MAX_CHAR_LENGTH];
      int32_t length = 0;
      UTF_APPEND_CHAR_UNSAFE(buffer, length, c);
      return indexOf(buffer, length, 0);
    }
  }

  inline int32_t
  UnicodeStringRef::indexOf(UChar c,
                            int32_t start) const {
    return doIndexOf(c, start, iv_uiLength - start);
  }

  inline int32_t
  UnicodeStringRef::indexOf(UChar32 c,
                            int32_t start) const {
    if (!UTF_NEED_MULTIPLE_UCHAR(c)) {
      return doIndexOf((UChar)c, start, iv_uiLength - start);
    } else {
      UChar buffer[UTF_MAX_CHAR_LENGTH];
      int32_t length = 0;
      UTF_APPEND_CHAR_UNSAFE(buffer, length, c);
      return indexOf(buffer, length, start);
    }
  }

  inline int32_t
  UnicodeStringRef::indexOf(UChar c,
                            int32_t start,
                            int32_t length) const {
    return doIndexOf(c, start, length);
  }

  inline int32_t
  UnicodeStringRef::indexOf(UChar32 c,
                            int32_t start,
                            int32_t length) const {
    if (!UTF_NEED_MULTIPLE_UCHAR(c)) {
      return doIndexOf((UChar)c, start, length);
    } else {
      UChar buffer[UTF_MAX_CHAR_LENGTH];
      int32_t cLength = 0;
      UTF_APPEND_CHAR_UNSAFE(buffer, cLength, c);
      return indexOf(buffer, cLength, start, length);
    }
  }

  inline int32_t
  UnicodeStringRef::lastIndexOf(const UnicodeStringRef& text) const {
    return lastIndexOf(text, 0, text.iv_uiLength, 0, iv_uiLength);
  }

  inline int32_t
  UnicodeStringRef::lastIndexOf(const UnicodeStringRef& text,
                                int32_t start) const {
    return lastIndexOf(text, 0, text.iv_uiLength, start, iv_uiLength - start);
  }

  inline int32_t
  UnicodeStringRef::lastIndexOf(const UnicodeStringRef& text,
                                int32_t start,
                                int32_t length) const {
    return lastIndexOf(text, 0, text.iv_uiLength, start, length);
  }

  inline int32_t
  UnicodeStringRef::lastIndexOf(const UnicodeStringRef& srcText,
                                int32_t srcStart,
                                int32_t srcLength,
                                int32_t start,
                                int32_t length) const {
    return lastIndexOf(srcText.getBuffer(), srcStart, srcLength, start, length);
  }

  inline int32_t
  UnicodeStringRef::lastIndexOf(const UChar *srcChars,
                                int32_t srcLength,
                                int32_t start) const {
    return lastIndexOf(srcChars, 0, srcLength, start, iv_uiLength - start);
  }

  inline int32_t
  UnicodeStringRef::lastIndexOf(const UChar *srcChars,
                                int32_t srcLength,
                                int32_t start,
                                int32_t length) const {
    return lastIndexOf(srcChars, 0, srcLength, start, length);
  }

  inline int32_t
  UnicodeStringRef::lastIndexOf(UChar c) const {
    return doLastIndexOf(c, 0, iv_uiLength);
  }

  inline int32_t
  UnicodeStringRef::lastIndexOf(UChar32 c) const {
    if (!UTF_NEED_MULTIPLE_UCHAR(c)) {
      return doLastIndexOf((UChar)c, 0, iv_uiLength);
    } else {
      UChar buffer[UTF_MAX_CHAR_LENGTH];
      int32_t count = 0;
      UTF_APPEND_CHAR_UNSAFE(buffer, count, c);
      return lastIndexOf(buffer, count, 0);
    }
  }

  inline int32_t
  UnicodeStringRef::lastIndexOf(UChar c,
                                int32_t start) const {
    return doLastIndexOf(c, start, iv_uiLength - start);
  }

  inline int32_t
  UnicodeStringRef::lastIndexOf(UChar32 c,
                                int32_t start) const {
    if (!UTF_NEED_MULTIPLE_UCHAR(c)) {
      return doLastIndexOf((UChar)c, start, iv_uiLength - start);
    } else {
      UChar buffer[UTF_MAX_CHAR_LENGTH];
      int32_t count = 0;
      UTF_APPEND_CHAR_UNSAFE(buffer, count, c);
      return lastIndexOf(buffer, count, start);
    }
  }

  inline int32_t
  UnicodeStringRef::lastIndexOf(UChar c,
                                int32_t start,
                                int32_t length) const {
    return doLastIndexOf(c, start, length);
  }

  inline int32_t
  UnicodeStringRef::lastIndexOf(UChar32 c,
                                int32_t start,
                                int32_t length) const {
    if (!UTF_NEED_MULTIPLE_UCHAR(c)) {
      return doLastIndexOf((UChar)c, start, length);
    } else {
      UChar buffer[UTF_MAX_CHAR_LENGTH];
      int32_t count = 0;
      UTF_APPEND_CHAR_UNSAFE(buffer, count, c);
      return lastIndexOf(buffer, count, start, length);
    }
  }

  inline bool
  UnicodeStringRef::startsWith(const UnicodeStringRef& text) const {
    return compare(0, text.iv_uiLength, text, 0, text.iv_uiLength) == 0;
  }

  inline bool
  UnicodeStringRef::startsWith(const UnicodeStringRef& srcText,
                               int32_t srcStart,
                               int32_t srcLength) const {
    return doCompare(0, srcLength, srcText, srcStart, srcLength) == 0;
  }

  inline bool
  UnicodeStringRef::startsWith(const UChar *srcChars,
                               int32_t srcLength) const {
    return doCompare(0, srcLength, srcChars, 0, srcLength) == 0;
  }

  inline bool
  UnicodeStringRef::startsWith(const UChar *srcChars,
                               int32_t srcStart,
                               int32_t srcLength) const {
    return doCompare(0, srcLength, srcChars, srcStart, srcLength) == 0;
  }

  inline bool
  UnicodeStringRef::endsWith(const UnicodeStringRef& text) const {
    return doCompare(iv_uiLength - text.iv_uiLength, text.iv_uiLength,
                     text, 0, text.iv_uiLength) == 0;
  }

  inline bool
  UnicodeStringRef::endsWith(const UnicodeStringRef& srcText,
                             int32_t srcStart,
                             int32_t srcLength) const {
    return doCompare(iv_uiLength - srcLength, srcLength,
                     srcText, srcStart, srcLength) == 0;
  }

  inline bool
  UnicodeStringRef::endsWith(const UChar *srcChars,
                             int32_t srcLength) const {
    return doCompare(iv_uiLength - srcLength, srcLength,
                     srcChars, 0, srcLength) == 0;
  }

  inline bool
  UnicodeStringRef::endsWith(const UChar *srcChars,
                             int32_t srcStart,
                             int32_t srcLength) const {
    return doCompare(iv_uiLength - srcLength, srcLength,
                     srcChars, srcStart, srcLength) == 0;
  }

// ============================
// extract implementations (some in .cpp)
// ============================
  inline void
  UnicodeStringRef::extract(int32_t start,
                            int32_t length,
                            UChar *dst,
                            int32_t dstStart) const {
    pinIndices(start, length);
    memcpy(dst+dstStart, getBuffer()+start, length*sizeof(UChar));
  }


  inline void
  UnicodeStringRef::extract(int32_t start,
                            int32_t length,
                            icu::UnicodeString& target) const {
    target.replace(0, target.length(), getBuffer(), start, length);
  }
// Replaces all of target by substring of src
// Could use setTo(getBuffer()+start,length) but that is implemented as a replace

  inline void
  UnicodeStringRef::extractBetween(int32_t start,
                                   int32_t limit,
                                   UChar *dst,
                                   int32_t dstStart) const {
    extract(start, limit - start, dst, dstStart);
  }

  inline void
  UnicodeStringRef::extractBetween(int32_t start,
                                   int32_t limit,
                                   icu::UnicodeString& dst) const {
    extract(start, limit - start, dst);
  }



  inline int32_t
  UnicodeStringRef::extract(int32_t start,
                            int32_t length,
                            char *target,
                            const char *codepage) const {
    // User-beware ... assumes target buffer is large enough
    // Capacity assumed to be either large, or 0 if no buffer provided (pre-flighting)
    return extract(start, length, target, target!=0 ? 0xffffffff : 0, codepage);
  }

  inline int32_t
  UnicodeStringRef::extract(std::string & target,
                            const char *codepage) const {
    return extract(0, iv_uiLength, target, codepage);
  }

  inline std::string
  UnicodeStringRef::asUTF8(void) const {
    std::string target;
    extractUTF8(target);
    return target;
  }

  inline UChar
  UnicodeStringRef::charAt(int32_t offset) const {
    assert(EXISTS(iv_pUChars));
    if ((uint32_t)offset < (uint32_t)iv_uiLength) {
      return iv_pUChars[offset];
    } else {
      return kInvalidUChar;
    }
  }

  inline UChar32
  UnicodeStringRef::char32At(int32_t offset) const {
    if ((uint32_t)offset < (uint32_t)iv_uiLength) {
      UChar32 c;
      UTF_GET_CHAR(iv_pUChars, 0, offset, iv_uiLength, c);
      return c;
    } else {
      return kInvalidUChar;
    }
  }

  inline int32_t
  UnicodeStringRef::getChar32Start(int32_t offset) const {
    if ((uint32_t)offset < (uint32_t)iv_uiLength) {
      UTF_SET_CHAR_START(iv_pUChars, 0, offset);
      return offset;
    } else {
      return 0;
    }
  }

  inline int32_t
  UnicodeStringRef::getChar32Limit(int32_t offset) const {
    if ((uint32_t)offset < (uint32_t)iv_uiLength) {
      UTF_SET_CHAR_LIMIT(iv_pUChars, 0, offset, iv_uiLength);
      return offset;
    } else {
      return iv_uiLength;
    }
  }

  inline bool
  UnicodeStringRef::isEmpty() const {
    return iv_uiLength == 0;
  }

  inline UChar const *
  UnicodeStringRef::getBuffer() const {
    return iv_pUChars;
  }

  inline int8_t
  UnicodeStringRef::doCaseCompare(int32_t start,
                                  int32_t length,
                                  const UChar *srcChars,
                                  int32_t srcStart,
                                  int32_t srcLength,
                                  uint32_t options) const {
    icu::UnicodeString s(iv_pUChars+start, (int32_t)length);
    return s.caseCompare(srcChars + srcStart, (int32_t)srcLength, options);
  }

  inline UnicodeStringRef& UnicodeStringRef::setTo(const UnicodeStringRef& srcText) {
    iv_pUChars  = srcText.iv_pUChars;
    iv_uiLength = srcText.iv_uiLength;
    return (*this);
  }

  inline UnicodeStringRef& UnicodeStringRef::setTo(const icu::UnicodeString& srcText) {
    iv_pUChars  = srcText.getBuffer();
    iv_uiLength = srcText.length();
    return (*this);
  }

  inline UnicodeStringRef& UnicodeStringRef::setTo(const UChar *srcChars, int32_t srcLength) {
    iv_pUChars  = srcChars;
    iv_uiLength = srcLength;
    return (*this);
  }

  UIMA_LINK_IMPORTSPEC std::ostream &
  operator << (
    std::ostream           & rclOStream,
    const UnicodeStringRef & crclLString
  );



  /* ----------------------------------------------------------------------- */
  /** @name vector to/from delimited string conversion routines              */
  /* ----------------------------------------------------------------------- */
  /*@{*/

  /**
     Removes whitespace from both ends of a string.
     Template function using <TT>isspace_templ()</TT>.
  */
  inline UnicodeStringRef
  strtrim(
    const UnicodeStringRef & s
  ) {
    if (s.length() == 0) {
      return s;
    }
    UChar const * beg = s.getBuffer();
    UChar const * end = s.getBuffer()+s.length()-1;
    while (end >= beg && u_isspace(*end) ) {
      --end;
    }
    while (beg < end && u_isspace(*beg) ) {
      ++beg;
    }
    return UnicodeStringRef(beg, end-beg+1);
  }

  /**
     Splits a delimited string into pieces and stores the results in a vector
     of strings. Delimiters are passed as a zero terminated string.

     @param rveclstrOutput      (Output) The vector where the results are stored
     @param pcInput             The delimited string to split.
     @param uiInputLength       The number of chars in pcInput
     @param cpszDelimiters      The delimiters. UChar* are interpreted as a set of delimiters.
     @param bTrimString         Flag: If true, all pieces will be trimmed before storing in <TT>storeVar</TT>
     @param bInsertEmptyStrings Flag: If false, pieces that have length 0 will not be stored in  <TT>storeVar</TT>

     @return The number of strings added to <TT>rvecstrOutput</TT>
  */
  UIMA_LINK_IMPORTSPEC int32_t
  delimitedUnicodeStringRef2Vector(
    std::vector< uima::UnicodeStringRef > & rveclstrOutput,
    const UChar                          * pcInput,
    int32_t                                 uiInputLength,
    const UChar                          * cpszDelimiters,
    bool                                   bTrimString,
    bool                                   bInsertEmptyStrings
  );

  inline int32_t
  delimitedUnicodeStringRef2Vector(
    std::vector< UnicodeStringRef > & veclstrOutput,
    const UChar                     * pcInput,
    const UChar                     * cpszDelimiters,
    bool                              bTrimString,
    bool                              bInsertEmptyStrings
  ) {
    return delimitedUnicodeStringRef2Vector(veclstrOutput, pcInput, u_strlen(pcInput), cpszDelimiters, bTrimString, bInsertEmptyStrings);
  }

//@}

} // namespace uima

#endif /* UIMA_UNICODESTRINGREF_HPP */

/* <EOF> */

