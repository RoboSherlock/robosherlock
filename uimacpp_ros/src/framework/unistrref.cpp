/**
-----------------------------------------------------------------------------

           string interface of icu::UnicodeString

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


       6/26/1998     Initial creation

-----------------------------------------------------------------------------
*/

#include <uima/unistrref.hpp>
#include <algorithm> // for min
#ifdef _MSC_VER
#include <minmax.h> // for min
#endif
/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */
using namespace std;
namespace uima {


//========================================
// Read-only implementation
//========================================

  int8_t
  UnicodeStringRef::doCompare( int32_t start,
                               int32_t length,
                               const UChar *srcChars,
                               int32_t srcStart,
                               int32_t srcLength) const {
    // compare illegal string values
    if (srcChars==0) {
      return 1;
    }

    // pin indices to legal values
    pinIndices(start, length);

    // get the correct pointer
    const UChar *chars = getBuffer();

    chars += start;
    srcChars += srcStart;

    int32_t minLength;
    int8_t lengthResult;

    // are we comparing different lengths?
    if (length != srcLength) {
      if (length < srcLength) {
        minLength = length;
        lengthResult = -1;
      } else {
        minLength = srcLength;
        lengthResult = 1;
      }
    } else {
      minLength = length;
      lengthResult = 0;
    }

    /*
     * note that uprv_memcmp() returns an int but we return an int8_t;
     * we need to take care not to truncate the result -
     * one way to do this is to right-shift the value to
     * move the sign bit into the lower 8 bits and making sure that this
     * does not become 0 itself
     */

    if (minLength > 0 && chars != srcChars) {
      int32_t result;

#   ifdef WORDS_BIGENDIAN
      // big-endian: byte comparison works
      result = memcmp(chars, srcChars, minLength * sizeof(UChar));
      if (result != 0) {
        return (int8_t)(result >> 15 | 1);
      }
#   else
      // little-endian: compare UChar units
      do {
        result = ((int32_t)*(chars++) - (int32_t)*(srcChars++));
        if (result != 0) {
          return (int8_t)(result >> 15 | 1);
        }
      } while (--minLength > 0);
#   endif
    }
    return lengthResult;
  }


  /* String compare in code point order - doCompare() compares in code unit order. */
  int8_t
  UnicodeStringRef::doCompareCodePointOrder(int32_t start,
      int32_t length,
      const UChar *srcChars,
      int32_t srcStart,
      int32_t srcLength) const {
    if (srcChars==NULL) {
      return 1;
    }

    // pin indices to legal values
    pinIndices(start, length);

    int32_t diff = u_strncmpCodePointOrder(getBuffer() + start, srcChars + srcStart, min(length, srcLength));
    /* translate the 32-bit result into an 8-bit one */
    if (diff!=0) {
      return (int8_t)(diff >> 15 | 1);
    } else {
      return 0;
    }
  }

  int32_t UnicodeStringRef::indexOf(UChar const *srcChars,
                                    int32_t srcStart,
                                    int32_t srcLength,
                                    int32_t start,
                                    int32_t length) const {
    if (srcChars == 0 || srcLength == 0) {
      return -1;
    }

    // get the indices within bounds
    pinIndices(start, length);

    if (length < srcLength) {
      return -1;
    }

    // now we will only work with srcLength-1
    --srcLength;

    // set length for the last possible match start position
    // note the --srcLength above
    length -= srcLength;


    const UChar *array = getBuffer();
    int32_t limit = start + length;

    // search for the first char, then compare the rest of the string
    // increment srcStart here for that, matching the --srcLength above
    UChar ch = srcChars[srcStart++];

    do {
      if (array[start] == ch && (srcLength == 0 || compare(start + 1, srcLength, srcChars, srcStart, srcLength) == 0)) {
        return start;
      }
    } while (++start < limit);

    return -1;
  }

  int32_t UnicodeStringRef::lastIndexOf(UChar const *srcChars,
                                        int32_t srcStart,
                                        int32_t srcLength,
                                        int32_t start,
                                        int32_t length) const {
    if (srcChars == 0 || srcLength == 0) {
      return -1;
    }

    // get the indices within bounds
    pinIndices(start, length);

    if (length < srcLength) {
      return -1;
    }

    // now we will only work with srcLength-1
    --srcLength;

    // set length for the last possible match start position
    // note the --srcLength above
    length -= srcLength;

    const UChar *array = getBuffer();
    int32_t pos;

    // search for the first char, then compare the rest of the string
    // increment srcStart here for that, matching the --srcLength above
    UChar ch = srcChars[srcStart++];

    pos = start + length;
    do {
      if (array[--pos] == ch && (srcLength == 0 || compare(pos + 1, srcLength, srcChars, srcStart, srcLength) == 0)) {
        return pos;
      }
    } while (pos > start);

    return -1;
  }


  int32_t
  UnicodeStringRef::doIndexOf(UChar c,
                              int32_t start,
                              int32_t length) const {
    // pin indices
    pinIndices(start, length);
    if (length == 0) {
      return -1;
    }

    // find the first occurrence of c
    const UChar *begin = getBuffer() + start;
    const UChar *limit = begin + length;

    do {
      if (*begin == c) {
        return (int32_t)(begin - getBuffer());
      }
    } while (++begin < limit);

    return -1;
  }

  int32_t
  UnicodeStringRef::doLastIndexOf(UChar c,
                                  int32_t start,
                                  int32_t length) const {
    // pin indices
    pinIndices(start, length);
    if (length == 0) {
      return -1;
    }

    const UChar *begin = getBuffer() + start;
    const UChar *limit = begin + length;

    do {
      if (*--limit == c) {
        return (int32_t)(limit - getBuffer());
      }
    } while (limit > begin);

    return -1;
  }

  int32_t UnicodeStringRef::moveIndex32(int32_t index, int32_t delta) const {
    icu::UnicodeString s((UBool)false, getBuffer(), length());
    return s.moveIndex32(index, delta);
  }

  int32_t
  UnicodeStringRef::extract(UChar *dest, int32_t destCapacity,
                            UErrorCode &errorCode) const {
    // This readonly aliasing constructor should be cheap as no copy is done
    icu::UnicodeString s((UBool)false, getBuffer(), length());
    return s.extract(dest, destCapacity, errorCode);
  }

  int32_t UnicodeStringRef::extract(int32_t start,
                                    int32_t startLength,
                                    char *target,
                                    uint32_t targetLength,
                                    const char *codepage) const {
    icu::UnicodeString s((UBool)false, getBuffer(), length());
    return s.extract(start, startLength, target, targetLength, codepage);
  }

  int32_t UnicodeStringRef::extract(char *target, int32_t targetCapacity,
                                    UConverter *cnv,
                                    UErrorCode &errorCode) const {
    icu::UnicodeString s((UBool)false, getBuffer(), length());
    return s.extract(target, targetCapacity, cnv, errorCode);
  }

// Copy with conversion into a std::string
  int32_t UnicodeStringRef::extract(int32_t start,
                                    int32_t startLength,
                                    std::string & target,
                                    const char *codepage) const {
    if (length() == 0) {
      target.clear();
      return 0;
    }

    // First use a buffer on the stack ... if too small allocate and try again
    const int32_t STACK_BUF_SIZE = 256;
    char  stackBuf [STACK_BUF_SIZE];
    char* heapBuf = NULL;
    char* buf = stackBuf;

    // Use a converter so can be left open if have to convert twice
    // If fail to open converter simply return empty string ... must be unknown!
    UErrorCode err = U_ZERO_ERROR;
    UConverter* cnv = ucnv_open(codepage, &err);
    if ( U_FAILURE(err) ) {
      target.clear();
      return 0;
    }

    const UChar* src = getBuffer() + start;
    int len = ucnv_fromUChars(cnv, buf, STACK_BUF_SIZE, src, startLength, &err);
    if ( err == U_BUFFER_OVERFLOW_ERROR || err == U_STRING_NOT_TERMINATED_WARNING ) {
      buf = heapBuf = new char [len+1];
      err = U_ZERO_ERROR;
      len = ucnv_fromUChars(cnv, buf, len+1, src, startLength, &err);
    }

    target.assign(buf, len);                   // Copy the result to the string

    if (heapBuf != NULL)
      delete [] heapBuf;
    ucnv_close(cnv);

    return len;
  }

// Extract into a UTF-8 std::string
  int32_t UnicodeStringRef::extractUTF8(std::string & target) const {
    if (length() == 0) {
      target.clear();
      return 0;
    }

    // First use a buffer on the stack ... if too small allocate and try again
    const int32_t STACK_BUF_SIZE = 256;
    char  stackBuf [STACK_BUF_SIZE];
    char* heapBuf = NULL;
    char* buf = stackBuf;
    int32_t len;

    UErrorCode err = U_ZERO_ERROR;
    u_strToUTF8(buf, STACK_BUF_SIZE, &len, getBuffer(), length(), &err);
    if ( err == U_BUFFER_OVERFLOW_ERROR || err == U_STRING_NOT_TERMINATED_WARNING ) {
      buf = heapBuf = new char [len+1];
      err = U_ZERO_ERROR;
      u_strToUTF8(buf, len+1, &len, getBuffer(), length(), &err);
    }

    target.assign(buf, len);                   // Copy the result to the string

    if (heapBuf != NULL)
      delete [] heapBuf;

    return len;
  }

// Static method releases contents of string container allocated by extract methods
  void UnicodeStringRef::release(std::string & target) {
    target.clear();               // Empty string
    target.reserve(1);            // Reduce capacity so will use internal buffer & free external one
  }


  void
  UnicodeStringRef::toSingleByteStream(std::ostream & outStream) const {
    const char* codepage;

    // If output goes to console use default encoding
    if (outStream.rdbuf() == cout.rdbuf() || outStream.rdbuf() == cerr.rdbuf()) {
      codepage = 0;
    } else {
      codepage = "utf-8";
    }
    std::string s;
    extract(s, codepage);                       // get a single byte string
    outStream << s;
  }

  std::ostream &
  operator << (
    std::ostream                & outStream,
    const uima::UnicodeStringRef & crUStrRef
  ) {
    crUStrRef.toSingleByteStream(outStream);
    return outStream;
  }

  int32_t
  delimitedUnicodeStringRef2Vector(
    std::vector< uima::UnicodeStringRef > & rveclstrOutput,
    const UChar                          * pcInput,
    int32_t                                 uiInputLength,
    const UChar                          * cpszDelimiters,
    bool                                   bTrimString,
    bool                                   bInsertEmptyStrings
  ) {
    UChar const * pcBegin = pcInput;
    int32_t uiEnd;
    UChar const * pcEnd = pcBegin;
    int32_t uiNumFound = 0;
    int32_t uiDelimitersLen = u_strlen(cpszDelimiters);

    if (uiInputLength == 0) {
      return 0;
    }
    UChar const * pcInputEnd = pcInput + uiInputLength;
    UnicodeStringRef _s;

    while (pcBegin < pcInputEnd) {
      //      uiBegin--;
      uiEnd   = str_find_first_of(cpszDelimiters, uiDelimitersLen, pcBegin, (int32_t)(pcInputEnd-pcBegin));
      pcEnd = pcBegin+uiEnd;
      if (uiEnd != STRING_NPOS) {
        ++pcEnd;
      }
      if (uiEnd == STRING_NPOS) {
        uiEnd = uiInputLength+1;
        pcEnd = pcInputEnd+1;
      }
      assert(pcEnd > pcBegin);
      _s = UnicodeStringRef(pcBegin, pcEnd-pcBegin-1);
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

} // namespace uima

std::ostream &
operator << (
  std::ostream                & outStream,
  const uima::UnicodeStringRef & crUStrRef
) {
  crUStrRef.toSingleByteStream(outStream);
  return outStream;
}



/* <EOF> */

