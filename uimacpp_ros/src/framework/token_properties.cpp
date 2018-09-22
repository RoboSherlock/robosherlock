/*
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


   8/11/00  Initial creation

-------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/token_properties.hpp>
#include <uima/unistrref.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */
using namespace std;
namespace uima {

  TokenProperties::TokenProperties(
    const icu::UnicodeString & ustrInputString
  ) {
    initFromString(ustrInputString.getBuffer(), ustrInputString.getBuffer()+ustrInputString.length());
  }

  TokenProperties::TokenProperties(
    const UnicodeStringRef & ulstrInputString
  ) {
    if (ulstrInputString.length() == 0) {
      return;
    }
    initFromString(ulstrInputString.getBuffer(), ulstrInputString.getBuffer()+ulstrInputString.length());
  }

  TokenProperties::TokenProperties(
    const UChar * cpucCurrent,
    const UChar * cpucEnd
  ) {
    initFromString(cpucCurrent, cpucEnd);
  }

// some quick check macros solving the task for characters < 128 inline
#define CHECK_U_UPPER( c ) \
   ( ( c >= (UChar)65 && c <=  (UChar)90) || u_isupper( c ) )
#define CHECK_U_LOWER( c ) \
   ( ( c >= (UChar)97 && c <= (UChar)122) || u_islower( c ) )
#define CHECK_U_DIGIT( c ) \
   ( ( c >= (UChar)48 && c <=  (UChar)57) || u_isdigit( c ) )

  void
  TokenProperties::initFromString(
    const UChar * cpucCurrent,
    const UChar * cpucEnd
  ) {
    iv_w32Bits = 0;

    assert(EXISTS(cpucCurrent));
    assert(EXISTS(cpucEnd));


    if (cpucCurrent == cpucEnd) {
      return;
    }
    if (CHECK_U_LOWER(*cpucCurrent)) {
      setLower();
    } else if (CHECK_U_UPPER(*cpucCurrent)) {
      setLeadingUpper();
    } else if (CHECK_U_DIGIT(*cpucCurrent)) {
      setNumeric();
    } else {
      setSpecial();
    }
    ++cpucCurrent;
    while (cpucCurrent < cpucEnd) {
      if (CHECK_U_LOWER(*cpucCurrent)) {
        setLower();
      } else if (CHECK_U_UPPER(*cpucCurrent)) {
        setTrailingUpper();
      } else if (CHECK_U_DIGIT(*cpucCurrent)) {
        setNumeric();
      } else {
        setSpecial();
      }
      ++cpucCurrent;
    }
  }

  string
  TokenProperties::to_string( void ) const {
    string s;
    for (size_t i = UIMA_TOKEN_PROP_SPECIAL; i > 0; i /= 2) {
      s += ((iv_w32Bits & i) != 0 ? '1' : '0');
    }
    return s;
  }

} //namespace uima

/* ----------------------------------------------------------------------- */
/* <EOF> */

