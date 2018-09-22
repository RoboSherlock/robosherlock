/**
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


   5/18/1999  Initial creation

-------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/language.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */
using namespace std;
namespace uima {

  char const * Language::INVALID        = "??";
  char const * Language::UNSPECIFIED    = "";

}

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

namespace uima {

  // Locale is stored as ll-cc, e.g. "en-us"
  // or just the language ll, e.g. "en",
  // or as "" if unspecified (matches any language)
  // or as "??" if invalid.
  // Both fields are restricted to 2 characters in lowercase.
  // Note: Jedii does not restrict the lengths of the fields, and normalizes both to
  // lowercase, e.g. fr-ca or en-nzl, but does not check for legal locales.
  // So restricting each to 2 characters for UIMA should be OK, and makes it easy
  // to represent a language as a 4-byte number.

  void
  Language::init( const char * cpszLanguageCode ) {
    // Check that have 2 chars (a-z) or 2 such pairs separated by - or _

    static const char legalChars[] = "abcdefghijklmnopqrstuvwzyz";

    memset(iv_locale, '\0', 6);                           // Start with all 0's
    memset(iv_lang,   '\0', 3);

    unsigned int len = strlen(cpszLanguageCode);
    if (len == 0)
      return;                                             // Unspecified

    if (len == 2 || len == 5) {
      iv_locale[0] = tolower(cpszLanguageCode[0]);        // language "ll"
      iv_locale[1] = tolower(cpszLanguageCode[1]);
      if (len == 5) {
        if (cpszLanguageCode[2] == '-' || cpszLanguageCode[2] == '_')
          iv_locale[2] = 'a';                             // Temporarily "legal"
        else
          iv_locale[2] = '?';                             // Illegal separator
        iv_locale[3] = tolower(cpszLanguageCode[3]);      // territory "cc"
        iv_locale[4] = tolower(cpszLanguageCode[4]);
      }
    } else
      iv_locale[0] = '?';                                 // Illegal length 1,3,4,6,7,...

    if ( strspn(iv_locale, legalChars) < len )            // Illegal characters?
      strcpy(iv_locale, "??");                            // Invalid
    else {
      if ( len == 5 )
        iv_locale[2] = '-';                               // Normalize to -
    }
    strncpy(iv_lang, iv_locale, 2);                       // Also save 2-char lang part
    iv_lang[2] = '\0';
  }

  void
  Language::_initFromString( const string & crstrLanguageCode ) {
    init(crstrLanguageCode.c_str());
  }

  void
  Language::_initFromString( UnicodeStringRef crustrLanguageCode ) {
    size_t const MAX_LEN = 32;
    char buff[MAX_LEN];
    size_t uiLen = min(MAX_LEN-1, (size_t)crustrLanguageCode.length());
    // this conversion does invariant chars only but it is fast
    u_UCharsToChars(crustrLanguageCode.getBuffer(), buff, uiLen);
    // zero terminate buffer
    buff[uiLen] = char(0);
    init(buff);
  }

  Language::Language( TyLanguageAsNumber lLanguageAsNumber ) {
    unsigned long ul = (unsigned long)lLanguageAsNumber;

    iv_locale[0] = ul>>24;
    iv_locale[1] = (ul >> 16) & 0x000000ff;
    iv_locale[3] = (ul >> 8)  & 0x000000ff;
    iv_locale[4] = ul         & 0x000000ff;
    iv_locale[5] = '\0';
    if (iv_locale[3] == '\0')              // No territory
      iv_locale[2] = '\0';
    else
      iv_locale[2] = '-';
    strncpy(iv_lang, iv_locale, 2);
    iv_lang[2] = '\0';
  }

  bool Language::matches( const Language & crclCompareLang ) const {
    // Full locale matchs, or
    // one is unspecified, or
    // language part matches and one territory is unspecified (missing)
    return ( strcasecmp(iv_locale, crclCompareLang.iv_locale) == 0 ||
             iv_locale[0] == '\0' ||
             crclCompareLang.iv_locale[0] == '\0' ||
             ( strcasecmp(iv_lang, crclCompareLang.iv_lang) == 0 &&
               (iv_locale[3] == '\0' ||  crclCompareLang.iv_locale[3] == '\0') ) );
  }

} //namespace uima
/* <EOF> */

