#ifndef UIMA_LANGUAGE_HPP
#define UIMA_LANGUAGE_HPP
/** \file language.hpp .
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

    \brief  Contains the Language class

   Description:

-----------------------------------------------------------------------------


   6/10/1999  Initial creation

-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas

#include <uima/assertmsg.h>
#include <uima/unistrref.hpp>
#include "apr_general.h"                // For strcasecmp

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  /**
     The class <TT>Language</TT> models languages in UIMACPP.
     A language is specified as a string holding a 2-character language and
     an optional 2-character territory, i.e. as "ll-cc" or "ll".

     String representation of the simple language sub-part is according to
     ISO standard 639 "Codes for the representation of the names of languages".

     String representation of the territory sub-part is according to
     ISO standard 3166 "Codes for the Representation of Names of Countries".

     String representation of the full language object is according to
     IANA RFC 1766 "Tags for the Identification of Languages": <TT>&lt;LANG>-&lt;SUBTAG></TT>

     There is also an internal technical numeric representation of a
     language as a 4 byte number (32 bit, high-word language value and low-word
     territory value).  Conversion to or from numeric representation is provided
     via a constructor or conversion operator.

     The class distinguishes between unspecified and invalid
     languages and territories.  For example, in "en" the territory and sub-language is
     valid, but unspecified, as opposed to "en-US" where the
     territory and sub-language is specified as "US". However, in "en-FOO" the
     territory and sub-language is invalid as there is no such territory code.

     Because of this, there is more than one way for two language objects to be
     compatible with each other:

     <OL>
     <LI> Full identity: language and sub-language/territory are identical
        (for example, en-US == en-us).

     <LI> Match: the language part is identical and the sub-language and territory of
        at least one of the objects is set to unspecified
        (for example, en matches en-US).

     <LI> Match ignoring territory: the language part is identical, both
        sub-languages are specified but they are different from each other
        (for example, en-US matches-without-territory en-AU).
     </OL>

     Match type 2 is used if a annotator specifies that it can deal with
     any kind of english text and is not limited, or specialized to
     US-English.

     Match type 3 is not supported.

  \code
     Language clLanguage(argv[1]);
     if(!clLanguage.isValid()) {
        // abort with error
           //...
     }
     if (! (   clLanguage.matches("en")
            || clLanguage.matches("de") ) ) {
        // abort with error
           //...
     }
  \endcode

     Note: As the class is simple, compiler generated copy constructor
     and the assignment operator can be used.
  */
  class UIMA_LINK_IMPORTSPEC Language {
  public:
    /** @name Language constants and types */
    /**
        Special constants for the invalid & unspecified languages
    */
    static char const * INVALID;
    static char const * UNSPECIFIED;

    /** A typedef for representing a languages as a numeric value */
    typedef long TyLanguageAsNumber;
    /*@{*/

    /** @name Constructors */
    /*@{*/
    /** Default Constructor: Language::UNSPECIFIED
    */
    Language (void);

    /** Constructor from a C string.
        String must have form <TT>language_territory</TT>. For example, "en-US"
        or just language "en".
    */
    Language (
      const TCHAR * cpszLanguageCode
    );

    /** Constructor from a single-byte string (std::string).
        String must have form <TT>language_territory</TT>. For example, "en-US"
        or just language "en".
    */
    Language (
      const std::string & languageCode
    );

    /** Constructor from a ICU Unicode string.
        String must have form <TT>language_territory</TT>. For example, "en-US"
        or just language "en".
    */
    Language (
      const icu::UnicodeString & languageCode
    );

    /** Constructor from a UnicodeStringRef.
        String must have form <TT>language_territory</TT>. For example, "en-US"
        or just language "en".
    */
    Language (
      const UnicodeStringRef & languageCode
    );

    /**
       Constructor from a 32 bit representation of a language (see asNumber)
    */
    Language (
      TyLanguageAsNumber ulLanguageAsLong
    );
    /*@}*/

    /** @name Match functions */
    /*@{*/

    /** Returns TRUE, if this language is identical to the specified language. */
    bool
    operator== (
      const Language & crclObject
    ) const;

    /** Returns TRUE, if this language is identical to the specified language. */
    bool
    operator!= (
      const Language & crclObject
    ) const;

    /** Returns TRUE, if this language code sorts before the specified language. */
    bool
    operator< (
      const Language & crclOther
    ) const;

    /** Returns TRUE, if the languages are identical and
        either the territories are equal or one is unspecified,
        or if one of the languages is unspecified.
    */
    bool
    matches(
      const Language & crclCompareLang
    ) const;
    /*@}*/

    /** @name Miscellaneous */
    /*@{*/
    /**
       Returns TRUE if language is valid (territory may be missing)
     */
    bool
    isValid(void) const;

    /**
       Get just the 2-character language part, or an empty string if unspecified.
     */
    const char *
    getLanguageCode(void) const;

    /**
       Get a numeric form of just the language (2-characters in top 2-bytes)
     */
    TyLanguageAsNumber
    getLanguage(void) const;

    /**
       Returns TRUE if language has been specified
     */
    bool
    hasLanguage(void) const;

    /**
       Get just the 2-character territory part, or an empty string if unspecified.
     */
    const char *
    getTerritoryCode(void) const;

    /**
       Get a numeric form of just the territory (2-characters in bottom 2-bytes)
     */
    TyLanguageAsNumber
    getTerritory(void) const;

    /**
       Returns TRUE if territory has been specified
     */
    bool
    hasTerritory(void) const;

    /**
       Sets the value according to string <TT>crclString</TT>.
       <TT>crclString</TT> must have the form <TT>&lt;LANG_ID>[-&lt;TERR_ID>]</TT>.
     */
    void
    setValue(
      const std::string & crclString
    );

    /** Returns the object in the form &lt;LANGUAGE_CODE>-&lt;TERRITORY_CODE>.
       For example, en-US.
    */
    std::string
    asString( void ) const;
    /** Returns the object in the form &lt;LANGUAGE_CODE>-&lt;TERRITORY_CODE>.
       For example, en-US.
    */
    icu::UnicodeString
    asUnicodeString( void ) const;
    /** Returns the object as a 4-byte "number"
       (actually just the 4 character bytes, e.g. x656e7472 'enus')
    */
    TyLanguageAsNumber
    asNumber(void) const;

    /*@}*/
  protected:
    /* --- functions -------------------------------------------------------- */
    /* --- variables -------------------------------------------------------- */
  private:
    /* --- functions -------------------------------------------------------- */

    /// Used in ctors to init from a string.
    void
    _initFromString(
      const std::string & crstrLanguageCode
    );
    /// Used in ctors to init from a Unicode string.
    void
    _initFromString(
      UnicodeStringRef crustrLanguageCode
    );
    /// Used in ctors to init from a C string.
    void
    init(
      const char*  cpszLanguageCode
    );

    /* --- variables -------------------------------------------------------- */
    char                     iv_locale[6];
    char                     iv_lang[3];

  }
  ; /* Language */


  inline Language::Language() {
    init(Language::UNSPECIFIED);
  }

  inline Language::Language(
    std::string const & crstrLanguageCode
  ) {
    _initFromString(crstrLanguageCode);
  }

  inline Language::Language(
    icu::UnicodeString const & crustrLanguageCode
  ) {
    _initFromString(UnicodeStringRef(crustrLanguageCode));
  }

  inline Language::Language(
    UnicodeStringRef const & crustrLanguageCode
  ) {
    _initFromString(crustrLanguageCode);
  }

  inline Language::Language(
    TCHAR const * cpszLanguageCode
  ) {
    init(cpszLanguageCode);
  }

  inline bool
  Language::operator== (
    const Language & crclObject
  ) const {
    // Both language & territory must match
    return (strcasecmp(iv_locale, crclObject.iv_locale) == 0);
  }

  inline bool
  Language::operator!= (
    const Language & crclObject
  ) const {
    return (strcasecmp(iv_locale, crclObject.iv_locale) != 0);
  }

  inline bool
  Language::operator< (
    const Language & crclOther
  ) const {
    // Used to order by language & territory enums ... full string should be OK
    return (strcasecmp(iv_locale, crclOther.iv_locale) < 0);
  }

  inline void
  Language::setValue(
    const std::string & crclString
  ) {
    _initFromString(crclString);
  }

  inline std::string
  Language::asString( void ) const {
    return (iv_locale);
  }

  inline icu::UnicodeString
  Language::asUnicodeString( void ) const {
    // this ctor does invariant chars only but it is fast
    return icu::UnicodeString(iv_locale, "");
  }

  inline Language::TyLanguageAsNumber
  Language::asNumber(void) const {
    unsigned long num;
    const unsigned char * uc = (const unsigned char *) iv_locale;
    num = uc[0] << 24 | uc[1] << 16 | uc[3] << 8  | uc[4];
    return (TyLanguageAsNumber)num;
  }

  inline const char *
  Language::getLanguageCode(void) const {
    return iv_lang;
  }

  inline Language::TyLanguageAsNumber
  Language::getLanguage(void) const {
    // Replace old enum by 4-byte "number" with 2-char language in top 2 bytes
    int num = iv_lang[0] << 24 | iv_lang[1] << 16;
    return num;
  }

  inline bool
  Language::hasLanguage(void) const {
    return (iv_locale[0] != '\0');
  }

  inline const char *
  Language::getTerritoryCode(void) const {
    return &iv_locale[3];
  }

  inline Language::TyLanguageAsNumber
  Language::getTerritory(void) const {
    // Replace old enum by 4-byte "number" with 2-char territory in bottom 2 bytes
    int num = iv_locale[3] << 8 | iv_locale[4];
    return num;
  }

  inline bool
  Language::isValid(void) const {
    return (iv_locale[0] != '?');
  }

  inline bool
  Language::hasTerritory(void) const {
    return (iv_locale[2] != '\0');
  }

  /* ----------------------------------------------------------------------- */

}  // namespace uima

#endif /* UIMA_LANGUAGE_HPP */
/* <EOF> */

