/** \file token_properties.hpp .
-----------------------------------------------------------------------------



           (upper, lower, etc.)

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

    \brief  Contains TokenProperties a class encapsulting information about the characters occuring in a token

   Description:

-----------------------------------------------------------------------------


   8/11/00  Initial creation

-------------------------------------------------------------------------- */

#ifndef UIMA_TOKEN_PROPERTIES_HPP
#define UIMA_TOKEN_PROPERTIES_HPP

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
// extended assert: must be before system includes to make sure our version is used
#include <uima/assertmsg.h>
#include "unicode/unistr.h"
#include <uima/types.h>
#include <string>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
#ifndef UIMA_STRPTRLENPAIR_HPP
  class UnicodeStringRef;
#endif
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {

  /**
     The class <TT>TokenProperties</TT> is used to encapsulate information about
     the characters occuring in a token (for example, upper and lower).
     At the centre it is a bitset, but with inline member functions
     for convenient access.
     This has to be filled by each compliant tokenizer and stored
     with each token.
     Example:
     \code
     \endcode
     @see
  */
  class UIMA_LINK_IMPORTSPEC TokenProperties {
  public:
    /** @name Constructors */
    /*@{*/
    /// Constructs an object, initializing all bit values to zero.
    TokenProperties( void );
    /// Constructs an object from a UString, computing the bit values for the string.
    TokenProperties( const icu::UnicodeString & ustrInputString);
    /// Constructs an object from a UString, computing the bit values for the string.
    TokenProperties( const UnicodeStringRef & ulstrInputString);
    /** Constructs an object from a two pointers, computing the bit values for the string.
     * Note: cpucEnd points beyond the end of the string
     */
    TokenProperties(
      const UChar * cpucCurrent,
      const UChar * cpucEnd);
    /**
     * initializes bits to value of <TT>w32Val</TT>
     */
    TokenProperties( WORD32 w32Val );
    /*@}*/
    /** @name Properties */
    /*@{*/
    /// true if the first char in the token is upper case
    bool hasLeadingUpper( void ) const;
    /// sets the <TT>hasLeadingUpper()</TT> property to <TT>bSetOn</TT>
    void setLeadingUpper( bool bSetOn = true );

    /// true if some char after the first char in the token is upper case
    bool hasTrailingUpper( void ) const;
    /// sets the <TT>hasTrailingUpper()</TT> property to <TT>bSetOn</TT>
    void setTrailingUpper( bool bSetOn = true );

    /// true if the token has upper case chars (leading or trailing)
    bool hasUpper( void ) const;

    /// true if the token has lower case chars
    bool hasLower( void ) const;
    /// sets the <TT>hasLower()</TT> property to <TT>bSetOn</TT>
    void setLower( bool bSetOn = true );

    /// true if the token has numeric chars
    bool hasNumeric( void ) const;
    /// sets the <TT>hasNumeric()</TT> property to <TT>bSetOn</TT>
    void setNumeric( bool bSetOn = true);

    /// true if the token has special chars (e.g. hyphen, period etc.)
    bool hasSpecial( void ) const;
    /// sets the <TT>hasSpecial()</TT> property to <TT>bSetOn</TT>
    void setSpecial( bool bSetOn = true );
    /*@}*/
    /** @name Miscellaneous */
    /*@{*/
    /// true if not hasSpecial() and not hasNumeric()
    bool isPlainWord() const;

    /// true if only hasUpper()
    bool isAllUppercaseWord( void ) const;
    /// true if only hasLower()
    bool isAllLowercaseWord( void ) const;
    /// true if only hasLeadingUpper() and hasTrailingUpper()
    bool isInitialUppercaseWord( void ) const;
    /** true if hasNumeric() && !(hasLower() || hasUpper())
     *  Note: this might have decimal point and sign
     */
    bool isPlainNumber() const;

    /** unlike isPlainNumber() this only allows for digits (no sign and point)
     */
    bool isPureNumber() const;

    /** true if hasSpecail() && !(hasLower() || hasUpper() || hasNumeric())
     *  Note: this might have decimal point and sign
     */
    bool isPureSpecial() const;

    /// Resets all bits in *this, and returns *this
    void reset( void );

    /// Resets all bits and reinitializes from the string
    void
    initFromString(
      const UChar * cpucCurrent,
      const UChar * cpucEnd
    );

    /**
       Returns an object of type string, N characters long.
       Each position in the new string is initialized with a character
       ('0' for zero and '1' for one), representing the value stored in the
       corresponding bit position of this.
       Character position N - 1 corresponds to bit position 0.
       Subsequent decreasing character positions correspond to increasing
       bit positions. */
    std::string to_string( void ) const;

    /// Returns the integral value corresponding to the bits in *this.
    unsigned long to_ulong( void ) const;

    /*@}*/
  protected:
    /* --- functions -------------------------------------------------------- */
    /* --- variables -------------------------------------------------------- */
  private:
    /* --- functions -------------------------------------------------------- */
    /* --- variables -------------------------------------------------------- */
    WORD32  iv_w32Bits;

  }
  ; /* TokenProperties */

#define UIMA_TOKEN_PROP_LEADING_UPPER   1
#define UIMA_TOKEN_PROP_TRAILING_UPPER  2
#define UIMA_TOKEN_PROP_LOWER           4
#define UIMA_TOKEN_PROP_NUMERIC         8
#define UIMA_TOKEN_PROP_SPECIAL        16

  /* ----------------------------------------------------------------------- */
  /*       Inline Functions                                                  */
  /* ----------------------------------------------------------------------- */


  inline TokenProperties::TokenProperties( void ) :
      iv_w32Bits(0) {}
  inline TokenProperties::TokenProperties( WORD32 w32Val ) :
      iv_w32Bits(w32Val) {}
  inline bool TokenProperties::hasTrailingUpper( void )  const {
    return((iv_w32Bits & UIMA_TOKEN_PROP_TRAILING_UPPER) != 0);
  }
  inline void TokenProperties::setTrailingUpper( bool bSetOn ) {
    if (bSetOn) {
      iv_w32Bits |= UIMA_TOKEN_PROP_TRAILING_UPPER;
    } else {
      iv_w32Bits &= (~UIMA_TOKEN_PROP_TRAILING_UPPER);
    }
  }

  inline bool TokenProperties::hasLeadingUpper( void ) const {
    return((iv_w32Bits & UIMA_TOKEN_PROP_LEADING_UPPER) != 0);
  }
  inline void TokenProperties::setLeadingUpper( bool bSetOn ) {
    if (bSetOn) {
      iv_w32Bits |= UIMA_TOKEN_PROP_LEADING_UPPER;
    } else {
      iv_w32Bits &= (~UIMA_TOKEN_PROP_LEADING_UPPER);
    }
  }
  inline bool TokenProperties::hasUpper( void )  const {
    return((iv_w32Bits & (UIMA_TOKEN_PROP_LEADING_UPPER | UIMA_TOKEN_PROP_TRAILING_UPPER)) != 0);
  }

  inline bool TokenProperties::hasLower( void ) const {
    return((iv_w32Bits & UIMA_TOKEN_PROP_LOWER) != 0);
  }
  inline void TokenProperties::setLower( bool bSetOn) {
    if (bSetOn) {
      iv_w32Bits |= UIMA_TOKEN_PROP_LOWER;
    } else {
      iv_w32Bits &= (~UIMA_TOKEN_PROP_LOWER);
    }
  }

  inline bool TokenProperties::hasNumeric( void ) const {
    return((iv_w32Bits & UIMA_TOKEN_PROP_NUMERIC) != 0);
  }
  inline void TokenProperties::setNumeric( bool bSetOn ) {
    if (bSetOn) {
      iv_w32Bits |= UIMA_TOKEN_PROP_NUMERIC;
    } else {
      iv_w32Bits &= (~UIMA_TOKEN_PROP_NUMERIC);
    }
  }

  inline bool TokenProperties::hasSpecial( void ) const {
    return((iv_w32Bits & UIMA_TOKEN_PROP_SPECIAL) != 0);
  }
  inline void TokenProperties::setSpecial( bool bSetOn ) {
    if (bSetOn) {
      iv_w32Bits |= UIMA_TOKEN_PROP_SPECIAL;
    } else {
      iv_w32Bits &= (~UIMA_TOKEN_PROP_SPECIAL);
    }
  }

  inline bool TokenProperties::isPlainWord( void ) const {
    return(   ((iv_w32Bits &  (UIMA_TOKEN_PROP_TRAILING_UPPER | UIMA_TOKEN_PROP_LOWER | UIMA_TOKEN_PROP_LEADING_UPPER)) != 0)
              && ((iv_w32Bits & ~(UIMA_TOKEN_PROP_TRAILING_UPPER | UIMA_TOKEN_PROP_LOWER | UIMA_TOKEN_PROP_LEADING_UPPER)) == 0));
  }
  inline bool TokenProperties::isAllUppercaseWord( void ) const {
    return(   ((iv_w32Bits &  (UIMA_TOKEN_PROP_TRAILING_UPPER | UIMA_TOKEN_PROP_LEADING_UPPER)) != 0)
              && ((iv_w32Bits & ~(UIMA_TOKEN_PROP_TRAILING_UPPER | UIMA_TOKEN_PROP_LEADING_UPPER)) == 0));
  }
  inline bool TokenProperties::isAllLowercaseWord( void ) const {
    return(iv_w32Bits  == UIMA_TOKEN_PROP_LOWER);
  }
  inline bool TokenProperties::isInitialUppercaseWord( void ) const {
    return( iv_w32Bits == (UIMA_TOKEN_PROP_LOWER | UIMA_TOKEN_PROP_LEADING_UPPER));
  }
  inline bool TokenProperties::isPlainNumber() const {
    return(   ((iv_w32Bits &  (UIMA_TOKEN_PROP_NUMERIC)) != 0)
              && ((iv_w32Bits & ~(UIMA_TOKEN_PROP_SPECIAL | UIMA_TOKEN_PROP_NUMERIC)) == 0));
  }
  inline bool TokenProperties::isPureNumber() const {
    return(iv_w32Bits  == UIMA_TOKEN_PROP_NUMERIC);
  }
  inline bool TokenProperties::isPureSpecial() const {
    return(iv_w32Bits == UIMA_TOKEN_PROP_SPECIAL);
  }


  inline void TokenProperties::reset( void ) {
    iv_w32Bits = 0;
  }

  inline unsigned long TokenProperties::to_ulong( void ) const {
    return(unsigned long)iv_w32Bits;
  }

}

/* ----------------------------------------------------------------------- */
#endif /* UIMA_TOKEN_PROPERTIES_HPP */

/* <EOF> */




