#ifndef UIMA_STRCONVERT_HPP
#define UIMA_STRCONVERT_HPP
/** \file strconvert.hpp .
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

    \brief Contains various string conversion routines (e.g. string2Long)

-------------------------------------------------------------------------------
*/

/* ----------------------------------------------------------------------- */
/*      Include dependencies                                               */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <uima/assertmsg.h>
#include <uima/unistrref.hpp> // for UnicodeStringRef class
#include <uima/strtools.hpp>     //string tools
#include "apr_general.h"         // For strcasecmp

/* ----------------------------------------------------------------------- */
/*      Implementation dependencies                                        */
/* ----------------------------------------------------------------------- */

namespace uima {

  /* ----------------------------------------------------------------------- */
  /*      Constants                                                          */
  /* ----------------------------------------------------------------------- */

  /* ----------------------------------------------------------------------- */
  /*      Implementation                                                     */
  /* ----------------------------------------------------------------------- */

  /*---------------------------------------------------------------------------*/


  /**
     @name Conventional conversion functions: POD to/from string.
  */
  /*@{*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  long2String(
    long                    l,
    BASIC_STRING_TEMPLATE & s
  );
#endif

  /** Converts a long to a string.
      Cannot be implemented as single argument  function because of template definitions.
      @param l input parameter
      @param s output parameter
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  long2String(
    long                    l,
    BASIC_STRING_TEMPLATE & s
  ) {
    const int MAXDIGITS = 33;
    CharT  buff[MAXDIGITS];
    assertWithMsg(sizeof(CharT) == sizeof(char), "Double check the use of sprintf() with wchar."); //lint !e506: Constant value Boolean
    sprintf(buff, "%li", l);
    s = BASIC_STRING_TEMPLATE(buff);
    return s;
  }




  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)  //lint !e506: Constant value Boolean
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  long
  string2Long(
    const BASIC_STRING_TEMPLATE & s
  );
#endif

///Converts a string to a long.
  template BS_TEMPLATE_DEFINITION_ARGS
  long
  string2Long(
    const BASIC_STRING_TEMPLATE & s
  ) {
    assertWithMsg(sizeof(CharT) == sizeof(char), "Double check the use of atol() with wchar.");  //lint !e506: Constant value Boolean
    return atol(s.c_str());
  }



  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  double2String(
    double                  d,
    BASIC_STRING_TEMPLATE & s
  );
#endif

  /** Converts a double to a string.
      Cannot be implemented as single argument  function because of template definitions.
      @param d input parameter
      @param s output parameter
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  double2String(
    double                  d,
    BASIC_STRING_TEMPLATE & s
  ) {
    const int MAXDIGITS = 40;
    CharT  buff[MAXDIGITS];
    assertWithMsg(sizeof(CharT) == sizeof(char), "Double check the use of sprintf() with wchar.");  //lint !e506: Constant value Boolean
    ::sprintf(buff, "%g", d);
    s = BASIC_STRING_TEMPLATE(buff);
    return s;
  }



  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  double2String(
    double                  d,
    size_t                  uiDigitsAfterPeriod,
    BASIC_STRING_TEMPLATE & s
  );
#endif

  /** Converts a double to a string with a specified number of digits after the period.
      Cannot be implemented as single argument function because of template definitions.
      @param d input parameter.
      @param uiDigitsAfterPeriod nbr of digits after period.
      @param s output parameter.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  double2String(
    double                  d,
    size_t                  uiDigitsAfterPeriod,
    BASIC_STRING_TEMPLATE & s
  ) {
//    if (d <= DBL_MIN) {
//       return BASIC_STRING_TEMPLATE("0");
//    }
    const int MAXDIGITS = 50;
    CharT buff[MAXDIGITS];
    assertWithMsg(sizeof(CharT) == sizeof(char), "Double check the use of sprintf() with wchar.");//lint !e506: Constant value Boolean
    sprintf(buff, "%0#.*f", uiDigitsAfterPeriod, d); //lint !e567 !e557 unrecognized format
    s = BASIC_STRING_TEMPLATE(buff);
    return s;
  }



  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  double
  string2Double(
    const BASIC_STRING_TEMPLATE & s
  );
#endif

///Converts a string to a double.
  template BS_TEMPLATE_DEFINITION_ARGS
  double
  string2Double(
    const BASIC_STRING_TEMPLATE & s
  ) {
    assertWithMsg(sizeof(CharT) == sizeof(char), "Double check the use of strtod() with wchar.");//lint !e506: Constant value Boolean
    return strtod(s.c_str(), NULL);
  }



  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  bool2String(
    bool                    b,
    BASIC_STRING_TEMPLATE & s
  );
#endif

  /** Converts a bool to a string, either "true" or "false".
      Cannot be implemented as single argument function because of template definitions.
      @param b input parameter.
      @param s output parameter.
  */
  template BS_TEMPLATE_DEFINITION_ARGS
  BASIC_STRING_TEMPLATE
  bool2String(
    bool                    b,
    BASIC_STRING_TEMPLATE & s  //output
  ) {
    s = (b ? BASIC_STRING_TEMPLATE("true") : BASIC_STRING_TEMPLATE("false"));
    return s;
  }



  /*---------------------------------------------------------------------------*/

#if defined(UNDECLARED_FUNCTION_TEMPLATES_LINK_BUG)
// To work around "unsatisfied symbols" during linking,
// we need a declaration in addition to the definition below
  template BS_TEMPLATE_DEFINITION_ARGS
  bool
  string2Bool(
    const BASIC_STRING_TEMPLATE & s
  );
#endif

///Converts a string ("true", "ON", "YES", "T", "Y", "+", "1" = true else false) to a bool.
  template BS_TEMPLATE_DEFINITION_ARGS
  bool
  string2Bool(
    const BASIC_STRING_TEMPLATE & s
  ) {
    return (    (::strcmp(s.c_str(), "1") == 0)
                || (::strcmp(s.c_str(), "+") == 0)
                || (::strcasecmp(s.c_str(), "true") == 0)
                || (::strcasecmp(s.c_str(), "on") == 0)
                || (::strcasecmp(s.c_str(), "yes") == 0)
                || (::strcasecmp(s.c_str(), "t") == 0)
                || (::strcasecmp(s.c_str(), "y") == 0)
           );
  }
  /*@}*/

  /**
     @name Template function for conversion <EM>to</EM> string.
  */
  /*@{*/

/// Generic template function (assumes T to be integral numeric type).
  template < class T >
  void
  convertToString(
    const T     & crtSource,
    std::string & rstrTarget
  ) {
    long2String((long)crtSource, rstrTarget);
  }

/// Explicit specialization for UString
//template < class UString >
  inline void
  convertToString(
    const icu::UnicodeString & crtSource,
    std::string              & rstrTarget
  ) {
    UnicodeStringRef(crtSource).extractUTF8(rstrTarget);
  }

/// Explicit specialization for string
//template < class string >
  inline void
  convertToString(
    const std::string & crtSource,
    std::string       & rstrTarget
  ) {
    rstrTarget = crtSource;
  }

/// Explicit specialization for double
//template < class double >
  inline void
  convertToString(
    const double & crtSource,
    std::string  & rstrTarget
  ) {
    double2String(crtSource, rstrTarget);
  }

/// Explicit specialization for bool
//template < class double >
  inline void
  convertToString(
    bool const &  crtSource,
    std::string & rstrTarget
  ) {
    bool2String((bool)crtSource, rstrTarget);
  }

/// Explicit specialization for float
//template < class float >
  inline void
  convertToString(
    const float  & crtSource,
    std::string  & rstrTarget
  ) {
    double2String((double)crtSource, rstrTarget);
  }

  /*@}*/

  /**
     @name Template function for conversion <EM>from</EM> string.
  */
  /*@{*/
/// Generic template function (assumes T to be integral numeric type).
  template < class T >
  void
  convertFromString(
    const std::string & crstrSource,
    T                 & crtTarget
  ) {
    crtTarget = (T)string2Long(crstrSource);  //lint !e1024 !e69 !e737 !e734: can't cast from long to struct
  }

/// Explicit specialization for UString
//template < class UString >
  inline void
  convertFromString(
    const std::string  & crstrSource,
    icu::UnicodeString & crTarget
  ) {
    crTarget = icu::UnicodeString(crstrSource.data(), (int32_t)crstrSource.length(), CCSID::getDefaultSBCSInputCCSID());
  }

/// Explicit specialization for string
//template < class string >
  inline void
  convertFromString(
    const std::string & crstrSource,
    std::string       & crtTarget
  ) {
    crtTarget = crstrSource;
  }

/// Explicit specialization for double
//template < class double >
  inline void
  convertFromString(
    const std::string & crstrSource,
    double            & crtTarget
  ) {
    crtTarget = (double)string2Double(crstrSource); //lint !e64 !e1024: No function has same argument count as 'string2Double()', 1 candidates found
  }

/// Explicit specialization for float
//template < class float >
  inline void
  convertFromString(
    const std::string & crstrSource,
    float             & crtTarget
  ) {
    crtTarget = (float)string2Double(crstrSource); //lint !e64 !e1024: No function has same argument count as 'string2Double()', 1 candidates found
  }

/// Explicit specialization for boole
//template < class bool >
  inline void
  convertFromString(
    const std::string & crstrSource,
    bool& crtTarget
  ) {
    crtTarget = string2Bool(crstrSource); //lint !e64 !e1024: No function has same argument count as 'string2Bool()', 1 candidates found
  }
  /*@}*/

} //namespace uima

#endif /* UIMA_STRCONVERT_HPP */
/* <EOF> */

