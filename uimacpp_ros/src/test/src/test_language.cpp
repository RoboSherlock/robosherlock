/** @name uimatest_language.cpp

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

-------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <iomanip>
using namespace std;

#include "uima/assertmsg.h"
#include "uima/consoleui.hpp"

#include "uima/ftools.hpp"
#include "uima/err_ids.h"
#include "uima/strtools.hpp"
#include "uima/strconvert.hpp"
#include "uima/language.hpp"

using namespace uima;

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

#define MAIN_TITLE            _TEXT("Language Class Unit Tester")

const TCHAR * gs_szUsage = _TEXT("\t[--verbose]"
                                );

static const TCHAR *          gs_szHelp = _TEXT("\t"
    "Perform some test with the Language class.\n\t"
    "\n\t"
    "[--verbose]                 verbose display mode\n\t"
                                               );

static bool                   gs_bVerbose = false;

#define ASSERT(x) if (!(x)) { cerr << "FAILED ASSERTION '" << UIMA_STRINGIFY(x) \
 << "' in " << __FILE__ << " at line " << __LINE__ << endl; exit(1); }

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

void mainDisplay(util::ConsoleUI & rclConsole, Language & rclLang)
/* ----------------------------------------------------------------------- */
{
  char   s1[50];
  string str2;
  int id = rclLang.asNumber();
  sprintf(s1,"0x%08x %d",id,id);
  if (rclLang.hasLanguage())
    str2 = rclLang.getLanguageCode();
  else
    str2 = "**";
  if ( rclLang.hasTerritory() )
    str2 += " - " + (string)rclLang.getTerritoryCode();

  rclConsole.format(s1, str2.c_str());
}

// Display an array of Unicode characters
void ucharDisplay(util::ConsoleUI & rclConsole, char* tag, const UChar* ucbuff, int len) {
  char cbuf[1024];
  char* s = cbuf;
  *s++ = '\"';
  for ( int i = 0; i < len; ++i ) {
    if (ucbuff[i] < 128)
      s += sprintf(s,"%c",ucbuff[i]);
    else
      s += sprintf(s,"\\u%04x",ucbuff[i]);
  }
  *s++ = '\"';
  *s = '\0';
  rclConsole.format(tag, cbuf);
}


TyErrorId
mainTest(
  util::ConsoleUI &       rclConsole,
  bool                  bVerbose
) {
  TyErrorId               enErrorId = UIMA_ERR_NONE;

  rclConsole.formatHeader(_TEXT("Performing language tests:"));

  Language clTestLang1("en-US");
  Language clTestLang2("En_us");
  Language clTestLang3("en-GB");
  Language clTestLang4("en");
  Language clTestLang5("fr");
  Language clTestLang6;

  Language clInvalid1("foo");
  Language clInvalid2("fu-bar");
  Language clInvalid3("f0-ba");
  Language clInvalid4("en us");
  Language clInvalid5("és-ca");

  ASSERT( !clInvalid1.isValid());
  ASSERT( !clInvalid2.isValid());
  ASSERT( !clInvalid3.isValid());
  ASSERT( !clInvalid4.isValid());
  ASSERT( !clInvalid5.isValid());
  ASSERT( clTestLang1.isValid());
  ASSERT( clTestLang3.hasTerritory());
  ASSERT(!clTestLang4.hasTerritory());

  mainDisplay ( rclConsole, clTestLang1 );
  mainDisplay ( rclConsole, clTestLang2 );
  mainDisplay ( rclConsole, clTestLang3 );
  mainDisplay ( rclConsole, clTestLang4 );
  mainDisplay ( rclConsole, clTestLang5 );
  mainDisplay ( rclConsole, clTestLang6 );

  rclConsole.format("construct tests", "OK");

  // Test the prefix-match concept, e.g. "en" matches "en-US"
  ASSERT(!clTestLang1.matches(clTestLang3));
  ASSERT( clTestLang1.matches(clTestLang2));
  ASSERT( clTestLang1.matches(clTestLang4));
  ASSERT(!clTestLang1.matches(clTestLang5));
  ASSERT( clTestLang1.matches(clTestLang6));

  rclConsole.format("match tests", "OK");

  // Test the comparison operators
  ASSERT(clTestLang1 == clTestLang2);
  ASSERT(clTestLang1 != clTestLang3);
  ASSERT(clTestLang4 < clTestLang3);
  ASSERT(clTestLang3 < clTestLang1);

  rclConsole.format("comparison tests", "OK");

  // Test the construction from the numeric form
  Language clTestLang7(clTestLang1.asNumber());
  ASSERT(clTestLang7 == clTestLang1);

  // Test the construction of one from just the language part of another
  Language clTestLang8(clTestLang1.getLanguageCode());
  ASSERT(clTestLang1.matches(clTestLang8));
  ASSERT(clTestLang8 == clTestLang4);

  // Same test but construct from the numeric form of the language
  Language clTestLang9(clTestLang1.getLanguage());
  ASSERT(clTestLang1.matches(clTestLang9));
  ASSERT(clTestLang9 == clTestLang4);

  rclConsole.format("numeric construct tests", "OK");

  //------------------------------------------------------
  // Test some UnicodeStringRef copy functions
  //------------------------------------------------------

  int len;
  UChar ucbuf[100];
  UErrorCode err;
  std::string ss;

  char* cstr = "abcdefghijklmnopqrstuvyzyz 0123456789";
  UnicodeString us1(cstr);
  UnicodeStringRef usr(us1);

  // into std::string with default conversion
  usr.extract(ss);
  if (bVerbose) rclConsole.format("extract to std::string",ss.c_str());
  ASSERT (strlen(cstr) == ss.length());

  // substring into UnicodeString
  UnicodeString us2("initialValue");
  UnicodeString us3("z 0");
  usr.extractBetween(25,28,us2);               // Should be "z 0"
  if (bVerbose) ucharDisplay(rclConsole, "extractBetween into UChar buffer", us2.getBuffer(), us2.length());
  ASSERT ( us2 == us3 );

  // substring into UChar buffer
  err = U_ZERO_ERROR;
  len = us1.extract(ucbuf,100,err);         // Pre-fill buffer

  usr.extract(27,3,ucbuf,5);                // extract part of USR into the buffer
  UnicodeString us4("abcde012ijklm");
  if (bVerbose) ucharDisplay(rclConsole, "extract into UChar buffer", ucbuf, 13);
  ASSERT ( us4.compare(ucbuf,13) == 0 );

  // extract into too-small UChar buffer
  err = U_ZERO_ERROR;
  len = usr.extract(ucbuf,36,err);                // too small
  ASSERT(err == U_BUFFER_OVERFLOW_ERROR);
  err = U_ZERO_ERROR;
  len = usr.extract(ucbuf,37,err);                // no room for final 0
  ASSERT(err == U_STRING_NOT_TERMINATED_WARNING);
  err = U_ZERO_ERROR;
  len = usr.extract(ucbuf,38,err);                // just right
  ASSERT(err == U_ZERO_ERROR);
  if (bVerbose) ucharDisplay(rclConsole, "extract into UChar buffer", ucbuf, len);
  ASSERT ( us1.compare(ucbuf,len) == 0 );


  //------------------------------------------------------
  // Test some UnicodeStringRef conversion functions
  //------------------------------------------------------

  // Create a UnicodeString with 8 Arabic characters followed by a blank.
  // Also get the utf-8 form as a reference
  UChar uc[] = {0x062c, 0x0648, 0x0631, 0x062c, 0x062a, 0x0627, 0x0648, 0x0646, 0x0020};
  int nchars = sizeof(uc)/sizeof(UChar);
  UnicodeString US1(uc, nchars);
  char u8[100];
  US1.extract(0, US1.length(), u8, 100, "utf-8");

  // Create two UnicodeStringRef and compare them
  UnicodeStringRef USR1(US1);
  UnicodeStringRef USR2(uc, nchars);
  if (bVerbose) ucharDisplay(rclConsole, "Construct from UnicodeString", USR1.getBuffer(), USR1.length());
  if (bVerbose) ucharDisplay(rclConsole, "Construct from UChar array",   USR2.getBuffer(), USR2.length());
  ASSERT ( USR1.compare(US1) == 0 );

  // Extract into a buffer using the utf-8 converter
  char cbuf[100];
  *cbuf = 0;
  len = USR1.extract(0, USR1.length(), cbuf, 100, "utf-8");
  if (bVerbose) rclConsole.format("extract into buffer with utf-8 converter", cbuf);
  ASSERT ( strcmp(u8,cbuf) == 0 );

  // Extract into a string using the utf-8 converter
  USR1.extract(ss, "utf-8");
  if (bVerbose) rclConsole.format("extract into string with utf-8 converter", ss.c_str());
  ASSERT ( strlen(u8) == ss.length() );
  ASSERT ( strncmp(u8,ss.data(),strlen(u8)) == 0 );

  // Test the "re-try when overflows" logic in unistrref.cpp
  // Create a string that converts to >255 chars
  UnicodeString US2(uc, nchars);
  for ( int i=0; i < 15; ++i )
    US2.append(uc, nchars);

  // Extract the 12th repeat
  UnicodeStringRef USR3(US2);
  USR3.extract(11*nchars, nchars, ss, "utf-8");
  if (bVerbose) rclConsole.format("extract part into string with utf-8 converter", ss.c_str());
  ASSERT ( strlen(u8) == ss.length() );

  // Extract all to string with converter
  USR3.extract(ss, "utf-8");
  ASSERT ( 16*strlen(u8) == ss.length() );

  // Explicit extract to utf-8 (no ICU converter)
  std::string ss2;
  USR3.extractUTF8(ss2);
  ASSERT ( ss2 == ss );

  // Convert to utf-8 (no ICU converter)
  std::string ss3 = USR3.asUTF8();
  ASSERT ( ss3 == ss );

  rclConsole.format("UnicodeStringRef tests", "OK");

  return enErrorId;
}

int main(int argc, char * argv[]) /*
---------------------------------- */
{
  util::ConsoleUI         clConsole(argc, argv, MAIN_TITLE, "");
  TyErrorId               enErrorId;

  clConsole.handleUsageHelp(gs_szUsage, gs_szHelp);
  gs_bVerbose = clConsole.hasArgSwitch(_TEXT("verbose"));

  enErrorId = mainTest(clConsole, gs_bVerbose);

  if (enErrorId == UIMA_ERR_NONE) {
    clConsole.info(_TEXT("The program terminated successfully."));
  } else {
    clConsole.info(_TEXT("The program terminated with an error."));
  }
  return(int) enErrorId;
} //lint !e529 rclResMgr not subsequently referenced

/* <EOF> */

