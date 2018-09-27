/** \file ss_tokenizer.hpp .
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

-------------------------------------------------------------------------- */

#ifndef _INCLUDE_UIMASS
#define _INCLUDE_UIMASS

#include "uima/language.hpp"
#include "uima/token_properties.hpp"

namespace uima {

  class ResourceABR;



  static const int MAXWARD = 6;

  typedef unsigned short TyCharmap [MAXWARD+1][256];

  /**character types used in our char map*/
  typedef enum {
    CH_INVALID = 0,
    CH_LWR = 1,    // lowercase characters
    CH_UPR = 2,    // uppercase characters
    CH_NUM = 4,    // number or currency symbol
    CH_USC = 8,    // underscore: like a character, no upper/lower information
    CH_PRD = 16,   // period (full stop)
    CH_SND = 32,   // sentence end: '?' and '!'
    CH_BLK = 64,   // blank
    CH_NWL = 128,  // newline
    CH_SPC = 256,  // special character (or whitespace)
    CH_CWS = 512,  // conditional whitespace: if character is between two
    // alphanumeric characters, then it becomes part of
    // the word, e.g / @ -
    // if not, it's treated as a whitespace
    CH_NSP = 1024, // number seperator ':' and ',' part of the number
    // if between digits
    CH_APS = 2048, // apostroph
    CH_NPA = 4096, // new paragraph
    CH_CUR = 8192  // currency and degree symbol: part of number if after of before digit
  }
  EnCharClass;

#define CHAR_CLASS_IS_TOKEN_PART(x)     ((x) < CH_PRD)

  /** @name Tokenizer
     The class <TT>Tokenizer</TT> is the implementation of an universal Unicode
     Tokenizer which is used in the UIMA tokenizer annotator.
     @see AnnotatorTokenizer
  */
  class Tokenizer {
  public:
    /** Default Constructor.
    */
    Tokenizer( void );
    virtual                    ~Tokenizer();
    /// Main tokenization function
    void                       process( const UChar *cpszStart, const UChar *cpszEnd );
    /// Specify language to use (needed for stopword recognition only)
    void                       setLanguage( const Language & crclLanguage );
    /// Callback function triggered on token recognition
    virtual int                tokenCallback( unsigned long ulLocation,
        unsigned long ulLength,
        TokenProperties & crclTokenProperties,
        bool bNewPara, bool bNewSent ) = 0;

    EnCharClass                getCharClass(UChar c);

    // change the character class for a code point
    void                       setCharClass(WORD16 uiUnicodeCodePoint,
                                            EnCharClass enCharClass);

    // reset char class table to initial values
    void                       resetCharClasses(void);

  protected:
    int                        tokenEntry( const UChar *, size_t ulLocation,
                                           size_t ulLength,
                                           TokenProperties & crclTokenProperties,
                                           bool &bNewPara, bool &bNewSent,
                                           size_t & rulNewlines);

  private:
    bool                       isAbreviation(const UChar * pw16String, size_t uiLength) const;
    EnCharClass                getCharClassInl( UChar c );

    // get character class to a character
    bool                       iv_bUseAlternateTerritories;
    Language                   iv_clLanguageABR;
    ResourceABR *              iv_pclResourceABR;
    // this will either point to our constant static map or
    // to a freshly allocated writable map if setCharClass has been called
    TyCharmap             *    iv_pauiCharMapWard;

  };

} // namespace uima

#endif /* _INCLUDE_UIMASS */
