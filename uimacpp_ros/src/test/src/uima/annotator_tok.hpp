#ifndef UIMA_ANNOTATOR_TOK_H$
#define UIMA_ANNOTATOR_TOK_H$
/** \file annotator_tok.hpp .
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


   \brief  Contains AnnotatorTokenizer a Unicode UIMA Tokenizer Annotator.

-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

/* We want the timers in this annotator to be only active if the annotator specific
   define ANNOTATOR_TIMERS is set (e.g. in the automake.pro file for this annotator)
   in all other cases we don't want the timers.
   Since all the timers depend on the more generic define DEBUG_TIMING we
   define DEBUG_TIMING if and only if ANNOTATOR_TIMERS is set.
   Specificaly we don't want the timers set when DEBUG_TIMING is defined to
   build a generic timing driver of the whole system.
   Our internal annotator timers would bias the whole timing driver with the
   overhead involved in calling them in this annotator. This is why we specificaly
   undefine DEBUG_TIMING even if it might be set in the makefile to build this
   annotator.
   If you want timing in this annotator use ANNOTATOR_TIMERS not DEBUG_TIMING
 */
#ifdef ANNOTATOR_TIMERS
#  ifndef DEBUG_TIMING
#     define DEBUG_TIMING
#  endif
#else
#  ifdef DEBUG_TIMING
#     undef DEBUG_TIMING
#  endif
#endif

#include "uima/timedatetools.hpp"
#include "uima/api.hpp"                          /* UIMA API */
///////#include "uima/u2cpcnvrtbuff.hpp"                 /* U2CpConvertBuffer */
#include "uima/ss_tokenizer.hpp"
#include "uima/internal_casimpl.hpp"

#define STEMMER_BUF_LEN 50

using namespace uima;

/** @name AnnotatorTokenizer
   The class <TT>AnnotatorTokenizer</TT> is used a universal Unicode Tokenizer.

   It uses a little trick to check API consistency via an abstract base class,
   without having the overhead of virtual functions in our ship version.
   <TT>AnnotatorABase</TT> defines all non-static member functions a plug-in needs
   to define as pure virtual functions. By making this class inherit from
   this base class we can make sure that compilation will fail if the
   interfaces change.
   Since we don't really use the inheritance relationship we don't define
   it in the ship version.

   Example:
   \code
   \endcode
   @see AnnotatorABase
*/
class AnnotatorTokenizer : public Tokenizer , public TextAnnotator {
public:
  /** @name Constructors */
  /*@{*/
  /** Default Constructor.
  */
  AnnotatorTokenizer(void);

  /*@}*/
  virtual ~AnnotatorTokenizer(void);  //lint !e1509: base class destructor for class 'AnnotatorABase' is not virtual

  /** @name Annotator Processing Functions */
  /*@{*/
  /** call the UIMA Annotator to initialize itself based on a UIMA engine
      and a UIMA Configuration section and return a UIMA error code */
  TyErrorId
  initialize(
    AnnotatorContext & rclAnnotatorContext
  );  //lint !e1909: 'virtual' assumed, see: AnnotatorABase::init(Engine &, ConfigAnnotator &) (line 79, file d:\develop\uima\current\code\engine\include\annotator_abase.hpp)

  TyErrorId typeSystemInit(TypeSystem const &);

  /** call the UIMA Annotator to deinitialize itself based on a UIMA engine
      and return a UIMA error code */
  TyErrorId
  destroy();  //lint !e1909: 'virtual' assumed, see: AnnotatorABase::deInit(Engine &) (line 83, file d:\develop\uima\current\code\engine\include\annotator_abase.hpp)

  /** call the UIMA Annotator to reconfigure itself based on a UIMA Configuration
      section and return a UIMA error code */
  TyErrorId
  reconfigure(
  );  //lint !e1909: 'virtual' assumed, see: AnnotatorABase::config(ConfigAnnotator &) (line 87, file d:\develop\uima\current\code\engine\include\annotator_abase.hpp)

protected:
  /** call the UIMA Annotator to perform its doc related duty based on a UIMA engine
      and return a UIMA error code */
  TyErrorId
  process(
    CAS &,
    const ResultSpecification & crclTargetSet
  );  //lint !e1909: 'virtual' assumed, see: AnnotatorABase::processDocument(Engine &, const TargetSetAT &, const TargetSetTT &) (line 91, file d:\develop\uima\current\code\engine\include\annotator_abase.hpp)

  /*@}*/
protected:

  virtual int tokenCallback( unsigned long ulLocation, unsigned long ulLength,
                             TokenProperties & crclTokenProperties,
                             bool bNewPara, bool bNewSent );

  TyDocIndex                 iv_uiParagraphStartIndex;
  TyDocIndex                 iv_uiSentenceStartIndex;
  // segment numbers
  size_t                     iv_uiTokenNbr;
  size_t                     iv_uiSentenceNbr;
  size_t                     iv_uiParagraphNbr;
private:
  // number of the first token/sentence/paragraph
  const size_t               iv_cuiCOUNTER_START;

  /* --------------------------------------------------*/
  /*   config values we use                            */
  /* --------------------------------------------------*/

  /// Enum listing all the config option we support
  enum EnAnnotatorConfigOptions {
    enConfigOption_TokenNumbersIncludeStopwords,
    enConfigOption_UseRelativeTokenAndSentenceNumbers,
    enConfigOption_IgnorePunctuationTokens,
    // (drop inifile support) enConfigOption_CharMapConfigFilename,
    enNumberOfConfigOptions  // must be last in enum
  };

  // our config table
  static const ConfigOptionInfo::StOptionInfo cv_astConfigOptionInfo[enNumberOfConfigOptions];

  // Variables the config options are stored in:

  // if this is true the token numbers are counted including stopwords
  bool                    iv_bTokenNumbersIncludeStopwords;
  // if this is true token and sentence number are reset to 1
  // for each new sentence/paragraph
  bool                    iv_bUseRelativeTokenAndSentenceNumbers;
  // If true, punctuation tokens are ignored
  bool                    iv_bIgnorePunctuationTokens;
  // trace component ID
  uima::TyComponentId                   iv_iTraceCompID;

  // Some pointers for quick access to UIMA objects. Initialized in init()
  uima::internal::CASImpl * iv_pCASImpl;
  lowlevel::FSHeap * iv_pFSHeap;
  lowlevel::IndexRepository * iv_pIndexRepository;
  // FSTypes and corresponding sizes
  lowlevel::TyFSType iv_tyTokenType;
  lowlevel::TyFeatureOffset iv_tyTokenTypeSize;
  lowlevel::TyFSType iv_tySentenceType;
  lowlevel::TyFeatureOffset iv_tySentenceTypeSize;
  lowlevel::TyFSType iv_tyParagraphType;
  lowlevel::TyFeatureOffset iv_tyParagraphTypeSize;

  // FSFeatures
  lowlevel::TyFeatureOffset  iv_tySofaFeatureOffset;
  lowlevel::TyFeatureOffset  iv_tyBeginPositionFeatureOffset;
  lowlevel::TyFeatureOffset  iv_tyEndPositionFeatureOffset;

  lowlevel::TyFeatureOffset  iv_tyTokenPropertiesFeatureOffset;
  lowlevel::TyFeatureOffset  iv_tyTokenNbrFeatureOffset;
  lowlevel::TyFeatureOffset  iv_tySentenceNbrFeatureOffset;
  lowlevel::TyFeatureOffset  iv_tyParagraphNbrFeatureOffset;
  lowlevel::TyFSFeature  iv_stemFeature;

  // needed output types
  bool                       iv_bIsTokenReq;
  bool                       iv_bIsSentenceReq;
  bool                       iv_bIsParagraphReq;
  bool                       iv_stemsRequired;
  TokenProperties            iv_clTokenProperties;

#ifdef DEBUG_TIMING
  uima::Timer                 iv_clTotalTimer;
  uima::Timer                 iv_clSSTokTimer;
  uima::Timer                 iv_clUimaAnCreateTimer;
  uima::Timer                 iv_clUimaAnSetValTimer;
#endif


  /* --- functions --- */
  /* COPY CONSTRUCTOR NOT SUPPORTED */
  AnnotatorTokenizer(const AnnotatorTokenizer & ); //lint !e1704
  /* ASSIGNMENT OPERATOR NOT SUPPORTED */
  AnnotatorTokenizer & operator=(const AnnotatorTokenizer & crclObject);

  /// (Re-) Access config values. Used in init() and config().
  TyErrorId
  getConfigValues(
    AnnotatorContext          & rclConfig
  );

  /// member functions for adding annotations
  void
  addNewTokenAnnotation(
    TyDocIndex           tyBeginPos,
    TyDocIndex           tyEndPos
  );

  /// member functions for adding annotations
  void
  addNewSentenceAnnotation(
    TyDocIndex           tyBeginPos,
    TyDocIndex           tyEndPos
  );

  /// member functions for adding annotations
  void
  addNewParagraphAnnotation(
    TyDocIndex           tyBeginPos,
    TyDocIndex           tyEndPos
  );

#if defined(DEBUG_TIMING)
  void
  dumpTimingData( void ) const;
#endif

}
; /* AnnotatorTokenizer */

/* ----------------------------------------------------------------------- */
#endif /* UIMA_ANNOTATOR_TOK_H */

/* <EOF> */

