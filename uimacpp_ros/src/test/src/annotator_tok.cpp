/**

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

   Description: A Unicode UIMA Tokenizer Annotator.

-------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

// this is included ONCE for the main source file of each binary
#include "uima/annotator_tok.hpp"
#include "uima/ss_tokenizer.hpp"               // sentsep for Uima
#include "uima/tt_types.hpp"

#include "uima/assertmsg.h"
#include "uima/macros.h"
#include "uima/trace.hpp"
#include "uima/comp_ids.h"                             /* for trace */

/* ----------------------------------------------------------------------- */
/*       Globals                                                           */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Config values                                                     */
/* ----------------------------------------------------------------------- */

// Const table with all the option we access in the config file
// The sequence of entries herer must match the enum EnAnnotatorConfigOptions
// in the hpp file.
const ConfigOptionInfo::StOptionInfo
AnnotatorTokenizer::cv_astConfigOptionInfo[] = {
      {
        "TokenNumbersIncludeStopwords",              //cpszOptionName
        ConfigOptionInfo::enValueType_Boolean,  //enValueType
        false,                                       //bOptionIsMultiValued
        0,                                           //uiNbrOfValuesRequired
        "true",                                      //cpszDefaultValueAsString
        "If true token numbers are counted including stopwords"    //cpszComment
      },
      {
        "UseRelativeTokenAndSentenceNumbers",        //cpszOptionName
        ConfigOptionInfo::enValueType_Boolean,  //enValueType
        false,                                       //bOptionIsMultiValued
        0,                                           //uiNbrOfValuesRequired
        "false",                                     //cpszDefaultValueAsString
        "If true token and sentence numbers are reset to 1 for each new sentence/paragraph"    //cpszComment
      },
      {
        "IgnorePunctuationTokens",                   //cpszOptionName
        ConfigOptionInfo::enValueType_Boolean,  //enValueType
        false,                                       //bOptionIsMultiValued
        0,                                           //uiNbrOfValuesRequired
        "false",                                     //cpszDefaultValueAsString
        "If true, punctuation tokens are ignored"    //cpszComment
      }
    };

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

/** Default Constructor.
*/
AnnotatorTokenizer::AnnotatorTokenizer(void) :
    iv_uiParagraphStartIndex(0),
    iv_uiSentenceStartIndex(0),
    iv_uiTokenNbr(0),
    iv_uiSentenceNbr(0),
    iv_uiParagraphNbr(0),
    iv_cuiCOUNTER_START(1),
    iv_bTokenNumbersIncludeStopwords(true),
    iv_bUseRelativeTokenAndSentenceNumbers(false),
    iv_bIgnorePunctuationTokens(false),
    iv_iTraceCompID(UIMA_TRACE_COMPID_ANNOTATOR_DEFAULT),
    iv_pCASImpl(NULL),
    iv_pFSHeap(NULL),
    iv_tyTokenType(0),
    iv_tyTokenTypeSize(0),
    iv_tySentenceType(0),
    iv_tySentenceTypeSize(0),
    iv_tyParagraphType(0),
    iv_tyParagraphTypeSize(0),
    iv_tySofaFeatureOffset(0),
    iv_tyBeginPositionFeatureOffset(0),
    iv_tyEndPositionFeatureOffset(0),
    iv_tyTokenNbrFeatureOffset(0),
    iv_tySentenceNbrFeatureOffset(0),
    iv_tyParagraphNbrFeatureOffset(0),
    iv_stemFeature(0),
    iv_bIsTokenReq(false),
    iv_bIsSentenceReq(false),
    iv_bIsParagraphReq(false),
    iv_stemsRequired(false)
#ifdef DEBUG_TIMING
    ,
    iv_clTotalTimer(),
    iv_clSSTokTimer(),
    iv_clUimaAnCreateTimer(),
    iv_clUimaAnSetValTimer()
#endif
 {}

    AnnotatorTokenizer::~AnnotatorTokenizer(void) {
  util::Trace clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, iv_iTraceCompID);
  // set pointer to NULL to make clear this object did not own them
  iv_pFSHeap                 = NULL;
}

TyErrorId
AnnotatorTokenizer::getConfigValues(AnnotatorContext & rANC) {
  // this must be done before any trace call
  iv_iTraceCompID = rANC.getTraceCompId();

  util::Trace clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, iv_iTraceCompID);

  // make sure we have an entry in our config value table for each enum
  assert(NUMBEROF(cv_astConfigOptionInfo) == enNumberOfConfigOptions);

  TyErrorId utErrID;

  // Get Option: token numbers are counted including stopwords?
  utErrID = extractConfigOptionBoolean(
              rANC,
              cv_astConfigOptionInfo[enConfigOption_TokenNumbersIncludeStopwords],
              iv_bTokenNumbersIncludeStopwords
            );
  if (utErrID != UIMA_ERR_NONE) {
    return utErrID;
  }

  // Get Option: token and sentence number are reset to 1 for each new sentence/paragraph
  utErrID = extractConfigOptionBoolean(
              rANC,
              cv_astConfigOptionInfo[enConfigOption_UseRelativeTokenAndSentenceNumbers],
              iv_bUseRelativeTokenAndSentenceNumbers
            );
  if (utErrID != UIMA_ERR_NONE) {
    return utErrID;
  }

  // Get Option:  If true, punctuation tokens are ignored
  utErrID = extractConfigOptionBoolean(
              rANC,
              cv_astConfigOptionInfo[enConfigOption_IgnorePunctuationTokens],
              iv_bIgnorePunctuationTokens
            );
  if (utErrID != UIMA_ERR_NONE) {
    return utErrID;
  }

  return UIMA_ERR_NONE;
}

TyErrorId
AnnotatorTokenizer::initialize(
  AnnotatorContext & rclAnnotatorContext
) {
  util::Trace clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, iv_iTraceCompID);
  return UIMA_ERR_NONE;
}

TyErrorId AnnotatorTokenizer::typeSystemInit(TypeSystem const & typeSystem) {

  if (getConfigValues(getAnnotatorContext()) != UIMA_ERR_NONE) {
    return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
  }

  lowlevel::TypeSystem const & crTypeSystem = uima::lowlevel::TypeSystem::promoteTypeSystem(typeSystem);

  iv_tyTokenType = crTypeSystem.getTypeByName( TT::TYPE_NAME_TOKEN_ANNOTATION );
  iv_tySentenceType = crTypeSystem.getTypeByName( TT::TYPE_NAME_SENTENCE_ANNOTATION );
  iv_tyParagraphType = crTypeSystem.getTypeByName( TT::TYPE_NAME_PARAGRAPH_ANNOTATION );
  iv_tySentenceTypeSize = crTypeSystem.getFeatureNumber( iv_tySentenceType );
  iv_tyParagraphTypeSize = crTypeSystem.getFeatureNumber( iv_tyParagraphType );
  iv_tyTokenTypeSize = crTypeSystem.getFeatureNumber( iv_tyTokenType );
  iv_tySofaFeatureOffset = crTypeSystem.getFeatureOffset( uima::internal::gs_tySofaRefFeature );
  iv_tyBeginPositionFeatureOffset = crTypeSystem.getFeatureOffset( uima::internal::gs_tyBeginPosFeature );
  iv_tyEndPositionFeatureOffset = crTypeSystem.getFeatureOffset( uima::internal::gs_tyEndPosFeature );
  iv_tyTokenPropertiesFeatureOffset = crTypeSystem.getFeatureOffset( crTypeSystem.getFeatureByBaseName( iv_tyTokenType, TT::FEATURE_BASE_NAME_TOKEN_PROPERTIES ) );
  iv_tyTokenNbrFeatureOffset = crTypeSystem.getFeatureOffset( crTypeSystem.getFeatureByBaseName( iv_tyTokenType, TT::FEATURE_BASE_NAME_TOKEN_NUMBER ) );
  iv_tySentenceNbrFeatureOffset = crTypeSystem.getFeatureOffset( crTypeSystem.getFeatureByBaseName( iv_tySentenceType, TT::FEATURE_BASE_NAME_SENTENCE_NUMBER ) );
  iv_tyParagraphNbrFeatureOffset = crTypeSystem.getFeatureOffset( crTypeSystem.getFeatureByBaseName( iv_tyParagraphType, TT::FEATURE_BASE_NAME_PARAGRAPH_NUMBER ) );
  iv_stemFeature = crTypeSystem.getFeatureByBaseName( iv_tyTokenType, "stem");
  return (TyErrorId)UIMA_ERR_NONE;
}

/** call the UIMA Annotator to deinitialize itself based on a UIMA engine
    and return a UIMA error code */
TyErrorId
AnnotatorTokenizer::destroy() {
  util::Trace  clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, iv_iTraceCompID);

#if defined(DEBUG_TIMING)
  dumpTimingData();
#endif

  resetCharClasses();

  return (TyErrorId)UIMA_ERR_NONE;
}

#if defined(DEBUG_TIMING)
void AnnotatorTokenizer::dumpTimingData( void ) const {
  cout << "----------------------------------------------------------------" << endl;
  cout << " AnnotatorTokenizer: " << endl;
  cout << " total amount of time:" << iv_clTotalTimer.timeString() << endl;
  ;
  cout << "   Sentsep        :  " << (iv_clSSTokTimer-iv_clUimaAnCreateTimer-iv_clUimaAnSetValTimer).timeAndPercentString(iv_clTotalTimer) << endl;
  ;
  cout << "   Create ANs     :  " << iv_clUimaAnCreateTimer.timeAndPercentString(iv_clTotalTimer) << endl;
  ;
  cout << "   Set AN Values  :  " << iv_clUimaAnSetValTimer.timeAndPercentString(iv_clTotalTimer) << endl;
  ;
  cout << "----------------------------------------------------------------" << endl;
}
#endif

/** call the UIMA Annotator to reconfigure itself based on a UIMA Configuration
    section and return a UIMA error code */
TyErrorId
AnnotatorTokenizer::reconfigure(
) {
  util::Trace  clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, iv_iTraceCompID);
  resetCharClasses();
  if (getConfigValues(getAnnotatorContext()) != UIMA_ERR_NONE) {
    return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
  }
  return (TyErrorId)UIMA_ERR_NONE;
}

inline void
AnnotatorTokenizer::addNewTokenAnnotation(
  TyDocIndex           tyBeginPos,
  TyDocIndex           tyEndPos
) {
  // If we should filter punctuation chars (in TSE mode we have to do that!)
  if (    iv_bIgnorePunctuationTokens
          && (tyEndPos == tyBeginPos+1) // length == 1
          && iv_clTokenProperties.hasSpecial()
     ) {
    return;
  }

  // Create new Annotation of type Token
  lowlevel::TyFS tyNewToken = iv_pFSHeap->createFSWithSize( iv_tyTokenType, iv_tyTokenTypeSize );

  // Set the TokenNumber Attribute for the new Token Annotation
  iv_pFSHeap->setFeatureWithOffset( tyNewToken, iv_tySofaFeatureOffset, lowlevel::FSHeap::getAsFS((int) iv_pCASImpl->getSofaNum()) );
  iv_pFSHeap->setFeatureWithOffset( tyNewToken, iv_tyBeginPositionFeatureOffset, lowlevel::FSHeap::getAsFS((int) tyBeginPos) );
  iv_pFSHeap->setFeatureWithOffset( tyNewToken, iv_tyEndPositionFeatureOffset, lowlevel::FSHeap::getAsFS((int) tyEndPos) );
  iv_pFSHeap->setFeatureWithOffset( tyNewToken, iv_tyTokenNbrFeatureOffset, lowlevel::FSHeap::getAsFS( (int) iv_uiTokenNbr) );

  unsigned long ulTokProp = iv_clTokenProperties.to_ulong();
// **why this?**    assert( sizeof(unsigned long) <= sizeof(lowlevel::TyFS) );
  iv_pFSHeap->setFeatureWithOffset( tyNewToken, iv_tyTokenPropertiesFeatureOffset, (lowlevel::TyFS) ulTokProp);

  // add to index
  iv_pIndexRepository->add(tyNewToken);
  ++iv_uiTokenNbr;
}

inline void
AnnotatorTokenizer::addNewSentenceAnnotation(
  TyDocIndex           tyBeginPos,
  TyDocIndex           tyEndPos
) {
  if (   (!iv_bIsSentenceReq)
         || (tyEndPos == tyBeginPos+1) // length == 1
     ) {
    return;
  }

  // Create new Annotation of type Sentence
  lowlevel::TyFS tyNewSentence = iv_pFSHeap->createFSWithSize( iv_tySentenceType, iv_tySentenceTypeSize );

  // Set the SentenceNumber Attribute for the new Sentence Annotation
  iv_pFSHeap->setFeatureWithOffset( tyNewSentence, iv_tySofaFeatureOffset, lowlevel::FSHeap::getAsFS((int) iv_pCASImpl->getSofaNum()) );
  iv_pFSHeap->setFeatureWithOffset( tyNewSentence, iv_tyBeginPositionFeatureOffset, lowlevel::FSHeap::getAsFS((int)tyBeginPos) );
  iv_pFSHeap->setFeatureWithOffset( tyNewSentence, iv_tyEndPositionFeatureOffset, lowlevel::FSHeap::getAsFS((int)tyEndPos) );
  iv_pFSHeap->setFeatureWithOffset( tyNewSentence, iv_tySentenceNbrFeatureOffset, lowlevel::FSHeap::getAsFS( (int) iv_uiSentenceNbr) );

  // add to index
  iv_pIndexRepository->add(tyNewSentence);
}

inline void
AnnotatorTokenizer::addNewParagraphAnnotation(
  TyDocIndex           tyBeginPos,
  TyDocIndex           tyEndPos
) {
  if (   (!iv_bIsParagraphReq)
         || (tyEndPos == tyBeginPos+1) // length == 1
     ) {
    return;
  }
  // Create new Annotation of type Paragrah
  lowlevel::TyFS tyNewParagraph = iv_pFSHeap->createFSWithSize( iv_tyParagraphType, iv_tyParagraphTypeSize );

  // Set the ParagraphNumber Attribute for the new Paragraph Annotation
  iv_pFSHeap->setFeatureWithOffset( tyNewParagraph, iv_tySofaFeatureOffset, lowlevel::FSHeap::getAsFS((int) iv_pCASImpl->getSofaNum()) );
  iv_pFSHeap->setFeatureWithOffset( tyNewParagraph, iv_tyBeginPositionFeatureOffset, lowlevel::FSHeap::getAsFS((int)tyBeginPos) );
  iv_pFSHeap->setFeatureWithOffset( tyNewParagraph, iv_tyEndPositionFeatureOffset, lowlevel::FSHeap::getAsFS((int)tyEndPos) );
  iv_pFSHeap->setFeatureWithOffset( tyNewParagraph, iv_tyParagraphNbrFeatureOffset, lowlevel::FSHeap::getAsFS( (int) iv_uiParagraphNbr) );

  // add to index
  iv_pIndexRepository->add(tyNewParagraph);
}

int AnnotatorTokenizer::tokenCallback(
  unsigned long location,
  unsigned long length,
  TokenProperties & rclTokenProperties,
  bool bNewPara,
  bool bNewSent
) {
  // compute begin and end index of new token
  TyDocIndex uiTokenStartIndex = (TyDocIndex) location;
  TyDocIndex uiTokenEndIndex   = uiTokenStartIndex + (TyDocIndex) (length);
  iv_clTokenProperties = rclTokenProperties;

  if ( (bNewPara || bNewSent) &&  iv_bIsSentenceReq) {

    // new paragaph means no explicit new sentence - end current sentence anyway
    addNewSentenceAnnotation(iv_uiSentenceStartIndex, uiTokenStartIndex );

    // new sentence starts at beginning of current word
    iv_uiSentenceStartIndex = uiTokenStartIndex;
    ++iv_uiSentenceNbr;

    // reset token number
    if (iv_bUseRelativeTokenAndSentenceNumbers) {
      iv_uiTokenNbr = iv_cuiCOUNTER_START;
    }
  }

  // new paragraph started
  if ( bNewPara && iv_bIsParagraphReq) {

    // new paragraph is from StartParagraphIndex to EndLastWordIndex
    addNewParagraphAnnotation(iv_uiParagraphStartIndex, uiTokenStartIndex );

    // new start of paragraph is at beginning of current word
    iv_uiParagraphStartIndex = uiTokenStartIndex;
    ++iv_uiParagraphNbr;

    // reset sentence number
    if (iv_bUseRelativeTokenAndSentenceNumbers) {
      iv_uiSentenceNbr = iv_cuiCOUNTER_START;
      assert( iv_uiTokenNbr == iv_cuiCOUNTER_START );
    }
  }

  // finally, mark new token
  addNewTokenAnnotation(uiTokenStartIndex, uiTokenEndIndex );

  return 0;
}

/** call the UIMA Annotator to perform its duty based on a UIMA engine
    and return a UIMA error code */
TyErrorId
AnnotatorTokenizer::process(
  CAS & tcas,
  const ResultSpecification & crclTargetSet
) {
  util::Trace                  clTrace(util::enTraceDetailMedium, UIMA_TRACE_ORIGIN, iv_iTraceCompID);

  iv_pCASImpl = & uima::internal::CASImpl::promoteCAS(tcas);
  iv_pFSHeap = & iv_pCASImpl->getHeap();
  iv_pIndexRepository = & iv_pCASImpl->getIndexRepository();


  UIMA_TIMING(iv_clTotalTimer.start());

#ifdef DEBUG_VERBOSE
  UIMA_TPRINT( "ResultSpec" );
  crclTargetSet.print(cout);
#endif

  uima::lowlevel::TypeSystem const & lolTS = iv_pCASImpl->getHeap().getTypeSystem();
  Type tokType = uima::internal::FSPromoter::promoteType( iv_tyTokenType, lolTS);
  Type sentType = uima::internal::FSPromoter::promoteType( iv_tySentenceType, lolTS);
  Type parType = uima::internal::FSPromoter::promoteType( iv_tyParagraphType, lolTS);
  Feature stemFeature = uima::internal::FSPromoter::promoteFeature( iv_stemFeature, lolTS );

  // Check the target AT set: can contain token/sentence/paragraph
  iv_bIsTokenReq    = crclTargetSet.shouldBeCreatedByAnnotator( tokType );
  iv_bIsSentenceReq = crclTargetSet.shouldBeCreatedByAnnotator( sentType );
  iv_bIsParagraphReq= crclTargetSet.shouldBeCreatedByAnnotator( parType );
  iv_stemsRequired = crclTargetSet.shouldBeCreatedByAnnotator( stemFeature );

  // if none of them is required, why are we beeing called?
  assert(iv_bIsTokenReq || iv_bIsSentenceReq || iv_bIsParagraphReq);

  iv_uiTokenNbr     = iv_cuiCOUNTER_START;
  iv_uiSentenceNbr  = iv_cuiCOUNTER_START;
  iv_uiParagraphNbr = iv_cuiCOUNTER_START;
  iv_uiParagraphStartIndex = 0;
  iv_uiSentenceStartIndex = 0;

//    setLanguage(tcas.getDocumentAnnotation().getLanguage());

  UnicodeStringRef ulStrDoc(tcas.getDocumentText());

  if (ulStrDoc.length() == 0) {
    return(TyErrorId)UIMA_ERR_NONE;
  }

  UIMA_TIMING(iv_clSSTokTimer.start());
  Tokenizer::process( ulStrDoc.getBuffer(), ulStrDoc.getBuffer()+ulStrDoc.length()-1 );
  UIMA_TIMING(iv_clSSTokTimer.stop());

  //
  // terminate both sentence and paragraph up to the end of the document
  //
  if (iv_uiSentenceStartIndex < ulStrDoc.length()-1) {
    addNewSentenceAnnotation(iv_uiSentenceStartIndex, ulStrDoc.length() );
  }

  if (iv_uiParagraphStartIndex < ulStrDoc.length()-1) {
    addNewParagraphAnnotation(iv_uiParagraphStartIndex, ulStrDoc.length() );
  }
  UIMA_TIMING(iv_clTotalTimer.stop());

  return(TyErrorId)UIMA_ERR_NONE;
}


/* ----------------------------------------------------------------------- */
/*   Mapping for generic C API wrapper                                     */
/* ----------------------------------------------------------------------- */

typedef AnnotatorTokenizer UserDefinedAnnotator;
// define for error/exception info in annotator_generic.inl
#define UIMA_ANNOTATOR_NAME "annotator_tok"

/* ----------------------------------------------------------------------- */
/*   Include generic C API wrapper                                         */
/* ----------------------------------------------------------------------- */

//#include "uima/annotator_generic.inl"
MAKE_AE(AnnotatorTokenizer);
/* <EOF> */

