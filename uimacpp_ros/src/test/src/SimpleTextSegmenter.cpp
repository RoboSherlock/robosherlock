/** SimpleTextSegmenter.cpp

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


Example CAS Multiplier
*/
#include <stdio.h>
#include "uima/api.hpp"
using namespace uima;
using namespace std;

class  SimpleTextSegmenter : public Annotator {
private:

  UnicodeStringRef docTextUS;

  size_t threshhold;
  size_t docLen;
  size_t start;
  size_t delimLen;
  size_t remainingLen;

  UnicodeString delimUS;
  const UChar * delimP;
  const UChar * docTextBeginP;
  const UChar * remainingTextP;
  bool hasMore;

  AnnotatorContext * pAnc;

  int segmentNum;

  /* We have a separate function getConfigValues()
     for initialize() and reconfigure()
  */
  TyErrorId getConfigValues() {
    return (TyErrorId)UIMA_ERR_NONE;
  }


public:

  SimpleTextSegmenter(void) {
    //   cout << "SimpleTextSegmenter: Constructor" << endl;
  }

  ~SimpleTextSegmenter(void) {
    //   cout << "SimpleTextSegmenter: Destructor" << endl;
  }

  /** */
  TyErrorId initialize(AnnotatorContext & rclAnnotatorContext) {
    cout << "SimpleTextSegmenter: initialize()" << endl;

    pAnc = &rclAnnotatorContext;

    /* default delimiter */
    delimUS = "\n";

    /* read in configuration parameter setting */
    UnicodeString param("SegmentDelimiter");
    if (rclAnnotatorContext.isParameterDefined(param) ) {
      rclAnnotatorContext.extractValue(param, delimUS);
    }

    delimP = delimUS.getBuffer();
    delimLen = delimUS.length();
    if (delimLen < 1 ) {
      pAnc->getLogger().logError("initialize() Invalid delimiter specified. Must be at least one character in length");
      UIMA_EXC_THROW_NEW(Exception,
                         UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT,
                         UIMA_MSG_ID_EXCON_CONFIG_VALUE_EXTRACT,
                         ErrorMessage(UIMA_MSG_ID_LITERAL_STRING, "Invalid value for SegmentDelimiter"),
                         ErrorInfo::unrecoverable);
    }

    cout << "initialize() using segment delimiter " << delimUS << " with length " << delimLen << endl;

    pAnc->getLogger().logMessage("initialize() Using Segment Delimiter '" + delimUS + "'");
    segmentNum=0;
    return (TyErrorId)UIMA_ERR_NONE;
  }

  // Segment the input text.
  TyErrorId process(CAS & cas, ResultSpecification const & crResultSpecification) {
    cout  << "SimpleTextSegmenter: process()" << endl;

	cout << endl << "Test custom index..............." << endl;
	FSIndex featureIndex = cas.getIndexRepository().getIndex("TestIndex"); 
	cout << "featureIndex.isValid() = " << featureIndex.isValid() << endl;
	cout << "Test custom index ok..............." << endl << endl;

    // Get the text document
    docTextUS = cas.getDocumentText();
    /* Get number of Unicode chars (UTF-16 code units)  */
    docLen = docTextUS.length();
    start = 0;
    remainingTextP = docTextUS.getBuffer();
    remainingLen=docLen;
    return (TyErrorId)UIMA_ERR_NONE;
  }


  bool  hasNext() {
    cout << "SimpleTextSegmenter:hasNext() " << remainingLen << endl;

    if (remainingLen < 1 || remainingTextP==NULL) {
      hasMore=false;
    } else {
      hasMore = true;
    }
    return hasMore;
  }

  /** */
  CAS &  next() {

    cout << "SimpleTextSegmenter: next()" << endl;

    if (!hasMore) {
      UIMA_EXC_THROW_NEW(Exception,
                         UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS,
                         UIMA_MSG_ID_EXCON_PROCESSING_CAS,
                         ErrorMessage(UIMA_MSG_ID_LITERAL_STRING, "There is not next() available."),
                         ErrorInfo::unrecoverable);
    }

    UChar * segEndP =  u_strFindFirst(remainingTextP, remainingLen, delimP, delimLen);
    size_t segLen=remainingLen;

    if (segEndP != NULL) {
      segLen = segEndP - remainingTextP;
    }

    //get a CAS from pool.
    CAS & cas = pAnc->getEmptyCAS();

    //create a sofa for the segment
    if (segEndP) {
      UnicodeStringRef segStr(remainingTextP, segLen+delimLen);
      remainingTextP = segEndP + delimLen;
      remainingLen = remainingLen-(segLen+delimLen);
      cas.setDocumentText(segStr.getBuffer(), segStr.length());
    } else {
      if (remainingLen > 0) {   //when delim not found, create Sofa with remaining
        UnicodeStringRef segStr(remainingTextP, remainingLen);

        cas.setDocumentText(segStr.getBuffer(), segStr.length());
      }
      remainingLen=0;
      remainingTextP = NULL;
    }

    return cas;
  }

  int  getCasInstancesRequired() {
    return 1;
  }

  /** */
  TyErrorId typeSystemInit(TypeSystem const & crTypeSystem) {
    cout << " typeSystemInit()" << endl;

    return (TyErrorId)UIMA_ERR_NONE;
  }

  /** */
  TyErrorId destroy() {
    cout << "SimpleTextSegmenter: destroy()" << endl;
    return (TyErrorId)UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.

MAKE_AE(SimpleTextSegmenter);
