/*
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
 */

#include "uima/api.hpp"
using namespace std;
using namespace uima;


class SofaDataAnnotator : public Annotator {
private:
  AnnotatorContext * pAnc;
  Type annot;
public:

  SofaDataAnnotator(void) {
    cout << "SofaDataAnnotator: Constructor" << endl;
  }

  ~SofaDataAnnotator(void) {
    cout << "SofaDataAnnotator: Destructor" << endl;
  }

  /** */
  TyErrorId initialize(AnnotatorContext & rclAnnotatorContext) {
    cout << "SofaDataAnnotator: initialize()" << endl;

    // Save the annotator context for use in process()
    pAnc = &rclAnnotatorContext;

    return (TyErrorId)UIMA_ERR_NONE;
  }


  TyErrorId typeSystemInit(TypeSystem const & crTypeSystem) {
    cout << "SofaDataAnnotator: typeSystemInit()" << endl;
    annot  = crTypeSystem.getType("uima.tcas.Annotation");
    return(TyErrorId)UIMA_ERR_NONE;
  }

  /** */
  TyErrorId destroy() {
    cout << "SofaDataAnnotator: destroy()" << endl;
    return (TyErrorId)UIMA_ERR_NONE;
  }

  // Look for "EnglishDocument" sofa and read it as a stream
  TyErrorId process(CAS & rCas, ResultSpecification const & crResultSpecification) {
    cout << "SofaDataAnnotator: process() begins" << endl;

    /** get the CAS view of the sofa */
    CAS * tcas = rCas.getView("EnglishDocument");
    /** get the handle to the index repository */
    FSIndexRepository & indexRep = tcas->getIndexRepository();

    /** get the default text sofa */
    SofaFS textSofa = tcas->getSofa();

    /** get the handle to the sofa data stream */
    SofaDataStream * pStream = textSofa.getSofaDataStream();
    /** open the stream */
    int rc = pStream->open();
    if (rc != 0) {
      cout << "open failed "  << rc << endl;
      return (TyErrorId)UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS;
    }
    /** get the total stream size */
    size_t streamSize = pStream->getTotalStreamSizeInBytes();

    /** read file contents into a buffer */
    char * pBuffer = new char[streamSize+1];
    memset(pBuffer,'\n' ,streamSize+1);
    int elementsize=1;
    pStream->read(pBuffer, elementsize, streamSize);

    cout << endl;
    cout.write(pBuffer, streamSize);
    cout << endl;

    /** convert to unicode */
    UnicodeString ustrInputText(pBuffer, streamSize+1, "utf-8");

    /** find tokens and annotate */
    UnicodeString delim(" ");
    UChar *myLocalSaveState;
    UChar * pInputText = (UChar*) ustrInputText.getBuffer();
    const UChar * pToken = pInputText;
    const UChar * pNextToken = u_strtok_r((UChar*) pInputText, delim.getBuffer(), &myLocalSaveState);
    int start = 1;
    int tokenlength=0;
    int nTokens = 0;
    while ( (pNextToken=u_strtok_r(NULL, delim.getBuffer(), &myLocalSaveState)) ) {
      tokenlength = pNextToken - pToken;
      AnnotationFS annotFS = tcas->createAnnotation(annot, start, start+tokenlength-2);
      indexRep.addFS(annotFS);
      ++nTokens;
      start += tokenlength;
      pToken = pNextToken;
    }
    /* last token */
    tokenlength = pNextToken - pToken;
    AnnotationFS annotFS = tcas->createAnnotation(annot, start, streamSize);
    indexRep.addFS(annotFS);
    ++nTokens;
    cout << endl << "   Annotated " << nTokens << " tokens." << endl << endl;

    /** close the stream */
    pStream->close();
    delete pStream;
    delete[] pBuffer;

    cout << "SofaDataAnnotator: process() ends" << endl;
    return (TyErrorId)UIMA_ERR_NONE;
  }

};

// This macro exports an entry point that is used to create the annotator.

MAKE_AE(SofaDataAnnotator);
