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

const UChar * translate(UChar *);

class SofaExampleAnnotator : public Annotator {
private:
  Type cross, annot;
  Feature other;
  icu::UnicodeString us_SofaString;
  AnnotatorContext * pAnc;

public:

  SofaExampleAnnotator(void) {
    cout << "SofaExampleAnnotator: Constructor" << endl;
  }

  ~SofaExampleAnnotator(void) {
    cout << "SofaExampleAnnotator: Destructor" << endl;
  }

  /** */
  TyErrorId initialize(AnnotatorContext & rclAnnotatorContext) {
    cout << "SofaExampleAnnotator: initialize()" << endl;

    // Save the annotator context for use in process()
    pAnc = &rclAnnotatorContext;

    return (TyErrorId)UIMA_ERR_NONE;
  }


  /** */
  TyErrorId typeSystemInit(TypeSystem const & crTypeSystem) {
    cout << "SofaExampleAnnotator::typeSystemInit()" << endl;

    // get Type and Feature objects for use in process()
    annot  = crTypeSystem.getType("uima.tcas.Annotation");
    cross  = crTypeSystem.getType("sofa.test.CrossAnnotation");
    other = cross.getFeatureByBaseName("otherAnnotation");
    if (!(annot.isValid() && cross.isValid() && other.isValid())) {
      cout << "SofaExampleAnnotator::typeSystemInit() - Error getting Type or Feature objects" << endl;
      return (TyErrorId)UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }
    return(TyErrorId)UIMA_ERR_NONE;
  }


  /** */
  TyErrorId destroy() {
    cout << "SofaExampleAnnotator: destroy()" << endl;
    return (TyErrorId)UIMA_ERR_NONE;
  }

  /** */
  TyErrorId process(CAS & rCAS, ResultSpecification const & crResultSpecification) {
    CAS *engTcas, *germTcas;
    UChar *myLocalSaveState;

    // Look for english document and "translate" to German
    cout << "SofaExampleAnnotator: process() begins" << endl;

    // get English view
    engTcas = rCAS.getView("EnglishDocument");
    DocumentFS adocFS = engTcas->getDocumentAnnotation();
    UnicodeStringRef aengText = adocFS.getCoveredText();
    cout << "      English Input: " << aengText << endl;

    // Create the output German text Sofa and open CAS view
    germTcas = rCAS.createView("GermanDocument");

    // Get pointer to the English text document
    DocumentFS docFS = engTcas->getDocumentAnnotation();
    UnicodeStringRef engText = docFS.getCoveredText();

    // make copy of document for the u_strtok_r function (100 character limit!)
    UChar uWork[100];
    u_strncpy(uWork, engText.getBuffer(), 99);

    // Setup for translated text
    int germBegin = 0;
    int germEnd = 0;
    UChar translation[400];
    translation[0]=0;

    // get two IR handles for adding annotations to the appropriate view
    FSIndexRepository & engIndexRep = engTcas->getIndexRepository();
    FSIndexRepository & germIndexRep = germTcas->getIndexRepository();

    // Parse the English text
    UChar uDelim[2];
    UnicodeString delimUS(" ");
    u_strncpy(uDelim, delimUS.getBuffer(), 1);
	uDelim[1] = 0;
    UChar * next = u_strtok_r(uWork, uDelim, &myLocalSaveState);

    while (next) {
      // Create annotation on source text
      AnnotationFS engAnnot =
        engTcas->createAnnotation(annot, next-uWork, (next-uWork)+u_strlen(next));
      engIndexRep.addFS(engAnnot);

      // Translate word-by-word
      const UChar * gword = translate(next);

      // Accumulate the total translated document
      if (germBegin > 0) {
        // if not the first word, add space before
        u_strncat(translation, uDelim, 1);
        germBegin += 1;
      }
      u_strcat(translation, gword);

      // Create annotation on output text
      germEnd = germBegin + u_strlen(gword);
      AnnotationFS germAnnot = germTcas->createAnnotation(cross, germBegin, germEnd);
      germIndexRep.addFS(germAnnot);
      // add link to English text
      germAnnot.setFSValue(other, engAnnot);
      germBegin = germEnd;

      next = u_strtok_r(NULL, uDelim, &myLocalSaveState);
    }
    // set documentText with accumulated transation
    germTcas->setDocumentText( translation, u_strlen(translation), true );

    cout << "   German(!) Output: " << germTcas->getDocumentText() << endl;

    cout << "SofaExampleAnnotator: process() ends" << endl;
    return (TyErrorId)UIMA_ERR_NONE;
  }
};


/** */
static UnicodeString const uThis("this");
static UnicodeString const uBeer("beer");
static UnicodeString const uIs("is");
static UnicodeString const uDas("das");
static UnicodeString const uBier("bier");
static UnicodeString const uIst("ist");
static UnicodeString const uGut("gut");
const UChar * translate(UChar * eword) {
  if (0 == u_strcasecmp(eword, uThis.getBuffer(), 0))
    return uDas.getBuffer();
  if (0 == u_strcasecmp(eword, uBeer.getBuffer(), 0))
    return uBier.getBuffer();
  if (0 == u_strcasecmp(eword, uIs.getBuffer(), 0))
    return uIst.getBuffer();
  return uGut.getBuffer();
}

// This macro exports an entry point that is used to create the annotator.

MAKE_AE(SofaExampleAnnotator);
