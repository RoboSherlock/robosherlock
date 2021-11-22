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

/**------------------------------------------------------------------------
 * A simple Multiple SOFA test application
 * Creates a text SOFA with English text,
 * calls an annotator that creates a text SOFA with German text,
 * then dumps all annotation found in both SOFAs
 *
 * The application takes no arguments.
-------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <stdio.h>
#include "uima/api.hpp"
#include "uima/envvars.h"
#include "unicode/ucnv.h"
using namespace std;
using namespace uima;

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

/* Little helper routine to check and report errors.
   This routine just does a hard program exit for any failure! */
static void CheckError(TyErrorId utErrorId,
                       const AnalysisEngine &  crEngine);
static void CheckError(ErrorInfo const &);

int main(int argc, char * argv[]) {
  int runQuiet = 0;
  int runTimes = 1;
  /* Access the command line arguments. */

  /* filename of descriptor required */
  /* this must be SofaExampleAnnotator.xml found in */
  /* $(UIMACPP_HOME)/docs/examples/descriptors       */
  if (argc < 2) {
    cout << "Usage: SofaExampleApplication pathToSofaExampleAnnotator.xml [Quiet [repeatCount]]" << endl;
    exit (1);
  }
  if (argc > 2) {
    if ('q' == *argv[2] || 'Q' == *argv[2])
      runQuiet = 1;
  }
  if (argc > 3) {
    runTimes = atoi(argv[3]);
    cout << "Running " << runTimes << " times." << endl;
  }

  /* Create/link up to a resource manager instance (singleton) */
  (void) ResourceManager::createInstance("SOFA_EXAMPLE_APPLICATION");

  TyErrorId utErrorId;          // Variable to store return codes
  ErrorInfo errorInfo;          // Variable to stored detailed error info

  try {
    /* locate SofaExampleAnnotator.xml descriptor in $(UIMACPP_HOME)/docs/examples/descriptors */
    string descriptor = "SofaExampleAnnotator.xml";
    util::EnvironmentVariableQueryOnly uimarootEnvVar(UIMA_ENVVAR_HOME);
    if (uimarootEnvVar.hasValue()) {
      descriptor = uimarootEnvVar.getValue();
      descriptor += "/docs/examples/descriptors/SofaExampleAnnotator";
    }
    cout << argv[1] << endl;
    /* Initialize engine with filename of config-file */
    AnalysisEngine * pEngine =
      Framework::createAnalysisEngine(argv[1], errorInfo);
    CheckError(errorInfo);

    // Construct a Unicode input string out of our single-byte string.
    UnicodeString ustrInputText("This is a text document for analysis");

    if (!runQuiet)
      cout << "SofaExampleApplication.cpp: document length= " << ustrInputText.length() << endl;

    // Initialize the CAS
    CAS* aCas = pEngine->newCAS();
    for (int jj=0; jj<runTimes; jj++) {
      //create a text view with appropriate name for English->German translator
      CAS * eTcas = aCas->createView("EnglishDocument");

      //and set the Sofa data with the English document
      eTcas->setSofaDataString(ustrInputText, "text");

// Get pointer to the English text document
      DocumentFS docFS = eTcas->getDocumentAnnotation();
      UnicodeStringRef eText = docFS.getCoveredText();
      //cout << "eng tcas covered text " << eText << endl;

      // Call the translation annotator
      utErrorId = ((AnalysisEngine*)pEngine)->process(*aCas);
      CheckError(utErrorId, *pEngine);


      if (!runQuiet)
        cout << endl << "Retrieving all annotation for the English document:" << endl;
      CAS* engTcas = aCas->getView("EnglishDocument");

      ANIndex anIdx = engTcas->getAnnotationIndex(
                        engTcas->getTypeSystem().getType("uima.tcas.Annotation"));
      ANIterator itAn = anIdx.iterator();
      itAn.moveToFirst();
      while (itAn.isValid()) {
        // display the name of the Annotation Type and the covered text
        if (!runQuiet) {
          cout << itAn.get().getType().getName() << ": "
          << itAn.get().getBeginPosition() << ", " << itAn.get().getEndPosition()
          << " \"" << itAn.get().getCoveredText() << "\"" << endl;
        }
        itAn.moveToNext();
      }

      if (!runQuiet)
        cout << endl << "Retrieving all annotation for the German document:" << endl;
      CAS* germTcas = aCas->getView("GermanDocument");

      Type crossType  = engTcas->getTypeSystem().getType("sofa.test.CrossAnnotation");
      Feature otherFeat = crossType.getFeatureByBaseName("otherAnnotation");
      anIdx = germTcas->getAnnotationIndex(engTcas->getTypeSystem().getType("uima.tcas.Annotation"));
      itAn = anIdx.iterator();
      itAn.moveToFirst();
      while (itAn.isValid()) {
        // display the name of the Annotation Type and the covered text
        if (!runQuiet) {
          cout << itAn.get().getType().getName() << ": "
          << itAn.get().getBeginPosition() << ", " << itAn.get().getEndPosition()
          << " \"" << itAn.get().getCoveredText() << "\"";
        }
        // if annotation type = CrossAnnotation...
        if (itAn.get().getType() == crossType) {
          AnnotationFS otherAn = (AnnotationFS)itAn.get().getFSValue(otherFeat);
          if (!runQuiet) {
            cout << "  -->  otherAnnotation: "
            << otherAn.getBeginPosition() << ", " << otherAn.getEndPosition()
            << " \"" << otherAn.getCoveredText() << "\"" << endl;
          }
        } else
          cout << endl;
        itAn.moveToNext();
      }
      if (!runQuiet)
        cout << endl;

      /* Tell UIMA we are finished with this document */
      utErrorId = aCas->reset();
      CheckError(utErrorId, *pEngine);
    }
    /* Free ressorces in UIMA */
    utErrorId = pEngine->destroy();
    CheckError(utErrorId, *pEngine);
    delete aCas;
    delete pEngine;

    /* If we got this far everything went OK */
    if (!runQuiet)
      cout << "UIMA processing finished sucessfully! " << endl;
  } catch (Exception e) {
    cout << "error " << endl;
  }
  return (0);
}



/* Little helper routine to check and report errors.
   This routine just does a hard program exit for any failure!
*/
static void CheckError(TyErrorId utErrorId,
                       const AnalysisEngine &  crEngine) {

  if (utErrorId != UIMA_ERR_NONE) {
    cerr << endl << "   *** SofaExampleApplication - UIMA Error info:" << endl;
    cerr << "Error string        : "
    << AnalysisEngine::getErrorIdAsCString(utErrorId) << endl;
    cerr << "Error number        : "
    << utErrorId << endl;
    const TCHAR* errStr = crEngine.getAnnotatorContext().getLogger().getLastErrorAsCStr();
    if (errStr != NULL)
      cerr << "  Last logged message : " <<
      errStr << endl;
    exit((int)utErrorId);
  }
}

/* Similar routine as above just with error info objects instead of err-ids.
   This routine just does a hard program exit for any failure!
*/
static void CheckError(ErrorInfo const & errInfo) {
  if (errInfo.getErrorId() != UIMA_ERR_NONE) {
    cerr << endl << "   *** SofaExampleApplication - UIMA Error info:" << endl
    << "Error string  : "
    << AnalysisEngine::getErrorIdAsCString(errInfo.getErrorId())
    << errInfo << endl;                      /* (errInfo starts with a newline) */
    exit((int)errInfo.getErrorId());
  }

}



