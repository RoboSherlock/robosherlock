/** \file test_xcasdeserialization.cpp .

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
#include "uima/api.hpp"
#include "uima/xmlwriter.hpp"
#include "uima/xcasdeserializer.hpp"
#include "uima/casdefinition.hpp"
#include "uima/xmiwriter.hpp"

#include <fstream>
using namespace std;
#ifndef NDEBUG
#define ASSERT_OR_THROWEXCEPTION(x) assert(x)
#else
#define ASSERT_OR_THROWEXCEPTION(x) if (!(x)) { cerr << __FILE__ << ": Error in line " << __LINE__ << endl; exit(1); }
#endif

#define LOG(x) cout << __FILE__ << __LINE__ << ": " << x << endl

using namespace uima;


/* ----------------------------------------------------------------------- */
/*       Main routine                                                      */
/* ----------------------------------------------------------------------- */

int main(int argc, char * argv[]) /*
---------------------------------- */
{
  LOG("UIMATEST_XCASSERIALIZATION started");
  int iRetVal = 0;
  //iRetVal = _CrtSetBreakAlloc(824);
  try {

    ResourceManager::createInstance("test");
    ofstream outputStream;

    TextAnalysisEngineSpecifierBuilder builder;
    TextAnalysisEngineSpecifier apTAESpecifier;
    UnicodeString dataFile("ExampleCas/testTae.xml");
    UnicodeString datafn = ResourceManager::resolveFilename(dataFile, dataFile);
    builder.buildTaeFromFile(apTAESpecifier, datafn);

    internal::CASDefinition * casDef = internal::CASDefinition::createCASDefinition(*apTAESpecifier.getAnalysisEngineMetaData());
    ErrorInfo errorInfo;
    CAS* cas =  Framework::createCAS(*casDef, errorInfo);
    ASSERT_OR_THROWEXCEPTION( EXISTS(cas) );
    CAS* v1cas =  Framework::createCAS(*casDef, errorInfo);
    ASSERT_OR_THROWEXCEPTION( EXISTS(v1cas) );

    // get a v2 CAS
    UnicodeString v2casFile("ExampleCas/cas.xml");
    UnicodeString v2casfn = ResourceManager::resolveFilename(v2casFile, v2casFile);
    XCASDeserializer::deserialize(v2casfn, *cas);

    // get a v1.x version of the same CAS
    UnicodeString v1casFile("ExampleCas/v1cas.xml");
    UnicodeString v1casfn = ResourceManager::resolveFilename(v1casFile, v1casFile);
    XCASDeserializer::deserialize(v1casfn, *v1cas);

    // compare
    ASSERT_OR_THROWEXCEPTION(cas->getAnnotationIndex().getSize() == v1cas->getAnnotationIndex().getSize());

    // Serialize XCAS
    outputStream.open("temp.xcas");
    if ( !outputStream ) {
      cerr << "Error opening output stream" << endl;
      return 1;
    }
    XCASWriter writerx(*v1cas, false);
    writerx.write(outputStream);
    outputStream.close();

    // Deserialize XCAS ...
    v1cas->reset();
    XCASDeserializer::deserialize("temp.xcas", *v1cas);

    // ... and compare
    ASSERT_OR_THROWEXCEPTION(cas->getAnnotationIndex().getSize() == v1cas->getAnnotationIndex().getSize());



    //testNoInitialSofa
    cas->reset();
    // create non-annotation type so as not to create the _InitialView Sofa
    IntArrayFS intArrayFS = cas->createIntArrayFS(5);
    intArrayFS.set(0,1);
    intArrayFS.set(1,2);
    intArrayFS.set(2,3);
    intArrayFS.set(3,4);
    intArrayFS.set(4,5);
    cas->getIndexRepository().addFS(intArrayFS);

    // Serialize XCAS
    outputStream.open("temp.xcas");
    if ( !outputStream ) {
      cerr << "Error opening output stream" << endl;
      return 1;
    }
    XCASWriter writer(*cas, false);
    writer.write(outputStream);
    outputStream.close();

    // Deserialize XCAS
    cas->reset();
    XCASDeserializer::deserialize("temp.xcas", *cas);
    
    //TODO compare!



//     // now a v1.x version of a multiple Sofa CAS
    v1cas->reset();
    UnicodeString v1McasFile("ExampleCas/v1MultiSofaCas.xml");
    UnicodeString v1Mcasfn = ResourceManager::resolveFilename(v1McasFile, v1McasFile);
    XCASDeserializer::deserialize(v1Mcasfn, *v1cas);

    // test it
    CAS* engView = v1cas->getView("EnglishDocument");
    ASSERT_OR_THROWEXCEPTION(0==engView->getDocumentText().compare("this beer is good"));
    ASSERT_OR_THROWEXCEPTION(engView->getAnnotationIndex().getSize() == 5); // 4 annots plus documentAnnotation
    CAS* gerView = v1cas->getView("GermanDocument");
    ASSERT_OR_THROWEXCEPTION(0==gerView->getDocumentText().compare("das bier ist gut"));
    ASSERT_OR_THROWEXCEPTION(gerView->getAnnotationIndex().getSize() == 5); // 4 annots plus documentAnnotation

//     // reserialize
//     StringWriter sw = new StringWriter();
//     XMLSerializer xmlSer = new XMLSerializer(sw, false);
//     XCASSerializer xcasSer = new XCASSerializer(v1cas.getTypeSystem());
//     xcasSer.serialize(v1cas, xmlSer.getContentHandler(), true);
//     String xml = sw.getBuffer().toString();

//     // deserialize into another CAS
//     cas.reset();
//     XCASDeserializer deser2 = new XCASDeserializer(cas.getTypeSystem());
//     ContentHandler deserHandler2 = deser2.getXCASHandler(cas);
//     xmlReader.setContentHandler(deserHandler2);
//     xmlReader.parse(new InputSource(new StringReader(xml)));

//     // test it
//     assertTrue(v1cas.getDocumentText().equals("some text for the default text sofa."));
//     engView = cas.getView("EnglishDocument");
//     assertTrue(engView.getDocumentText().equals("this beer is good"));
//     assertTrue(engView.getAnnotationIndex().size() == 5); // 4 annots plus documentAnnotation
//     gerView = cas.getView("GermanDocument");
//     assertTrue(gerView.getDocumentText().equals("das bier ist gut"));
//     assertTrue(gerView.getAnnotationIndex().size() == 5); // 4 annots plus documentAnnotation



    delete  casDef;
    delete cas;
    delete v1cas;
    LOG("UIMATEST_XCASSERIALIZATION finished");

  } catch (Exception & exc) {
    cerr << exc.asString() << endl;
    iRetVal = 1;
  } catch (...) {
    cerr << "Unexpected exception " << endl;
    iRetVal = 1;
  }
#ifndef NDEBUG
  ResourceManager::deleteInstance();
#endif 
  return iRetVal;
}

/* <EOF> */
