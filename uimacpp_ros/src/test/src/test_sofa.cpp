/** \file sofatest.cpp .

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
#include "uima/internal_casserializer.hpp"
#include "uima/internal_casdeserializer.hpp"

#include <fstream>

#ifndef NDEBUG
#define ASSERT_OR_THROWEXCEPTION(x) assert(x)
#else
#define ASSERT_OR_THROWEXCEPTION(x) if (!(x)) { cerr << __FILE__ << ": Error in line " << __LINE__ << endl; exit(1); }
#endif

#define LOG(x) cout << __FILE__ << __LINE__ << ": " << x << endl

using namespace uima;
using namespace std;

/* ----------------------------------------------------------------------- */
/*       Main routine                                                      */
/* ----------------------------------------------------------------------- */

int main(int argc, char * argv[]) /*
---------------------------------- */
{
  LOG("UIMATEST_SOFATEST started");
  int iRetVal = 0;
  //iRetVal = _CrtSetBreakAlloc(824);
  try {
    char const * config =
      "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
      "<typeSystemDescription xmlns=\"http://uima.apache.org/resourceSpecifier\">"
      " <types>"
      "   <typeDescription>"
      "        <name>sofa.test.CrossAnnotation</name>"
      "        <description></description>"
      "        <supertypeName>uima.tcas.Annotation</supertypeName>"
      "        <features>"
      "   <featureDescription>"
      "     <name>otherAnnotation</name>"
      "     <description></description>"
      "     <rangeTypeName>uima.tcas.Annotation</rangeTypeName>"
      "   </featureDescription>"
      "        </features>"
      "   </typeDescription>"
      " </types>"
      "</typeSystemDescription>";

    ResourceManager::createInstance("test");

    ErrorInfo errorInfo;
    ofstream outputStream;

    static TypeSystem *ts = Framework::createTypeSystemFromXMLBuffer(config, errorInfo );
    CAS* cas =  Framework::createCAS(*ts, errorInfo);
    ASSERT_OR_THROWEXCEPTION( EXISTS(cas) );

    Type annotationType = ts->getType(CAS::TYPE_NAME_ANNOTATION);
    Type docAnnotationType = ts->getType(CAS::TYPE_NAME_DOCUMENT_ANNOTATION);
    Type crossType = ts->getType("sofa.test.CrossAnnotation");
    Feature otherFeat = crossType.getFeatureByBaseName("otherAnnotation");

    // Create a Sofa (using old APIs for now)
    SofaID * id = new SofaID;
    id->setSofaId("EnglishDocument");
    SofaFS es = cas->createSofa(*id, "text");
    ASSERT_OR_THROWEXCEPTION(es.isValid());
    // Initial View is #1!!!
    ASSERT_OR_THROWEXCEPTION(2 == es.getSofaRef());
    delete id;
    // Set the document text
//TODO?   es.setLocalSofaData("this beer is good");
    es.setLocalSofaData(icu::UnicodeString("this beer is good"));

    // Test Multiple Sofas across XCAS serialization
    outputStream.open("temp.xcas");
    if ( !outputStream ) {
      cerr << "Error opening output stream" << endl;
      return 1;
    }
    // Serialize XCAS
    XCASWriter writer(*cas, false);
    writer.write(outputStream);
    outputStream.close();

    // Deserialize XCAS
    cas->reset();
    XCASDeserializer::deserialize("temp.xcas", *cas);

    // Add a new Sofa
    CAS* gerTcas = cas->createView("GermanDocument");
    ASSERT_OR_THROWEXCEPTION(0 == gerTcas->getViewName().compare("GermanDocument"));
    SofaFS gs = gerTcas->getSofa();

    ASSERT_OR_THROWEXCEPTION(gs.isValid());
    ASSERT_OR_THROWEXCEPTION(3 == gs.getSofaRef());

    // Set the document text
//TODO?   gerTcas->setDocumentText("das bier ist gut");
    gerTcas->setDocumentText(icu::UnicodeString("das bier ist gut"));

    // test getView()->getDocumentAnnotation()->getCoveredText()
    AnnotationFS gerDocAnn = cas->getView("GermanDocument")->getDocumentAnnotation();
    ASSERT_OR_THROWEXCEPTION(0==gerDocAnn.getCoveredText().compare("das bier ist gut"));   


// Test Multiple Sofas across XCAS serialization
    outputStream.open("temp.xcas");
    if ( !outputStream ) {
      cerr << "Error opening output stream" << endl;
      return 1;
    }
    // Serialize XCAS
//   XCASWriter writer2(*cas, false);
    writer.write(outputStream);
    outputStream.close();

    // Deserialize XCAS
    cas->reset();
    XCASDeserializer::deserialize("temp.xcas", *cas);


    // Test multiple Sofas across binary serialization
    internal::CASSerializer serializer(false);
    internal::CASDeserializer deserializer;
    size_t blobsz = serializer.getBlobSize(*cas);
    char* blob = new char[blobsz];
    size_t blobsz2 = serializer.getBlob(*cas, blob, blobsz);
    ASSERT_OR_THROWEXCEPTION(blobsz == blobsz2);
    cas->reset();
    deserializer.deserializeBlob(blob, *cas);
    delete[] blob;

    // Add a new Sofa
    CAS* frT = cas->createView("FrenchDocument");
    ASSERT_OR_THROWEXCEPTION(0 == frT->getViewName().compare("FrenchDocument"));
    SofaFS fs = frT->getSofa();
    ASSERT_OR_THROWEXCEPTION(fs.isValid());
    ASSERT_OR_THROWEXCEPTION(4 == fs.getSofaRef());

    // Test multiple Sofas across blob serialization
    blobsz = serializer.getBlobSize(*cas);
    blob = new char[blobsz];
    blobsz2 = serializer.getBlob(*cas, blob, blobsz);
    ASSERT_OR_THROWEXCEPTION(blobsz == blobsz2);
    cas->reset();
    deserializer.deserializeBlob(blob, *cas);
    delete[] blob;

    // Open Cas views of some Sofas
    CAS* engTcas = cas->getView(es);
    ASSERT_OR_THROWEXCEPTION(0 == engTcas->getViewName().compare("EnglishDocument"));
    CAS* frTcas = cas->getView("FrenchDocument");

    // Set the document text off SofaFS after the CAS view exists
//   frTcas->setSofaDataString("cette biere est bonne", "text");
    frTcas->setSofaDataString(icu::UnicodeString("cette biere est bonne"), "text");

    // Create standard annotations against one and cross annotations against the other
    AnnotationFS engAnnot = engTcas->createAnnotation(annotationType, 0, 4);
    engTcas->getIndexRepository().addFS(engAnnot);

    AnnotationFS frAnnot = frTcas->createAnnotation(annotationType, 0, 5);
    frTcas->getIndexRepository().addFS(frAnnot);

    AnnotationFS gerAnnot = gerTcas->createAnnotation(crossType, 0, 3);
    gerAnnot.setFeatureValue(otherFeat, engAnnot);
    gerTcas->getIndexRepository().addFS(gerAnnot);

    // test getView()->getDocumentAnnotation()->getCoveredText()
    gerDocAnn = cas->getView("GermanDocument")->getDocumentAnnotation();
    ASSERT_OR_THROWEXCEPTION(0==gerDocAnn.getCoveredText().compare("das bier ist gut"));   

    // Test that the annotations are in separate index spaces, and that Sofas are indexed
    FSIterator sofaIter = cas->getSofaIterator();
    int numSofas = 0;
    while (sofaIter.isValid()) {
      numSofas++;
      sofaIter.moveToNext();
    }

    FSIndex engIndex = engTcas->getAnnotationIndex();
    FSIndex gerIndex = gerTcas->getAnnotationIndex();
    FSIndex frIndex = frTcas->getAnnotationIndex();
    ASSERT_OR_THROWEXCEPTION(numSofas == 3);

    ASSERT_OR_THROWEXCEPTION(engIndex.getSize() == 2);  // 1 annots plus documentAnnotation
    ASSERT_OR_THROWEXCEPTION(gerIndex.getSize() == 2);  // 1 annots plus documentAnnotation
    ASSERT_OR_THROWEXCEPTION(frIndex.getSize() == 2);  // 1 annots plus documentAnnotation

    // Test that the annotations are of the correct types
    FSIterator engIt = engIndex.iterator();
    FSIterator gerIt = gerIndex.iterator();
    FSIterator frIt = frIndex.iterator();
    engAnnot = (AnnotationFS)engIt.get();
    gerAnnot = (AnnotationFS)gerIt.get();
    frAnnot = (AnnotationFS)frIt.get();
    ASSERT_OR_THROWEXCEPTION(0==docAnnotationType.getName().compare(engAnnot.getType().getName()));
    ASSERT_OR_THROWEXCEPTION(0==docAnnotationType.getName().compare(gerAnnot.getType().getName()));
    ASSERT_OR_THROWEXCEPTION(0==docAnnotationType.getName().compare(frAnnot.getType().getName()));

    engIt.moveToNext();
    gerIt.moveToNext();
    frIt.moveToNext();
    engAnnot = (AnnotationFS)engIt.get();
    gerAnnot = (AnnotationFS)gerIt.get();
    frAnnot = (AnnotationFS)frIt.get();
    ASSERT_OR_THROWEXCEPTION(0==annotationType.getName().compare(engAnnot.getType().getName()));
    ASSERT_OR_THROWEXCEPTION(0==engAnnot.getCoveredText().compare("this"));
    ASSERT_OR_THROWEXCEPTION(0==annotationType.getName().compare(frAnnot.getType().getName()));
    ASSERT_OR_THROWEXCEPTION(0==frAnnot.getCoveredText().compare("cette"));
    ASSERT_OR_THROWEXCEPTION(0==crossType.getName().compare(gerAnnot.getType().getName()));
    ASSERT_OR_THROWEXCEPTION(0==gerAnnot.getCoveredText().compare("das"));

    // Test that the other annotation feature of cross annotations works
    AnnotationFS crossAnnot = (AnnotationFS) gerAnnot.getFeatureValue(otherFeat);
    ASSERT_OR_THROWEXCEPTION(0==annotationType.getName().compare(crossAnnot.getType().getName()));
    ASSERT_OR_THROWEXCEPTION(0==crossAnnot.getCoveredText().compare("this"));



    // --------------------------------------------------------
    // Test that annotations accessed from a reference in the base CAS work correctly
    // --------------------------------------------------------

    ArrayFS anArray = cas->createArrayFS(3);
    anArray.set(0, engAnnot);
    anArray.set(1, frAnnot);
    anArray.set(2, gerAnnot);
    AnnotationFS tstAnnot = (AnnotationFS)anArray.get(0);
    ASSERT_OR_THROWEXCEPTION(0==tstAnnot.getCoveredText().compare("this"));
    tstAnnot = (AnnotationFS)anArray.get(1);
    ASSERT_OR_THROWEXCEPTION(0==tstAnnot.getCoveredText().compare("cette"));
    tstAnnot = (AnnotationFS)anArray.get(2);
    ASSERT_OR_THROWEXCEPTION(0==tstAnnot.getCoveredText().compare("das"));


    // --------------------------------------------------------
    // Test that a FS can be indexed in multiple Views
    // --------------------------------------------------------

    CAS* marView = cas->createView("MartinView");
    marView->getIndexRepository().addFS(engAnnot);

    // Serialize XCAS
    outputStream.open("temp.xcas");
    if ( !outputStream ) {
      cerr << "Error opening output stream" << endl;
      return 1;
    }
    writer.write(outputStream);
    outputStream.close();
    // Deserialize XCAS
    cas->reset();
    XCASDeserializer::deserialize("temp.xcas", *cas);

    marView = cas->getView("MartinView");
    FSIndex mrIndex = marView->getAnnotationIndex(annotationType);
    FSIterator mrIt = mrIndex.iterator();
    ASSERT_OR_THROWEXCEPTION(mrIt.isValid());
    engAnnot = (AnnotationFS)mrIt.get();
    ASSERT_OR_THROWEXCEPTION(0==engAnnot.getCoveredText().compare("this"));

    // --------------------------------------------------------
    // Test reuse of views with createSofa
    // --------------------------------------------------------

	cas->reset();
	cas->setDocumentText(icu::UnicodeString("setDocumentText creates the _InitialView Sofa"));
	CAS* testView = cas->createView("anotherView");
    ASSERT_OR_THROWEXCEPTION(0 == testView->getViewName().compare("anotherView"));
	ASSERT_OR_THROWEXCEPTION(0 == cas->getViewName().compare("_InitialView"));

	cas->reset();
    SofaID * nuid = new SofaID;
    nuid->setSofaId("testView");
    SofaFS tv = cas->createSofa(*nuid, "text");
	testView = cas->getView(tv);
    ASSERT_OR_THROWEXCEPTION(0 == testView->getViewName().compare("testView"));
    delete nuid;

    // --------------------------------------------------------
    // Test sofa data stream
    // --------------------------------------------------------

    char dest[100];
    //Test reading sofa data set as String feature.
	cas->reset();
    CAS * stringView = cas->createView("StringSofaData");
    icu::UnicodeString ustrText("this beer is good");
    stringView->setDocumentText(ustrText);
    SofaDataStream * pStream = stringView->getSofaDataStream();
    ASSERT_OR_THROWEXCEPTION(pStream != NULL);
    ASSERT_OR_THROWEXCEPTION( pStream->open()== 0);
    int totsize = pStream->getTotalStreamSizeInBytes();
    ASSERT_OR_THROWEXCEPTION(totsize > 0);
    ASSERT_OR_THROWEXCEPTION(pStream->read(dest,1,totsize) == totsize);
    ASSERT_OR_THROWEXCEPTION(strncmp(dest, "this beer is good", totsize) == 0);
    delete pStream;

    //read sofa data from SofaURI.
    CAS * remoteView = cas->createView("remoteSofaData");
    ustrText = "text";
    std::string fn = "file://";
    fn.append(UnicodeStringRef(ResourceManager::resolveFilename("example.txt", ".")).asUTF8());
    remoteView->setSofaDataURI(fn.c_str(), "text");
    pStream = remoteView->getSofaDataStream();
    ASSERT_OR_THROWEXCEPTION(pStream != NULL);
    ASSERT_OR_THROWEXCEPTION( pStream->open()== 0);
    totsize = pStream->getTotalStreamSizeInBytes();
    ASSERT_OR_THROWEXCEPTION(totsize > 0);
    ASSERT_OR_THROWEXCEPTION(pStream->read(dest,1,totsize) == totsize);
    ASSERT_OR_THROWEXCEPTION(strncmp(dest, "This is a text document with a Dave for analysis.", totsize) == 0);
    delete pStream;


    //read sofa data set as int array FS.
    IntArrayFS intArrayFS = cas->createIntArrayFS(5);
    intArrayFS.set(0,1);
    intArrayFS.set(1,2);
    intArrayFS.set(2,3);
    intArrayFS.set(3,4);
    intArrayFS.set(4,5);
    CAS * intArrayView = cas->createView("intArraySofaData");
    intArrayView->setSofaDataArray(intArrayFS, "integers");
    pStream = intArrayView->getSofaDataStream();
    ASSERT_OR_THROWEXCEPTION(pStream != NULL);
    ASSERT_OR_THROWEXCEPTION( pStream->open()== 0);
    int value;
    int i=0;
    while (pStream->read(&value, sizeof(int), 1) != -1) {
      ASSERT_OR_THROWEXCEPTION(value == intArrayFS.get(i++));
    }
    delete pStream;

    //read sofa data set as float array FS.
    FloatArrayFS  floatArrayFS = cas->createFloatArrayFS(5);
    floatArrayFS.set(0,(float) 0.1);
    floatArrayFS.set(1,(float) 0.2);
    floatArrayFS.set(2,(float) 0.3);
    floatArrayFS.set(3,(float) 0.4);
    floatArrayFS.set(4,(float) 0.5);
    CAS * floatArrayView = cas->createView("floatArraySofaData");
    floatArrayView->setSofaDataArray(floatArrayFS,"floats");
    pStream = floatArrayView->getSofaDataStream();
    ASSERT_OR_THROWEXCEPTION(pStream != NULL);
    ASSERT_OR_THROWEXCEPTION( pStream->open()== 0);
    float floatvalue;
    i=0;
    while (pStream->read(&floatvalue, sizeof(float), 1) != -1) {
      ASSERT_OR_THROWEXCEPTION(floatvalue == floatArrayFS.get(i++));
    }
    delete pStream;

    //create a Sofa and set the SofaArray feature to a short array FS.
    ShortArrayFS shortArrayFS = cas->createShortArrayFS(5);
    shortArrayFS.set(0,(short)128);
    shortArrayFS.set(1,(short)127);
    shortArrayFS.set(2,(short)126);
    shortArrayFS.set(3,(short)125);
    shortArrayFS.set(4,(short)124);
    CAS * shortArrayView = cas->createView("shortArraySofaData");
    shortArrayView->setSofaDataArray(shortArrayFS, "shorts");
    pStream = shortArrayView->getSofaDataStream();
    ASSERT_OR_THROWEXCEPTION(pStream != NULL);
    ASSERT_OR_THROWEXCEPTION( pStream->open()== 0);
    short shortvalue;
    i=0;
    while (pStream->read(&shortvalue, sizeof(short), 1) != -1) {
      //cout << shortvalue << " " << shortArrayFS.get(i << endl;
      ASSERT_OR_THROWEXCEPTION(shortvalue == shortArrayFS.get(i++));
    }
    delete pStream;

    //create a Sofa and set the SofaArray feature to a byte array FS.
    ByteArrayFS byteArrayFS = cas->createByteArrayFS(4);
    byteArrayFS.set(0, 'a' );
    byteArrayFS.set(1, 'b');
    byteArrayFS.set(2, 'c');
    byteArrayFS.set(3, 'd');
    CAS * byteArrayView = cas->createView("byteArraySofaData");
    byteArrayView->setSofaDataArray(byteArrayFS, "bytes");
    pStream = byteArrayView->getSofaDataStream();
    ASSERT_OR_THROWEXCEPTION(pStream != NULL);
    ASSERT_OR_THROWEXCEPTION( pStream->open()== 0);
    char bytevalue;
    i=0;
    while (pStream->read(&bytevalue, 1, 1) != -1) {
      ASSERT_OR_THROWEXCEPTION(bytevalue == byteArrayFS.get(i++));
    }
    delete pStream;

    //create a Sofa and set the SofaArray feature to a long array FS.
    LongArrayFS longArrayFS = cas->createLongArrayFS(5);
    longArrayFS.set(0,11);
    longArrayFS.set(1,21);
    longArrayFS.set(2,31);
    longArrayFS.set(3,41);
    longArrayFS.set(4,51);
    CAS * longArrayView = cas->createView("longArraySofaData");
    longArrayView->setSofaDataArray(longArrayFS, "longs");
    pStream = longArrayView->getSofaDataStream();
    ASSERT_OR_THROWEXCEPTION(pStream != NULL);
    ASSERT_OR_THROWEXCEPTION( pStream->open()== 0);
    INT64 longvalue;
    i=0;
    while (pStream->read(&longvalue, sizeof(INT64), 1) != -1) {
      ASSERT_OR_THROWEXCEPTION(longvalue == longArrayFS.get(i++));
    }
    delete pStream;

    //create a Sofa and set the SofaArray feature to a Double Array FS.
    DoubleArrayFS doubleArrayFS = cas->createDoubleArrayFS(5);
    doubleArrayFS.set(0,21E10);
    doubleArrayFS.set(1,31E10);
    doubleArrayFS.set(2,41E10);
    doubleArrayFS.set(3,51E10);
    doubleArrayFS.set(4,61E10);
    CAS * doubleArrayView = cas->createView("doubleArraySofaData");
    doubleArrayView->setSofaDataArray(doubleArrayFS, "doubles");
    pStream = doubleArrayView->getSofaDataStream();
    ASSERT_OR_THROWEXCEPTION(pStream != NULL);
    ASSERT_OR_THROWEXCEPTION( pStream->open()== 0);
    i=0;
    double doubleValue;
    while (pStream->read(&doubleValue, sizeof(double), 1) != -1) {
      ASSERT_OR_THROWEXCEPTION( doubleArrayFS.get(i++) == doubleValue);
    }
    delete pStream;

    delete ts;
    delete cas;
    
    LOG("UIMATEST_SOFATEST finished");
  } catch (Exception & exc) {
    cerr << exc.asString() << endl;
    iRetVal = 1;
  }
#ifdef NDEBUG
  catch (...) {
    cerr << "Unexpected exception " << endl;
    iRetVal = 1;
  }
#else
  ResourceManager::deleteInstance();
#endif
  return iRetVal;
}

/* <EOF> */
