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
#include "uima/xmiwriter.hpp"
#include "uima/xmideserializer.hpp"
#include "uima/xcasdeserializer.hpp"
#include "uima/casdefinition.hpp"
#include "xercesc/framework/LocalFileInputSource.hpp"

#include <fstream>

#ifndef NDEBUG
#define ASSERT_OR_THROWEXCEPTION(x) assert(x)
#else
#define ASSERT_OR_THROWEXCEPTION(x) if (!(x)) { cerr << __FILE__ << ": Error in line " << __LINE__ << endl; exit(1); }
#endif
#define LOG(x) cout << __FILE__ << __LINE__ << ": " << x << endl

using namespace uima;
using namespace std;

icu::UnicodeString str1("string1");
icu::UnicodeString str2("string2");
icu::UnicodeString str3("string3");
icu::UnicodeString str4("string4");
icu::UnicodeString str5("string5");

UnicodeStringRef strings[] = {
                               str1, str3, str2, str5, str4
                             };

int ints[] = {
               1,2,3,4,5
             };
float floats[] = {
                   (float)1.1, (float)2.2, (float)3.3, (float)4.4, (float)5.5
                 };
char chars[] = {
                 'a','b','c','c','d'
               };
int shorts[] = {
                 10,20,30,40,50
               };
int longs[] = {
                10000,20000,30000,40000,50000
              };
double doubles[] = {
                     21E10,
                     31E10,
                     41E10,
                     51E10,
                     61E10

                   };

bool val = false;
UnicodeString ustr("this beer is good");
UnicodeString ustrWithXmlEscapeChars("TestingXmlEscapeChars'\"&><\r\n");
int begin = 1;
int end = 5;
char * viewName = "EnglishDocument";
#define BOOLEAN_ARRAY_SIZE 20
void validateFS(CAS & cas)  {


  Type testType = cas.getTypeSystem().getType("test.primitives.Example");
  Feature intF = testType.getFeatureByBaseName("intFeature");
  Feature floatF = testType.getFeatureByBaseName("floatFeature");
  Feature stringF = testType.getFeatureByBaseName("stringFeature");
  Feature booleanF = testType.getFeatureByBaseName("boolFeature");
  Feature byteF = testType.getFeatureByBaseName("byteFeature");
  Feature shortF = testType.getFeatureByBaseName("shortFeature");
  Feature longF = testType.getFeatureByBaseName("longFeature");
  Feature doubleF = testType.getFeatureByBaseName("doubleFeature");

  Feature intArrayF = testType.getFeatureByBaseName("intArrayFeature");
  Feature floatArrayF = testType.getFeatureByBaseName("floatArrayFeature");
  Feature stringArrayF = testType.getFeatureByBaseName("stringArrayFeature");
  Feature booleanArrayF = testType.getFeatureByBaseName("boolArrayFeature");
  Feature byteArrayF = testType.getFeatureByBaseName("byteArrayFeature");
  Feature shortArrayF = testType.getFeatureByBaseName("shortArrayFeature");
  Feature longArrayF = testType.getFeatureByBaseName("longArrayFeature");
  Feature doubleArrayF = testType.getFeatureByBaseName("doubleArrayFeature");
  Feature intListF = testType.getFeatureByBaseName("intListFeature");
  Feature floatListF = testType.getFeatureByBaseName("floatListFeature");
  Feature stringListF = testType.getFeatureByBaseName("stringListFeature");
  Feature fsListF = testType.getFeatureByBaseName("fsListFeature");
  Feature fsArrayF = testType.getFeatureByBaseName("fsArrayFeature");

  //get index repository
  FSIndexRepository & indexRep = cas.getIndexRepository();

  //   get a view
  CAS * englishView = cas.getView(viewName);
  ASSERT_OR_THROWEXCEPTION(EXISTS(englishView));
  ASSERT_OR_THROWEXCEPTION(0==englishView->getDocumentText().compare(ustr));

  ANIndex index = englishView->getAnnotationIndex(testType);
  ANIterator iter = index.iterator();
  AnnotationFS fs = iter.get();
  ASSERT_OR_THROWEXCEPTION(fs.isValid());

  size_t num;
  
  StringListFS strListFS = fs.getStringListFSValue(stringListF);
  for (num=0; num< NUMBEROF(floats); num++) {
    ASSERT_OR_THROWEXCEPTION(strListFS.getHead()==strings[num]);
	strListFS = strListFS.getTail();
  }
  ASSERT_OR_THROWEXCEPTION(NUMBEROF(strings)==num);
  
  StringArrayFS stringArrayFS = fs.getStringArrayFSValue(stringArrayF);
  ASSERT_OR_THROWEXCEPTION(NUMBEROF(strings)==stringArrayFS.size());
  for (size_t i=0; i< NUMBEROF(strings); ++i) {
    ASSERT_OR_THROWEXCEPTION(0==stringArrayFS.get(i).compare(strings[i]));
  }
  
  IntListFS intListFS = fs.getIntListFSValue(intListF);
  for (num=0; num< NUMBEROF(ints); num++) {
    ASSERT_OR_THROWEXCEPTION(intListFS.getHead()==ints[num]);
	intListFS = intListFS.getTail();
  }
  ASSERT_OR_THROWEXCEPTION(NUMBEROF(ints)==num);
  
  IntArrayFS intArrayFS = fs.getIntArrayFSValue(intArrayF);
  ASSERT_OR_THROWEXCEPTION(NUMBEROF(ints)==intArrayFS.size());
  for (size_t i=0; i< NUMBEROF(ints); ++i) {
    ASSERT_OR_THROWEXCEPTION(intArrayFS.get(i)==ints[i]);
  }

  FloatListFS floatListFS = fs.getFloatListFSValue(floatListF);
  for (num=0; num< NUMBEROF(floats); num++) {
    ASSERT_OR_THROWEXCEPTION(floatListFS.getHead()==floats[num]);
	floatListFS = floatListFS.getTail();
  }
  ASSERT_OR_THROWEXCEPTION(NUMBEROF(floats)==num);

  FloatArrayFS floatArrayFS = fs.getFloatArrayFSValue(floatArrayF);
  ASSERT_OR_THROWEXCEPTION(NUMBEROF(floats)==floatArrayFS.size());
  for (size_t i=0; i< NUMBEROF(floats); ++i) {
    ASSERT_OR_THROWEXCEPTION(floatArrayFS.get(i)==floats[i]);
  }

  ByteArrayFS byteArrayFS = fs.getByteArrayFSValue(byteArrayF);
  ASSERT_OR_THROWEXCEPTION(NUMBEROF(chars)==byteArrayFS.size());
  for (size_t i=0; i< NUMBEROF(chars); ++i) {
    ASSERT_OR_THROWEXCEPTION(byteArrayFS.get(i)==chars[i]);
  }

  BooleanArrayFS boolArrayFS = fs.getBooleanArrayFSValue(booleanArrayF);
  ASSERT_OR_THROWEXCEPTION(BOOLEAN_ARRAY_SIZE==boolArrayFS.size());
  for (size_t i=0; i< BOOLEAN_ARRAY_SIZE; ++i) {
    val = !val;
    ASSERT_OR_THROWEXCEPTION(boolArrayFS.get(i)==val);
  }

  ShortArrayFS shortArrayFS = fs.getShortArrayFSValue(shortArrayF);
  ASSERT_OR_THROWEXCEPTION(NUMBEROF(shorts)==shortArrayFS.size());
  for (size_t i=0; i< NUMBEROF(shorts); ++i) {
    ASSERT_OR_THROWEXCEPTION(shortArrayFS.get(i)==shorts[i]);
  }

  LongArrayFS longArrayFS = fs.getLongArrayFSValue(longArrayF);
  ASSERT_OR_THROWEXCEPTION(NUMBEROF(longs)==longArrayFS.size());
  for (size_t i=0; i< NUMBEROF(longs); ++i) {
    ASSERT_OR_THROWEXCEPTION(longArrayFS.get(i)==longs[i]);
  }

  DoubleArrayFS doubleArrayFS = fs.getDoubleArrayFSValue(doubleArrayF);
  ASSERT_OR_THROWEXCEPTION(NUMBEROF(doubles)==doubleArrayFS.size());
  for (size_t i=0; i< NUMBEROF(doubles); ++i) {
    ASSERT_OR_THROWEXCEPTION(doubleArrayFS.get(i) == doubles[i]);
  }

  FeatureStructure listFS = fs.getFSValue(fsListF);
  num=0;
  ListFS curNode(listFS);
  while (!curNode.isEmpty()) {
    num++;
	FeatureStructure fs = curNode.getHead();
	ASSERT_OR_THROWEXCEPTION(fs.getType().getName().compare(CAS::TYPE_NAME_ANNOTATION)==0);
    curNode = curNode.getTail();
  }
  ASSERT_OR_THROWEXCEPTION(num==2);
  
  ArrayFS arrayFS = (ArrayFS) fs.getFSValue(fsArrayF);
  ASSERT_OR_THROWEXCEPTION(3==arrayFS.size());
  for (int t=0;t<3;t++) {
    FeatureStructure fs = arrayFS.get(t);
	ASSERT_OR_THROWEXCEPTION(fs.getType().getName().compare(CAS::TYPE_NAME_ANNOTATION)==0);
  }

  // check scalar values
  ASSERT_OR_THROWEXCEPTION(0==fs.getStringValue(stringF).compare(strings[0]));
  ASSERT_OR_THROWEXCEPTION(fs.getBeginPosition()==begin);
  ASSERT_OR_THROWEXCEPTION(fs.getEndPosition()==end);
  ASSERT_OR_THROWEXCEPTION(fs.getFloatValue(floatF)==floats[0]);
  ASSERT_OR_THROWEXCEPTION(fs.getBooleanValue(booleanF)==val);
  ASSERT_OR_THROWEXCEPTION(fs.getByteValue(byteF)=='z');
  ASSERT_OR_THROWEXCEPTION(fs.getShortValue(shortF)==shorts[0]);
  ASSERT_OR_THROWEXCEPTION(fs.getLongValue(longF)==longs[0]);
  ASSERT_OR_THROWEXCEPTION(fs.getDoubleValue(doubleF)==doubles[0]);
}


void doTestXmlEscapeChars(internal::CASDefinition * casDef) {
  ErrorInfo errorInfo;
  CAS * cas1 = Framework::createCAS(*casDef,errorInfo);
  CAS * cas2 = Framework::createCAS(*casDef,errorInfo);
  ASSERT_OR_THROWEXCEPTION( EXISTS(cas1) );
  ASSERT_OR_THROWEXCEPTION( EXISTS(cas2) );

   cas1->setDocumentText(ustrWithXmlEscapeChars);
  // Serialize Xmi
  ofstream outputStream;
  outputStream.open("temp.xmi");
  ASSERT_OR_THROWEXCEPTION(outputStream.is_open());

  XmiWriter writer(*cas1, false);
  writer.write(outputStream);
  outputStream.close();

  // Deserialize xmi
  cas2->reset();
  XmiDeserializer::deserialize("temp.xmi", *cas2);

  //compare document text
  ASSERT_OR_THROWEXCEPTION(cas1->getDocumentText().compare(ustrWithXmlEscapeChars)==0);
  ASSERT_OR_THROWEXCEPTION(cas1->getDocumentText().compare(cas2->getDocumentText())==0);
  ASSERT_OR_THROWEXCEPTION(cas1->getDocumentText().length() == cas2->getDocumentText().length());

  //reserialize cas1
  stringstream str1;
  XmiWriter writerstr1(*cas1, false);
  writerstr1.write(str1);
  
  //reserialize cas2
  stringstream str2;
  XmiWriter writer2(*cas2,true);
  writer2.write(str2);

 
  delete cas1;
  delete cas2;
}



void testMultipleSofas(internal::CASDefinition * casDef)  {
  try {
    ErrorInfo errorInfo;
    CAS * cas = uima::Framework::createCAS(*casDef, errorInfo);
    ASSERT_OR_THROWEXCEPTION(EXISTS(cas));

    // set document text for the initial view
    cas->setDocumentText(UnicodeString("This is a test"));
    // create a new view and set its document text
    CAS * cas2 = cas->createView("OtherSofa");
    cas2->setDocumentText(UnicodeString("This is only a test"));

    // create an annotation and add to index of both views
    Type annotType = cas->getTypeSystem().getType(CAS::TYPE_NAME_ANNOTATION);
    AnnotationFS anAnnot = cas->createAnnotation(annotType, 0, 5);
    cas->getIndexRepository().addFS(anAnnot);
    cas2->getIndexRepository().addFS(anAnnot);
    FSIndex tIndex = cas->getAnnotationIndex();
    FSIndex t2Index = cas2->getAnnotationIndex();
    ASSERT_OR_THROWEXCEPTION(tIndex.getSize() == 2); // document annot and new Annotation
    ASSERT_OR_THROWEXCEPTION(t2Index.getSize()== 2); 

    //serialize
    ofstream outputStream;
    outputStream.open("temp.xmi");
    ASSERT_OR_THROWEXCEPTION(outputStream.is_open());

    XmiWriter writer(*cas,true);
    writer.write(outputStream);
    outputStream.close();

    ostringstream str;
    XmiWriter writerstr(*cas,true);
    writerstr.write(str);
    str.flush();

    // deserialize into another CAS (repeat twice to check it still works after reset)
    CAS * newCas = uima::Framework::createCAS(*casDef, errorInfo);
    ASSERT_OR_THROWEXCEPTION(EXISTS(cas));

    for (int i = 0; i < 2; i++) {
      uima::XmiDeserializer::deserialize("temp.xmi",*newCas);
      // check sofas
      ASSERT_OR_THROWEXCEPTION(newCas->getDocumentText().compare(UnicodeString("This is a test"))==0);
      CAS * newCas2 = newCas->getView("OtherSofa");
      ASSERT_OR_THROWEXCEPTION(newCas2->getDocumentText().compare(UnicodeString("This is only a test"))==0);
      ASSERT_OR_THROWEXCEPTION(newCas->getAnnotationIndex().getSize() == 2); // document annot and new Annotation
      ASSERT_OR_THROWEXCEPTION(newCas2->getAnnotationIndex().getSize()== 2); 
      newCas->reset();
    }
    delete cas;
    delete newCas;
  } catch (Exception e) {
    ASSERT_OR_THROWEXCEPTION(e.getErrorInfo().getErrorId()==0);
  }
}

void doTestNoInitialSofa(internal::CASDefinition * casDef) {
  ErrorInfo errorInfo;
  CAS * cas1 = Framework::createCAS(*casDef,errorInfo);
  CAS * cas2 = Framework::createCAS(*casDef,errorInfo);
  ASSERT_OR_THROWEXCEPTION( EXISTS(cas1) );
  ASSERT_OR_THROWEXCEPTION( EXISTS(cas2) );
  // create non-annotation type so as not to create the _InitialView Sofa
  IntArrayFS intArrayFS = cas1->createIntArrayFS(5);
  intArrayFS.set(0,1);
  intArrayFS.set(1,2);
  intArrayFS.set(2,3);
  intArrayFS.set(3,4);
  intArrayFS.set(4,5);
  cas1->getIndexRepository().addFS(intArrayFS);

  // Serialize Xmi
  ofstream outputStream;
  outputStream.open("temp.xmi");
  ASSERT_OR_THROWEXCEPTION(outputStream.is_open());

  XmiWriter writer(*cas1, false);
  writer.write(outputStream);
  outputStream.close();

  // Deserialize xmi
  cas2->reset();
  XmiDeserializer::deserialize("temp.xmi", *cas2);

  //reserialize cas1
  stringstream str1;
  XmiWriter writerstr1(*cas1, false);
  writerstr1.write(str1);
  
  //reserialize cas2
  stringstream str2;
  XmiWriter writer2(*cas2,true);
  writer2.write(str2);

  //compare serialized xmi string
  ASSERT_OR_THROWEXCEPTION(str1.str().compare(str2.str())==0);

  delete cas1;
  delete cas2;
}

void doTestDeserializeAndReserialize(internal::CASDefinition * casDef) {

  ErrorInfo errInfo;
  //LOG("deserialize xcas into cas1");
  CAS * cas1 = uima::Framework::createCAS(*casDef, errInfo);
  ASSERT_OR_THROWEXCEPTION( EXISTS(cas1) );
  UnicodeString xcasFile("ExampleCas/cas.xml");
  UnicodeString xcasfn = ResourceManager::resolveFilename(xcasFile, xcasFile);
  XCASDeserializer::deserialize(xcasfn, *cas1);

  // Serialize Xmi
  ofstream outputStream;
  outputStream.open("temp.xmi");
  ASSERT_OR_THROWEXCEPTION(outputStream.is_open());

  // reserialize as XMI
  //LOG("serialize as xmi");
  XmiWriter xmiw1(*cas1, false);
  xmiw1.write(outputStream);
  outputStream.close();

  // deserialize XMI into another CAS
  CAS * cas2 = uima::Framework::createCAS(*casDef, errInfo);
  ASSERT_OR_THROWEXCEPTION( EXISTS(cas2) );
  XmiDeserializer::deserialize("temp.xmi",*cas2);

  // compare
  //LOG("comparing cas1 and cas2 getDocumentText() ");
  ASSERT_OR_THROWEXCEPTION(cas1->getAnnotationIndex().getSize() == cas2->getAnnotationIndex().getSize());
  ASSERT_OR_THROWEXCEPTION(cas1->getDocumentText().length() == cas2->getDocumentText().length());
  ASSERT_OR_THROWEXCEPTION(cas1->getDocumentText().compare(cas2->getDocumentText()) == 0 );

  // check that array refs are not null
  //LOG("comparing cas1 and cas2 array refs");
  Type entityType = cas2->getTypeSystem().getType("org.apache.uima.testTypeSystem.Entity");
  Feature classesFeat = entityType.getFeatureByBaseName("classes");
  FSIterator iter = cas2->getIndexRepository().getIndex("testEntityIndex").iterator();
  ASSERT_OR_THROWEXCEPTION(iter.isValid());
  while (iter.isValid()) {
    FeatureStructure fs = (FeatureStructure) iter.get();
    ASSERT_OR_THROWEXCEPTION(fs.isValid());
    StringArrayFS arrayFS = (StringArrayFS) fs.getFeatureValue(classesFeat);
    ASSERT_OR_THROWEXCEPTION(arrayFS.isValid());
    for (size_t i = 0; i < arrayFS.size(); i++) {
      ASSERT_OR_THROWEXCEPTION(arrayFS.get(i).length() != 0);
    }
    iter.moveToNext();
  }

  delete cas1;
  delete cas2;
}

void testOotsNewPrimitives(internal::CASDefinition * partialCasDef,
				                 internal::CASDefinition * casDef,
				                const char * inputXCas) {
	ErrorInfo errInfo;
	XmiSerializationSharedData sharedData;

  //LOG("deserialize xcas into cas1 and serialize as XMI");
  CAS * cas1 = uima::Framework::createCAS(*casDef, errInfo);
  ASSERT_OR_THROWEXCEPTION( EXISTS(cas1) );
  UnicodeString xFile(inputXCas);
  UnicodeString xfn = ResourceManager::resolveFilename(xFile, xFile);
  XCASDeserializer::deserialize(xfn, *cas1);

	ofstream outputStream;
	outputStream.open("oots.xmi");
	ASSERT_OR_THROWEXCEPTION(outputStream.is_open());
	XmiWriter writer(*cas1, false);
	writer.write(outputStream);
	outputStream.close();

  //LOG("deserialize XMI into partial cas2 and serialize ");
  CAS * cas2 = uima::Framework::createCAS(*partialCasDef, errInfo);
  ASSERT_OR_THROWEXCEPTION( EXISTS(cas2) );
  XmiDeserializer::deserialize("oots.xmi", *cas2, sharedData);
  XmiWriter writer2(*cas2, false, &sharedData);
  outputStream.open("oots2.xmi");
  writer2.write(outputStream);
  outputStream.close();

  //LOG("deserialize XMI into complete typesystem cas3 ");
  CAS * cas3 = uima::Framework::createCAS(*casDef, errInfo);
  ASSERT_OR_THROWEXCEPTION( EXISTS(cas3) );
  XmiDeserializer::deserialize("oots2.xmi", *cas3);

  //LOG("compare" );
  validateFS(*cas3);

  delete cas1;
  delete cas2;
  delete cas3;
}  


void testOotsComplexCas(internal::CASDefinition * partialCasDef,
				                  internal::CASDefinition * casDef,
				                  const char * inputXCas) {
	ErrorInfo errInfo;
	XmiSerializationSharedData sharedData;

  //LOG("deserialize xcas into cas1 and serialize as XMI");
  CAS * cas1 = uima::Framework::createCAS(*casDef, errInfo);
  ASSERT_OR_THROWEXCEPTION( EXISTS(cas1) );
  UnicodeString xFile(inputXCas);
  UnicodeString xfn = ResourceManager::resolveFilename(xFile, xFile);
  XCASDeserializer::deserialize(xfn, *cas1);

	ofstream outputStream;
	outputStream.open("oots.xmi");
	ASSERT_OR_THROWEXCEPTION(outputStream.is_open());
	XmiWriter writer(*cas1, false);
	writer.write(outputStream);
	outputStream.close();

  //LOG("deserialize XMI into partial cas2 and serialize ");
  CAS * cas2 = uima::Framework::createCAS(*partialCasDef, errInfo);
  ASSERT_OR_THROWEXCEPTION( EXISTS(cas2) );
  XmiDeserializer::deserialize("oots.xmi", *cas2, sharedData);
  XmiWriter writer2(*cas2, false, &sharedData);
  outputStream.open("oots2.xmi");
  writer2.write(outputStream);
  outputStream.close();
 
  //LOG("deserialize XMI into complete typesystem cas3 ");
  CAS * cas3 = uima::Framework::createCAS(*casDef, errInfo);
  ASSERT_OR_THROWEXCEPTION( EXISTS(cas3) );
  XmiDeserializer::deserialize("oots2.xmi", *cas3);

  //LOG("compare" );
  ASSERT_OR_THROWEXCEPTION(cas1->getAnnotationIndex().getSize() == cas3->getAnnotationIndex().getSize());
  ASSERT_OR_THROWEXCEPTION(cas1->getDocumentText().length() == cas3->getDocumentText().length());
  ASSERT_OR_THROWEXCEPTION(cas1->getDocumentText().compare(cas3->getDocumentText()) == 0 );
  //TODO more compare

  delete cas1;
  delete cas2;
  delete cas3;
}  


void doTestFSRefReserialization(internal::CASDefinition * casDef) {

  ErrorInfo errInfo;
  XmiSerializationSharedData sharedData;
  
  CAS * cas = uima::Framework::createCAS(*casDef, errInfo);
  ASSERT_OR_THROWEXCEPTION( EXISTS(cas) );
  CAS * view = cas->createView("AView");
  view->setDocumentText(UnicodeString("sample text for AView"));

  Type testType = view->getTypeSystem().getType("test.primitives.Example");
  ASSERT_OR_THROWEXCEPTION( testType.isValid() );
  Feature stringF = testType.getFeatureByBaseName("stringFeature");
  ASSERT_OR_THROWEXCEPTION( stringF.isValid() );
  Feature beginF = testType.getFeatureByBaseName("begin");
  ASSERT_OR_THROWEXCEPTION( beginF.isValid() );
  Feature endF = testType.getFeatureByBaseName("end");
  ASSERT_OR_THROWEXCEPTION( endF.isValid() );
  Feature stringArrayF = testType.getFeatureByBaseName("stringArrayFeatureMultiRef");
  ASSERT_OR_THROWEXCEPTION( stringArrayF.isValid() );
  Feature otherF = testType.getFeatureByBaseName("otherAnnotation");
  ASSERT_OR_THROWEXCEPTION( otherF.isValid() );
  Type annotType = cas->getTypeSystem().getType(CAS::TYPE_NAME_ANNOTATION);

  //get index repository
  FSIndexRepository & indexRep = view->getIndexRepository();

  //create FS but festures that are FS refs unset
  FeatureStructure fs = view->createFS(testType);
  ASSERT_OR_THROWEXCEPTION( fs.isValid() );
  fs.setStringValue(stringF, "example");
  indexRep.addFS(fs);

  // Serialize Xmi
  ofstream outputStream;
  outputStream.open("temp.xmi");
  ASSERT_OR_THROWEXCEPTION(outputStream.is_open());
  XmiWriter xmiwriter(*cas, false);
  xmiwriter.write(outputStream);
  outputStream.close();

  // deserialize XMI into another CAS
  CAS * cas1 = uima::Framework::createCAS(*casDef, errInfo);
  ASSERT_OR_THROWEXCEPTION( EXISTS(cas1) );
  XmiDeserializer::deserialize("temp.xmi",*cas1, sharedData);

  CAS * view1 = cas1->getView("AView");
  // compare
  ASSERT_OR_THROWEXCEPTION(view1->getAnnotationIndex().getSize() == view->getAnnotationIndex().getSize());
  ANIterator iter = view1->getAnnotationIndex().iterator();
  ASSERT_OR_THROWEXCEPTION(iter.isValid());
  FeatureStructure fs1;

  fs1 = (FeatureStructure) iter.get(); //document  annotation
  ASSERT_OR_THROWEXCEPTION(fs1.isValid());
  iter.moveToNext();
  fs1 = (FeatureStructure) iter.get(); //example fs
  ASSERT_OR_THROWEXCEPTION(fs1.isValid());
  ASSERT_OR_THROWEXCEPTION(fs1.getIntValue(beginF) == fs.getIntValue(beginF));
  ASSERT_OR_THROWEXCEPTION(fs1.getStringValue(stringF).compare(fs.getStringValue(stringF)) == 0);
  
  //create FS and set FS ref feature
  AnnotationFS otherFS = view1->createAnnotation(annotType, 0,10);
  fs1.setFSValue(otherF, otherFS);
  //create a StringArray FS and set StringArrayFS ref feature
  StringArrayFS strArrayFS = view1->createStringArrayFS(5);
  strArrayFS.set(0,UnicodeString("first string"));
  strArrayFS.set(1, UnicodeString("second string"));
  fs1.setFSValue(stringArrayF, strArrayFS);

  //serialize
  outputStream.open("temp.xmi");
  ASSERT_OR_THROWEXCEPTION(outputStream.is_open());
  XmiWriter xmiwriter1(*cas1, false, &sharedData);
  xmiwriter1.write(outputStream);
  outputStream.close();

  //deserialize and check that FS reference feature is as expected.
  CAS * cas2 = uima::Framework::createCAS(*casDef, errInfo);
  ASSERT_OR_THROWEXCEPTION( EXISTS(cas2) );
  XmiSerializationSharedData sharedData1;
  XmiDeserializer::deserialize("temp.xmi",*cas2, sharedData1);
  CAS * view2 = cas2->getView("AView");

  // check that array refs are not null
  ASSERT_OR_THROWEXCEPTION(view2->getAnnotationIndex().getSize() == view1->getAnnotationIndex().getSize());
  iter = view2->getAnnotationIndex().iterator();
  ASSERT_OR_THROWEXCEPTION(iter.isValid());
  FeatureStructure fs2;
  fs2 = (FeatureStructure) iter.get();
  ASSERT_OR_THROWEXCEPTION(fs2.isValid());
  iter.moveToNext();
  fs2 = (FeatureStructure) iter.get();
  ASSERT_OR_THROWEXCEPTION(fs2.isValid());
  ASSERT_OR_THROWEXCEPTION(fs2.getStringValue(stringF).compare("example") == 0);
  AnnotationFS otherfs2 = (AnnotationFS) fs2.getFSValue(otherF);
  ASSERT_OR_THROWEXCEPTION(otherfs2.isValid());
  StringArrayFS strArrayFS1 = fs2.getStringArrayFSValue(stringArrayF);
  ASSERT_OR_THROWEXCEPTION(strArrayFS1.isValid());
  ASSERT_OR_THROWEXCEPTION(strArrayFS1.size() == 5);

  delete cas;
  delete cas1;
  delete cas2;
}


/* ----------------------------------------------------------------------- */
/*       Main routine                                                      */
/* ----------------------------------------------------------------------- */

int main(int argc, char * argv[]) /*
---------------------------------- */
{
  LOG("UIMATEST_XMISERIALIZATION started");

  int iRetVal = 0;

//#if !defined(NDEBUG) && defined(_MSC_VER)
//   iRetVal = _CrtSetBreakAlloc(662909);
//#endif
  try {
    ResourceManager::createInstance("testxmi");
    ofstream outputStream;
    ErrorInfo errorInfo;

    TextAnalysisEngineSpecifierBuilder builder;
	
	  //setup
	  UnicodeString tsFile("ExampleCas/testTypeSystem.xml");
    UnicodeString tsfn = ResourceManager::resolveFilename(tsFile, tsFile);
    TypeSystem * ts = Framework::createTypeSystem( ((UnicodeStringRef)tsfn).asUTF8().c_str(),errorInfo);
    ASSERT_OR_THROWEXCEPTION( EXISTS(ts) );
	
	  UnicodeString tsFile2("ExampleCas/testTypeSystem_withMultiRefs.xml");
    UnicodeString tsfn2 = ResourceManager::resolveFilename(tsFile2, tsFile2);
    TypeSystem * ts2 = Framework::createTypeSystem( ((UnicodeStringRef)tsfn2).asUTF8().c_str(),errorInfo);
    ASSERT_OR_THROWEXCEPTION( EXISTS(ts2) );

    UnicodeString indexFile("ExampleCas/testIndexes.xml");
    UnicodeString indexfn = ResourceManager::resolveFilename(indexFile, indexFile);
	  size_t uiLen = indexfn.length();
    auto_array<UChar> arBuffer( new UChar[uiLen + 1] );
    assert( EXISTS(arBuffer.get()));
    indexfn.extract(0, uiLen, arBuffer.get());
    (arBuffer.get())[uiLen] = 0; // terminate the buffer with 0
	  LocalFileInputSource fileIS((XMLCh const *) arBuffer.get());
	  AnalysisEngineMetaData::TyVecpFSIndexDescriptions fsDesc;
	  builder.parseFSIndexDescription(fsDesc,fileIS);
    ASSERT_OR_THROWEXCEPTION( EXISTS(fsDesc.size() > 0) );

	  internal::CASDefinition * casDef = internal::CASDefinition::createCASDefinition(*ts,fsDesc);
    //test complete ts 
	  LOG("UIMACPP_XMITEST Complex CAS with complete typesystem Start");
		doTestDeserializeAndReserialize(casDef);
	  delete casDef;
	  LOG("UIMACPP_XMITEST Complex CAS with complete typesystem Finished");

    //test complete ts with multiple refs
	  LOG("UIMACPP_XMITEST Complex CAS with MultiRefs Start");
	  casDef = internal::CASDefinition::createCASDefinition(*ts2,fsDesc);
    doTestDeserializeAndReserialize(casDef);
	  delete casDef;
	  LOG("UIMACPP_XMITEST Complex CAS with MultiRefs Finished");

	  casDef = internal::CASDefinition::createCASDefinition(*ts,fsDesc);
    //testNoInitialSofa
	  LOG("UIMACPP_XMITEST doTestNoInitialSofa Start"); 
	  doTestNoInitialSofa(casDef);
	  LOG("UIMACPP_XMITEST doTestNoInitialSofa Finished"); 

	  //testMultipleSofas
	  LOG("UIMACPP_XMITEST testMultipleSofas Start");
	  testMultipleSofas(casDef);
	  LOG("UIMACPP_XMITEST testMultipleSofas Finished");

    //test Xml Escape Chars
	  LOG("UIMACPP_XMITEST doTestXmlEscapeChars Start"); 
	  doTestXmlEscapeChars(casDef);
	  LOG("UIMACPP_XMITEST doTestXmlEscapeChars Finished"); 

    
	  //test OOTS Missing Type 1
	  TypeSystemDescription * baseTSDesc = new TypeSystemDescription();
	  TypeSystem * baseTS = Framework::createTypeSystem(*baseTSDesc,"base",errorInfo);
	  internal::CASDefinition * baseCasDef = internal::CASDefinition::createCASDefinition(*baseTS);

    UnicodeString newpFile("ExampleCas/newprimitivesTypeSystem.xml");
    UnicodeString newpfn = ResourceManager::resolveFilename(newpFile, newpFile);
	  TypeSystem *primitivests = Framework::createTypeSystem( ((UnicodeStringRef)newpfn).asUTF8().c_str(),
										errorInfo );
	  internal::CASDefinition * primitivesCasDef = 
	  internal::CASDefinition::createCASDefinition(*primitivests);

    LOG("UIMACPP_XMITEST OOTS new primitives missing type Start");
	  UnicodeString newpxcasFile("ExampleCas/newprimitives.xcas");
    UnicodeString newpxcasfn = ResourceManager::resolveFilename(newpxcasFile, newpxcasFile);
    testOotsNewPrimitives(baseCasDef, primitivesCasDef, ((UnicodeStringRef)newpxcasfn).asUTF8().c_str());
     LOG("UIMACPP_XMITEST OOTS new primitives missing type Finished");


    //test OOTS missing feature
	  LOG("UIMACPP_XMITEST OOTS new primitives missing feature Start");
	  UnicodeString newpPartialFile("ExampleCas/newprimitivesPartialTypeSystem.xml");
    UnicodeString newpPartialfn = ResourceManager::resolveFilename(newpPartialFile, newpFile);
	  TypeSystem * newpPartialts = Framework::createTypeSystem( ((UnicodeStringRef)newpPartialfn).asUTF8().c_str(),
										errorInfo );
	  internal::CASDefinition * newpPartialCasDef = 
                    internal::CASDefinition::createCASDefinition(*newpPartialts);
    testOotsNewPrimitives(newpPartialCasDef, primitivesCasDef, "ExampleCas/newprimitives.xcas");
	  delete newpPartialts;
	  delete newpPartialCasDef;
	  LOG("UIMACPP_XMITEST OOTS new primitives missing features Finished");

	  //test OOTS complex cas missing types
	  LOG("UIMACPP_XMITEST OOTS Complex CAS Missing Types Start");
	  testOotsComplexCas(baseCasDef, casDef, "ExampleCas/cas.xml");
	  LOG("UIMACPP_XMITEST OOTS Complex CAS Missing Types Finished");

    UnicodeString partialTSFile("ExampleCas/partialTestTypeSystem.xml");
    UnicodeString partialTSfn = ResourceManager::resolveFilename(partialTSFile, partialTSFile);
	  TypeSystem * partialts = Framework::createTypeSystem( ((UnicodeStringRef)partialTSfn).asUTF8().c_str(),
										errorInfo );
	  internal::CASDefinition * partialTSCasDef = 
	                internal::CASDefinition::createCASDefinition(*partialts);

    //test OOTS complex cas missing types and features
    LOG("UIMACPP_XMITEST OOTS Complex CAS with partial typesystem Start");
	  testOotsComplexCas(partialTSCasDef, casDef, "ExampleCas/cas.xml");
	  LOG("UIMACPP_XMITEST OOTS Complex CAS with partial typesystem Finished");

 		//test that some xml doc fails
    LOG("UIMACPP_XMITEST Valid XML but not Xmi Cas doc Start");
 		UnicodeString someXmlFile("ExampleCas/cas.xml");
    UnicodeString xmlfn = ResourceManager::resolveFilename(someXmlFile, someXmlFile);
 		CAS * pCas = Framework::createCAS(*casDef,errorInfo);

    bool bExceptionThrown = false;
 		try {
 			XmiDeserializer::deserialize(xmlfn, *pCas);
 		} catch (Exception e)  {
 			LOG("Exception thrown correctly: ");
 			//LOG(e.asString());
 			bExceptionThrown =true;
 		}
 	  ASSERT_OR_THROWEXCEPTION(bExceptionThrown); 
    LOG("UIMACPP_XMITEST Valid XML but not Xmi Cas doc Finished");
  	delete pCas;
		
    //test serialization of FS reference in incoming FS
    //when reserializing the CAS
    doTestFSRefReserialization(primitivesCasDef);   
    LOG("UIMACPP_XMITEST Test reserialization of FS reference Finished");


    delete partialts;
    delete partialTSCasDef;
    delete primitivests;
	  delete primitivesCasDef;
    delete baseTSDesc;
	  delete baseTS;
	  delete baseCasDef;
    
		delete casDef;
    delete ts;
		delete ts2;
    for (size_t i=0; i < fsDesc.size();i++) {
      delete fsDesc.at(i);
    }
    
    LOG("UIMATEST_XMISERIALIZATION finished");
  
  } catch (Exception & exc) {
    cerr << exc.asString() << endl;
    iRetVal = 1;
  } catch (...) {
    cerr << "Unexpected exception " << endl;
    iRetVal = 1;
  }
  return iRetVal;
}

/* <EOF> */
