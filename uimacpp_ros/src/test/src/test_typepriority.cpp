/** \file test_typepriority.cpp .

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
#include "uima/engine.hpp"
#include "uima/typesystem.hpp"
#include "uima/tt_types.hpp"
#include "uima/cas.hpp"
#include "uima/fsindexrepository.hpp"
#include "uima/resmgr.hpp"

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */
#ifndef NDEBUG
#define ASSERT_OR_THROWEXCEPTION(x) assert(x)
#else
#define ASSERT_OR_THROWEXCEPTION(x) if (!(x)) { cerr << __FILE__ << ": Error in line " << __LINE__ << endl; exit(1); }
#endif

#define LOG(x) cout << __FILE__ << __LINE__ << ": " << x << endl

using namespace uima;
using namespace std;
/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */
int findLastPosition(vector<uima::Type> const & types, Type t) {
  int result = -1;
  int i;
  for (i=0; i<types.size(); ++i) {
    if (t.subsumes( types[i] ) ) {
      result = i;
    }
  }
  return result;
}


void checkPriority(vector<uima::Type> const & types, Type t1, Type t2) {
  LOG("Checking priority between " << t1.getName() << " and " <<  t2.getName() );
  int iPos1 = findLastPosition(types, t1);
  ASSERT_OR_THROWEXCEPTION( iPos1 >= 0);
  int iPos2 = findLastPosition(types, t2);
  ASSERT_OR_THROWEXCEPTION( iPos2 >= 0);
  LOG("Last position of annotation of type " << t1.getName() << ": " << iPos1);
  LOG("Last position of annotation of type " << t2.getName() << ": " << iPos2);

  ASSERT_OR_THROWEXCEPTION( iPos1 < iPos2 );
}

void checkIndex(CAS const & rTCAS) {
  uima::Type annType = rTCAS.getTypeSystem().getType(uima::CAS::TYPE_NAME_ANNOTATION);
  uima::Type tokType = rTCAS.getTypeSystem().getType(uima::TT::TYPE_NAME_TOKEN_ANNOTATION);
  uima::Type sentType = rTCAS.getTypeSystem().getType(uima::TT::TYPE_NAME_SENTENCE_ANNOTATION);
  uima::Type parType = rTCAS.getTypeSystem().getType(uima::TT::TYPE_NAME_PARAGRAPH_ANNOTATION);
//   uima::Type docType = rTCAS.getTypeSystem().getType(uima::TT::TYPE_NAME_DOCUMENT_ANNOTATION);
  uima::Type placeType = rTCAS.getTypeSystem().getType(uima::TT::TYPE_NAME_PLACE_NAME_ANNOTATION);
  uima::Type lexType = rTCAS.getTypeSystem().getType(uima::TT::TYPE_NAME_LEXICAL_ANNOTATION);
  uima::Type docStructType = rTCAS.getTypeSystem().getType(uima::TT::TYPE_NAME_DOC_STRUCTURE_ANNOTATION);
  uima::Type nameType = rTCAS.getTypeSystem().getType(uima::TT::TYPE_NAME_NAME_ANNOTATION);
  uima::Type mulTokType = rTCAS.getTypeSystem().getType(uima::TT::TYPE_NAME_MULTI_TOKEN_ANNOTATION);
  FSIndex ix = rTCAS.getIndexRepository().getIndex("TestIndex", annType);
  FSIterator it = ix.iterator();

  size_t i=0;
  vector<uima::Type> types;
  for (it.moveToFirst(); it.isValid(); it.moveToNext() ) {
    AnnotationFS annFS = (AnnotationFS) it.get();
    LOG("Annotation " << annFS.getType().getName());
    if (annFS.getBeginPosition() == 0) {
      types.push_back( it.get().getType() );
      LOG("Type " << i << ": " << types[i].getName() );
      ++i;
    }
  }

  /*
  Normally longer annots come first when starting at same position.
  Override this for token (lextype) over sent & para (docstructtype)
  */
  checkPriority( types, lexType, docStructType);

  // Override again for sentence over paragraph
  checkPriority( types, sentType, parType );

}

/* ----------------------------------------------------------------------- */
/*       Main routine                                                      */
/* ----------------------------------------------------------------------- */

int main(int argc, char * argv[]) /*
---------------------------------- */
{
  LOG("UIMATEST_TYPE_PRIORITY started");
  int iRetVal = 0;
  try {
    char const * config =
      "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>"
      "<taeDescription"
      "   xmlns=\"http://uima.apache.org/resourceSpecifier\""
      "   xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\""
      "   xsi:schemaLocation=\"http://uima.apache.org/resourceSpecifier http://uima.apache.org/resourceSpecifierSchema.xsd\""
      "   >"
      "    <frameworkImplementation>UIMA</frameworkImplementation>"
      "    <primitive>false</primitive>"
      "    <delegateAnalysisEngineSpecifiers>"
      "        <delegateAnalysisEngine key=\"frost\">"
  //ee???      "            <includeFile>tok.xml</includeFile>"
      "            <import location=\"tok.xml\" />"
      "        </delegateAnalysisEngine>"
      /* ee - reducing number of annotators used for testing
            "        <delegateAnalysisEngine key=\"doxtract\">"
            "            <includeFile>doxtract.xml</includeFile>"
            "        </delegateAnalysisEngine>"
      */
      "        <delegateAnalysisEngine key=\"dump\">"
  //ee      "            <includeFile>dump.xml</includeFile>"
      "            <import location=\"dump.xml\" />"
      "        </delegateAnalysisEngine>"
      "    </delegateAnalysisEngineSpecifiers>"
      "    <analysisEngineMetaData>"
      "        <name>UIMAAnnotator</name>"
      "        <description></description>"
      "        <version>1.0</version>"
      "        <vendor>IBM Corporation</vendor>"
      "        <flowConstraints>"
      "            <fixedFlow>"
      "                <node>frost</node>"
  //ee      "                <node>doxtract</node>"
      "                <node>dump</node>"
      "            </fixedFlow>"
      "        </flowConstraints>"
      "        <typePriorities>"

      "         <priorityList>"
      "             <type>uima.tt.LexicalAnnotation</type>"
      "             <type>uima.tt.DocStructureAnnotation</type>"
      "         </priorityList>"

      "         <priorityList>"
      "             <type>uima.tt.SentenceAnnotation</type> "
      "             <type>uima.tt.ParagraphAnnotation</type>"
      "         </priorityList>"
      /*
            "         <priorityList>"
            "             <type>uima.tt.MultiTokenAnnotation</type>"
            "             <type>uima.tt.TokenAnnotation</type>"
            "         </priorityList>"
            "            <priorityList>"
            "                <type>uima.tt.TokenAnnotation</type>"
            "                <type>uima.tt.SentenceAnnotation</type>"
            "                <type>uima.tt.ParagraphAnnotation</type>"
            "            </priorityList>"
            "            <priorityList>"
            "                <type>uima.tt.TokenAnnotation</type>"
            "                <type>uima.tcas.DocumentAnnotation</type>"
            "                <type>uima.tt.ParagraphAnnotation</type>"
            "            </priorityList>"
            "            <priorityList>"
            "                <type>uima.tt.TokenAnnotation</type>"
            "                <type>uima.tt.PlaceName</type>"
            "            </priorityList>"
      */
      "        </typePriorities>"
      "        <fsIndexCollection>"
      "          <imports> "
      "            <import location=\"tt_indexes.xml\"/>"
      "           </imports> "
      "           <fsIndexes> "
      "            <fsIndexDescription>"
      "                <label>TestIndex</label>"
      "                <typeName>uima.tcas.Annotation</typeName>"
      "                <kind>sorted</kind>"
      "                <keys>"
      "                    <fsIndexKey>"
      "                        <featureName>begin</featureName>"
      "                        <comparator>standard</comparator>"
      "                    </fsIndexKey>"
      "                    <fsIndexKey>"
      "                        <typePriority></typePriority>"
      "                    </fsIndexKey>"
      "                </keys>"
      "            </fsIndexDescription>"
      "        </fsIndexes> "
      "        </fsIndexCollection> "
      "        <capabilities>"
      "            <capability>"
      "                <inputs></inputs>"
      "                <outputs>"
      "                    <type>uima.tt.TokenAnnotation</type>"
      "                    <type>uima.tt.SentenceAnnotation</type>"
      "                    <type>uima.tt.ParagraphAnnotation</type>"
      "                    <type>uima.tt.NameAnnotation</type>"
      "                </outputs>"
      "                <languagesSupported>"
      "                    <language>en</language>"
      "                </languagesSupported>"
      "            </capability>"
      "        </capabilities>"
      "    </analysisEngineMetaData>"
      "</taeDescription>";

    icu::UnicodeString configUS(config);
    UChar const * buf = configUS.getBuffer();
    size_t len = configUS.length();

    uima::ResourceManager::createInstance("test");

    ErrorInfo errInfo;
    uima::TextAnalysisEngine * pEngine = TextAnalysisEngine::createTextAnalysisEngine(buf, len, errInfo);
    if (pEngine == NULL ) {
      LOG("Error: " << errInfo.asString());
      ASSERT_OR_THROWEXCEPTION(false);
    }
    ASSERT_OR_THROWEXCEPTION(EXISTS(pEngine));
    ASSERT_OR_THROWEXCEPTION( errInfo.getErrorId() == UIMA_ERR_NONE );

    CAS * tcas = pEngine->newCAS();
    ASSERT_OR_THROWEXCEPTION( EXISTS(tcas) );

    icu::UnicodeString us("London calling. The Clash.");
    tcas->setDocumentText(us.getBuffer(), us.length());
    tcas->getDocumentAnnotation().setLanguage("en");
    TyErrorId err = pEngine->process(*tcas);
    ASSERT_OR_THROWEXCEPTION( err == UIMA_ERR_NONE );

    checkIndex( *tcas );

    err = pEngine->destroy();
    ASSERT_OR_THROWEXCEPTION( err == UIMA_ERR_NONE );

    delete tcas;
    delete pEngine;
    LOG("UIMATEST_TYPE_PRIORITY finished");
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


