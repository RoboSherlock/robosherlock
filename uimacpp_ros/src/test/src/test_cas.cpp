/** \file test_engine_basic.cpp .

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

#include "uima/pragmas.hpp"

#include "uima/macros.h"
#include "uima/engine.hpp"

#include "uima/resmgr.hpp"

#include <iostream>
#include "uima/lowlevel_indexiterator.hpp"
#include "uima/lowlevel_fsheap.hpp"
#include "uima/lowlevel_typesystem.hpp"

#include "uima/fsfilterbuilder.hpp"
#include "uima/cas.hpp"
#include "uima/listfs.hpp"
#include "uima/arrayfs.hpp"

#include "uima/lowlevel_indexrepository.hpp"
#include "uima/lowlevel_fsfilter.hpp"

#include "uima/xmltypesystemreader.hpp"

#include "xercesc/util/PlatformUtils.hpp"
#include "xercesc/sax/SAXParseException.hpp"

#include "uima/internal_casimpl.hpp"
#include "uima/internal_fspromoter.hpp"
#include "uima/fsindexrepository.hpp"
#include "uima/typenamespace.hpp"
#include "uima/tt_types.hpp"
#include "uima/internal_aggregate_engine.hpp"
#include "uima/casdefinition.hpp"
#include "uima/taespecifierbuilder.hpp"
#include "xercesc/parsers/XercesDOMParser.hpp"
#include "xercesc/parsers/SAXParser.hpp"
#include "xercesc/dom/DOMException.hpp"
#include "xercesc/dom/DOMNamedNodeMap.hpp"
#include "xercesc/dom/DOMDocument.hpp"
#include "xercesc/dom/DOMElement.hpp"
#include "xercesc/dom/DOMNodeList.hpp"
#include "xercesc/framework/MemBufInputSource.hpp"
#include "uima/xmlerror_handler.hpp"

using namespace uima;
using namespace uima::internal;
using namespace std;
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

#ifndef NDEBUG
#define ASSERT_OR_THROWEXCEPTION(x) assert(x)
#else
#define ASSERT_OR_THROWEXCEPTION(x) if (!(x)) { cerr << __FILE__ << ": Error in line " << __LINE__ << endl; exit(1); }
#endif


#define LOG(x) cout << "[" << __FILE__ << ", " << __LINE__ << "]: " << x << endl

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

static icu::UnicodeString ustrCreatorID("test");

namespace uima {
  namespace internal {
    class TTDefinition : public CASDefinition {
    public:
      void createTTTAESpecifier(TextAnalysisEngineSpecifier & result) {
        /* insert lines to match original code so LOG msgs have same line numbers!
        */
        const util::Location & crDataDir = ResourceManager::getInstance().getLocationData();
        std::string nameInDataDir(crDataDir.getAsCString() );
        nameInDataDir += "descriptors";

        util::Filename typesystemFile(nameInDataDir.c_str(),"tt_typesystem.xml");
        typesystemFile.normalize();

        util::Filename indexesFile(nameInDataDir.c_str(),"tt_indexes.xml");
        indexesFile.normalize();

        icu::UnicodeString typesystemFileUnicode( typesystemFile.getAsCString() );
        icu::UnicodeString indexesFileUnicode( indexesFile.getAsCString() );

        // add the typesystem from an xml file
		std::string tt_types("<?xml version=\"1.0\"?> <taeDescription><analysisEngineMetaData> <typeSystemDescription> <imports> <import location=\"");
        tt_types.append( typesystemFile.getAsCString() );
 
        tt_types.append("\"/> </imports> </typeSystemDescription> <fsIndexCollection> <imports> <import location=\"");
        tt_types.append(indexesFile.getAsCString() );
        tt_types.append("\"/> </imports> </fsIndexCollection> </analysisEngineMetaData> </taeDescription>");

        uima::TextAnalysisEngineSpecifierBuilder builder;
        XercesDOMParser parser;
        uima::XMLErrorHandler errorHandler;
        parser.setErrorHandler(& errorHandler);

		MemBufInputSource memSource((XMLByte const *) tt_types.c_str(), tt_types.length(), "TT typesystem" );

        ///XIncluder includer;
        ///InputSource const & is = includer.resolveXIncludes(memSource);
        ///parser.parse( is );
        parser.parse(memSource);
        DOMDocument* doc = parser.getDocument();
        assert(EXISTS(doc));

        // get top node
        DOMElement * rootElem = doc->getDocumentElement();
        assert(EXISTS(rootElem));

        //         icu::UnicodeString fileLoc( typesystemFile.getAsCString() );
        builder.buildTae(result, rootElem, ".");
      }

      void createPredefinedTTTypes() {
        TextAnalysisEngineSpecifier specifier;
        createTTTAESpecifier(specifier);

        mergeTypeSystem(specifier);

        addTypePriorities(specifier);
      }


      void createPredefinedTTIndexes() {
        TextAnalysisEngineSpecifier specifier;
        createTTTAESpecifier(specifier);

        uima::AnnotatorContext anc(& specifier);
        createIndexesFromANC(anc);
      }

      void createTypes() {
        CASDefinition::createTypes();
        createPredefinedTTTypes();
      }

      void createIndexes() {
        CASDefinition::createIndexes();
        createPredefinedTTIndexes();
      }



      TTDefinition()
          : CASDefinition(NULL) {}


      ~TTDefinition()   { }

      void commitTypeSystem() {
        CASDefinition::commitTypeSystem();
      }

      void commitIndexDefinition() {
        CASDefinition::commitIndexDefinition();
      }


    };
  }
}

uima::internal::CASImpl * getCASImpl(uima::internal::CASDefinition & casDef) {
  ErrorInfo errorInfo;
  return (uima::internal::CASImpl*) Framework::createCAS(casDef, errorInfo);
}


class TestCAS {
  uima::internal::TTDefinition * iv_casDef;
  uima::internal::CASImpl * iv_cas;
public:
  TestCAS()
      : iv_casDef() {
    iv_casDef = new uima::internal::TTDefinition();
    iv_casDef->init();
    iv_casDef->commit();
//       iv_cas = new uima::internal::TCASImpl(*iv_casDef, 5000, 5000, 5000);
    ErrorInfo errorInfo;
    iv_cas =  (uima::internal::CASImpl*)Framework::createCAS(*iv_casDef, errorInfo);
  }
  ~TestCAS() {
    delete iv_cas;
    delete iv_casDef;
  }

  uima::internal::TTDefinition & getCASDef() {
    return *iv_casDef;
  }
  uima::internal::CASImpl & getCAS() {
    return * iv_cas;
  }
};

/*
void prepareCAS(CAS & cas) {
   uima::internal::TCASImpl & tcas = uima::internal::TCASImpl::promoteCAS(cas);
   tcas.createPredefinedTTTypes();

   tcas.commitTypeSystem();

   tcas.createPredefinedTTIndexes();
}
*/


void testLowLevelTypeSystem() {
  LOG("testLowlevelTypeSystem() started");


  ASSERT_OR_THROWEXCEPTION( uima::TypeSystem::NAMESPACE_SEPARATOR == UIMA_NAMESPACE_SEPARATOR_CHAR );
  ASSERT_OR_THROWEXCEPTION( uima::TypeSystem::FEATURE_SEPARATOR == UIMA_TYPE_FEATURE_SEPARATOR_CHAR);

  /*
    top
       t1 [f1: t3, g1:t4]
         t11 [f11: t21]
            t111
         t12 [f12: t211]
         t13 [f13: t14]
            t131 [f131: t132, g131: top]
            t132
         t14 [f14: t2]
       t2 [f2:t2]
         t21 [f21:t2, g21: t211]
           t211
              t2111
       t3
         t31 [f31: t32, g31: t321]
         t32
            t321 [f321: t4]
       t4
   */
  TTDefinition ttdef;
  ttdef.init();
  lowlevel::TypeSystem& ts = ttdef.getTypeSystem();

  lowlevel::TyFSType top = ts.getTopType();
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(top) );
  lowlevel::TyFSType t1 = ts.createType(top, "t1", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t1) );
  lowlevel::TyFSType t2 = ts.createType(top, "t2", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t2) );
  lowlevel::TyFSType t3 = ts.createType(top, "t3", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t3) );
  lowlevel::TyFSType t31 = ts.createType(t3, "t31", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t31) );
  lowlevel::TyFSType t32 = ts.createType(t3, "t32", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t32) );
  lowlevel::TyFSType t321 = ts.createType(t32, "t321", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t321) );
  lowlevel::TyFSType t4 = ts.createType(top, "t4", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t4) );
  lowlevel::TyFSType t21 = ts.createType(t2, "t21", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t21) );
  lowlevel::TyFSType t211 = ts.createType(t21, "t211", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t211) );
  lowlevel::TyFSType t2111 = ts.createType(t211, "t2111", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t2111) );
  lowlevel::TyFSType t11 = ts.createType(t1, "t11", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t11) );
  lowlevel::TyFSType t111 = ts.createType(t11, "t111", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t111) );
  lowlevel::TyFSType t12 = ts.createType(t1, "t12", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t12) );
  lowlevel::TyFSType t13 = ts.createType(t1, "t13", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t13) );
  lowlevel::TyFSType t14 = ts.createType(t1, "t14", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t14) );
  lowlevel::TyFSType t131 = ts.createType(t13, "t131", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t131) );
  lowlevel::TyFSType t132 = ts.createType(t13, "t132", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t132) );

  lowlevel::TyFSFeature f1 = ts.createFeature(t1, t3, false, "f1", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(f1) );
  lowlevel::TyFSFeature g1 = ts.createFeature(t1, t4, false, "g1", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(g1) );
  lowlevel::TyFSFeature f11 = ts.createFeature(t11, t21, false, "f11", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(f11) );
  lowlevel::TyFSFeature f12 = ts.createFeature(t12, t211, false, "f12", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(f12) );
  lowlevel::TyFSFeature f13 = ts.createFeature(t13, t14, false, "f13", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(f13) );
  lowlevel::TyFSFeature f131 = ts.createFeature(t131, t132, false, "f131", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(f131) );
  lowlevel::TyFSFeature g131 = ts.createFeature(t131, top, false, "g131", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(g131) );
  lowlevel::TyFSFeature f14 = ts.createFeature(t14, t2, false, "f14", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(f14) );
  lowlevel::TyFSFeature f2 = ts.createFeature(t2, t2, false, "f2", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(f2) );
  lowlevel::TyFSFeature f21 = ts.createFeature(t21, t2, false, "f21", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(f21) );
  lowlevel::TyFSFeature g21 = ts.createFeature(t21, t211, false, "g21", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(g21) );
  lowlevel::TyFSFeature f31 = ts.createFeature(t31, t32, false, "f31", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(f31) );
  lowlevel::TyFSFeature g31 = ts.createFeature(t31, t321, false, "g31", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(g31) );
  lowlevel::TyFSFeature f321 = ts.createFeature(t321, t4, false, "f321", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(f321) );

  ttdef.commit();

  assert( ts.debugIsConsistent() );
#ifndef NDEBUG
  ts.print(cout);
#endif

  // check subsumption
#define SUBSCHECK_N 17
  lowlevel::TyFSType subscheck[SUBSCHECK_N][2] = {
        {top, t1},     // 1
        {top, top},    // 2
        {t11, t111},  // 3
        {t13, t131}, // 4
        {t13, t132}, // 5
        {t2, t21}, // 6
        {t21, t211}, // 7
        {t211, t2111}, // 8
        {t3, t31}, // 9
        {t3, t32}, // 10
        {t4, t4}, // 11
        {top, t2111}, // 12
        {t1, t111}, // 13
        {t2, t2111}, // 14
        {t3, t321}, // 15
        {top, t321}, // 16
        {top, top}  // 17
      };

  int i;
  for (i=0; i<SUBSCHECK_N; ++i) {
    lowlevel::TyFSType st1 = subscheck[i][0];
    lowlevel::TyFSType st2 = subscheck[i][1];
    ASSERT_OR_THROWEXCEPTION( ts.subsumes(st1, st2) );
    if (st1 == st2) {
      ASSERT_OR_THROWEXCEPTION( ts.subsumes(st2, st1) );
    } else {
      ASSERT_OR_THROWEXCEPTION( ! ts.subsumes(st2, st1) );
    }

  }

  // check appropriateness
#define APPROPCHECK 14
  int appropcheck[APPROPCHECK][2] = {
                                      {
                                        t1, f1
                                      }
                                      , // 0
                                      {t1, g1}, // 1
                                      {t111, f1}, //2
                                      {t12, g1}, //3
                                      {t131, g1}, //4
                                      {t131, f131}, //5
                                      {t131, g131}, //6
                                      {t14, f14}, //7
                                      {t21, f21}, //8
                                      {t211, f21}, //9
                                      {t2111, f21}, //10
                                      {t31, f31}, //11
                                      {t31, f31}, //12
                                      {t321, f321} //13
                                    };

  for (i=0; i< APPROPCHECK; ++i) {
    lowlevel::TyFSType act = appropcheck[i][0];
    lowlevel::TyFSFeature acf = appropcheck[i][1];

    UIMA_TPRINT(" Checking approp: feature " << acf << ", type " << act);
    ASSERT_OR_THROWEXCEPTION( ts.isAppropriateFeature(act, acf) );

    ASSERT_OR_THROWEXCEPTION( acf == ts.getFeatureFromOffset(act, ts.getFeatureOffset( acf) ) );

  }

  LOG("  Testing OO API TypeSystem calls");
  Type oot1 = ts.getType("t1");
  ASSERT_OR_THROWEXCEPTION( oot1.isValid() );
  Type oot11 = ts.getType("t11");
  ASSERT_OR_THROWEXCEPTION( oot11.isValid() );

  ASSERT_OR_THROWEXCEPTION( oot1.subsumes(oot11) );
  ASSERT_OR_THROWEXCEPTION( ! oot11.subsumes(oot1) );

//   FeatureStructure oofs11 = cas.createFS(oot11);

  LOG("  Finished Testing OO API TypeSystem calls");


  LOG("testLowlevelTypeSystem() finished");
}


void testLowLevelTypeSystemExceptions() {
  LOG("testLowlevelTypeSystenExceptions() started");

  TTDefinition ttdef;
  ttdef.init();

  uima::lowlevel::TypeSystem & rTypeSystem = ttdef.getTypeSystem();

  uima::lowlevel::TyFSType intType = rTypeSystem.getTypeByName(CAS::TYPE_NAME_INTEGER);

  bool bExceptionThrown = false;

  LOG("Creating feature on top");
  try {
    rTypeSystem.createFeature( rTypeSystem.getTopType(), intType, false, "bla", "test");
  } catch (uima::lowlevel::FeatureIntroductionFailedException & exc) {
    LOG("Exception thrown correctly: " << exc.asString());
    bExceptionThrown = true;
  }
  ASSERT_OR_THROWEXCEPTION( bExceptionThrown );

  LOG("Creating subtype of integer");
  bExceptionThrown = false;
  try {
    rTypeSystem.createType( intType, "bla", "test");
  } catch (uima::lowlevel::TypeCreationFailedException & exc) {
    LOG("Exception thrown correctly: " << exc.asString());
    bExceptionThrown = true;
  }
  ASSERT_OR_THROWEXCEPTION( bExceptionThrown );

  LOG("Creating feature on integer");
  bExceptionThrown = false;
  try {
    rTypeSystem.createFeature( intType, intType, false, "bla", "test");
  } catch (uima::lowlevel::FeatureIntroductionFailedException & exc) {
    LOG("Exception thrown correctly: " << exc.asString());
    bExceptionThrown = true;
  }
  ASSERT_OR_THROWEXCEPTION( bExceptionThrown );

  LOG("testLowlevelTypeSystenExceptions() finished");
}


/************************************************************************************/
/*
void testOOTypeSystem() {
   LOG("testOOTypeSystem() started");
   / *
     top
        t1 [f1: t3, g1:t4]
          t11 [f11: t21]
             t111
          t12 [f12: t211]
          t13 [f13: t14]
             t131 [f131: t132, g131: top]
             t132
          t14 [f14: t2]
        t2 [f2:t2]
          t21 [f21:t2, g21: t211]
            t211
               t2111
        t3
          t31 [f31: t32, g31: t321]
          t32
             t321 [f321: t4]
        t4
    * /

   Type someType;
   ASSERT_OR_THROWEXCEPTION( ! someType.isValid() );
   Feature someFeature;
   ASSERT_OR_THROWEXCEPTION( ! someFeature.isValid() );

   uima::internal::TCASImpl cas(50000);
   FSSystem & fsSystem = cas.getFSSystem();
   Type top = fsSystem.getTopType();
   Type t1 = top.addSubType("t1");
   ASSERT_OR_THROWEXCEPTION( t1.isValid() );
   Type t2 = top.addSubType("t2");
   ASSERT_OR_THROWEXCEPTION( t2.isValid() );
   Type t21 = t2.addSubType("t21");
   ASSERT_OR_THROWEXCEPTION( t21.isValid() );
   Type t3 = top.addSubType("t3");
   ASSERT_OR_THROWEXCEPTION( t3.isValid() );
   Type t4 = top.addSubType("t4");
   ASSERT_OR_THROWEXCEPTION( t4.isValid() );

   Feature f1 = t1.introduceFeature("f1", t3);
   ASSERT_OR_THROWEXCEPTION( f1.isValid() );
   Feature g1 = t1.introduceFeature("g1", t4);
   ASSERT_OR_THROWEXCEPTION( g1.isValid() );

   Type t11 = t1.addSubType("t11");
   ASSERT_OR_THROWEXCEPTION( t11.isValid() );
   Feature f11 = t11.introduceFeature("f11", t21);
   ASSERT_OR_THROWEXCEPTION( f11.isValid() );

   Type t11prime = fsSystem.findTypeByName("t11");
   ASSERT_OR_THROWEXCEPTION( t11 == t11prime );

   cas.commitTypeSystem();

   ASSERT_OR_THROWEXCEPTION( t1.subsumes(t11) );
   ASSERT_OR_THROWEXCEPTION( ! t11.subsumes(t1) );

   FeatureStructure fs11 = fsSystem.createFS(t11);


   LOG("testOOTypeSystem() finished");
}
*/


/************************************************************************************/

void testLowLevelFSHeap() {
  LOG("testLowlevelFSHeap started");

  TestCAS testcas;
  lowlevel::TypeSystem& ts = testcas.getCASDef().getTypeSystem();


  assert( ts.debugIsConsistent() );
  LOG("Type system commited");

  lowlevel::FSHeap & heap = testcas.getCAS().getHeap();
//   FSHeap heap(ts, 50000);
  LOG("Creating annotation name string");
  icu::UnicodeString usAnnotationTypeName(CAS::TYPE_NAME_ANNOTATION);
  LOG(" annotation name string created");
  lowlevel::TyFSType annotationType = ts.getTypeByName( usAnnotationTypeName );
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(annotationType) );
  lowlevel::TyFSFeature beginPosFeat = ts.getFeatureByBaseName(annotationType, CAS::FEATURE_BASE_NAME_BEGIN );
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(beginPosFeat) );
  lowlevel::TyFSFeature endPosFeat = ts.getFeatureByBaseName( annotationType, CAS::FEATURE_BASE_NAME_END );
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(endPosFeat) );

  lowlevel::TyFSType tokenAnnotationType = ts.getTypeByName( TT::TYPE_NAME_TOKEN_ANNOTATION );
  ASSERT_OR_THROWEXCEPTION( ts.isValidType( tokenAnnotationType ) );
  lowlevel::TyFSFeature lemmaEntryFeat = ts.getFeatureByBaseName( tokenAnnotationType, TT::FEATURE_BASE_NAME_LEMMA_ENTRIES );
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature( lemmaEntryFeat ) );

  lowlevel::TyFSType lemmaType = ts.getTypeByName( TT::TYPE_NAME_LEMMA );
  ASSERT_OR_THROWEXCEPTION( ts.isValidType( lemmaType ) );
  lowlevel::TyFSType keyFeature = ts.getFeatureByBaseName( lemmaType, TT::FEATURE_BASE_NAME_KEY );
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature( keyFeature ) );

  lowlevel::TyFSType listType = ts.getTypeByName(CAS::TYPE_NAME_FS_LIST );
  ASSERT_OR_THROWEXCEPTION( ts.isValidType( listType ) );
  lowlevel::TyFSType ne_listType = ts.getTypeByName(CAS::TYPE_NAME_NON_EMPTY_FS_LIST );
  ASSERT_OR_THROWEXCEPTION( ts.isValidType( ne_listType ) );
  lowlevel::TyFSType e_listType = ts.getTypeByName(CAS::TYPE_NAME_EMPTY_FS_LIST );
  ASSERT_OR_THROWEXCEPTION( ts.isValidType( e_listType ) );
  lowlevel::TyFSFeature headFeature = ts.getFeatureByBaseName(ne_listType, CAS::FEATURE_BASE_NAME_HEAD );
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature( headFeature ) );
  lowlevel::TyFSFeature tailFeature = ts.getFeatureByBaseName(ne_listType, CAS::FEATURE_BASE_NAME_TAIL );
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature( tailFeature ) );

  lowlevel::TyFS annot1 = heap.createFS(annotationType);
  ASSERT_OR_THROWEXCEPTION( heap.getType(annot1) == annotationType );
  heap.setIntValue(annot1, beginPosFeat, 42);
  ASSERT_OR_THROWEXCEPTION( heap.getIntValue(annot1, beginPosFeat) == 42 );
  heap.setIntValue(annot1, endPosFeat, 37);
  ASSERT_OR_THROWEXCEPTION( heap.getIntValue(annot1, endPosFeat) == 37 );


  lowlevel::TyFS lemma1 = heap.createFS(lemmaType);
  icu::UnicodeString uskey("lemmaKey");
  UnicodeStringRef ulsKey(uskey.getBuffer(), uskey.length());

//   lowlevel::TyFS stringFS = heap.createStringFeatureStructure(ulsKey);
  int ulsOff = heap.addString(ulsKey);
  heap.setStringValue(lemma1, keyFeature, ulsOff);
  UnicodeStringRef ulsKey1 = heap.getFSAsString( heap.getFSValue(lemma1, keyFeature) );
  ASSERT_OR_THROWEXCEPTION( ulsKey1 == ulsKey );

  lowlevel::TyFS lemmaList = heap.createFS( heap.getTypeSystem().getTypeByName(CAS::TYPE_NAME_FS_LIST) );


  lowlevel::TyFS tokenAnnot1 = heap.createFS(tokenAnnotationType);
  ASSERT_OR_THROWEXCEPTION( heap.getType(tokenAnnot1) == tokenAnnotationType );
  heap.setIntValue(tokenAnnot1, beginPosFeat, 43);
  ASSERT_OR_THROWEXCEPTION( heap.getIntValue(tokenAnnot1, beginPosFeat) == 43 );
  heap.setIntValue(tokenAnnot1, endPosFeat, -37);
  ASSERT_OR_THROWEXCEPTION( heap.getIntValue(tokenAnnot1, endPosFeat) == -37 );
  heap.setFSValue(tokenAnnot1, lemmaEntryFeat, lemmaList);
  ASSERT_OR_THROWEXCEPTION( heap.getFSValue(tokenAnnot1, lemmaEntryFeat) == lemmaList );

  lowlevel::TyFS tokenAnnot2 = heap.createFS(tokenAnnotationType);
  ASSERT_OR_THROWEXCEPTION( heap.getType(tokenAnnot2) == tokenAnnotationType );
  heap.setIntValue(tokenAnnot2, beginPosFeat, 0);
  ASSERT_OR_THROWEXCEPTION( heap.getIntValue(tokenAnnot2, beginPosFeat) == 0 );
  heap.setIntValue(tokenAnnot2, endPosFeat, 5);
  ASSERT_OR_THROWEXCEPTION( heap.getIntValue(tokenAnnot2, endPosFeat) == 5 );
  heap.setFSValue(tokenAnnot2, lemmaEntryFeat, lemmaList);
  ASSERT_OR_THROWEXCEPTION( heap.getFSValue(tokenAnnot2, lemmaEntryFeat) == lemmaList );

  LOG("testLowlevelFSHeap finished");
}


/************************************************************************************/

struct StrAnnotation {
  lowlevel::TyFSType type;
  int begin;
  int end;
};

struct StrToken {
  StrAnnotation annot;
  size_t lemma;
};


struct StrSentence {
  StrAnnotation annot;
};


struct StrLemma {
  lowlevel::TyFSType type;
  int keyIndex;
//   icu::UnicodeString* key;
  int pos;
};


void checkForwardOrder(lowlevel::FSHeap const & heap, auto_ptr<uima::lowlevel::IndexIterator> & it, uima::lowlevel::IndexComparator const * cpComp, vector<lowlevel::TyFS> const & fsVec) {
  size_t i=0;
  vector<lowlevel::TyFS> retrievedFSs;
  for (it->moveToFirst(); it->isValid(); it->moveToNext()) {
    LOG("  Found FS");
    retrievedFSs.push_back(it->get());
  }

  // check that FSs were retrieved in correct order
  LOG("retrieved FSs: " << retrievedFSs.size() << ", expected FSs: " << fsVec.size());
  ASSERT_OR_THROWEXCEPTION(retrievedFSs.size() == fsVec.size());

  for (i=0; i<retrievedFSs.size(); ++i) {
    if (retrievedFSs[i] != fsVec[i]) {
      // if the FSs are not token identical, they must be equal w.r.t. the comparator of the index
//         ASSERT_OR_THROWEXCEPTION( ix.getComparator()->compare( retrievedFSs[i], fsVec[i] ) == 0 );
      ASSERT_OR_THROWEXCEPTION( cpComp->compare(heap, retrievedFSs[i], fsVec[i] ) == 0 );
    }
  }

}


void checkReverseOrder(lowlevel::FSHeap const & heap,auto_ptr<uima::lowlevel::IndexIterator> & it, uima::lowlevel::IndexComparator const * cpComp, vector<lowlevel::TyFS> const & fsVec) {
  size_t i=0;
  vector<lowlevel::TyFS> retrievedFSs;
  for (it->moveToLast(); it->isValid(); it->moveToPrevious()) {
    LOG("  found FS");
    retrievedFSs.push_back(it->get());
  }
  LOG("retrieved FSs: " << retrievedFSs.size() << ", expected FSs: " << fsVec.size());
  ASSERT_OR_THROWEXCEPTION(retrievedFSs.size() == fsVec.size());
  for (i=0; i<retrievedFSs.size(); ++i) {
    size_t revi = fsVec.size() - i - 1;
    if (retrievedFSs[i] != fsVec[revi] ) {
      // if the FSs are not token identical, they must be equal w.r.t. the comparator of the index
      ASSERT_OR_THROWEXCEPTION( cpComp->compare(heap, retrievedFSs[i], fsVec[revi] ) == 0 );
    }
  }

}

void checkPreviousNextMovements(auto_ptr<uima::lowlevel::IndexIterator> & it, uima::lowlevel::IndexComparator const * cpComp, vector<lowlevel::TyFS> const & fsVec) {
  size_t i=0;
  vector<lowlevel::TyFS> retrievedFSs;
  for (it->moveToLast(); it->isValid(); it->moveToPrevious()) {
    LOG("  found FS");
    retrievedFSs.push_back(it->get());
  }
  LOG("retrieved FSs: " << retrievedFSs.size() << ", expected FSs: " << fsVec.size());
  ASSERT_OR_THROWEXCEPTION(retrievedFSs.size() == fsVec.size());

  size_t n = retrievedFSs.size();
  it->moveToFirst();
  for (i=0; i<n-1; ++i) {
    ASSERT_OR_THROWEXCEPTION(it->isValid());
    uima::lowlevel::TyFS current = it->get();
    LOG("move forward");
    it->moveToNext();
    ASSERT_OR_THROWEXCEPTION(it->isValid());
    //uima::lowlevel::TyFS next = it->get();
    LOG("move back again");
    it->moveToPrevious();
    ASSERT_OR_THROWEXCEPTION(it->isValid());
    LOG("checking");
    ASSERT_OR_THROWEXCEPTION( it->get() == current );
  }

}



void checkOrderedIndex(uima::lowlevel::IndexABase const & ixBase, vector<lowlevel::TyFS> const & fsVec) {
  lowlevel::FSHeap const & heap = ixBase.getFSHeap();
  bool bIsMaximalType = ixBase.getFSHeap().getTypeSystem().isMaximalType( ixBase.getType() );

  uima::lowlevel::IndexComparator const * cpComp = NULL;
  if (bIsMaximalType) {
    uima::lowlevel::internal::OrderedSingleIndex const * cpIXSingle = (uima::lowlevel::internal::OrderedSingleIndex const *) & ixBase;
    cpComp = cpIXSingle->getComparator();
  } else {
    uima::lowlevel::internal::OrderedCompositeIndex const * cpIXComposite = (uima::lowlevel::internal::OrderedCompositeIndex const *) & ixBase;
    cpComp = cpIXComposite->getComparator();

  }
  ASSERT_OR_THROWEXCEPTION( EXISTS(cpComp) );
  auto_ptr<lowlevel::IndexIterator> it(ixBase.createIterator());

  LOG("checking forward order");
  checkForwardOrder(heap, it, cpComp, fsVec);

  LOG("checking reverse order");
  checkReverseOrder(heap, it, cpComp, fsVec);

  LOG("checking forward order again");
  checkForwardOrder(heap, it, cpComp, fsVec);

  LOG("checking forward order again");
  checkForwardOrder(heap, it, cpComp, fsVec);

  LOG("checking reverse order again");
  checkReverseOrder(heap, it, cpComp, fsVec);


  LOG("checking moveToPrevious/moveToNext pairs");
  checkPreviousNextMovements(it, cpComp, fsVec);
}

void checkSetIndex(lowlevel::IndexABase const & ix, size_t num) {
  LOG("Checking set index");
  LOG("Index size: " << ix.getSize() );
  vector<lowlevel::TyFS> retrievedFSs;
  auto_ptr<lowlevel::IndexIterator> it(ix.createIterator());
  for (it->moveToFirst(); it->isValid(); it->moveToNext()) {
    retrievedFSs.push_back(it->get());
  }
  size_t i;
  for (i=0; i<retrievedFSs.size(); ++i) {
    LOG("FS retrieved: " << (int) retrievedFSs[i]);
  }
  LOG("Retrieved FSs: " << retrievedFSs.size() << ", expected FS: " << num);
  ASSERT_OR_THROWEXCEPTION( retrievedFSs.size() == num );
}

void testLowLevelIndex() {
  LOG("testLowlevelIndex started");

  TTDefinition ttdef;
  ttdef.init();
  ttdef.commitTypeSystem();
  lowlevel::TypeSystem & ts = ttdef.getTypeSystem();

  int i = 0;

  assert( ts.debugIsConsistent() );

  uima::lowlevel::IndexDefinition & ixDef = ttdef.getIndexDefinition();

  // get some types and features
  lowlevel::TyFSType annotType = ts.getTypeByName( CAS::TYPE_NAME_ANNOTATION );
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(annotType) );
  lowlevel::TyFSType lemmaType = ts.getTypeByName( TT::TYPE_NAME_LEMMA );
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(lemmaType) );
  lowlevel::TyFSType tokenAnnotType = ts.getTypeByName( TT::TYPE_NAME_TOKEN_ANNOTATION );
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(tokenAnnotType) );
  lowlevel::TyFSType sentAnnotType = ts.getTypeByName( TT::TYPE_NAME_SENTENCE_ANNOTATION );
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(sentAnnotType) );

  lowlevel::TyFSFeature beginPosFeat = ts.getFeatureByBaseName( annotType, CAS::FEATURE_BASE_NAME_BEGIN );
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(beginPosFeat) );
  lowlevel::TyFSFeature endPosFeat = ts.getFeatureByBaseName( annotType, CAS::FEATURE_BASE_NAME_END );
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(endPosFeat) );
  lowlevel::TyFSFeature keyFeat = ts.getFeatureByBaseName( lemmaType, TT::FEATURE_BASE_NAME_KEY );
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(keyFeat) );
  lowlevel::TyFSFeature posFeat = ts.getFeatureByBaseName( lemmaType, TT::FEATURE_BASE_NAME_PART_OF_SPEECH );
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(posFeat) );
  lowlevel::TyFSFeature lemmaEntryFeat = ts.getFeatureByBaseName( tokenAnnotType, TT::FEATURE_BASE_NAME_LEMMA_ENTRIES );
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(lemmaEntryFeat) );

  // prepare comparators
  vector<lowlevel::TyFSFeature> tokenKeyFeatures;
  vector<uima::lowlevel::IndexComparator::EnKeyFeatureComp> tokenComparators;
  tokenKeyFeatures.push_back(beginPosFeat);
  tokenKeyFeatures.push_back(endPosFeat);
  tokenComparators.resize(2, uima::lowlevel::IndexComparator::STANDARD_COMPARE );
//   lowlevel::IndexComparator annotComp(heap, annotType, keyFeatures);

  vector<lowlevel::TyFSFeature> lemmaKeyFeatures;
  vector<uima::lowlevel::IndexComparator::EnKeyFeatureComp> lemmaComparators;
  lemmaKeyFeatures.push_back( keyFeat );
  lemmaKeyFeatures.push_back( posFeat );
  lemmaComparators.resize(2, uima::lowlevel::IndexComparator::STANDARD_COMPARE );
//   lowlevel::IndexComparator lemmaComp(heap, lemmaType, keyFeatures);

  // create indexes
  const icu::UnicodeString ORDEREDIX = "42";
  const icu::UnicodeString SETIX = "37";
  const icu::UnicodeString FIFOIX = "4711";
  ixDef.defineIndex(lowlevel::IndexDefinition::enOrdered, annotType, tokenKeyFeatures, tokenComparators, ORDEREDIX);
  ASSERT_OR_THROWEXCEPTION( ixDef.getIndexKind(ORDEREDIX) == lowlevel::IndexDefinition::enOrdered );
//   ASSERT_OR_THROWEXCEPTION( ixStore.getComparator(ORDEREDIX) == & annotComp );

  ixDef.defineIndex(lowlevel::IndexDefinition::enSet, lemmaType, lemmaKeyFeatures, lemmaComparators, SETIX);
  ASSERT_OR_THROWEXCEPTION( ixDef.getIndexKind(SETIX) == lowlevel::IndexDefinition::enSet );
//   ASSERT_OR_THROWEXCEPTION( ixStore.getComparator(SETIX) == & lemmaComp );
//   ixStore.createFIFOIndex(annotType, FIFOIX);

  ttdef.commitIndexDefinition();

  // create some feature structures
//#define LEMMA_NUM 5
  const size_t DUPLICATE_LEMMAS = 1;
  vector<icu::UnicodeString> lemmaStrings;
  lemmaStrings.push_back("s1");
  lemmaStrings.push_back("s2");
  lemmaStrings.push_back("s3");
  struct StrLemma lemmasTobeCreated[] = {
                                          {
                                            lemmaType, 0, 1
                                          }
                                          , // 0
                                          { lemmaType, 0, 1}, // 1
                                          { lemmaType, 1, 1}, // 2
                                          { lemmaType, 0, 2}, // 3
                                          { lemmaType, 2, 3}, // 4
                                          /*
                                          { lemmaType, & lemmaStrings[0], 1}, // 0
                                          { lemmaType, & lemmaStrings[0], 1}, // 1
                                          { lemmaType, & lemmaStrings[1], 1}, // 2
                                          { lemmaType, & lemmaStrings[0], 2}, // 3
                                          { lemmaType, & lemmaStrings[2], 3}, // 4
                                          */
                                        };
  size_t LEMMA_NUM = NUMBEROF(lemmasTobeCreated);

  struct StrToken tokensToBeCreated[] = {
                                          {
                                            tokenAnnotType, 0, 5, 0
                                          }
                                          ,  // 0
                                          { tokenAnnotType, 5, 8, 1},  // 1
                                          { tokenAnnotType, 5, 8, 1},  // 2
                                        };
//#define TOKEN_NUM 3
  size_t TOKEN_NUM = NUMBEROF(tokensToBeCreated);
  assert( TOKEN_NUM == 3);

  struct StrSentence sentencesToBeCreated[] = {
        {
          sentAnnotType, 0, 8
        }
        ,  //0
      };
  size_t SENTENCE_NUM = NUMBEROF(sentencesToBeCreated);

  // define the orders of all annotations
  int annotationOrder[] = {
                          //      int annotationOrder[TOKEN_NUM + SENTENCE_NUM] = {
                            // sentence 0, token 0, token 1, token 2
                            0, SENTENCE_NUM + 0, SENTENCE_NUM + 1, SENTENCE_NUM + 2
                          };
  assert( NUMBEROF(annotationOrder) == SENTENCE_NUM + TOKEN_NUM );

  LOG("Token number: " << TOKEN_NUM);
  LOG("Lemma number: " << LEMMA_NUM);

//      uima::internal::TCASImpl tcasimpl(ttdef, 5000, 5000, 5000);
   uima::internal::CASImpl * pCas = getCASImpl(ttdef);
  uima::internal::CASImpl & tcasimpl = *pCas;

  uima::lowlevel::FSHeap & heap = tcasimpl.getHeap();
  uima::lowlevel::IndexRepository & ixStore = tcasimpl.getIndexRepository();

  vector<lowlevel::TyFS> tokens;
  vector<lowlevel::TyFS> sentences;
  vector<lowlevel::TyFS> lemmas;

  for (i=0; i<(int)LEMMA_NUM; ++i) {
    lowlevel::TyFS fs = heap.createFS( lemmasTobeCreated[i].type );
    icu::UnicodeString const & lemmaKey = lemmaStrings[ lemmasTobeCreated[i].keyIndex ];
    UnicodeStringRef key(lemmaKey );
    int keyOff = heap.addString(key);
    heap.setStringValue(fs, keyFeat, keyOff);
    heap.setIntValue(fs, posFeat, lemmasTobeCreated[i].pos);

    lowlevel::TyFS lemmaList = heap.createFS( heap.getTypeSystem().getTypeByName(CAS::TYPE_NAME_NON_EMPTY_FS_LIST) );
    lowlevel::TyFS eList = heap.createFS( heap.getTypeSystem().getTypeByName(CAS::TYPE_NAME_EMPTY_FS_LIST) );
    heap.setFSValue( lemmaList, heap.getTypeSystem().getFeatureByBaseName(CAS::TYPE_NAME_NON_EMPTY_FS_LIST, CAS::FEATURE_BASE_NAME_TAIL), eList );
    heap.setFSValue( lemmaList, heap.getTypeSystem().getFeatureByBaseName(CAS::TYPE_NAME_NON_EMPTY_FS_LIST, CAS::FEATURE_BASE_NAME_HEAD), fs );
    lemmas.push_back(lemmaList);
  }
  ASSERT_OR_THROWEXCEPTION( lemmas.size() == LEMMA_NUM );


  for (i=0; i<(int)TOKEN_NUM; ++i) {
    ASSERT_OR_THROWEXCEPTION( tokensToBeCreated[i].annot.type == tokenAnnotType );
    lowlevel::TyFS fs = heap.createFS( tokensToBeCreated[i].annot.type );
    heap.setIntValue(fs, beginPosFeat, tokensToBeCreated[i].annot.begin );
    heap.setIntValue(fs, endPosFeat, tokensToBeCreated[i].annot.end );
    heap.setFSValue(fs, lemmaEntryFeat, lemmas[ tokensToBeCreated[i].lemma ] );
    tokens.push_back(fs);
  }
  ASSERT_OR_THROWEXCEPTION( tokens.size() == TOKEN_NUM );

  for (i=0; i<(int)SENTENCE_NUM; ++i) {
    ASSERT_OR_THROWEXCEPTION( sentencesToBeCreated[i].annot.type == sentAnnotType );
    lowlevel::TyFS fs = heap.createFS( sentencesToBeCreated[i].annot.type );
    heap.setIntValue(fs, beginPosFeat, sentencesToBeCreated[i].annot.begin );
    heap.setIntValue(fs, endPosFeat, sentencesToBeCreated[i].annot.end );
    sentences.push_back(fs);
  }

  vector<lowlevel::TyFS> annotations;
  for (i=0; i<(int)(TOKEN_NUM + SENTENCE_NUM); ++i) {
    int fsix = annotationOrder[i];
    /*
    if (fsix < TOKEN_NUM) {
       annotations.push_back(tokens[fsix]);
    }
    else if (fsix < (TOKEN_NUM + SENTENCE_NUM) ) {
       annotations.push_back( sentences[fsix - TOKEN_NUM] );
    }
    */
    if (fsix < (int)SENTENCE_NUM) {
      annotations.push_back( sentences[fsix] );
    } else if (fsix < (int)(TOKEN_NUM + SENTENCE_NUM) ) {
      annotations.push_back(tokens[fsix - SENTENCE_NUM]);
    } else {
      ASSERT_OR_THROWEXCEPTION(false);
    }
  }

#ifndef NDEBUG
  heap.print(cout);
#endif

  // off we go...


  //Tests:
//   1. add tokens/lemmas to all indexes
//      ensure that tokens are returned in the right order
//      ensure that duplicates of lemmas are not inserted
  //

  // 1. add to all indexes
  LOG("Adding FSs in right order...");
  for (i=0; i<(int)tokens.size(); ++i) {
    LOG("Adding token " << (size_t) tokens[i]);
    LOG("  type of token: " << ts.getTypeName( heap.getType(tokens[i]) ) );
    ixStore.add(tokens[i]);
  }
  LOG("  tokens added");
  LOG("  Adding lemmas....");
  for (i=0; i<(int)lemmas.size(); ++i) {
    lowlevel::TyFS lemma = heap.getFSValue(lemmas[i], heap.getTypeSystem().getFeatureByBaseName(CAS::TYPE_NAME_NON_EMPTY_FS_LIST, CAS::FEATURE_BASE_NAME_HEAD) );
    ASSERT_OR_THROWEXCEPTION( heap.getType( lemma ) == heap.getTypeSystem().getTypeByName(TT::TYPE_NAME_LEMMA) );
    ixStore.add( lemma );
  }
  LOG("   " << lemmas.size() << " lemmas added");
  LOG("Lemmas in index:");
  {
    i=0;
    auto_ptr<uima::lowlevel::IndexIterator> it( ixStore.getLowlevelIndex(SETIX, lemmaType).createIterator() );
    for (it->moveToFirst(); it->isValid(); it->moveToNext() ) {
      LOG(" lemma number: " << i);
      uima::lowlevel::TyFS lemma = it->get();
      LOG("   key: " << heap.getStringValue(lemma, keyFeat));
      LOG("   pos: " << heap.getIntValue(lemma, posFeat) );
      ++i;
    }
  }
  LOG("FSs added");

  LOG("Check indexes...");
  // check ordered index
  checkOrderedIndex(ixStore.getLowlevelIndex(ORDEREDIX, tokenAnnotType), tokens);
  // check built-in annotation index
  checkOrderedIndex(ixStore.getLowlevelIndex( tcasimpl.getAnnotationIndexID(), tokenAnnotType), tokens);
  // check set index
  checkSetIndex(ixStore.getLowlevelIndex(SETIX, lemmaType), (LEMMA_NUM - DUPLICATE_LEMMAS) );

  LOG("  Indexes checked");

  ixStore.reset();
  // add in reverse order
  LOG("Adding FSs in reverse order...");
  for (i=tokens.size() - 1; i>=0; i--) {
    LOG("i: " << i );
    assert( heap.debugIsValidHeapCell(tokens[i]) );
    ixStore.add(tokens[i]);
  }
  for (i=lemmas.size() - 1; i>=0; --i) {
    lowlevel::TyFS lemma = heap.getFSValue(lemmas[i], heap.getTypeSystem().getFeatureByBaseName(CAS::TYPE_NAME_NON_EMPTY_FS_LIST, CAS::FEATURE_BASE_NAME_HEAD) );
    ASSERT_OR_THROWEXCEPTION( heap.getType( lemma ) == heap.getTypeSystem().getTypeByName(TT::TYPE_NAME_LEMMA) );
    ixStore.add( lemma );
  }
  LOG("Checking indexes");
  // check ordered index
  checkOrderedIndex(ixStore.getLowlevelIndex(ORDEREDIX, tokenAnnotType), tokens);
  // check built-in annotation index
  checkOrderedIndex(ixStore.getLowlevelIndex(tcasimpl.getAnnotationIndexID(),tokenAnnotType), tokens);
  // check set index
  checkSetIndex(ixStore.getLowlevelIndex(SETIX, lemmaType), (LEMMA_NUM - DUPLICATE_LEMMAS) );
  LOG("Indexes checked");

  // 2. add to a single index with id
  ixStore.reset();

  LOG("Add to single index with ID");
  for (i=0; i<(int)tokens.size(); ++i) {
    ixStore.add(tokens[i], ORDEREDIX);
  }
  LOG("Checking index");
  checkOrderedIndex(ixStore.getLowlevelIndex(ORDEREDIX, tokenAnnotType), tokens);
  ASSERT_OR_THROWEXCEPTION( ixStore.getLowlevelIndex( tcasimpl.getAnnotationIndexID(), tokenAnnotType).getSize() == 0 );
  LOG("Index checked");

  // 3. test composite indexes
  ixStore.reset();
  LOG("Add to composite index");
  for (i=0; i<(int)tokens.size(); ++i) {
    ixStore.add(tokens[i]);
  }
  for (i=0; i<(int)sentences.size(); ++i) {
    ixStore.add(sentences[i]);
  }

  lowlevel::IndexABase const & ix = ixStore.getLowlevelIndex( tcasimpl.getAnnotationIndexID(), annotType);
  LOG("Checking index");
  checkOrderedIndex(ix, annotations);
  LOG("Index checked");

  // 4. test delete index
  icu::UnicodeString anIxID = tcasimpl.getAnnotationIndexID();
  ixStore.remove(tokens[0]);
  auto_ptr<lowlevel::IndexIterator> it( ixStore.getLowlevelIndex( anIxID, tokenAnnotType).createIterator() );

  it->moveToFirst();
  ASSERT_OR_THROWEXCEPTION( it->get() != tokens[0]);
  ASSERT_OR_THROWEXCEPTION( it->get() == tokens[1]);

  i=1;
  while (it->isValid()) {
    ASSERT_OR_THROWEXCEPTION( it->get() == tokens[i] );
    it->moveToNext();
    ++i;
  }
  delete pCas;
  LOG("testLowlevelIndex finished");

}


/************************************************************************************/
void testEmptyStrings() {
  LOG("testEmptyStrings() started");
  TTDefinition ttdef;
  ttdef.init();

  lowlevel::TypeSystem& ts = ttdef.getTypeSystem();
  lowlevel::TyFSType t = ts.createType(ts.getTopType(), "t", ustrCreatorID);

  lowlevel::TyFSType floatType = ts.getTypeByName(CAS::TYPE_NAME_STRING);

  lowlevel::TyFSFeature f = ts.createFeature(t, floatType, false, "f", ustrCreatorID);

  ttdef.commit();

//      uima::internal::TCASImpl tcas(ttdef, 300, 300, 300);
   uima::internal::CASImpl * pCas = getCASImpl(ttdef);
  uima::internal::CASImpl & tcas = *pCas;

  uima::lowlevel::FSHeap& heap = tcas.getHeap();

  lowlevel::TyFS fs = heap.createFS(t);
  UnicodeStringRef uref = heap.getStringValue(fs, f);
  delete pCas;
  ASSERT_OR_THROWEXCEPTION( uref.getBuffer() == NULL );
  LOG("testEmptyStrings() finished");
}

/************************************************************************************/
void testFloatFeatures() {
  LOG("testFloatFeatures started");
  TTDefinition ttdef;
  ttdef.init();

  lowlevel::TypeSystem& ts = ttdef.getTypeSystem();
  lowlevel::TyFSType t = ts.createType(ts.getTopType(), "t", ustrCreatorID);


  lowlevel::TyFSType floatType = ts.getTypeByName(CAS::TYPE_NAME_FLOAT);

  lowlevel::TyFSFeature f = ts.createFeature(t, floatType, false, "f", ustrCreatorID);

  ttdef.commit();
//       uima::internal::TCASImpl tcas(ttdef, 30, 30, 30);
//       uima::internal::CASImpl tcas(ttdef, 30, 30, 30);
//        uima::internal::CASImpl & tcas = *uima::internal::CASImpl::createCASImpl(ttdef, false);

//     ErrorInfo errorInfo;
//     uima::internal::CASImpl & tcas = *(uima::internal::CASImpl*) Framework::createCAS(ttdef, errorInfo);
   uima::internal::CASImpl * pCas = getCASImpl(ttdef);
  uima::internal::CASImpl & tcas = *pCas;

  lowlevel::FSHeap& heap = tcas.getHeap();

  lowlevel::TyFS fs = heap.createFS(t);
  float x = 0.4237f;
  heap.setFloatValue(fs, f, x);
  
  float y = heap.getFloatValue(fs, f);
  ASSERT_OR_THROWEXCEPTION(x == y);
  delete pCas;
  LOG("testFloatFeatures finished");
}



/************************************************************************************/

void testNestedStructures() {
  LOG("testNestedStructures started");

  TTDefinition ttdef;
  ttdef.init();

  lowlevel::TypeSystem& ts = ttdef.getTypeSystem();
  lowlevel::TyFSType top = ts.getTopType();
  lowlevel::TyFSType t1 = ts.createType(top, "t1", ustrCreatorID);
  lowlevel::TyFSType t2 = ts.createType(top, "t2", ustrCreatorID);
  lowlevel::TyFSType t3 = ts.createType(top, "t3", ustrCreatorID);
  lowlevel::TyFSType t4 = ts.createType(top, "t4", ustrCreatorID);
  lowlevel::TyFSType t5 = ts.createType(top, "t5", ustrCreatorID);

  lowlevel::TyFSFeature f11 = ts.createFeature(t1, t2, false, "f11", ustrCreatorID);
  lowlevel::TyFSFeature f12 = ts.createFeature(t1, t3, false, "f12", ustrCreatorID);
  lowlevel::TyFSFeature f2 = ts.createFeature(t2, t4, false, "f2", ustrCreatorID);
  lowlevel::TyFSFeature f3 = ts.createFeature(t3, t5, false, "f3", ustrCreatorID);

  ttdef.commit();

//      uima::internal::TCASImpl tcas(ttdef, 400, 400, 400);
   uima::internal::CASImpl * pCas = getCASImpl(ttdef);
  uima::internal::CASImpl & tcas = *pCas;

//#ifndef NDEBUG
//   ts.print(cout);
//#endif

  lowlevel::FSHeap & heap = tcas.getHeap();

  lowlevel::TyFS fs_t1 = heap.createFS( t1 );
  lowlevel::TyFS fs_t2 = heap.createFS( t2 );
  lowlevel::TyFS fs_t3 = heap.createFS( t3 );
  lowlevel::TyFS fs_t4 = heap.createFS( t4 );
  lowlevel::TyFS fs_t5 = heap.createFS( t5 );

  ASSERT_OR_THROWEXCEPTION( heap.isUntouchedFSValue( fs_t1, f11 ) );
  heap.setFSValue(fs_t1, f11, fs_t2);
  ASSERT_OR_THROWEXCEPTION( ! heap.isUntouchedFSValue( fs_t1, f11 ) );
  heap.setFSValue(fs_t1, f12, fs_t3);
  heap.setFSValue(fs_t2, f2, fs_t4);
  heap.setFSValue(fs_t3, f3, fs_t5);

//#ifndef NDEBUG
//   heap.print(cout);
//#endif

  ASSERT_OR_THROWEXCEPTION( heap.getType( heap.getFSValue(fs_t1, f11) ) == t2 );
  ASSERT_OR_THROWEXCEPTION( heap.getType( heap.getFSValue(fs_t1, f12) ) == t3 );
  ASSERT_OR_THROWEXCEPTION( heap.getType( heap.getFSValue(fs_t2, f2) ) == t4 );
  ASSERT_OR_THROWEXCEPTION( heap.getType( heap.getFSValue(fs_t3, f3) ) == t5 );

  ASSERT_OR_THROWEXCEPTION( heap.getFSValue(fs_t1, f11) == fs_t2 );
  ASSERT_OR_THROWEXCEPTION( heap.getFSValue(fs_t1, f12) == fs_t3 );
  ASSERT_OR_THROWEXCEPTION( heap.getFSValue(fs_t2, f2) == fs_t4 );
  ASSERT_OR_THROWEXCEPTION( heap.getFSValue(fs_t3, f3) == fs_t5 );

  delete pCas;
  LOG("testNestedStructures finished");
}



/************************************************************************************/


void testOOIndex() {
  LOG("testOOIndex() started");
  TestCAS testcas;
  uima::internal::CASImpl & tcas = testcas.getCAS();
  uima::FSIndexRepository & indexRep = tcas.getIndexRepository();

  TypeSystem const & typeSys = tcas.getTypeSystem();

  Type annotType = typeSys.getType(TT::TYPE_NAME_TOKEN_ANNOTATION);
  Feature beginPosFeature = annotType.getFeatureByBaseName( CAS::FEATURE_BASE_NAME_BEGIN);
  Feature endPosFeature = annotType.getFeatureByBaseName(CAS::FEATURE_BASE_NAME_END);


  LOG("Creating FSs");
  FeatureStructure fs1 = tcas.createFS(annotType);
  ASSERT_OR_THROWEXCEPTION( fs1.isUntouchedFSValue(beginPosFeature) );
  fs1.setIntValue( beginPosFeature, 1 );
  ASSERT_OR_THROWEXCEPTION( ! fs1.isUntouchedFSValue(beginPosFeature) );
  fs1.setIntValue( endPosFeature, 2 );
  FeatureStructure fs2 = tcas.createFS(annotType);
  fs2.setIntValue( beginPosFeature, 2 );
  fs2.setIntValue( endPosFeature, 3 );
  FeatureStructure fs3 = tcas.createFS(annotType);
  fs3.setIntValue( beginPosFeature, 3 );
  fs3.setIntValue( endPosFeature, 4 );
  FeatureStructure fs4 = tcas.createFS(annotType);
  fs4.setIntValue( beginPosFeature, 4 );
  fs4.setIntValue( endPosFeature, 5 );
  FeatureStructure fs41 = tcas.createFS(annotType);
  fs41.setIntValue( beginPosFeature, 4 );
  fs41.setIntValue( endPosFeature, 5 );
  LOG("...FSs created");

  FeatureStructure fsArray[] = {
                                 fs1, fs2, fs3, fs4, fs41
                               };
  size_t fsArraySize = NUMBEROF(fsArray);

  LOG("Adding FSs to index");
  indexRep.addFS(fs1);
  indexRep.addFS(fs3);
  indexRep.addFS(fs41);
  indexRep.addFS(fs4);
  indexRep.addFS(fs2);
  LOG("...FSs Added");

  FSIndex ordIx = tcas.getAnnotationIndex(annotType);
  ASSERT_OR_THROWEXCEPTION( ordIx.getSize() == fsArraySize );

  LOG("Checking ordered index...");
  FSIterator ordIt = ordIx.iterator();
  size_t i=0;
  for (ordIt.moveToFirst(); ordIt.isValid(); ordIt.moveToNext()) {
    LOG("Checking " << i << "th FS");
    FeatureStructure fs1 = ordIt.get();
    FeatureStructure fs2 = fsArray[i];
//      ASSERT_OR_THROWEXCEPTION( ordIt.get() == fsArray[i] );
    ASSERT_OR_THROWEXCEPTION( fs1.getIntValue( beginPosFeature ) == fs2.getIntValue( beginPosFeature ) );
    ASSERT_OR_THROWEXCEPTION( fs1.getIntValue( endPosFeature ) == fs2.getIntValue( endPosFeature ) );
    ++i;
  }
  LOG("...Ordered index checked");


  LOG("Checking deletion");
  indexRep.removeFS(fsArray[0]);
  FSIterator ordIt2 = ordIx.iterator();
  ordIt2.moveToFirst();
  ASSERT_OR_THROWEXCEPTION( ! (ordIt2.get() == fsArray[0]) );
  ASSERT_OR_THROWEXCEPTION( ordIt2.get() == fsArray[1] );
  i=1;
  while (ordIt2.isValid()) {
    ASSERT_OR_THROWEXCEPTION( ordIt2.get() != fsArray[0] );
    ordIt2.moveToNext();
    ++i;
  }

  LOG("Deletion checked");


  LOG("testOOIndex() finished");
}




/************************************************************************************/

void testOOExceptions() {
  LOG("testOOExceptions() started");

  bool bExceptionThrown = false;

  Type t;
  try {
    UnicodeStringRef name = t.getName();
  } catch (InvalidFSTypeObjectException & exc) {
    LOG("Exception thrown correctly: " << exc.asString());
    bExceptionThrown = true;
  }

  ASSERT_OR_THROWEXCEPTION(bExceptionThrown);

  TTDefinition ttdef;
  ttdef.init();

  Type lemmaType = ttdef.getTypeSystem().getType(TT::TYPE_NAME_LEMMA);
  Type annotType = ttdef.getTypeSystem().getType(CAS::TYPE_NAME_ANNOTATION);
  Feature beginPosFeat = annotType.getFeatureByBaseName(CAS::FEATURE_BASE_NAME_BEGIN);
  Feature endPosFeat = annotType.getFeatureByBaseName(CAS::FEATURE_BASE_NAME_END);

  Type invalidType;
  Feature invalidFeat;


  ttdef.commit();
//   tcas.createPredefinedTTIndexes();

//      uima::internal::TCASImpl tcas(ttdef, 50000, 50000, 50000);
   uima::internal::CASImpl * pCas = getCASImpl(ttdef);
  uima::internal::CASImpl & tcas = *pCas;


  // check invalid fs
  FeatureStructure invalidFS;
  bExceptionThrown = false;
  try {
    (void)invalidFS.getIntValue(beginPosFeat);
  } catch (InvalidFSObjectException & exc) {
    LOG("Exception thrown correctly: " << exc.asString());
    bExceptionThrown = true;
  }
  ASSERT_OR_THROWEXCEPTION(bExceptionThrown);

  // check fs creation with invalid type
  bExceptionThrown = false;
  try {
    FeatureStructure fs1 = tcas.createFS(invalidType);
  } catch (InvalidFSTypeObjectException & exc) {
    LOG("Exception thrown correctly: " << exc.asString());
    bExceptionThrown = true;
  }
  ASSERT_OR_THROWEXCEPTION(bExceptionThrown);

  FeatureStructure annotFS = tcas.createFS(annotType);

  // check getting invalid feature
  // check getting inappropriate feature
  bExceptionThrown = false;
  try {
    FeatureStructure fs = annotFS.getFSValue(invalidFeat);
  } catch (InvalidFSFeatureObjectException & exc) {
    LOG("Exception thrown correctly: " << exc.asString());
    bExceptionThrown = true;
  }
  ASSERT_OR_THROWEXCEPTION(bExceptionThrown);

  // check getting inappropriate feature

  bExceptionThrown = false;
  try {
    FeatureStructure lemmaFS = tcas.createFS(lemmaType);
    FeatureStructure fs = lemmaFS.getFSValue(beginPosFeat);
  } catch (FeatureNotAppropriateException & exc) {
    LOG("Exception thrown correctly: " << exc.asString());
    bExceptionThrown = true;
  }
  ASSERT_OR_THROWEXCEPTION(bExceptionThrown);

  // getting an approproate feature with non builtin range type
  bExceptionThrown = false;
  try {
    FeatureStructure fs = annotFS.getFSValue(beginPosFeat);
  } catch (FeatureNotAppropriateException & exc) {
    LOG("Exception thrown correctly: " << exc.asString());
    bExceptionThrown = true;
  }
  ASSERT_OR_THROWEXCEPTION(bExceptionThrown);

  // setting incompatible value
  bExceptionThrown = false;
  try {
    annotFS.setFSValue(beginPosFeat, annotFS);
  } catch (IncompatibleValueTypeException & exc) {
    LOG("Exception thrown correctly: " << exc.asString());
    bExceptionThrown = true;
  }
  ASSERT_OR_THROWEXCEPTION(bExceptionThrown);

  /*
  // getting string/array value from fs
  bExceptionThrown = false;
  try {
     internal::StringFS strFS(annotFS);
     UnicodeStringRef p = strFS.getStringValue();
  }
  catch (FSIsNotStringException & exc) {
     LOG("Exception thrown correctly: " << exc.asString());
     bExceptionThrown = true;
  }
  ASSERT_OR_THROWEXCEPTION(bExceptionThrown);
  */

  // invalid arrays
  ArrayFS invalidArrayFS(invalidFS);
  bExceptionThrown = false;
  try {
    size_t i = invalidArrayFS.size();
  } catch (InvalidFSObjectException & exc) {
    LOG("Exception thrown correctly: " << exc.asString());
    bExceptionThrown = true;
  }
  ASSERT_OR_THROWEXCEPTION(bExceptionThrown);

  // array size
  bExceptionThrown = false;
  try {
    ArrayFS arrFS(annotFS);
    (void)arrFS.size();
  } catch (FSIsNotArrayException & exc) {
    LOG("Exception thrown correctly: " << exc.asString());
    bExceptionThrown = true;
  }
  ASSERT_OR_THROWEXCEPTION(bExceptionThrown);


  bExceptionThrown = false;
  LOG("Testing InvalidIndexIDException");
  try {
    tcas.getIndexRepository().getIndex("bla", annotType);
  } catch (InvalidIndexIDException & exc) {
    LOG("Exception thrown correctly: " << exc.asString());
    bExceptionThrown = true;
  }
  ASSERT_OR_THROWEXCEPTION(bExceptionThrown);


  bExceptionThrown = false;
  LOG("Testing WrongFSTypeForIndexException");
  icu::UnicodeString const & crIndexID = tcas.getAnnotationIndexID();
  try {
    tcas.getIndexRepository().getIndex(crIndexID, lemmaType);
  } catch (WrongFSTypeForIndexException & exc) {
    LOG("Exception thrown correctly: " << exc.asString());
    bExceptionThrown = true;
  }
  ASSERT_OR_THROWEXCEPTION(bExceptionThrown);

  bExceptionThrown = false;
  LOG("Testing CouldNotCreateFSOfFinalTypeException");
  try {
    tcas.createFS( tcas.getTypeSystem().getType( uima::CAS::TYPE_NAME_INTEGER ) );
  } catch ( CouldNotCreateFSOfFinalTypeException & exc) {
    LOG("Exception thrown correctly: " << exc.asString());
    bExceptionThrown = true;
  }
  delete pCas;
  LOG("testOOExceptions() finished");
}


/************************************************************************************/


void testOOFilterBuilder() {
  LOG("testOOFilterBuilder() started");

  TTDefinition ttdef;
  ttdef.init();
  


  TypeSystem const & typeSystem  = ttdef.getTypeSystem();

  Type tokenType = typeSystem.getType(TT::TYPE_NAME_TOKEN_ANNOTATION);
  Feature beginFeat = tokenType.getFeatureByBaseName( CAS::FEATURE_BASE_NAME_BEGIN);
  Feature endFeat = tokenType.getFeatureByBaseName( CAS::FEATURE_BASE_NAME_END);

  Type stringType = typeSystem.getType(CAS::TYPE_NAME_STRING);


  uima::lowlevel::TypeSystem & lolTS = ttdef.getTypeSystem();
  lowlevel::TyFSType lolTokenType = uima::internal::FSPromoter::demoteType( tokenType );
  lowlevel::TyFSType lolMyTokenType = lolTS.createType(lolTokenType, "MyToken", ustrCreatorID);
  lolTS.createFeature(lolMyTokenType, lolTokenType, false, "MyRecursiveFeature", ustrCreatorID);
  lolTS.createFeature(lolMyTokenType, uima::internal::FSPromoter::demoteType(stringType), false, "MyStringFeature", ustrCreatorID);


  Type myTokenType = typeSystem.getType("MyToken");
  Feature myRecursiveFeature = myTokenType.getFeatureByBaseName("MyRecursiveFeature");
  Feature myStringFeature = myTokenType.getFeatureByBaseName("MyStringFeature");


//   lowlevel::TypeSystem const & typeSystem = tcas.getFSSystem().getLowlevelTypeSystem();
//   lowlevel::TyFSType lolTokenType = typeSystem.getType(CAS::TYPE_NAME_TOKEN_ANNOTATION);
//   lowlevel::TyFSType lolMyTokenType = typeSystem.getType("MyToken");

  ttdef.commit();
//      uima::internal::TCASImpl tcas(ttdef, 4000, 4000, 4000);
   uima::internal::CASImpl * pCas = getCASImpl(ttdef);
  uima::internal::CASImpl & tcas = *pCas;

  uima::FSIndexRepository & indexRep = tcas.getIndexRepository();

  StrToken tokens[] = {
                        {
                          lolTokenType, 0, 3, 0
                        }
                        , // 0
                        { lolMyTokenType, 3, 5, 0}, // 1
                        { lolTokenType, 5, 8, 0}, // 2
                        { lolMyTokenType, 9, 11, 0}, // 3
                        { lolMyTokenType, 15, 16, 0} // 4
                      };
  size_t tokenNum = NUMBEROF(tokens);
  size_t myTokenNum = 3;

  FeatureStructure oldFS;
  size_t i;
  for (i=0; i<tokenNum; ++i) {
    icu::UnicodeString typeName = lolTS.getTypeName( tokens[i].annot.type );
    Type type = typeSystem.getType(typeName);

    FeatureStructure fs = tcas.createFS(type);
    fs.setIntValue(beginFeat, tokens[i].annot.begin);
    fs.setIntValue(endFeat, tokens[i].annot.end);
    if (type == myTokenType) {
      ASSERT_OR_THROWEXCEPTION(oldFS.isValid() );
      fs.setFSValue(myRecursiveFeature, oldFS);
      char c[32];
      sprintf(c,"%d",i);                  //cosItoa(i, c);

      icu::UnicodeString us("string");
      us.append(icu::UnicodeString(c));
//         icu::UnicodeString icuus( (UChar*) us.data(), us.length() );
//         UnicodeStringRef p = fsSystem.addString(icuus);
//         UnicodeStringRef lstr( (UniChar const *) p.first, p.second);
//         LOG("Adding string '" << lstr << "'");
//         FeatureStructure stringFS = fsSystem.createStringFS(p.first, p.second);
      fs.setStringValue(myStringFeature, us);
    }
    oldFS = fs;
    indexRep.addFS(fs);
  }
  LOG("Annotations created");

  FSIterator ixit;
  ANIndex ix = tcas.getAnnotationIndex(tokenType);

  vector<Feature> vFeaturePath;
  FSFilterBuilder const & builder = tcas.getFSFilterBuilder();

  //////////////////////////////////////////
  LOG("Checking type filter");

  //TODO: the filter passed to ix.filteredIterator is wrapped in another object,
  // added to ixit, and never deleted.
  // LEAK!!!

  FSFilter * typeFilter = builder.createTypeFilter(tokenType, false);
  ixit = ix.filteredIterator(typeFilter);
  i=0;
  for (ixit.moveToFirst(); ixit.isValid(); ixit.moveToNext()) {
    ASSERT_OR_THROWEXCEPTION( ixit.get().getType() == myTokenType );
    ++i;
  }
  ASSERT_OR_THROWEXCEPTION(i == myTokenNum);

  delete typeFilter;
  typeFilter = NULL;
  LOG("...type filter checked");

  //////////////////////////////////////////
  LOG("Checking AND/int feature value filter");
  vFeaturePath.clear();
  vFeaturePath.push_back(beginFeat);
  FSFilter * intFeatureFilter1 = builder.createIntFeatureFilter(vFeaturePath, FSFilterBuilder::EQUALS, 5);
  vFeaturePath.clear();
  vFeaturePath.push_back(endFeat);
  FSFilter * intFeatureFilter2 = builder.createIntFeatureFilter(vFeaturePath, FSFilterBuilder::LESS, 10);
  FSFilter * conjFeatureFilter = builder.createANDFilter(intFeatureFilter1, intFeatureFilter2);

  ixit = ix.filteredIterator(conjFeatureFilter);
  i=0;
  for (ixit.moveToFirst(); ixit.isValid(); ixit.moveToNext()) {
    FeatureStructure fs = ixit.get();
    ASSERT_OR_THROWEXCEPTION( !( (fs.getIntValue(beginFeat) == 5) && (fs.getIntValue(endFeat) < 10) ) );
    ++i;
  }
  LOG(" Found " << i << " filtered FSs");
  ASSERT_OR_THROWEXCEPTION( i == 4 );

  LOG("... AND/int feature value filter checked");

  /////////////////////////////////////////////
  LOG("Checking NOT filter");
  FSFilter * notFilter = builder.createNOTFilter(conjFeatureFilter);

  ixit = ix.filteredIterator(notFilter);
  i=0;
  for (ixit.moveToFirst(); ixit.isValid(); ixit.moveToNext()) {
    FeatureStructure fs = ixit.get();
    ASSERT_OR_THROWEXCEPTION( ( (fs.getIntValue(beginFeat) == 5) && (fs.getIntValue(endFeat) < 10) ) );
    ++i;
  }
  LOG(" Found " << i << " filtered FSs");
  ASSERT_OR_THROWEXCEPTION( i == 1 );
  notFilter->deInit();
  delete notFilter;
  notFilter = NULL;
  LOG("... NOT filter checked");

  ////////////////////////////////////////////////////

  LOG("Checking match filter");
  FSFilter * myTokenTypeFilter1 = builder.createTypeFilter(myTokenType, true);
  FSFilter * myTokenTypeFilter2 = builder.createTypeFilter(myTokenType, true);
  vFeaturePath.clear();
  vFeaturePath.push_back(myRecursiveFeature);
  FSFilter * matchFilter = builder.createMatchFilter(vFeaturePath, myTokenTypeFilter2);
  FSFilter * andFilter = builder.createANDFilter(myTokenTypeFilter1, matchFilter);

  LOG("Creating iterator");
  ixit = ix.filteredIterator(andFilter);
  LOG("... Iterator created");

  i=0;
  for (ixit.moveToFirst(); ixit.isValid(); ixit.moveToNext()) {
    FeatureStructure fs = ixit.get();
    ASSERT_OR_THROWEXCEPTION( tokenType.subsumes( fs.getType() ));
    ++i;
  }
  LOG("Found " << i << " FSs");
  ASSERT_OR_THROWEXCEPTION(i==4);

  andFilter->deInit();
  delete andFilter;
  LOG("... match filter checked");

  //////////////////////////////////////////////////////
  LOG("Checking string filter");
  FSFilter * myTokenTypeFilter3 = builder.createTypeFilter(myTokenType, true);

  vFeaturePath.clear();
  vFeaturePath.push_back(myStringFeature);

  icu::UnicodeString usMatch("string3");
  FSFilter * stringFilter = builder.createStringFeatureFilter(vFeaturePath, usMatch.getBuffer(), usMatch.length() );
//   FSFilter * matchFilter2 = builder.createMatchFilter(vFeaturePath, stringFilter);
  FSFilter * andFilter2 = builder.createANDFilter(myTokenTypeFilter3, stringFilter);

  LOG("Creating iterator");
  ixit = ix.filteredIterator(andFilter2);
  LOG("... Iterator created");

  i=0;
  for (ixit.moveToFirst(); ixit.isValid(); ixit.moveToNext()) {
    FeatureStructure fs = ixit.get();
    if (fs.getType() == myTokenType) {
      UnicodeStringRef p = fs.getStringValue(myStringFeature);
      icu::UnicodeString str( p.getBuffer(), p.length() );
      LOG(" found string: " << str);
      ASSERT_OR_THROWEXCEPTION( ! (str == usMatch ));
    }
    ++i;
  }
  LOG("Found " << i << " FSs");
  ASSERT_OR_THROWEXCEPTION(i == (tokenNum - 1));
  andFilter2->deInit();
  delete andFilter2;
  delete pCas;

  LOG("... string filter checked");

  LOG("testOOFilterBuilder() finished");
}



/************************************************************************************/

void testIteratorSetToPosition() {
  LOG("testIteratorSetToPosition() started");
  TestCAS testcas;
  uima::internal::CASImpl & tcas = testcas.getCAS();

  lowlevel::TypeSystem const & typeSystem = testcas.getCASDef().getTypeSystem();
  lowlevel::TyFSType tokenType = typeSystem.getTypeByName(TT::TYPE_NAME_TOKEN_ANNOTATION);
  lowlevel::TyFSFeature beginFeature = typeSystem.getFeatureByBaseName(tokenType, CAS::FEATURE_BASE_NAME_BEGIN);
  lowlevel::TyFSFeature endFeature = typeSystem.getFeatureByBaseName(tokenType, CAS::FEATURE_BASE_NAME_END);

  lowlevel::FSHeap & fsHeap = tcas.getHeap();

  StrToken tokens[] = {
                        {
                          tokenType, 0, 3, 0
                        }
                        , // 0
                        { tokenType, 3, 5, 0}, // 1
                        { tokenType, 5, 8, 0}, // 2
                        { tokenType, 9, 11, 0}, // 3
                        { tokenType, 15, 16, 0}, // 4
                        { tokenType, 17, 20, 0} // 5
                      };
  size_t tokenNum = NUMBEROF(tokens);

  lowlevel::IndexRepository & indexRep = tcas.getIndexRepository();

  size_t i;
  for (i=0; i<tokenNum; ++i) {
    lowlevel::TyFS token = fsHeap.createFS(tokens[i].annot.type);
    fsHeap.setIntValue(token, beginFeature, tokens[i].annot.begin);
    fsHeap.setIntValue(token, endFeature, tokens[i].annot.end);
    indexRep.add(token);
  }

  icu::UnicodeString const & ixID = tcas.getAnnotationIndexID();

  lowlevel::IndexABase const & ix = indexRep.getLowlevelIndex(ixID, tokenType);

  auto_ptr<lowlevel::IndexIterator> it1(ix.createIterator() );
  auto_ptr<lowlevel::IndexIterator> it2(ix.createIterator());

  const size_t numberOfAdvances = 2;
  it1->moveToFirst();

  for (i=0; i<numberOfAdvances; ++i) {
    ASSERT_OR_THROWEXCEPTION( it1->isValid() );
    it1->moveToNext();
  }

  ASSERT_OR_THROWEXCEPTION( it1->isValid() );

  LOG("Setting it2 to point to token from " << fsHeap.getIntValue(it1->get(), beginFeature) << " to " << fsHeap.getIntValue(it1->get(), endFeature) );
  bool bCouldBeSet = it2->moveTo(it1->get());

  ASSERT_OR_THROWEXCEPTION(bCouldBeSet);
  ASSERT_OR_THROWEXCEPTION( it2->isValid() );
  ASSERT_OR_THROWEXCEPTION( it2->get() == it1->get() );

  // check that rest tokens are also found
  for (i=numberOfAdvances; i<tokenNum; ++i) {
    ASSERT_OR_THROWEXCEPTION( it1->isValid() );
    ASSERT_OR_THROWEXCEPTION( it2->isValid() );
    LOG("Checking token from " << fsHeap.getIntValue(it1->get(), beginFeature) << " to " << fsHeap.getIntValue(it1->get(), endFeature) );
    ASSERT_OR_THROWEXCEPTION( it1->get() == it2->get() );
    it1->moveToNext();
    it2->moveToNext();
  }
  ASSERT_OR_THROWEXCEPTION( ! it1->isValid() );
  ASSERT_OR_THROWEXCEPTION( ! it2->isValid() );


  LOG("testIteratorSetToPosition() finished");
}



/************************************************************************************/


class MyLowlevelFilter : public lowlevel::FSFilter {
private:
  uima::internal::CASImpl const & iv_tcas;
  lowlevel::TyFSType annotType;
  lowlevel::TyFSFeature beginFeat;
public:
  MyLowlevelFilter(uima::internal::CASImpl const & tcas)
      : iv_tcas(tcas) {
    annotType = tcas.getHeap().getTypeSystem().getTypeByName(CAS::TYPE_NAME_ANNOTATION);
    beginFeat = tcas.getHeap().getTypeSystem().getFeatureByBaseName( annotType, CAS::FEATURE_BASE_NAME_BEGIN);
  }

  // filter those annotations whose begin position is less than 10
  bool isFiltered(lowlevel::TyFS fs) const {
    lowlevel::FSHeap const & crFSHeap = iv_tcas.getHeap();
    lowlevel::TypeSystem const & crTypeSystem = crFSHeap.getTypeSystem();

    // filter only annotations
    if (! crTypeSystem.subsumes( annotType, crFSHeap.getType(fs)) ) {
      return false;
    }
    int iBegin = crFSHeap.getIntValue(fs, beginFeat);
    return(iBegin < 10);
  }

};

void testFilters() {
  LOG("testFilters() started");
  TestCAS testcas;
  uima::internal::CASImpl & tcas = testcas.getCAS();

  lowlevel::TypeSystem const & typeSystem = testcas.getCASDef().getTypeSystem();
  lowlevel::TyFSType tokenType = typeSystem.getTypeByName(TT::TYPE_NAME_TOKEN_ANNOTATION);
  lowlevel::TyFSFeature beginFeature = typeSystem.getFeatureByBaseName(tokenType, CAS::FEATURE_BASE_NAME_BEGIN);
  lowlevel::TyFSFeature endFeature = typeSystem.getFeatureByBaseName(tokenType, CAS::FEATURE_BASE_NAME_END);

  lowlevel::FSHeap & fsHeap = tcas.getHeap();

  StrToken tokens[] = {
                        {
                          tokenType, 0, 3, 0
                        }
                        , // 0
                        { tokenType, 3, 5, 0}, // 1
                        { tokenType, 5, 8, 0}, // 2
                        { tokenType, 9, 11, 0}, // 3
                        { tokenType, 15, 16, 0}, // 4
                        { tokenType, 17, 20, 0} // 5
                      };
  size_t tokenNum = NUMBEROF(tokens);

  lowlevel::IndexRepository & indexRep = tcas.getIndexRepository();

  size_t i;
  for (i=0; i<tokenNum; ++i) {
    lowlevel::TyFS token = fsHeap.createFS(tokens[i].annot.type);
    fsHeap.setIntValue(token, beginFeature, tokens[i].annot.begin);
    fsHeap.setIntValue(token, endFeature, tokens[i].annot.end);
    indexRep.add(token);
  }

  icu::UnicodeString const & ixID = tcas.getAnnotationIndexID();

  lowlevel::IndexABase const & ix = indexRep.getLowlevelIndex(ixID, tokenType);

  LOG("Annotations created");

  MyLowlevelFilter* myFilter = new MyLowlevelFilter(tcas);

  lowlevel::IndexIterator* it = ix.createFilteredIterator(myFilter);
  LOG("filtered iterator created");

  it->moveToFirst();
  LOG("Checking first annotation");
  ASSERT_OR_THROWEXCEPTION( it->isValid() );
  lowlevel::TyFS fs = it->get();
  ASSERT_OR_THROWEXCEPTION( fsHeap.getIntValue(fs, beginFeature) == 15 );
  it->moveToNext();

  ASSERT_OR_THROWEXCEPTION( it->isValid() );
  fs = it->get();
  ASSERT_OR_THROWEXCEPTION( fsHeap.getIntValue(fs, beginFeature) == 17 );
  it->moveToNext();
  ASSERT_OR_THROWEXCEPTION( ! it->isValid() );

  delete it;
  delete myFilter;
  LOG("testFilters() finished");
}


/************************************************************************************/

/*
class PermanentComparator : public lowlevel::IndexComparator {
public:
   PermanentComparator(lowlevel::FSHeap const & heap, lowlevel::TyFSType type)
   : lowlevel::IndexComparator(heap, type) {
   }

   int compare(lowlevel::TyFS, lowlevel::TyFS) const {
      return 1;
   }
};
*/
void testPermanentStructures() {
  LOG("testPermanentStructures() skipped!!!");
#ifdef BYEBYEPERM
  LOG("testPermanentStructures() started");
  TTDefinition ttdef;
  ttdef.init();

  ttdef.commitTypeSystem();
  uima::lowlevel::IndexDefinition & indexDef = ttdef.getIndexDefinition();

  lowlevel::TypeSystem const & typeSystem = ttdef.getTypeSystem();
  lowlevel::TyFSType tokenType = typeSystem.getTypeByName(TT::TYPE_NAME_TOKEN_ANNOTATION);
  lowlevel::TyFSFeature beginFeature = typeSystem.getFeatureByBaseName(tokenType, CAS::FEATURE_BASE_NAME_BEGIN);
  lowlevel::TyFSFeature endFeature = typeSystem.getFeatureByBaseName(tokenType, CAS::FEATURE_BASE_NAME_END);

  lowlevel::TyFeatureOffset beginFeatOffset = typeSystem.getFeatureOffset(beginFeature);
  lowlevel::TyFeatureOffset endFeatOffset = typeSystem.getFeatureOffset(endFeature);



  vector<lowlevel::TyFSFeature> tokenKeyFeatures;
  vector<uima::lowlevel::IndexComparator::EnKeyFeatureComp> tokenComparators;
  tokenKeyFeatures.push_back(beginFeature);
  tokenKeyFeatures.push_back(endFeature);
  tokenComparators.resize(2, uima::lowlevel::IndexComparator::STANDARD_COMPARE );

//   PermanentComparator* permComp = new PermanentComparator(heap, tokenType);
  indexDef.defineIndex(uima::lowlevel::IndexDefinition::enSet,
                       tokenType,
                       tokenKeyFeatures,
                       tokenComparators,
                       "2001",
                       true );
  ASSERT_OR_THROWEXCEPTION( indexDef.getIndexKind("2001") == uima::lowlevel::IndexDefinition::enSet );

  ttdef.commitIndexDefinition();

//      uima::internal::TCASImpl tcas(ttdef, 4000, 4000, 4000);
  uima::internal::CASImpl & tcas = getCASImpl(ttdef);

  icu::UnicodeString const & annotationIndexID = tcas.getAnnotationIndexID();

  lowlevel::FSHeap & heap = tcas.getHeap();
  lowlevel::IndexRepository & indexRep = tcas.getIndexRepository();

  lowlevel::TyFS permFS = heap.createFS( lowlevel::FSHeap::PERMANENT, tokenType );
  heap.setIntValue(permFS, beginFeature, 1);
  heap.setIntValue(permFS, endFeature, 2);

  lowlevel::TyFS tempFS1 = heap.createFS( lowlevel::FSHeap::TEMPORARY, tokenType );
  heap.setIntValue(tempFS1, beginFeature, 3);
  heap.setIntValue(tempFS1, endFeature, 4);

  ASSERT_OR_THROWEXCEPTION( permFS != lowlevel::FSHeap::INVALID_FS );
  ASSERT_OR_THROWEXCEPTION( tempFS1 != lowlevel::FSHeap::INVALID_FS );

  indexRep.add(tempFS1);

  lowlevel::IndexABase const & index = indexRep.getLowlevelIndex( annotationIndexID, tokenType );
  lowlevel::IndexABase const & permIndex = indexRep.getLowlevelIndex( "2001", tokenType);

  ASSERT_OR_THROWEXCEPTION( index.getSize() == 1 );
  ASSERT_OR_THROWEXCEPTION( permIndex.getSize() == 1 );
  LOG("FSs created");


  tcas.resetWithoutCreatingDocAnnotation();

  LOG("TCAS reset'd");

  ASSERT_OR_THROWEXCEPTION( index.getSize() == 0 );
  ASSERT_OR_THROWEXCEPTION( permIndex.getSize() == 1 );

  ASSERT_OR_THROWEXCEPTION( *tempFS1 == 0 );
  ASSERT_OR_THROWEXCEPTION( *(tempFS1 + beginFeatOffset) == 0 );
  ASSERT_OR_THROWEXCEPTION( *(tempFS1 + endFeatOffset) == 0 );

  LOG("temp fs checked");

  ASSERT_OR_THROWEXCEPTION( heap.getType(permFS) == tokenType );
  ASSERT_OR_THROWEXCEPTION( (size_t) heap.getFeatureWithOffset(permFS, beginFeatOffset) == 1 );
  ASSERT_OR_THROWEXCEPTION( (size_t) heap.getFeatureWithOffset(permFS, endFeatOffset) == 2 );

  LOG("perm fs checked");
//   delete permComp;
  LOG("testPermanentStructures() finished");
#endif
}


/************************************************************************************/

/*
// obsolete since IDs are now strings.
void testNewIndexID() {
   LOG("testNewIndexID() started");
   uima::internal::TCASImpl tcas(4000);

   tcas.commitTypeSystem();

   uima::lowlevel::TypeSystem & ts = tcas.getFSSystem().getLowlevelTypeSystem();

   // get some types and features
   lowlevel::TyFSType annotType = ts.getTypeByName( CAS::TYPE_NAME_ANNOTATION );
   ASSERT_OR_THROWEXCEPTION( ts.isValidType(annotType) );

   lowlevel::TyFSFeature beginPosFeat = ts.getFeatureByBaseName( CAS::FEATURE_BASE_NAME_BEGIN );
   ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(beginPosFeat) );
   lowlevel::TyFSFeature endPosFeat = ts.getFeatureByBaseName( CAS::FEATURE_BASE_NAME_END );
   ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(endPosFeat) );

   vector<lowlevel::TyFSFeature> keyFeatures;
   keyFeatures.push_back(beginPosFeat);
   keyFeatures.push_back(endPosFeat);
   lowlevel::KeyFeatureComparator annotComp(tcas.getFSSystem().getLowlevelFSHeap(), annotType, keyFeatures);

   uima::lowlevel::IndexRepository & ixRep = tcas.getFSSystem().getLowlevelIndexRepository();

   int id1 = ixRep.getNewIndexID();
   LOG("New index ID: " << id1);
   ASSERT_OR_THROWEXCEPTION( ! ixRep.isValidIndexId(id1) );
   ixRep.createOrderedIndex(annotType, & annotComp, id1);

   int id2 = ixRep.getNewIndexID();
   ASSERT_OR_THROWEXCEPTION( ! ixRep.isValidIndexId(id2) );
   ASSERT_OR_THROWEXCEPTION( id1 != id2 );


   LOG("testNewIndexID() finished");
}
*/

/************************************************************************************/
void testOOArrayFS() {
  LOG("testOOArrayFS() started");
  TestCAS testcas;
  uima::internal::CASImpl & tcas  = testcas.getCAS();

  FeatureStructure strFS1 = tcas.createListFS();
  FeatureStructure strFS2 = tcas.createListFS();
  FeatureStructure strFS3 = tcas.createListFS();
  FeatureStructure strFS4 = tcas.createListFS();
  FeatureStructure strFS5 = tcas.createListFS();

  FeatureStructure strings[] = {
                                 strFS1, strFS3, strFS2, strFS5, strFS4
                               };

  const size_t length = NUMBEROF(strings);
  size_t i=0;

  ArrayFS fs = tcas.createArrayFS(length);
  ASSERT_OR_THROWEXCEPTION( fs.size() == length );

  for (i=0; i<length; ++i) {
    fs.set(i, strings[i]);
  }

  for (i=0; i<length; ++i) {
    ASSERT_OR_THROWEXCEPTION( fs.get(i) == strings[i] );
  }

  LOG("testOOArrayFS() finished");
}

/************************************************************************************/
void testOOStringArrayFS() {
  LOG("testOOStringArray() started");
  TestCAS testcas;
  uima::internal::CASImpl & tcas  = testcas.getCAS();

  icu::UnicodeString str1("string1");
  icu::UnicodeString str2("string2");
  icu::UnicodeString str3("string3");
  icu::UnicodeString str4("string4");
  icu::UnicodeString str5("string5");

  UnicodeStringRef strings[] = {
                                 str1, str3, str2, str5, str4
                               };

  const size_t length = NUMBEROF(strings);
  size_t i=0;

  StringArrayFS fs = tcas.createStringArrayFS(length);
  ASSERT_OR_THROWEXCEPTION( fs.size() == length );

  for (i=0; i<length; ++i) {
    fs.set(i, strings[i]);
  }

  for (i=0; i<length; ++i) {
    ASSERT_OR_THROWEXCEPTION( fs.get(i) == strings[i] );
  }

  LOG("testOOStringArrayFS() finished");
}


/************************************************************************************/

void testOOListFunctionsOld() {
  LOG("testOOListFunctions() started");
  TestCAS testcas;
  uima::internal::CASImpl & tcas = testcas.getCAS();

  TypeSystem const & typeSystem = tcas.getTypeSystem();

  Type tokenAnnType = typeSystem.getType(TT::TYPE_NAME_TOKEN_ANNOTATION);
  Feature tokenNbrFeat = tokenAnnType.getFeatureByBaseName(TT::FEATURE_BASE_NAME_TOKEN_NUMBER);

  // create tokens
  FeatureStructure tok0 = tcas.createFS(tokenAnnType);
  tok0.setIntValue(tokenNbrFeat, 0);

  FeatureStructure tok1 = tcas.createFS(tokenAnnType);
  tok1.setIntValue(tokenNbrFeat, 1);

  FeatureStructure tok2 = tcas.createFS(tokenAnnType);
  tok2.setIntValue(tokenNbrFeat, 2);

  FeatureStructure tok3 = tcas.createFS(tokenAnnType);
  tok3.setIntValue(tokenNbrFeat, 3);

  FeatureStructure tok4 = tcas.createFS(tokenAnnType);
  tok4.setIntValue(tokenNbrFeat, 4);

  FeatureStructure tok5 = tcas.createFS(tokenAnnType);
  tok5.setIntValue(tokenNbrFeat, 5);

  FeatureStructure tok6 = tcas.createFS(tokenAnnType);
  tok6.setIntValue(tokenNbrFeat, 6);

  FeatureStructure tokens[] = {
                                tok0, tok1, tok2, tok3, tok4, tok5, tok6
                              };
  size_t tokNum = NUMBEROF(tokens);
  ASSERT_OR_THROWEXCEPTION( tokNum == 7);

  ListFS tokList = tcas.createListFS();

  ASSERT_OR_THROWEXCEPTION( tokList.isEmpty() );
  ASSERT_OR_THROWEXCEPTION( tokList.getLength() == 0 );

  tokList.addFirst(tok3);
  ASSERT_OR_THROWEXCEPTION( tokList.getLength() == 1 );

  tokList.addLast(tok4);
  tokList.addFirst(tok2);
  tokList.addLast(tok5);
  tokList.addFirst(tok1);
  tokList.addLast(tok6);
  tokList.addFirst(tok0);

  ListFS fs(tokList);
  ASSERT_OR_THROWEXCEPTION( tokList.getLength() == tokNum );
  size_t i;
  for (i=0; i<tokNum; ++i) {
    LOG("Examining list element " << i);
    assert(!fs.isEmpty());
    FeatureStructure element = fs.getHead();
    int j = element.getIntValue(tokenNbrFeat);
    ASSERT_OR_THROWEXCEPTION(i == j);
    fs.moveToNext();
  }
  ASSERT_OR_THROWEXCEPTION( fs.isEmpty() );

  ASSERT_OR_THROWEXCEPTION( tokList.getHead() == tok0 );

  tokList.setHead(tok6);
  ASSERT_OR_THROWEXCEPTION( tokList.getHead() == tok6);

  {
    // test appending of lists
    ListFS eList1 = tcas.createListFS();
    ASSERT_OR_THROWEXCEPTION( eList1.isEmpty() );
    ListFS neList1 = tcas.createListFS();
    neList1.addFirst(tok1);
    ASSERT_OR_THROWEXCEPTION( neList1.getLength() == 1 );
    eList1.append(neList1); // append a non-empty list to an empty list
    ASSERT_OR_THROWEXCEPTION( eList1.getLength() == 1 );
    ASSERT_OR_THROWEXCEPTION( neList1.getLength() == eList1.getLength() );

    ListFS eList2  = tcas.createListFS();
    ASSERT_OR_THROWEXCEPTION( eList2.isEmpty() );
    ListFS neList2 = tcas.createListFS();
    neList2.addFirst(tok1);
    ASSERT_OR_THROWEXCEPTION( neList2.getLength() == 1 );
    neList2.append(eList2); // append an empty list to a non-empty list
    ASSERT_OR_THROWEXCEPTION( neList2.getLength() == 1 );
    ASSERT_OR_THROWEXCEPTION( eList2.getLength() == 0 );

    ListFS neList3 = tcas.createListFS();
    neList3.addFirst(tok1);
    ASSERT_OR_THROWEXCEPTION( neList3.getLength() == 1 );
    ListFS neList4 = tcas.createListFS();
    neList4.addFirst(tok2);
    ASSERT_OR_THROWEXCEPTION( neList4.getLength() == 1 );
    neList3.append(neList4); // append a non-empty list to a non-empty list
    ASSERT_OR_THROWEXCEPTION( neList3.getLength() == 2 );
    ASSERT_OR_THROWEXCEPTION( neList4.getLength() == 1 );

    bool bExcCaught = false;
    ListFS neList5 = tcas.createListFS();
    neList5.addFirst(tok1);
    ASSERT_OR_THROWEXCEPTION( neList5.getLength() == 1 );
    try {
      neList5.append(neList5); // append a non-empty list to itself
    } catch (ListIsCircularException e) {
      bExcCaught = true;
    }
    ASSERT_OR_THROWEXCEPTION( bExcCaught );

    ListFS eList6 = tcas.createListFS();
    ASSERT_OR_THROWEXCEPTION( eList6.getLength() == 0 );
    eList6.append(eList6); // append an empty list to itself
    ASSERT_OR_THROWEXCEPTION( eList6.getLength() == 0 );

    ListFS eList7 = tcas.createListFS();
    ASSERT_OR_THROWEXCEPTION( eList7.getLength() == 0 );
    ListFS eList8 = tcas.createListFS();
    ASSERT_OR_THROWEXCEPTION( eList8.getLength() == 0 );
    eList7.append(eList8); // append an empty list to an empty list
    ASSERT_OR_THROWEXCEPTION( eList7.getLength() == 0 );
    ASSERT_OR_THROWEXCEPTION( eList8.getLength() == 0 );

  }

  {
    // test prepending of lists
    ListFS eList1 = tcas.createListFS();
    ASSERT_OR_THROWEXCEPTION( eList1.isEmpty() );
    ListFS neList1 = tcas.createListFS();
    neList1.addFirst(tok1);
    ASSERT_OR_THROWEXCEPTION( neList1.getLength() == 1 );
    eList1.prepend(neList1); // prepend a non-empty list to an empty list
    ASSERT_OR_THROWEXCEPTION( eList1.getLength() == 1 );
    ASSERT_OR_THROWEXCEPTION( neList1.getLength() == eList1.getLength() );

    ListFS eList2 = tcas.createListFS();
    ASSERT_OR_THROWEXCEPTION( eList2.isEmpty() );
    ListFS neList2 = tcas.createListFS();
    neList2.addFirst(tok1);
    ASSERT_OR_THROWEXCEPTION( neList2.getLength() == 1 );
    neList2.prepend(eList2); // prepend an empty list to a non-empty list
    ASSERT_OR_THROWEXCEPTION( neList2.getLength() == 1 );
    ASSERT_OR_THROWEXCEPTION( eList2.getLength() == 0 );

    ListFS neList3 = tcas.createListFS();
    neList3.addFirst(tok1);
    ASSERT_OR_THROWEXCEPTION( neList3.getLength() == 1 );
    ListFS neList4 = tcas.createListFS();
    neList4.addFirst(tok2);
    ASSERT_OR_THROWEXCEPTION( neList4.getLength() == 1 );
    neList3.prepend(neList4); // prepend a non-empty list to a non-empty list
    ASSERT_OR_THROWEXCEPTION( neList3.getLength() == 2 );
    ASSERT_OR_THROWEXCEPTION( neList4.getLength() == 2 );

    bool bExcCaught = false;
    ListFS neList5 = tcas.createListFS();
    neList5.addFirst(tok1);
    ASSERT_OR_THROWEXCEPTION( neList5.getLength() == 1 );
    try {
      neList5.prepend(neList5); // prepend a non-empty list to itself
    } catch (ListIsCircularException e) {
      bExcCaught = true;
    }
    ASSERT_OR_THROWEXCEPTION( bExcCaught );

    ListFS eList6 = tcas.createListFS();
    ASSERT_OR_THROWEXCEPTION( eList6.getLength() == 0 );
    eList6.prepend(eList6); // prepend an empty list to itself
    ASSERT_OR_THROWEXCEPTION( eList6.getLength() == 0 );

    ListFS eList7 = tcas.createListFS();
    ASSERT_OR_THROWEXCEPTION( eList7.getLength() == 0 );
    ListFS eList8 = tcas.createListFS();
    ASSERT_OR_THROWEXCEPTION( eList8.getLength() == 0 );
    eList7.prepend(eList8); // prepend an empty list to an empty list
    ASSERT_OR_THROWEXCEPTION( eList7.getLength() == 0 );
    ASSERT_OR_THROWEXCEPTION( eList8.getLength() == 0 );
  }

  ;
  // test nested addFirst/addLast
  ListFS list  = tcas.createListFS();
  list.addFirst(tok0);
  ListFS list0 = list;
  ASSERT_OR_THROWEXCEPTION( list.getLength()  == 1 );
  ASSERT_OR_THROWEXCEPTION( list0.getLength() == 1 );
  list = list.addLast(tok1).addLast(tok2).addLast(tok3);
  ASSERT_OR_THROWEXCEPTION( list.getLength()  == 1 );
  ASSERT_OR_THROWEXCEPTION( list0.getLength() == 4);
  list = list.addFirst(tok1).addFirst(tok2).addFirst(tok3);
  ASSERT_OR_THROWEXCEPTION( list.getLength()  == 4 );
  ASSERT_OR_THROWEXCEPTION( list0.getLength() == 4 );
  list = list.addLast(tok1).addFirst(tok2).addLast(tok3);
  ASSERT_OR_THROWEXCEPTION( list.getLength()  == 1 );
  ASSERT_OR_THROWEXCEPTION( list0.getLength() == 6 );
  // test nested append/prepend
  ListFS list1, list2, list3;
  list1 = tcas.createListFS();
  list1.addFirst(tok1);
  ASSERT_OR_THROWEXCEPTION( list1.getLength() == 1 );
  list2 = tcas.createListFS();
  list2.addFirst(tok2);
  ASSERT_OR_THROWEXCEPTION( list2.getLength() == 1 );
  list3 = tcas.createListFS();
  list3.addFirst(tok3);
  ASSERT_OR_THROWEXCEPTION( list3.getLength() == 1 );

  list1 = list1.append(list2).append(list3);
  ASSERT_OR_THROWEXCEPTION( list1.getLength() == 3 );

  list1 = tcas.createListFS();
  list1.addFirst(tok1);
  ASSERT_OR_THROWEXCEPTION( list1.getLength() == 1 );
  list2 = tcas.createListFS();
  list2.addFirst(tok2);
  ASSERT_OR_THROWEXCEPTION( list2.getLength() == 1 );
  list3 = tcas.createListFS();
  list3.addFirst(tok3);
  ASSERT_OR_THROWEXCEPTION( list3.getLength() == 1 );

  list1 = list1.prepend(list2).prepend(list3);
  ASSERT_OR_THROWEXCEPTION( list1.getLength() == 3 );


  list1 = tcas.createListFS();
  list1.addFirst(tok1);
  ASSERT_OR_THROWEXCEPTION( list1.getLength() == 1 );
  list2 = tcas.createListFS();
  list2.addFirst(tok2);
  ASSERT_OR_THROWEXCEPTION( list2.getLength() == 1 );
  list3 = tcas.createListFS();
  list3.addFirst(tok3);
  ASSERT_OR_THROWEXCEPTION( list3.getLength() == 1 );

  list1 = list1.append(list2).prepend(list3);
  ASSERT_OR_THROWEXCEPTION( list1.getLength() == 3 );



  LOG("testOOListFunctions() finished");
}


template<class T>
class SimpleEquals {
public:
  bool equals(T t1, T t2) const {
    return t1 == t2;
  }
};

class StringEquals {
public:
  bool equals(UnicodeStringRef const & crp1,
              UnicodeStringRef const & crp2) const {
    LOG("Comparing: " << crp1 << " with " << crp2);
    return((UnicodeStringRef) crp1) == ((UnicodeStringRef) crp2);
  }
};

template< class LFS_TYPE, class COMP>
void testOOListFunctionsTempl(CAS & tcas, vector< typename LFS_TYPE::HeadType > & vec, COMP const & crComp) {
  LOG("testOOListFunctionsTempl() started");

  LFS_TYPE list = LFS_TYPE::createListFS(tcas);

  ASSERT_OR_THROWEXCEPTION( list.isEmpty() );
  ASSERT_OR_THROWEXCEPTION( list.getLength() == 0 );

  list.addFirst(vec[3]);
  ASSERT_OR_THROWEXCEPTION( list.getLength() == 1 );

  list.addLast(vec[4]);
  list.addFirst(vec[2]);
  list.addLast(vec[5]);
  list.addFirst(vec[1]);
  list.addLast(vec[6]);
  list.addFirst(vec[0]);

  const size_t uiNUMBER_OF_ELEMENTS = 7;

  LFS_TYPE fs(list);
  ASSERT_OR_THROWEXCEPTION( list.getLength() == uiNUMBER_OF_ELEMENTS );
  size_t i;
  for (i=0; i<uiNUMBER_OF_ELEMENTS; ++i) {
    LOG("Examining list element " << i);
    assert(!fs.isEmpty());
    typename LFS_TYPE::HeadType element = fs.getHead();
    ASSERT_OR_THROWEXCEPTION( crComp.equals( element, vec[i]) );
    fs.moveToNext();
  }
  ASSERT_OR_THROWEXCEPTION( fs.isEmpty() );

  ASSERT_OR_THROWEXCEPTION( crComp.equals( list.getHead(), vec[0]) );

  list.setHead(vec[6]);
  ASSERT_OR_THROWEXCEPTION( crComp.equals( list.getHead(), vec[6]) );

  {
    // test appending of lists
    LFS_TYPE eList1 = LFS_TYPE::createListFS(tcas);
    ASSERT_OR_THROWEXCEPTION( eList1.isEmpty() );
    LFS_TYPE neList1 = LFS_TYPE::createListFS(tcas, vec[1]);
    ASSERT_OR_THROWEXCEPTION( neList1.getLength() == 1 );
    eList1.append(neList1); // append a non-empty list to an empty list
    ASSERT_OR_THROWEXCEPTION( eList1.getLength() == 1 );
    ASSERT_OR_THROWEXCEPTION( neList1.getLength() == eList1.getLength() );

    LFS_TYPE eList2  = LFS_TYPE::createListFS(tcas);
    ASSERT_OR_THROWEXCEPTION( eList2.isEmpty() );
    LFS_TYPE neList2 = LFS_TYPE::createListFS(tcas, vec[1]);
    ASSERT_OR_THROWEXCEPTION( neList2.getLength() == 1 );
    neList2.append(eList2); // append an empty list to a non-empty list
    ASSERT_OR_THROWEXCEPTION( neList2.getLength() == 1 );
    ASSERT_OR_THROWEXCEPTION( eList2.getLength() == 0 );

    LFS_TYPE neList3 = LFS_TYPE::createListFS(tcas, vec[1]);
    ASSERT_OR_THROWEXCEPTION( neList3.getLength() == 1 );
    LFS_TYPE neList4 = LFS_TYPE::createListFS(tcas, vec[2]);
    ASSERT_OR_THROWEXCEPTION( neList4.getLength() == 1 );
    neList3.append(neList4); // append a non-empty list to a non-empty list
    ASSERT_OR_THROWEXCEPTION( neList3.getLength() == 2 );
    ASSERT_OR_THROWEXCEPTION( neList4.getLength() == 1 );

    bool bExcCaught = false;
    LFS_TYPE neList5 = LFS_TYPE::createListFS(tcas, vec[1]);
    ASSERT_OR_THROWEXCEPTION( neList5.getLength() == 1 );
    try {
      neList5.append(neList5); // append a non-empty list to itself
    } catch (ListIsCircularException e) {
      bExcCaught = true;
    }
    ASSERT_OR_THROWEXCEPTION( bExcCaught );

    LFS_TYPE eList6 = LFS_TYPE::createListFS(tcas);
    ASSERT_OR_THROWEXCEPTION( eList6.getLength() == 0 );
    eList6.append(eList6); // append an empty list to itself
    ASSERT_OR_THROWEXCEPTION( eList6.getLength() == 0 );

    LFS_TYPE eList7 = LFS_TYPE::createListFS(tcas);
    ASSERT_OR_THROWEXCEPTION( eList7.getLength() == 0 );
    LFS_TYPE eList8 = LFS_TYPE::createListFS(tcas);
    ASSERT_OR_THROWEXCEPTION( eList8.getLength() == 0 );
    eList7.append(eList8); // append an empty list to an empty list
    ASSERT_OR_THROWEXCEPTION( eList7.getLength() == 0 );
    ASSERT_OR_THROWEXCEPTION( eList8.getLength() == 0 );

  }

  {
    // test prepending of lists
    LFS_TYPE eList1 = LFS_TYPE::createListFS(tcas);
    ASSERT_OR_THROWEXCEPTION( eList1.isEmpty() );
    LFS_TYPE neList1 = LFS_TYPE::createListFS(tcas, vec[1]);
    ASSERT_OR_THROWEXCEPTION( neList1.getLength() == 1 );
    eList1.prepend(neList1); // prepend a non-empty list to an empty list
    ASSERT_OR_THROWEXCEPTION( eList1.getLength() == 1 );
    ASSERT_OR_THROWEXCEPTION( neList1.getLength() == eList1.getLength() );

    LFS_TYPE eList2 = LFS_TYPE::createListFS(tcas);
    ASSERT_OR_THROWEXCEPTION( eList2.isEmpty() );
    LFS_TYPE neList2 = LFS_TYPE::createListFS(tcas, vec[1]);
    ASSERT_OR_THROWEXCEPTION( neList2.getLength() == 1 );
    neList2.prepend(eList2); // prepend an empty list to a non-empty list
    ASSERT_OR_THROWEXCEPTION( neList2.getLength() == 1 );
    ASSERT_OR_THROWEXCEPTION( eList2.getLength() == 0 );

    LFS_TYPE neList3 = LFS_TYPE::createListFS(tcas, vec[1]);
    ASSERT_OR_THROWEXCEPTION( neList3.getLength() == 1 );
    LFS_TYPE neList4 = LFS_TYPE::createListFS(tcas, vec[2]);
    ASSERT_OR_THROWEXCEPTION( neList4.getLength() == 1 );
    neList3.prepend(neList4); // prepend a non-empty list to a non-empty list
    ASSERT_OR_THROWEXCEPTION( neList3.getLength() == 2 );
    ASSERT_OR_THROWEXCEPTION( neList4.getLength() == 2 );

    bool bExcCaught = false;
    LFS_TYPE neList5 = LFS_TYPE::createListFS(tcas, vec[1]);
    ASSERT_OR_THROWEXCEPTION( neList5.getLength() == 1 );
    try {
      neList5.prepend(neList5); // prepend a non-empty list to itself
    } catch (ListIsCircularException e) {
      bExcCaught = true;
    }
    ASSERT_OR_THROWEXCEPTION( bExcCaught );

    LFS_TYPE eList6 = LFS_TYPE::createListFS(tcas);
    ASSERT_OR_THROWEXCEPTION( eList6.getLength() == 0 );
    eList6.prepend(eList6); // prepend an empty list to itself
    ASSERT_OR_THROWEXCEPTION( eList6.getLength() == 0 );

    LFS_TYPE eList7 = LFS_TYPE::createListFS(tcas);
    ASSERT_OR_THROWEXCEPTION( eList7.getLength() == 0 );
    LFS_TYPE eList8 = LFS_TYPE::createListFS(tcas);
    ASSERT_OR_THROWEXCEPTION( eList8.getLength() == 0 );
    eList7.prepend(eList8); // prepend an empty list to an empty list
    ASSERT_OR_THROWEXCEPTION( eList7.getLength() == 0 );
    ASSERT_OR_THROWEXCEPTION( eList8.getLength() == 0 );
  }

  ;
  // test nested addFirst/addLast
  list  = LFS_TYPE::createListFS(tcas, vec[0]);
  LFS_TYPE list0 = list;
  ASSERT_OR_THROWEXCEPTION( list.getLength()  == 1 );
  ASSERT_OR_THROWEXCEPTION( list0.getLength() == 1 );
  list = list.addLast(vec[1]).addLast(vec[2]).addLast(vec[3]);
  ASSERT_OR_THROWEXCEPTION( list.getLength()  == 1 );
  ASSERT_OR_THROWEXCEPTION( list0.getLength() == 4);
  list = list.addFirst(vec[1]).addFirst(vec[2]).addFirst(vec[3]);
  ASSERT_OR_THROWEXCEPTION( list.getLength()  == 4 );
  ASSERT_OR_THROWEXCEPTION( list0.getLength() == 4 );
  list = list.addLast(vec[1]).addFirst(vec[2]).addLast(vec[3]);
  ASSERT_OR_THROWEXCEPTION( list.getLength()  == 1 );
  ASSERT_OR_THROWEXCEPTION( list0.getLength() == 6 );
  // test nested append/prepend
  LFS_TYPE list1, list2, list3;
  list1 = LFS_TYPE::createListFS(tcas, vec[1]);
  ASSERT_OR_THROWEXCEPTION( list1.getLength() == 1 );
  list2 = LFS_TYPE::createListFS(tcas, vec[2]);
  ASSERT_OR_THROWEXCEPTION( list2.getLength() == 1 );
  list3 = LFS_TYPE::createListFS(tcas, vec[3]);
  ASSERT_OR_THROWEXCEPTION( list3.getLength() == 1 );

  list1 = list1.append(list2).append(list3);
  ASSERT_OR_THROWEXCEPTION( list1.getLength() == 3 );

  list1 = LFS_TYPE::createListFS(tcas, vec[1]);
  ASSERT_OR_THROWEXCEPTION( list1.getLength() == 1 );
  list2 = LFS_TYPE::createListFS(tcas, vec[2]);
  ASSERT_OR_THROWEXCEPTION( list2.getLength() == 1 );
  list3 = LFS_TYPE::createListFS(tcas, vec[3]);
  ASSERT_OR_THROWEXCEPTION( list3.getLength() == 1 );

  list1 = list1.prepend(list2).prepend(list3);
  ASSERT_OR_THROWEXCEPTION( list1.getLength() == 3 );


  list1 = LFS_TYPE::createListFS(tcas, vec[1]);
  ASSERT_OR_THROWEXCEPTION( list1.getLength() == 1 );
  list2 = LFS_TYPE::createListFS(tcas, vec[2]);
  ASSERT_OR_THROWEXCEPTION( list2.getLength() == 1 );
  list3 = LFS_TYPE::createListFS(tcas, vec[3]);
  ASSERT_OR_THROWEXCEPTION( list3.getLength() == 1 );

  list1 = list1.append(list2).prepend(list3);
  ASSERT_OR_THROWEXCEPTION( list1.getLength() == 3 );

  LOG("testOOListFunctionsTempl() finished");
}

void testOOListFunctions() {
  LOG("testOOFSListFunctions() started");
  testOOListFunctionsOld();

  TestCAS testcas;
  uima::internal::CASImpl& tcas = testcas.getCAS();

  TypeSystem const & typeSystem = tcas.getTypeSystem();

  Type tokenAnnType = typeSystem.getType(TT::TYPE_NAME_TOKEN_ANNOTATION);

  size_t i;
  vector<FeatureStructure> vecFS;
  for (i=0; i < 10; ++i) {
    vecFS.push_back(tcas.createFS(tokenAnnType));
  }
  SimpleEquals<FeatureStructure> fsComp;
  testOOListFunctionsTempl<ListFS>(tcas, vecFS, fsComp);

  vector<float> vecFloat;
  for (i=0; i < 10; ++i) {
    vecFloat.push_back((float)i);
  }
  SimpleEquals<float> floatComp;
  testOOListFunctionsTempl<FloatListFS>(tcas, vecFloat, floatComp);

  vector<int> vecInt;
  for (i=0; i < 10; ++i) {
    vecInt.push_back((int)i);
  }
  SimpleEquals<int> intComp;
  testOOListFunctionsTempl<IntListFS>(tcas, vecInt, intComp);

  LOG("Testing string lists");
  icu::UnicodeString us("12345678901234");
  UChar const * pu = us.getBuffer();
  vector<UnicodeStringRef > vecStr;
  for (i=0; i < 10; ++i) {
    UnicodeStringRef p(pu, i + 1);
    vecStr.push_back(p);
    LOG("string list element: " << i << ": " << p );
  }
  StringEquals streq;
  testOOListFunctionsTempl<StringListFS>(tcas, vecStr, streq);


  LOG("test deletion");

  IntListFS intList = tcas.createIntListFS();
  intList = intList.addLast(0);
  for (i=1; i<vecInt.size(); ++i) {
    intList.addLast(i);
  }

  IntListFS tempIntList = intList;
  size_t j=0;
  while (!tempIntList.isEmpty()) {
    int head = tempIntList.getHead();
    LOG("Head: " << head << ", expected int: " << j);
    ASSERT_OR_THROWEXCEPTION(j < vecInt.size() );
    ASSERT_OR_THROWEXCEPTION(head == vecInt[j]);
    ++j;
    tempIntList = tempIntList.getTail();
  }

  int iElemToBeDeleted = 2;
  tempIntList = intList;
  j=0;
  while (!tempIntList.isEmpty()) {
    int head = tempIntList.getHead();
    ASSERT_OR_THROWEXCEPTION(j < vecInt.size() );
    ASSERT_OR_THROWEXCEPTION(head == vecInt[j]);
    if (j == iElemToBeDeleted) {
      LOG("Deleting element: " << iElemToBeDeleted+1);
      IntListFS intList2 = tempIntList.getTail();
      ASSERT_OR_THROWEXCEPTION( ! intList2.isEmpty() );
      tempIntList.setTail( intList2.getTail() );
      break;
    }
    ++j;
    tempIntList = tempIntList.getTail();
  }
  ASSERT_OR_THROWEXCEPTION( ! tempIntList.isEmpty() );

  bool bDeleted = false;
  tempIntList = intList;
  j=0;
  while (!tempIntList.isEmpty()) {
    int head = tempIntList.getHead();
    ASSERT_OR_THROWEXCEPTION(j < vecInt.size() );
//      LOG("Head: " << head << ", expected int: " << j);

    if (j == iElemToBeDeleted+1) {
      LOG("Skipping deleted element: " << j );
      ++j;
      bDeleted = true;
    }
    LOG("Head: " << head << ", expected int: " << j);
    ASSERT_OR_THROWEXCEPTION(head == vecInt[j]);
    ++j;
    tempIntList = tempIntList.getTail();
  }
  ASSERT_OR_THROWEXCEPTION( bDeleted );

  LOG("testOOListFunctions() finished");
}


/************************************************************************************/

template < class T,
const uima::lowlevel::TyFSType LType,
const uima::lowlevel::TyFSType ELType,
const uima::lowlevel::TyFSType NELType,
const uima::lowlevel::TyFSType HeadFeat,
const uima::lowlevel::TyFSType TailFeat>
void testDeleteElementFromList(BasicListFS< T, LType, ELType, NELType, HeadFeat, TailFeat> list,
                               T const & element,
                               vector<T> const & v) {
  BasicListFS< T, LType, ELType, NELType, HeadFeat, TailFeat> newList = list.removeElement(element);
  int i=0;
  while (! list.isEmpty() ) {

    if (list.getHead() == element) {
      LOG("Element: " << element << " should have been deleted!!");
      ASSERT_OR_THROWEXCEPTION(false);
    }
    if (element == v[i]) {
      ASSERT_OR_THROWEXCEPTION( newList.isValid() );
      ++i;
    }
    list = list.getTail();
    ++i;
  }
  if ( newList.isValid() ) {
    ASSERT_OR_THROWEXCEPTION( newList.getHead() == element );
    BasicListFS< T, LType, ELType, NELType, HeadFeat, TailFeat> newListTail = newList.getTail();
    ASSERT_OR_THROWEXCEPTION( newListTail.isEmpty() );
  }
}

void testDeleteElementFromList() {
  LOG("testDeleteElementFromList() started");
  TestCAS testcas;
  uima::internal::CASImpl & tcas = testcas.getCAS();

  const int ELEMENT_NUM = 10;
  int i,j;
  for (j=0; j<ELEMENT_NUM; ++j) {
    IntListFS intList = tcas.createIntListFS();
    vector<int> v;
    LOG("Testing list of length " << j);
    for (i=0; i<j; ++i) {
      intList.addLast(i);
      v.push_back(i);
    }
    ASSERT_OR_THROWEXCEPTION( intList.getLength() == j );

    for (i=0; i<ELEMENT_NUM; ++i) {
      testDeleteElementFromList(intList, i, v);
    }

  }
  LOG("testDeleteElementFromList() finished");
}

/************************************************************************************/

void testNullFSs() {
  LOG("testNullFSs() started");
  ASSERT_OR_THROWEXCEPTION( uima::lowlevel::FSHeap::INVALID_FS == 0 );
  TestCAS testcas;
  uima::internal::CASImpl & tcas = testcas.getCAS();

  LOG("Test Null FS on lowlevel API");
  uima::lowlevel::TypeSystem const & lolTypeSys = tcas.getHeap().getTypeSystem();
  uima::lowlevel::FSHeap & lolFSHeap = tcas.getHeap();
  uima::lowlevel::TyFSType lolAnnotType = lolTypeSys.getTypeByName(TT::TYPE_NAME_TOKEN_ANNOTATION);
  uima::lowlevel::TyFSFeature lolBeginPosFeature = lolTypeSys.getFeatureByBaseName(lolAnnotType, CAS::FEATURE_BASE_NAME_BEGIN);
  uima::lowlevel::TyFSFeature lolLemmaListFeature = lolTypeSys.getFeatureByBaseName(lolAnnotType, TT::FEATURE_BASE_NAME_LEMMA_ENTRIES);

  lowlevel::TyFS lolFS = lolFSHeap.createFS(lolAnnotType);
  ASSERT_OR_THROWEXCEPTION( lolFSHeap.getIntValue(lolFS, lolBeginPosFeature) == 0 );
  lowlevel::TyFS lolInvalidLemmaList = lolFSHeap.getFSValue(lolFS, lolLemmaListFeature );
  ASSERT_OR_THROWEXCEPTION( lolInvalidLemmaList == 0 );

  LOG("Test Null FS on OO API");
  TypeSystem const & typeSys = tcas.getTypeSystem();

  Type annotType = typeSys.getType(TT::TYPE_NAME_TOKEN_ANNOTATION);
  Feature beginPosFeature = annotType.getFeatureByBaseName( CAS::FEATURE_BASE_NAME_BEGIN);
  Feature lemmaListFeature = annotType.getFeatureByBaseName( TT::FEATURE_BASE_NAME_LEMMA_ENTRIES);

  FeatureStructure fs = tcas.createFS(annotType);
  ASSERT_OR_THROWEXCEPTION( fs.getIntValue(beginPosFeature) == 0 );
  FeatureStructure invalidLemmaList = fs.getFSValue(lemmaListFeature);
  ASSERT_OR_THROWEXCEPTION( ! invalidLemmaList.isValid() );

  LOG("testNullFSs() finished");
}

/************************************************************************************/

void testFullyQualifiedFeatureNames() {
  LOG("testFullyQualifiedFeatureNames() started");
  TestCAS testcas;
  uima::internal::CASImpl & tcas = testcas.getCAS();

  icu::UnicodeString annotBeginPos(CAS::TYPE_NAME_ANNOTATION);
  annotBeginPos.append(":");
  annotBeginPos.append(CAS::FEATURE_BASE_NAME_BEGIN);

  icu::UnicodeString tokenBeginPos(TT::TYPE_NAME_TOKEN_ANNOTATION);
  tokenBeginPos.append(":");
  tokenBeginPos.append(CAS::FEATURE_BASE_NAME_BEGIN);

  uima::TypeSystem const & typeSys = tcas.getTypeSystem();
  Type annotType = typeSys.getType(CAS::TYPE_NAME_ANNOTATION);
  ASSERT_OR_THROWEXCEPTION( annotType.isValid() );
  Type tokenType = typeSys.getType(TT::TYPE_NAME_TOKEN_ANNOTATION);
  ASSERT_OR_THROWEXCEPTION( tokenType.isValid() );

  LOG("get feature: '" << annotBeginPos << "'");
  Feature beginPosFeature = typeSys.getFeatureByFullName(annotBeginPos);
  ASSERT_OR_THROWEXCEPTION( beginPosFeature.isValid() );

  uima::Feature beginPosFeature1 = typeSys.getFeatureByFullName( tokenBeginPos );
  ASSERT_OR_THROWEXCEPTION( beginPosFeature1.isValid() );
  ASSERT_OR_THROWEXCEPTION( beginPosFeature1 == beginPosFeature );

  uima::Feature beginPosFeature2 = annotType.getFeatureByBaseName( CAS::FEATURE_BASE_NAME_BEGIN);
  ASSERT_OR_THROWEXCEPTION( beginPosFeature2.isValid() );
  ASSERT_OR_THROWEXCEPTION( beginPosFeature2 == beginPosFeature );

  uima::Feature beginPosFeature3 = tokenType.getFeatureByBaseName( CAS::FEATURE_BASE_NAME_BEGIN);
  ASSERT_OR_THROWEXCEPTION( beginPosFeature3.isValid() );
  ASSERT_OR_THROWEXCEPTION( beginPosFeature3 == beginPosFeature );

  /*
  uima::Feature beginPosFeature4 = typeSys.getFeature( CAS::TYPE_NAME_ANNOTATION, CAS::FEATURE_BASE_NAME_BEGIN);
  ASSERT_OR_THROWEXCEPTION( beginPosFeature4.isValid() );
  ASSERT_OR_THROWEXCEPTION( beginPosFeature4 == beginPosFeature );

  uima::Feature beginPosFeature5 = typeSys.getFeature( TT::TYPE_NAME_TOKEN_ANNOTATION, CAS::FEATURE_BASE_NAME_BEGIN);
  ASSERT_OR_THROWEXCEPTION( beginPosFeature5.isValid() );
  ASSERT_OR_THROWEXCEPTION( beginPosFeature5 == beginPosFeature );
  */
 
  LOG("testFullyQualifiedFeatureNames() finished");
}
/************************************************************************************/
void testTypeNameSpaces() {
  LOG("testTypeNameSpaces() started");
  /*
    top
       n1.t1 [f1: t3, g1:t4]
       n1.n2.t2 [f2:t2]
       n1.n2.t3
       n4.t1
   */
  TTDefinition ttdef;
  ttdef.init();

  lowlevel::TypeSystem& ts = ttdef.getTypeSystem();

  lowlevel::TyFSType top = ts.getTopType();
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(top) );
  lowlevel::TyFSType t1 = ts.createType(top, "n1.t1", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t1) );
  lowlevel::TyFSType t2 = ts.createType(top, "n1.n2.t2", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t2) );
  lowlevel::TyFSType t3 = ts.createType(top, "n1.n2.t3", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t3) );
  lowlevel::TyFSType t4 = ts.createType(top, "n4.t1", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidType(t4) );

  lowlevel::TyFSFeature f1 = ts.createFeature(t1, t3, false, "f1", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(f1) );
  lowlevel::TyFSFeature g1 = ts.createFeature(t1, t4, false, "g1", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(g1) );
  lowlevel::TyFSFeature f2 = ts.createFeature(t2, t2, false, "f2", ustrCreatorID);
  ASSERT_OR_THROWEXCEPTION( ts.isValidFeature(f2) );

  ttdef.commit();

//      uima::internal::TCASImpl cas(ttdef, 1000, 1000, 1000);
   uima::internal::CASImpl * pCas = getCASImpl(ttdef);
  uima::internal::CASImpl & cas = *pCas;
  TypeSystem const & typeSys = cas.getTypeSystem();

  TypeNameSpace n1(typeSys, "n1");
  vector<Type> types;
  n1.getAllTypes(types);
  LOG("namespace size: " << types.size() << ", expected size: 1");
  ASSERT_OR_THROWEXCEPTION( types.size() == 1);

  ASSERT_OR_THROWEXCEPTION( n1.getType("t1").isValid() );

  TypeNameSpace n1n2(typeSys, "n1.n2");
  n1n2.getAllTypes(types);
  LOG("namespace size: " << types.size() << ", expected size: 2");
  ASSERT_OR_THROWEXCEPTION( types.size() == 2);
  ASSERT_OR_THROWEXCEPTION( n1n2.getType("t2").isValid() );
  ASSERT_OR_THROWEXCEPTION( n1n2.getType("t3").isValid() );

  TypeNameSpace n4(typeSys, "n4");
  n4.getAllTypes(types);
  LOG("namespace size: " << types.size() << ", expected size: 1");
  ASSERT_OR_THROWEXCEPTION( types.size() == 1);
  ASSERT_OR_THROWEXCEPTION( n4.getType("t1").isValid() );


  TypeNameSpaceImport import(typeSys);
  import.addNameSpace(n1);
  import.addNameSpace(n1n2);
  import.addNameSpace(n4);

  import.getAllTypes(types);

  LOG("namespace size: " << types.size() << ", expected size: 4");
  ASSERT_OR_THROWEXCEPTION(types.size() == 4);

  for ( unsigned int i = 0; i < types.size(); ++i ) {
    Type typ = types[i];
    LOG("  Type: " << typ.getName());
  }

  Type t;
  bool noConflict;
  noConflict = import.getType("t2", t);
  ASSERT_OR_THROWEXCEPTION( noConflict );
  ASSERT_OR_THROWEXCEPTION( t.isValid() );

  noConflict = import.getType("t3", t);
  ASSERT_OR_THROWEXCEPTION( noConflict );
  ASSERT_OR_THROWEXCEPTION( t.isValid() );

  noConflict = import.getType("t1", t);
  ASSERT_OR_THROWEXCEPTION( ! noConflict );
  ASSERT_OR_THROWEXCEPTION( ! t.isValid() );

  delete pCas;
  LOG("testTypeNameSpaces() finished");
}
/************************************************************************************/

void testCASReset() {
  LOG("testCASReset() started");
  TestCAS testcas;
  uima::internal::CASImpl & tcas = testcas.getCAS();
  uima::FSIndexRepository & indexRep = tcas.getIndexRepository();

  TypeSystem const & typeSys = tcas.getTypeSystem();

  Type annotType = typeSys.getType(TT::TYPE_NAME_TOKEN_ANNOTATION);
  Feature beginPosFeature = annotType.getFeatureByBaseName( CAS::FEATURE_BASE_NAME_BEGIN);
  Feature endPosFeature = annotType.getFeatureByBaseName( CAS::FEATURE_BASE_NAME_END);

  LOG("Creating FSs");
  FeatureStructure fs1 = tcas.createFS(annotType);
  ASSERT_OR_THROWEXCEPTION( fs1.isUntouchedFSValue(beginPosFeature) );
  fs1.setIntValue( beginPosFeature, 1 );
  ASSERT_OR_THROWEXCEPTION( ! fs1.isUntouchedFSValue(beginPosFeature) );
  fs1.setIntValue( endPosFeature, 2 );
  indexRep.addFS( fs1 );

  FeatureStructure fs2 = tcas.createFS(annotType);
  fs2.setIntValue( beginPosFeature, 1 );
  fs2.setIntValue( endPosFeature, 3 );
  indexRep.addFS( fs2 );

  FeatureStructure fs3 = tcas.createFS(annotType);
  fs3.setIntValue( beginPosFeature, 3 );
  fs3.setIntValue( endPosFeature, 4 );
  indexRep.addFS( fs3 );

  FeatureStructure fs4 = tcas.createFS(annotType);
  fs4.setIntValue( beginPosFeature, 4 );
  fs4.setIntValue( endPosFeature, 5 );
  indexRep.addFS( fs4 );

  FeatureStructure fs41 = tcas.createFS(annotType);
  fs41.setIntValue( beginPosFeature, 4 );
  fs41.setIntValue( endPosFeature, 5 );
  indexRep.addFS( fs41 );
  LOG("...FSs created");

  icu::UnicodeString const & indexID = tcas.getAnnotationIndexID();
  ASSERT_OR_THROWEXCEPTION( tcas.getIndexRepository().getIndex(indexID, annotType).getSize() == 5);
  ASSERT_OR_THROWEXCEPTION( fs1.isValid() );

  tcas.reset();

  ASSERT_OR_THROWEXCEPTION( tcas.getIndexRepository().getIndex(indexID, annotType).getSize() == 0);
  ASSERT_OR_THROWEXCEPTION( ! fs1.isValid() );



  LOG("testCASReset() finished");
}

/************************************************************************************/

void testClone() {
  LOG("testClone() started");
  TestCAS testcas;
  uima::internal::CASImpl & tcas = testcas.getCAS();

  uima::TypeSystem const & ts = tcas.getTypeSystem();

  uima::Type lemmaType = ts.getType(TT::TYPE_NAME_LEMMA);
  uima::Feature posFeature = lemmaType.getFeatureByBaseName( TT::FEATURE_BASE_NAME_PART_OF_SPEECH);
  uima::Feature morphIDFeature = lemmaType.getFeatureByBaseName( TT::FEATURE_BASE_NAME_MORPH_ID);
  uima::Feature keyFeature = lemmaType.getFeatureByBaseName( TT::FEATURE_BASE_NAME_KEY);

  const int pos = 1;
  const int morphID = 2;
  icu::UnicodeString str("str");
  UnicodeStringRef uref(str.getBuffer(), str.length());

  uima::FeatureStructure fs1 = tcas.createFS(lemmaType);
  fs1.setIntValue(posFeature, pos);
  fs1.setIntValue(morphIDFeature, morphID);
  fs1.setStringValueExternal(keyFeature, uref);

  uima::FeatureStructure fs2 = fs1.clone();

//   tcas.getFSSystem().getLowlevelFSHeap().print(cout);

  ASSERT_OR_THROWEXCEPTION( fs2.getIntValue(posFeature) == pos );
  UnicodeStringRef uref2 = fs2.getStringValue(keyFeature);
  ASSERT_OR_THROWEXCEPTION( uref2 == uref );
#ifdef EXTERNAL_STRINGS_ARE_SUPPORTED_AGAIN
  ASSERT_OR_THROWEXCEPTION( uref2.getBuffer() == uref.getBuffer() );
#endif
  ASSERT_OR_THROWEXCEPTION( fs2.getIntValue(morphIDFeature) == morphID );
  LOG("testClone() finished");
}


/************************************************************************************/

void testXMLTypeSystemReader() {
  LOG("testXMLTypeSystemReader() started");

  try {
    XMLPlatformUtils::Initialize();
  } catch (const XMLException&) {
    ASSERT_OR_THROWEXCEPTION(false);
  }

  TTDefinition ttdef;
  ttdef.init();

  lowlevel::TypeSystem & ts = ttdef.getTypeSystem();

  uima::XMLTypeSystemReader xmlReader(ts);
  LOG("Reading file");

  char const * xmlfile =
    "<?xml version=\"1.0\"?>"
    "<type_system>"
    "<type name=\"top\">"
    "  <type name=\"t1\">"
    "    <feature name=\"f1\" range=\"t21\" multirefs=\"false\"/>"
    "    <feature name=\"f2\" range=\"t1\" multirefs=\"true\"/>"
    "  </type>"
    "  <type name=\"t2\">"
    "     <type name=\"t21\">"
    "       <feature name=\"f1\" range=\"t21\" multirefs=\"false\"/>"
    "     </type>"
    "  </type>"
    "</type>"
    "</type_system>";

  xmlReader.readMemory(xmlfile, ustrCreatorID);
  ttdef.commit();

//      uima::internal::TCASImpl tcas(ttdef, 5000, 5000, 5000);
   uima::internal::CASImpl * pCas = getCASImpl(ttdef);
  uima::internal::CASImpl & tcas = *pCas;
  uima::TypeSystem const & crTypeSystem = tcas.getTypeSystem();

  uima::Type top = crTypeSystem.getTopType();
  ASSERT_OR_THROWEXCEPTION( top.isValid());

  uima::Type t1 = crTypeSystem.getType("t1");
  ASSERT_OR_THROWEXCEPTION( t1.isValid() );
  ASSERT_OR_THROWEXCEPTION( top.subsumes(t1) );

  uima::Type t2 = crTypeSystem.getType("t2");
  ASSERT_OR_THROWEXCEPTION( t2.isValid() );
  ASSERT_OR_THROWEXCEPTION( top.subsumes(t2) );

  uima::Type t21 = crTypeSystem.getType("t21");
  ASSERT_OR_THROWEXCEPTION( t21.isValid() );
  ASSERT_OR_THROWEXCEPTION( top.subsumes(t21) );
  ASSERT_OR_THROWEXCEPTION( t2.subsumes(t21) );

  uima::Feature f1 = t1.getFeatureByBaseName( "f1");
  ASSERT_OR_THROWEXCEPTION( f1.isValid() );
  uima::Type f1dom;
  f1.getRangeType(f1dom);
  ASSERT_OR_THROWEXCEPTION( f1dom == t21 );
  ASSERT_OR_THROWEXCEPTION( ! f1.isMultipleReferencesAllowed() );

  uima::Feature f2 = crTypeSystem.getFeatureByFullName("t1:f2");
  ASSERT_OR_THROWEXCEPTION( f2.isValid() );
  uima::Type f2dom;
  f2.getRangeType(f2dom);
  ASSERT_OR_THROWEXCEPTION( f2dom == t1 );
  ASSERT_OR_THROWEXCEPTION( f2.isMultipleReferencesAllowed() );



#ifndef NDEBUG
  ts.print(cout);
#endif

  XMLPlatformUtils::Terminate();
  delete pCas;
  LOG("testXMLTypeSystemReader() finished");
}



void testStringSubypes() {
  LOG("testStringSubypes() started");
  TTDefinition ttdef;
  ttdef.init();

  vector<icu::UnicodeString> v1;
  v1.push_back("a1");
  v1.push_back("b1");
  v1.push_back("c1");

  vector<icu::UnicodeString> v2;
  v2.push_back("a2");
  v2.push_back("b2");

  lowlevel::TypeSystem & ts = ttdef.getTypeSystem();
  lowlevel::TyFSType t1 = ts.createStringSubtype("t1", v1, "kreator");
  lowlevel::TyFSType t2 = ts.createStringSubtype("t2", v2, "kreator");

  lowlevel::TyFSType t = ts.createType(ts.getTopType(), "t", "kreator");
  lowlevel::TyFSFeature f1 = ts.createFeature(t, t1, false, "f1", "kreator");
  (void)ts.createFeature(t, t2, false, "f2", "kreator");

  ttdef.commit();

  ASSERT_OR_THROWEXCEPTION( ts.getTypeByName("t1") == t1 );
  ASSERT_OR_THROWEXCEPTION( ts.getTypeByName("t2") == t2 );

//      uima::internal::TCASImpl tcas(ttdef, 5000, 5000, 5000);
   uima::internal::CASImpl * pCas = getCASImpl(ttdef);
  uima::internal::CASImpl & tcas = *pCas;
  // lowlevel API
  lowlevel::FSHeap & heap = tcas.getHeap();

  lowlevel::TyFS fs1 = heap.createFS( t );

  icu::UnicodeString a1("a1");
  UnicodeStringRef a1Ref(a1);

  int a1Off = heap.addString(a1Ref);
  heap.setStringValue( fs1, f1, a1Off);
  ASSERT_OR_THROWEXCEPTION( heap.getStringValue( fs1, f1) == a1Ref);


  // OO API
  icu::UnicodeString inval("bla");

  FeatureStructure oo_fs = uima::internal::FSPromoter::promoteFS(fs1, tcas );
  Feature oo_f1 = tcas.getTypeSystem().getFeatureByFullName("t:f1");
  ASSERT_OR_THROWEXCEPTION( oo_f1.isValid() );
  icu::UnicodeString b1("b1");
  oo_fs.setStringValue( oo_f1, b1 );
  ASSERT_OR_THROWEXCEPTION( oo_fs.getStringValue(oo_f1) == b1 );

  bool bExcCaught = false;
  try {
    oo_fs.setStringValue(oo_f1, inval);
  } catch (WrongStringValueException const & exc) {
    bExcCaught = true;
    LOG("Exception thrown correctly: " << exc.asString());
  }
  ASSERT_OR_THROWEXCEPTION( bExcCaught );

  delete pCas;
  LOG("testStringSubypes() finished");
}


void testClone2() {
  LOG("testClone2() started");
  TestCAS testcas;
  uima::internal::CASImpl & tcas = testcas.getCAS();
  uima::TypeSystem const & ts = tcas.getTypeSystem();

  uima::Type lemmaType = ts.getType(TT::TYPE_NAME_LEMMA);
//   uima::Feature posFeature = lemmaType.getFeatureByBaseName( TT::FEATURE_BASE_NAME_PART_OF_SPEECH);
//   uima::Feature morphIDFeature = lemmaType.getFeatureByBaseName( TT::FEATURE_BASE_NAME_MORPH_ID);
  uima::Feature keyFeature = lemmaType.getFeatureByBaseName( TT::FEATURE_BASE_NAME_KEY);


  icu::UnicodeString str1("str1");
  uima::UnicodeStringRef ref1(str1);

  icu::UnicodeString str2("str2");
  uima::UnicodeStringRef ref2(str2);

  FeatureStructure fs1 = tcas.createFS(lemmaType);
  fs1.setStringValue(keyFeature, ref1);
  ASSERT_OR_THROWEXCEPTION( ref1 == fs1.getStringValue(keyFeature) );

//   tcas.getFSSystem().getLowlevelFSHeap().print(cout);

  FeatureStructure fs2 = fs1.clone();
//   tcas.getFSSystem().getLowlevelFSHeap().print(cout);
  fs2.setStringValue(keyFeature, ref2);
  ASSERT_OR_THROWEXCEPTION( ref2 == fs2.getStringValue(keyFeature) );

//   tcas.getFSSystem().getLowlevelFSHeap().print(cout);

  ASSERT_OR_THROWEXCEPTION( ref1 == fs1.getStringValue(keyFeature) );
  ASSERT_OR_THROWEXCEPTION( fs1.getStringValue(keyFeature) != fs2.getStringValue(keyFeature) );

  LOG("testClone2() finished");
}


void testClone3() {
  LOG("testClone3() started");
  TestCAS testcas;
  uima::internal::CASImpl & tcas = testcas.getCAS();
  uima::TypeSystem const & ts = tcas.getTypeSystem();
  uima::Type lemmaType = ts.getType(TT::TYPE_NAME_LEMMA);
  uima::Type keyStringEntryType = ts.getType(TT::TYPE_NAME_KEY_STRING_ENTRY);
  uima::Feature keyFeature = lemmaType.getFeatureByBaseName( TT::FEATURE_BASE_NAME_KEY);
  icu::UnicodeString str1("str1");
  uima::UnicodeStringRef ref1(str1);

  // 1. clone downward
  FeatureStructure fs1 = tcas.createFS(keyStringEntryType);
  fs1.setStringValue(keyFeature, ref1);
  ASSERT_OR_THROWEXCEPTION( ref1 == fs1.getStringValue(keyFeature) );

  FeatureStructure fs2 = fs1.clone( lemmaType );
  ASSERT_OR_THROWEXCEPTION( fs2.getType() == lemmaType );
  ASSERT_OR_THROWEXCEPTION( fs1.getStringValue(keyFeature) == fs2.getStringValue(keyFeature) );

  // 2. clone upward
  FeatureStructure fs3 = fs2.clone( keyStringEntryType );
  ASSERT_OR_THROWEXCEPTION( fs3.getType() == keyStringEntryType );
  ASSERT_OR_THROWEXCEPTION( fs3.getStringValue(keyFeature) == fs2.getStringValue(keyFeature) );

  // 3. clone sideways

  uima::Type canonFormType = ts.getType(TT::TYPE_NAME_CANONICAL_FORM);
  FeatureStructure fs4 = fs2.clone(canonFormType);
  ASSERT_OR_THROWEXCEPTION( fs4.getType() == canonFormType );
  ASSERT_OR_THROWEXCEPTION( fs4.getStringValue(keyFeature) == fs2.getStringValue(keyFeature) );

  ASSERT_OR_THROWEXCEPTION( fs2.getStringValue(keyFeature) == ref1);
  ASSERT_OR_THROWEXCEPTION( fs3.getStringValue(keyFeature) == ref1);
  ASSERT_OR_THROWEXCEPTION( fs4.getStringValue(keyFeature) == ref1);


  LOG("testClone3() finished");
}

/************************************************************************************/


int main() {
  LOG("*********************************");
  LOG("   UIMA5 regression test");
  LOG(" Please note that when running in debug");
  LOG(" mode the test routine testOOFilterBuilder()");
  LOG(" will report memory leaks because the TTDefinition");
  LOG(" does not properly clean up memory allocations.");

  LOG("*********************************");
  LOG("Go...");

  bool bSuccess = true;
  try {
    ResourceManager::createInstance("rsmgr");


    testLowLevelTypeSystem();
    testLowLevelTypeSystemExceptions();
    testLowLevelFSHeap();
    testFloatFeatures();
    testEmptyStrings();
    testLowLevelIndex();
    testNestedStructures();


  
    testStringSubypes();
    testFilters();
    testPermanentStructures();
    testIteratorSetToPosition();
   
    testOOIndex();
    testOOExceptions();

    testOOFilterBuilder();  //TTDefinition leak
    testOOArrayFS();
    testOOStringArrayFS();
    testClone();
    testClone2();
    testClone3();
    testOOListFunctions();
    testDeleteElementFromList();

    testCASReset();

    testNullFSs();


    testFullyQualifiedFeatureNames();
    testTypeNameSpaces();

    testXMLTypeSystemReader();
    
  } catch (Exception & exc) {
    LOG("UIMA exception: " << endl << exc.asString());
    bSuccess = false;
  } catch (SAXParseException const & saxexc) {
    LOG("SAXException: ");
    LOG( "Error at file " << UnicodeStringRef( (UChar*) saxexc.getSystemId() )
         << ", line " << saxexc.getLineNumber()
         << ", char " << saxexc.getColumnNumber()
         << "\n  Message: " << UnicodeStringRef( (UChar*)saxexc.getMessage()) );
    bSuccess = false;
  }
  catch (XMLException const & xmlexc) {
    LOG("XMLException: " << UnicodeStringRef( (UChar*) xmlexc.getMessage()) );
    bSuccess = false;
  }
  /*
  catch (...) {
     LOG("Unknown exception" << endl);
     bSuccess = false;
  }
  */

  if (!bSuccess) {
    LOG("*********************************");
    LOG("  UIMA5 regression test failed!!");
    LOG("*********************************");
    return 1;
  }

  LOG("*********************************");
  LOG("  UIMA5 regression test finished");
  LOG("                  successfully.");
  LOG("*********************************");
  return 0;
}


