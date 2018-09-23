/** \file test_compositeindex.cpp .

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


// define this first to get your application name added to command line
// used inside "cmdline_driver_args.h"
#define MAIN_TITLE            _TEXT("UIMA Test iterators")

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include "uima/pragmas.hpp" //must be first include to surpress warnings
#include "uima/api.hpp"
#include "uima/internal_casimpl.hpp"
///#include "uima/lowlevel_indexiterator.hpp"
///#include "uima/internal_fspromoter.hpp"
#include "uima/tt_types.hpp"
#include <sys/stat.h>
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */
#ifndef NDEBUG
#define ASSERT_OR_THROWEXCEPTION(x) assert(x)
#else
#define ASSERT_OR_THROWEXCEPTION(x) if (!(x)) { cerr << __FILE__ << ": Error in line " << __LINE__ << endl; exit(1); }
#endif

#define LOG(x) cout << __FILE__ << __LINE__ << ": " << x << endl

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */
using namespace uima;
using namespace std;

bool checkIndex(AnnotationFS const & anFS, ANIndex const & ix) {
  ANIterator it = ix.iterator();
  for (it.moveToFirst(); it.isValid(); it.moveToNext() ) {
    if (it.get() == anFS) {
      return true;
    }
  }
  return false;
}

/*-----------------------------------------------------------------------------*/
uima::lowlevel::IndexIterator * createIterator(uima::lowlevel::IndexABase const & crAnnIndex,
    set<uima::lowlevel::TyFSType> const & crTypes) {
  if (crTypes.size() == 0) {
    return crAnnIndex.createIterator();
  } else {
    return crAnnIndex.createTypeSetIterator(crTypes);
  }
}

void createNewIterator(uima::lowlevel::IndexIterator*& rpIt,
                       uima::lowlevel::IndexABase const & crAnnIndex,
                       set<uima::lowlevel::TyFSType> const & crTypes,
                         bool bUseOnlyOneIterator )  {
  uima::lowlevel::IndexIterator * pOldIt = rpIt;
  if (rpIt == NULL ) {
    rpIt = createIterator(crAnnIndex, crTypes);
    return;
  }
  if (! bUseOnlyOneIterator) {
    delete rpIt;
    rpIt = createIterator(crAnnIndex, crTypes);
  } else {
    assert( rpIt == pOldIt );
  }
}


void checkIterator(uima::lowlevel::IndexABase const & crIndex,
                   set<uima::lowlevel::TyFSType> const & crTypes,
                     bool bUseOnlyOneIterator,
                     uima::internal::CASImpl * tcas,
                     util::ConsoleUI * pConsole) {
  ////uima::internal::TCASImpl * tcas = iv_pTCASImpl;

  bool bIsIteratorOverTypeSet = ( crTypes.size() != 0 );

  int iSize = 0;
  if (!bIsIteratorOverTypeSet) {
    iSize = crIndex.getSize();
  }
  pConsole->format("Index size", iSize);

//      uima::lowlevel::TyFS * arFSs = new uima::lowlevel::TyFS[iSize];
  vector<uima::lowlevel::TyFS> arFSs;
  int j;

  uima::lowlevel::IndexIterator * pIt = NULL;
  createNewIterator(pIt, crIndex, crTypes, bUseOnlyOneIterator);

  // fill array with one single forward movement
  pConsole->format("Filling array", "");
  for (pIt->moveToFirst(); pIt->isValid(); pIt->moveToNext() ) {
    ASSERT_OR_THROWEXCEPTION( pIt->isValid() );
    arFSs.push_back( pIt->get() );
    if (bIsIteratorOverTypeSet) {
      ++iSize;
    }
  }
  ASSERT_OR_THROWEXCEPTION( ! pIt->isValid() );
  ASSERT_OR_THROWEXCEPTION( arFSs.size() == iSize );
  pConsole->format("Array full", true);

  /////////////////////////////////////////////////
  // check array with forward movement
  createNewIterator(pIt, crIndex, crTypes, bUseOnlyOneIterator);
  pConsole->format("Checking forward iterator", "");
  pIt->moveToFirst();
  for (j=0; j<iSize; ++j) {
    ASSERT_OR_THROWEXCEPTION( pIt->isValid() );
    ASSERT_OR_THROWEXCEPTION( arFSs[j] == pIt->get() );
    pIt->moveToNext();
  }
  ASSERT_OR_THROWEXCEPTION( ! pIt->isValid() );
  pConsole->format("Forward iterator", true);

  /////////////////////////////////////////////////
  // check array with reverse movement
  createNewIterator(pIt, crIndex, crTypes, bUseOnlyOneIterator);
  pConsole->format("Checking reverse iterator", "");
  pIt->moveToLast();
  for (j=iSize-1; j>=0; --j) {
    ASSERT_OR_THROWEXCEPTION( pIt->isValid() );
    ASSERT_OR_THROWEXCEPTION( arFSs[j] == pIt->get() );
    pIt->moveToPrevious();
  }
  ASSERT_OR_THROWEXCEPTION( ! pIt->isValid() );
  pConsole->format("Reverse iterator", true);

  ///////////////////////////////////////////////
  // check array with one single forth and back movement
  createNewIterator(pIt, crIndex, crTypes, bUseOnlyOneIterator);
  pConsole->format("Checking one forth and back movement", "");
  pIt->moveToFirst();
  for (j=0; j<iSize-1; ++j) {
//            iv_pclConsole->format("  checking fs", j);
    ASSERT_OR_THROWEXCEPTION( pIt->isValid() );
    ASSERT_OR_THROWEXCEPTION( arFSs[j] == pIt->get() );
    pIt->moveToNext();
  }
  pConsole->format("    forth movement", true);

  for (j=iSize-1; j>=0; --j) {
    ASSERT_OR_THROWEXCEPTION( pIt->isValid() );
    ASSERT_OR_THROWEXCEPTION( arFSs[j] == pIt->get() );
    pIt->moveToPrevious();
  }
  ASSERT_OR_THROWEXCEPTION( ! pIt->isValid() );
  pConsole->format("    back movement", true);

  /////////////////////////////////////////////////
  // check array with many back and forth movements
  createNewIterator(pIt, crIndex, crTypes, bUseOnlyOneIterator);
  pConsole->format("Checking many forth and back movements", "");
  pIt->moveToFirst();
  for (j=0; j<iSize; ++j) {
    ASSERT_OR_THROWEXCEPTION( pIt->isValid() );
    ASSERT_OR_THROWEXCEPTION( arFSs[j] == pIt->get() );

    if (j>1) {
      int k;
      for (k=j; k>=2; --k) {
        ASSERT_OR_THROWEXCEPTION( pIt->isValid() );
        ASSERT_OR_THROWEXCEPTION( pIt->get() == arFSs[k] );
        pIt->moveToPrevious();
        ASSERT_OR_THROWEXCEPTION( pIt->isValid() );
        ASSERT_OR_THROWEXCEPTION( pIt->get() == arFSs[k-1] );
        uima::lowlevel::TyFS prevFS = pIt->get();
        pIt->moveToPrevious();
        ASSERT_OR_THROWEXCEPTION( pIt->isValid() );
        ASSERT_OR_THROWEXCEPTION( pIt->get() == arFSs[k-2] );
        pIt->moveToNext();
        ASSERT_OR_THROWEXCEPTION( pIt->isValid() );
        ASSERT_OR_THROWEXCEPTION( pIt->get() == arFSs[k-1] );
        ASSERT_OR_THROWEXCEPTION( prevFS == pIt->get() );
      }
      ASSERT_OR_THROWEXCEPTION(k==1);
      for (k=1; k<j; ++k) {
        ASSERT_OR_THROWEXCEPTION( pIt->isValid() );
        ASSERT_OR_THROWEXCEPTION( pIt->get() == arFSs[k] );
        pIt->moveToNext();
        ASSERT_OR_THROWEXCEPTION( pIt->get() == arFSs[k+1] );
        ASSERT_OR_THROWEXCEPTION( pIt->isValid() );
      }
      ASSERT_OR_THROWEXCEPTION(k==j);
      ASSERT_OR_THROWEXCEPTION( arFSs[j] == pIt->get() );
    }

    if (j<iSize-1) {
      ASSERT_OR_THROWEXCEPTION( arFSs[j] == pIt->get() );
      int k;
      for (k=j; k<iSize-2; ++k) {
        ASSERT_OR_THROWEXCEPTION( pIt->isValid() );
        ASSERT_OR_THROWEXCEPTION( pIt->get() == arFSs[k] );
        pIt->moveToNext();
        ASSERT_OR_THROWEXCEPTION( pIt->isValid() );
        ASSERT_OR_THROWEXCEPTION( pIt->get() == arFSs[k+1] );
        uima::lowlevel::TyFS prevFS = pIt->get();
        pIt->moveToNext();
        ASSERT_OR_THROWEXCEPTION( pIt->isValid() );
        ASSERT_OR_THROWEXCEPTION( pIt->get() == arFSs[k+2] );
        pIt->moveToPrevious();
        ASSERT_OR_THROWEXCEPTION( pIt->isValid() );
        ASSERT_OR_THROWEXCEPTION( pIt->get() == arFSs[k+1] );
        ASSERT_OR_THROWEXCEPTION( prevFS == pIt->get() );
      }
      ASSERT_OR_THROWEXCEPTION( k == iSize-2 );
      for (k=iSize-2; k>j; --k) {
        ASSERT_OR_THROWEXCEPTION( pIt->isValid() );
        pIt->moveToPrevious();
        ASSERT_OR_THROWEXCEPTION( pIt->isValid() );
      }
      ASSERT_OR_THROWEXCEPTION( arFSs[j] == pIt->get() );
    }
    ASSERT_OR_THROWEXCEPTION( arFSs[j] == pIt->get() );

    pIt->moveToNext();
  }
  pConsole->format("  Many forth and back movements", true);


  /////////////////////////////////////////////////
  // check peek() methods
  pConsole->format("Checking peek() functions", "");

  uima::lowlevel::IndexIterator * it2 = pIt->clone();
  uima::FSIterator fsit = uima::internal::FSPromoter::promoteIterator(it2, *tcas );
  j=0;
  for (fsit.moveToFirst(); fsit.isValid(); fsit.moveToNext()) {
    // test peekPrevious
    ASSERT_OR_THROWEXCEPTION( arFSs[j] == uima::internal::FSPromoter::demoteFS(fsit.get()) );
    uima::FeatureStructure fsPrev = fsit.peekPrevious();
    ASSERT_OR_THROWEXCEPTION( arFSs[j] == uima::internal::FSPromoter::demoteFS(fsit.get()) );
    if (j == 0) {
      ASSERT_OR_THROWEXCEPTION(!fsPrev.isValid());
    } else {
      ASSERT_OR_THROWEXCEPTION(fsPrev.isValid());
      ASSERT_OR_THROWEXCEPTION( uima::internal::FSPromoter::demoteFS( fsPrev ) == arFSs[j-1] );
    }

    // test peekNext
    ASSERT_OR_THROWEXCEPTION( arFSs[j] == uima::internal::FSPromoter::demoteFS(fsit.get()) );
    uima::FeatureStructure fsNext = fsit.peekNext();
    ASSERT_OR_THROWEXCEPTION( arFSs[j] == uima::internal::FSPromoter::demoteFS(fsit.get()) );
    if (j == (arFSs.size()-1)) {
      ASSERT_OR_THROWEXCEPTION(!fsNext.isValid());
    } else {
      ASSERT_OR_THROWEXCEPTION(fsNext.isValid());
      ASSERT_OR_THROWEXCEPTION( uima::internal::FSPromoter::demoteFS( fsNext ) == arFSs[j+1] );
    }

    ++j;
  }
  ASSERT_OR_THROWEXCEPTION( j == arFSs.size() );
  pConsole->format("  peek() functions", true);

  /////////////////////////////////////////////////////////
  // cleanup
  delete pIt;
//      delete[] arFSs;
//      arFSs = NULL;

}


void checkIterators(bool bUseOnlyOneIterator, util::ConsoleUI * pConsole,
                    uima::internal::CASImpl * tcas) {
  ///uima::internal::TCASImpl * tcas = iv_pTCASImpl;
  uima::lowlevel::IndexRepository const & crIndexRep = tcas->getIndexRepository();

  uima::lowlevel::TypeSystem const & crTypeSystem = tcas->getHeap().getTypeSystem();

  vector<icu::UnicodeString> allIndexes = crIndexRep.getAllIndexIDs();

  size_t n;
  for (n=0; n<allIndexes.size(); ++n) {

    icu::UnicodeString const & ixid = allIndexes[n];

    uima::lowlevel::TyFSType tyType = crIndexRep.getIndexDefinition().getTypeForIndex(ixid);
    pConsole->format("Processing index with ID", UnicodeStringRef(ixid).asUTF8().c_str() );
    pConsole->format("     on type", UnicodeStringRef(crTypeSystem.getTypeName(tyType)).asUTF8().c_str() );

    vector<uima::lowlevel::TyFSType> subsumedTypes;
    crTypeSystem.getSubsumedTypes(tyType, subsumedTypes);

    size_t i;
    for (i=0; i<subsumedTypes.size(); ++i) {

      uima::lowlevel::TyFSType tySubType = subsumedTypes[i];
      ASSERT_OR_THROWEXCEPTION( tcas->getHeap().getTypeSystem().isValidType( tySubType ) );

      pConsole->format("Processing type", UnicodeStringRef(crTypeSystem.getTypeName(tySubType)).asUTF8().c_str() );
      pConsole->format("Use new iterator for every test", ! bUseOnlyOneIterator);

      ASSERT_OR_THROWEXCEPTION( crTypeSystem.subsumes( tyType, tySubType ) );
      ASSERT_OR_THROWEXCEPTION( crTypeSystem.subsumes( crIndexRep.getIndexDefinition().getTypeForIndex(ixid), tySubType ) );

      uima::lowlevel::IndexABase const & crIndex = crIndexRep.getLowlevelIndex(ixid, tySubType);

      set<uima::lowlevel::TyFSType> indexTypes;

      // check normal iterators
      ASSERT_OR_THROWEXCEPTION( indexTypes.size() == 0 );
      checkIterator(crIndex, indexTypes, bUseOnlyOneIterator, tcas, pConsole);

      // check type set iterators
      vector<uima::lowlevel::TyFSType> subTypes;
      crTypeSystem.getSubsumedTypes(tySubType, subTypes);
      if ( subTypes.size() > 1) {
        pConsole->format("Processing iterator over type sets", "");

        size_t j = 0;
        for (j=0; j<subTypes.size(); ++j) {
          if (subTypes[j] != tyType) {
            indexTypes.insert(subTypes[j]);

          }
        }

        // make vector shorter to make test faster
        const int MAXTYPESETSIZE = 15;
        if (indexTypes.size() > MAXTYPESETSIZE) {
          for (j = indexTypes.size() - MAXTYPESETSIZE; j>0; --j) {
            set<uima::lowlevel::TyFSType>::iterator itLast = indexTypes.end();
            --itLast;
            indexTypes.erase( itLast );
          }
        }

        while (indexTypes.size() > 0 ) {
          set<uima::lowlevel::TyFSType>::const_iterator cit;
          for (cit = indexTypes.begin(); cit != indexTypes.end(); ++cit) {
            pConsole->format("     choosing type", UnicodeStringRef(crTypeSystem.getTypeName(*cit)).asUTF8().c_str() );
          }

          checkIterator(crIndex, indexTypes, bUseOnlyOneIterator, tcas, pConsole);
          indexTypes.erase( indexTypes.begin() );
        }
      }
    }
  }
}

//do we need this (bi)
void defect_011303_subiterator(uima::ANIterator & forConstituent, uima::AnnotationFS const & sent, util::ConsoleUI * pConsole) {
  int subfsnum = 0;
  for ( forConstituent.moveToFirst(); forConstituent.isValid() ; forConstituent.moveToNext() ) {
    subfsnum++;
    uima::AnnotationFS annot = forConstituent.get();
    ASSERT_OR_THROWEXCEPTION( annot.getBeginPosition() >= sent.getBeginPosition() );
    ASSERT_OR_THROWEXCEPTION( annot.getEndPosition() <= sent.getEndPosition() );
    ASSERT_OR_THROWEXCEPTION( annot != sent );
  }
  pConsole->format("number of sub FSs", subfsnum);

}
//do we need this test (bi)
void defect_011303_subiterator(uima::AnnotationFS const & sent, Type const & tAnnotation, EnIteratorAmbiguity ambiguity, util::ConsoleUI * pConsole) {

  // the next three variants should do the same, but the assignment operators/copy constructors
  // may be called differently. Different compilers may create different code
  // in particular for variants 1 and 2.

  // variant 1
  uima::ANIterator forConstituent1( sent.subIterator( tAnnotation, ambiguity ) );
  defect_011303_subiterator(forConstituent1, sent, pConsole);

  // variant 2
  uima::ANIterator forConstituent2 =  sent.subIterator( tAnnotation, ambiguity );
  defect_011303_subiterator(forConstituent2, sent, pConsole);

  // variant 3
  uima::ANIterator forConstituent3_tmp =  sent.subIterator( tAnnotation, ambiguity );
  uima::ANIterator forConstituent3 = forConstituent3_tmp;
  defect_011303_subiterator(forConstituent3, sent, pConsole);

}

// defect in Mary's mail from 01/13/2003
// do we need this test (bi)
void defect_011303(uima::CAS const & tcas, util::ConsoleUI * pConsole) {
  pConsole->format("Checking Mary's defect 01/13/2003", "");
  uima::Type tSent = tcas.getTypeSystem().getType(uima::TT::TYPE_NAME_SENTENCE_ANNOTATION);
  uima::Type tAnnotation = tcas.getTypeSystem().getType(uima::CAS::TYPE_NAME_ANNOTATION);

  uima::ANIterator forSent =
    tcas.getAnnotationIndex( tSent ).iterator();
  int sentnum = 1;
  for ( forSent.moveToFirst() ; forSent.isValid() ; forSent.moveToNext() ) {
    pConsole->format("Sentence", sentnum);
    sentnum++;
    uima::AnnotationFS sent = forSent.get();

    defect_011303_subiterator(sent, tAnnotation, enAmbiguous,pConsole);
    defect_011303_subiterator(sent, tAnnotation, enUnambiguous,pConsole);

  }
  pConsole->format("Done Checking Mary's defect 01/13/2003", "");
}

void testTypeSetIterator(uima::CAS & tcas, util::ConsoleUI * pConsole) {
  pConsole->format("Checking Type set iterators", "");

  uima::Type tAnnotation = tcas.getTypeSystem().getType(uima::CAS::TYPE_NAME_ANNOTATION);
  uima::Type tSent = tcas.getTypeSystem().getType(uima::TT::TYPE_NAME_SENTENCE_ANNOTATION);
  uima::Type tPar = tcas.getTypeSystem().getType(uima::TT::TYPE_NAME_PARAGRAPH_ANNOTATION);

  set<Type> s;
  s.insert(tSent);
  s.insert(tPar);

  uima::FSIterator it = tcas.getAnnotationIndex( tAnnotation ).typeSetIterator(s);
  for ( it.moveToFirst() ; it.isValid() ; it.moveToNext() ) {
    uima::FeatureStructure fs = it.get();
    uima::Type t = fs.getType();
    ASSERT_OR_THROWEXCEPTION( (t == tSent) || (t == tPar) );
  }
  pConsole->format("Type set iterators check successful?", true);

}

void checkTypePriority(uima::AnnotationFS const & crFS1, uima::AnnotationFS const & crFS2)  {
  if ( ! crFS1.isValid() ) {
    return;
  }

  if ( (crFS1.getBeginPosition() == crFS2.getBeginPosition())
       && (crFS1.getEndPosition() == crFS2.getEndPosition() ) ) {
    uima::lowlevel::TyFSType t1 = uima::internal::FSPromoter::demoteType( crFS1.getType() );
    uima::lowlevel::TyFSType t2 = uima::internal::FSPromoter::demoteType( crFS2.getType() );

    // current implementation of type priority simply assumes this
    ASSERT_OR_THROWEXCEPTION( t1 <= t2 );
  }

}

void checkSubIterators(char const * mainIteratorTypeName, char const * subIteratorTypeName, EnIteratorAmbiguity ambiguity, uima::CAS const & crTCAS, util::ConsoleUI * pConsole) {

  pConsole->format("Main iterator type", mainIteratorTypeName);
  pConsole->format("Sub iterator type", subIteratorTypeName);
  pConsole->format("Ambiguous", (ambiguity == enAmbiguous));

  uima::Type mainIteratorType = crTCAS.getTypeSystem().getType(mainIteratorTypeName);
  uima::Type subIteratorType = crTCAS.getTypeSystem().getType(subIteratorTypeName);
  ASSERT_OR_THROWEXCEPTION( mainIteratorType.isValid() );
  ASSERT_OR_THROWEXCEPTION( subIteratorType.isValid() );
  uima::ANIndex ix = crTCAS.getAnnotationIndex(mainIteratorType);
  uima::ANIterator it = ix.iterator();
  for (it.moveToFirst(); it.isValid(); it.moveToNext()) {

    uima::AnnotationFS mainFS = it.get();

    // don't use tokens as main type
    if (mainFS.getType() != crTCAS.getTypeSystem().getType(uima::TT::TYPE_NAME_TOKEN_ANNOTATION)) {

      pConsole->format("Found main annotation from", (unsigned long)mainFS.getBeginPosition());
      pConsole->format("                        to", (unsigned long)mainFS.getEndPosition());
      UIMA_TPRINT("Found sentence '" << sentFS.getCoveredText() << "' from " << sentFS.getBeginPosition() << " to " << sentFS.getEndPosition() );
      uima::ANIterator subIt = mainFS.subIterator(subIteratorType, ambiguity);

      uima::AnnotationFS subFS;
      pConsole->format("Traversing sub iterator from left to right", "");
      for (subIt.moveToFirst(); subIt.isValid(); subIt.moveToNext() ) {
        checkTypePriority( subFS, subIt.get() );
        subFS = subIt.get();
        UIMA_TPRINT("Found token '" << tokFS.getCoveredText() << "' from " << tokFS.getBeginPosition() << " to " << tokFS.getEndPosition());

        ASSERT_OR_THROWEXCEPTION( subFS.getBeginPosition() >= mainFS.getBeginPosition() );
        ASSERT_OR_THROWEXCEPTION( subFS.getBeginPosition() < mainFS.getEndPosition() );

        ASSERT_OR_THROWEXCEPTION( subFS != mainFS );
      }
      pConsole->format("Success?", true);

      pConsole->format("Traversing sub iterator from right to left", "");
      for (subIt.moveToLast(); subIt.isValid(); subIt.moveToPrevious() ) {
        subFS = subIt.get();
        UIMA_TPRINT("Found token '" << tokFS.getCoveredText() << "' from " << tokFS.getBeginPosition() << " to " << tokFS.getEndPosition() );
        ASSERT_OR_THROWEXCEPTION( subFS.getBeginPosition() >= mainFS.getBeginPosition() );
        ASSERT_OR_THROWEXCEPTION( subFS.getBeginPosition() < mainFS.getEndPosition() );
      }
      pConsole->format("Success?", true);


      pConsole->format("Moving sub iterator to last and beyond", "");
      subIt.moveToLast();                 // That's a UIMA subiterator

      if ( subIt.isValid() ) {
        uima::AnnotationFS remember = subIt.get();

        subIt.moveToNext();

        if ( subIt.isValid() ) {
          ostream& os = pConsole->getOutputStream();
          os << "Remembered: ";                           // This trace actually gets executed
          os << remember.getType().getName() << " between " << remember.getBeginPosition() << " and " << remember.getEndPosition() << endl;
          remember.getCoveredText().toSingleByteStream( os );
          os << endl;
          os << "Supposed to be invalid: ";
          uima::AnnotationFS current = subIt.get();
          os << current.getType().getName() << " between " << current.getBeginPosition() << " and " << current.getEndPosition() << endl;
          current.getCoveredText().toSingleByteStream(os);
          os << endl;
        }

        ASSERT_OR_THROWEXCEPTION( !subIt.isValid());           // And this assertion fails
      }
      pConsole->format("checkSubIterators Success?", true);
    }
  }
}

/* ----------------------------------------------------------------------- */
/* TESTS                                                                   */
/* ----------------------------------------------------------------------- */
/**
 *
 * test subiterators
 */
void testSubIterators(util::ConsoleUI * pConsole) {
  pConsole->info("Testing SubIterators");

  ErrorInfo errInfo;
  UnicodeString filename("toktest.xml");
  UnicodeString fn = ResourceManager::resolveFilename(filename, filename);

  /* create engine */
  uima::TextAnalysisEngine * pEngine =
    TextAnalysisEngine::createTextAnalysisEngine(UnicodeStringRef(fn).asUTF8().c_str(), errInfo);
  if (pEngine == NULL ) {
    LOG("Error: " << errInfo.asString());
    ASSERT_OR_THROWEXCEPTION(false);
  }
  ASSERT_OR_THROWEXCEPTION(EXISTS(pEngine));
  ASSERT_OR_THROWEXCEPTION( errInfo.getErrorId() == UIMA_ERR_NONE );

  UnicodeString dataFile("tdoc_001_en_850.asc");
  UnicodeString datafn = ResourceManager::resolveFilename(dataFile, dataFile);
  std::string dataFilename = UnicodeStringRef(datafn).asUTF8();
  /* read in file contents and set TCAS Document text */
  FILE * pFile = fopen( dataFilename.c_str(),"rb");
  ASSERT_OR_THROWEXCEPTION(pFile != NULL );

  /* allocate buffer for file contents */
  struct stat stat_result;
  stat(dataFilename.c_str(), &stat_result);
  int filesize = stat_result.st_size;
  char * pBuffer = new char[filesize+1];
  ASSERT_OR_THROWEXCEPTION(pBuffer != NULL );

  /* read the file */
  size_t numread = fread(pBuffer,1,filesize,pFile);
  fclose(pFile);

  /* convert to unicode and set tcas document text*/
  UnicodeString ustrInputText(pBuffer, (int32_t)numread, "utf-8");
  delete[] pBuffer;
  /* set TCAS Document text */
  CAS * tcas = pEngine->newCAS();
  ASSERT_OR_THROWEXCEPTION( EXISTS(tcas) );
  tcas->setDocumentText(ustrInputText.getBuffer(), ustrInputText.length(), true);
  tcas->getDocumentAnnotation().setLanguage("en");

  /* call process */
  TyErrorId err = pEngine->process(*tcas);
  ASSERT_OR_THROWEXCEPTION( err == UIMA_ERR_NONE );

  defect_011303(*tcas, pConsole);
  testTypeSetIterator(*tcas, pConsole);

  checkSubIterators(uima::TT::TYPE_NAME_SENTENCE_ANNOTATION, uima::TT::TYPE_NAME_TOKEN_ANNOTATION, enAmbiguous, *tcas, pConsole );
  checkSubIterators(uima::CAS::TYPE_NAME_ANNOTATION, uima::TT::TYPE_NAME_TOKEN_ANNOTATION, enAmbiguous, *tcas, pConsole );
  checkSubIterators(uima::TT::TYPE_NAME_LEXICAL_ANNOTATION, uima::TT::TYPE_NAME_LEXICAL_ANNOTATION, enAmbiguous, *tcas, pConsole );

  checkSubIterators(uima::TT::TYPE_NAME_SENTENCE_ANNOTATION, uima::TT::TYPE_NAME_TOKEN_ANNOTATION, enUnambiguous, *tcas, pConsole );
  checkSubIterators(uima::CAS::TYPE_NAME_ANNOTATION, uima::TT::TYPE_NAME_TOKEN_ANNOTATION, enUnambiguous, *tcas, pConsole );
  checkSubIterators(uima::TT::TYPE_NAME_LEXICAL_ANNOTATION, uima::TT::TYPE_NAME_LEXICAL_ANNOTATION, enUnambiguous, *tcas, pConsole );
  delete tcas;
  delete pEngine;
  pConsole->info("Sub Iterators test finished.");
}
/**
 * test iterators
 *
 */
void testIterators(util::ConsoleUI * pConsole) {

  pConsole->info("Testing Iterators");

  ErrorInfo errInfo;
  UnicodeString filename("toktest.xml");
  UnicodeString fn = ResourceManager::resolveFilename(filename, filename);

  /* create engine */
  uima::TextAnalysisEngine * pEngine =
    TextAnalysisEngine::createTextAnalysisEngine(UnicodeStringRef(fn).asUTF8().c_str(), errInfo);
  if (pEngine == NULL ) {
    LOG("Error: " << errInfo.asString());
    ASSERT_OR_THROWEXCEPTION(false);
  }
  ASSERT_OR_THROWEXCEPTION(EXISTS(pEngine));
  ASSERT_OR_THROWEXCEPTION( errInfo.getErrorId() == UIMA_ERR_NONE );

  UnicodeString dataFile("toktest.xml");
  UnicodeString datafn = ResourceManager::resolveFilename(dataFile, dataFile);
  std::string dataFilename = UnicodeStringRef(datafn).asUTF8();
  /* read in file contents and set TCAS Document text */
  FILE * pFile = fopen( dataFilename.c_str(),"rb");
  ASSERT_OR_THROWEXCEPTION(pFile != NULL );

  /* allocate buffer for file contents */
  struct stat stat_result;
  stat(dataFilename.c_str(), &stat_result);
  int filesize = stat_result.st_size;
  char * pBuffer = new char[filesize+1];
  ASSERT_OR_THROWEXCEPTION(pBuffer != NULL );

  /* read the file */
  size_t numread = fread(pBuffer,1,filesize,pFile);
  fclose(pFile);

  /* convert to unicode and set tcas document text*/
  UnicodeString ustrInputText(pBuffer, (int32_t)numread, "utf-8");
  delete[] pBuffer;
  /* set TCAS Document text */
  CAS * tcas = pEngine->newCAS();
  ASSERT_OR_THROWEXCEPTION( EXISTS(tcas) );
  tcas->setDocumentText(ustrInputText.getBuffer(), ustrInputText.length(), true);
  tcas->getDocumentAnnotation().setLanguage("en");

  /* call process */
  TyErrorId err = pEngine->process(*tcas);
  ASSERT_OR_THROWEXCEPTION( err == UIMA_ERR_NONE );

  /* checkIterators twice - true/false */
  checkIterators(true, pConsole, &(uima::internal::CASImpl::promoteCAS(*tcas)));
  checkIterators(false, pConsole,  &(uima::internal::CASImpl::promoteCAS(*tcas)));
  delete tcas;
  delete pEngine;
  pConsole->info("Iterators test finished");
}

/**
 *
 * Test composite index
 *
 */
void testCaching(util::ConsoleUI * pConsole) {

  pConsole->info("Testing Caching");

  ErrorInfo errInfo;
  UnicodeString filename("toktest.xml");
  UnicodeString fn = ResourceManager::resolveFilename(filename, filename);

  uima::TextAnalysisEngine * pEngine =
    TextAnalysisEngine::createTextAnalysisEngine(UnicodeStringRef(fn).asUTF8().c_str(), errInfo);
  if (pEngine == NULL ) {
    LOG("Error: " << errInfo.asString());
    ASSERT_OR_THROWEXCEPTION(false);
  }
  ASSERT_OR_THROWEXCEPTION(EXISTS(pEngine));
  ASSERT_OR_THROWEXCEPTION( errInfo.getErrorId() == UIMA_ERR_NONE );

  /* set TCAS Document text */
  CAS * tcas = pEngine->newCAS();
  ASSERT_OR_THROWEXCEPTION( EXISTS(tcas) );
  UnicodeString ustrInputText("This is test doc for testing iteration.");
  tcas->setDocumentText(ustrInputText.getBuffer(), ustrInputText.length(), true);
  tcas->getDocumentAnnotation().setLanguage("en");

  /* call process */
  TyErrorId err = pEngine->process(*tcas);
  ASSERT_OR_THROWEXCEPTION( err == UIMA_ERR_NONE );

  // iterator over a composite index
  FSIndexRepository & ixRep = tcas->getIndexRepository();
  Type annType = tcas->getTypeSystem().getType(CAS::TYPE_NAME_ANNOTATION);
  ANIndex ix = tcas->getAnnotationIndex(annType);

  vector<Type> subTypes;
  annType.getSubTypes(subTypes);
  size_t i;
  // now create some annotations and check they are in the index
  for (i=0; i<subTypes.size(); ++i) {
    AnnotationFS an1 = tcas->createAnnotation(subTypes[i], i, i+10);
    AnnotationFS an2 = tcas->createAnnotation(subTypes[i], i+1, i+11);

    ixRep.addFS(an1);
    ixRep.addFS(an2);

    ASSERT_OR_THROWEXCEPTION( checkIndex(an1, ix) );
    ASSERT_OR_THROWEXCEPTION( checkIndex(an2, ix) );
  }
  delete tcas;
  delete pEngine;
  pConsole->info("Caching test finished");
  return;
}


/* ----------------------------------------------------------------------- */
/*      UTILS                                                  */
/* ----------------------------------------------------------------------- */
void displayException(util::ConsoleUI & console, Exception & crclException)
/* ----------------------------------------------------------------------- */
{
  console.formatHeader(_TEXT("Exception"));
  console.format(_TEXT("Exception error id"), crclException.getErrorInfo().getErrorId());
  console.format(_TEXT("Exception name"), crclException.getName());
  console.format(_TEXT("Exception what"), crclException.what());
  console.format(_TEXT("Exception message"), crclException.getErrorInfo().getMessage().asString().c_str());
  console.formatBool(_TEXT("Exception recoverable"), crclException.getErrorInfo().isRecoverable());
  const TCHAR * cpszSavePrefix = ErrorInfo::getGlobalErrorInfoIndent();
  ErrorInfo::setGlobalErrorInfoIndent("  ");
  console.getOutputStream() << crclException.getErrorInfo() << endl;
  ErrorInfo::setGlobalErrorInfoIndent(cpszSavePrefix);
}

/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Main routine                                                      */
/* ----------------------------------------------------------------------- */

int main(int argc, char * argv[]) /*
---------------------------------- */
{
  /* create console */
  util::ConsoleUI * pConsole = new util::ConsoleUI(argc, argv, MAIN_TITLE, "\n");
  assert(EXISTS(pConsole));

  /* create a UIMA resource */
  try {
    ResourceManager::createInstance(MAIN_TITLE);
    testCaching(pConsole);
    testIterators(pConsole);
    testSubIterators(pConsole);
    ResourceManager::deleteInstance();
  } catch (Exception & rclException) {
    displayException(*pConsole, rclException);
    pConsole->error("Unexpected UIMA exception");
    return 1;
  } catch (exception & rclException) {
    pConsole->error(rclException.what());
    return 1;
  }
  delete pConsole;
  return(0);
}


