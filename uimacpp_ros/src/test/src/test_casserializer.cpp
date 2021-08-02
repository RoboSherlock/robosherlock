/** \file test_casserializer.cpp .

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
#define MAIN_TITLE            _TEXT("UIMA Test CAS Serializer")


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include "uima/pragmas.hpp" //must be first include to surpress warnings
#include "uima/api.hpp"
#include "uima/internal_casimpl.hpp"
#include "uima/casdefinition.hpp"
#include "uima/internal_casserializer.hpp"
#include "uima/internal_casdeserializer.hpp"
#include <sys/stat.h>

using namespace std;
using namespace uima;
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


template<class T>
bool equalVectors(vector<T> const & crV1, vector<T> const & crV2, util::ConsoleUI * pConsole) {
  bool bResult = true;
  if ( crV1.size() != crV2.size() ) {
    pConsole->getOutputStream()  << "size 1: " << crV1.size() << ", size 2: " << crV2.size() << endl;
    return false;
  }

  size_t i;
  for (i=0; i<crV1.size(); ++i) {
    bool bIsEqual = (crV1[i] == crV2[i]);

    if (!bIsEqual) {
      bResult = false;
    }
  }

  if (!bResult) {
    size_t j;
    for (j=0; j<crV1.size(); ++j) {
      if (crV1[j] != crV2[j]) {
        pConsole->getOutputStream() << j << ": " << crV1[j] << "         " << crV2[j] << endl;
      }
    }
  }
  return bResult;
}

/* ----------------------------------------------------------------------- */
/*       Tests                                                             */
/* ----------------------------------------------------------------------- */

void testSerializeDefinitions(util::ConsoleUI * pConsole) {
  uima::internal::CASDefinition * iv_newCASDefinition;
  uima::internal::SerializedCAS iv_serializedCAS;
  uima::Timer iv_serializeTimerDefinitions;

  /* create engine */
  ErrorInfo errInfo;
  icu::UnicodeString filename("toktest.xml");
  icu::UnicodeString fn = ResourceManager::resolveFilename(filename, filename);

  uima::TextAnalysisEngine * pEngine = TextAnalysisEngine::createTextAnalysisEngine
                                       (UnicodeStringRef(fn).asUTF8().c_str(), errInfo);
  if (pEngine == NULL ) {
    LOG("Error: " << errInfo.asString());
    ASSERT_OR_THROWEXCEPTION(false);
  }
  ASSERT_OR_THROWEXCEPTION(EXISTS(pEngine));
  ASSERT_OR_THROWEXCEPTION( errInfo.getErrorId() == UIMA_ERR_NONE );

  iv_newCASDefinition = uima::internal::CASDefinition::createCASDefinition(pEngine->getAnnotatorContext());

  pConsole->info("Serializing Type System and Index definitions CAS");
  uima::internal::CASSerializer serializer(true);
  iv_serializedCAS.reset();

  // serialize type system and index definition

  uima::internal::CASDeserializer deserializer;

  iv_serializeTimerDefinitions.reset();
  iv_serializeTimerDefinitions.start();
  serializer.serializeDefinitions(*iv_newCASDefinition, iv_serializedCAS);
  iv_serializeTimerDefinitions.stop();
#ifndef UIMA_SUPPRESS_TIMING
  pConsole->format("  Serialization of Definitions", uima::Timer::timeString( iv_serializeTimerDefinitions.getAccumulatedTime() ).c_str() );
#endif

  iv_serializeTimerDefinitions.reset();
  iv_serializeTimerDefinitions.start();
  deserializer.deserializeDefinitions(iv_serializedCAS , *iv_newCASDefinition );
  iv_serializeTimerDefinitions.stop();
#ifndef UIMA_SUPPRESS_TIMING
  pConsole->format("  Deserialization of Definitions", uima::Timer::timeString( iv_serializeTimerDefinitions.getAccumulatedTime() ).c_str() );
#endif


  // serializing again
  uima::internal::SerializedCAS serializedCAS2;

  serializer.serializeDefinitions(*iv_newCASDefinition, serializedCAS2);

  // checking
  pConsole->format("Checking", "type symbol table");
  ASSERT_OR_THROWEXCEPTION( equalVectors( iv_serializedCAS.getTypeSymbolTable(), serializedCAS2.getTypeSymbolTable(), pConsole ));
  pConsole->format("   success", true);

  pConsole->format("Checking", "type inheritance table");
  ASSERT_OR_THROWEXCEPTION( equalVectors( iv_serializedCAS.getTypeInheritanceTable(), serializedCAS2.getTypeInheritanceTable(), pConsole));
  pConsole->format("   success", true);

  pConsole->format("Checking", "feature symbol table");
  ASSERT_OR_THROWEXCEPTION( equalVectors( iv_serializedCAS.getFeatureSymbolTable(), serializedCAS2.getFeatureSymbolTable(), pConsole));
  pConsole->format("   success", true);

  pConsole->format("Checking", "feature offset table");
  ASSERT_OR_THROWEXCEPTION( equalVectors( iv_serializedCAS.getFeatureOffsetTable(), serializedCAS2.getFeatureOffsetTable(), pConsole));
  pConsole->format("   success", true);

  pConsole->format("Checking", "feature def table");
  ASSERT_OR_THROWEXCEPTION( equalVectors( iv_serializedCAS.getFeatureDefinitionTable(), serializedCAS2.getFeatureDefinitionTable(), pConsole));
  pConsole->format("   success", true);

  pConsole->format("Checking", "type priorities");
  ASSERT_OR_THROWEXCEPTION( equalVectors( iv_serializedCAS.getTypePriorityTable(), serializedCAS2.getTypePriorityTable(), pConsole));
  pConsole->format("   success", true);

  pConsole->format("Checking", "index IDs");
  ASSERT_OR_THROWEXCEPTION( equalVectors( iv_serializedCAS.getIndexIDTable(), serializedCAS2.getIndexIDTable(), pConsole ));
  pConsole->format("   success", true);

  pConsole->format("Checking", "index kinds");
  ASSERT_OR_THROWEXCEPTION( equalVectors(  iv_serializedCAS.getIndexKindTable(), serializedCAS2.getIndexKindTable(), pConsole) );
  pConsole->format("   success", true);

  pConsole->format("Checking", "comparator start");
  ASSERT_OR_THROWEXCEPTION( equalVectors( iv_serializedCAS.getComparatorStartTable(), serializedCAS2.getComparatorStartTable(), pConsole) );
  pConsole->format("   success", true);

  pConsole->format("Checking", "comparator def");
  ASSERT_OR_THROWEXCEPTION( equalVectors( iv_serializedCAS.getComparatorDefinitionTable(), serializedCAS2.getComparatorDefinitionTable(), pConsole) );
  pConsole->format("   success", true);

  delete iv_newCASDefinition;
  delete pEngine;

}

void testSerializeData(util::ConsoleUI * pConsole ) {
  pConsole->info("Serializing Document Data CAS");
  uima::Timer iv_serializeTimerData;

  ErrorInfo errInfo;
  icu::UnicodeString filename("toktest.xml");
  icu::UnicodeString fn = ResourceManager::resolveFilename(filename, filename);

  uima::TextAnalysisEngine * pEngine =
    TextAnalysisEngine::createTextAnalysisEngine(UnicodeStringRef(fn).asUTF8().c_str(), errInfo);
  if (pEngine == NULL ) {
    LOG("Error: " << errInfo.asString());
    ASSERT_OR_THROWEXCEPTION(false);
  }
  ASSERT_OR_THROWEXCEPTION(EXISTS(pEngine));
  ASSERT_OR_THROWEXCEPTION( errInfo.getErrorId() == UIMA_ERR_NONE );

  /* read in a file */
  icu::UnicodeString dataFile("tdoc_001_en_850.asc");
  icu::UnicodeString datafn = ResourceManager::resolveFilename(dataFile, dataFile);
  std::string dataFilename = UnicodeStringRef(datafn).asUTF8();
  /* open file for read */
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
  icu::UnicodeString ustrInputText(pBuffer, (int32_t)numread, "utf-8");
  delete[] pBuffer;

  /* set TCAS Document text */
  CAS * tcas = pEngine->newCAS();
  ASSERT_OR_THROWEXCEPTION( EXISTS(tcas) );

  tcas->setDocumentText(ustrInputText.getBuffer(), ustrInputText.length(), true);
  tcas->getDocumentAnnotation().setLanguage("en");

  /* call process */
  TyErrorId err = pEngine->process(*tcas);
  ASSERT_OR_THROWEXCEPTION( err == UIMA_ERR_NONE );

  uima::internal::CASImpl & cas = uima::internal::CASImpl::promoteCAS( *tcas);
  uima::internal::CASSerializer serializer(true);

  uima::internal::SerializedCAS iv_serializedCAS;
  iv_serializedCAS.reset();

  iv_serializeTimerData.reset();
  iv_serializeTimerData.start();
  serializer.serializeData(cas, iv_serializedCAS);
  iv_serializeTimerData.stop();
#ifndef UIMA_SUPPRESS_TIMING
  pConsole->format("  Serialization per-document data", uima::Timer::timeString( iv_serializeTimerData.getAccumulatedTime() ).c_str() );
#endif

  iv_serializeTimerData.reset();
  iv_serializeTimerData.start();
  size_t blobsz = serializer.getBlobSize(cas);
  char* blob = new char[blobsz];
  blobsz = serializer.getBlob(cas, blob, blobsz);
  iv_serializeTimerData.stop();
#ifndef UIMA_SUPPRESS_TIMING
  pConsole->format("  Blob serialization per-document data", uima::Timer::timeString( iv_serializeTimerData.getAccumulatedTime() ).c_str() );
#endif

  pConsole->formatHeader("CAS Serialization Results:");
  pConsole->format("  FS Heap size", (unsigned long)iv_serializedCAS.getFSHeapArray().size() );
  pConsole->format("  String Heap size", (unsigned long)iv_serializedCAS.getStringSymbolTable().size() );
  pConsole->format("  Number of Indexed FSs", (unsigned long)iv_serializedCAS.getIndexedFSs().size() );
  pConsole->format("  Blob size:", (unsigned long)blobsz);


//      iv_serializedCAS.print(cout);

  pConsole->info("Deserializing CAS");

//ee      uima::internal::TCASImpl * newTCAS = uima::internal::TCASImpl::createTCASImpl( *iv_newCASDefinition, getEngine().getAnnotatorContext() );
  uima::internal::CASImpl * newTCAS = &uima::internal::CASImpl::promoteCAS( *(pEngine->newCAS()) );

  uima::internal::CASDeserializer deserializer;

  /*
  iv_serializeTimerFSAndStringHeap.reset();
  iv_serializeTimerFSAndStringHeap.start();
  deserializer.deserializeFSHeapAndStringTable(iv_serializedCAS, *iv_newTCAS);
  iv_serializeTimerFSAndStringHeap.stop();
  getConsole().format("  Deserialization of FS/String Heap", uima::Timer::timeString( iv_serializeTimerFSAndStringHeap.getAccumulatedTime() ).c_str() );
  */

  iv_serializeTimerData.reset();
  iv_serializeTimerData.start();
  deserializer.deserializeData(iv_serializedCAS, *newTCAS);
  iv_serializeTimerData.stop();
#ifndef UIMA_SUPPRESS_TIMING
  getConsole().format("  Deserialization document data", uima::Timer::timeString( iv_serializeTimerData.getAccumulatedTime() ).c_str() );
#endif

  // deserialize the blob on top for a quick test
  iv_serializeTimerData.reset();
  iv_serializeTimerData.start();
  deserializer.deserializeBlob(blob, *newTCAS);
  iv_serializeTimerData.stop();
#ifndef UIMA_SUPPRESS_TIMING
  pConsole->format("  Deserialization blob", uima::Timer::timeString( iv_serializeTimerData.getAccumulatedTime() ).c_str() );
#endif

//      deserializer.deserializeDocument(iv_serializedCAS, *iv_newTCAS);

//      iv_newTCAS->getFSSystem().getLowlevelFSHeap().print(cout);
//      cout << "DocID: " << newTCAS.getDocumentAnnotation().getID() << endl;

//      cout << "========SERIALIZE AGAIN======" << endl;
  pConsole->info("Serializing CAS again");
  uima::internal::SerializedCAS serializedCAS2;
  uima::internal::CASSerializer serializer2(true);

  /*
  serializer2.serializeDocument(*iv_newTCAS, serializedCAS2);
  serializer2.serializeFSHeapAndStringHeap(*iv_newTCAS, serializedCAS2);
  serializer2.serializeIndexedFSs(*iv_newTCAS, serializedCAS2);
  */

  serializer2.serializeData(*newTCAS, serializedCAS2);

//    serCas2.print(cout);

  pConsole->info("Checking results");
  pConsole->format("Checking", "fs heap");
  ASSERT_OR_THROWEXCEPTION(equalVectors( iv_serializedCAS.getFSHeapArray(), serializedCAS2.getFSHeapArray(), pConsole ) );
  pConsole->format("   success", true);

  pConsole->format("Checking", "string symbol table");
  ASSERT_OR_THROWEXCEPTION( equalVectors( iv_serializedCAS.getStringSymbolTable(), serializedCAS2.getStringSymbolTable(), pConsole ) );
  pConsole->format("   success", true);

  pConsole->format("Checking", "document");
  ASSERT_OR_THROWEXCEPTION( iv_serializedCAS.getDocument() == serializedCAS2.getDocument() );
  pConsole->format("   success", true);

  pConsole->format("Checking", "indexed FSs");
  ASSERT_OR_THROWEXCEPTION( equalVectors( iv_serializedCAS.getIndexedFSs(), serializedCAS2.getIndexedFSs(), pConsole ));
  pConsole->format("   success", true);

  pConsole->info("Done");
  delete tcas;
  delete newTCAS;
  delete pEngine;
  delete blob;
}

/* ----------------------------------------------------------------------- */
/*       Main routine                                                      */
/* ----------------------------------------------------------------------- */

int main(int argc, char * argv[]) /*
---------------------------------- */
{
  /* create console */
  util::ConsoleUI * pConsole = new util::ConsoleUI(argc, argv, MAIN_TITLE, "\n");
  assert(EXISTS(pConsole));
  #if !defined(NDEBUG) && defined(_MSC_VER)
   //int iRetVal = _CrtSetBreakAlloc(124613);
  #endif
  /* create a UIMA resource */
  try {
    ResourceManager::createInstance(MAIN_TITLE);
    testSerializeDefinitions(pConsole);
    testSerializeData(pConsole);
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

/* <EOF> */



