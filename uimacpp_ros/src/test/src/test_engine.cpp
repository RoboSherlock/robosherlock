/** @name uimatest_engine.cpp

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

#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <iomanip>
using namespace std;

#include "uima/consoleui.hpp"
#include "uima/assertmsg.h"

#include "uima/conui.hpp"
#include "uima/doc_buffer.hpp"
#include "uima/ftools.hpp"

#include "uima/internal_aggregate_engine.hpp"
#include "xercesc/util/XMLException.hpp"

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

#define MAIN_TITLE            _TEXT("Commandline Tester with extended tests")

#define MAIN_DEFAULT_CCSID            1208      /* v must match v */
#define MAIN_DEFAULT_CCSID_STR       "cp1208"   /* ^ must match ^ */

#define MAIN_DEFAULT_LANG     _TEXT("En_US")

const TCHAR * gs_szUsage = _TEXT("\t"
                                 "[--verbose]\n\t"
                                 "--config <ConfigFilename>\n\t"
                                 "[--ccsid <CCSID>]\n\t"
                                 "[--lang <langId>]\n\t"
                                 "[--ni <NumberOfIterations>]\n\t"
                                 "[--iteratortest]\n\t"
                                 "[--initrc <rc>]\n\t"
                                 "[--processdocrc <rc>]\n\t"
                                 "[--docisterm]\n\t"
                                 "[--inpterm <term>] | <InputFilename> <...>\n"
                                );

static const TCHAR *          gs_szHelp = _TEXT("\t"
    "Load the specified annotator(s) and feed them the specifed input text file(s).\n\t"
    "\n\t"
    "[--verbose]               verbose display mode\n\t"
    "-config <ConfigFilename>  The file name of the config file to use.\n\t"
    "[--ccsid <CCSID>]         The CCSID for the text files to be processed\n\t"
    "                             (default is: " MAIN_DEFAULT_CCSID_STR ")\n\t"
    "[--lang <langId>]         The Language ID for the text files to be processed\n\t"
    "                             (default is: En_US)\n\t"
    "[--ni <NumberOfIt>]       Number of iterations for process\n\t"
    "[--initrc <rc>]           uima::Engine.init() must fail with specified RC\n\t"
    "[--iteratortest]          Run an iterator/output test after processing the doc(s)\n\t"
    "[--processdocrc <rc>]     uima::Engine.processDocument() must fail with specified RC\n\t"
    "[--docisterm]             Annotate the whole document as a term\n\t"
    "[--inpterm <term>]        Use the specified term instead of text files\n\t"
    "<InputFilename> <...>     The file name(s) of the text files to be processed\n"
                                               );

static bool                   gs_bVerbose = false;
static bool                   gs_bDocIsTerm = false;
static bool                   gs_bDoIterTest = false;
static long                   gs_lExpectedInitRc = UIMA_ERR_NONE;
static long                   gs_lExpectedProcessDocumentRc = UIMA_ERR_NONE;

using namespace uima;

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


#define failIfNotTrue( expr ) /*lint -save -e611 -e923 -e731 -e909 -e506 */ \
   ( ( expr ) ? ( void )0 : failImpl( #expr, __FILE__, (unsigned int) __LINE__ ) ) /*lint -restore */

void failImpl( const char * cpszExpr, const char * cpszFile, unsigned int uiLine) {
  /* compose error message */
  cerr << "\n"
  "*** TEST CONDITION FAILED ***\n";
  /* compose error message */
  cerr << "***             TEST: " << cpszExpr << "\n"
  "***             LINE: " << uiLine << "\n"
  "***           MODULE: " << cpszFile << "\n"
  << endl;

  UIMA_EXC_THROW_NEW(ConsoleAbortExc,
                     UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION,
                     UIMA_MSG_ID_EXC_UNEXPECTED_ERROR,
                     ErrorMessage(UIMA_MSG_ID_EXCON_UNKNOWN_CONTEXT),
                     ErrorInfo::unrecoverable);
}

class TestLogger : public Logger {
public:
  string message;
  long errcode;
  TestLogger() : message(), errcode(0) {}
  void log(LogStream::EnEntryType entrytype,
    string classname, string methodname, string msg, long code)  {
      this->message = msg;
      errcode = code;
  }
  void reset() {
    message.clear();
    errcode =0;
  }
  
};

void testCallingSequence1(uima::util::ConsoleUI & rclConsole, const TCHAR * cpszConfigFilename)
/* ----------------------------------------------------------------------- */
{
  ErrorInfo errInfo;
  uima::TextAnalysisEngine * pEngine = TextAnalysisEngine::createTextAnalysisEngine(cpszConfigFilename, errInfo);

  failIfNotTrue( errInfo.getErrorId() == UIMA_ERR_NONE );
  failIfNotTrue( pEngine != NULL );
  CAS * cas = pEngine->newCAS();
  failIfNotTrue( cas != NULL );

  uima::UnicodeStringRef us(icu::UnicodeString("a") );
//   UnicodeStringRef uRef(us);
  rclConsole.formatHeader(_TEXT("testing Engine CallingSequence1"));

  cas->setDocumentText(us.getBuffer(), us.length());
  cas->getDocumentAnnotation().setLanguage("en");
  failIfNotTrue(pEngine->process(*cas) == UIMA_ERR_NONE);
  failIfNotTrue(cas->reset() == UIMA_ERR_NONE);
  failIfNotTrue(pEngine->destroy() == UIMA_ERR_NONE);

  cas->setDocumentText(us.getBuffer(), us.length());
  cas->getDocumentAnnotation().setLanguage("en");
  failIfNotTrue(pEngine->process(*cas) == UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE);

  TyErrorId deInitRC = pEngine->destroy();
  rclConsole.format("RC of deInit()", deInitRC);
  failIfNotTrue( deInitRC == UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE);
  rclConsole.formatBool(_TEXT("testing Engine CallingSequence1 OK"), true);  //lint !e944: argument for operator '!' always evaluates to False
  delete cas;
  delete pEngine;
}  //lint !e715: cpszConfigFilename (line 99) not referenced

void testCallingSequence2(uima::util::ConsoleUI & rclConsole, const TCHAR * cpszConfigFilename)
/* ----------------------------------------------------------------------- */
{
  rclConsole.formatHeader(_TEXT("testing Engine CallingSequence2"));
  {

    uima::TextAnalysisEngine * pEngine = NULL;
    ErrorInfo errInfo;

    pEngine = TextAnalysisEngine::createTextAnalysisEngine(cpszConfigFilename, errInfo);
    failIfNotTrue(errInfo.getErrorId() == UIMA_ERR_NONE);
    failIfNotTrue(pEngine != NULL );
    delete pEngine;
  }
  rclConsole.formatBool(_TEXT("testing Engine CallingSequence2 OK"), true);  //lint !e944: argument for operator '!' always evaluates to False
}

#include "unicode/utypes.h"

void testCallingSequence3(uima::util::ConsoleUI & rclConsole, const TCHAR * cpszConfigFilename)
/* ----------------------------------------------------------------------- */
{
  uima::TextAnalysisEngine *  pEngine = NULL;
  uima::Language              clLanguage(MAIN_DEFAULT_LANG);
  const char *               clCCSID = MAIN_DEFAULT_CCSID_STR;
  TyErrorId                  utErrorId;

  icu::UnicodeString us("a");
  UnicodeStringRef uref( us);

  rclConsole.formatHeader(_TEXT("testing Engine CallingSequence3"));

  ErrorInfo errInfo;
  pEngine = TextAnalysisEngine::createTextAnalysisEngine(cpszConfigFilename, errInfo);
  failIfNotTrue(errInfo.getErrorId() == UIMA_ERR_NONE);
  failIfNotTrue(pEngine != NULL );
  CAS * cas = pEngine->newCAS();
  failIfNotTrue( cas != NULL );

  /* test for NULL ptrs */

  UnicodeStringRef uref2(NULL);
  cas->setDocumentText(uref2.getBuffer(), uref2.length());
  cas->getDocumentAnnotation().setLanguage("en");
  failIfNotTrue(pEngine->process(*cas) == UIMA_ERR_NONE);
  failIfNotTrue(cas->reset() == UIMA_ERR_NONE);


  /* test for subsequent processes */
  cas->setDocumentText(uref2.getBuffer(), uref2.length());
  cas->getDocumentAnnotation().setLanguage("en");

  failIfNotTrue(pEngine->process(*cas) == UIMA_ERR_NONE);

  failIfNotTrue(pEngine->process(*cas) == UIMA_ERR_NONE);


  utErrorId = pEngine->destroy();
  failIfNotTrue(utErrorId == UIMA_ERR_NONE);
  delete cas;
  delete pEngine;
  rclConsole.formatBool(_TEXT("testing Engine CallingSequence3 OK"), true);  //lint !e944: argument for operator '!' always evaluates to False
}

void testMissingResMgr(uima::util::ConsoleUI & rclConsole)
/* ----------------------------------------------------------------------- */
{
  rclConsole.info("testMissingResMgr start.");
  uima::TextAnalysisEngine * pEngine;

  failIfNotTrue( ! ResourceManager::hasInstance() );
  ErrorInfo errInfo;
  pEngine = TextAnalysisEngine::createTextAnalysisEngine(_TEXT("DUMMY"), errInfo );
  failIfNotTrue( ! ResourceManager::hasInstance() );
  failIfNotTrue(errInfo.getErrorId() == UIMA_ERR_ENGINE_RESMGR_NOT_INITIALIZED);
  failIfNotTrue(pEngine == NULL);
  rclConsole.info("testMissingResMgr finished." );
}

void testRegisterLoggers(uima::util::ConsoleUI & rclConsole)
/* ----------------------------------------------------------------------- */
{
  rclConsole.info("testRegisterLoggers start.");

  failIfNotTrue( ResourceManager::hasInstance() );
  ErrorInfo errInfo;
  TestLogger * pLogger1 = new TestLogger();
  TestLogger * pLogger2 = new TestLogger();

  
  LogFacility & logFacility = ResourceManager::getInstance().getLogger();
  /* register a logger */
  ResourceManager::getInstance().registerLogger(pLogger1);
  /* write log messages */
  string message = "This is message 1.";
  logFacility.logMessage(message);
  failIfNotTrue(message.compare(pLogger1->message) == 0);
  message = "This is an error message.";
  logFacility.logError(message,100);
  failIfNotTrue(message.compare(pLogger1->message) == 0);
  failIfNotTrue(100 == pLogger1->errcode);
 
  /* register second logger */
  pLogger1->reset();
  pLogger2->reset();
  ResourceManager::getInstance().registerLogger(pLogger2);
  /* write log messages */
  message = "This is message 2.";
  logFacility.logMessage(message);
  failIfNotTrue(message.compare(pLogger1->message) == 0);
  failIfNotTrue(message.compare(pLogger2->message) == 0);
  message = "This is an error message 2.";
  logFacility.logError(message,200);
  failIfNotTrue(message.compare(pLogger1->message) == 0);
  failIfNotTrue(message.compare(pLogger2->message) == 0);
  failIfNotTrue(200 == pLogger1->errcode);
  failIfNotTrue(200 == pLogger2->errcode);

  /* unregister 2nd logger */
  pLogger1->reset();
  pLogger2->reset();
  ResourceManager::getInstance().unregisterLogger(pLogger2);
  /* write log messages */
  message = "This is message 3.";
  logFacility.logMessage(message);
  failIfNotTrue(message.compare(pLogger1->message) == 0);
  failIfNotTrue(pLogger2->message.length() == 0);
  message = "This is an error message 3.";
  logFacility.logError(message,100);
  failIfNotTrue(message.compare(pLogger1->message) == 0);
  failIfNotTrue(pLogger2->message.length() == 0);
  failIfNotTrue(100 == pLogger1->errcode);
  failIfNotTrue(0 == pLogger2->errcode);

  /* cleanup - unregister the first logger */
  ResourceManager::getInstance().unregisterLogger(pLogger1);
  delete pLogger1;
  delete pLogger2;
  rclConsole.info("testRegisterLoggers finished." );
}

void testProcessDocu(uima::util::ConsoleUI & rclConsole,
                     uima::TextAnalysisEngine & rclEngine,
                     const char * crclCCSID,
                     const uima::Language & crclLanguage)
/* ----------------------------------------------------------------------- */
{
  TyErrorId                  utErrorId;
  string                     clstrInputFileContent;
  size_t                     uiNumOfInputDocs = 0;

  uima::DocBuffer docBuffer;
  CAS * cas = rclEngine.newCAS();
  failIfNotTrue(cas != NULL );

  /* iterate through all doc specs on command line */
  for (rclConsole.setToFirst(); rclConsole.isValid(); rclConsole.setToNext()) {
    ////uima::util::Filename     clInputFilename(rclConsole.getAsCString());
    //replaced with a hard wired data file
    icu::UnicodeString filename("tdoc_001_enus_850.asc");
    icu::UnicodeString fn = ResourceManager::resolveFilename(filename, filename);
    uima::util::Filename clInputFilename(UnicodeStringRef(fn).asUTF8().c_str());

    size_t                  uiSize;

    if (!clInputFilename.isExistent()) {
      rclConsole.fatal(1, _TEXT("Input file not found"), clInputFilename.getAsCString());
    }
    if (crclCCSID == NULL) /**** (!crclCCSID.isValid()) ***/
    {
      rclConsole.fatal(1, _TEXT("Invalid CCSID specified - cannot load document"), crclCCSID /**crclCCSID.getName() **/);
    }
    rclConsole.format(_TEXT("Adding Document"), clInputFilename.getAsCString());
    uiSize = ftool_ReadFileToString(clInputFilename, clstrInputFileContent);

    docBuffer.addDocPart(clstrInputFileContent.data(),uiSize,crclCCSID);
    // For real file based documents we only add a term annotation for the
    // whole "document" if the appropriate switch is set
    if (gs_bDocIsTerm) {
      assert(false);
    }

    icu::UnicodeString  ustrInputFileContent(clstrInputFileContent.data(), uiSize, crclCCSID);
    /* since we already added a complete doc, we may not add anything else */
///      failIfNotTrue(rclEngine.addDocPart(ustrInputFileContent) == UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE);
///      failIfNotTrue(rclEngine.addDoc(ustrInputFileContent) == UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE);

    cas->setDocumentText(docBuffer.getDocBuffer(), docBuffer.getLength() );
    cas->getDocumentAnnotation().setLanguage(crclLanguage);

    utErrorId = rclEngine.process(*cas);
    uimaToolHandleErrorId(rclConsole, utErrorId, rclEngine.getAnnotatorContext().getLogger().getLastErrorAsCStr(), _TEXT("uima::Engine::processDocument"), gs_lExpectedProcessDocumentRc);

    if (utErrorId == UIMA_ERR_NONE && gs_bDoIterTest) {
      failIfNotTrue(false);
      //         iteratorTest(rclConsole, rclEngine);
    }

    utErrorId = cas->reset();
    uimaToolHandleErrorId(rclConsole, utErrorId, rclEngine.getAnnotatorContext().getLogger().getLastErrorAsCStr(), _TEXT("uima::Engine::resetDocument"));
    ++uiNumOfInputDocs;
  }
  if (uiNumOfInputDocs == 0) {
    rclConsole.warning(_TEXT("No input file(s) specified"));
  }
  delete cas;
}

void testProcessTerm(uima::util::ConsoleUI & rclConsole,
                     uima::TextAnalysisEngine & rclEngine,
                     ///const uima::CCSID & crclCCSID,
                     const char * crclCCSID,
                     const uima::Language & crclLanguage,
                     const TCHAR * cpszInpTerm)
/* ----------------------------------------------------------------------- */
{
  TyErrorId               utErrorId;

  failIfNotTrue(EXISTS(cpszInpTerm));
  rclConsole.format(_TEXT("Input term"), cpszInpTerm);

  DocBuffer docBuffer;
  docBuffer.addDocPart(cpszInpTerm, strlen(cpszInpTerm), crclCCSID);

  //? assert(false);
  CAS * cas = rclEngine.newCAS();
  failIfNotTrue(cas != NULL );

  // For terms we always add a term annotation for the whole "document"
  /* since we already added a complete doc, we may not add anything else */
  cas->setDocumentText(docBuffer.getDocBuffer(), docBuffer.getLength() );
  cas->getDocumentAnnotation().setLanguage(crclLanguage);

  utErrorId = rclEngine.process(*cas);
  uimaToolHandleErrorId(rclConsole, utErrorId, rclEngine.getAnnotatorContext().getLogger().getLastErrorAsCStr(), _TEXT("uima::Engine::processDocument"), gs_lExpectedProcessDocumentRc);

  if (utErrorId == UIMA_ERR_NONE && gs_bDoIterTest) {
    failIfNotTrue(false);
    //      iteratorTest(rclConsole, rclEngine);
  }

  utErrorId = cas->reset();
  uimaToolHandleErrorId(rclConsole, utErrorId, rclEngine.getAnnotatorContext().getLogger().getLastErrorAsCStr(), _TEXT("uima::Engine::resetDocument"));
  delete cas;
}




bool testLoadData(uima::util::ConsoleUI & rclConsole,
                  const char *  cpszCCSID,
                  const TCHAR * cpszConfigFilename,
                  const TCHAR * cpszLanguage,
                  size_t uiNumOfIterations)
/* ----------------------------------------------------------------------- */
{
  uima::TextAnalysisEngine * pEngine = NULL;
  ///uima::CCSID                 clCCSID(lCCSID);
  uima::Language              clLanguage(cpszLanguage);
  ErrorInfo errInfo;

  rclConsole.formatHeader(_TEXT("testing UIMA Engine loading a document"));
  /* init engine */
  pEngine = TextAnalysisEngine::createTextAnalysisEngine(cpszConfigFilename, errInfo);
  uimaToolHandleErrorId(rclConsole, errInfo.getErrorId(), errInfo.asString().c_str(), _TEXT("uima::Engine::init()"), gs_lExpectedInitRc);
  if (gs_lExpectedInitRc != UIMA_ERR_NONE) {
    return(false);
  }
  failIfNotTrue( pEngine != NULL );

  for (size_t ui = 0; ui < uiNumOfIterations; ui++) {
    const TCHAR *           cpszInpTerm = 0;

    /* read in file */
    if (uiNumOfIterations > 1) {
      rclConsole.newline();
      rclConsole.format(_TEXT("Iteration"), (unsigned long)ui);
    }
    if (rclConsole.hasArgString(_TEXT("inpterm"), cpszInpTerm)) {
      testProcessTerm(rclConsole, *pEngine, cpszCCSID, clLanguage, cpszInpTerm);
    } else {
      testProcessDocu(rclConsole, *pEngine, cpszCCSID, clLanguage);
    }
  }
  TyErrorId utErrorId = pEngine->destroy();
  uimaToolHandleErrorId(rclConsole, utErrorId, pEngine->getAnnotatorContext().getLogger().getLastErrorAsCStr(), _TEXT("uima::Engine::deInit"));
  delete pEngine;
  return(true);
}


void testDescriptorNotFound(uima::util::ConsoleUI & rclConsole)
/* ----------------------------------------------------------------------- */
{
  rclConsole.info("testDescriptorNotFound start.");
  uima::TextAnalysisEngine * pEngine;

  ErrorInfo errInfo;
  pEngine = TextAnalysisEngine::createTextAnalysisEngine(_TEXT("DUMMY"), errInfo );
  failIfNotTrue(errInfo.getErrorId() == UIMA_ERR_RESOURCE_CORRUPTED);
  failIfNotTrue(pEngine == NULL);
  rclConsole.info("testDescriptorNotFound finished." );
}


void testCasMultiplier(uima::util::ConsoleUI & rclConsole)
/* ----------------------------------------------------------------------- */
{
  rclConsole.info("testCasMultiplier start.");
  uima::TextAnalysisEngine * pEngine;

  ErrorInfo errInfo;

  icu::UnicodeString filename("SimpleTextSegmenter.xml");
  icu::UnicodeString fn = ResourceManager::resolveFilename(filename, filename);
  pEngine = TextAnalysisEngine::createTextAnalysisEngine(UnicodeStringRef(fn).asUTF8().c_str(), errInfo );
  failIfNotTrue(errInfo.getErrorId() == UIMA_ERR_NONE);
  failIfNotTrue(pEngine != NULL);


  //test operational properties settings
  failIfNotTrue(pEngine->getAnalysisEngineMetaData().getOperationalProperties()->getOutputsNewCASes() == true);
  failIfNotTrue(pEngine->getAnalysisEngineMetaData().getOperationalProperties()->getModifiesCas() == false);
  failIfNotTrue(pEngine->getAnalysisEngineMetaData().getOperationalProperties()->isMultipleDeploymentAllowed() == true);

 
  CAS * cas = pEngine->newCAS();
  cas->setDocumentText(icu::UnicodeString("This is the first sentence. This is the second sentence. This is the third sentence."));

  CASIterator iter = pEngine->processAndOutputNewCASes(*cas);
  int num=0;
  while (iter.hasNext()) {
    num++;
    CAS & seg = iter.next();
    failIfNotTrue(seg.getDocumentText().length() > 0);
    pEngine->getAnnotatorContext().releaseCAS(seg);
  }
  failIfNotTrue(num==3);
  delete pEngine;
  delete cas;
  rclConsole.info("testCasMultiplier finished." );
}





void mainTest(uima::util::ConsoleUI & rclConsole,
              const char  * cpszCCSID,
              const TCHAR * cpszConfigFilename,
              const TCHAR * cpszLanguage,
              size_t uiNumOfIterations)
/* ----------------------------------------------------------------------- */
{
  testDescriptorNotFound(rclConsole);
  if (testLoadData(rclConsole, cpszCCSID, cpszConfigFilename, cpszLanguage, uiNumOfIterations)) {
    testCallingSequence1(rclConsole, cpszConfigFilename);
    testCallingSequence2(rclConsole, cpszConfigFilename);
    testCallingSequence3(rclConsole, cpszConfigFilename);
  }
  testCasMultiplier(rclConsole);
}

int main(int argc, char * argv[]) /*
---------------------------------- */
{
  uima::util::ConsoleUI       clConsole(argc, argv, MAIN_TITLE, "");

  const char *        cpszCCSID = MAIN_DEFAULT_CCSID_STR;
  long                       lNumberOfIterations = 1;
  const TCHAR *              cpszLanguage = MAIN_DEFAULT_LANG;
  const TCHAR *              cpszConfigFilename = NULL;
  int                        iRetVal = 0;

  clConsole.handleUsageHelp(gs_szUsage, gs_szHelp);

  /* check cmd line args */
  gs_bVerbose = clConsole.hasArgSwitch(_TEXT("verbose"));
  gs_bDocIsTerm  = clConsole.hasArgSwitch(_TEXT("docisterm"));
  gs_bDoIterTest = clConsole.hasArgSwitch(_TEXT("iteratortest"));
  /*** run with toktest.xml only
  if (!clConsole.hasArgString(_TEXT("config"), cpszConfigFilename)) {
    cerr << "ERROR: required option \"config\" is missing.\n";
    clConsole.displayUsage();
  } ***/

  ///(void) clConsole.hasArgNumval(_TEXT("ccsid"), lCCSID);
  (void) clConsole.hasArgString(_TEXT("ccsid"), cpszCCSID);
  (void) clConsole.hasArgString(_TEXT("lang"), cpszLanguage);
  (void) clConsole.hasArgNumval(_TEXT("ni"), lNumberOfIterations);
  (void) clConsole.hasArgNumval(_TEXT("initrc"), gs_lExpectedInitRc);
  (void) clConsole.hasArgNumval(_TEXT("processdocrc"), gs_lExpectedProcessDocumentRc);

  /* before we init the res mgr, we test for the correct error */
  testMissingResMgr(clConsole);

  try {
    /* create a UIMA resource */
    (void) uima::ResourceManager::createInstance(MAIN_TITLE);
    /* test registering user specified loggers */
    testRegisterLoggers(clConsole);
    ///mainTest(clConsole, lCCSID, cpszConfigFilename, cpszLanguage, (size_t) lNumberOfIterations);
    icu::UnicodeString filename("toktest.xml");
    icu::UnicodeString fn = ResourceManager::resolveFilename(filename, filename);
    UnicodeStringRef fnRef(fn);
    string fnStr = fnRef.asUTF8();
    mainTest(clConsole, cpszCCSID, fnStr.c_str(),
             cpszLanguage, (size_t) lNumberOfIterations);
    fnRef.release(fnStr);             // release storage returned by asUTF8()
    clConsole.info(_TEXT("The program terminated successfully."));
    iRetVal = 0;
  } catch (ConsoleAbortExc & ) { // only thrown by uimaToolHandleErrorId to signal abortion
    clConsole.error(_TEXT("Aborting UIMA execution"));
    iRetVal = 1;
  } catch (Exception & rclException) {
    clConsole.getOutputStream() << rclException;
    clConsole.error(_TEXT("Unexpected UIMA exception"));
    iRetVal = 1;
  } catch (XMLException & rException) {
    clConsole.getOutputStream() << "XML Error: "
    << icu::UnicodeString( (UChar const *) rException.getMessage() ) << endl;
    iRetVal = 5041; // hack that our testcase basic_engine_bad_params works
  } catch (exception & rclException) {
    clConsole.error(rclException.what());
    iRetVal = 1;
  } catch (...) {
    clConsole.error(_TEXT("Unexpected exception"));
    iRetVal = 1;
  }
  return(iRetVal);
}

/* <EOF> */




