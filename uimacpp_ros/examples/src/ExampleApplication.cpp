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
#include "uima/dirwalk.hpp"
#include "uima/filename.hpp"
#include "uima/xmlwriter.hpp"

#include <sys/stat.h>
using namespace std;
using namespace uima;
/**
 * An example application that reads documents from files, sends them
 * though a Text Analysis Engine, and prints all discovered annotations to
 * the console.
 * <p>
 * The application takes two arguments:
 * <ol type="1">
 * <li>The path to an XML descriptor for the TAE to be executed</li>
 * <li>An input directory containing files to be processed</li>
 * <li>Optionally, the logging level</li>
 * </ol>
 *
 *
 */

/**
* Main program.
*
* @param args Command-line arguments - see class description
*/

/* Little helper routine to check and report errors.
   This routine just does a hard program exit for any failure! */
static void CheckError(TyErrorId utErrorId,
                       const AnalysisEngine &  crEngine);
static void CheckError(ErrorInfo const &);

void processFile (std::string filename, AnalysisEngine * pEngine, CAS * tcas);

void tell() {
  cerr << "Usage: ExampleApplication UimaCppDescriptor InputFileOrDirectory <-l LogLevel>" << endl;
}

int main(int argc, char * argv[]) {
  try {
    int loglevel = -1;

    /* check  the number of command line args */
    if (argc != 3 && argc != 5 ) {
      tell();
      return 1;
    }

    if (argc == 5) {
      if (!strcmp(argv[3], "-l")) {
        loglevel = atoi(argv[4]);
        if (loglevel < LogStream::EnMessage) {
          cerr << "LogLevel less than minimum value (Message) = " << LogStream::EnMessage << endl;
          return 1;
        }
        if (loglevel > LogStream::EnError) {
          cerr << "LogLevel greater than maximum value (Error) = " << LogStream::EnError << endl;
          return 1;
        }
      } else {
        cerr << "Inexpected option: " << argv[3] << endl;
        tell();
        return 1;
      }
    }

    /* Create/link up to a resource manager instance (singleton) */
    (void) ResourceManager::createInstance("UIMACPP_EXAMPLE_APPLICATION");

    if (loglevel >= 0) {
      ResourceManager::getInstance().setLoggingLevel((LogStream::EnEntryType)loglevel);
    }

    TyErrorId utErrorId;          // Variable to store return codes
    ErrorInfo errorInfo;          // Variable to stored detailed error info
    /* Initialize engine with filename of config-file */
    AnalysisEngine * pEngine =
      Framework::createAnalysisEngine(argv[1], errorInfo);
    CheckError(errorInfo);

    /* Get a new CAS */
    CAS* tcas = pEngine->newCAS();

    /* process input xcas */
    util::DirectoryWalk dirwalker(argv[2]);
    if (dirwalker.isValid()) {

      util::Filename infile(argv[2],"FilenamePlaceHolder");
      while (dirwalker.isValid()) {
        // Process all files or just the ones with matching suffix
        if ( dirwalker.isFile() &&
             dirwalker.matchesWildcardPattern("*.txt") ) {
          infile.setNewName(dirwalker.getNameWithoutPath());
          std::string afile(infile.getAsCString());

          //process the file
          processFile(afile, pEngine, tcas);

          //reset the cas
          tcas->reset();
        }
        //get the next xcas file in the directory
        dirwalker.setToNext();
      }
    } else {
      /* If has no directory entries then probably a file */
      cout << "ExampleApplication: processing file " << argv[2] << endl;
      std::string afile(argv[2]);
      //process the cas
      processFile(afile, pEngine, tcas);
    }

    /* call collectionProcessComplete */
    utErrorId = pEngine->collectionProcessComplete();

    /* Free ressorces */
    utErrorId = pEngine->destroy();
    CheckError(utErrorId, *pEngine);

    delete tcas;
    delete pEngine;
  } catch (Exception e) {
    cout << "ExampleApplication " << e << endl;
  }
  /* If we got this far everything went OK */
  cout << "ExampleApplication: processing finished sucessfully! " << endl;

  return(0);
}

/* Little helper routine to check and report errors.
   This routine just does a hard program exit for any failure!
*/
static void CheckError(TyErrorId utErrorId,
                       const AnalysisEngine &  crEngine) {
  if (utErrorId != UIMA_ERR_NONE) {
    cerr << endl << "   *** ExampleApplication - Error info:" << endl;
    cerr << "Error number        : "
    << utErrorId << endl;
    cerr << "Error string        : "
    << AnalysisEngine::getErrorIdAsCString(utErrorId) << endl;
    const TCHAR* errStr = crEngine.getAnnotatorContext().getLogger().getLastErrorAsCStr();
    if (errStr != NULL)
      cerr << "  Last logged message : "  << errStr << endl;
    exit((int)utErrorId);
  }
}

/* Similar routine as above just with error info objects instead of err-ids.
This routine just does a hard program exit for any failure!
*/
static void CheckError(ErrorInfo const & errInfo) {
  if (errInfo.getErrorId() != UIMA_ERR_NONE) {
    cerr << endl << "   *** ExampleApplication - Error info:" << endl
    << "Error string  : "
    << AnalysisEngine::getErrorIdAsCString(errInfo.getErrorId())
    << errInfo << endl;                      /* (errInfo starts with a newline) */
    exit((int)errInfo.getErrorId());
  }
}

void processFile (std::string filename, AnalysisEngine * pEngine, CAS * tcas ) {
  cout << "processing file " << filename << endl;
  try {

    /* read in file contents and set CAS Document text */
    FILE * pFile = fopen(filename.c_str(),"rb");
    int filesize;
    if (pFile == NULL) {
      cerr << "ExampleApplication: Error reading file " << filename << endl;
      exit(-1);
    }

    /* allocate buffer for file contents */
    struct stat fstat;
    stat(filename.c_str(), &fstat);
    filesize = fstat.st_size;
    char * pBuffer = new char[filesize+1];
    if (pBuffer == NULL) {
      cerr << "ExampleApplication: Error allocating buffer to hold contents of file  " << filename << endl;
      exit(-1);
    }

    /* read the file */
    size_t numread = fread(pBuffer,1,filesize,pFile);
    fclose(pFile);

    /* convert to unicode and set tcas document text*/
    UnicodeString ustrInputText(pBuffer, filesize, "utf-8");

    tcas->setDocumentText(ustrInputText.getBuffer(), ustrInputText.length(), true);

    delete[] pBuffer;
    /* process the CAS */
    TyErrorId utErrorId = ((AnalysisEngine*)pEngine)->process(*tcas);
    CheckError(utErrorId, *pEngine);

    /* serialize the cas to stdout */
    cout << "ExampleApplication: write out cas after processing file " << filename << endl << endl;
    XCASWriter writer(*tcas, true);
    writer.write(cout);
    cout << endl;

  } catch (Exception e) {
    cout << "ExampleApplication: " << e << endl;
  }
}

/* <EOF> */



