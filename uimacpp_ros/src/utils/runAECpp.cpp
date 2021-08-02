/*------------------------------------------------------------------------

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

--------------------------------------------------------------------------

  Test driver that reads text files or XCASs or XMIs and calls the annotator

-------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>


#include "xercesc/framework/LocalFileInputSource.hpp"
#include "xercesc/util/PlatformUtils.hpp"
#include "xercesc/util/XMLString.hpp"

#include <uima/pragmas.hpp>
#include <uima/filename.hpp>
#include <uima/dirwalk.hpp>

#include <uima/api.hpp>
#include <uima/xmiwriter.hpp>
#include <uima/xcasdeserializer.hpp>
#include <uima/xmideserializer.hpp>
#include <apr_portable.h>
#include <apr_thread_proc.h>
using namespace uima;
using namespace std;

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

/* Little helper routine to check and report errors.
   This routine just does a hard program exit for any failure! */
static void tafCheckError(TyErrorId utErrorId,
                          const AnalysisEngine &  crEngine);
static void tafCheckError(ErrorInfo const &);

void processInputFiles(AnalysisEngine * pEngine);

// input file or directory
 std::string in;
// output directory
 std::string out;
//AE descriptor filename
 const char* cnfg = NULL;
// sofa to use for creating a tcas
bool useSofa, lenient;
const char* sofaName;
//logging level
int loglevel;
//num iterations over input data.
int numruns;
//num annotator instances
int numinstances;
//randomize input and delay between call to process.
bool randomize;
long rdelay;

// input data types
enum dataFormats { textFormat, xcasFormat, xmiFormat };
dataFormats xcasInput;

void process (AnalysisEngine * pEngine, CAS * cas, std::string in, std::string out);
void writeXmi (CAS & outCas, int num,  std::string in, std::string outfn);

void tell() {
  cerr << "Usage: runAECpp UimaCppDescriptor <-x> InputFileOrDir <OutputDir>" << endl
  << "                <-s Sofa>  <-l LogLevel>" << endl;
  cerr << "  UimaCppDescriptor   Analysis Engine descriptor for a CPP annotator" << endl;
  cerr << "  InputFileOrDir      Input file or directory of files to process" << endl;
  cerr << "  OutputDir           Existing directory for Xmi outputs (optional)" << endl;
  cerr << "       Options:" << endl;
  cerr << "   -x [-xmi]     Input(s) must be in XCAS [XMI] format (default is raw text)" << endl;
  cerr << "   -lenient      For -xmi, ignore unknown types & features" << endl;
  cerr << "   -s Sofa       Name of a Sofa to process (input must be an XCAS or XMI)" << endl;
  cerr << "   -l logLevel   Set to 0, 1, or 2 for Message, Warning, or Error" << endl;
  cerr << "   -n numInstances number of annotator instances each running in a separate thread" << endl;
  cerr << "   -r numRuns    number of iterations over the same input." << endl;
  cerr << "   -rand         randomize the selection on next input to process." << endl;
  cerr << "   -rdelay Max   add random delay between 0 and Max milliseconds between calls to process." << endl;

}



//=================================================================
//
//Annotator instance threads that process input.
//-----------------------------------------------------------------
 static void* APR_THREAD_FUNC process(apr_thread_t *thd, void *data) {
   cout << endl << "ThreadId: " << apr_os_thread_current() << " runAECpp::processing... starting thread." << endl;
   processInputFiles((AnalysisEngine *) data);
   apr_thread_exit(thd, APR_SUCCESS);
   return NULL;
 }


int main(int argc, char * argv[]) {

 
  try {

    /* Access the command line arguments to get the name of the input text. */
    //if (argc != 3 && argc != 5 && argc != 7 && argc != 9 && argc != 11) {
    if (argc < 3) {
      tell();
      return 1;
    }
    useSofa = false;
    lenient = false;
    xcasInput = textFormat;
    //std::string sofa;
    //std::string pattern("*");
    cnfg = NULL;
    loglevel = -1;
    numinstances = 1;
    numruns = 1;
    randomize = false;
    rdelay = 0;

    int index = 0;
    while (++index < argc) {
      char* arg = argv[index];
      if (0 == strcmp(arg, "-x")) {
        xcasInput = xcasFormat;
      } else if (0 == strcmp(arg, "-xcas")) {
        xcasInput = xcasFormat;
      } else if (0 == strcmp(arg, "-xmi")) {
        xcasInput = xmiFormat;
      } else if (0 == strcmp(arg, "-lenient")) {
        lenient = true;
      } else if (0 == strcmp(arg, "-s")) {
        if ( ++index < argc ) {
          sofaName = argv[index];
          useSofa = true;
        }
      } else if (0 == strcmp(arg, "-l")) {
        if ( ++index < argc ) {
          loglevel = atoi(argv[index]);
          if (loglevel < LogStream::EnMessage) {
            cerr << "LogLevel less than minimum value (Message) = " << LogStream::EnMessage << endl;
            return 1;
          }
          if (loglevel > LogStream::EnError) {
            cerr << "LogLevel greater than maximum value (Error) = " << LogStream::EnError << endl;
            return 1;
          }
        }
      } else if (0 == strcmp(arg, "-n")) {
        if ( ++index < argc ) {
          numinstances = atoi(argv[index]);
          if (numinstances < 1) {
            cerr << "NumInstances less than minimum value 1 "<< endl;
            return 1;
          }
          if (out.length() > 0) {
            cerr << "Output directory may not be specified when NumInstances is more than 1." << endl;
            return 1;         
          }
        }
      } else if (0 == strcmp(arg, "-rand")) {
        randomize = true;
      } else if (0 == strcmp(arg, "-r")) {
        if ( ++index < argc ) {
          numruns = atoi(argv[index]);
          if (numruns < 1) {
            cerr << "Number of runs less than minimum value 1 "<< endl;
            return 1;
          }
        }
      } else if (0 == strcmp(arg, "-rdelay")) {
        if ( ++index < argc ) {
          rdelay = atol(argv[index]);
          rdelay = rdelay; //convert to microsec
          if (rdelay < 1) {
            cerr << "Random delay in millis less than minimum value of 1 "<< endl;
            return 1;
          }
        }
      } else { //one of the standard params - whichever we haven't read yet
        if (cnfg == NULL) {
          cnfg = arg;
        } else if (in.length() == 0) {
          in.append(arg);
        } else if (out.length() == 0) {
          out.append(arg);
          if (numinstances > 1) {
            cerr << "Output directory may not be specified when NumInstances is more than 1." << endl;
            return 1;
          }
        }
      }
    } //while

    if (in.length() == 0 || index > argc) {   // Too few args or no arg after -s or -l
      tell();
      return 1;
    }

    if (out == in) {
      cout << "runAECpp: ERROR: input and output file paths are the same " << endl;
      return -1;
    }

    /* Create/link up to a UIMACPP resource manager instance (singleton) */
    (void) ResourceManager::createInstance("runAECpp");

    /* check if input file / directory exists */
    uima::util::Filename infn(in.c_str());
    if (!infn.isExistent() ) {
      cout << "runAECpp: ERROR: input file / directory does not exist. " << endl;
      return -1;
    }

    if (loglevel >= 0) {
      ResourceManager::getInstance().setLoggingLevel((LogStream::EnEntryType)loglevel);
    }
    ErrorInfo errorInfo;
    if (numinstances ==  1) { 
      AnalysisEngine * pEngine = Framework::createAnalysisEngine(cnfg, errorInfo);
      if (errorInfo.getErrorId() != UIMA_ERR_NONE) {
        cerr << "runAECpp:" << endl
        << "  Error string  : "
        << AnalysisEngine::getErrorIdAsCString(errorInfo.getErrorId()) << endl
        << "  UIMACPP Error info:" << endl
        << errorInfo << endl;
        exit((int)errorInfo.getErrorId());
      }
      processInputFiles(pEngine);
    } else {
      apr_status_t rv = 0;

      //APR pool
      apr_pool_t *pool;
      rv = apr_pool_create(&pool, NULL);
      if (rv != APR_SUCCESS) {
        cerr << "ERROR: apr_pool_create() failed. " << endl;
        return -1;
      }

   
      /*create as many AnalysisEngine instances as specified 
       by numinstances. */
      vector<AnalysisEngine *> analysisEngines;  

      for (int i=0; i < numinstances; i++) {
        AnalysisEngine * pEngine = Framework::createAnalysisEngine(cnfg, errorInfo);
        if (errorInfo.getErrorId() != UIMA_ERR_NONE) {
          cerr << "runAECpp:" << endl
          << "  Error string  : "
          << AnalysisEngine::getErrorIdAsCString(errorInfo.getErrorId()) << endl
          << "  UIMACPP Error info:" << endl
          << errorInfo << endl;
          exit((int)errorInfo.getErrorId());
        }
        analysisEngines.push_back(pEngine);
      }
      cerr << "Initialized AnalysisEngine " << endl;


      /* create and start the processing threads */
      apr_threadattr_t * thd_attr=0;
      rv = apr_threadattr_create(&thd_attr, pool);
      assert(rv == APR_SUCCESS);

      vector<apr_thread_t *> processingThreads;
      for (int i=0; i < numinstances; i++) {
        apr_thread_t *thread=0;
        rv = apr_thread_create(&thread, thd_attr, process, analysisEngines.at(i), pool);
        assert(rv == APR_SUCCESS);
        processingThreads.push_back(thread);
        apr_sleep(10000); //required so that time function to distinctly seed randomizer.
      }

      cerr << "Wait for processing threads to finish " << endl;
      /* wait for threads to end */
      for (size_t i=0; i < processingThreads.size(); i++) {
        //cout << "runAECpp: wait for thread " << i << " to end " << endl;
        apr_thread_join(&rv, processingThreads.at(i));
      }

      if (pool) {
        apr_pool_destroy(pool);
        pool=0;
      }
    }
  } catch (Exception e) {
    cout << "runAECpp: " << e << endl;
  }
  /* If we got this far everything went OK */
  cout << "runAECpp: processing finished sucessfully! " << endl;

  return(0);
}



/* Little helper routine to check and report errors.
   This routine just does a hard program exit for any failure!
*/
static void tafCheckError(TyErrorId utErrorId,
                          const AnalysisEngine &  crEngine) {
  if (utErrorId != UIMA_ERR_NONE) {
    cerr << "runAECpp:" << endl;
    cerr << "  Error number        : "
    << utErrorId << endl;
    cerr << "  Error string        : "
    << AnalysisEngine::getErrorIdAsCString(utErrorId) << endl;
    const TCHAR * errStr = crEngine.getAnnotatorContext().getLogger().getLastErrorAsCStr();
    if (errStr != NULL)
      cerr << "  Last logged message : " << errStr << endl;
    exit((int)utErrorId);
  }
}

/* Similar routine as above just with error info objects instead of err-ids.
   This routine just does a hard program exit for any failure!
*/
static void tafCheckError(ErrorInfo const & errInfo) {
  if (errInfo.getErrorId() != UIMA_ERR_NONE) {
    cerr << "runAECpp:" << endl
    << "  Error string  : "
    << AnalysisEngine::getErrorIdAsCString(errInfo.getErrorId()) << endl
    << "  UIMACPP Error info:" << endl
    << errInfo << endl;
    exit((int)errInfo.getErrorId());
  }
}

void process (AnalysisEngine * pEngine, CAS * cas, std::string in, std::string outfn) {
  cout << endl << "ThreadId: " << apr_os_thread_current() << " runAECpp::processing " << in << " " << out << endl;
  try {
    if (xcasInput != textFormat) {
      /* initialize from an xcas or xmicas */
      //cout << "runAECpp::processing xml file " << in << endl;
	  XMLCh* native = XMLString::transcode(in.c_str());
	  LocalFileInputSource fileIS(native);
	  XMLString::release(&native);
	  if (xcasInput == xcasFormat) {
	    XCASDeserializer::deserialize(fileIS, *cas);
	  }
	  else {
		XmiDeserializer::deserialize(fileIS, *cas, lenient);
	  }
    } else {
      /* read as text file and set document text of default view */
      FILE * pFile = fopen(in.c_str(),"rb");
      int filesize;
      if (pFile == NULL) {
        cerr << "RunAECpp: Error reading file " << in << endl;
        exit(-1);
      }

      /* allocate buffer for file contents */
      struct stat fstat;
      stat(in.c_str(), &fstat);
      filesize = fstat.st_size;
      char * pBuffer = new char[filesize+1];
      if (pBuffer == NULL) {
        cerr << "RunAECpp: Error allocating buffer to hold contents of file  " << in << endl;
        exit(-1);
      }

      /* read the file */
      size_t numread = fread(pBuffer,1,filesize,pFile);
      fclose(pFile);
      /* convert to unicode and set tcas document text*/
      icu::UnicodeString ustrInputText(pBuffer, (int32_t)numread, "utf-8");
      cas->setDocumentText(UnicodeStringRef(ustrInputText));
      delete[] pBuffer;
    }

    // Is the input a tcas?
    if (!useSofa && cas->isBackwardCompatibleCas()) {
      useSofa = true;
      sofaName = CAS::NAME_DEFAULT_TEXT_SOFA;
    }

    // Is a specific Sofa view specified?
    if (useSofa) {
      /* process the specified TCAS */
      SofaFS mySofa = cas->getSofa(pEngine->getAnnotatorContext().mapToSofaID(sofaName));
      if (!mySofa.isValid()) {
        cerr << "runAECpp:" << endl
        << "  Specified Sofa named " << sofaName
        << " not found in the input file" << endl;
        exit(99);
      }

      CASIterator casIter = pEngine->processAndOutputNewCASes(*cas->getView(mySofa));
      int i=0;
      while (casIter.hasNext()) {
        i++;
        CAS  & outCas = casIter.next();

        //write out xmi
        if (outfn.length() > 0) {
          writeXmi(outCas, i, in, outfn);
        }

        //release the CAS
        pEngine->getAnnotatorContext().releaseCAS(outCas);

        cout << "runAECpp::processing new Cas " << i << endl;
      }

    } else {
      /* process the CAS */
      
      CASIterator casIter = ((AnalysisEngine*)pEngine)->processAndOutputNewCASes(*cas);
      int i=0;
      while (casIter.hasNext()) {
        i++;
        CAS & outCas = casIter.next();
        //write out xmi
        if (outfn.length() > 0) {
          writeXmi(outCas, i, in, outfn);
        }

        //release CAS
        pEngine->getAnnotatorContext().releaseCAS(outCas);

        cout << "runAECpp::processing new Cas " << i << endl;
      }

    }

    if (outfn.length() > 0) {
      util::Filename infile((TCHAR*) in.c_str());

      outfn.append("/");
      outfn.append(infile.getName());

      //open a file stream for output xmi
      ofstream file;
      file.open (outfn.c_str(), ios::out | ios::binary);
      if ( !file ) {
        cerr << "runAECpp: Error opening output xmi: " << outfn.c_str() << endl;
        exit(99);
      }

      //serialize the input cas
      cout << "runAECpp::processing write out xmi " << outfn << endl;
      XmiWriter writer(*cas, true);
      writer.write(file);
      file.close();
    }

  } catch (Exception e) {
    ErrorInfo errInfo = e.getErrorInfo();
    cerr << "runAECPP::Error " << errInfo.getErrorId() << " " << errInfo.getMessage() << endl;
    cerr << errInfo << endl;
  }
}

void writeXmi (CAS & outCas, int num,  std::string in, std::string outfn)  {

  util::Filename infile((TCHAR*) in.c_str());
  std::string ofn;
  ofn.append(outfn.c_str());
  ofn.append("/");
  ofn.append(infile.getName());
  ofn.append("_seg_");
  stringstream s;
  s << num;
  ofn.append(s.str());

  //open a file stream for output xmi
  ofstream file;
  file.open (ofn.c_str(), ios::out | ios::binary);
  if ( !file ) {
    cerr << "Error opening output xmi: " << ofn.c_str() << endl;
    exit(99);
  }

  //serialize the cas
  cout << "runAECpp::processing write out xmi " << ofn << endl;
  XmiWriter writer(outCas, true);
  writer.write(file);
  file.close();
}

void processInputFiles(AnalysisEngine * pEngine) {
    TyErrorId utErrorId;          // Variable to store UIMACPP return codes
    ErrorInfo errorInfo;          // Variable to stored detailed error info

    int count = 0;

    stringstream str;
    str << endl << "ThreadId: " << apr_os_thread_current();
    str << " runAECpp: Processing started. Number of runs " << numruns 
                << " rdelay " << rdelay << " millis. " ;
    if (randomize)
        str << " Inputs processed in random order. ";
    
    cout << str.str() << endl;
    //uima::ResourceManager::getInstance().getLogger().logMessage(str.str() + " started: " );

    /* Get a new CAS */
    CAS* cas = pEngine->newCAS();
    if (cas == NULL) {
      cerr << "runAECpp: pEngine->newCAS() failed." << endl;
      exit (1);
    }

    /* initialize random seed: */
    srand( time(NULL) + (apr_time_now() % 10000) );

    for (int i=0; i < numruns; i++) {
      stringstream str;
      cout << endl << "ThreadId: " << apr_os_thread_current() << " runAECpp::processing start iteration: " << i << endl;
      //uima::ResourceManager::getInstance().getLogger().logMessage(str.str() );
      /* process input */
      util::DirectoryWalk dirwalker(in.c_str());
      if (dirwalker.isValid()) {
        cout << "ThreadId: " << apr_os_thread_current() << " runAECpp::processing all files in directory: " << in.c_str() << endl;
        util::Filename infile(in.c_str(),"FilenamePlaceHolder");
        if (!randomize) { 
          while (dirwalker.isValid()) {
          // Process all files or just the ones with matching suffix
            if ( dirwalker.isFile() ) {
              infile.setNewName(dirwalker.getNameWithoutPath());
              std::string afile(infile.getAsCString());
            
              stringstream str;
              if (count % 100 == 0 && count > 0)  {
                str << apr_time_now() << " ThreadId: " << apr_os_thread_current() <<  " numProcessed=" << count;
                cerr << str.str() << endl;  
                uima::ResourceManager::getInstance().getLogger().logMessage(str.str() );
              }
              //process the cas
              process(pEngine,cas,afile, out);

              //reset the cas
              cas->reset();
              if (rdelay > 0) {
                int howlong = rand() % rdelay;
                cout << "ThreadId: " << apr_os_thread_current() << " runAECpp::processing sleep for " << howlong << " millis " << endl;
                apr_sleep(howlong*1000);
              }
              count++;
            }
            //get the next input file in the directory
            dirwalker.setToNext();
          }
        } else {
          //construct a list of the input files.
          vector<std::string> filenames;
          while (dirwalker.isValid()) {
             // Process all files or just the ones with matching suffix
             if ( dirwalker.isFile() ) {
              infile.setNewName(dirwalker.getNameWithoutPath());
              filenames.push_back(infile.getAsCString());
             }
             //get the next input file in the directory
             dirwalker.setToNext();
          } 
         
          //how many to process in this run.
          int num = filenames.size();   
          for (int i=0; i < num; i++) {
            //select next file to be processed.   
            int index =   rand() % filenames.size();  //number between 1 and number of files
            
            stringstream str;
            if (count % 100 == 0 && count > 0)  {
              str << apr_time_now() << " ThreadId: " << apr_os_thread_current() <<  " runAECpp::processing numProcessed=" << count;
              cerr << str.str() << endl;  
              uima::ResourceManager::getInstance().getLogger().logMessage(str.str() );
            }

            string afile = filenames.at(index);
            //cout << "ThreadId: " << apr_os_thread_current() << "runAECpp::processing file " << index << " " << afile  << endl;
            //process 
            process(pEngine, cas, afile, out);
            cas->reset();

            //sleep for time specified by rdelay
            if (rdelay > 0) {
              int howlong = rand() % rdelay;
              cout << "ThreadId: " << apr_os_thread_current() << " runAECpp::processing sleep for " << howlong << " millis " << endl;
              apr_sleep(howlong*1000);
            }
            count++;
          }
        }
      } else {
        //process the cas
        process(pEngine,cas, in, out);
      }
      /* call collectionProcessComplete */
      utErrorId = pEngine->collectionProcessComplete();
    }
    /* Free annotator */
    utErrorId = pEngine->destroy();

    delete cas;
    delete pEngine;
    cout << "ThreadId: " << apr_os_thread_current()  << " runAECpp finished processing." << endl;
}
/* <EOF> */



