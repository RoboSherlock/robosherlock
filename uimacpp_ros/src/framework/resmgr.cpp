/** @name resmgr.cpp
-----------------------------------------------------------------------------

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

-----------------------------------------------------------------------------

   Description: This file contains class {\tt ResourceManager}.
                Note: code parts which should be made thread-safe
                      are marked with "mutex" (suhre)

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */
//#define DEBUG_VERBOSE
#include <uima/resmgr.hpp>
#include "apr.h"

/* ----------------------------------------------------------------------- */
/*       Implementation dependencies                                       */
/* ----------------------------------------------------------------------- */
#if (__GNUC__ < 3)
#include <clocale>
#else
#include <locale>
#endif

#if defined(_MSC_VER)
int rc = _CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
// Send all reports to STDOUT
int rc1 =   _CrtSetReportMode( _CRT_WARN, _CRTDBG_MODE_FILE );
_HFILE h1 =  _CrtSetReportFile( _CRT_WARN, _CRTDBG_FILE_STDOUT );
int rc3 =   _CrtSetReportMode( _CRT_ERROR, _CRTDBG_MODE_FILE );
_HFILE h2 = _CrtSetReportFile( _CRT_ERROR, _CRTDBG_FILE_STDOUT );
int rc5 =   _CrtSetReportMode( _CRT_ASSERT, _CRTDBG_MODE_FILE );
_HFILE h3 = _CrtSetReportFile( _CRT_ASSERT, _CRTDBG_FILE_STDOUT );
#endif

#include <jni.h>
#include <uima/envvar.hpp>
#include <uima/trace.hpp>
#include <uima/dirwalk.hpp>
#include <uima/dllfile.hpp>

#include <uima/macros.h>
#include <uima/comp_ids.h>
#include <uima/res_abase.hpp>
#include <uima/res_annotator.hpp>
#include <uima/envvars.h>
#include <uima/exceptions.hpp>
#include <uima/language.hpp>
#include <uima/msg.h>
#include <uima/msgstrtab.h>
#include <uima/casexception.hpp>

#include "xercesc/util/PlatformUtils.hpp"
#include "unicode/uclean.h"
#include <uima/stltools.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

#define JAVA_PROXY "org/apache/uima/uimacpp/UimacppAnalysisComponent"

#ifndef UIMA_VERSION
#error UIMA_VERSION must be defined in the compilation environment
#endif

/* max. number of characters in a valid filename for shipment */
const size_t UIMA_MAX_VALID_FILENAME_SIZE = 8;   /* DOS 8+3 */

XERCES_CPP_NAMESPACE_USE

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */
using namespace std;
namespace uima {

  /*
   * The class ResourceManagerAutomaticInstanceDestructor is used to call method
   * ResourceManager::deleteInstance().
   * There needs to be one instance of ResourceManagerAutomaticInstanceDestructor
   * for which the destructor will be called whenever the DLL goes out of scope.
   * This destructor will then call ResourceManager::deleteInstance()
   * automatically shutting down the resource manager.
   */
  class ResourceManagerAutomaticInstanceDestructor {
  public:
    ResourceManagerAutomaticInstanceDestructor(void) {
      ; /* do nothing */
      UIMA_TPRINT(_TEXT("INIT ..."));
    }
    ~ResourceManagerAutomaticInstanceDestructor(void) {
      UIMA_TPRINT(_TEXT("deleting ResourceManagerInstance ..."));
      ResourceManager::deleteInstance();
    }
  }
  ; /* ResourceManagerAutomaticInstanceDestructor */

  /* ----------------------------------------------------------------------- */
  /*       Globals                                                           */
  /* ----------------------------------------------------------------------- */
  

  ResourceManager * ResourceManager::cv_pclSingletonInstance = 0;
  TyProcedure               iv_traceProc;

  /* see docu for class ResourceManagerAutomaticInstanceDestructor above */
  static ResourceManagerAutomaticInstanceDestructor gs_clResourceManagerAutomaticInstanceDestructor;

  /* ----------------------------------------------------------------------- */
  /*       Function declarations                                             */
  /* ----------------------------------------------------------------------- */

  /* ----------------------------------------------------------------------- */
  /*       Macro definitions                                                 */
  /* ----------------------------------------------------------------------- */

  /* ----------------------------------------------------------------------- */
  /*       Private implementation                                            */
  /* ----------------------------------------------------------------------- */

  ResourceManager::ResourceManager(const TCHAR * cpszInstance, const TCHAR * cpszProductPrefix) :
      iv_utLastErrorId(UIMA_ERR_NONE),
      iv_locationWork("."),
      iv_locationData("."),
      iv_frameworkLogger(NULL),
      iv_logLevel(LogStream::EnMessage),
      iv_fileLogger(NULL),
      iv_loggers()
      /* ----------------------------------------------------------------------- */
  {
    
    string                    str;

    assert(EXISTS(cpszInstance));
    UIMA_TPRINT(_TEXT("instance: ") << cpszInstance);

    /* set the product prefix -- dropped this code */

    /* after the trace instance has been created and enabled,
       we may instantiate a first trace object */
    util::Trace                 clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_RESOURCE_MGR);
    clTrace.dump(_TEXT("UIMACPP Instance"), cpszInstance);

    /* determine xsd path ... use $UIMACPP_HOME/data instead of iv_locationData
       (which defaults to $UIMACPP_DATAPATH) as the latter is for user data. */
    bDoSchemaValidation=true;
    str = UIMA_ENVVAR_HOME;
    util::EnvironmentVariableQueryOnly clEnvVarSchemaPath(str.c_str());
    UIMA_TPRINT(_TEXT("querying envvar: ") << str.c_str());
    schemaInfo[0] = '\0';                   // Play safe
    if (clEnvVarSchemaPath.hasValue()) {
      string schemaloc = clEnvVarSchemaPath.getValue();
      UIMA_TPRINT(_TEXT("value: ") << schemaloc);
      schemaloc += "/data";

      // Create full name & transform path to absolute s.t. the XML parser can handle it
      util::Filename schemaFilename(schemaloc.c_str(),UIMA_XSD_FILENAME);
      schemaFilename.normalizeAbsolute();
      bIsSchemaAvailable = schemaFilename.isExistent();

      // schema filename must not contain blanks as the parser schemaLocation
      // attribute is a blank-separated pair of strings
      if (bIsSchemaAvailable) {
        icu::UnicodeString schemaPath(schemaFilename.getAsCString());
        schemaPath.findAndReplace(" ", "%20");
        int len = strlen(UIMA_XML_NAMESPACE);
        memcpy(schemaInfo, UIMA_XML_NAMESPACE, len);
        schemaInfo[len] = ' ';
        schemaPath.extract(0, schemaPath.length(), schemaInfo+len+1, "UTF-8");
      }
    } else {
      bIsSchemaAvailable = false;
    }

    /* determine data path */
    str = UIMA_ENVVAR_SUFFIX_DATAPATH;
    util::EnvironmentVariableQueryOnly clEnvVarDataPath(str.c_str());
    UIMA_TPRINT(_TEXT("querying envvar: ") << str.c_str());
    if (clEnvVarDataPath.hasValue()) {
      iv_locationData = util::Location(clEnvVarDataPath.getValue());
    } else {
      iv_locationData = util::Location(".");
    }
    assert(EXISTS(iv_locationData));
    UIMA_TPRINT(_TEXT("value: ") << iv_locationData);

    /* determine work path */
    str = UIMA_ENVVAR_SUFFIX_WORKPATH;
    util::EnvironmentVariableQueryOnly clEnvVarWorkPath(str.c_str());
    UIMA_TPRINT(_TEXT("querying envvar: ") << str.c_str());
    if (clEnvVarWorkPath.hasValue()) {
      iv_locationWork = util::Location(clEnvVarWorkPath.getValue());
    } else {
      iv_locationWork = util::Location();                   // TMP directory
    }
    assert(EXISTS(iv_locationWork));
    UIMA_TPRINT(_TEXT("value: ") << iv_locationWork);

    /* determine iv_bIgnoreAnnotatorPathSpec flag */       // ** Dropped this

    // we must make sure real values our ini files are read with . (e.g. "0.5")
    setlocale(LC_NUMERIC, "C");

    UIMA_TPRINT(_TEXT("workpath: ") << (const char*)iv_locationWork);
    UIMA_TPRINT(_TEXT("datapath: ") << (const char*)iv_locationData);
    clTrace.dump(_TEXT("Work path"), iv_locationWork.getAsCString());
    clTrace.dump(_TEXT("Data path"), iv_locationData.getAsCString());

#ifndef NDEBUG
    /* check whether the message string table apepars correct and consistent */
    ErrorMessage                clMsgSig1(UIMA_MSG_ID_SIGNATURE_BEGIN);
    ErrorMessage                clMsgSig2(UIMA_MSG_ID_SIGNATURE_END);

    /* the signatures at the start and end must match! */
    if (strcmp(clMsgSig1.asString().c_str(), clMsgSig2.asString().c_str()) != 0) {
      cerr << "Internal build error"
      << "String table in uima/msgstrtab.h is inconsistent!" << endl
      << "Signature Id BEGIN: " << clMsgSig1.getMessageID() << endl
      << "   Found message: "   << clMsgSig1 << endl
      << "Signature Id END: "   << clMsgSig2.getMessageID() << endl
      << "   Found message:"    << clMsgSig2 << endl;
      assert(false);
    }
#endif

    //read environement setting to register Sofa Stream handlers
    //mappings are specified as:
    //         UIMACPP_STREAMHANDLERS=urischeme:dllfilename urischeme2:dllfilename2
    str = UIMA_ENVVAR_SOFA_STREAM_HANDLERS;
    util::EnvironmentVariableQueryOnly clEnvVarStreamHandlers(str.c_str());
    UIMA_TPRINT(_TEXT("querying envvar: ") << str.c_str());
    TCHAR *  cpszURIToStreamHandlerMap=0;
    if (clEnvVarStreamHandlers.hasValue()) {
      cpszURIToStreamHandlerMap = (TCHAR*) clEnvVarStreamHandlers.getValue();
      assert(EXISTS(cpszURIToStreamHandlerMap));
      UIMA_TPRINT(_TEXT("value: ") << cpszURIToStreamHandlerMap);

      TCHAR * urischeme =0;
      TCHAR * handlerdllfilename=0;

      TCHAR * curptr = cpszURIToStreamHandlerMap;
      TCHAR * chptr  = 0;

      //parse string to get each uri:dllfilename pair ... separated by one or more spaces
      while (curptr != NULL) {
        while (*curptr == ' ')
          ++curptr;
        chptr = strchr(curptr, ':');

        int len = chptr - curptr;
        if (len > 0) {
          //get the uri scheme
          urischeme = new char[len+1];
          strncpy(urischeme, curptr, len);
          urischeme[len]='\0';

          curptr = chptr+1;
          chptr = strchr(curptr, ' ');
          if (chptr == NULL)
            len =  strlen(curptr);
          else
            len = chptr - curptr;

          //get dll and register
          if (len > 0) {
            handlerdllfilename = new char[len+1];
            strncpy(handlerdllfilename,curptr, len);
            handlerdllfilename[len]='\0';
            registerStreamHandlerForURIScheme(urischeme, handlerdllfilename);
            delete [] handlerdllfilename;
          }
          delete [] urischeme;
        }

        //move ptr
        if (chptr==NULL)
          curptr=chptr;
        else
          curptr = chptr+1;
      }  //while
    } //if

    //register the Sofa Data Stream Handler for the 'file' URI scheme
    //registerStreamHandlerForURIScheme("file", "sofafilestreamhandler");
  }


  void ResourceManager::deleteResourceList( TyResourceList & rResList) {
    int i;
    for (i=rResList.size()-1; i>=0 ;--i) {
      ResourceABase * rResource = rResList[i];
      UIMA_TPRINT("   deleting resource " << i << ": " << rResource);
      assert( EXISTS(rResource) );
      UIMA_TPRINT("       key: " << rResource->getKey() );
      UIMA_TPRINT("  de-initing resource");
      rResource->deInit();
      assert( EXISTS(rResource) );
      delete rResource;
      rResList[i] = NULL;
      UIMA_TPRINT("   done");
    }
  }


  /* ----------------------------------------------------------------------- */
  /*       Protected implementation                                          */
  /* ----------------------------------------------------------------------- */

  /* ----------------------------------------------------------------------- */
  /*       Public implementation                                             */
  /* ----------------------------------------------------------------------- */

  ResourceManager::~ResourceManager(void)
  /* ----------------------------------------------------------------------- */
  {
    UIMA_TPRINT("Searching...");
    // it is important that annotators are deInited (unloaded)
    // last due to the following scenario:
    // a annotator p derives a class from ResourceABase, the class definition
    // is thus only valid during the lifetime in p. However, all resources
    // are handled by the resource manager. After the annotator is unloaded
    // the virtual function table of the resource is destroyed
    // (MS .NET and MSVC++6), thus the deInit() call fails.

    // use this factory just to get the kind string
    internal::ResourceAnnotatorFileFactory annotatorFactory;
    icu::UnicodeString const & crAnnotatorKind = annotatorFactory.getKind();

    TyResourceList * pAnnotators = NULL;

    TyResources::iterator it;
    for (it = iv_resources.begin(); it != iv_resources.end(); ++it) {
      TyResourceList & rResList = (*it).second;
      UIMA_TPRINT("Deleting all resources with kind: " << (*it).first);
      if ( (*it).first == crAnnotatorKind ) {
        assert( pAnnotators == NULL );
        pAnnotators = & rResList;
      } else {
        deleteResourceList( rResList );
      }
    }

    TyURIStreamHandlers::iterator ite;
    for (ite = iv_streamhandlers.begin(); ite != iv_streamhandlers.end(); ++ite) {
      util::Filename * pDllFile = (util::Filename *) (*ite).second;
      UIMA_TPRINT("Deleting stream handler dll file for uri scheme: " << (*ite).first);
      delete pDllFile;
    }


    // unload annotators at last, if any
    if (pAnnotators != NULL) {
      deleteResourceList( *pAnnotators );
    }
    iv_resources.clear();

    UIMA_TPRINT("  ...seek and destroy!");
  }

  const util::Location & ResourceManager::getLocationWork(void) const
  /* ----------------------------------------------------------------------- */
  {
    assert(iv_utLastErrorId != UIMA_ERR_RESMGR_OUT_OF_MEMORY );
    return iv_locationWork;
  }

  const util::Location & ResourceManager::getLocationData(void) const
  /* ----------------------------------------------------------------------- */
  {
    assert(iv_utLastErrorId != UIMA_ERR_RESMGR_OUT_OF_MEMORY );
    return iv_locationData;
  }

  void ResourceManager::setNewLocationWork(const util::Location & crclLocation)
  /* ----------------------------------------------------------------------- */
  {
    util::Trace                 clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_RESOURCE_MGR);

    assert(iv_utLastErrorId != UIMA_ERR_RESMGR_OUT_OF_MEMORY );
    clTrace.dump(_TEXT("New work path"), crclLocation.getAsCString());
    iv_locationWork = crclLocation;
  }

  void ResourceManager::setNewLocationData(const util::Location & crclLocation)
  /* ----------------------------------------------------------------------- */
  {
    util::Trace                 clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_RESOURCE_MGR);

    assert(iv_utLastErrorId != UIMA_ERR_RESMGR_OUT_OF_MEMORY );
    clTrace.dump(_TEXT("New data path"), crclLocation.getAsCString());
    iv_locationData = crclLocation;
  }

  util::DllProcLoaderFile * ResourceManager::requestAnnotatorFile(const util::Filename & crclFilename)
  /* ----------------------------------------------------------------------- */
  {
    internal::ResourceAnnotatorFileFactory factory;

    icu::UnicodeString us( crclFilename.getAsCString() );
    uima::ErrorInfo errInfo;
    internal::ResourceAnnotatorFile const * cpResAnnotatorFile = (internal::ResourceAnnotatorFile const *) getResource( us, factory, errInfo);
	if (errInfo.getErrorId()==UIMA_ERR_NONE && EXISTS(cpResAnnotatorFile)) {
      return cpResAnnotatorFile->getAnnotatorFile();
	} else {
		std::string err = "ResourceManager::requestAnnotatorFile() failed to find ";
        err += crclFilename;
        std::cerr<<"\033[31m"<<err<<std::endl;
        ResourceManager::getInstance().getLogger().logError(err);
        UIMA_EXC_THROW_NEW(Uima_runtime_error,
                              errInfo.getErrorId(),
                              errInfo.getMessage(),
                              errInfo.getMessage().getMessageID(),
                              ErrorInfo::unrecoverable);
	}
  }

  ResourceABase const * ResourceManager::getResource(uima::Language const & crLang,
      ResourceFactoryABase const & crFactory,
      ErrorInfo & rErrInfo) {
    icu::UnicodeString us( crLang.asString().c_str() );
    return getResource(us, crFactory, rErrInfo);
  }

  ResourceABase const * ResourceManager::getResource(icu::UnicodeString const & crKey,
      ResourceFactoryABase const & crFactory,
      uima::ErrorInfo & rErrInfo) {
//      assertWithMsg(false, "Implement caching functionality");
    icu::UnicodeString const & crKind = crFactory.getKind();
    UIMA_TPRINT("Creating resource with kind " << crKind << " and key " << crKey);
    // acquire mutex lock

    TyResourceList & rResList = iv_resources[crKind];
    ResourceABase * pResource = NULL;

    TyResourceList::const_iterator cit;
    for (cit = rResList.begin(); cit != rResList.end(); ++cit) {
      if ( (*cit)->getKey() == crKey ) {
        pResource = *cit;
        break;
      }
    }
    // if the resource was not found
    if (pResource == NULL) {
      pResource = crFactory.createResource( crKey );
      pResource->init(rErrInfo);
      if (rErrInfo.getErrorId() != UIMA_ERR_NONE) {
        delete pResource;
        return NULL;
      }
      rResList.push_back(pResource);
    }
    // release mutex lock
    UIMA_TPRINT("...resource created: " << pResource);
    assert( EXISTS( pResource ) );
    return pResource;

  }

  void ResourceManager::enableSchemaValidation(bool aEnable) {
    bDoSchemaValidation=aEnable;
  }

  bool ResourceManager::doSchemaValidation() {
    return bDoSchemaValidation;
  }

  bool  ResourceManager::isSchemaAvailable() {
    return bIsSchemaAvailable;
  }

  LogStream::EnEntryType  ResourceManager::getLoggingLevel() {
    return iv_logLevel;
  }

  void ResourceManager::setLoggingLevel(LogStream::EnEntryType level) {
    iv_logLevel=level;
  }

  TCHAR const *  ResourceManager::getSchemaInfo() {
    return schemaInfo;
  }

  void ResourceManager::registerLogger(Logger * pLogger) {
 	iv_loggers.push_back(pLogger);
  }

  vector<Logger*> & ResourceManager::getLoggers() {
 	return iv_loggers;
  }
  
  void ResourceManager::unregisterLogger(Logger * pLogger) {
      vector<Logger*>::iterator iter;
 	for (iter=iv_loggers.begin(); iter  != iv_loggers.end();iter++) {
  		if (*iter == pLogger) {
		    iv_loggers.erase(iter);
                return;
            }
      }
 	
      string str = "Logger not found. Could not unregister.";
      UIMA_EXC_THROW_NEW(Uima_runtime_error,
                  UIMA_MSG_ID_LITERAL_STRING,
                  UIMA_MSG_ID_LITERAL_STRING,
                  ErrorMessage(UIMA_MSG_ID_LITERAL_STRING, str.c_str()),
                  ErrorInfo::unrecoverable);

  }


  void ResourceManager::registerFactory(icu::UnicodeString const & crKind, ResourceFactoryABase & crFactory) {
    iv_resourceFactories.insert(TyResourceFactories::value_type(crKind, &crFactory));
  }

  void ResourceManager::deRegisterFactory(icu::UnicodeString const & crKind, ResourceFactoryABase & crFactory) {
    TyResourceFactories::iterator it = iv_resourceFactories.find(crKind);
    if (it != iv_resourceFactories.end()) {
      assert( (*it).second == &crFactory );
      iv_resourceFactories.erase(it);
    }
  }

  ResourceABase const * ResourceManager::getResource(
    icu::UnicodeString const & crKey,
    icu::UnicodeString const & crKind,
    ErrorInfo &                errorInfo) {
    TyResourceFactories::iterator  it = iv_resourceFactories.find(crKind);
    if (it == iv_resourceFactories.end()) {
      errorInfo.setErrorId(UIMA_ERR_RESMGR_NO_RESOURCE_FACTORY_FOR_KIND);
      return NULL;
    }
    assert(it->first == crKind);
    assert(EXISTS(it->second));

    return getResource(crKey, *(it->second), errorInfo);
  }


  /**
   * Register a stream handler dll filename for a given URI scheme.
   * A URI scheme may be registered only once in an application. 
   */
  util::Filename const * ResourceManager::registerStreamHandlerForURIScheme(TCHAR const * uriScheme,
      TCHAR const * dllFilename) {
    std::string uriSchemeStr(uriScheme);
    TyURIStreamHandlers::iterator  ite;
    ite = iv_streamhandlers.find(uriSchemeStr);
    if (ite != iv_streamhandlers.end() ) {
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_SCHEMEHANDLER_DUPLICATE);
      errMsg.addParam(dllFilename);
      errMsg.addParam(uriScheme);
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    }
    util::Filename * pDllFile = new util::Filename(dllFilename);
    if (pDllFile == NULL) {
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_SCHEMEHANDLER_LOAD);
      errMsg.addParam(dllFilename);
      errMsg.addParam(uriScheme);
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    }
    iv_streamhandlers.insert(TyURIStreamHandlers::value_type(uriSchemeStr, pDllFile));
    return(util::Filename*)pDllFile;
  }


  /**
   * Return the dll file registered for the specified uri scheme if found.
   * Otherwise, returns null.
   */
  util::Filename const * ResourceManager::getStreamHandlerForURIScheme(std::string uriScheme) {

    TyURIStreamHandlers::iterator  ite = iv_streamhandlers.find(uriScheme);
    if (ite == iv_streamhandlers.end() ) {
      return NULL;
    } else {
      return(util::Filename *) ite->second;
    }

  }




  /* ----------------------------------------------------------------------- */
  /*       Static implementation                                             */
  /* ----------------------------------------------------------------------- */

  /* static */ ResourceManager & ResourceManager::createInstance(const TCHAR * cpszInstance, const TCHAR * cpszProductPrefix)
  /* ----------------------------------------------------------------------- */
  {
    // acquire mutex
    if (NOTEXISTS(cv_pclSingletonInstance)) {

      // First must initialize apr (re-init is OK)
      apr_status_t rv = apr_initialize();
      if (rv != APR_SUCCESS) {
        char errBuf[256];
        apr_strerror(rv, errBuf, sizeof(errBuf));
        UIMA_EXC_THROW_NEW(AprFailureException,
                           UIMA_ERR_APR_FAILURE,
                           ErrorMessage(UIMA_MSG_ID_EXC_APR_ERROR,errBuf),
                           ErrorMessage(UIMA_MSG_ID_EXCON_APR_FUNCTION,"apr_initialize"),
                           ErrorInfo::unrecoverable);
      }

      cv_pclSingletonInstance = new ResourceManager(cpszInstance, cpszProductPrefix);
      assert(EXISTS(cv_pclSingletonInstance));
      // Initialize the ICU
      UErrorCode status = U_ZERO_ERROR;
      u_init(&status);
      if (status != U_ZERO_ERROR) {
        char buffer[100];
        sprintf(buffer, "ICU init failed with %d", status);
        UIMA_EXC_THROW_NEW(Uima_runtime_error,
                           UIMA_MSG_ID_LITERAL_STRING,
                           UIMA_MSG_ID_LITERAL_STRING,
                           ErrorMessage(UIMA_MSG_ID_LITERAL_STRING, buffer),
                           ErrorInfo::unrecoverable);
      }
      try {
        // initialize XML4C stuff
        XMLPlatformUtils::Initialize();
      } catch (XMLException& ) {
        cv_pclSingletonInstance->iv_utLastErrorId = UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_XML4C;
        assertWithMsg(false, "XML4C initialization failed");
      }

      //create the fileLogger if UIMACPP_LOGFILE env variable is set
      /* determine log file name */
      string str = UIMA_ENVVAR_LOG_FILE;
      util::EnvironmentVariableQueryOnly clEnvVarLogFilePath(str.c_str());
      UIMA_TPRINT(_TEXT("querying envvar: ") << str.c_str());
      if (clEnvVarLogFilePath.hasValue()) {
        const TCHAR *  cpszLogFile = clEnvVarLogFilePath.getValue();
        UIMA_TPRINT(_TEXT("value: ") << cpszLogFile);

        /* create an instance of FileLogger and register it. */
        cv_pclSingletonInstance->iv_fileLogger = new FileLogger(cpszLogFile);

        if (cv_pclSingletonInstance->iv_fileLogger == NULL) {   //Need to handle this better
          //cerr << "Could not open the log file " << cpszLogFile << endl;
          str = "Could not create FileLogger";
          str += cpszLogFile;
          UIMA_EXC_THROW_NEW(Uima_runtime_error,
                           UIMA_MSG_ID_LITERAL_STRING,
                           UIMA_MSG_ID_LITERAL_STRING,
                           ErrorMessage(UIMA_MSG_ID_LITERAL_STRING, str.c_str()),
                           ErrorInfo::unrecoverable);
        } else {
          cv_pclSingletonInstance->registerLogger(cv_pclSingletonInstance->iv_fileLogger);
        }
      } 

      //instantiate framework logger
      cv_pclSingletonInstance->iv_frameworkLogger = new LogFacility(UnicodeString("org.apache.uima.cpp"), cv_pclSingletonInstance->iv_logLevel);

      cv_pclSingletonInstance->iv_frameworkLogger->logMessage("ResourceManager Instance created.");
    } // release mutex
    UIMA_TPRINT("ResourceManager instance created");
    return(*cv_pclSingletonInstance);
  }

  /* static */ ResourceManager & ResourceManager::getInstance(void)
  /* ----------------------------------------------------------------------- */
  {
    assert(EXISTS(cv_pclSingletonInstance));
    return(*cv_pclSingletonInstance);
  }

  /* static */ bool ResourceManager::hasInstance(void)
  /* ----------------------------------------------------------------------- */
  {
    return((bool) EXISTS(cv_pclSingletonInstance));
  }
  /* ----------------------------------------------------------------------- */
  void ResourceManager::deleteInstance(void) {
    UIMA_TPRINT(_TEXT("deleting..."));
    // acquire mutex
    if (cv_pclSingletonInstance != 0 ) {
      try {
        XMLPlatformUtils::Terminate();
      }
      catch (const XMLException& ) {
        cv_pclSingletonInstance->iv_utLastErrorId = UIMA_ERR_RESMGR_COULD_NOT_TERMINATE_XML4C;
        assertWithMsg(false, "XML4C termination failed");
      }
     
      if (cv_pclSingletonInstance->iv_frameworkLogger != NULL) {
        delete cv_pclSingletonInstance->iv_frameworkLogger;
      }

      if (cv_pclSingletonInstance->iv_fileLogger != NULL) {
        delete cv_pclSingletonInstance->iv_fileLogger;
      }

      assert(EXISTS(cv_pclSingletonInstance));
      delete cv_pclSingletonInstance;
      cv_pclSingletonInstance = 0;

      // Terminate apr (undo matching apr_initialize)
      apr_terminate();
//#if !defined( NDEBUG ) && defined(_MSC_VER) && defined(_CRTDBG_MAP_ALLOC)   
//      int iRetVal = _CrtDumpMemoryLeaks();
//#endif
      UIMA_TPRINT("ResMgr instance deleted");
    }
    assert( cv_pclSingletonInstance == 0 );
    // release mutex
  }

  bool
  ResourceManager::createFilenameForLanguage(Language & rclLanguage,
      const TCHAR * cpszExtension,
      bool bUseAlternateTerritories,
      const util::Location & crclDirToUse,
      util::Filename & rclFilename)  {
    string                     str(rclLanguage.asString());

    assert(EXISTS(cpszExtension));
    assert(*cpszExtension == _TEXT('.'));              /* extension starts with a dot */
    /* we need to restrict ourselves to DOS 8+3 filenames */
    if (str.length() > UIMA_MAX_VALID_FILENAME_SIZE) {
      str.resize(UIMA_MAX_VALID_FILENAME_SIZE);
    }
    /* create a filename based on the complete language name
       e.g. language is en-us and file is en-us.tsw */
    util::Filename              clFilename(crclDirToUse, str.c_str(), cpszExtension);
    UIMA_TPRINT(_TEXT("1st Filename: '") << clFilename.getAsCString() << _TEXT("' existent: ") << clFilename.isExistent());
    if (clFilename.isExistent()) {
      rclFilename = clFilename;
      return(true);                                   /* done! */
    } else {
      /* if the user did specify a territory, we could look a little bit further... */
      if (rclLanguage.hasTerritory()) {
        /* give it another try using just the language name without the territory
           e.g. language is en-us and file is en.tsw */
        clFilename.setNew( crclDirToUse,rclLanguage.getLanguageCode(),cpszExtension );
        UIMA_TPRINT(_TEXT("2nd Filename: '") << clFilename.getAsCString() << _TEXT("' existent: ") << clFilename.isExistent());
      }
    }
    if (bUseAlternateTerritories && !clFilename.isExistent()) {
      /* give it another try using just the language name without the territory
         e.g. language is en and file is en-us.tsw
         this means a little bit more effort - we need to walk the directory */
      util::DirectoryWalk     clDirWalk(crclDirToUse.getAsCString());
      string                  strSearchPattern(rclLanguage.getLanguageCode());

      strSearchPattern += _TEXT("*");
      strSearchPattern += cpszExtension;
      UIMA_TPRINT(_TEXT("Search pattern: '") << strSearchPattern.c_str() << _TEXT("'"));
      while (clDirWalk.isValid()) {
        if (clDirWalk.isFile() && clDirWalk.matchesWildcardPattern(strSearchPattern.c_str())) {
          clFilename.setNewName(clDirWalk.getNameWithoutPath());
          break;
        }
        clDirWalk.setToNext();
      }
      UIMA_TPRINT(_TEXT("3rd Filename: '") << clFilename.getAsCString() << _TEXT("' existent: ") << clFilename.isExistent());
    }
    rclFilename = clFilename;
    return(clFilename.isExistent());
  }

  LogFacility & ResourceManager::getLogger() {
    return *iv_frameworkLogger;
  }

  icu::UnicodeString ResourceManager::resolveFilename(icu::UnicodeString const & filename, icu::UnicodeString const & lastFilename) {
    auto_array<char> filename_cstr( new char[filename.length() + 1] );
    filename.extract(0, filename.length(), filename_cstr.get());

    auto_array<char> lastFilename_cstr( new char[lastFilename.length() + 1] );
    lastFilename.extract(0, lastFilename.length(), lastFilename_cstr.get());

    //build the filename
    util::Filename fileLoc(filename_cstr.get());

    // don't try our search mimic for absolute paths
    if (fileLoc.isAbsolute()) {
      return filename;
    }

    // relative path to the current directory
    if (fileLoc.isExistent()) {
      return filename;
    }

    // try in the same directory as lastFilename
    util::Filename locSameDirAsLast(lastFilename_cstr.get());
    locSameDirAsLast.setNewName(filename_cstr.get());

    if (locSameDirAsLast.isExistent()) {
      return locSameDirAsLast.getAsCString();
    }

    // try in the UIMACPP data directory
    util::Location const & fallbackLoc = ResourceManager::getInstance().getLocationData();

    std::string nameInDataDir( fallbackLoc.getAsCString() );
    nameInDataDir += filename_cstr.get();
    util::Filename fileInDataDir( nameInDataDir.c_str() );
    fileInDataDir.normalizeAbsolute();                // Normalize to native / or \ separators
    if (fileInDataDir.isExistent()) {
      return icu::UnicodeString(fileInDataDir.getAsCString());
    }

    nameInDataDir.clear();
    nameInDataDir = fallbackLoc.getAsCString();
    nameInDataDir += "descriptors/";
    nameInDataDir += filename_cstr.get();
    fileInDataDir.setNew(nameInDataDir.c_str());
    fileInDataDir.normalizeAbsolute();
    if (fileInDataDir.isExistent()) {
      return icu::UnicodeString(fileInDataDir.getAsCString());
    }

    nameInDataDir.clear();
    nameInDataDir = fallbackLoc.getAsCString();
    nameInDataDir += "specifiers/";
    nameInDataDir += filename_cstr.get();
    fileInDataDir.setNew(nameInDataDir.c_str());
    fileInDataDir.normalizeAbsolute();
    if (fileInDataDir.isExistent()) {
      return icu::UnicodeString(fileInDataDir.getAsCString());
    }

    // return the original filename here, will trigger an XML exception since
    // it couldn't be found anywhere
    return filename;
  }



}

/* <EOF> */




