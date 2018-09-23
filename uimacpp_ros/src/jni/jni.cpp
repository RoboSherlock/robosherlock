/** @name jni.cpp

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

   Description: The C++ functions for the Java - C++ interface

-----------------------------------------------------------------------------


   08/04/2000  Initial creation

-------------------------------------------------------------------------- */

//#define DEBUG_VERBOSE 1
/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
// this is included ONCE for the main source file of each binary
//#include "uima/configure.h"

#include <uima/pragmas.hpp>

#include <uima/macros.h>
#include <uima/location.hpp>
#include <uima/filename.hpp>

#include <uima/api.hpp>
#include <uima/jni.hpp>

#include <uima/sofaid.hpp>
#include <uima/internal_casimpl.hpp>
#include <uima/cas.hpp>
#include <uima/internal_engine_base.hpp>
#include <uima/internal_casdeserializer.hpp>
#include <uima/result_specification.hpp>
#include <uima/org_apache_uima_uimacpp_UimacppEngine.h>
#include <uima/jni_utils.hpp>
#include <uima/casiterator.hpp>
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */


#define UIMA_JNI_CONCAT(x,y) x##y
#define JAVA_PREFIX(FUNC) UIMA_JNI_CONCAT(Java_org_apache_uima_uimacpp_UimacppEngine_, FUNC )
#define CONST_PREFIX(CONST) UIMA_JNI_CONCAT(org_apache_uima_uimacpp_UimacppEngine_, CONST )

#define JAVA_LOGGER_PROXY "org/apache/uima/uimacpp/UimacppAnalysisComponent"

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
static JNILogger * singleton_jni_logger = 0;
static jobject getSerializedCasData (JNIEnv* jeEnv, jobject joJTaf, jint jiWhichData, uima::internal::SerializedCAS & crSerializedCAS);

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */
using namespace std;

/********************************************************************
 ***** JNILogger
 ********************************************************************/
  JNILogger::JNILogger(JNIEnv * env) : /*iv_jnienv(env),*/ cv_clazz(0), cv_logMethod(0) {
    assert( EXISTS(env) );
    try {
    env->GetJavaVM(&iv_jvm);
    CHECK_FOR_JNI_EXCEPTION(env);

    cv_clazz = env->FindClass(JAVA_LOGGER_PROXY);
    if (cv_clazz == NULL ) {
        uima::ResourceManager::getInstance().getLogger().logError( 
        JAVA_LOGGER_PROXY " class not found. Could not setup Java logging. ");
      env->ExceptionClear();
      return;
    }
    CHECK_FOR_JNI_EXCEPTION(env);

    cv_clazz = (jclass) env->NewGlobalRef(cv_clazz);
    if (cv_clazz == NULL) {
      uima::ResourceManager::getInstance().getLogger().logError( 
                  "Setup global reference to " JAVA_LOGGER_PROXY " class failed. Could not setup Java logging. ");
      cerr << "JNILogger() ERROR: CPPJEDIIEngine could not construct " << endl;
      env->ExceptionClear();
      return;
    }
    CHECK_FOR_JNI_EXCEPTION(env);

     //query the current logging level
     jmethodID iv_getLoggingLevelMethod = env->GetStaticMethodID(cv_clazz,
                                             "getLoggingLevel",
                                             "()I");
     if (iv_getLoggingLevelMethod == NULL) {
       uima::ResourceManager::getInstance().getLogger().logError( 
            JAVA_LOGGER_PROXY ".getLoggingLevel() not found. Could not setup Java logging. " );
       cout << "JNILogger() ERROR: CPPJEDIIEngine.getLoggingLevel() not found " << endl;
       env->ExceptionClear();
       return;
      }
     CHECK_FOR_JNI_EXCEPTION(env);

      //log method
      cv_logMethod = env->GetStaticMethodID(cv_clazz, "log", "("
                       "I"       // level
                       "Ljava/lang/String;"  // source class
                       "Ljava/lang/String;" // source method
                       "Ljava/lang/String;" // message
                       ")V");

      if (cv_logMethod == NULL) {
        uima::ResourceManager::getInstance().getLogger().logError( 
            JAVA_LOGGER_PROXY ".log(int,string,string,string) not found. Could not setup Java logging. " );
          cout << "JNILogger() ERROR: CPPJEDIIEngine.log() not found " << endl;
          env->ExceptionClear();
          return;
      }
      CHECK_FOR_JNI_EXCEPTION(env);

      //get the current logging level
      jint logginglevel = env->CallStaticIntMethod(cv_clazz, iv_getLoggingLevelMethod);

      if (logginglevel == 0) {
        uima::ResourceManager::getInstance().getLogger().logError( 
            "JNILogger() Could not determine current logging level. Setup Java logging failed. " );
          cout << "JNILogger() ERROR: calling CPPJEDIIEngine.getLoggingLevel() " << endl;
          env->ExceptionClear();
          return;
      }
      CHECK_FOR_JNI_EXCEPTION(env);

      if (logginglevel == 3) {
        uima::ResourceManager::getInstance().setLoggingLevel(uima::LogStream::EnError);
      } else if (logginglevel == 2) {
        uima::ResourceManager::getInstance().setLoggingLevel(uima::LogStream::EnWarning);
      } else {
        uima::ResourceManager::getInstance().setLoggingLevel(uima::LogStream::EnMessage);
      }

    } catch (...) {
      cout << "Exception in JNILogger() " << endl;
      env->ExceptionDescribe();
    }
  }
  

  void JNILogger::log(uima::LogStream::EnEntryType entype,
                 string classname,
                 string methodname,
                 string message,
                 long lUserCode) {

      //get JNI env associated with the current thread
      JNIEnv * jnienv=0;
      try {
      iv_jvm->AttachCurrentThread((void**)&jnienv, NULL);

      if (jnienv == 0) {
        cerr << "JNILogger::log() failed to get JNI env handle." << endl;
        return;
      }

      stringstream str;
      if (entype == uima::LogStream::EnMessage) {
        if (lUserCode != 0) {
           str << lUserCode << " " << message;
        } else {
		  str << message;
        }
      } else {
        str << lUserCode << " " << message;
      }
      
      UnicodeString msg(str.str().c_str());
      // Convert the std::strings to Unicode using the default converter
      UnicodeString ustrsource(classname.c_str(), classname.length());
      UnicodeString ustrmethod(methodname.c_str(), methodname.length());
      jstring jsrcclass = jnienv->NewString((jchar const *) ustrsource.getBuffer(), ustrsource.length());
      jstring jsrcmethod = jnienv->NewString((jchar const *) ustrmethod.getBuffer(), ustrmethod.length());
      jstring jmessage = jnienv->NewString((jchar const *) msg.getBuffer(), msg.length());

      jint loglevel;
      if ( entype == uima::LogStream::EnError) {
        loglevel = 3;
      } else  if ( entype == uima::LogStream::EnWarning )  {
        loglevel  = 2;
      } else {
        loglevel = 1;
      }

      // Call exception clear
      jnienv->ExceptionClear();

      jnienv->CallStaticVoidMethod(cv_clazz,
                                   cv_logMethod,
                                   loglevel,
                                   jsrcclass,
                                   jsrcmethod,
                                   jmessage);

      // Check for exceptions :
      jthrowable exc = jnienv->ExceptionOccurred();
      if (exc != NULL) {
        jnienv->ExceptionDescribe();
        jnienv->ExceptionClear();
      }
      iv_jvm->DetachCurrentThread();
      //cout << "ThreadId: " << threadid << " JNILogger::log() DONE" << endl;
    } catch (...) {
      iv_jvm->DetachCurrentThread();
      cout << "JNILogger::log(...) Exception in JavaLogging()" << endl;
    }
  }
/********************************************************************
 ***** JNI Functions
 ********************************************************************/

/*********************************************************************
 * static method to get this UIMACPP JNI version
 */
JNIEXPORT jstring JNICALL JAVA_PREFIX(getVersionJNI) (JNIEnv* jeEnv, jclass) {
  try {
    jstring jsVersion = jeEnv->NewStringUTF(UIMA_JNI_VERSION);
    ASSERT_NO_JNI_EXCEPTION(jeEnv);
    return jsVersion;
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return NULL;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION_WITH_RETURN_VALUE(jeEnv, NULL);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return NULL;
  }
}

/*******************************************************************
 * create UIMACPP resource manager
 */
JNIEXPORT void JNICALL JAVA_PREFIX(createResourceManagerJNI) (JNIEnv* jeEnv, jclass) {
  try {
    UIMA_TPRINT("createResourceManagerJNI() entered");

    (void) uima::ResourceManager::createInstance("Main", UIMA_STRINGIFY(UIMA_PREFIX));

    UIMA_TPRINT("createResourceManagerJNI() finished");
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString());
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION(jeEnv);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return;
  }

}


/********************************************************************
 * configure UIMACPP resource manager
 */
JNIEXPORT void JNICALL JAVA_PREFIX(configureResourceManagerJNI) (JNIEnv* jeEnv,
    jclass,
    jstring jsWorkDirectory,
    jstring jsDataDirectory) {
  try {
    UIMA_TPRINT("configureResourceManagerJNI entered");
 
    uima::ResourceManager& rResourceManager = uima::ResourceManager::getInstance();

    JNIUString workDir(jeEnv, jsWorkDirectory);
    string strWorkDirectory;
    workDir.convertToSystemCodePage(strWorkDirectory);

    JNIUString dataDir(jeEnv, jsDataDirectory);
    string strDataDirectory;
    dataDir.convertToSystemCodePage(strDataDirectory);

    UIMA_TPRINT(" configuring resource manager with");
    UIMA_TPRINT("   working directory: " << strWorkDirectory);
    UIMA_TPRINT("   data    directory: " << strDataDirectory);

    // Since UIMA_DATAPATH is a list of paths and UIMACPP wants just one,
    // we'll take the first by pretending to search for a empty file.
    // Will find the first path that exists, or the last if none exist.
    // No longer check that the data path exists.

    uima::util::Filename dataPath("");
    dataPath.determinePath( strDataDirectory.c_str());
    uima::util::Location dataLocation(dataPath.getAsCString());

    uima::util::Location workLocation(strWorkDirectory.c_str());
    if (! workLocation.isExistent() ) {
      uima::ErrorInfo errInfo(uima::ErrorMessage(UIMA_MSG_ID_RESMGR_WORKDIR_DOES_NOT_EXIST, strWorkDirectory.c_str()),
                              UIMA_ERR_RESMGR_WORK_DIR_DOES_NOT_EXIST,
                              uima::ErrorInfo::recoverable);
      JNIUtils::throwNewInternalException(jeEnv, errInfo);
      return;
    }

    rResourceManager.setNewLocationWork(workLocation);
    rResourceManager.setNewLocationData(dataLocation);
    
    UIMA_TPRINT("configureResourceManagerJNI finished");
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION(jeEnv);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return;
  }
}

/**********************************************************************
 * constructs a new UIMACPP engine and CASSerializer
 * and sets the field in the Java object
 */
JNIEXPORT void JNICALL JAVA_PREFIX(constructorJNI) (JNIEnv* jeEnv,
    jobject joJTaf) {
  try {
    UIMA_TPRINT("constructorJNI() entered");

    uima::JNIInstance * pInstance = new uima::JNIInstance();
    if (pInstance == NULL) {
      uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_OUT_OF_MEMORY,
                              UIMA_ERR_ENGINE_OUT_OF_MEMORY,
                              uima::ErrorInfo::unrecoverable);
      JNIUtils::throwNewInternalException(jeEnv, errInfo);
      return;
    }
    //setup JNI logger
    if (singleton_jni_logger == NULL) {
      cout << "creating JNILogger" << endl;
      singleton_jni_logger = new JNILogger(jeEnv);
      uima::ResourceManager::getInstance().registerLogger(singleton_jni_logger);
    }

    // setting engine
    JNIUtils::setCppInstance(jeEnv, joJTaf, pInstance);

    UIMA_TPRINT("constructorJNI() finished");
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION(jeEnv);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return;
  }
}


/*********************************************************************
 * deletes the objects created by constructorJNI()
 */
JNIEXPORT void JNICALL JAVA_PREFIX(destructorJNI) (JNIEnv* jeEnv, jobject joJTaf) {
  try {
    UIMA_TPRINT("destructorJNI() entered");

    uima::JNIInstance* pInstance = JNIUtils::getCppInstance(jeEnv, joJTaf);
    assert( EXISTS(pInstance) );

    delete pInstance;

    UIMA_TPRINT("destructorJNI() finished");
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION(jeEnv);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return;
  }
}


/**********************************************************************
 * initialize engine
 */
JNIEXPORT void JNICALL JAVA_PREFIX(initializeJNI) (JNIEnv* jeEnv,
    jobject joJTaf,
    jstring jsConfigFile
                                                  ) {
  try {
    UIMA_TPRINT("entering initializeJNI()");

    assert(jsConfigFile != NULL);

    JNIUString configFile(jeEnv, jsConfigFile);

    uima::UnicodeStringRef uref = configFile.toUStrPtrLenPair();

    uima::JNIInstance* pInstance = JNIUtils::getCppInstance(jeEnv, joJTaf);
    assert( EXISTS(pInstance) );

    uima::ErrorInfo errInfo;

    UIMA_TPRINT("creating engine");
    UIMA_TPRINT(uref);
    uima::ResourceManager::getInstance().enableSchemaValidation(false);
    // initialize engine
    uima::AnalysisEngine * pEngine = uima::Framework::createAnalysisEngine( (UChar*)uref.getBuffer(), 
       uref.length(), errInfo);
    if (errInfo.getErrorId() != UIMA_ERR_NONE) {
      assert( pEngine == NULL );
      UIMA_TPRINT("Error during initializing engine (code= " << errInfo.getErrorId() << ")" );
      JNIUtils::throwNewInternalException(jeEnv, errInfo );
      return;
    }
    pInstance->setEngine( pEngine );
    UIMA_TPRINT("engine created and set");

    UIMA_TPRINT("Creating CAS");
    // create CAS
    uima::CAS * pCAS = pEngine->newCAS();
    assert( EXISTS(pCAS) );
    pInstance->setCAS(pCAS);
    UIMA_TPRINT("CAS created");

    UIMA_TPRINT("initializeJNI() finished");
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return;
  }
#ifdef NDEBUG
  catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION(jeEnv);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return;
  }
#endif
}


/**********************************************************************
 * initialize engine
 */
JNIEXPORT void JNICALL JAVA_PREFIX(typeSystemInitJNI) (JNIEnv* jeEnv,
    jobject joJTaf,

    jobjectArray typeNames,
    jobjectArray featureNames,
    jintArray typeInheritance,
    jintArray featDecls,
    jint topTypeCode,
    jintArray featureOffsets,
    jintArray typeOrder,

    jintArray stringSubTypes,
    jobjectArray stringSubTypeValues,
    jintArray stringSubTypeValuePos,

    jobjectArray indexNames,
    jintArray nameToIndexMap,
    jintArray indexingStrategy,
    jintArray comparatorIndex,
    jintArray comparators
                                                      ) {
  try {
    UIMA_TPRINT("entering typeSystemInitJNI()");
    uima::JNIInstance* pInstance = JNIUtils::getCppInstance(jeEnv, joJTaf);
    assert( EXISTS(pInstance) );

    assert( typeNames != NULL );
    assert( featureNames != NULL );
    assert( typeInheritance != NULL );
    assert( featDecls != NULL );
    assert( typeOrder != NULL );
//      assert( featureOffsets != NULL );

    assert( indexNames != NULL );
    assert( nameToIndexMap != NULL );
    assert( indexingStrategy != NULL );
    assert( comparatorIndex != NULL );
    assert( comparators != NULL);

    vector<icu::UnicodeString> vecTypeNames, vecFeatureNames, vecIndexNames, vecSubStringValues;

    // prepare deserialization
    uima::internal::SerializedCAS rSerCAS;
    UIMA_TPRINT("converting type inh vector");
    JNIUtils::createIntVector(jeEnv, typeInheritance, rSerCAS.iv_vecTypeInheritanceTable );
    UIMA_TPRINT("   converted");
    JNIUtils::createIntVector(jeEnv, featDecls, rSerCAS.iv_vecFeatureDefinitionTable );
    UIMA_TPRINT("converting type name vector");
    JNIUtils::createStringVector(jeEnv, typeNames, rSerCAS.iv_vecTypeSymbolTable, vecTypeNames);
    UIMA_TPRINT("   converted");
    JNIUtils::createStringVector( jeEnv, featureNames,  rSerCAS.iv_vecFeatureSymbolTable, vecFeatureNames );
    JNIUtils::createIntVector(jeEnv, typeOrder, rSerCAS.iv_vecTypePriorityTable );
    //      JNIUtils::createIntVector(jeEnv, featureOffsets, rSerCAS.iv_vecFeatureOffsetTable );
    rSerCAS.iv_vecFeatureOffsetTable.clear();

    JNIUtils::createIntVector(jeEnv, stringSubTypes, rSerCAS.iv_stringSubTypes);
    JNIUtils::createStringVector(jeEnv, stringSubTypeValues, rSerCAS.iv_stringSubTypeValues, vecSubStringValues);
    JNIUtils::createIntVector(jeEnv, stringSubTypeValuePos, rSerCAS.iv_stringSubTypeValuePos);

    JNIUtils::createStringVector(jeEnv, indexNames, rSerCAS.iv_vecIndexIDTable, vecIndexNames );
    JNIUtils::createIntVector(jeEnv,comparators, rSerCAS.iv_vecComparatorDefinitionTable);
    JNIUtils::createIntVector(jeEnv,comparatorIndex, rSerCAS.iv_vecComparatorStartTable);
    JNIUtils::createIntVector(jeEnv, indexingStrategy, rSerCAS.iv_vecIndexKindTable );

    uima::AnalysisEngine * pEngine =  pInstance->getEngine();
    uima::internal::EngineBase & engineBase = uima::internal::EngineBase::promoteEngine(*pEngine);
    uima::internal::CASDefinition & casDef = engineBase.getCASDefinition();

    UIMA_TPRINT("deserializng definitions");
    // deserialize CAS definitions (type system and index info)
    uima::internal::CASDeserializer deSerializer;
    deSerializer.deserializeDefinitions( rSerCAS, casDef );
    UIMA_TPRINT("   done deserializing definitions");

    engineBase.reinitTypeSystem();

    uima::CAS * pCAS = pInstance->getCAS();
    if (pCAS != NULL) {
      delete pCAS;
      pInstance->setCAS(NULL);
    }

    UIMA_TPRINT("recreate CAS");
    pCAS = pEngine->newCAS();
    assert( EXISTS(pCAS) );
    pInstance->setCAS(pCAS);

    UIMA_TPRINT("typeSystemInitJNI() finished");
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return;
  }
#ifdef NDEBUG
  catch (...) {
    //cout << "typeSystemInitJNI: Unknown Exception " << endl;
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION(jeEnv);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return;
  }
#endif
}


/**********************************************************************
 * fill CAS
 */

JNIEXPORT void JNICALL JAVA_PREFIX(fillCASJNI) (JNIEnv* jeEnv,
    jobject joJTaf,
    jintArray heapArray,
    jintArray fsIndex,
    jobjectArray stringTable,
    jbyteArray byteHeapArray,
    jshortArray shortHeapArray,
    jlongArray longHeapArray ) {
  try {
    UIMA_TPRINT("fillCASJNI() started");
    uima::JNIInstance* pInstance = JNIUtils::getCppInstance(jeEnv, joJTaf);
    assert( EXISTS(pInstance) );

    uima::CAS * cas = pInstance->getCAS();
    assert( EXISTS(cas) );
    uima::internal::SerializedCAS serCAS;

    vector<icu::UnicodeString> vecStringTable;

    JNIUtils::createStringVector(jeEnv, stringTable, serCAS.iv_vecStringSymbolTable, vecStringTable );
#ifdef DEBUG_VERBOSE
    size_t iVerbose;
    UIMA_TPRINT("STRINGHeap: ");
    for (iVerbose=0; iVerbose<serCAS.iv_vecStringSymbolTable.size(); ++iVerbose) {
      UIMA_TPRINT("   " << iVerbose << ": " << serCAS.iv_vecStringSymbolTable[iVerbose]);
    }
#endif

    JNIUtils::createIntVector(jeEnv, heapArray, serCAS.iv_vecFSHeapArray);
#ifdef DEBUG_VERBOSE
    UIMA_TPRINT("HEAP: ");
    for (iVerbose=0; iVerbose<serCAS.iv_vecFSHeapArray.size(); ++iVerbose) {
      UIMA_TPRINT("   " << iVerbose << ": " << serCAS.iv_vecFSHeapArray[iVerbose]);
    }
#endif

    JNIUtils::createIntVector(jeEnv, fsIndex, serCAS.iv_vecIndexedFSs);

    JNIUtils::createByteVector(jeEnv,byteHeapArray, serCAS.iv_vecByteHeapArray);
    JNIUtils::createShortVector(jeEnv,shortHeapArray, serCAS.iv_vecShortHeapArray);
    JNIUtils::createLongVector(jeEnv,longHeapArray, serCAS.iv_vecLongHeapArray);
    uima::internal::CASDeserializer casDeSerializer;
    casDeSerializer.deserializeData( serCAS, *cas );
    UIMA_TPRINT("fillCASJNI() finished");
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION(jeEnv);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return;
  }
}

/*********************************************************************
 * de initialize engine
 */
JNIEXPORT void JNICALL JAVA_PREFIX(destroyJNI) (JNIEnv* jeEnv, jobject joJTaf) {
  try {
    UIMA_TPRINT("entering destroyJNI()");

    uima::JNIInstance* pInstance = JNIUtils::getCppInstance(jeEnv, joJTaf);
    assert( EXISTS(pInstance) );

    uima::AnalysisEngine * pEngine =  pInstance->getEngine();

    uima::TyErrorId tyErrorId = pEngine->destroy();
    if (tyErrorId != UIMA_ERR_NONE) {
      UIMA_TPRINT("Error during deInitializing engine (code= " << tyErrorId << ")" );
      JNIUtils::throwNewInternalException(jeEnv, pEngine->getAnnotatorContext().getLogger().getLastError() );
      return;
    }

    if (pEngine != NULL) {
      delete pEngine;
      pInstance->setEngine(NULL);
    }

    JNIUtils::setCppInstance(jeEnv, joJTaf, 0);

    uima::CAS * pCAS = pInstance->getCAS();
    if (pCAS != NULL) {
      delete pCAS;
      pInstance->setCAS(NULL);
    }

    delete pInstance;

    UIMA_TPRINT("exiting destroy()");
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION(jeEnv);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return;
  }
}





/************************************************************
 * process the CAS.
 */
JNIEXPORT void JNICALL JAVA_PREFIX(processJNI) (JNIEnv* jeEnv,
    jobject joJTaf,
    jint isTCas,
    jstring sofaName,
    jintArray resultSpecTypes,
    jintArray resultSpecFeatures) {
  try {
    //    cout << "processJNI" << endl;
    UIMA_TPRINT("entering processDocument");

    uima::JNIInstance* pInstance = JNIUtils::getCppInstance(jeEnv, joJTaf);
    assert( EXISTS(pInstance) );

    uima::AnalysisEngine * pEngine = pInstance->getEngine();
    assert( EXISTS(pEngine) );

    uima::CAS & cas = *pInstance->getCAS();
    uima::ResultSpecification rs;

    vector<uima::internal::SerializedCAS::TyNum> rsTypes;
    JNIUtils::createIntVector(jeEnv, resultSpecTypes, rsTypes);
    vector<uima::internal::SerializedCAS::TyNum> rsFeatures;
    JNIUtils::createIntVector(jeEnv, resultSpecFeatures, rsFeatures);
    uima::internal::CASDeserializer::deserializeResultSpecification(rsTypes, rsFeatures, pInstance->getCASImpl().getCASDefinition(), rs);

//      cerr << __FILE__ << __LINE__ << ": ";
//      rs.print(cerr);

    UIMA_TPRINT("CAS BEFORE process: ");
#ifdef DEBUG_VERBOSE
#ifndef NDEBUG
    ///uima::internal::CASImpl const & crCASImpl = uima::internal::CASImpl::promoteCAS( *pInstance->getCAS() );
    ///crCASImpl.getHeap().print(cerr);
#endif
#endif
    //uima::TyErrorId tyErrorId;
    uima::CAS* tcas;
    if (isTCas) {
      JNIUString usSofaName(jeEnv, sofaName);
      UIMA_TPRINT("TCAS BEFORE process: " );
      UIMA_TPRINT(sofaName );
      uima::SofaID sofaid;
      uima::UnicodeStringRef ref = usSofaName.toUStrPtrLenPair();
      icu::UnicodeString usofa = icu::UnicodeString( ref.getBuffer(), ref.length() );
      sofaid.setSofaId( UnicodeString(usofa) );
      sofaid.setComponentSofaName(usofa);
      tcas = cas.getView(cas.getSofa(sofaid));
      ///tyErrorId = ((uima::AnalysisEngine*) pEngine)->process(*tcas);
      uima::CASIterator iter  = ((uima::AnalysisEngine*) pEngine)->processAndOutputNewCASes(*tcas);
    } else {
      /////tyErrorId = ((uima::AnalysisEngine*) pEngine)->process(cas);
      UIMA_TPRINT("CAS BEFORE process: " );
      uima::CASIterator iter = ((uima::AnalysisEngine*) pEngine)->processAndOutputNewCASes(cas);
    }
    /****
    if (tyErrorId != UIMA_ERR_NONE) {
       UIMA_TPRINT("Error during processDocument() (code= " << tyErrorId << ")" );
       uima::ErrorInfo const & lastError = pEngine->getAnnotatorContext(). getLogger().getLastError();
       if ( lastError.getErrorId() == UIMA_ERR_NONE ) {
          uima::ErrorInfo newError;
          newError.setErrorId(tyErrorId);
          JNIUtils::throwNewInternalException(jeEnv, newError);
       }
       else {
          assert(lastError.getErrorId() == tyErrorId);
          JNIUtils::throwNewInternalException(jeEnv, lastError);
       }
       return;
    } ****/

    UIMA_TPRINT("CAS AFTER process: ");
#ifdef DEBUG_VERBOSE
#ifndef NDEBUG
    crCASImpl.getHeap().print(cerr);
#endif
#endif


    UIMA_TPRINT("exiting processDocument");
  } catch (uima::Exception & rException) {
    cout << "processJNI: Exception " << rException.asString() << endl;
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return;
  }
#ifdef NDEBUG
//#ifdef TEMP_DISABLED
  catch (...) {
    //cout << "processJNI: all other execptions" << endl;
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION(jeEnv);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return;
  }
#endif
}


/***********************************************************************
 * delete the current document.
 */
JNIEXPORT void JNICALL JAVA_PREFIX(resetJNI) (JNIEnv* jeEnv, jobject joJTaf) {
  try {
    UIMA_TPRINT("entering resetDocument()");

    uima::JNIInstance* pInstance = JNIUtils::getCppInstance(jeEnv, joJTaf);
    assert( EXISTS(pInstance) );
    pInstance->getSerializedCAS().reset();

    uima::TyErrorId tyErrorId = pInstance->getCAS()->reset();

    //TODO: handle CAS multiplier - return new CASs to pool
    //     end the segementation.

    if (tyErrorId != UIMA_ERR_NONE) {
      UIMA_TPRINT("Error during resetDocument() (code= " << tyErrorId << ")" );
      uima::ErrorInfo errInfo(UIMA_MSG_ID_NO_MESSAGE_AVAILABLE,
                              tyErrorId,
                              uima::ErrorInfo::unrecoverable);
      JNIUtils::throwNewInternalException(jeEnv, errInfo);
      return;
    }
    UIMA_TPRINT("exiting resetDocument()");
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION(jeEnv);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return;
  }
}


/***********************************************************************
 */
JNIEXPORT void JNICALL JAVA_PREFIX(serializeCASJNI) (JNIEnv* jeEnv, jobject joJTaf, jboolean jbSerializeData) {
  try {
    UIMA_TPRINT("entering serializeCASJNI()");

    uima::JNIInstance* pInstance = JNIUtils::getCppInstance(jeEnv, joJTaf);
    assert( EXISTS(pInstance) );

    uima::internal::CASSerializer serializer(false);

    // definitions should not be called from here any more (see initializeJNI() )
    assert( jbSerializeData == JNI_TRUE );
    serializer.serializeData( pInstance->getCASImpl(), pInstance->getSerializedCAS() );

#ifdef UIMA_ENABLE_SERIALIZATION_TIMING
    cout << __FILE__ << __LINE__ << ": CPP/JEDII JNI layer - Time for serialization of heap: " << serializer.getHeapTimer().timeString() << endl;
    cout << __FILE__ << __LINE__ << ": CPP/JEDII JNI layer - Time for serialization of indexed FSs: " << serializer.getIndexTimer().timeString() << endl;
#endif
    UIMA_TPRINT("serializeCASJNI() finished");
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION(jeEnv);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return;
  }
}

/***********************************************************************
 */
JNIEXPORT void JNICALL JAVA_PREFIX(serializeSegmentJNI) (JNIEnv* jeEnv, jobject joJTaf, jboolean jbSerializeData) {
  try {
    UIMA_TPRINT("entering serializeSegmentJNI()");

    uima::JNIInstance* pInstance = JNIUtils::getCppInstance(jeEnv, joJTaf);
    assert( EXISTS(pInstance) );

    uima::internal::CASSerializer serializer(false);

    // definitions should not be called from here any more (see initializeJNI() )
    assert( jbSerializeData == JNI_TRUE );
    uima::internal::CASImpl & casimpl = uima::internal::CASImpl::promoteCAS( *(pInstance->getSegment()) );
    UIMA_TPRINT(casimpl.getDocumentText());
    serializer.serializeData( casimpl, pInstance->getSerializedSegment() );

#ifdef UIMA_ENABLE_SERIALIZATION_TIMING
    cout << __FILE__ << __LINE__ << ": TAF/JEDII JNI layer - Time for serialization of heap: " << serializer.getHeapTimer().timeString() << endl;
    cout << __FILE__ << __LINE__ << ": TAF/JEDII JNI layer - Time for serialization of indexed FSs: " << serializer.getIndexTimer().timeString() << endl;
#endif
    UIMA_TPRINT("serializeSegmentJNI() finished");
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION(jeEnv);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return;
  }
}



class JavaIntegerObject : public JavaObject {
private:
  JavaIntegerObject(JavaIntegerObject const &);

public:
  JavaIntegerObject(JNIEnv * jeEnv, int iNum)
      : JavaObject(jeEnv, "java/lang/Integer", "(I)V") {
    assertWithMsg( (sizeof(jint) == sizeof(int)), "Port required");
    jvalue args[1];
    args[0].i = iNum;
    createJObject(args);
  }
};


JNIEXPORT jobject JNICALL JAVA_PREFIX(getSerializedDataJNI) (JNIEnv* jeEnv, jobject joJTaf, jint jiWhichData) {
  jobject joResult = NULL;
  try {
    UIMA_TPRINT("getSerializedDataJNI() entered");
    uima::JNIInstance* pInstance = JNIUtils::getCppInstance(jeEnv, joJTaf);
    assert( EXISTS(pInstance) );
    uima::internal::SerializedCAS & crSerializedCAS = pInstance->getSerializedCAS();
    joResult = getSerializedCasData(jeEnv, joJTaf, jiWhichData, crSerializedCAS);
    UIMA_TPRINT("getSerializedDataJNI() finished");
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return NULL;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION_WITH_RETURN_VALUE(jeEnv, NULL);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return NULL;
  }
  return joResult;
}

JNIEXPORT jobject JNICALL JAVA_PREFIX(getSerializedSegmentDataJNI) (JNIEnv* jeEnv, jobject joJTaf, jint jiWhichData) {
  jobject joResult = NULL;
  try {
    UIMA_TPRINT("getSerializedSegmentDataJNI() entered");
    uima::JNIInstance* pInstance = JNIUtils::getCppInstance(jeEnv, joJTaf);
    assert( EXISTS(pInstance) );
    uima::internal::SerializedCAS & crSerializedCAS = pInstance->getSerializedSegment();
    joResult = getSerializedCasData(jeEnv, joJTaf, jiWhichData, crSerializedCAS);
    UIMA_TPRINT("getSerializedDataJNI() finished");
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return NULL;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION_WITH_RETURN_VALUE(jeEnv, NULL);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return NULL;
  }
  UIMA_TPRINT("getSerializedSegmentDataJNI() done");
  return joResult;
}


jobject getSerializedCasData (JNIEnv* jeEnv, jobject joJTaf, jint jiWhichData, uima::internal::SerializedCAS & crSerializedCAS) {
  jobject joResult = NULL;
  try {
    UIMA_TPRINT("getSerializedDataJNI() entered");
    uima::JNIInstance* pInstance = JNIUtils::getCppInstance(jeEnv, joJTaf);
    assert( EXISTS(pInstance) );
    //////uima::internal::SerializedCAS const & crSerializedCAS = pInstance->getSerializedCAS();

    {

      // the C++ objects created for TOPTYPE and DOCUMENT are destroyed after the switch
      // but the created Java object inside is still alive

      switch (jiWhichData) {
      case CONST_PREFIX( TYPE_INH ):
              joResult = JNIUtils::createIntArray(jeEnv, crSerializedCAS.getTypeInheritanceTable());
        break;
      case CONST_PREFIX(FEATURE_DEF):
              joResult = JNIUtils::createIntArray(jeEnv, crSerializedCAS.getFeatureDefinitionTable());
        break;
      case CONST_PREFIX(FEATURE_OFFSET):
              joResult = JNIUtils::createIntArray(jeEnv, crSerializedCAS.getFeatureOffsetTable());
        break;
      case CONST_PREFIX(TYPE_SYMBOLS):
              joResult = JNIUtils::createStringArray(jeEnv, crSerializedCAS.getTypeSymbolTable());
        break;
      case CONST_PREFIX(FEATURE_SYMBOLS):
              joResult = JNIUtils::createStringArray(jeEnv, crSerializedCAS.getFeatureSymbolTable());
        break;
      case CONST_PREFIX( TYPE_PRIORITIES):
              joResult = JNIUtils::createIntArray(jeEnv, crSerializedCAS.getTypePriorityTable() );
        break;
      case CONST_PREFIX(TOPTYPE): {
        assert( sizeof(uima::lowlevel::TyFSType) == sizeof(jint) );
        uima::lowlevel::TyFSType tyTop = pInstance->getCASImpl().getHeap().getTypeSystem().getTopType();
        assert(pInstance->getCASImpl().getHeap().getTypeSystem().isValidType(tyTop));
        JavaIntegerObject javaInt(jeEnv, (int) tyTop );
        joResult = javaInt.getJObject();
      }
      break;
      case CONST_PREFIX(FSHEAP):
              joResult = JNIUtils::createIntArray(jeEnv, crSerializedCAS.getFSHeapArray());
        break;
      case CONST_PREFIX(BYTEHEAP):
              joResult = JNIUtils::createByteArray(jeEnv, crSerializedCAS.getByteHeapArray());
        break;
      case CONST_PREFIX(SHORTHEAP):
              joResult = JNIUtils::createShortArray(jeEnv, crSerializedCAS.getShortHeapArray());
        break;
      case CONST_PREFIX(LONGHEAP):
              joResult = JNIUtils::createLongArray(jeEnv, crSerializedCAS.getLongHeapArray());
        break;
      case CONST_PREFIX(STRINGSYMBOL):
              joResult = JNIUtils::createStringArray(jeEnv, crSerializedCAS.getStringSymbolTable());
        break;
      case CONST_PREFIX(DOCUMENT): {
//               uima::UnicodeStringRef ustrp = pInstance->getCAS()->getDocumentText();
//            UIMA_TPRINT("Doc text: " << ustrp);
//               JavaString javaStr(jeEnv, ustrp);
//               joResult = javaStr.getJObject();
        break;
      }
      case CONST_PREFIX(INDEXEDFSS):
              joResult = JNIUtils::createIntArray(jeEnv, crSerializedCAS.getIndexedFSs());
        break;
      case CONST_PREFIX(INDEXID):
              joResult = JNIUtils::createStringArray(jeEnv, crSerializedCAS.getIndexIDTable() );
        break;
      case CONST_PREFIX(INDEXKIND):
              joResult = JNIUtils::createIntArray(jeEnv, crSerializedCAS.getIndexKindTable() );
        break;
      case CONST_PREFIX(COMPARATORDEF):
              joResult = JNIUtils::createIntArray(jeEnv, crSerializedCAS.getComparatorDefinitionTable() );
        break;
      case CONST_PREFIX(COMPARATORSTART):
              joResult = JNIUtils::createIntArray(jeEnv, crSerializedCAS.getComparatorStartTable() );
        break;
      default:
        assert(false);
      }
    }
    UIMA_TPRINT("getSerializedDataJNI() finished");
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return NULL;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION_WITH_RETURN_VALUE(jeEnv, NULL);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return NULL;
  }
  return joResult;
}


/*********************************************************************
 * static method to get the error message from an errorId.
 */
JNIEXPORT jstring JNICALL JAVA_PREFIX(getErrorMessageJNI) (JNIEnv* jeEnv,
    jclass,
    jlong jlErrorId) {
  try {
    const char* cpszErrorId = (const char*) uima::AnalysisEngine::getErrorIdAsCString( (long) jlErrorId);
    jstring jsError = jeEnv->NewStringUTF(cpszErrorId);
    ASSERT_NO_JNI_EXCEPTION(jeEnv);
    return jsError;
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return NULL;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION_WITH_RETURN_VALUE(jeEnv, NULL);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return NULL;
  }
}

/*********************************************************************
 * batchProcessComplete
 */
JNIEXPORT void JNICALL JAVA_PREFIX(batchProcessCompleteJNI) (JNIEnv* jeEnv, jobject joJTaf) {
  try {
    UIMA_TPRINT("entering batchProcessCompleteJNI()");

    uima::JNIInstance* pInstance = JNIUtils::getCppInstance(jeEnv, joJTaf);
    assert( EXISTS(pInstance) );

    uima::AnalysisEngine * pEngine =  pInstance->getEngine();

    uima::TyErrorId tyErrorId = pEngine->batchProcessComplete();
    if (tyErrorId != UIMA_ERR_NONE) {
      UIMA_TPRINT("Error during deInitializing engine (code= " << tyErrorId << ")" );
      JNIUtils::throwNewInternalException(jeEnv, pEngine->getAnnotatorContext().getLogger().getLastError() );
      return;
    }

    UIMA_TPRINT("exiting batchProcessComplete()");
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION(jeEnv);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return;
  }
}

/*********************************************************************
 * collectionProcessComplete
 */
JNIEXPORT void JNICALL JAVA_PREFIX(collectionProcessCompleteJNI) (JNIEnv* jeEnv, jobject joJTaf) {
  try {
    UIMA_TPRINT("entering collectionProcessCompleteJNI()");

    uima::JNIInstance* pInstance = JNIUtils::getCppInstance(jeEnv, joJTaf);
    assert( EXISTS(pInstance) );

    uima::AnalysisEngine * pEngine =  pInstance->getEngine();

    uima::TyErrorId tyErrorId = pEngine->collectionProcessComplete();
    if (tyErrorId != UIMA_ERR_NONE) {
      UIMA_TPRINT("Error during deInitializing engine (code= " << tyErrorId << ")" );
      JNIUtils::throwNewInternalException(jeEnv, pEngine->getAnnotatorContext().getLogger().getLastError() );
      return;
    }

    UIMA_TPRINT("exiting collectionProcessComplete()");
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION(jeEnv);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return;
  }
}


/*********************************************************************
 * hasNextSegmentJNI
 */
JNIEXPORT jboolean JNICALL JAVA_PREFIX(hasNextSegmentJNI) (JNIEnv* jeEnv, jobject joJTaf) {
  try {
    UIMA_TPRINT("entering hasNextSegmentJNI()");

    uima::JNIInstance* pInstance = JNIUtils::getCppInstance(jeEnv, joJTaf);
    assert( EXISTS(pInstance) );

    jboolean hasNext = pInstance->hasNext();

    UIMA_TPRINT("exiting hasNextSegmentJNI()");

    return hasNext;

  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return false;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION_WITH_RETURN_VALUE(jeEnv, false);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return false;
  }
  return false;
}


/*********************************************************************
 * nextSegmentJNI
 */
JNIEXPORT void JNICALL JAVA_PREFIX(nextSegmentJNI) (JNIEnv* jeEnv, jobject joJTaf) {
  try {
    UIMA_TPRINT("entering nextSegmentJNI()");

    uima::JNIInstance* pInstance = JNIUtils::getCppInstance(jeEnv, joJTaf);
    assert( EXISTS(pInstance) );

    //release the previous segment if necessary
    UIMA_TPRINT("nextSegmentJNI() release previous segment ");
    pInstance->releaseSegment();

    pInstance->next();
    ////pInstance->setSegment(pInstance->getCASIterator().next());

    if (pInstance->getSegment() == NULL) {
      UIMA_TPRINT("Error during engine.next() " );
      JNIUtils::throwNewInternalException(jeEnv,  pInstance->getEngine()->getAnnotatorContext().getLogger().getLastError() );
      return;
    }

    UIMA_TPRINT("exiting nextSegmentJNI()");
    return;
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION(jeEnv);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return;
  }
}


/*********************************************************************
 * releaseSegmentJNI
 */
JNIEXPORT void JNICALL JAVA_PREFIX(releaseSegmentJNI) (JNIEnv* jeEnv, jobject joJTaf) {
  try {
    UIMA_TPRINT("entering releaseSegmentJNI()");

    uima::JNIInstance* pInstance = JNIUtils::getCppInstance(jeEnv, joJTaf);
    assert( EXISTS(pInstance) );

    uima::AnalysisEngine * pEngine =  pInstance->getEngine();

    //release cas.
    pInstance->releaseSegment();

    UIMA_TPRINT("exiting releaseSegmentJNI()");
    return;
  } catch (uima::Exception & rException) {
    UIMA_TPRINT("Exception: " << rException.asString() );
    JNIUtils::throwNewInternalException(jeEnv, rException);
    return;
  } catch (...) {
    UIMA_TPRINT("Unknown Exception" );
    CHECK_FOR_JNI_EXCEPTION(jeEnv);
    uima::ErrorInfo errInfo(UIMA_MSG_ID_EXC_UNEXPECTED_ERROR, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION, uima::ErrorInfo::unrecoverable);
    JNIUtils::throwNewInternalException(jeEnv, errInfo);
    return;
  }
}










