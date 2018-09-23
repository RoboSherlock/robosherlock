/** \file internal_jedii_engine.cpp .
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

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */

//#define DEBUG_VERBOSE
/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp>

#include <uima/trace.hpp>
#include <uima/envvar.hpp>
#include <uima/macros.h>
#include <uima/macros.h>

#include <uima/stltools.hpp>
#include <uima/msg.h>
#include <uima/internal_jedii_engine.hpp>
#include <uima/internal_casserializer.hpp>
#include <uima/internal_casdeserializer.hpp>
#include <uima/annotator_context.hpp>
#include <uima/resmgr.hpp>
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

/*
There's a lot of JNI code in here. If you are interested in the details you should
consult some book or go to
http://java.sun.com/j2se/1.3/docs/guide/jni/spec/jniTOC.doc.html
*/
using namespace std;
namespace uima {
  namespace internal {

    UIMA_EXC_CLASSIMPLEMENT(JavaException, Exception);

#define USED_JAVA_VERSION JNI_VERSION_1_2

    // not NLS enabled !!!!!!!
    icu::UnicodeString javaErrorToString(jint res) {
      switch (res) {
      case JNI_OK:
        return "no error";
      case JNI_ERR:
        return "unknown error";
      case JNI_EDETACHED:
        return "thread detached from the VM";
      case JNI_EVERSION:
        return "JNI version error";
      case JNI_ENOMEM:
        return "not enough memory";
      case JNI_EEXIST:
        return "VM already created";
      case JNI_EINVAL:
        return "invalid arguments";
      default:
        char buf[2056];
        sprintf(buf, "%d", res);
        return icu::UnicodeString(buf);
      }
    }

    JEDIIEngine::JEDIIEngine(AnnotatorContext & anc, bool bOwnsANC,
                             bool bOwnsTAESpecififer, uima::internal::CASDefinition & casdef, bool ownsCasDef)
        : EngineBase(anc, bOwnsANC, bOwnsTAESpecififer, casdef, ownsCasDef),
        iv_vm(NULL),
        iv_env(NULL),
        iv_obj(NULL),
        iv_createdVM(false),
        iv_initializeMethod(NULL),
        iv_processMethod(NULL),
        iv_destroyMethod(NULL),
        iv_getLastExceptionStringMethod(NULL),
        iv_getHeapMethod(NULL),
        iv_getIndexedFSsMethod(NULL),
        iv_getStringTableMethod(NULL),
        iv_getVersionMethod(NULL),
        iv_isPrimitive(0),
        iv_JVMDllLoader(NULL),
        iv_createJavaVMFunc(NULL),
        iv_getCreatedJavaVMsFunc(NULL) {
    }


    JEDIIEngine::~JEDIIEngine() {
      destroyIfNeeded();
    }


    void JEDIIEngine::checkForUnexpectedJNIException() {
      jthrowable exc = iv_env->ExceptionOccurred();
      if (exc != NULL) {
#ifndef NDEBUG
        iv_env->ExceptionDescribe();
#endif
        // if exception was thrown, get class name of exception plus the message
        // and throw a C++ exception

        // don't check for JNI exceptions in these calls because exception is already thrown
        jclass throwableClass = iv_env->FindClass("java/lang/Throwable");
        jmethodID getClassID = iv_env->GetMethodID(throwableClass, "getClass", "()Ljava/lang/Class;");
        assert( getClassID != NULL);
        jclass classClass = iv_env->FindClass("java/lang/Class");
        assert( classClass != NULL);
        jmethodID getNameID = iv_env->GetMethodID(classClass, "getName", "()Ljava/lang/String;");
        assert( getNameID != NULL );
        jobject classObject = iv_env->CallObjectMethod(exc, getClassID);
        assert ( classObject != NULL );
        jstring msg = (jstring) iv_env->CallObjectMethod(classObject, getNameID);
        jboolean copied;
        jchar const * jc = iv_env->GetStringChars(msg, &copied);
        assert( jc != NULL );
        size_t len = iv_env->GetStringLength(msg);
        icu::UnicodeString usMsg((UChar const *)jc, len);
        iv_env->ReleaseStringChars(msg, jc);
        // msg now contains the class name

        // add getMessage() of the throwable
        assert( throwableClass != NULL );
        jmethodID getMessageID = iv_env->GetMethodID(throwableClass, "getMessage", "()Ljava/lang/String;");
        assert ( getMessageID != NULL );
        msg = (jstring) iv_env->CallObjectMethod(exc, getMessageID);
        if ( msg != NULL ) {
          usMsg.append(": ");
          jc = iv_env->GetStringChars(msg, &copied);
          len = iv_env->GetStringLength(msg);
          usMsg.append( icu::UnicodeString((UChar const *)jc, len) );
          iv_env->ReleaseStringChars(msg, jc);
        }

        UIMA_EXC_THROW_NEW(JavaException,
                           UIMA_ERR_JAVA_EXCEPTION,
                           ErrorMessage(UIMA_MSG_ID_EXC_UNKNOWN_JAVA_EXCEPTION, usMsg),
                           UIMA_MSG_ID_EXCON_PROCESSING_JAVA_TAE,
                           ErrorInfo::unrecoverable);
      }
    }


    // create a new Java int[] from a C++ vector
    jintArray JEDIIEngine::createJIntArray(vector<internal::SerializedCAS::TyNum> const & v) {
      jintArray result = iv_env->NewIntArray(v.size());
      checkForUnexpectedJNIException();
      size_t i;
      for (i=0; i<v.size(); ++i) {
        jint n = v[i];
        iv_env->SetIntArrayRegion(result, i,1, &n );
        checkForUnexpectedJNIException();
#ifndef NDEBUG
        jint m;
        iv_env->GetIntArrayRegion(result, i, 1, &m );
        checkForUnexpectedJNIException();
        assert( n == m );
#endif
      }
      return result;
    }

    // create a new Java byte[] from a C++ vector
    jbyteArray JEDIIEngine::createJByteArray(vector<char> const & v) {
      jbyteArray result = iv_env->NewByteArray(v.size());
      checkForUnexpectedJNIException();
      size_t i;
      for (i=0; i<v.size(); ++i) {
        jbyte n = v[i];
        iv_env->SetByteArrayRegion(result, i,1, &n );
        checkForUnexpectedJNIException();
#ifndef NDEBUG
        jbyte m;
        iv_env->GetByteArrayRegion(result, i, 1, &m );
        checkForUnexpectedJNIException();
        assert( n == m );
#endif
      }
      return result;
    }

    // create a new Java short[] from a C++ vector
    jshortArray JEDIIEngine::createJShortArray(vector<short> const & v) {
      jshortArray result = iv_env->NewShortArray(v.size());
      checkForUnexpectedJNIException();
      size_t i;
      for (i=0; i<v.size(); ++i) {
        jshort n = v[i];
        iv_env->SetShortArrayRegion(result, i,1, &n );
        checkForUnexpectedJNIException();
#ifndef NDEBUG
        jshort m;
        iv_env->GetShortArrayRegion(result, i, 1, &m );
        checkForUnexpectedJNIException();
        assert( n == m );
#endif
      }
      return result;
    }

    // create a new Java long[] from a C++ vector
    jlongArray JEDIIEngine::createJLongArray(vector<INT64> const & v) {
      jlongArray result = iv_env->NewLongArray(v.size());
      checkForUnexpectedJNIException();
      size_t i;
      for (i=0; i<v.size(); ++i) {
        jlong n = v[i];
        iv_env->SetLongArrayRegion(result, i,1, &n );
        checkForUnexpectedJNIException();
#ifndef NDEBUG
        jlong m;
        iv_env->GetLongArrayRegion(result, i, 1, &m );
        checkForUnexpectedJNIException();
        assert( n == m );
#endif
      }
      return result;
    }



    // create a new Object[] (which all contains java.lang.Strings) from a C++ vector
    jobjectArray JEDIIEngine::createJStringArray(vector<UnicodeStringRef> const & v) {
      jclass stringClass = iv_env->FindClass("java/lang/String");
      checkForUnexpectedJNIException();
      jobjectArray result = iv_env->NewObjectArray(v.size(), stringClass, NULL);
      checkForUnexpectedJNIException();
      size_t i;
      for (i=0; i<v.size(); ++i) {
        assertWithMsg(sizeof(UChar) == sizeof(jchar), "Port required");
        jstring str = iv_env->NewString( (const jchar*)v[i].getBuffer(), v[i].length() );
        checkForUnexpectedJNIException();
        iv_env->SetObjectArrayElement(result, i, str);
        checkForUnexpectedJNIException();
      }
      return result;
    }

    // create a java.lang.String from a C++ unicode string
    jstring JEDIIEngine::createJString(UnicodeStringRef const & ref) {
      assertWithMsg(sizeof(UChar) == sizeof(jchar), "Port required");
      jstring result = iv_env->NewString((jchar const *) ref.getBuffer(), ref.length());
      checkForUnexpectedJNIException();
      return result;
    }

    // create a C++ vector from a Java int[]
    void JEDIIEngine::createIntVector(jintArray ar, vector<internal::SerializedCAS::TyNum>& result) {
      result.clear();
      jsize len = iv_env->GetArrayLength(ar);
      checkForUnexpectedJNIException();
      jint* tempAr = new jint[len];
      iv_env->GetIntArrayRegion(ar, 0, len, tempAr);
      checkForUnexpectedJNIException();

      result.reserve(len);
      int i;
      for (i=0; i<len; ++i) {
        result.push_back(tempAr[i]);
      }

      delete[] tempAr;
    }

    // create a C++ string vector from a java.lang.String[]
    void JEDIIEngine::createStringVector(jobjectArray ar, vector<UnicodeStringRef>& result, vector<icu::UnicodeString> & pool) {
      pool.clear();
      result.clear();
      jsize len = iv_env->GetArrayLength(ar);
      checkForUnexpectedJNIException();
      result.reserve(len);
      pool.reserve(len); // with this reserve, the vector will never be re-allocated, thus the internal pointer in the UnicodeString objects remain valid throughout this method!
      int i;
      for (i=0; i<len; ++i) {
        jstring js = (jstring) iv_env->GetObjectArrayElement(ar, i);
        checkForUnexpectedJNIException();
        if (js != NULL) {
          jboolean isCopy;
          jchar const * jc = iv_env->GetStringChars(js, &isCopy);
          checkForUnexpectedJNIException();
          jsize strLen = iv_env->GetStringLength(js);
          checkForUnexpectedJNIException();
          icu::UnicodeString  us((UChar const *) jc, (size_t) strLen);
          iv_env->ReleaseStringChars(js, jc);
          checkForUnexpectedJNIException();
          pool.push_back(us);

          icu::UnicodeString const & poolString = pool.back();
          UnicodeStringRef uref( poolString.getBuffer(), poolString.length() );
          result.push_back(uref);
        } else {
          UnicodeStringRef emptyURef;
          result.push_back(emptyURef);
        }
        iv_env->DeleteLocalRef(js);
      }
    }

    // create a C++ vector from a Java byte[]
    void JEDIIEngine::createByteVector(jbyteArray ar, vector<char>& result) {
      result.clear();
      jsize len = iv_env->GetArrayLength(ar);
      checkForUnexpectedJNIException();
      jbyte* tempAr = new jbyte[len];
      iv_env->GetByteArrayRegion(ar, 0, len, tempAr);
      checkForUnexpectedJNIException();

      result.reserve(len);
      int i;
      for (i=0; i<len; ++i) {
        result.push_back(tempAr[i]);
      }

      delete[] tempAr;
    }


    // create a C++ vector from a Java short[]
    void JEDIIEngine::createShortVector(jshortArray ar, vector<short>& result) {
      result.clear();
      jsize len = iv_env->GetArrayLength(ar);
      checkForUnexpectedJNIException();
      jshort* tempAr = new jshort[len];
      iv_env->GetShortArrayRegion(ar, 0, len, tempAr);
      checkForUnexpectedJNIException();

      result.reserve(len);
      int i;
      for (i=0; i<len; ++i) {
        result.push_back(tempAr[i]);
      }

      delete[] tempAr;
    }

    // create a C++ vector from a Java short[]
    void JEDIIEngine::createLongVector(jlongArray ar, vector<long>& result) {
      result.clear();
      jsize len = iv_env->GetArrayLength(ar);
      checkForUnexpectedJNIException();
      jlong* tempAr = new jlong[len];
      iv_env->GetLongArrayRegion(ar, 0, len, tempAr);
      checkForUnexpectedJNIException();

      result.reserve(len);
      int i;
      for (i=0; i<len; ++i) {
        result.push_back(tempAr[i]);
      }

      delete[] tempAr;
    }

    template <class T>
    void printVector(ostream& os, char const * c, vector<T> const & v) {
      os << __FILE__ << __LINE__ << ": " << c << endl;
      size_t i;
      for (i=0; i<v.size(); ++i) {
        os << v[i] << " ";
      }
      os << endl;
    }


    bool JEDIIEngine::isPrimitive() const {
      assert( iv_isPrimitive == 1 || iv_isPrimitive == 2);
      if (iv_isPrimitive == 1) {
        return true;
      }
      return false;
    }


    void JEDIIEngine::handleEngineProxyError(jint rc) {
      if (rc != JNI_OK) {
        // some error occurred in the JEDIIEngine object
        jobject excStringObj = iv_env->CallObjectMethod(iv_obj, iv_getLastExceptionStringMethod);
        checkForUnexpectedJNIException();
        jstring excString = (jstring) excStringObj;
        jboolean isCopy;
        jchar const * jc = iv_env->GetStringChars(excString, &isCopy);
        checkForUnexpectedJNIException();
        jsize js = iv_env->GetStringLength(excString);
        checkForUnexpectedJNIException();
        icu::UnicodeString  us((UChar const *) jc, (size_t) js);
        iv_env->ReleaseStringChars(excString, jc);
        checkForUnexpectedJNIException();

        handleJNIError(rc, us);
      }
    }


    void JEDIIEngine::handleJNIError(jint rc, icu::UnicodeString const & us) {
      if (rc != JNI_OK) {
        ErrorMessage msg(UIMA_MSG_ID_EXC_JAVA_EXCEPTION);
        msg.addParam( javaErrorToString(rc) );
        msg.addParam( us);
        UIMA_EXC_THROW_NEW(JavaException,
                           UIMA_ERR_JAVA_EXCEPTION,
                           msg,
                           UIMA_MSG_ID_EXCON_PROCESSING_JAVA_TAE,
                           ErrorInfo::unrecoverable);
      }
    }


    void JEDIIEngine::handleJNIError(jint rc) {
      if (rc != JNI_OK) {
        ErrorMessage msg(UIMA_MSG_ID_EXC_JNI_CALL_FAILED);
        msg.addParam( javaErrorToString(rc) );
        UIMA_EXC_THROW_NEW(JavaException,
                           UIMA_ERR_JAVA_EXCEPTION,
                           msg,
                           UIMA_MSG_ID_EXCON_PROCESSING_JAVA_TAE,
                           ErrorInfo::unrecoverable);
      }
    }

    void JEDIIEngine::initializeJVMFunctions() {
      /*
          If we linked against the jvm DLL, the following three lines would do the trick:
               iv_createJavaVMFunc = (TyCreateJavaVMFunc) JNI_CreateJavaVM;
               iv_getCreatedJavaVMsFunc = (TyGetCreatedJavaVMsFunc) JNI_GetCreatedJavaVMs;
               return ;
      */
      // names of the JVM library on the different platforms
      // (APR should add the appropriate suffix)
#if defined(WIN32)
#define JVMDLL_FILE_NAME "jvm"
#else
#define JVMDLL_FILE_NAME "libjvm"
#endif

      // dynamically load jvm DLL
      util::Filename jvmDll(JVMDLL_FILE_NAME);

      assert( iv_JVMDllLoader == NULL );
      iv_JVMDllLoader = new util::DllProcLoaderFile(jvmDll);
      assert( EXISTS(iv_JVMDllLoader) );

      if (!iv_JVMDllLoader->isValid()) {
        delete iv_JVMDllLoader;
        iv_JVMDllLoader = NULL;
        UIMA_EXC_THROW_NEW(JavaException,
                           UIMA_ERR_COULD_NOT_LOAD_JAVA_DLL,
                           UIMA_MSG_ID_EXC_COULD_NOT_LOAD_JAVA_DLL,
                           UIMA_MSG_ID_EXCON_PROCESSING_JAVA_TAE,
                           ErrorInfo::unrecoverable);
      } else {
        // get function pointers
        iv_createJavaVMFunc = (TyCreateJavaVMFunc) iv_JVMDllLoader->getProcedure( "JNI_CreateJavaVM");
        if ( iv_createJavaVMFunc == NULL ) {
          UIMA_EXC_THROW_NEW(JavaException,
                             UIMA_ERR_COULD_NOT_LOAD_JAVA_DLL,
                             UIMA_MSG_ID_EXC_COULD_NOT_LOAD_JAVA_DLL,
                             UIMA_MSG_ID_EXCON_PROCESSING_JAVA_TAE,
                             ErrorInfo::unrecoverable);

        }
        iv_getCreatedJavaVMsFunc = (TyGetCreatedJavaVMsFunc) iv_JVMDllLoader->getProcedure("JNI_GetCreatedJavaVMs");
        if ( iv_getCreatedJavaVMsFunc == NULL ) {
          UIMA_EXC_THROW_NEW(JavaException,
                             UIMA_ERR_COULD_NOT_LOAD_JAVA_DLL,
                             UIMA_MSG_ID_EXC_COULD_NOT_LOAD_JAVA_DLL,
                             UIMA_MSG_ID_EXCON_PROCESSING_JAVA_TAE,
                             ErrorInfo::unrecoverable);
        }

      }

    }

    void JEDIIEngine::destroyJVMFunctions() {
      if (iv_JVMDllLoader != NULL) {
        assert( EXISTS(iv_JVMDllLoader) );
        delete iv_JVMDllLoader;
        iv_createJavaVMFunc = NULL;
        iv_getCreatedJavaVMsFunc = NULL;
      }
    }


    void JEDIIEngine::initializeJavaVM() {
      util::Trace clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);

      JavaVM* vms[1];
      vms[0] = NULL;
      jsize existingVMs = 0;
      iv_createdVM = false;

      UIMA_TPRINT("check if VM exists");
      assert( EXISTS(iv_getCreatedJavaVMsFunc) );
      jint existingVMsRes = (*iv_getCreatedJavaVMsFunc)(vms, 1, & existingVMs);
      UIMA_TPRINT("JNI return code of getCreatedVMs: " << existingVMsRes);
      if (existingVMsRes != JNI_OK) {
        handleJNIError(existingVMsRes);
      }
      UIMA_TPRINT("existing VMs: " << existingVMs);

      // if we don't already run within a Java context
      if (existingVMs == 0) {
        UIMA_TPRINT("new VM must be created");
        iv_createdVM = true;
        // get classpath from environment
        util::EnvironmentVariableQueryOnly envVar("CLASSPATH");
        char const * classpath = envVar.getValue();
        char const * cpOption = "-Djava.class.path=";
        char const * emptyString = "";
        if (classpath == NULL) {
          classpath = emptyString;
        }

        size_t cpBufLen = strlen(classpath) + strlen(cpOption) + 1;
        auto_array<char> cpBuf( new char[cpBufLen] );
        cpBuf[0] = 0;
        strcat( strcat(cpBuf.get(), cpOption), classpath );

        // create and initialize Java VM
        char * optionStrings[] = {
                                   cpBuf.get(),
                                   "-Xmx256MB",
#ifndef NDEBUG
                                   "-Xcheck:jni",
                                   "-Xcheck:nabounds",
#endif
                                 };

        JavaVMInitArgs vmArgs;

        vmArgs.version = USED_JAVA_VERSION;

        JavaVMOption options[NUMBEROF(optionStrings)];
        size_t i;
        for (i=0; i<NUMBEROF(optionStrings); ++i) {
          options[i].optionString = optionStrings[i];
        }
        vmArgs.nOptions = NUMBEROF(optionStrings);
        vmArgs.options = options;

        vmArgs.ignoreUnrecognized = JNI_FALSE;

        UIMA_TPRINT("creating VM");
        assert( iv_createJavaVMFunc != NULL);
        jint vmCreationRes = (*iv_createJavaVMFunc)(& iv_vm, (void **)& iv_env, &vmArgs);
        UIMA_TPRINT("VM created");
        if (vmCreationRes != JNI_OK) {
          clTrace.dump("Java VM could not be created, return code:", vmCreationRes);
          iv_vm = NULL;
          iv_env = NULL;
          UIMA_EXC_THROW_NEW(JavaException,
                             UIMA_ERR_JAVA_EXCEPTION,
                             ErrorMessage(UIMA_MSG_ID_EXC_JAVA_VM_COULD_NOT_BE_CREATED, javaErrorToString(vmCreationRes)),
                             UIMA_MSG_ID_EXCON_PROCESSING_JAVA_TAE,
                             ErrorInfo::unrecoverable);
        }
        UIMA_TPRINT("VM created");
      } else {
        // re-use existing VM
        UIMA_TPRINT("Using existing VM: " << vms[0]);
        assert( existingVMsRes == JNI_OK );
        assert( existingVMs == 1);

        jint getenvRes = vms[0]->GetEnv((void**) & iv_env, USED_JAVA_VERSION);
        handleJNIError(getenvRes);
        assert( getenvRes == JNI_OK );
        iv_vm = vms[0];
      }

      assert( EXISTS(iv_env) );
      assert( EXISTS(iv_vm) );

    }


    TyErrorId JEDIIEngine::initializeImpl(AnalysisEngineDescription const & specifier) {
      util::Trace clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);

      UIMA_TPRINT("initializing JVM DLL");
      initializeJVMFunctions();
      UIMA_TPRINT("JVM DLL loaded");

      assert( iv_createJavaVMFunc != NULL );
      assert( iv_getCreatedJavaVMsFunc != NULL );

      if (specifier.isPrimitive()) {
        iv_isPrimitive = 1;
      } else {
        iv_isPrimitive = 2;
      }

      icu::UnicodeString config;
      specifier.toXMLBuffer(config);

//         ofstream ofs("jedii_tmp_specifier.xml"); ofs << config << endl; ofs.close();

      UIMA_TPRINT("Initializing Java VM");
      initializeJavaVM();
      UIMA_TPRINT("VM initialized");

      assert( EXISTS(iv_env) );
      assert( EXISTS(iv_vm) );

      TyErrorId err = UIMA_ERR_NONE;

      UIMA_TPRINT("setup VM");
      // create Java object and initialize method IDs
      assert( EXISTS(iv_env) );
      jclass clazz = iv_env->FindClass("org/apache/uima/uimacpp/CppUimajEngine");
      checkForUnexpectedJNIException();
      jmethodID constructor = iv_env->GetMethodID(clazz, "<init>", "()V");
      checkForUnexpectedJNIException();

      iv_initializeMethod = iv_env->GetMethodID(clazz, "initialize", "("
                            "Ljava/lang/String;"  // config,
                            "Ljava/lang/String;"  // datapath
                            "[I"                  // typeInheritance
                            "[I"                  // typePriorities
                            "[I"                  // featureDefs
                            "[I"                  // featureOffset
                            "[Ljava/lang/String;" // typeNames
                            "[Ljava/lang/String;" // featureNames
                            "[I"                  // stringSubTypes
                            "[Ljava/lang/String;" // stringSubTypeValues
                            "[I"                  // stringSubTypeValuePos
                            "[Ljava/lang/String;" // indexIDs
                            "[I"                  // indexKinds
                            "[I"                  // compStarts
                            "[I"                  // compDefs
                            ")I");
      checkForUnexpectedJNIException();

      //check if UIMA Framework version is compatible
      iv_getVersionMethod = iv_env->GetStaticMethodID(clazz, "getVersion", "()Ljava/lang/String;");
      checkForUnexpectedJNIException();

      jobject versionStringObj = iv_env->CallStaticObjectMethod(clazz, iv_getVersionMethod);
      checkForUnexpectedJNIException();
      jstring versionString = (jstring) versionStringObj;
      const TCHAR ** compatibleVersions = gs_compatibleUIMAVersions;
      if (versionString != NULL) {
        cout << versionString << endl;
        jboolean isCopy;
        jchar const * jc = iv_env->GetStringChars(versionString, &isCopy);
        checkForUnexpectedJNIException();
        jsize js = iv_env->GetStringLength(versionString);
        checkForUnexpectedJNIException();
        icu::UnicodeString  uVersion((UChar const *) jc, (size_t) js);
        cout << uVersion << endl;
        bool compatible = false;

        for (int i=0; i < NUM_COMPATIBLE_UIMA_JAVA_VERSIONS; i++) {
          if (uVersion.compare(compatibleVersions[i]) == 0) {
            compatible=true;
          }
        }

        if (!compatible) {
          ErrorMessage msg = ErrorMessage(UIMA_MSG_ID_INCOMPATIBLE_UIMA_JAVA_VERSION);
          msg.addParam(uVersion);
          for (int i=0; i < NUM_COMPATIBLE_UIMA_JAVA_VERSIONS; i++) {
            msg.addParam(compatibleVersions[i]);
          }
          UIMA_EXC_THROW_NEW(JavaException,
                             UIMA_ERR_JAVA_EXCEPTION,
                             msg,
                             UIMA_MSG_ID_INCOMPATIBLE_UIMA_JAVA_VERSION,
                             ErrorInfo::unrecoverable);
        }
        iv_env->ReleaseStringChars(versionString, jc);
        checkForUnexpectedJNIException();
      }  else {
        ErrorMessage msg = ErrorMessage(UIMA_MSG_ID_INCOMPATIBLE_UIMA_JAVA_VERSION);
        msg.addParam("could not determine UIMA Java version");
        UIMA_EXC_THROW_NEW(JavaException,
                           UIMA_ERR_JAVA_EXCEPTION,
                           msg,
                           UIMA_MSG_ID_INCOMPATIBLE_UIMA_JAVA_VERSION,
                           ErrorInfo::unrecoverable);
      }




      iv_processMethod = iv_env->GetMethodID(clazz, "process", "("
                                             "Ljava/lang/String;"  // doc
                                             "[I"                  // heap
                                             "[I"                  // fsIndex
                                             "[Ljava/lang/String;" // stringTable
                                             "[I"                  // resultSpecTypes
                                             "[I"                  // resultSpecFeatures
                                             "I"                   // sofanum
                                             "[B"         // byte heap
                                             "[S"      // short heap
                                             "[J"      // long heap
                                             ")I");
      checkForUnexpectedJNIException();
      iv_getByteHeapMethod = iv_env->GetMethodID(clazz, "getByteHeap", "()[B");
      checkForUnexpectedJNIException();
      iv_getShortHeapMethod = iv_env->GetMethodID(clazz, "getShortHeap", "()[S");
      checkForUnexpectedJNIException();
      iv_getLongHeapMethod = iv_env->GetMethodID(clazz, "getLongHeap", "()[J");
      checkForUnexpectedJNIException();






      iv_destroyMethod = iv_env->GetMethodID(clazz, "destroy", "()I");
      checkForUnexpectedJNIException();
      iv_getLastExceptionStringMethod = iv_env->GetMethodID(clazz, "getLastExceptionString", "()Ljava/lang/String;");
      checkForUnexpectedJNIException();
      iv_getHeapMethod = iv_env->GetMethodID(clazz, "getHeap", "()[I");
      checkForUnexpectedJNIException();
      iv_getIndexedFSsMethod = iv_env->GetMethodID(clazz, "getIndexedFSs", "()[I");
      checkForUnexpectedJNIException();
      iv_getStringTableMethod = iv_env->GetMethodID(clazz, "getStringTable", "()[Ljava/lang/String;");
      checkForUnexpectedJNIException();

      iv_batchProcessCompleteMethod = iv_env->GetMethodID(clazz, "batchProcessComplete", "()I");
      checkForUnexpectedJNIException();

      iv_collectionProcessCompleteMethod = iv_env->GetMethodID(clazz, "collectionProcessComplete", "()I");
      checkForUnexpectedJNIException();



      iv_obj = iv_env->NewObject(clazz, constructor);
      checkForUnexpectedJNIException();
      iv_obj = iv_env->NewGlobalRef(iv_obj);
      checkForUnexpectedJNIException();
      UIMA_TPRINT("setup VM complete");


      // serialize type system and pass it to the initialize method
      CASSerializer serializer(true);
      SerializedCAS serializedCAS;
      serializer.serializeDefinitions( *iv_casDefinition, serializedCAS);
      UIMA_TPRINT("definitions serialized");
      {
        jint pushFrameResult = iv_env->PushLocalFrame(serializedCAS.getNumberOfJavaObjectsToBeCreatedDefinitions() );
        checkForUnexpectedJNIException();
        handleEngineProxyError(pushFrameResult);
        checkForUnexpectedJNIException();

        // prepare arguments for initialize()
        jintArray typeInheritance = createJIntArray(serializedCAS.getTypeInheritanceTable() );
        jintArray typePriorities = createJIntArray(serializedCAS.getTypePriorityTable());
        jintArray featureDefs = createJIntArray(serializedCAS.getFeatureDefinitionTable());
        jintArray featureOffset = createJIntArray(serializedCAS.getFeatureOffsetTable());

        jobjectArray typeNames = createJStringArray(serializedCAS.getTypeSymbolTable());
        jobjectArray featureNames = createJStringArray(serializedCAS.getFeatureSymbolTable());

        jintArray subStringTypes = createJIntArray(serializedCAS.getStringSubTypes());
        jobjectArray subStringTypeValues = createJStringArray(serializedCAS.getStringSubTypeValues());
        jintArray subStringTypeValuePos = createJIntArray(serializedCAS.getStringSubTypeValuePos());

        jintArray indexKinds = createJIntArray(serializedCAS.getIndexKindTable());
        jintArray compStarts = createJIntArray(serializedCAS.getComparatorStartTable());
        jintArray compDefs = createJIntArray(serializedCAS.getComparatorDefinitionTable());
        jobjectArray indexIDs = createJStringArray(serializedCAS.getIndexIDTable());

        jstring conf = iv_env->NewString((jchar const *)config.getBuffer(), config.length());
        checkForUnexpectedJNIException();

        icu::UnicodeString dataPathUS(uima::ResourceManager::getInstance().getLocationData().getAsCString() );
        jstring dataPath = iv_env->NewString((jchar const *)dataPathUS.getBuffer(), dataPathUS.length() );
        checkForUnexpectedJNIException();

        UIMA_TPRINT("Java args created");
        // call initialize on org.apache.uima.uimacpp.CppUimajEngine
        jint initResult = iv_env->CallIntMethod(iv_obj, iv_initializeMethod,
                                                conf,
                                                dataPath,

                                                typeInheritance,
                                                typePriorities,
                                                featureDefs,
                                                featureOffset,
                                                typeNames,
                                                featureNames,

                                                subStringTypes,
                                                subStringTypeValues,
                                                subStringTypeValuePos,

                                                indexIDs,
                                                indexKinds,
                                                compStarts,
                                                compDefs);
        checkForUnexpectedJNIException();
        handleEngineProxyError(initResult);
        UIMA_TPRINT("Java initalize called");

        iv_env->PopLocalFrame(NULL);
        checkForUnexpectedJNIException();
      }

      UIMA_TPRINT("result spec initialized");
      return err;
    }


    TyErrorId JEDIIEngine::reinitTypeSystemImpl() {
      return initializeImpl(getAnnotatorContext().getTaeSpecifier());
    }


    TyErrorId JEDIIEngine::processImpl(CAS & cas, ResultSpecification const & resultSpec) {
      util::Trace             clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);

      UIMA_TPRINT("serializing data");
      TyErrorId err = UIMA_ERR_NONE;

      // serialize CAS and pass it to process method
      CASSerializer serializer(true);
      SerializedCAS serializedInCAS;
//         serializer.serializeData( TCAS::promoteCAS(cas), serializedInCAS);
      serializer.serializeData(cas, serializedInCAS);
      UIMA_TPRINT("data serialized");
      {
        jint pushFrameRes = iv_env->PushLocalFrame( serializedInCAS.getNumberOfJavaObjectsToBeCreatedData());
        checkForUnexpectedJNIException();
        handleJNIError(pushFrameRes);
        // prepare arguments
        jstring doc = createJString( serializedInCAS.getDocument() );
        jintArray inHeap = createJIntArray(serializedInCAS.getFSHeapArray() );
        jintArray inIndexedFSs = createJIntArray(serializedInCAS.getIndexedFSs() );
        jobjectArray inStrings = createJStringArray(serializedInCAS.getStringSymbolTable());
        jbyteArray inByteHeap = createJByteArray(serializedInCAS.getByteHeapArray() );
        jshortArray inShortHeap = createJShortArray(serializedInCAS.getShortHeapArray() );
        jlongArray inLongHeap = createJLongArray(serializedInCAS.getLongHeapArray() );
        vector<internal::SerializedCAS::TyNum> resultSpecTypes;
        vector<internal::SerializedCAS::TyNum> resultSpecFeatures;

        CASSerializer::serializeResultSpec(resultSpec, resultSpecTypes, resultSpecFeatures);

        jintArray inRSTypes = createJIntArray(resultSpecTypes);
        jintArray inRSFeatures = createJIntArray(resultSpecFeatures);
        UIMA_TPRINT("args to Java initialize created");

        int sofaNum=0;
        if (cas.isView() ) {
          sofaNum = cas.getSofaNum();
        }

        // call process()
        jint result = iv_env->CallIntMethod(iv_obj, iv_processMethod,
                                            doc,
                                            inHeap,
                                            inIndexedFSs,
                                            inStrings,
                                            inRSTypes,
                                            inRSFeatures,
                                            sofaNum,
                                            inByteHeap,
                                            inShortHeap,
                                            inLongHeap);
        checkForUnexpectedJNIException();
        handleEngineProxyError(result);
        UIMA_TPRINT("Java initialize done");

        iv_env->PopLocalFrame(NULL);
        checkForUnexpectedJNIException();
      }

      if (err != UIMA_ERR_NONE) {
        return err;
      }

      UIMA_TPRINT("De-serializing per-document data");

      SerializedCAS serializedOutCAS;

      // get serialized CAS from Java object
      jintArray outHeapArray = (jintArray) iv_env->CallObjectMethod(iv_obj, iv_getHeapMethod);
      checkForUnexpectedJNIException();
      createIntVector(outHeapArray, serializedOutCAS.iv_vecFSHeapArray );

      jbyteArray byteHeapArray = (jbyteArray) iv_env->CallObjectMethod(iv_obj, iv_getByteHeapMethod);
      checkForUnexpectedJNIException();
      createByteVector(byteHeapArray, serializedOutCAS.iv_vecByteHeapArray );

      jintArray outIndexedFSsArray = (jintArray) iv_env->CallObjectMethod(iv_obj, iv_getIndexedFSsMethod);
      checkForUnexpectedJNIException();
      createIntVector(outIndexedFSsArray, serializedOutCAS.iv_vecIndexedFSs);

      jobjectArray outStringsArray = (jobjectArray) iv_env->CallObjectMethod(iv_obj, iv_getStringTableMethod);
      checkForUnexpectedJNIException();
      createStringVector(outStringsArray, serializedOutCAS.iv_vecStringSymbolTable, iv_stringPool);

      serializedOutCAS.iv_ulstrDocument = serializedInCAS.getDocument();
      UIMA_TPRINT("De-serialized data received from Java");
      CASDeserializer deSerializer;
      //         deSerializer.deserializeData(serializedOutCAS, TCAS::promoteCAS( cas ));
      deSerializer.deserializeData(serializedOutCAS, cas);
      UIMA_TPRINT("data deserialized");
      return err;
    }



    TyErrorId JEDIIEngine::reconfigureImpl() {
      util::Trace trace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      // no reconfiguration of the JEDII engine possible yet
      return UIMA_ERR_NONE;
    }


    bool JEDIIEngine::hasNextImpl() {
      util::Trace trace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      //TODO:
      return false;
    }


    CAS & JEDIIEngine::nextImpl() {
      util::Trace trace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      //TODO::return iv_pAnnotator->next();
      UIMA_EXC_THROW_NEW(ExcInvalidRequest,
                         UIMA_ERR_NOT_YET_IMPLEMENTED,
                         UIMA_MSG_ID_EXC_INVALID_CALL_TO_NEXT,
                         UIMA_MSG_ID_EXC_INVALID_CALL_TO_NEXT,
                         ErrorInfo::unrecoverable);
    }


    int JEDIIEngine::getCasInstancesRequiredImpl() {
      util::Trace trace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      //TODO::return iv_pAnnotator->next();
      return 0;
    }


    void JEDIIEngine::destroyJavaVM() {
      util::Trace             clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);

      if (iv_createdVM) {
        iv_createdVM = false;
        jint destroyRC = iv_vm->DestroyJavaVM();
        checkForUnexpectedJNIException();
        if (destroyRC != JNI_OK) {
          char c[2056];
          icu::UnicodeString us = javaErrorToString(destroyRC);
          us.extract(0, us.length(), c);
          clTrace.dump("Java VM could not be destroyed, return code: ", c);
#ifndef NDEBUG
          cerr << __FILE__ << __LINE__ << ": Java VM could not be destroyed (return code: " << javaErrorToString(destroyRC) << ")" << endl;
#endif

        }
      }

      iv_vm = NULL;
      iv_env = NULL;
    }


    TyErrorId JEDIIEngine::destroyImpl() {
      util::Trace             clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);
      if (iv_vm != NULL) {
        assert( iv_env != NULL );
        if (iv_obj != NULL) {
          jint destroyResult = iv_env->CallIntMethod(iv_obj, iv_destroyMethod);
          checkForUnexpectedJNIException();
          handleEngineProxyError(destroyResult);

          iv_env->DeleteGlobalRef(iv_obj);
          checkForUnexpectedJNIException();
          iv_obj = NULL;
        }

        destroyJavaVM();
        assert( iv_vm == NULL );
        assert( iv_env == NULL );

        destroyJVMFunctions();
      }
      return UIMA_ERR_NONE;
    }


    TyErrorId JEDIIEngine::batchProcessCompleteImpl() {
      util::Trace             clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);

      TyErrorId err = UIMA_ERR_NONE;
      jint result = iv_env->CallIntMethod(iv_obj, iv_batchProcessCompleteMethod);

      checkForUnexpectedJNIException();
      handleEngineProxyError(result);
      return err;
    }

    TyErrorId JEDIIEngine::collectionProcessCompleteImpl() {
      util::Trace             clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_ENGINE);

      TyErrorId err = UIMA_ERR_NONE;
      jint result = iv_env->CallIntMethod(iv_obj, iv_collectionProcessCompleteMethod);

      checkForUnexpectedJNIException();
      handleEngineProxyError(result);
      return err;
    }



  }
}




