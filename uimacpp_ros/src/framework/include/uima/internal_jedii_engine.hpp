/** \file internal_jedii_engine.hpp .
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


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <jni.h>

#include <uima/dllfile.hpp>

#include <uima/internal_engine_base.hpp>
#include <uima/internal_serializedcas.hpp>
#include <uima/timedatetools.hpp>
#include <uima/consoleui.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace internal {

    UIMA_EXC_CLASSDECLARE(JavaException, Exception);
    //Compatible UIMA Java version
#define NUM_COMPATIBLE_UIMA_JAVA_VERSIONS 2

    static const TCHAR * gs_compatibleUIMAVersions[] = {"2.0.0"
                                                       };
    /**
     * Concrete engine implementation for a JEDII engine.
     * Uses JNI API.
     */
    class UIMA_LINK_IMPORTSPEC JEDIIEngine : public EngineBase {
    private:
      /// The Java VM handle
      JavaVM * iv_vm;
      // A handle to the JNI Environment
      JNIEnv * iv_env;
      // A reference to the proxy TAE object on the Java side
      jobject iv_obj;
      bool iv_createdVM;

      // method IDs for the methods to be called on iv_obj
      jmethodID iv_initializeMethod;
      jmethodID iv_processMethod;
      jmethodID iv_destroyMethod;
      jmethodID iv_getLastExceptionStringMethod;
      jmethodID iv_getHeapMethod;
      jmethodID iv_getIndexedFSsMethod;
      jmethodID iv_getStringTableMethod;

      jmethodID iv_getByteHeapMethod;
      jmethodID iv_getShortHeapMethod;
      jmethodID iv_getLongHeapMethod;

      jmethodID iv_batchProcessCompleteMethod;
      jmethodID iv_collectionProcessCompleteMethod;
      jmethodID iv_getVersionMethod;


      // temporary string pool for deserialized strings
      std::vector<icu::UnicodeString> iv_stringPool;

      // 0: not set, 1: primitive, 2: aggregate
      int iv_isPrimitive;

      /**
       * We load the Java DLL dynamically so the UIMACPP DLL does not have a 
       * hard dependency to Java. This way, it can run also on a machine
       * where a JDK is not available.
       */
      util::DllProcLoaderFile * iv_JVMDllLoader;

      // function pointers for the JNI functions which are not inline
      typedef jint (JNICALL * TyCreateJavaVMFunc)(JavaVM**, void**, void*);
      typedef jint (JNICALL * TyGetCreatedJavaVMsFunc)(JavaVM**, jsize, jsize*);
      TyCreateJavaVMFunc iv_createJavaVMFunc;
      TyGetCreatedJavaVMsFunc iv_getCreatedJavaVMsFunc;

      // handle error on the Java proxy ("normal errors")
      void handleEngineProxyError(jint rc);
      // handle JNI Error (1)
      void handleJNIError(jint rc, icu::UnicodeString const &);
      // handle JNI Error (2)
      void handleJNIError(jint rc);
      // check if some JNI exception occurred and throw exception if yes
      void checkForUnexpectedJNIException();

      // functions to convert C++ data structures to the Java equivalent
      jintArray createJIntArray(std::vector<uima::internal::SerializedCAS::TyNum> const & v);
      jbyteArray createJByteArray(std::vector<char> const & v);
      jshortArray createJShortArray(std::vector<short> const & v);
      jlongArray createJLongArray(std::vector<INT64> const & v);


      jobjectArray createJStringArray(std::vector<UnicodeStringRef> const & v);
      jstring createJString(UnicodeStringRef const & ref);
      void createIntVector(jintArray ar, std::vector<uima::internal::SerializedCAS::TyNum>& result);
      void createStringVector(jobjectArray ar, std::vector<UnicodeStringRef>& result, std::vector<icu::UnicodeString> & pool);
      void createByteVector(jbyteArray ar,std:: vector<char>& result);
      void createShortVector(jshortArray ar, std::vector<short>& result);
      void createLongVector(jlongArray ar, std::vector<long>& result);
      // initialize a Java VM to iv_obj
      void initializeJavaVM();
      // destroy the Java VM
      void destroyJavaVM();

      // load JDK DLL and get handle to needed functions
      void initializeJVMFunctions();
      // unload JDK DLL
      void destroyJVMFunctions();

    public:
      JEDIIEngine(AnnotatorContext &,
                  bool bOwnsANC,
                  bool bOwnsTAESpecififer,
                  uima::internal::CASDefinition&,
                  bool ownsCAsDef);
      ~JEDIIEngine();

      virtual TyErrorId initializeImpl(AnalysisEngineDescription const & );
      virtual TyErrorId processImpl(CAS & cas, ResultSpecification const & crResultSpecification);
      virtual TyErrorId destroyImpl();
      virtual TyErrorId reconfigureImpl();
      virtual TyErrorId reinitTypeSystemImpl();
      virtual TyErrorId batchProcessCompleteImpl();
      virtual TyErrorId collectionProcessCompleteImpl();
      virtual bool hasNextImpl();
      virtual CAS & nextImpl();
      virtual int getCasInstancesRequiredImpl();
      virtual bool isPrimitive() const;


#ifdef UIMA_DEBUG_ANNOTATOR_TIMING
    private:
      Timer iv_timer;
    public:
      virtual const Timer &              getLoadTimer(void) const {
        return iv_timer;
      }

      virtual const Timer &              getInitTimer(void) const {
        return iv_timer;
      }

      virtual const Timer &              getDeInitTimer(void) const {
        return iv_timer;
      }

      virtual const Timer &              getConfigTimer(void) const {
        return iv_timer;
      }

      virtual const Timer &              getProcessDocumentTimer(void) const {
        return iv_timer;
      }

      virtual void displayTimingData(util::ConsoleUI const &, bool bVerbose = false) const {}
#endif

    };

  }
}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */



