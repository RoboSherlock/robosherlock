#ifndef UIMA_CASJNI_HPP
#define UIMA_CASJNI_HPP
/** \file jni.hpp .
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
#include <uima/internal_casserializer.hpp>
#include <uima/internal_casimpl.hpp>
#include <uima/engine.hpp>
#include <uima/annotator_context.hpp>
#include <uima/casiterator.hpp>
#include "apr_portable.h"
#include <jni.h>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */
#define UIMA_JNI_VERSION    "2.0"

#define CHECK_FOR_JNI_EXCEPTION(JEENV) \
  if (JEENV->ExceptionOccurred() != NULL) { \
      UIMA_TPRINT("JNI Exception occurred at line " << __LINE__); \
         return; \
   }


#define ASSERT_NO_JNI_EXCEPTION(JEENV) assert( JEENV->ExceptionOccurred() == NULL )
//#define ASSERT_NO_JNI_EXCEPTION(JEENV)

#define CHECK_FOR_JNI_EXCEPTION_WITH_RETURN_VALUE(JEENV, RETURN_VALUE) \
  if (JEENV->ExceptionOccurred() != NULL) { \
      UIMA_TPRINT("JNI Exception occurred at line " << __LINE__); \
      return RETURN_VALUE; \
   }

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */
class UIMA_LINK_IMPORTSPEC JNILogger : public  uima::Logger {
    public:
      JNILogger(JNIEnv * env); 
      
      virtual void log(uima::LogStream::EnEntryType entrytype, 
                  std::string classname,
                  std::string methodname,
                  std::string message,
                  long errorCode)  ;
   
    private:
      /** Format the log message */
      std::string format(uima::LogStream::EnEntryType enType,
                        const std::string & cpszMsg, 
                        long lUserCode) const;
      JavaVM * iv_jvm;
      jclass   cv_clazz ;    //proxy class on java side
      jmethodID cv_logMethod; //log method
  };
    

namespace uima {
  

class JNIInstance {
private:
  uima::AnalysisEngine * iv_pEngine;
  uima::CAS * iv_pCAS;
  uima::CAS * iv_pSegment;
  uima::internal::SerializedCAS iv_serializedCAS;
  uima::internal::SerializedCAS iv_serializedSegment;
  bool iv_hasNext;

public:
  //JNILogger * iv_logger;
  JNIInstance() :
      iv_pEngine(NULL),
      iv_pCAS(NULL),
      iv_serializedCAS(),
      iv_pSegment(NULL),
      iv_serializedSegment(),
      iv_hasNext(false){}

  ~JNIInstance() {
 
  }

  uima::AnalysisEngine * getEngine() {
    return iv_pEngine;
  }

  void setEngine(uima::AnalysisEngine * pEngine) {
    iv_pEngine = pEngine;
  }

  void setCAS(uima::CAS * pCAS) {
    iv_pCAS = pCAS;
  }

  /*
  uima::internal::CASSerializer & getCASSerializer() {
     return iv_CASSerializer;
  }
  */

  uima::internal::SerializedCAS & getSerializedCAS() {
    return iv_serializedCAS;
  }

  uima::CAS * getCAS() {
    return iv_pCAS;
  }

  bool hasNext() {
    iv_hasNext = iv_pEngine->hasNext();
    return iv_hasNext;
  }

  void next() {
    UIMA_TPRINT("Calling engine.next()");
    iv_pSegment = &(iv_pEngine->next());
    UIMA_TPRINT(iv_pSegment->getDocumentText());
    assert( EXISTS(iv_pSegment) );
  }

  uima::CAS * getSegment() {
    assert( EXISTS(iv_pSegment) );
    return iv_pSegment;
  }

  void releaseSegment() {
    if (EXISTS(iv_pSegment)) {
      iv_pEngine->getAnnotatorContext().releaseCAS(*iv_pSegment);
      iv_pSegment=NULL;
      iv_serializedSegment.reset();
    }
  }

  uima::internal::SerializedCAS & getSerializedSegment() {
    return iv_serializedSegment;
  }

  uima::internal::CASImpl & getCASImpl() {
    assert( EXISTS(iv_pCAS) );
    return uima::internal::CASImpl::promoteCAS( *iv_pCAS );
  }

};
}
/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif
