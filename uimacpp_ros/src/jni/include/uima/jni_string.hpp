#ifndef UIMA_JNI_STRING_HPP
#define UIMA_JNI_STRING_HPP
/** \file jni_string.hpp .
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


   07/26/2000  Initial creation

-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/assertmsg.h>

#include <uima/jni.hpp>

#include <jni.h>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

// strings which are passed to the JNI code

class JNIStringABase {
protected:
  JNIEnv* iv_jeEnv;
  jboolean iv_jbCopied;
//   void const * iv_cpvData;
  unsigned long iv_ulLength;
  jstring iv_jsString;
public:
  JNIStringABase(JNIEnv* jeEnv, jstring jsString);

  virtual ~JNIStringABase() {}

  unsigned long getLength() {
    return iv_ulLength;
  }
};

class JNIUString : public JNIStringABase {
private:
  jchar const * iv_cpjcData;

public:
  JNIUString(JNIEnv* jeEnv, jstring jsString);
  virtual ~JNIUString();

  jchar const * getData() const {
    return iv_cpjcData;
  }

  //   CosClCCSID::CosEnCCSID getCCSID() const {
  //      return CosClCCSID::CosEnCCSID_13488;
  //   }

  uima::UnicodeStringRef toUStrPtrLenPair() const {
    assert( sizeof(UChar) == sizeof(jchar) );
    return uima::UnicodeStringRef( (UChar const *) iv_cpjcData, iv_ulLength );
  }

  void convertToSystemCodePage(std::string& rsResult);

};

//////////////////////////////////////////////////////////////
// java objects to be created from the JNI code

class JavaObject {
protected:
  JNIEnv* iv_jeEnv;
  jclass iv_jclass;
  jobject iv_jObject;
  jmethodID iv_jmidConstructor;

  void createJObject(jvalue* pjvArgs) {
    iv_jObject = iv_jeEnv->NewObjectA(iv_jclass, iv_jmidConstructor, pjvArgs);
    ASSERT_NO_JNI_EXCEPTION(iv_jeEnv);
  }
  JavaObject(JNIEnv* jeEnv);
  JavaObject(JNIEnv* jeEnv, char const * cpszClass, char const * cpszConstructorSignature);
public:
  jobject getJObject() const {
    return iv_jObject;
  }

  JNIEnv* getJEnv() const {
    return iv_jeEnv;
  }

};


class JavaString : public JavaObject {
public:
  JavaString(JNIEnv* jeEnv, uima::UnicodeStringRef const & rulString);
};


/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */

#endif
/* <EOF> */

