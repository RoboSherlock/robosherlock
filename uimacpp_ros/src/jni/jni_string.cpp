/** @name jni_string.cpp

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

/*@{*/                                                /* Doc++ Group */

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/jni_string.hpp>
#include <uima/ccsid.hpp>
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/*
 If UIMA_JNI_RELEASE_STRING_BUG is defined, the ReleaseString JNI calls
 are done even if the GetString call did not copy the string.

 See also
 http://www-106.ibm.com/developerworks/java/library/j-jtctips/j-jtc0117d.html
 */
#define UIMA_JNI_RELEASE_STRING_BUG

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */
using namespace std;
//CosClCCSID::CosEnCCSID JNIUString::iv_enSystemCCSID = CosClCCSID::getSystemCCSID();

JNIStringABase::JNIStringABase(JNIEnv* jeEnv, jstring jsString) :
    iv_jeEnv(jeEnv),
    iv_jbCopied(false),
//    iv_cpjcData(NULL),
    iv_ulLength(0),
    iv_jsString(jsString) {
  assert( EXISTS(jeEnv) );
}


JNIUString::JNIUString(JNIEnv* jeEnv, jstring jsString) :
    JNIStringABase(jeEnv, jsString),
    iv_cpjcData(NULL) {
  iv_cpjcData = iv_jeEnv->GetStringChars(iv_jsString, &iv_jbCopied);
  assert( iv_cpjcData != NULL );
  iv_ulLength = iv_jeEnv->GetStringLength(iv_jsString);
}

JNIUString::~JNIUString() {
#ifndef UIMA_JNI_RELEASE_STRING_BUG
  if (iv_jbCopied == JNI_TRUE) {
#endif
    iv_jeEnv->ReleaseStringChars(iv_jsString, iv_cpjcData);
#ifndef UIMA_JNI_RELEASE_STRING_BUG
  }
#endif
}

void JNIUString::convertToSystemCodePage(string& rsResult) {
  // Convert to default encoding for platform
  assertWithMsg(sizeof(UChar) == sizeof(jchar), "Port required");
  uima::UnicodeStringRef usr((const UChar*)getData(), getLength());
  usr.extract(rsResult);
}


//////////////////////////////////////////////////////////////

JavaObject::JavaObject(JNIEnv* jeEnv)
    : iv_jeEnv(jeEnv),
    iv_jclass(NULL),
    iv_jmidConstructor(NULL),
    iv_jObject(NULL) {
  assert( EXISTS(jeEnv) );
}

JavaObject::JavaObject(JNIEnv* jeEnv, char const * cpszClass, char const * cpszConstructorSignature)
    : iv_jeEnv(jeEnv),
    iv_jclass(NULL),
    iv_jmidConstructor(NULL),
    iv_jObject(NULL) {
  assert( EXISTS(iv_jeEnv) );
  iv_jclass = iv_jeEnv->FindClass(cpszClass);
  ASSERT_NO_JNI_EXCEPTION(iv_jeEnv);
  iv_jmidConstructor = iv_jeEnv->GetMethodID(iv_jclass, "<init>", cpszConstructorSignature);
  ASSERT_NO_JNI_EXCEPTION(iv_jeEnv);
}


JavaString::JavaString(JNIEnv* jeEnv, uima::UnicodeStringRef const & rulString)
    : JavaObject(jeEnv) {
  assertWithMsg(sizeof(jchar) == sizeof(UChar), "Port required");
  jchar const * cpjcData = (jchar const *) rulString.getBuffer();
  iv_jObject = NULL;
  if ( cpjcData != NULL ) {
    assert( cpjcData != NULL );
    jsize jsLength = (jsize) rulString.length();
    iv_jObject = iv_jeEnv->NewString(cpjcData, jsLength);
    ASSERT_NO_JNI_EXCEPTION(iv_jeEnv);
  }
}


/* ----------------------------------------------------------------------- */
/*@}*/                                                /* e-o-Doc++ Group */




