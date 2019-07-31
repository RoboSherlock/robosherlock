/** @name jni_utils.cpp

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
#include <uima/pragmas.hpp>
#include <uima/jni_utils.hpp>
#include <uima/macros.h>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */
#define JAVA_CLASS_INTERNALTAFEXCEPTION "org/apache/uima/uimacpp/InternalTafException"
#define JUIMA_FIELD_CPPENGINEPOINTER "cppEnginePointer"
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
jfieldID JNIUtils::getFieldID(JNIEnv* jeEnv, jobject jo, char const * sig, char const * cpszFieldID) {
  jclass clazz = jeEnv->GetObjectClass(jo);
  ASSERT_NO_JNI_EXCEPTION(jeEnv);
  jfieldID jFID = jeEnv->GetFieldID(clazz, cpszFieldID, sig);
  ASSERT_NO_JNI_EXCEPTION(jeEnv);
  return jFID;
}


long JNIUtils::getLongField(JNIEnv* jeEnv, jobject jo, char const * cpszFieldName) {
  assertWithMsg(sizeof(long) == sizeof(jlong), "Port required");
  jlong jValue = jeEnv->GetLongField(jo, getFieldID(jeEnv, jo, "J", cpszFieldName) );
  ASSERT_NO_JNI_EXCEPTION(jeEnv);
  return (long) jValue;
}

void JNIUtils::setLongField(JNIEnv* jeEnv, jobject jo, char const * cpszFieldName, long l) {
  assertWithMsg(sizeof(long) == sizeof(jlong), "Port required");
  jeEnv->SetLongField(jo, getFieldID(jeEnv, jo, "J", cpszFieldName), (jlong) l);
  ASSERT_NO_JNI_EXCEPTION(jeEnv);
}

/*
void JNIUtils::throwNewInternalException(JNIEnv* jeEnv,
                                            icu::UnicodeString errorMessage) {
   throwNewInternalException(jeEnv, errorMessage, UIMA_ERR_ENGINE_UNEXPECTED_EXCEPTION);
}
*/

void JNIUtils::throwNewInternalException(JNIEnv* jeEnv,
    icu::UnicodeString errorMessage,
    uima::TyErrorId errorId) {
  jclass clazz = jeEnv->FindClass(JAVA_CLASS_INTERNALTAFEXCEPTION);
  ASSERT_NO_JNI_EXCEPTION(jeEnv);
  // get the constructor which takes a message and an errorID
  jmethodID jmExcConstructor = jeEnv->GetMethodID(clazz, "<init>", "(Ljava/lang/String;J)V");
  ASSERT_NO_JNI_EXCEPTION(jeEnv);
  assertWithMsg( sizeof(jchar) == sizeof(UChar), "Port required");
  assertWithMsg( sizeof(uima::TyErrorId) <= sizeof(jlong), "Port required");
  jstring jsErrorMessage = jeEnv->NewString( (const jchar*) errorMessage.getBuffer(), errorMessage.length());
  jthrowable jtIkfInternalException = (jthrowable) jeEnv->NewObject(clazz, jmExcConstructor, jsErrorMessage, (jlong) errorId);
  jeEnv->Throw(jtIkfInternalException);
}


void JNIUtils::throwNewInternalException(JNIEnv* jeEnv, uima::Exception & rException) {
  string sExc = rException.asString();
  icu::UnicodeString usExc(sExc.c_str(), sExc.length());
  throwNewInternalException(jeEnv, usExc, rException.getErrorInfo().getErrorId());
}

void JNIUtils::throwNewInternalException(JNIEnv* jeEnv, uima::ErrorInfo const & errInfo) {
  string sExc = errInfo.asString();
  icu::UnicodeString usExc(sExc.c_str(), sExc.length());
  throwNewInternalException(jeEnv, usExc, errInfo.getErrorId());
}


void JNIUtils::setCppInstance(JNIEnv* jeEnv,
                              jobject joJTaf,
                              uima::JNIInstance* pInstance) {
  jclass clazz = jeEnv->GetObjectClass(joJTaf);
  ASSERT_NO_JNI_EXCEPTION(jeEnv);
  // get the field of clazz named "cppEnginePointer" which is a long
  jfieldID fid = jeEnv->GetFieldID(clazz, JUIMA_FIELD_CPPENGINEPOINTER, "J");
  ASSERT_NO_JNI_EXCEPTION(jeEnv);
  assertWithMsg( sizeof(void*) <= sizeof(jlong), "Port required");
  jeEnv->SetLongField(joJTaf, fid, (jlong) pInstance);
  ASSERT_NO_JNI_EXCEPTION(jeEnv);
}

uima::JNIInstance* JNIUtils::getCppInstance(JNIEnv* jeEnv,
                                      jobject joJTaf) {
  jclass clazz = jeEnv->GetObjectClass(joJTaf);
  ASSERT_NO_JNI_EXCEPTION(jeEnv);
  jfieldID fid = jeEnv->GetFieldID(clazz, JUIMA_FIELD_CPPENGINEPOINTER, "J");
  ASSERT_NO_JNI_EXCEPTION(jeEnv);
  jlong jlPointer = jeEnv->GetLongField(joJTaf, fid);
  ASSERT_NO_JNI_EXCEPTION(jeEnv);
  assertWithMsg( sizeof(void*) <= sizeof(jlong), "Port required");
  return(uima::JNIInstance*) jlPointer;
}


jobjectArray JNIUtils::createStringArray(JNIEnv* jeEnv, vector<uima::UnicodeStringRef> const & crVec) {
  jobjectArray joArr = NULL;
  size_t uiLen = crVec.size();
  ASSERT_NO_JNI_EXCEPTION(jeEnv);
  jclass jcStringClass = jeEnv->FindClass("java/lang/String");
  assert( jcStringClass != NULL );
  ASSERT_NO_JNI_EXCEPTION(jeEnv);
  joArr = jeEnv->NewObjectArray( uiLen, jcStringClass, NULL );
  ASSERT_NO_JNI_EXCEPTION(jeEnv);

  size_t i;
  for (i=0; i<uiLen; ++i) {
    UIMA_TPRINT("Copying string " << i);
    JavaString str(jeEnv, crVec[i]);
    UIMA_TPRINT("Entering string " << i << " into array");
    jeEnv->SetObjectArrayElement(joArr, (jsize) i, str.getJObject());
    ASSERT_NO_JNI_EXCEPTION(jeEnv);
  }
  return joArr;
}

jintArray JNIUtils::createIntArray(JNIEnv* jeEnv, vector<uima::internal::SerializedCAS::TyNum> const & crVec) {
  jintArray jiArr = NULL;
  assert( sizeof(uima::internal::SerializedCAS::TyNum) == sizeof(jint) );

  size_t uiLength = crVec.size();
//   assert( uiLength > 0);
  jiArr = jeEnv->NewIntArray(uiLength);
  ASSERT_NO_JNI_EXCEPTION(jeEnv);

#ifdef TEMP_DISABLED
  // call to SetIntArrayRegion() assumes that vector iterators
  //   are implemented as pointers
  // check at least that they have equal size by the following assert
  assertWithMsg( sizeof(vector<uima::internal::CASSerializer::TyNum>::iterator) == sizeof(void*), "Port required!");
  jeEnv->SetIntArrayRegion(jiArr, 0, uiLength, (jint*) crVec.begin());
  ASSERT_NO_JNI_EXCEPTION(jeEnv);
#endif

  if (uiLength > 0) {
    // since the above code does not work for msc7,
    // allocate a new array and copy it by hand
    jint * pJIntArray = new jint[uiLength];
    size_t i=0;
    vector<uima::internal::SerializedCAS::TyNum>::const_iterator cit;
    for (cit = crVec.begin(); cit != crVec.end(); ++cit) {
      pJIntArray[i++] = (jint) *cit;
    }
    assert( i == uiLength );

    jeEnv->SetIntArrayRegion(jiArr, 0, uiLength, pJIntArray);
    ASSERT_NO_JNI_EXCEPTION(jeEnv);
    delete pJIntArray;
  }

  return jiArr;
}


void JNIUtils::createIntVector(JNIEnv* jeEnv, jintArray array, vector<uima::internal::SerializedCAS::TyNum> & rResult) {
  size_t uiLen = jeEnv->GetArrayLength(array);
  rResult.resize(uiLen);

  jeEnv->PushLocalFrame( uiLen );
  jboolean jbCopy;
  jint * pArray = jeEnv->GetIntArrayElements(array, & jbCopy);
  size_t i;
  for (i=0; i<uiLen; ++i) {
    rResult[i] = pArray[i];
  }
  jeEnv->ReleaseIntArrayElements(array, pArray, JNI_ABORT);
  jeEnv->PopLocalFrame(NULL);
}


void JNIUtils::createStringVector(JNIEnv* jeEnv, jobjectArray joArray, vector<uima::UnicodeStringRef> & rResult, vector<icu::UnicodeString> & rStringPool) {
  size_t uiLen = jeEnv->GetArrayLength(joArray);
  rResult.resize(uiLen);
  rStringPool.resize(uiLen);
  jeEnv->PushLocalFrame( uiLen );
//   cout << __FILE__ << __LINE__ << ": len: " << uiLen << endl;
  size_t i;
  for (i=0; i<uiLen; ++i) {
    jobject jo = jeEnv->GetObjectArrayElement(joArray, i);

    if (jo != 0) {
      jstring js = (jstring) jo;
      JNIUString uString(jeEnv, js);
      uima::UnicodeStringRef ref = uString.toUStrPtrLenPair();
//         cout << __FILE__ << __LINE__ << ":" << i << "  string " << uString.toUStrPtrLenPair() << endl;
      rStringPool[i] = UnicodeString( ref.getBuffer(), ref.length() );
      rResult[i] = uima::UnicodeStringRef(rStringPool[i]);
    } else {
      rResult[i] = uima::UnicodeStringRef();
    }
  }
   jeEnv->PopLocalFrame(NULL);
}

jbyteArray JNIUtils::createByteArray(JNIEnv* jeEnv, vector<char> const & crVec) {
  jbyteArray jiArr = NULL;

  size_t uiLength = crVec.size();
//   assert( uiLength > 0);
  jiArr = jeEnv->NewByteArray(uiLength);
  ASSERT_NO_JNI_EXCEPTION(jeEnv);

#ifdef TEMP_DISABLED
  // call to SetIntArrayRegion() assumes that vector iterators
  //   are implemented as pointers
  // check at least that they have equal size by the following assert
  //assertWithMsg( sizeof(vector<uima::internal::CASSerializer::TyNum>::iterator) == sizeof(void*), "Port required!");
  jeEnv->SetByteArrayRegion(jiArr, 0, uiLength, (jbyte*) crVec.begin());
  ASSERT_NO_JNI_EXCEPTION(jeEnv);
#endif

  if (uiLength > 0) {
    // since the above code does not work for msc7,
    // allocate a new array and copy it by hand
    jbyte * pJByteArray = new jbyte[uiLength];
    size_t i=0;
    vector<char>::const_iterator cit;
    for (cit = crVec.begin(); cit != crVec.end(); ++cit) {
      pJByteArray[i++] = (jbyte) *cit;
    }
    assert( i == uiLength );

    jeEnv->SetByteArrayRegion(jiArr, 0, uiLength, pJByteArray);
    ASSERT_NO_JNI_EXCEPTION(jeEnv);
    delete pJByteArray;
  }

  return jiArr;
}

jshortArray JNIUtils::createShortArray(JNIEnv* jeEnv, vector<short> const & crVec) {
  jshortArray jiArr = NULL;

  size_t uiLength = crVec.size();
//   assert( uiLength > 0);
  jiArr = jeEnv->NewShortArray(uiLength);
  ASSERT_NO_JNI_EXCEPTION(jeEnv);

#ifdef TEMP_DISABLED
  // call to SetIntArrayRegion() assumes that vector iterators
  //   are implemented as pointers
  // check at least that they have equal size by the following assert
  //assertWithMsg( sizeof(vector<uima::internal::CASSerializer::TyNum>::iterator) == sizeof(void*), "Port required!");
  jeEnv->SetShortArrayRegion(jiArr, 0, uiLength, (jshort*) crVec.begin());
  ASSERT_NO_JNI_EXCEPTION(jeEnv);
#endif

  if (uiLength > 0) {
    // since the above code does not work for msc7,
    // allocate a new array and copy it by hand
    jshort * pJShortArray = new jshort[uiLength];
    size_t i=0;
    vector<short>::const_iterator cit;
    for (cit = crVec.begin(); cit != crVec.end(); ++cit) {
      pJShortArray[i++] = (jshort) *cit;
    }
    assert( i == uiLength );

    jeEnv->SetShortArrayRegion(jiArr, 0, uiLength, pJShortArray);
    ASSERT_NO_JNI_EXCEPTION(jeEnv);
    delete pJShortArray;
  }

  return jiArr;
}


jlongArray JNIUtils::createLongArray(JNIEnv* jeEnv, vector<INT64> const & crVec) {
  jlongArray jiArr = NULL;

  size_t uiLength = crVec.size();
//   assert( uiLength > 0);
  jiArr = jeEnv->NewLongArray(uiLength);
  ASSERT_NO_JNI_EXCEPTION(jeEnv);

#ifdef TEMP_DISABLED
  // call to SetIntArrayRegion() assumes that vector iterators
  //   are implemented as pointers
  // check at least that they have equal size by the following assert
  //assertWithMsg( sizeof(vector<uima::internal::CASSerializer::TyNum>::iterator) == sizeof(void*), "Port required!");
  jeEnv->SetLongArrayRegion(jiArr, 0, uiLength, (jlong*) crVec.begin());
  ASSERT_NO_JNI_EXCEPTION(jeEnv);
#endif

  if (uiLength > 0) {
    // since the above code does not work for msc7,
    // allocate a new array and copy it by hand
    jlong * pJLongArray = new jlong[uiLength];
    size_t i=0;
    vector<INT64>::const_iterator cit;
    for (cit = crVec.begin(); cit != crVec.end(); ++cit) {
      pJLongArray[i++] = (jlong) *cit;
    }
    assert( i == uiLength );

    jeEnv->SetLongArrayRegion(jiArr, 0, uiLength, pJLongArray);
    ASSERT_NO_JNI_EXCEPTION(jeEnv);
    delete pJLongArray;
  }

  return jiArr;
}

void JNIUtils::createByteVector(JNIEnv* jeEnv, jbyteArray array, vector<char> & rResult) {
  size_t uiLen = jeEnv->GetArrayLength(array);
  rResult.resize(uiLen);
  jeEnv->PushLocalFrame( uiLen );
  jboolean jbCopy;
  jbyte * pArray = jeEnv->GetByteArrayElements(array, & jbCopy);
  size_t i;
  for (i=0; i<uiLen; ++i) {
    rResult[i] = pArray[i];
  }
  jeEnv->ReleaseByteArrayElements(array,pArray, JNI_ABORT);
  jeEnv->PopLocalFrame(NULL);
}

void JNIUtils::createShortVector(JNIEnv* jeEnv, jshortArray array, vector<short> & rResult) {
  size_t uiLen = jeEnv->GetArrayLength(array);
  rResult.resize(uiLen);
  jeEnv->PushLocalFrame( uiLen );
  jboolean jbCopy;
  jshort * pArray = jeEnv->GetShortArrayElements(array, & jbCopy);
  size_t i;
  for (i=0; i<uiLen; ++i) {
    rResult[i] = pArray[i];
  }
  jeEnv->ReleaseShortArrayElements(array,pArray, JNI_ABORT);
  jeEnv->PopLocalFrame(NULL);
}

void JNIUtils::createLongVector(JNIEnv* jeEnv, jlongArray array, vector<INT64> & rResult) {
  size_t uiLen = jeEnv->GetArrayLength(array);
  rResult.resize(uiLen);
  jeEnv->PushLocalFrame( uiLen );
  jboolean jbCopy;
  jlong * pArray = jeEnv->GetLongArrayElements(array, & jbCopy);
  size_t i;
  for (i=0; i<uiLen; ++i) {
    rResult[i] = pArray[i];
  }
  jeEnv->ReleaseLongArrayElements(array,pArray, JNI_ABORT);
  jeEnv->PopLocalFrame(NULL);
}



/* ----------------------------------------------------------------------- */
/*@}*/                                                /* e-o-Doc++ Group */

/* <EOF> */

