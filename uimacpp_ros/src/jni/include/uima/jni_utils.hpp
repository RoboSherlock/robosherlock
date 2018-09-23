#ifndef UIMA_JNI_UTILS_HPP
#define UIMA_JNI_UTILS_HPP
/** @name jni_utils.hpp
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
#include <jni.h>
#include <uima/jni.hpp>
#include <uima/jni_string.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  class Exception;
}
/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */



class JNIUtils {
private:
  static jfieldID getFieldID(JNIEnv* jeEnv, jobject jo, char const * sig, char const * cpszFieldID);
  /**
   * throws a Java exception of type <TT>org.apache.uima.uimacpp.UimacppException</TT>
   * with <TT>errorMessage</TT>.
   */
//   static void throwNewInternalException(JNIEnv* jeEnv,icu::UnicodeString errorMessage);
  /**
   * throws a Java exception with message <TT>errorMessage</TT> and an <TT>errorId</TT>.
   */
  static void throwNewInternalException(JNIEnv* jeEnv,
                                        icu::UnicodeString errorMessage,
                                        uima::TyErrorId errorId);

public:
  /*
  static bool getBooleanField(JNIEnv* jeEnv, jobject jo, char const * cpszFieldName);
  static int getIntField(JNIEnv* jeEnv, jobject jo, char const * cpszFieldName);
  static float getFloatField(JNIEnv* jeEnv, jobject jo, char const * cpszFieldName);
  */
  static long getLongField(JNIEnv* jeEnv, jobject jo, char const * cpszFieldName);
  static void setLongField(JNIEnv* jeEnv, jobject jo, char const * cpszFieldName, long l);
  /*
  static JNIUString getStringField(JNIEnv* jeEnv, jobject jo, char const * cpszFieldName);
  */

  static jobjectArray createStringArray(JNIEnv* jeEnv, std::vector<uima::UnicodeStringRef> const & crVec);
  static jintArray createIntArray(JNIEnv* jeEnv, std::vector<uima::internal::SerializedCAS::TyNum> const & crVec);
  static jbyteArray createByteArray(JNIEnv* jeEnv, std::vector<char> const & crVec);
  static jshortArray createShortArray(JNIEnv* jeEnv, std::vector<short> const & crVec);
  static jlongArray createLongArray(JNIEnv* jeEnv, std::vector<INT64> const & crVec);

  static void createStringVector(JNIEnv* jeEnv, jobjectArray, std::vector<uima::UnicodeStringRef> & rResult, std::vector<icu::UnicodeString> & );
  static void createIntVector(JNIEnv* jeEnv, jintArray, std::vector<uima::internal::SerializedCAS::TyNum> & rResult);
  static void createByteVector(JNIEnv* jeEnv, jbyteArray, std::vector<char> & rResult);
  static void createShortVector(JNIEnv* jeEnv, jshortArray, std::vector<short> & rResult);
  static void createLongVector(JNIEnv* jeEnv, jlongArray, std::vector<INT64> & rResult);


  static void throwNewInternalException(JNIEnv* jeEnv,
                                        uima::ErrorInfo const & msg);

  static void throwNewInternalException(JNIEnv* jeEnv,
                                        uima::Exception &);

  /**
   * sets the private field <TT>cppEnginePointer</TT> in the Java object to <TT>pclEngine</TT>.
   */
  static void setCppInstance(JNIEnv* jeEnv,
                             jobject joJUima,
                             uima::JNIInstance* pInstance);
  /**
   * gets the UimaClJNIInstance object from the java object.
   */
  static uima::JNIInstance* getCppInstance(JNIEnv* jeEnv, jobject joJUima);

};

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */

#endif
/* <EOF> */

