///*
// * Licensed to the Apache Software Foundation (ASF) under one
// * or more contributor license agreements.  See the NOTICE file
// * distributed with this work for additional information
// * regarding copyright ownership.  The ASF licenses this file
// * to you under the Apache License, Version 2.0 (the
// * "License"); you may not use this file except in compliance
// * with the License.  You may obtain a copy of the License at
// *
// *   http://www.apache.org/licenses/LICENSE-2.0
// *
// * Unless required by applicable law or agreed to in writing,
// * software distributed under the License is distributed on an
// * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// * KIND, either express or implied.  See the License for the
// * specific language governing permissions and limitations
// * under the License.
// */

#ifdef SWIGPYTHON
%module pythonnator
#endif
#ifdef SWIGPERL
%module perltator
#endif
#ifdef SWIGTCL
%module tclator
#endif

%{
#ifdef SWIGPERL
// isASCII in perl conflicts with UIMA definition
#undef isASCII
#endif
#include "uima/api.hpp"

using namespace uima;
using namespace icu;
using namespace std;

#define utf8sid "UTF-8"

#ifdef SWIGPYTHON
// convert from unicode
static bool PyUnicodeConvert(PyObject *obj, UnicodeString &rv) {
  if (sizeof(Py_UNICODE) == sizeof(UChar)) {
    rv.setTo((const UChar *) PyUnicode_AS_UNICODE(obj),
      (int32_t) PyUnicode_GET_SIZE(obj));
  } else {
    int32_t len = (int32_t) PyUnicode_GET_SIZE(obj);
    Py_UNICODE *pchars = PyUnicode_AS_UNICODE(obj);
    UChar *chars = new UChar[len * 3];
    UErrorCode status = U_ZERO_ERROR;
    int32_t dstLen;
    u_strFromUTF32(chars, len*3, &dstLen, (const UChar32 *) pchars,
	len, &status);
    if (U_FAILURE(status)) {
      delete chars;
      return false;
    }
    rv.setTo((const UChar *) chars, dstLen);
    delete chars;
  }
  return true;
}

// convert using default codepage
static bool PyStringConvert(PyObject *obj, UnicodeString &rv) {
  char *src;
  #ifdef PY_VERSION_HEX
  #if (PY_VERSION_HEX >= 0x02050000)
    /* Python version was greater than 2.5 */
    Py_ssize_t len;
     PyString_AsStringAndSize(obj, &src,  ( Py_ssize_t*)&len);
  #else
    /* Python version was less than 2.5 */
     int len;
     PyString_AsStringAndSize(obj, &src, &len);
  #endif
  #else
    /* Could not determine version */ 
    PyString_AsStringAndSize(obj, &src, &len);
  #endif
  rv = UnicodeString((const char *) src, (int32_t) len);
  return true;
}

static bool ConvertUnicodeStringRef(const UnicodeStringRef &ref,
        PyObject **rv) {
  if (sizeof(Py_UNICODE) == sizeof(UChar)) {
    *rv = PyUnicode_FromUnicode((const Py_UNICODE*) ref.getBuffer(),
        ref.length());
  } else {
    // test for big-endian, preset python decoder for native order
    // this will prevent PyUnicode_DecodeUTF16 from deleting byte order marks
    union { long l; char c[sizeof(long)]; } u;
    u.l = 1;
    int byteorder = (u.c[sizeof(long) - 1] == 1) ? 1 : -1;
    PyObject *r = PyUnicode_DecodeUTF16(
       (const char *) ref.getBuffer(), ref.getSizeInBytes(), 0, &byteorder);
    if (r==0) return false;
    *rv = r;
  }
  return true;
}
#endif

%}

%include exception.i

#ifdef SWIGPYTHON
%typemap(out) UnicodeStringRef {
  if (!ConvertUnicodeStringRef($1, &$result))
    SWIG_exception(SWIG_TypeError, "unicode string expected");
}

%typemap(out) UnicodeString {
  if (!ConvertUnicodeStringRef($1, &$result))
    SWIG_exception(SWIG_TypeError, "unicode string expected");
}

%typemap(in) const UnicodeString & (UnicodeString temp) {
  if ( !( (PyUnicode_CheckExact($input) && PyUnicodeConvert($input, temp))
	|| (PyString_CheckExact($input) && PyStringConvert($input, temp))))
    SWIG_exception(SWIG_TypeError, "string or unicode string expected or conversion failure");
  $1 = &temp;
}

%typemap(out) vector<Feature> {
  size_t len = $1.size();
  $result = PyList_New($1.size());
  for (size_t i=0; i<len; i++) {
    PyList_SetItem($result, i,
      SWIG_NewPointerObj(new Feature((($1_type &)$1)[i]),
         $descriptor(Feature *), 1));
  }
}

#endif

#ifdef SWIGPERL
%typemap(out) UnicodeStringRef {
  string s;
  $1.extract(s,utf8sid);
  if (argvi >= items) EXTEND(sp, 1);
  sv_setpvn($result = sv_newmortal(), s.data(), s.size());
  SvUTF8_on($result);
  ++argvi;
}

%typemap(out) UnicodeString {
  string s;
  UnicodeStringRef($1).extract(s,utf8sid);
  if (argvi >= items) EXTEND(sp, 1);
  sv_setpvn($result = sv_newmortal(), s.data(), s.size());
  SvUTF8_on($result);
  ++argvi;
}

%typemap(in) const UnicodeString & (UnicodeString temp) {
  STRLEN len;
  char *p = SvPVutf8($input, len);
  temp = UnicodeString(p, (int) len, "utf-8");
  $1 = &temp;
}

%typemap(out) vector<Feature> {
  size_t len = $1.size();
  SV **svs = new SV*[len];
  for (size_t i=0; i<len; i++) {
    svs[i] = sv_newmortal();
    sv_setsv(svs[i],
      SWIG_NewPointerObj(new Feature((($1_type &)$1)[i]),
         $descriptor(Feature *), 1));
  }
  AV *myav = av_make(len,svs);
  delete []svs;
  $result = newRV_noinc((SV*) myav);
  sv_2mortal($result);
  argvi++;
}

%insert(perl) {
# these are default methods that the user overrides, they
# are here to prevent errors if they are not defined
package main;
sub initialize {};
sub typeSystemInit {};
sub destroy {};
sub process {};
sub reconfigure {};
sub batchProcessComplete {};
sub collectionProcessComplete {};
}
#endif

#ifdef SWIGTCL
%typemap(out) UnicodeStringRef {
  string s;
  $1.extract(s,utf8sid);
  Tcl_SetStringObj($result, s.data(), s.size());
}

%typemap(out) UnicodeString {
  string s;
  UnicodeStringRef($1).extract(s,utf8sid);
  Tcl_SetStringObj($result, s.data(), s.size());
}

%typemap(in) const UnicodeString & (UnicodeString temp) {
  int len;
  char *p = Tcl_GetStringFromObj($input, &len);
  temp = UnicodeString(p, len, "utf-8");
  $1 = &temp;
}

%typemap(out) vector<Feature> {
  size_t len = $1.size();
  for (size_t i=0; i<len; i++) {
    Tcl_ListObjAppendElement(interp, $result,
      SWIG_NewPointerObj(new Feature((($1_type &)$1)[i]),
         $descriptor(Feature *), 1));
  }
}

#endif

class Type {
public:
  UnicodeStringRef getName() const;
  bool isValid() const;
  Feature getFeatureByBaseName(UnicodeString const &feature);
  %extend {
    vector<Feature> getAppropriateFeatures() const {
      vector<Feature> v;
      self->getAppropriateFeatures(v);
      return v;
    }
  }
};

class Feature {
public:
  bool isValid() const;
  UnicodeStringRef getName() const;
  %extend {
    Type getRangeType() {
      Type t;
      self->getRangeType(t);
      return t;
    }
    Type getIntroType() {
      Type t;
      self->getIntroType(t);
      return t;
    }
  }
};

%nodefault TypeSystem;
class TypeSystem {
public:
  Type getType(UnicodeString const &crName);
};

%nodefault CAS;
class CAS {
public:
  FSIterator iterator();
  FSIndexRepository &getIndexRepository() throw (CASException);
  FeatureStructure createFS(Type const &) throw (CASException);
  TypeSystem const &getTypeSystem() const;
  ANIndex getAnnotationIndex(Type const &type);
  void setDocumentText(UnicodeStringRef const &text) throw (CASException);
  UnicodeStringRef getDocumentText() const;
  void setSofaDataString(UnicodeStringRef const &text, UnicodeString const &mimetype) throw (CASException);
  void setSofaDataArray(FeatureStructure array, UnicodeString const &mime) throw (CASException);
  void setSofaDataURI(UnicodeString const &uri, UnicodeString const &mime) throw (CASException);
  UnicodeStringRef getSofaDataURI();
  AnnotationFS createAnnotation(Type const &type, size_t uiBeginPos, size_t uiEndPos) throw (CASException);
  SofaFS getSofa();
//Deprecated  SofaFS createSofa(const char* sofaName, const char* mimeType);
  CAS* getView(const char* viewName) throw (CASException);
  CAS* createView(const char* sofaName) throw (CASException);
  %extend {
    SofaFS getSofa(AnnotatorContext &ac, const UnicodeString &sofaName) {
      return self->getSofa(ac.mapToSofaID(sofaName));
    }
  }
};


class FeatureStructure {
public:
  CAS &getCAS();
  bool isValid() const;
  Type getType() const;
  FeatureStructure clone();
  FeatureStructure clone(Type const &t);
  bool isUntouchedFSValue(Feature const &crFeature) const;
  FeatureStructure getFSValue(Feature const &crFeature) const;
  void setFSValue(Feature const &crFeature, FeatureStructure const &anFS);
  int getIntValue(Feature const &crFeature) const;
  void setIntValue(Feature const &crFeature, int i);
  float getFloatValue(Feature const &crFeature) const;
  void setFloatValue(Feature const &crFeature, float);
  UnicodeStringRef getStringValue(Feature const &crFeature) const;
  void setStringValue(Feature const &crFeature, UnicodeString const &cuStr);
};

class AnnotationFS : public FeatureStructure {
public:
  size_t getBeginPosition() const;
  size_t getEndPosition() const;
  size_t getLength() const;
  UnicodeStringRef getCoveredText() const throw (CASException);
};

class SofaFS : public FeatureStructure {
public:
  UnicodeStringRef getSofaURI();
  UnicodeStringRef getSofaMime();
};

%nodefault FSIndexRepository;
class FSIndexRepository {
public:
  FSIndex getIndex(UnicodeString const &crLabel);
  /* As of SWIG 1.3.29, function overloading is broken for typemaps */
  /* See SWIG bugzilla http://sourceforge.net/tracker/index.php?func=detail&aid=1465952&group_id=1645&atid=101645 */
  %extend {
    FSIndex getIndexByType(UnicodeString const &crLabel, const Type &type)
    {
      return self->getIndex(crLabel,type);
    }
  }
  
  void addFS(FeatureStructure const &fs);
};


class FSIterator {
public:
  bool isValid();
  FeatureStructure get() const;
  void moveToNext();
  void moveToPrevious();
  void moveToFirst();
  void moveToLast();
  void moveTo(FeatureStructure fs);
  FeatureStructure peekNext() const;
  FeatureStructure peekPrevious() const;
};

%nodefault ANIterator;
class ANIterator : public FSIterator {
public:
  AnnotationFS get() const;
};

%nodefault ResultSpecification;
class ResultSpecification {
public:
};

%nodefault AnnotatorContext;
class AnnotatorContext {
public:
  %extend {
    UnicodeString extractValue(const UnicodeString &paramName) {
      UnicodeString v;
      TyErrorId e = self->extractValue(paramName, v);
      if (e != UIMA_ERR_NONE) {
        throw new runtime_error(
           TextAnalysisEngine::getErrorIdAsCString(e));
      }
      return v;
    }
    UnicodeString extractStringValue(const UnicodeString &paramName) {
      UnicodeString v;
      TyErrorId e = self->extractValue(paramName, v);
      if (e != UIMA_ERR_NONE) {
        throw new runtime_error(
           TextAnalysisEngine::getErrorIdAsCString(e));
      }
      return v;
    }
    int extractIntegerValue(const UnicodeString &paramName) {
      int v;
      TyErrorId e = self->extractValue(paramName, v);
      if (e != UIMA_ERR_NONE) {
        throw new runtime_error(
           TextAnalysisEngine::getErrorIdAsCString(e));
      }
      return v;
    }
    float extractFloatValue(const UnicodeString &paramName) {
      float v;
      TyErrorId e = self->extractValue(paramName, v);
      if (e != UIMA_ERR_NONE) {
        throw new runtime_error(
           TextAnalysisEngine::getErrorIdAsCString(e));
      }
      return v;
    }
    void logMessage(const UnicodeString &message) {
      self->getLogger().logMessage(message);
    }
    void logWarning(const UnicodeString &message) {
      self->getLogger().logWarning(message);
    }
    void logError(const UnicodeString &message) {
      self->getLogger().logError(message);
    }
  }
};

class FSIndex {
public:
  bool isValid() const;
  size_t getSize() const;
  FeatureStructure find(FeatureStructure &anFS) const;
  FSIterator iterator();
};

class ANIndex : public FSIndex {
public:
  ANIterator iterator();
};

#ifdef SWIGPERL
#endif

