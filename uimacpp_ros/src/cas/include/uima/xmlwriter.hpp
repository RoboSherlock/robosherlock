#ifndef UIMA_XMLWRITER_HPP
#define UIMA_XMLWRITER_HPP
/** \file xmlwriter.hpp .
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

   \brief Used to output the CAS in XCAS format

-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp>

#include <iostream>
#include <vector>
#include <set>
#include <map>

#include <uima/featurestructure.hpp>
#include <uima/caswriter_abase.hpp>
#include <uima/lowlevel_typedefs.hpp>
#include <uima/typesystem.hpp>


/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
  class CAS;
  namespace internal {
    class CASImpl;
  }
  namespace lowlevel {
    class IndexABase;
    class FSHeap;
    class TypeSystem;
  }
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/**
 * This file contains some useful CASWriters, one is used in the dump annotator,
 * the other one can be used to output the CAS in XCAS format.
 */
namespace uima {

/**
  class UIMA_LINK_IMPORTSPEC XMLWriterABase : public CASWriterABase {
  protected:
    void normalize(UnicodeStringRef const & in, icu::UnicodeString& out) const;
  public:
    XMLWriterABase(CAS const & crCAS, bool bAddDocBuffer);
    virtual ~XMLWriterABase();

    virtual void write(ostream& os) = 0;
  };
**/


  class UIMA_LINK_IMPORTSPEC XMLDumpWriter : public XMLWriterABase {
  private:
    uima::internal::CASImpl const & iv_crCAS;
    lowlevel::FSHeap const & iv_rFSHeap;
    lowlevel::TypeSystem const & iv_rTypeSystem;

    lowlevel::TyFSType iv_tyAnnotationType;
    lowlevel::TyFSType iv_tyIntegerType;
    lowlevel::TyFSType iv_tyStringType;
    lowlevel::TyFSType iv_tyFloatType;
    lowlevel::TyFSType iv_tyArrayType;
    lowlevel::TyFSType iv_tyStringArrayType;
    lowlevel::TyFSType iv_tyIntArrayType;
    lowlevel::TyFSType iv_tyFloatArrayType;
    lowlevel::TyFSType iv_tyListType;

    lowlevel::TyFSFeature iv_tyBeginPosFeature;
    lowlevel::TyFSFeature iv_tyEndPosFeature;

    icu::UnicodeString getTab(int tab) const;
    void writeFS(int tab, std::ostream& os, lowlevel::TyFS, bool bWriteShortStyle) const;
    void writeArray(int iTab, std::ostream& os, lowlevel::TyFSFeature & tyFeat, lowlevel::TyFS & tyValue, lowlevel::TyFSType tyRange, icu::UnicodeString & newTab) const;
  public:
    XMLDumpWriter(CAS const & crCAS, bool bAddDocBuffer);
    ~XMLDumpWriter();

    virtual void write(std::ostream& os);
  };



  class UIMA_LINK_IMPORTSPEC XCASWriter : public XMLWriterABase {
  private:
    Type iv_stringType;
    Type iv_integerType;
    Type iv_floatType;
    Type iv_byteType;
    Type iv_booleanType;
    Type iv_shortType;
    Type iv_longType;
    Type iv_doubleType;
    Type iv_arrayType;
    Type iv_intArrayType;
    Type iv_floatArrayType;
    Type iv_stringArrayType;
    Type iv_byteArrayType;
    Type iv_booleanArrayType;
    Type iv_shortArrayType;
    Type iv_longArrayType;
    Type iv_doubleArrayType;

    Type iv_sofaType;
    bool isBCCas;
    std::map<int, std::vector<int>*> enqueuedFS;

    void writeFeatureValue(std::ostream & os, FeatureStructure const & fs, Feature const & f);
    void writeFSFlat(std::ostream & os, FeatureStructure const & fs, std::vector<int>* indexInfo);
    void findReferencedFSs(FeatureStructure const & fs, bool check=true);
    bool isReferenceType(Type const & t) const;
    bool enqueueIndexed(FeatureStructure const &fs, int sofa);
    bool enqueueUnindexed(FeatureStructure const &fs);
		void writeStringArray(std::ostream & os, StringArrayFS const & array, char const * tag);
  public:
    XCASWriter(CAS const & crCAS, bool bAddDocBuffer);
    ~XCASWriter();

    virtual void write(std::ostream& os);
  };

}


/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif
