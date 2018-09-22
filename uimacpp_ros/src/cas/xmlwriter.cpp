/** \file xmlwriter.cpp .
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
#include <uima/pragmas.hpp>

#include <uima/xmlwriter.hpp>
#include <uima/arrayfs.hpp>
#include <uima/lowlevel_indexrepository.hpp>
#include <uima/lowlevel_indexiterator.hpp>
#include <uima/internal_casimpl.hpp>
#include <uima/internal_fspromoter.hpp>
#include <uima/fsindexrepository.hpp>

#include <uima/resmgr.hpp>
#include <uima/location.hpp>
#include "xercesc/framework/XMLFormatter.hpp"
#include "xercesc/framework/MemBufFormatTarget.hpp"
#include "xercesc/util/XMLString.hpp"
XERCES_CPP_NAMESPACE_USE
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */
#define TAB_INCREMENT 0

char const * gs_cpszFSTagName = "fs";
char const * gs_cpszFeatTagName = "feat";
char const * gs_cpszXslFileName = "fsdump.xsl";
int sofa;

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */
using namespace std;
namespace uima {
  XMLWriterABase::XMLWriterABase(CAS const & crCAS, bool bAddDocBuffer)
      : CASWriterABase(crCAS, bAddDocBuffer) {}

  XMLWriterABase::~XMLWriterABase() {}

  void XMLWriterABase::normalize(UnicodeStringRef const & in, icu::UnicodeString& out)  {
    //cout << "normalize() input " << in << endl;
    static icu::UnicodeString const ustrAMP_ESC("&amp;");
    static icu::UnicodeString const ustrGT_ESC("&gt;");
    static icu::UnicodeString const ustrLT_ESC("&lt;");
    static icu::UnicodeString const ustrAPOS_ESC("&apos;");
    static icu::UnicodeString const ustrQUOT_ESC("&quot;");
    static icu::UnicodeString const ustrCR_ESC("&#13;");
    static icu::UnicodeString const ustrLF_ESC("&#10;");

    static UChar const uAMP('&');
    static UChar const uGT('>');
    static UChar const uLT('<');
    static UChar const uAPOS('\'');
    static UChar const uQUOT('"');
    static UChar const uCR('\r');
    static UChar const uLF('\n');
    
    const UChar * srcPtr = in.getBuffer();

    for (int i=0; i < in.length(); i++) {
      switch (*srcPtr) {
        case uAMP: 
          out.append(ustrAMP_ESC);
          break;
        case uGT: 
          out.append(ustrGT_ESC);
          break;
        case uLT: 
          out.append(ustrLT_ESC);
          break;
        case uAPOS: 
          out.append(ustrAPOS_ESC);
          break;
        case uQUOT: 
          out.append(ustrQUOT_ESC);
          break;
        case uCR: 
          out.append(ustrCR_ESC);
          break;
        case uLF: 
          out.append(ustrLF_ESC);
          break;
        default:
          out.append(*srcPtr);
      }
      srcPtr++;
    }
  }


  /////////////////////////////////////////////////////////////////////////////
  // DumpWriter

  XMLDumpWriter::XMLDumpWriter(CAS const & crCAS,
    bool bAddDocBuffer)
    : XMLWriterABase(crCAS, bAddDocBuffer),
    iv_crCAS( ((uima::internal::CASImpl const &) crCAS) ),
    iv_rFSHeap( iv_crCAS.getHeap() ),
    iv_rTypeSystem( iv_rFSHeap.getTypeSystem() ),
    iv_tyAnnotationType( uima::internal::gs_tyAnnotationType),
    iv_tyIntegerType(uima::internal::gs_tyIntegerType),
    iv_tyStringType(uima::internal::gs_tyStringType),
    iv_tyFloatType(uima::internal::gs_tyFloatType),
    iv_tyArrayType(uima::internal::gs_tyArrayBaseType),
    iv_tyStringArrayType(uima::internal::gs_tyStringArrayType),
    iv_tyIntArrayType(uima::internal::gs_tyIntArrayType),
    iv_tyFloatArrayType(uima::internal::gs_tyFloatArrayType),
    iv_tyListType(uima::internal::gs_tyFSListType),
    iv_tyBeginPosFeature(uima::internal::gs_tyBeginPosFeature),
    iv_tyEndPosFeature(uima::internal::gs_tyEndPosFeature) {
    }

  XMLDumpWriter::~XMLDumpWriter() {}

  icu::UnicodeString XMLDumpWriter::getTab(int iTab) const {
    icu::UnicodeString usResult;
    icu::UnicodeString usSpace(" ");
    int i;
    for (i=0; i<iTab; ++i) {
      usResult.append(usSpace);
    }
    return usResult;
  }



  void XMLDumpWriter::writeFS(int iTab, ostream& os, lowlevel::TyFS tyFS, bool bWriteShortStyle=false) const {
    icu::UnicodeString usTab = getTab(iTab);
    lowlevel::TyFSType tyFSType = iv_rFSHeap.getType(tyFS);

    os << usTab << "<" << gs_cpszFSTagName << " type=\"" << iv_rTypeSystem.getTypeName(tyFSType) << "\"";
    os << " id=\"id" << (long)iv_rFSHeap.getUniqueID(tyFS) << "\"";

    uima::lowlevel::IndexRepository const & crIxRep = iv_crCAS.getIndexRepository();

    //if a feature value-FS is not in any index, e.g. a List FS,
    //it will be dumped in its expanded
    //form (i.e. with all features)
    if (bWriteShortStyle && crIxRep.contains(tyFS)) {
      os << "/>" << endl;
      return;
    }

    os << ">" <<  endl;

    if ( iv_addDocument && iv_rTypeSystem.subsumes( iv_tyAnnotationType, tyFSType ) ) {
      AnnotationFS fs(internal::FSPromoter::promoteFS(tyFS, iv_crCAS ));
      UnicodeStringRef ulsSpan = fs.getCoveredText();

      icu::UnicodeString usNormalizedSpan;
      normalize(ulsSpan, usNormalizedSpan);
      os << "<annotation_span>" << usNormalizedSpan << "</annotation_span>" << endl;
    }

    /*          TBD: when can this happen ? */
    if (tyFSType == iv_tyStringType) {
      UnicodeStringRef ulsString = iv_rFSHeap.getFSAsString(tyFS);
      icu::UnicodeString usNormalizedString;
      normalize(ulsString, usNormalizedString);

      os << "<string_value>" << usNormalizedString << "</string_value>" << endl;
      os << "</" << gs_cpszFSTagName << ">" << endl;
      return;
    }

    vector<lowlevel::TyFSFeature> vecFeatures;
    iv_rTypeSystem.getAppropriateFeatures( tyFSType, vecFeatures );
    size_t i;
    for (i=0; i<vecFeatures.size(); ++i) {
      lowlevel::TyFSFeature tyFeat = vecFeatures[i];
      lowlevel::TyFeatureOffset tyOffset = iv_rTypeSystem.getFeatureOffset( tyFeat );
      lowlevel::TyFSType tyRange = iv_rTypeSystem.getRangeType( tyFeat );
      lowlevel::TyFS tyValue = iv_rFSHeap.getFeatureWithOffset( tyFS, tyOffset );

      icu::UnicodeString newTab = getTab(iTab + TAB_INCREMENT);
      if (tyRange == iv_tyIntegerType) {
        os << newTab << "<" << gs_cpszFeatTagName << " name=\"" << iv_rTypeSystem.getFeatureName( tyFeat ) << "\"";
        os << " value=\"" << lowlevel::FSHeap::getFSAsInt(tyValue) << "\" />" << endl;
      } else if (tyRange == iv_tyFloatType) {
        os << newTab << "<" << gs_cpszFeatTagName << " name=\"" << iv_rTypeSystem.getFeatureName( tyFeat ) << "\"";
        os << " value=\"" << lowlevel::FSHeap::getFSAsFloat(tyValue) << "\" />" << endl;
      } else if (tyRange == iv_tyStringType) {
        if (tyValue != 0) {
          UnicodeStringRef  ulsString = iv_rFSHeap.getFSAsString(tyValue);
          icu::UnicodeString usNormalized;
          normalize(ulsString, usNormalized);
          os << newTab << "<" << gs_cpszFeatTagName <<" name=\"" << iv_rTypeSystem.getFeatureName( tyFeat ) << "\"";
          os << " value=\"" << usNormalized << "\" />" << endl;
        }
      } else if (iv_rTypeSystem.subsumes(iv_tyArrayType, tyRange)) {
        writeArray(iTab, os, tyFeat, tyValue, tyRange, newTab);
      } else {
        // "normal" FS
        if (tyValue != 0) {
          os << newTab << "<" << gs_cpszFeatTagName << " name=\"" << iv_rTypeSystem.getFeatureName( tyFeat ) << "\"";
          os << ">" << endl;
          writeFS(iTab + (2*TAB_INCREMENT), os, tyValue, true);
          os << newTab << "</"<< gs_cpszFeatTagName << ">" << endl;
        }
      }

    }
    os << usTab << "</"<< gs_cpszFSTagName << ">" << endl;
  }


  void XMLDumpWriter::writeArray(int iTab, ostream& os, lowlevel::TyFSFeature & tyFeat, lowlevel::TyFS & tyValue, lowlevel::TyFSType tyRange, icu::UnicodeString & newTab) const {
    if (tyValue != 0) {
      os << newTab << "<"<< gs_cpszFeatTagName<<" name=\"" << iv_rTypeSystem.getFeatureName( tyFeat ) << "\"";
      os << ">" << endl;
      icu::UnicodeString uiArrayTab = getTab( iTab + (2*TAB_INCREMENT) );
      icu::UnicodeString uiArrayValTab = getTab( iTab + (3*TAB_INCREMENT) );
      int iArraySize = iv_rFSHeap.getArraySize( tyValue );
      lowlevel::TyHeapCell const * ptyCArray = iv_rFSHeap.getCArrayFromFS( tyValue);
      os << uiArrayTab << "<array type=\""<< iv_rTypeSystem.getTypeName(tyRange) << "\" size=\"" << iArraySize << "\">" << endl;
      int j;
      if (iv_rTypeSystem.subsumes(iv_tyIntArrayType, tyRange)) {
        for (j=0; j < iArraySize; ++j) {
          os << uiArrayTab << "<array_element>" << endl;
          os << uiArrayValTab << "<int_value>" << lowlevel::FSHeap::getFSAsInt((lowlevel::TyFS)ptyCArray[j]) << "</int_value>" << endl;
          os << uiArrayTab << "</array_element>" << endl;
        }
      } else if (iv_rTypeSystem.subsumes(iv_tyFloatArrayType, tyRange)) {
        for (j=0; j < iArraySize; ++j) {
          os << uiArrayTab << "<array_element>" << endl;
          os << uiArrayValTab << "<float_value>" << lowlevel::FSHeap::getFSAsFloat((lowlevel::TyFS)ptyCArray[j]) << "</float_value>" << endl;
          os << uiArrayTab << "</array_element>" << endl;
        }
      } else if (iv_rTypeSystem.subsumes(iv_tyStringArrayType, tyRange)) {
        for (j=0; j < iArraySize; ++j) {
          os << uiArrayTab << "<array_element>" << endl;
          UnicodeStringRef ulsString = iv_rFSHeap.getFSAsString((lowlevel::TyFS)ptyCArray[j]);
          icu::UnicodeString usNormalizedString;
          normalize(ulsString, usNormalizedString);
          os << uiArrayValTab << "<string_value>" << usNormalizedString << "</string_value>" << endl;
          os << uiArrayTab << "</array_element>" << endl;
        }
      } else { // FSArray
        for (j=0; j < iArraySize; ++j) {
          os << uiArrayTab << "<array_element>" << endl;
          writeFS(uiArrayValTab.length() , os, (lowlevel::TyFS) ptyCArray[j], /*rFoundFSs,*/ true );
          os << uiArrayTab << "</array_element>" << endl;
        }
      }
      os << uiArrayTab << "</array>" << endl;
      os << newTab << "</" << gs_cpszFeatTagName << ">" << endl;
    }
  }

  void XMLDumpWriter::write(ostream& os) {
    util::Location const & xslFileLoc = ResourceManager::getInstance().getLocationData();

    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << endl;
    os << "<?xml-stylesheet type=\"text/xsl\" href=\"file:///" << xslFileLoc.getAsCString() << gs_cpszXslFileName << "\" ?>" << endl;
    os << "<taf>" << endl;

    if (iv_addDocument) {
      UnicodeStringRef ulstrBuffer(iv_crCAS.getDocumentText());

      os << "<doc_buffer>" << endl;
      os << "<![CDATA[";
      std::string s;
      ulstrBuffer.extract(s, CCSID::getDefaultSBCSInputCCSID() );
      os << s;
      os << "]]>" << endl;
      os << "</doc_buffer>" << endl;
    }

    os << "<feature_structures>" << endl;

    uima::lowlevel::IndexRepository const & crIxRep = iv_crCAS.getIndexRepository();
    uima::lowlevel::IndexDefinition const & indexDef = crIxRep.getIndexDefinition();

    vector<icu::UnicodeString> vIndexIDs;

    indexDef.getAllIndexIDs(vIndexIDs);

    icu::UnicodeString const & crAnnIndexID = iv_crCAS.getAnnotationIndexID();

    vector<icu::UnicodeString>::iterator it;
    for (it = vIndexIDs.begin(); it != vIndexIDs.end(); ++it) {
      if (*it == crAnnIndexID) {
        break;
      }
    }
    vIndexIDs.erase(it);
    vIndexIDs.insert(vIndexIDs.begin(), crAnnIndexID);

    assert( vIndexIDs.front() == crAnnIndexID );

    size_t i;
    for (i=0; i<vIndexIDs.size(); ++i) {

      icu::UnicodeString const & crIndexID = vIndexIDs[i];

      icu::UnicodeString const & crTypeName = iv_rTypeSystem.getTypeName( indexDef.getTypeForIndex(crIndexID) );
      UnicodeStringRef ulsTypeName(crTypeName.getBuffer(), crTypeName.length() );
      os << "<index id=\"" << crIndexID << "\" type=\"" << ulsTypeName << "\">" << endl;

      uima::lowlevel::TyFSType tyIndexType = indexDef.getTypeForIndex(crIndexID);
      lowlevel::IndexABase const & crIndex = crIxRep.getLowlevelIndex(crIndexID, tyIndexType);
      auto_ptr<uima::lowlevel::IndexIterator> pIterator(crIndex.createIterator());
      for (pIterator->moveToFirst(); pIterator->isValid(); pIterator->moveToNext()) {
        lowlevel::TyFS tyFS = pIterator->get();
        writeFS(0, os, tyFS);
      }
      os << "</index>" << endl;
    }
    os << "</feature_structures>" << endl;
    os << "</taf>" << endl;
  }


  /*         Currently not enabled. Problem: There can be more than one index for a given */
  /*            type. We don't want to spend the extra effort required to specify this    */
  /*            in the configuration file unless it is explicitly requested.              */
  /*         void XMLDumpWriter::setTypesToBeShown(vector<uima::Type> const & crTypeSet) {              */
  /*             iv_setTypesToBeShown.clear();                                                     */
  /*                                                                                               */
  /*             vector<uima::Type>::const_iterator cit;                                            */
  /*             for (cit = crTypeSet.begin(); cit != crTypeSet.end(); ++cit) {                    */
  /*                 uima::lowlevel::TyFSType tyType = uima::internal::FSPromoter::demoteType(*cit); */
  /*                 assert( iv_rTypeSystem.isValidType( tyType ) );                               */
  /*                 iv_setTypesToBeShown.insert(tyType);                                          */
  /*             }                                                                                 */
  /*                                                                                               */
  /*         }                                                                                     */


  ////////////////////////////////////////////////////////////////////////
  // XCASWriter

  XCASWriter::XCASWriter(CAS const & cas, bool addDocument)
      : XMLWriterABase(cas, addDocument),
      iv_stringType( cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_STRING) ),
      iv_integerType( cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_INTEGER) ),
      iv_floatType( cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_FLOAT) ),
      iv_byteType( cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_BYTE) ),
      iv_booleanType( cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_BOOLEAN) ),
      iv_shortType( cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_SHORT) ),
      iv_longType( cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_LONG) ),
      iv_doubleType( cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_DOUBLE) ),
      iv_arrayType( cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_ARRAY_BASE) ),
      iv_stringArrayType( cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_STRING_ARRAY) ),
      iv_intArrayType( cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_INTEGER_ARRAY) ),
      iv_floatArrayType( cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_FLOAT_ARRAY) ),
      iv_byteArrayType( cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_BYTE_ARRAY) ),
      iv_booleanArrayType( cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_BOOLEAN_ARRAY) ),
      iv_shortArrayType( cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_SHORT_ARRAY) ),
      iv_longArrayType( cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_LONG_ARRAY) ),
      iv_doubleArrayType( cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_DOUBLE_ARRAY) ),
      iv_sofaType( cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_SOFA) ) {
    assert( iv_stringType.isValid());
    assert( iv_integerType.isValid());
    assert( iv_floatType.isValid());
    assert( iv_byteType.isValid());
    assert( iv_booleanType.isValid());
    assert( iv_shortType.isValid());
    assert( iv_longType.isValid());
    assert( iv_doubleType.isValid());
    assert( iv_stringArrayType.isValid() );
    assert( iv_arrayType.isValid() );
    assert( iv_intArrayType.isValid());
    assert( iv_floatArrayType.isValid());
    assert( iv_byteArrayType.isValid());
    assert( iv_booleanArrayType.isValid());
    assert( iv_shortArrayType.isValid());
    assert( iv_longArrayType.isValid());
    assert( iv_doubleArrayType.isValid());
    assert( iv_sofaType.isValid());
  }

  XCASWriter::~XCASWriter() {
    for (size_t i = 0; i < enqueuedFS.size(); i++) {
      vector<int> * indexes = (vector<int>*) enqueuedFS[i];
      if (indexes != 0)
        delete indexes;
    }
  }


  bool XCASWriter::isReferenceType(Type const & t) const {
    return !( t.getTypeSystem().isPrimitive(uima::internal::FSPromoter::demoteType(t)) );
  }

  void XCASWriter::writeFeatureValue(ostream & os, FeatureStructure const & fs, Feature const & f) {
    assert( fs.isValid() );
    assert( f.isValid() );
    Type t;
    f.getRangeType(t);
    assert( t.isValid() );
    if ( t == iv_stringType || t.isStringSubType() ) {
      UnicodeStringRef ref = fs.getStringValue(f);
      if (ref.getBuffer() != NULL) {
        icu::UnicodeString us;
        normalize( ref, us );
        os << " " << f.getName() << "=\"";
        os << us << "\"";
      }
    } else if (t == iv_integerType) {
      os << " " << f.getName() << "=\"";
      os << fs.getIntValue(f) << "\"";
    } else if (t == iv_floatType) {
      os << " " << f.getName() << "=\"";
      os << fs.getFloatValue(f) << "\"";
    } else if (t == iv_byteType) {
      os << " " << f.getName() << "=\"";
      int val = fs.getByteValue(f);
      os << val << "\"";
    } else if (t == iv_booleanType) {
      os << " " << f.getName() << "=\"";
      os << fs.getBooleanValue(f) << "\"";
    } else if (t == iv_shortType) {
      os << " " << f.getName() << "=\"";
      os << fs.getShortValue(f) << "\"";
    } else if (t == iv_longType) {
      os << " " << f.getName() << "=\"";
      os << fs.getLongValue(f) << "\"";
    } else if (t == iv_doubleType) {
      os << " " << f.getName() << "=\"";
      stringstream s;
      s << fs.getDoubleValue(f);
      os << s.str() << "\"";
    } else {

      FeatureStructure referencedFS = fs.getFSValue(f);
      uima::lowlevel::TyFS lolFS = uima::internal::FSPromoter::demoteFS(referencedFS);
      if (lolFS != uima::lowlevel::FSHeap::INVALID_FS) {
        os << " " << "_ref_";
        os << f.getName() << "=\"";
        ptrdiff_t val = uima::internal::CASImpl::promoteCAS(iv_cas).getHeap().getUniqueID(lolFS);
        os << val << "\"";
      }
    }
  }

  template<class Array>
  void writeArray(ostream & os, Array const & array, char const * tag) {
    size_t i;
    if (array.size() > 0) {
      os << " size=\"" << array.size() << "\">" << endl;
      for (i=0; i<array.size(); ++i) {
        os << "  <" << tag << ">";
        os << array.get(i);
        os << "</" << tag << ">" << endl;
      }
      os << " </" << array.getType().getName() << ">" << endl;
    } else {
      os << " size=\"0\"/>" << endl;
    }
  }

	void XCASWriter::writeStringArray(ostream & os, StringArrayFS const & array, char const * tag) {
    size_t i;
		UnicodeString ustr;
    if (array.size() > 0) {
      os << " size=\"" << array.size() << "\">" << endl;
      for (i=0; i<array.size(); ++i) {
        os << "  <" << tag << ">";
        ustr.setTo("");
				normalize(array.get(i),ustr);
        os << ustr;
        os << "</" << tag << ">" << endl;
      }
      os << " </" << array.getType().getName() << ">" << endl;
    } else {
      os << " size=\"0\"/>" << endl;
    }
  }

  void XCASWriter::writeFSFlat(ostream & os,
                               FeatureStructure const & fs,
                               vector<int>* indexInfo) {
    uima::internal::CASImpl const & crCASImpl = uima::internal::CASImpl::promoteCAS(iv_cas);
    assert( fs.isValid() );
    Type t = fs.getType();
//       if ( iv_sofaType == t) {
//  if ( isBCCas )
//    return;
//       }
    os << " <" << t.getName();
    if (indexInfo->size() > 0) {
      os << " _indexed=\"";
      for (size_t i=0; i< indexInfo->size(); i++) {
        if (i != 0) {
          os << " ";
        }
        os << indexInfo->at(i);
      }
      os << "\"";
    }
    os << " _id=\"" << crCASImpl.getHeap().getUniqueID( uima::internal::FSPromoter::demoteFS(fs) ) << "\"";

    // if array
    if ( iv_arrayType.subsumes(t) ) {
      if ( t == iv_intArrayType ) {
        writeArray( os, IntArrayFS(fs), "i");
      } else if ( t == iv_floatArrayType ) {
        writeArray( os, FloatArrayFS(fs), "i");
      } else if ( t == iv_stringArrayType ) {
        writeStringArray( os, StringArrayFS(fs), "i");
      } else if ( t == iv_byteArrayType ) {
        ByteArrayFS array(fs);
        if (array.size() > 0) {
          os << " size=\"" << array.size() << "\">" << endl;
          size_t i;
          for (i=0; i<array.size(); ++i) {
            os << "  <" << "i" << ">";
            int val = array.get(i);
            os << val;
            os << "</" << "i" << ">" << endl;
          }
          os << " </" << array.getType().getName() << ">" << endl;
        }
      } else if ( t == iv_booleanArrayType ) {
        writeArray( os, BooleanArrayFS(fs), "i");
      } else if ( t == iv_shortArrayType ) {
        writeArray( os, ShortArrayFS(fs), "i");
      } else if ( t == iv_longArrayType ) {
        writeArray( os, LongArrayFS(fs), "i");
      } else if ( t == iv_doubleArrayType ) {
        DoubleArrayFS array(fs);
        if (array.size() > 0) {
          os << " size=\"" << array.size() << "\">" << endl;
          size_t i;
          for (i=0; i<array.size(); ++i) {
            os << "  <" << "i" << ">";
            stringstream s;
            s << array.get(i);
            os << s.str();
            os << "</" << "i" << ">" << endl;
          }
          os << " </" << array.getType().getName() << ">" << endl;
        } else {
          os << " size=\"0\"/>" << endl;
        }
      } else {
        assert( t == iv_cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_FS_ARRAY) );
        ArrayFS arrayFS = (ArrayFS) fs;
        if (arrayFS.size() > 0) {
          os << " size=\"" << arrayFS.size() << "\">" << endl;
          size_t i;
          for (i=0; i<arrayFS.size(); ++i) {
            os << " <" << "i" << ">";
            os << uima::internal::FSPromoter::demoteFS(arrayFS.get(i));
            //writeFSFlat(os, arrayFS.get(i), sofa);
            os << "</" << "i" << ">" << endl;
          }
          os << "</" << t.getName() << ">" << endl;
        } else {
          os << " size=\"0\"/>" << endl;
        }
      }
    } else {
      vector<Feature> features;
      fs.getType().getAppropriateFeatures(features);
      size_t i;
      for (i=0; i<features.size(); ++i) {
        uima::Feature const & f = features[i];
        Type range;
        f.getRangeType(range);
        writeFeatureValue(os, fs, f);
      }
      os << "/>" << endl;
    }
  }


  void XCASWriter::findReferencedFSs(FeatureStructure const & fs, bool check) {
    if (! fs.isValid() ) {
      return;
    }
    if (check) {
      if (! enqueueUnindexed(fs)) {
        // already been processed
        return;
      }
    }
    Type t = fs.getType();
    assert( t.isValid() );
    size_t i;
    if ( iv_arrayType.subsumes(t) ) {
      if ( t == iv_cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_FS_ARRAY) ) {
        ArrayFS array(fs);
        for (i=0; i<array.size(); ++i) {
          findReferencedFSs(array.get(i));
        }
      }
    } else {
      vector<Feature> features;
      fs.getType().getAppropriateFeatures(features);

      for (i=0; i<features.size(); ++i) {
        uima::Feature const & f = features[i];
        Type range;
        f.getRangeType(range);
        if (isReferenceType(range)) {
          findReferencedFSs(fs.getFSValue(f));
        }
      }
    }
  }

  void XCASWriter::write(ostream & os) {
    const uima::lowlevel::IndexRepository * ixRep;
    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << endl;
    os << "<CAS>" << endl;

    const CAS* ccasp = &iv_cas;
    CAS* casp = const_cast<CAS*> (ccasp);
    CAS* tcas;
    uima::internal::CASImpl const & crCASImpl = uima::internal::CASImpl::promoteCAS(*casp->getBaseCas());
    ixRep = &crCASImpl.getIndexRepository();
//       }

    int numViews = casp->getNumViews();
    set<FeatureStructure> referencedFSs;
    for (int view=0; view<=numViews; view++) {
      if (view==0) {
        // First time through is for base CAS
        sofa = 0;
      } else {
        // for all Sofa found in the CAS, get new IndexRepository
        tcas = casp->getViewBySofaNum(view);
        sofa = tcas->getSofaNum();
        uima::internal::CASImpl & crTCASImpl = uima::internal::CASImpl::promoteCAS(*tcas);
        ixRep = &crTCASImpl.getIndexRepository();
      }

      // enqueue indexed FSs in known indexes
      set<FeatureStructure> indexedFSs;
      vector<icu::UnicodeString> indexIDs = ixRep->getAllIndexIDs();
      size_t i;
      for (i=0; i<indexIDs.size(); ++i) {
        Type t = ixRep->getTypeForIndex(indexIDs[i]);
        FSIndex ix = ixRep->getIndex(indexIDs[i], t);
        FSIterator it = ix.iterator();
        for (it.moveToFirst(); it.isValid(); it.moveToNext()) {
          FeatureStructure fs = it.get();
          enqueueIndexed(fs, sofa);
        }
      }

      // enqueue the undefined index FSs
      for (size_t i=0;i < ixRep->iv_undefinedindex.size(); i++ ) {
        FeatureStructure fs = internal::FSPromoter::promoteFS(ixRep->iv_undefinedindex[i], iv_cas);
        enqueueIndexed(fs, sofa);
      }
    }

    // write out all enqueued FS
    map<int, vector<int>*>::iterator it;
    for (it = enqueuedFS.begin(); it != enqueuedFS.end(); it++) {
      FeatureStructure fs = internal::FSPromoter::promoteFS((*it).first, iv_cas);
      writeFSFlat(os, fs, (*it).second);
    }

    os << "</CAS>" << endl;
  }

  // return true if the fs was not previously enqueued
  bool XCASWriter::enqueueUnindexed(FeatureStructure const &fs) {
    vector<int> * indexes;
    size_t tyfs = uima::internal::FSPromoter::demoteFS(fs);
    indexes = enqueuedFS[tyfs];
    if (NULL != indexes) {
      return false;
    }
    // new FS, enqueue it
    indexes = new vector<int>;
    enqueuedFS[tyfs] = indexes;
    return true;
  }

  // return true if the fs was not previously enqueued
  bool XCASWriter::enqueueIndexed(FeatureStructure const &fs, int sofa) {
    vector<int> * indexes;
    size_t tyfs = uima::internal::FSPromoter::demoteFS(fs);
    indexes = enqueuedFS[tyfs];
    if (NULL != indexes) {
      for (size_t i=0; i<indexes->size(); i++) {
        if (sofa == indexes->at(i)) {
          return false;
        }
      }
      indexes->push_back(sofa);
      enqueuedFS[tyfs] = indexes;
      return false;
    }
    // new FS, enqueue it and note the indexed Sofa
    indexes = new vector<int>;
    indexes->push_back(sofa);
    enqueuedFS[tyfs] = indexes;
    // and look for references
    findReferencedFSs(fs, false);
    return true;
  }

}


/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */








