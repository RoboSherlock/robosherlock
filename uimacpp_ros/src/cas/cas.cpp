/** \file cas.cpp .
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
#include <uima/msg.h>
#include <uima/macros.h>

#include <uima/cas.hpp>
#include <uima/casdefinition.hpp>
#include <uima/internal_casimpl.hpp>
#include <uima/internal_typeshortcuts.hpp>
#include <uima/internal_fspromoter.hpp>
#include <uima/typesystem.hpp>
#include <uima/lowlevel_fsheap.hpp>
#include <uima/lowlevel_indexrepository.hpp>
#include <uima/listfs.hpp>
#include <uima/arrayfs.hpp>
#include <uima/lowlevel_typesystem.hpp>
#include <uima/lowlevel_typedefs.hpp>
#include "unicode/unistr.h"
#include <uima/fsfilterbuilder.hpp>
#include <uima/sofaid.hpp>
#include <uima/lowlevel_defaultfsiterator.hpp>
#include <uima/annotator_context.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

#define UIMA_CAS_NAMESPACE "uima" UIMA_NAMESPACE_SEPARATOR "cas"
#define UIMA_CAS_PFX UIMA_CAS_NAMESPACE UIMA_NAMESPACE_SEPARATOR
#define UIMA_TCAS_NAMESPACE "uima" UIMA_NAMESPACE_SEPARATOR "tcas"
#define UIMA_TCAS_PFX UIMA_TCAS_NAMESPACE UIMA_NAMESPACE_SEPARATOR

#define ANNOTATION UIMA_TCAS_PFX "Annotation"
#define DOCUMENTANNOTATION UIMA_TCAS_PFX "DocumentAnnotation"
#define SOFA "sofa"
#define BEGIN "begin"
#define END "end"
#define LANGUAGE "language"
using namespace std;
namespace uima {
  icu::UnicodeString const CAS::ustrCREATOR_ID_CAS(lowlevel::TypeSystem::ustrCREATOR_ID_SYSTEM);

  char const * CAS::NAME_SPACE_UIMA_CAS = UIMA_CAS_NAMESPACE;

  char const * CAS::TYPE_NAME_TOP              = UIMA_CAS_PFX "TOP";
  char const * CAS::TYPE_NAME_INTEGER          = UIMA_CAS_PFX "Integer";
  char const * CAS::TYPE_NAME_STRING           = UIMA_CAS_PFX "String";
  char const * CAS::TYPE_NAME_FLOAT            = UIMA_CAS_PFX "Float";
  char const * CAS::TYPE_NAME_LIST_BASE        = UIMA_CAS_PFX "ListBase";
  char const * CAS::TYPE_NAME_FS_LIST           = UIMA_CAS_PFX "FSList";
  char const * CAS::TYPE_NAME_EMPTY_FS_LIST           = UIMA_CAS_PFX "EmptyFSList";
  char const * CAS::TYPE_NAME_NON_EMPTY_FS_LIST          = UIMA_CAS_PFX "NonEmptyFSList";
  char const * CAS::TYPE_NAME_FLOAT_LIST        = UIMA_CAS_PFX "FloatList";
  char const * CAS::TYPE_NAME_NON_EMPTY_FLOAT_LIST     = UIMA_CAS_PFX "NonEmptyFloatList";
  char const * CAS::TYPE_NAME_EMPTY_FLOAT_LIST      = UIMA_CAS_PFX "EmptyFloatList";
  char const * CAS::TYPE_NAME_INTEGER_LIST          = UIMA_CAS_PFX "IntegerList";
  char const * CAS::TYPE_NAME_NON_EMPTY_INTEGER_LIST       = UIMA_CAS_PFX "NonEmptyIntegerList";
  char const * CAS::TYPE_NAME_EMPTY_INTEGER_LIST        = UIMA_CAS_PFX "EmptyIntegerList";
  char const * CAS::TYPE_NAME_STRING_LIST       = UIMA_CAS_PFX "StringList";
  char const * CAS::TYPE_NAME_NON_EMPTY_STRING_LIST    = UIMA_CAS_PFX "NonEmptyStringList";
  char const * CAS::TYPE_NAME_EMPTY_STRING_LIST     = UIMA_CAS_PFX "EmptyStringList";
  char const * CAS::TYPE_NAME_ARRAY_BASE       = UIMA_CAS_PFX "ArrayBase";
  char const * CAS::TYPE_NAME_FS_ARRAY          = UIMA_CAS_PFX "FSArray";
  char const * CAS::TYPE_NAME_FLOAT_ARRAY       = UIMA_CAS_PFX "FloatArray";
  char const * CAS::TYPE_NAME_INTEGER_ARRAY         = UIMA_CAS_PFX "IntegerArray";
  char const * CAS::TYPE_NAME_STRING_ARRAY      = UIMA_CAS_PFX "StringArray";
  char const * CAS::TYPE_NAME_SOFA              = UIMA_CAS_PFX "Sofa";
  char const * CAS::TYPE_NAME_LOCALSOFA         = UIMA_CAS_PFX "Sofa";
  char const * CAS::TYPE_NAME_REMOTESOFA        = UIMA_CAS_PFX "Sofa";

  char const * CAS::FEATURE_BASE_NAME_HEAD          =  "head";
  char const * CAS::FEATURE_BASE_NAME_TAIL          =  "tail";
  char const * CAS::FEATURE_FULL_NAME_FS_LIST_TAIL  =  UIMA_CAS_PFX "NonEmptyFSList" UIMA_TYPE_FEATURE_SEPARATOR "tail";
  char const * CAS::FEATURE_FULL_NAME_FS_LIST_HEAD  =  UIMA_CAS_PFX "NonEmptyFSList" UIMA_TYPE_FEATURE_SEPARATOR "head";
  char const * CAS::FEATURE_BASE_NAME_SOFANUM       = "sofaNum";
  char const * CAS::FEATURE_BASE_NAME_SOFAID        = "sofaID";
  char const * CAS::FEATURE_BASE_NAME_SOFAMIME      = "mimeType";
  char const * CAS::FEATURE_BASE_NAME_SOFAURI       = "sofaURI";
  char const * CAS::FEATURE_BASE_NAME_SOFASTRING    = "sofaString";
  char const * CAS::FEATURE_BASE_NAME_SOFAARRAY     = "sofaArray";

  char const * CAS::TYPE_NAME_BOOLEAN          = UIMA_CAS_PFX "Boolean";
  char const * CAS::TYPE_NAME_BYTE          = UIMA_CAS_PFX "Byte";
  char const * CAS::TYPE_NAME_SHORT         = UIMA_CAS_PFX "Short";
  char const * CAS::TYPE_NAME_LONG          = UIMA_CAS_PFX "Long";
  char const * CAS::TYPE_NAME_DOUBLE          = UIMA_CAS_PFX "Double";

  char const * CAS::TYPE_NAME_BOOLEAN_ARRAY          = UIMA_CAS_PFX "BooleanArray";
  char const * CAS::TYPE_NAME_BYTE_ARRAY          = UIMA_CAS_PFX "ByteArray";
  char const * CAS::TYPE_NAME_SHORT_ARRAY         = UIMA_CAS_PFX "ShortArray";
  char const * CAS::TYPE_NAME_LONG_ARRAY          = UIMA_CAS_PFX "LongArray";
  char const * CAS::TYPE_NAME_DOUBLE_ARRAY          = UIMA_CAS_PFX "DoubleArray";

  char const * CAS::FEATURE_FULL_NAME_SOFANUM       = UIMA_CAS_PFX "Sofa" UIMA_TYPE_FEATURE_SEPARATOR "sofaNum";
  char const * CAS::FEATURE_FULL_NAME_SOFAID        = UIMA_CAS_PFX "Sofa" UIMA_TYPE_FEATURE_SEPARATOR "sofaID";
  char const * CAS::FEATURE_FULL_NAME_SOFAMIME      = UIMA_CAS_PFX "Sofa" UIMA_TYPE_FEATURE_SEPARATOR "mimeType";
  char const * CAS::FEATURE_FULL_NAME_SOFAURI       = UIMA_CAS_PFX "RemoteSofa" UIMA_TYPE_FEATURE_SEPARATOR "sofaURI";
  char const * CAS::FEATURE_FULL_NAME_SOFASTRING    = UIMA_CAS_PFX "LocalSofa" UIMA_TYPE_FEATURE_SEPARATOR "sofaString";
  char const * CAS::FEATURE_FULL_NAME_SOFAARRAY     = UIMA_CAS_PFX "LocalSofa" UIMA_TYPE_FEATURE_SEPARATOR "sofaArray";

  char const * CAS::INDEXID_SOFA              = "SofaIndex";
  char const * CAS::NAME_DEFAULT_TEXT_SOFA    = "_InitialView";
  char const * CAS::NAME_DEFAULT_SOFA         = "_InitialView";

  char const * CAS::TYPE_NAME_ANNOTATION_BASE       = UIMA_CAS_PFX "AnnotationBase";
  char const * CAS::TYPE_NAME_ANNOTATION            = ANNOTATION;
  char const * CAS::TYPE_NAME_DOCUMENT_ANNOTATION = DOCUMENTANNOTATION;
  char const * CAS::FEATURE_BASE_NAME_SOFA       = SOFA;
  char const * CAS::FEATURE_FULL_NAME_SOFA       = SOFA UIMA_TYPE_FEATURE_SEPARATOR SOFA;
  char const * CAS::FEATURE_BASE_NAME_BEGIN      = BEGIN;
  char const * CAS::FEATURE_FULL_NAME_BEGIN      = ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR BEGIN;
  char const * CAS::FEATURE_BASE_NAME_END        = END;
  char const * CAS::FEATURE_FULL_NAME_END        = ANNOTATION UIMA_TYPE_FEATURE_SEPARATOR END;
  char const * CAS::FEATURE_BASE_NAME_LANGUAGE = LANGUAGE;
  char const * CAS::FEATURE_FULL_NAME_LANGUAGE = DOCUMENTANNOTATION UIMA_TYPE_FEATURE_SEPARATOR LANGUAGE;
  char const * CAS::INDEXID_ANNOTATION = "AnnotationIndex";
}
/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

/*
 * Most of the methods in here use the lowlevel API directly.
*/

namespace uima {
  UIMA_EXC_CLASSIMPLEMENT( CouldNotCreateFSOfFinalTypeException, CASException );
  UIMA_EXC_CLASSIMPLEMENT( DuplicateSofaNameException, CASException );
  UIMA_EXC_CLASSIMPLEMENT( InvalidBaseCasMethod, CASException );

  CAS::CAS(uima::internal::CASDefinition & casDefs,
           size_t uiFSHeapPageSize,
           size_t uiStringHeapPageSize,
           size_t uiStringRefHeapPageSize)
      : iv_casDefinition( & casDefs ),
      iv_typeSystem(NULL),
      iv_heap(NULL),
      initialSofaCreated(false),
      iv_sofaNum(0),
      iv_sofaCount(0),
      iv_initialView(NULL),
      iv_indexRepository(NULL),
      iv_filterBuilder(NULL),
      iv_componentInfo(NULL),
      iv_utDocumentType(uima::lowlevel::TypeSystem::INVALID_TYPE),
      iv_utDocumentLangAsIntFeat(uima::lowlevel::TypeSystem::INVALID_FEATURE),
      iv_utDocumentLangAsStrFeat(uima::lowlevel::TypeSystem::INVALID_FEATURE),
      iv_cpDocument(NULL),
      iv_uiDocumentLength(0),
      iv_copyOfDocument(NULL),
      iv_tyDocumentAnnotation(uima::lowlevel::FSHeap::INVALID_FS) {
    // leave those as assertions, don't throw exceptions
    assert( casDefs.getTypeSystem().isCommitted() );
    assert( casDefs.getIndexDefinition().isCommitted() );

    iv_typeSystem = &casDefs.getTypeSystem();
    iv_heap = new uima::lowlevel::FSHeap(casDefs.getTypeSystem(),
                                         uiFSHeapPageSize,
                                         uiStringHeapPageSize,
                                         uiStringRefHeapPageSize);
    assert( EXISTS(iv_heap) );

    iv_indexRepository = new uima::lowlevel::IndexRepository(casDefs.getIndexDefinition(),
                         *iv_heap, *this);
    assert( EXISTS( iv_indexRepository) );

    iv_filterBuilder = new uima::FSFilterBuilder();
    assert( EXISTS(iv_filterBuilder) );
    isbaseCas = true;
    iv_baseCas = this;
    isDeletingViews = false;
    bOwnsCASDefinition=false;
  }

  // Constructor used for views
  CAS::CAS(CAS* inCas, SofaFS inSofa):
      iv_sofaNum(0),
      iv_sofaCount(0),
      iv_cpDocument(NULL),
      iv_uiDocumentLength(0),
      iv_copyOfDocument(NULL),
      iv_tyDocumentAnnotation(uima::lowlevel::FSHeap::INVALID_FS) {
    iv_casDefinition = inCas->iv_casDefinition;
    iv_typeSystem = inCas->iv_typeSystem;
    iv_heap = inCas->iv_heap;
    iv_componentInfo = inCas->iv_componentInfo;
    iv_utDocumentLangAsIntFeat = uima::lowlevel::TypeSystem::INVALID_FEATURE;
    iv_utDocumentLangAsStrFeat = uima::lowlevel::TypeSystem::INVALID_FEATURE;
    refreshCachedTypes();

    if (inSofa.isValid()) {
      lowlevel::TyFS tySofa = internal::FSPromoter::demoteFS(inSofa);
      iv_sofaNum = iv_heap->getIntValue(tySofa, internal::gs_tySofaNumFeature);
      UnicodeStringRef pDoc = iv_heap->getStringValue(tySofa, internal::gs_tySofaStringFeature);
      copyDocumentString(pDoc);
    } else {
      iv_sofaNum = 1;
    }

    // each view has unique indexRepository
    iv_indexRepository = new uima::lowlevel::IndexRepository(iv_casDefinition->getIndexDefinition(),
                         *iv_heap, *this);
    assert( EXISTS(iv_indexRepository) );
    if ((int)inCas->iv_sofa2indexMap.size() < iv_sofaNum+1) {
      inCas->iv_sofa2indexMap.resize(iv_sofaNum + 1);
    }
    // map to a Sofa's IR. This map is deleted when the CAS definition may change.
    inCas->iv_sofa2indexMap[iv_sofaNum] = iv_indexRepository;
    iv_filterBuilder = inCas->iv_filterBuilder;
    isbaseCas = false;
    iv_baseCas = inCas;
    bOwnsCASDefinition = false;
    isDeletingViews = false;
  }

  CAS::CAS(uima::internal::CASDefinition & casDefs,
           bool ownsCasDef,
           size_t uiFSHeapPageSize,
           size_t uiStringHeapPageSize,
           size_t uiStringRefHeapPageSize)
      : iv_casDefinition( & casDefs ),
      iv_typeSystem(NULL),
      iv_heap(NULL),
      iv_sofaNum(0),
      iv_sofaCount(0),
      initialSofaCreated(false),
      iv_initialView(NULL),
      iv_indexRepository(NULL),
      iv_filterBuilder(NULL),
      iv_componentInfo(NULL),
      iv_utDocumentType(uima::lowlevel::TypeSystem::INVALID_TYPE),
      iv_utDocumentLangAsIntFeat(uima::lowlevel::TypeSystem::INVALID_FEATURE),
      iv_utDocumentLangAsStrFeat(uima::lowlevel::TypeSystem::INVALID_FEATURE),
      iv_cpDocument(NULL),
      iv_uiDocumentLength(0),
      iv_copyOfDocument(NULL),
      iv_tyDocumentAnnotation(uima::lowlevel::FSHeap::INVALID_FS) {
    // leave those as assertions, don't throw exceptions
    assert( casDefs.getTypeSystem().isCommitted() );
    assert( casDefs.getIndexDefinition().isCommitted() );

    iv_typeSystem = &casDefs.getTypeSystem();
    iv_heap = new uima::lowlevel::FSHeap(casDefs.getTypeSystem(),
                                         uiFSHeapPageSize,
                                         uiStringHeapPageSize,
                                         uiStringRefHeapPageSize);
    assert( EXISTS(iv_heap) );

    iv_indexRepository = new uima::lowlevel::IndexRepository(casDefs.getIndexDefinition(),
                         *iv_heap, *this);
    assert( EXISTS( iv_indexRepository) );

    iv_filterBuilder = new uima::FSFilterBuilder();
    assert( EXISTS(iv_filterBuilder) );
    isbaseCas = true;
    iv_baseCas = this;
    bOwnsCASDefinition=ownsCasDef;
    isDeletingViews = false;
  }

  CAS::~CAS() {
    
    //always delete index repository
    if (this->iv_indexRepository != NULL) {
      delete iv_indexRepository;
      iv_indexRepository = NULL;
		}
    if (this->iv_cpDocument != NULL) {
      delete[] this->iv_cpDocument;
      this->iv_cpDocument = NULL;
    }
    //initial call to delete object
    if (this->isbaseCas) {
      this->iv_baseCas->isDeletingViews = true;
      
      if (this->iv_baseCas->iv_heap != NULL) {
        delete this->iv_baseCas->iv_heap;
        this->iv_baseCas->iv_heap = NULL;
      }
      if (iv_baseCas->iv_filterBuilder != NULL) {
        delete iv_baseCas->iv_filterBuilder;
        iv_baseCas->iv_filterBuilder = NULL;
      }
      if (this->iv_baseCas->bOwnsCASDefinition ) {
		    if (this->iv_baseCas->iv_casDefinition != NULL) { 
               delete this->iv_baseCas->iv_casDefinition;
			    this->iv_baseCas->iv_casDefinition = NULL;
		    }
      }
      map<int, CAS*>::iterator mIter;
      map<int,CAS*> mapcopy(this->iv_baseCas->iv_sofa2tcasMap);
      for ( mIter = mapcopy.begin( );
            mIter != mapcopy.end( ); mIter++ ) {
        CAS* tcas = mIter->second;
        if ( tcas != NULL) {
          delete tcas;
        }
      //this->iv_baseCas->iv_sofa2tcasMap.clear( );
      //this->iv_baseCas->iv_sofa2indexMap.clear();
      } 
    } else {
      if (!this->iv_baseCas->isDeletingViews) {
        dropView(this->getSofaNum());
        this->iv_baseCas->isDeletingViews=true;
        delete this->iv_baseCas;
      }
    }
  }


  FSFilterBuilder const & CAS::getFSFilterBuilder() const {
    assert( EXISTS(iv_filterBuilder) );
    return *iv_filterBuilder;
  }


  TypeSystem const & CAS::getTypeSystem(void) const {
    assert( EXISTS(iv_heap) );
    return *iv_typeSystem;
  }

  uima::lowlevel::FSHeap * CAS::getHeap() {
    assert( EXISTS(iv_heap) );
    return iv_heap;
  }



  TyErrorId CAS::reset() {
    assert( EXISTS(iv_heap) );
    assert( EXISTS(iv_indexRepository) );
    if (!isbaseCas) {
      iv_baseCas->reset();
    } else {
      // reset all views for this CAS
      map<int, CAS*>::iterator mIter;
      for ( mIter = iv_sofa2tcasMap.begin( ); mIter != iv_sofa2tcasMap.end( ); mIter++ ) {
        CAS* tcas = mIter->second;
        tcas->iv_cpDocument = NULL;
        if (NULL != tcas->iv_copyOfDocument)
          delete [] tcas->iv_copyOfDocument;
        tcas->iv_copyOfDocument = NULL;
        tcas->iv_uiDocumentLength = 0;
        tcas->iv_indexRepository->reset();
        tcas->iv_tyDocumentAnnotation = uima::lowlevel::FSHeap::INVALID_FS;
      }
      iv_heap->reset();
      iv_baseCas->initialSofaCreated = false;
      iv_sofaCount = 1; // for initial view
      iv_indexRepository->reset();
    }
    return UIMA_ERR_NONE;
  }



  FeatureStructure CAS::createFS(Type const & crType) {
    if (!crType.isValid()) {
      UIMA_EXC_THROW_NEW(InvalidFSTypeObjectException,
                         UIMA_ERR_INVALID_FSTYPE_OBJECT,
                         UIMA_MSG_ID_EXC_INVALID_FSTYPE_OBJECT,
                         ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_FS),
                         ErrorInfo::recoverable
                        );
    }
    uima::lowlevel::TyFSType tyType = internal::FSPromoter::demoteType(crType);
    switch (tyType) {
    case uima::internal::gs_tyIntegerType:
    case uima::internal::gs_tyFloatType:
    case uima::internal::gs_tyStringType:
    case uima::internal::gs_tyBooleanType:
    case uima::internal::gs_tyByteType:
    case uima::internal::gs_tyShortType:
    case uima::internal::gs_tyLongType:
    case uima::internal::gs_tyDoubleType: {
        ErrorMessage errMessage(UIMA_MSG_ID_EXC_COULD_NOT_CREATE_FS_FINAL_TYPE );
        errMessage.addParam( crType.getName() );
        UIMA_EXC_THROW_NEW(CouldNotCreateFSOfFinalTypeException,
                           UIMA_ERR_COULD_NOT_CREATE_FS_OF_FINAL_TYPE,
                           errMessage,
                           UIMA_MSG_ID_EXCON_CREATING_FS,
                           ErrorInfo::recoverable
                          );
      }
    default:
      ;
    }

    // if new FS is annotation type, set the sofa feature
    lowlevel::TyFS temp= iv_heap->createFS(tyType);
    if (iv_heap->getTypeSystem().subsumes(internal::gs_tyAnnotationType, tyType)) {
      iv_heap->setFSValue(temp, internal::gs_tySofaRefFeature,
                          internal::FSPromoter::demoteFS(getSofa()));
    }
    return internal::FSPromoter::promoteFS(temp, *this );
  }

  int CAS::addString(UChar const * cpBuffer, size_t uiLength) {
    UnicodeStringRef uls( cpBuffer, uiLength );
    int shoff = iv_heap->addString(uls);
    return iv_heap->getStringAsFS(shoff);
  }


  int CAS::addString(icu::UnicodeString const & crString) {
    UnicodeStringRef uls( crString.getBuffer(), crString.length() );
    int shoff = iv_heap->addString(uls);
    return iv_heap->getStringAsFS(shoff);
  }

  int CAS::addString(const UnicodeStringRef & uls) {
    int shoff = iv_heap->addString(uls);
    return iv_heap->getStringAsFS(shoff);
  }

  ArrayFS CAS::createArrayFS(size_t uiLength) {
    return ArrayFS::createArrayFS(*this, uiLength);
  }
  FloatArrayFS CAS::createFloatArrayFS(size_t uiLength) {
    return FloatArrayFS::createArrayFS(*this, uiLength);
  }
  IntArrayFS CAS::createIntArrayFS(size_t uiLength) {
    return IntArrayFS::createArrayFS(*this, uiLength);
  }
  StringArrayFS CAS::createStringArrayFS(size_t uiLength) {
    return StringArrayFS::createArrayFS(*this, uiLength);
  }

  BooleanArrayFS CAS::createBooleanArrayFS(size_t uiLength) {
    return BooleanArrayFS::createArrayFS(*this, uiLength);
  }
  ByteArrayFS CAS::createByteArrayFS(size_t uiLength) {
    return ByteArrayFS::createArrayFS(*this, uiLength);
  }
  ShortArrayFS CAS::createShortArrayFS(size_t uiLength) {
    return ShortArrayFS::createArrayFS(*this, uiLength);
  }

  LongArrayFS CAS::createLongArrayFS(size_t uiLength) {
    return LongArrayFS::createArrayFS(*this, uiLength);
  }
  DoubleArrayFS CAS::createDoubleArrayFS(size_t uiLength) {
    return DoubleArrayFS::createArrayFS(*this, uiLength);
  }


  ListFS CAS::createListFS( ) {
    return ListFS::createListFS(*this);
  }
  FloatListFS CAS::createFloatListFS( ) {
    return FloatListFS::createListFS(*this);
  }
  IntListFS CAS::createIntListFS( ) {
    return IntListFS::createListFS(*this);
  }
  StringListFS CAS::createStringListFS( ) {
    return StringListFS::createListFS(*this);
  }

  /////////////////////////////////////////////////////////////
  // index

  FSIndexRepository & CAS::getIndexRepository( void ) {
    if (isbaseCas) {
      invalidBaseCasMethod();
    }
    assert( EXISTS(iv_indexRepository) );
    return *iv_indexRepository;
  }

  FSIndexRepository const & CAS::getIndexRepository( void ) const {
    if (isbaseCas) {
      ErrorMessage errMessage(UIMA_MSG_ID_EXC_INVALID_BASE_CAS_METHOD);
      UIMA_EXC_THROW_NEW(InvalidBaseCasMethod,
                         UIMA_ERR_INVALID_BASE_CAS_METHOD,
                         errMessage,
                         UIMA_MSG_ID_NO_MESSAGE_AVAILABLE,
                         ErrorInfo::unrecoverable
                        );
    }
    assert( EXISTS(iv_indexRepository) );
    return *iv_indexRepository;
  }

  lowlevel::IndexRepository  & CAS::getLowlevelIndexRepository( void ) const {
    if (isbaseCas) {
      ErrorMessage errMessage(UIMA_MSG_ID_EXC_INVALID_BASE_CAS_METHOD);
      UIMA_EXC_THROW_NEW(InvalidBaseCasMethod,
                         UIMA_ERR_INVALID_BASE_CAS_METHOD,
                         errMessage,
                         UIMA_MSG_ID_NO_MESSAGE_AVAILABLE,
                         ErrorInfo::unrecoverable
                        );
    }
    assert( EXISTS(iv_indexRepository) );
    return *iv_indexRepository;
  }

  FSIterator CAS::iterator() {
    lowlevel::IndexIterator* it = new uima::lowlevel::DefaultFSIterator(*iv_heap);
    return FSIterator(it, this );
  }

  /////////////////////////////////////////////////////////////////////////////
  // Get or Create CAS view for aSofa

  CAS* CAS::getView(SofaFS aSofa) {
    CAS* aTcas;
    int sofaNum = iv_heap->getIntValue(internal::FSPromoter::demoteFS(aSofa), internal::gs_tySofaNumFeature);
    map<int, CAS*>::iterator cit = iv_baseCas->iv_sofa2tcasMap.find(sofaNum);
    if (cit == iv_baseCas->iv_sofa2tcasMap.end()) {
      if (sofaNum > iv_baseCas->iv_sofaCount) {
        // This sofa must have just been created during binary deserialization
        assert(iv_baseCas->iv_sofaCount+1 == sofaNum);
        iv_baseCas->iv_sofaCount = sofaNum;
        if ( iv_sofaCount+1 > (int) iv_baseCas->iv_sofa2indexMap.size()) {
          iv_baseCas->iv_sofa2indexMap.resize(sofaNum + 1);
          iv_baseCas->iv_sofa2indexMap[sofaNum] = NULL;
        }
      }
      aTcas = (CAS*) new uima::internal::CASImpl(this->iv_baseCas, aSofa);
      assert ( EXISTS(aTcas) );
      pair<int, CAS*> p1(sofaNum, aTcas);
      iv_baseCas->iv_sofa2tcasMap.insert(p1);
      return aTcas;
    }
//    if (createDocAnn) {
//    (*cit).second->setDocumentAnnotationFromSofa( );
//    }
    if (sofaNum > iv_baseCas->iv_sofaCount) {
      // This sofa must have just been created during xcas deserialization
      assert(iv_baseCas->iv_sofaCount+1 == sofaNum);
      iv_baseCas->iv_sofaCount = sofaNum;
      if ( iv_sofaCount+1 > (int) iv_baseCas->iv_sofa2indexMap.size()) {
        iv_baseCas->iv_sofa2indexMap.resize(sofaNum + 1);
        iv_baseCas->iv_sofa2indexMap[sofaNum] = NULL;
      }
    }
    return (*cit).second;
  }


  // Get the CAS View from sofaNum value
  CAS* CAS::getViewBySofaNum(int sofaNum) {
    CAS* aTcas;
    map<int, CAS*>::iterator cit = iv_baseCas->iv_sofa2tcasMap.find(sofaNum);
    if (cit == iv_baseCas->iv_sofa2tcasMap.end()) {
      aTcas = (CAS*) new uima::internal::CASImpl(this->iv_baseCas, getSofa(sofaNum));
      assert ( EXISTS(aTcas) );
      pair<int, CAS*> p1(sofaNum, aTcas);
      iv_baseCas->iv_sofa2tcasMap.insert(p1);
      return aTcas;
    }
    return (*cit).second;
  }

  CAS* CAS::getView(const icu::UnicodeString & localViewName) {
    SofaID* sid;
    bool deleteSofaID = false;
    if (0 != iv_baseCas->iv_componentInfo) {
      sid = const_cast<SofaID*> (&iv_baseCas->iv_componentInfo->mapToSofaID(localViewName));
    } else {
      sid = new SofaID();
      deleteSofaID = true;
      sid->setSofaId(localViewName);
    }

    // if this resolves to the Initial View, return view(1)...
    // ... as the Sofa for this view may not exist yet
    if (0==sid->getSofaId().compare(CAS::NAME_DEFAULT_SOFA)) {
      if (deleteSofaID) {
        delete sid;
      }
      return getInitialView();
    }

    SofaFS as = getSofa(sid->getSofaId());
    if (!as.isValid()) {
      ErrorMessage errMessage(UIMA_MSG_ID_EXC_SOFA_NAME_NOT_FOUND);
      errMessage.addParam( sid->getSofaId());
      UIMA_EXC_THROW_NEW(DuplicateSofaNameException,
                         UIMA_ERR_RESOURCE_NOT_FOUND,
                         errMessage,
                         UIMA_MSG_ID_NO_MESSAGE_AVAILABLE,
                         ErrorInfo::unrecoverable
                        );
    }
    if (deleteSofaID) {
      delete sid;
    }
    return getView(as);
  }

  CAS* CAS::createView(icu::UnicodeString const & localViewName) {
    // map the input name
    SofaID* sid;
    bool deleteSofaID = false;
    if (0 != iv_baseCas->iv_componentInfo) {
      sid = const_cast<SofaID*> (&iv_baseCas->iv_componentInfo->mapToSofaID(localViewName));
    } else {
      sid = new SofaID();
      sid->setSofaId(localViewName);
      deleteSofaID = true;
    }
    // test if this is the reserved name
    if ( 0 == sid->getSofaId().compare(UnicodeString(CAS::NAME_DEFAULT_SOFA)) ) {
      ErrorMessage errMessage(UIMA_MSG_ID_EXC_SOFA_NAME_ALREADY_EXISTS);
      errMessage.addParam( localViewName );
      UIMA_EXC_THROW_NEW(DuplicateSofaNameException,
                         UIMA_ERR_DUPLICATE_EXISTS,
                         errMessage,
                         UIMA_MSG_ID_EXCON_CREATING_FS,
                         ErrorInfo::unrecoverable
                        );
    }

    SofaFS newSofa = createSofa(sid->getSofaId(), UnicodeStringRef());
    if (deleteSofaID) {
      delete sid;
    }
    return getView(newSofa);
  }

  UnicodeStringRef CAS::getViewName() {
    SofaFS thisSofa = getSofa();
    if (thisSofa.isValid()) {
      return getSofa().getSofaID();
    }
    return UnicodeStringRef();
  }

  void CAS::invalidBaseCasMethod() {
    ErrorMessage errMessage(UIMA_MSG_ID_EXC_INVALID_BASE_CAS_METHOD);
    UIMA_EXC_THROW_NEW(InvalidBaseCasMethod,
                       UIMA_ERR_INVALID_BASE_CAS_METHOD,
                       errMessage,
                       UIMA_MSG_ID_NO_MESSAGE_AVAILABLE,
                       ErrorInfo::unrecoverable
                      );
  }

  // deprecated version
  void CAS::setDocumentText(UChar const * cpDocument, size_t uiLength, bool bCopyToCAS ) {
    if (cpDocument == NULL) {
      assert( uiLength == 0 );
    }
    if (bCopyToCAS && ( cpDocument != NULL ) ) {
      copyDocumentString(UnicodeStringRef(cpDocument, uiLength));
    } else {
      iv_cpDocument = cpDocument;
      iv_uiDocumentLength = uiLength;
    }
    SofaFS thisSofa = getSofa();
    if (!thisSofa.isValid()) {
      thisSofa = createInitialSofa(UnicodeStringRef("text"));
    }
    thisSofa.setLocalSofaData(UnicodeStringRef(iv_cpDocument, iv_uiDocumentLength));
  }

  void CAS::setDocumentText(UnicodeStringRef const text) {
    if (isbaseCas) {
      invalidBaseCasMethod();
    }
    SofaFS thisSofa = getSofa();
    if (!thisSofa.isValid()) {
      thisSofa = createInitialSofa(UnicodeStringRef("text"));
    }
    copyDocumentString(text);
    thisSofa.setLocalSofaData(text);
  }

  // internal use only
  void CAS::setDocTextFromDeserializtion(UnicodeStringRef text) {
    copyDocumentString(text);
    SofaFS thisSofa = getSofa();
    assert( thisSofa.isValid() );
    thisSofa.setStringValue(internal::FSPromoter::promoteFeature(internal::gs_tySofaStringFeature,
                            getHeap()->getTypeSystem()), text);
  }

  void CAS::setSofaDataString(UnicodeStringRef const text, icu::UnicodeString const & mimetype) {
    if (isbaseCas) {
      invalidBaseCasMethod();
    }
    SofaFS thisSofa = getSofa();
    if (!thisSofa.isValid()) {
      thisSofa = createInitialSofa(mimetype);
    }
    setDocumentText(text);
  }

  UnicodeStringRef CAS::getDocumentText()const {
    if (isbaseCas) {
      return UnicodeStringRef();
    }
    return UnicodeStringRef(iv_cpDocument, iv_uiDocumentLength);
  }

  void CAS::createDocumentAnnotation() {
    if (!isbaseCas) {
      assert( EXISTS(iv_heap) );
      if (iv_tyDocumentAnnotation == uima::lowlevel::FSHeap::INVALID_FS) {
        iv_tyDocumentAnnotation = iv_heap->createFS( iv_utDocumentType );
        uima::lowlevel::TypeSystem const & ts = iv_heap->getTypeSystem();
        Language lang;
        int iLangID = (int) lang.asNumber();
        uima::lowlevel::TyFSFeature langFeat = ts.getFeatureByFullName(CAS::FEATURE_FULL_NAME_LANGUAGE);
        assert( ts.isValidFeature(langFeat) );
        int ref = iv_heap->addString( lang.asUnicodeString() );
        iv_heap->setStringValue( iv_tyDocumentAnnotation, langFeat, ref);
        iv_heap->setFSValue(iv_tyDocumentAnnotation, internal::gs_tySofaRefFeature,
                            internal::FSPromoter::demoteFS(getSofa()));
      }
      assert( iv_heap->isValid( iv_tyDocumentAnnotation ) );
      assert( iv_heap->resides( iv_tyDocumentAnnotation ) );
    }
  }

  DocumentFS const CAS::getDocumentAnnotation() const {
    if (isbaseCas || iv_tyDocumentAnnotation == uima::lowlevel::FSHeap::INVALID_FS ) {
      return DocumentFS();
    }
    return(DocumentFS)uima::internal::FSPromoter::promoteFS(iv_tyDocumentAnnotation, *this);
  }

  DocumentFS CAS::getDocumentAnnotation() {
    if (isbaseCas) {
      assertWithMsg(false, "no DocumentAnnotation in Base CAS!");
      return DocumentFS();
    }
    if ( iv_tyDocumentAnnotation == uima::lowlevel::FSHeap::INVALID_FS ) {
      createDocumentAnnotation();
    }
    return(DocumentFS)uima::internal::FSPromoter::promoteFS(iv_tyDocumentAnnotation, *this);
  }

  AnnotationFS CAS::createAnnotation( Type const & type, size_t uiBegin, size_t uiEnd) {
    if (isbaseCas) {
      assertWithMsg(false, "Cannot create an Annotation in the Base CAS!");
      return AnnotationFS();
    }
    assert(EXISTS(iv_heap));
    assert( getTypeSystem().getType(CAS::TYPE_NAME_ANNOTATION).subsumes(type) );
    if (!type.isValid()) {
      UIMA_EXC_THROW_NEW(InvalidFSTypeObjectException,
                         UIMA_ERR_INVALID_FSTYPE_OBJECT,
                         UIMA_MSG_ID_EXC_INVALID_FSTYPE_OBJECT,
                         ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_FS),
                         ErrorInfo::recoverable );
    }
    lowlevel::TyFS tyAn = iv_heap->createFS(internal::FSPromoter::demoteType(type));
    iv_heap->setFSValue(tyAn, internal::gs_tySofaRefFeature,
                        internal::FSPromoter::demoteFS(this->getSofa()));
    iv_heap->setIntValue(tyAn, internal::gs_tyBeginPosFeature, (int)uiBegin );
    iv_heap->setIntValue(tyAn, internal::gs_tyEndPosFeature, (int)uiEnd );
    return(AnnotationFS)internal::FSPromoter::promoteFS(tyAn, *this);
  }

  ANIndex const CAS::getAnnotationIndex(Type const & crType) const {
    if (isbaseCas) {
      assertWithMsg(false, "Annotation Index does not exist in Base CAS!");
      return ANIndex();
    }
    try {
      return(ANIndex)getIndexRepository().getIndex(CAS::INDEXID_ANNOTATION, crType);
    } catch ( InvalidIndexIDException & ) {
      assertWithMsg(false, "Annotation Index does not exist!");
      return ANIndex();
    } catch ( WrongFSTypeForIndexException & ) {
      assertWithMsg(false, "Annotation Index exists with wrong type!");
      return ANIndex();
    }
  }

  ANIndex CAS::getAnnotationIndex(Type const & crType) {
    if (isbaseCas) {
      assertWithMsg(false, "Annotation Index does not exist in Base CAS!");
      return ANIndex();
    }
    return((CAS const *) this)->getAnnotationIndex(crType);
  }

  ANIndex CAS::getAnnotationIndex() {
    if (isbaseCas) {
      assertWithMsg(false, "Annotation Index does not exist in Base CAS!");
      return ANIndex();
    }
    return getAnnotationIndex(iv_heap->getTypeSystem().getType(CAS::TYPE_NAME_ANNOTATION));
  }

  void CAS::setSofaDataArray(FeatureStructure array, icu::UnicodeString const & mime) {
    if (isbaseCas) {
      invalidBaseCasMethod();
    }
    SofaFS thisSofa = getSofa();
    if (!thisSofa.isValid()) {
      thisSofa = createInitialSofa(mime);
    }
    getSofa().setLocalSofaData(array);
  }

  FeatureStructure CAS::getSofaDataArray() {
    if (isbaseCas) {
      assertWithMsg(false, "no Sofa data in Base CAS!");
      return FeatureStructure();
    }
    return getSofa().getLocalFSData();
  }

  void CAS::setSofaDataURI(icu::UnicodeString const & uri, icu::UnicodeString const & mime) {
    if (isbaseCas) {
      invalidBaseCasMethod();
    }
    if (!getSofa().isValid()) {
      createInitialSofa(mime);
    }
    getSofa().setRemoteSofaURI(uri);
  }

  UnicodeStringRef CAS::getSofaDataURI() {
    if (isbaseCas) {
      assertWithMsg(false, "no Sofa data in Base CAS!");
      return UnicodeStringRef();
    }
    return getSofa().getSofaURI();
  }

  SofaDataStream * CAS::getSofaDataStream() {
    if (isbaseCas) {
      invalidBaseCasMethod();
    }
    return getSofa().getSofaDataStream();
  }



  bool CAS::moveToBeginPosition (ANIterator & itOfType, AnnotationFS const & crFsFromAn) {
    /* Find begin position of annotation crToFS */
    size_t uiBegPosFromAn = crFsFromAn.getIntValue(internal::FSPromoter::promoteFeature(internal::gs_tyBeginPosFeature, iv_heap->getTypeSystem() ));

    /* Iterate to the begin position of our given annotation crToFS */
    bool bPosIsValid;
    size_t uiBegPosOfType;
    itOfType.moveToFirst();
    while (itOfType.get().isValid()) {
      uiBegPosOfType = itOfType.get().getIntValue(internal::FSPromoter::promoteFeature(internal::gs_tyBeginPosFeature, iv_heap->getTypeSystem() ));
      if (uiBegPosOfType == uiBegPosFromAn) {
        return bPosIsValid = true;
      }
      itOfType.moveToNext();
      if (uiBegPosOfType > uiBegPosFromAn)
        return bPosIsValid = false;
    }
    return bPosIsValid = false;
  }

  // Drop View from sofaMap
  void CAS::dropView(int sofaNum) {
    map<int, CAS*>::iterator cit = iv_baseCas->iv_sofa2tcasMap.find(sofaNum);
    if (cit != iv_baseCas->iv_sofa2tcasMap.end()) {
      iv_baseCas->iv_sofa2tcasMap.erase(cit);
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  // Sofa utilities
  /*
  public FSIndexRepositoryImpl getSofaIndexRepository(SofaFS aSofa) {
    return (FSIndexRepositoryImpl) sofa2indexMap.get(aSofa);
  }

  public void setSofaIndexRepository(SofaFS aSofa, FSIndexRepositoryImpl index) {
    sofa2indexMap.put(aSofa, index);
  }
  */

  SofaFS CAS::createLocalSofa(const char* sofaName, const char* mimeType) {
    UnicodeString const uName(sofaName, strlen(sofaName), "utf8");
    UnicodeString const uMime(mimeType, strlen(mimeType), "utf8");
    return createSofa(uName, uMime);
  }

  SofaFS CAS::createSofa(const SofaID & sofaName, const char* mimeType) {
    UnicodeString const uMime(mimeType, strlen(mimeType), "utf8");
    return createSofa(sofaName.getSofaId(), uMime);
  }

  SofaFS CAS::createSofa(UnicodeStringRef const sofaName, UnicodeStringRef const mimeType) {
    assert(EXISTS(iv_heap));
    if (iv_baseCas->getSofa(sofaName).isValid()) {
      ErrorMessage errMessage(UIMA_MSG_ID_EXC_SOFA_NAME_ALREADY_EXISTS);
      errMessage.addParam( sofaName );
      UIMA_EXC_THROW_NEW(DuplicateSofaNameException,
                         UIMA_ERR_DUPLICATE_EXISTS,
                         errMessage,
                         UIMA_MSG_ID_EXCON_CREATING_FS,
                         ErrorInfo::unrecoverable
                        );
    }
    Type sofaT = getTypeSystem().getType(CAS::TYPE_NAME_SOFA);
    lowlevel::TyFS tySofa = iv_heap->createFS(internal::FSPromoter::demoteType(sofaT));
    SofaFS newSofa = (SofaFS)internal::FSPromoter::promoteFS(tySofa, *this->iv_baseCas);
    bumpSofaCount();
    newSofa.setIntValue(internal::FSPromoter::promoteFeature(internal::gs_tySofaNumFeature, iv_heap->getTypeSystem()), iv_baseCas->iv_sofaCount);
    newSofa.setStringValue(internal::FSPromoter::promoteFeature(internal::gs_tySofaIDFeature, iv_heap->getTypeSystem()), sofaName);
    if (mimeType.length() > 0) {
      newSofa.setStringValue(internal::FSPromoter::promoteFeature(internal::gs_tySofaMimeFeature, iv_heap->getTypeSystem()), mimeType);
    }
    iv_baseCas->iv_indexRepository->add(tySofa);
    return newSofa;
  }

  SofaFS CAS::getSofa() {
    if (iv_sofaNum > 0) {
      return getSofa(iv_sofaNum);
    }
    return SofaFS(FeatureStructure());
  }

  SofaFS CAS::getSofa(const SofaID & sofaName) {
    return getSofa(sofaName.getSofaId());
  }

  SofaFS CAS::getSofa(char* sofaName) {
    UnicodeString const uName(sofaName, strlen(sofaName), "utf8");
    return getSofa(uName);
  }

  SofaFS CAS::getSofa(UnicodeStringRef sofaName) {
    FSIndex fsIdx = getBaseIndexRepository().getIndex(CAS::INDEXID_SOFA);
    FSIterator fsIt = fsIdx.iterator();
    while (fsIt.isValid()) {
      Feature idFeat =
        getTypeSystem().getFeatureByFullName(CAS::FEATURE_FULL_NAME_SOFAID);
      UnicodeStringRef sofaID = fsIt.get().getStringValue(idFeat);
      if (0 == sofaName.compare(sofaID)) {
        return (SofaFS) fsIt.get();
      }
      fsIt.moveToNext();
    }
    //HOW TO FAIL HERE? Currently returning an INVALID_FS
    return (SofaFS) FeatureStructure();
  }

  FSIterator CAS::getSofaIterator() {
    FSIndex fsIdx = getBaseIndexRepository().getIndex(CAS::INDEXID_SOFA);
    return fsIdx.iterator();
  }

  SofaFS CAS::getSofa(int sofaNum) {
    FSIndex fsIdx = getBaseIndexRepository().getIndex(CAS::INDEXID_SOFA);
    FSIterator fsIt = fsIdx.iterator();
    while (fsIt.isValid()) {
      Feature numFeat =
        getTypeSystem().getFeatureByFullName(CAS::FEATURE_FULL_NAME_SOFANUM);
      if (sofaNum == fsIt.get().getIntValue(numFeat)) {
        return (SofaFS) fsIt.get();
      }
      fsIt.moveToNext();
    }
    //HOW TO FAIL HERE? Currently returning an INVALID_FS
    return (SofaFS) FeatureStructure();
  }

  uima::lowlevel::IndexRepository * CAS::getIndexRepositoryForSofa(SofaFS sofa) {
    return iv_baseCas->iv_sofa2indexMap[iv_heap->getIntValue(internal::FSPromoter::demoteFS(sofa),
                                        internal::gs_tySofaNumFeature)];
  }

  // Record a new sofaFS and reserve space in the index map if necessary
  void CAS::bumpSofaCount() {
    iv_baseCas->iv_sofaCount++;
    if ( (iv_baseCas->iv_sofaCount+1) > (int)iv_baseCas->iv_sofa2indexMap.size()) {
      // enlarge map and set new entry to NULL
      iv_baseCas->iv_sofa2indexMap.resize(iv_baseCas->iv_sofaCount + 1);
      iv_baseCas->iv_sofa2indexMap[iv_baseCas->iv_sofaCount] = NULL;
    }
  }

  void CAS::registerView(SofaFS aSofa ) {
    lowlevel::TyFS tySofa = internal::FSPromoter::demoteFS(aSofa);
    iv_sofaNum = iv_heap->getIntValue(tySofa, internal::gs_tySofaNumFeature);
  }

  void CAS::updateDocumentAnnotation( ) {
    SofaFS thisSofa = getSofa();
    if (thisSofa.isValid()) {
      lowlevel::TyFS tySofa = internal::FSPromoter::demoteFS(thisSofa);
      UnicodeStringRef pDoc = iv_heap->getStringValue(tySofa, internal::gs_tySofaStringFeature);
      if (pDoc.length() > 0) {
        copyDocumentString(pDoc);
        getDocumentAnnotation(); // create DocumentAnnotation if necessary
        iv_heap->setIntValue( iv_tyDocumentAnnotation, uima::internal::gs_tyEndPosFeature,
                              (int) iv_uiDocumentLength);
        // re-index the annotation
        iv_indexRepository->remove(iv_tyDocumentAnnotation);
        iv_indexRepository->add(iv_tyDocumentAnnotation);
      }
    }
  }

  void CAS::pickupDocumentAnnotation( ) {
    // called from CAS binary deserialization
    ANIndex ai = getAnnotationIndex(iv_heap->getTypeSystem().
                                    getType(CAS::TYPE_NAME_DOCUMENT_ANNOTATION));
    ANIterator ait = ai.iterator();
    // Not valid if a non-text or remote sofa
    if ( ! ait.isValid() ) {
      return;
    }
    iv_tyDocumentAnnotation = internal::FSPromoter::demoteFS(ait.get());
    iv_uiDocumentLength = ait.get().getEndPosition();
    uima::lowlevel::TyFS sofaAddr = iv_heap->getFSValue(iv_tyDocumentAnnotation, internal::gs_tySofaRefFeature);
    UnicodeStringRef pDoc = iv_heap->getStringValue(sofaAddr, internal::gs_tySofaStringFeature);
    copyDocumentString(pDoc);
  }

//   void CAS::updateAndIndexDocumentAnnotation() {
//  assert( EXISTS(iv_heap) );
//  if( iv_tyDocumentAnnotation == uima::lowlevel::FSHeap::INVALID_FS ) {
//    createDocumentAnnotation();
//  }
//  assert( iv_heap->resides( iv_tyDocumentAnnotation ) );
//  assert( EXISTS( iv_indexRepository ) );
//  iv_indexRepository->remove(iv_tyDocumentAnnotation);

//  iv_heap->setIntValue( iv_tyDocumentAnnotation, uima::internal::gs_tyEndPosFeature,
//         (int) iv_uiDocumentLength);
//  iv_heap->setIntValue(iv_tyDocumentAnnotation, uima::internal::gs_tySofaRefFeature,
//        (int) this->getSofaNum() );
//  iv_indexRepository->add(iv_tyDocumentAnnotation);
//   }

  void CAS::copyDocumentString(UnicodeStringRef pDoc) {
    // must have a 2nd copy of the document because the copy in the stringHeap will
    // move when the stringHeap grows, unlike the older segmented heap.
    if (pDoc.length() == 0) {
      return;
    }
    if (iv_copyOfDocument)
      delete [] iv_copyOfDocument;
    size_t len = pDoc.length();
    iv_copyOfDocument = new UChar [len + 1];
    memcpy(iv_copyOfDocument, pDoc.getBuffer(), 2*len);
    // The internal document must be 0 terminated!
    iv_copyOfDocument[len] = 0;
    iv_cpDocument = iv_copyOfDocument;
    iv_uiDocumentLength = len;
  }

  void CAS::refreshCachedTypes() {
    assert( EXISTS(iv_heap) );
    iv_utDocumentType =
      iv_heap->getTypeSystem().getTypeByName( uima::CAS::TYPE_NAME_DOCUMENT_ANNOTATION );
    assert( iv_utDocumentType != uima::lowlevel::TypeSystem::INVALID_TYPE );
    iv_utDocumentLangAsStrFeat =
      iv_heap->getTypeSystem().getFeatureByBaseName( iv_utDocumentType,
          uima::CAS::FEATURE_BASE_NAME_LANGUAGE );
    assert( iv_utDocumentLangAsStrFeat != uima::lowlevel::TypeSystem::INVALID_FEATURE );
  }

  // True if CAS contains one Sofa with special name and without SofaURI or SofaFSData set
  bool CAS::isBackwardCompatibleCas() {
    if (iv_sofaCount != 1 || !isInitialSofaCreated())
      return 0;
    Feature idFeat =
      getTypeSystem().getFeatureByFullName(CAS::FEATURE_FULL_NAME_SOFAID);
    SofaFS initSofa = getSofa(1);
    if (initSofa.getLocalFSData().isValid() ||
        initSofa.getSofaURI().length() > 0 ) {
      return 0;
    }
    UnicodeStringRef sofaID = getSofa(1).getStringValue(idFeat);
    if ( 0 == initSofa.getSofaID().compare(NAME_DEFAULT_SOFA) ) {
      return 1;
    }
    return 0;
  }

  void CAS::setCurrentComponentInfo(AnnotatorContext* info) {
    // always store component info in base CAS
    iv_baseCas->iv_componentInfo = info;
  }

  void CAS::registerInitialSofa() {
    iv_baseCas->initialSofaCreated = true;
  }

  bool CAS::isInitialSofaCreated() {
    return iv_baseCas->initialSofaCreated;
  }

  SofaFS CAS::createInitialSofa(UnicodeStringRef const mimeType) {
    if (isInitialSofaCreated()) {
      return getSofa(1);
    }
    Type sofaT = getTypeSystem().getType(CAS::TYPE_NAME_SOFA);
    lowlevel::TyFS tySofa = iv_heap->createFS(internal::FSPromoter::demoteType(sofaT));
    SofaFS newSofa = (SofaFS)internal::FSPromoter::promoteFS(tySofa, *this->iv_baseCas);
    newSofa.setIntValue(internal::FSPromoter::promoteFeature(internal::gs_tySofaNumFeature, iv_heap->getTypeSystem()), 1);
    newSofa.setStringValue(internal::FSPromoter::promoteFeature(internal::gs_tySofaIDFeature, iv_heap->getTypeSystem()), NAME_DEFAULT_SOFA);
    newSofa.setStringValue(internal::FSPromoter::promoteFeature(internal::gs_tySofaMimeFeature, iv_heap->getTypeSystem()), mimeType);
    iv_baseCas->iv_indexRepository->add(tySofa);
    registerInitialSofa();
    return newSofa;
  }

  CAS* CAS::getInitialView() {
    if (iv_baseCas->iv_initialView != 0) {
      return iv_baseCas->iv_initialView;
    }
    // create the initial view, without a Sofa
    CAS* aTcas = (CAS*) new uima::internal::CASImpl(this->iv_baseCas);
    assert ( EXISTS(aTcas) );
    pair<int, CAS*> p1(1, aTcas);
    iv_baseCas->iv_sofa2tcasMap.insert(p1);
    iv_baseCas->iv_sofaCount = 1;
    iv_baseCas->iv_initialView = aTcas;
    return aTcas;
  }

  FSIndexRepository & CAS::getBaseIndexRepository( void ) {
    assert( EXISTS(iv_baseCas->iv_indexRepository) );
    return *iv_baseCas->iv_indexRepository;
  }

  CAS & CAS::getCasForTyFS(lowlevel::TyHeapCell tyCell) {
    lowlevel::TyFS type = getHeap()->getHeap().getHeapValue(tyCell);
    if (type != uima::lowlevel::FSHeap::INVALID_FS &&
        getHeap()->getTypeSystem().subsumes(internal::gs_tyAnnotationType,
                                            internal::FSPromoter::demoteType(internal::FSPromoter::promoteFS(tyCell, *this).getType()))) {
      // an annotation. Check that sofaNum agrees with current CAS
      lowlevel::TyFS annSofaAddr = getHeap()->getFSValue(tyCell, internal::gs_tySofaRefFeature);
      if ( annSofaAddr != internal::FSPromoter::demoteFS(getSofa())) {
        // does not agree. Get appropriate View for annotation Sofa
        int annSofaNum = getHeap()->getIntValue(annSofaAddr, internal::gs_tySofaNumFeature);
        return *getViewBySofaNum(annSofaNum);
      }
    }
    return *this;
  }


  /*****************************************************************************/
  /*  SofaFS                                                             */
  /*****************************************************************************/
  SofaFS::SofaFS(FeatureStructure const & fs)
      : FeatureStructure(fs) {
    assert(sizeof(FeatureStructure) == sizeof(SofaFS)); // no additonal data members
  }

  void SofaFS::setSofaMime(icu::UnicodeString const & aString) {
    //TODO FAIL THIS IF MIME ALREADY SET ???
    lowlevel::TyFS tySofa = internal::FSPromoter::demoteFS(*this);
    this->setStringValue(internal::FSPromoter::promoteFeature(internal::gs_tySofaMimeFeature, iv_cas->getHeap()->getTypeSystem()), aString);
    return;
  }

  void SofaFS::setLocalSofaData(FeatureStructure aFS) {
    //TODO NEED TO FAIL THIS IF DATA, STRING, or URI ALREADY SET !!!
    lowlevel::TyFS tySofa = internal::FSPromoter::demoteFS(*this);
    iv_cas->getHeap()->setFSValue(tySofa, internal::gs_tySofaArrayFeature,
                                  internal::FSPromoter::demoteFS(aFS));
    return;
  }

  void SofaFS::setLocalSofaData(UnicodeStringRef const aString) {
    //TODO NEED TO FAIL THIS IF DATA, STRING, or URI ALREADY SET !!!
    this->setStringValue(internal::FSPromoter::promoteFeature(internal::gs_tySofaStringFeature,
                         iv_cas->getHeap()->getTypeSystem()), aString);
    int sofaNum = this->getIntValue(internal::FSPromoter::promoteFeature(internal::gs_tySofaNumFeature,
                                    iv_cas->getHeap()->getTypeSystem()));
    iv_cas->getViewBySofaNum(sofaNum)->updateDocumentAnnotation();
    return;
  }

  void SofaFS::setRemoteSofaURI(const char* aURI) {
    UnicodeString const ucURI(aURI, strlen(aURI), "utf8");
    return setRemoteSofaURI(ucURI);
  }

  void SofaFS::setRemoteSofaURI(icu::UnicodeString const & aString) {
    //TODO NEED TO FAIL THIS IF DATA, STRING, or URI ALREADY SET !!!
    this->setStringValue(internal::FSPromoter::promoteFeature(internal::gs_tySofaURIFeature, iv_cas->getHeap()->getTypeSystem()), aString);
    return;
  }

  UnicodeStringRef SofaFS::getSofaMime() {
    return this->getStringValue(internal::FSPromoter::promoteFeature(internal::gs_tySofaMimeFeature, iv_cas->getHeap()->getTypeSystem()));
  }

  UnicodeStringRef SofaFS::getSofaID() {
    return this->getStringValue(internal::FSPromoter::promoteFeature(internal::gs_tySofaIDFeature, iv_cas->getHeap()->getTypeSystem()));
  }

  UnicodeStringRef SofaFS::getSofaURI() {
    return this->getStringValue(internal::FSPromoter::promoteFeature(internal::gs_tySofaURIFeature, iv_cas->getHeap()->getTypeSystem()));
  }

  int SofaFS::getSofaRef() {
    return this->getIntValue(internal::FSPromoter::promoteFeature(internal::gs_tySofaNumFeature, iv_cas->getHeap()->getTypeSystem()));
  }

  FeatureStructure SofaFS::getLocalFSData() {
    return this->getFSValue(internal::FSPromoter::promoteFeature(internal::gs_tySofaArrayFeature, iv_cas->getHeap()->getTypeSystem()));
  }

  UnicodeStringRef SofaFS::getLocalStringData() {
    return this->getStringValue(internal::FSPromoter::promoteFeature(internal::gs_tySofaStringFeature, iv_cas->getHeap()->getTypeSystem()));
  }

  SofaDataStream * SofaFS::getSofaDataStream() {
    Type type = getType();
    Feature uriFeat = type.getFeatureByBaseName(CAS::FEATURE_BASE_NAME_SOFAURI);
    Feature stringFeat = type.getFeatureByBaseName(CAS::FEATURE_BASE_NAME_SOFASTRING);
    Feature arrayFeat = type.getFeatureByBaseName(CAS::FEATURE_BASE_NAME_SOFAARRAY);

    if ( !isUntouchedFSValue(uriFeat) ) {                //remote sofa
      return new RemoteSofaDataStream(*this);
    } else if  ( !isUntouchedFSValue(stringFeat) ) {     //local sofa - data in string
      return new LocalSofaDataStream(*this);
    } else if ( !isUntouchedFSValue(arrayFeat) ) {       //local sofa - data in fs array
      return new LocalSofaDataStream(*this);
    } else return NULL;                                   //no sofa data set
  }

  SofaDataStream * SofaFS::getSofaDataStream(FeatureStructure & fs) {
    return NULL;
  }

  /*****************************************************************************/
  /*  AnnotationFS                                                             */
  /*****************************************************************************/
  AnnotationFS::AnnotationFS(FeatureStructure const & fs)
      : FeatureStructure(fs) {
    assert(sizeof(FeatureStructure) == sizeof(AnnotationFS)); // no additonal data members
    assert(EXISTS(iv_cas));
    assert( iv_cas->getHeap()->getTypeSystem().subsumes(iv_cas->getHeap()->getTypeSystem().getTypeByName(CAS::TYPE_NAME_ANNOTATION),
            iv_cas->getHeap()->getType(internal::FSPromoter::demoteFS(fs)) ) );
  }

  CAS & AnnotationFS::getCAS() {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_FSTYPE);
    assert(EXISTS(iv_cas));
    return *iv_cas;
  }

  CAS const & AnnotationFS::getCAS() const {
    checkValidity(UIMA_MSG_ID_EXCON_GETTING_FSTYPE);
    assert(EXISTS(iv_cas));
    return *iv_cas;
  }

  CAS * AnnotationFS::getView() {
    assert(EXISTS(iv_cas));
    lowlevel::TyFS annSofaAddr = iv_cas->getHeap()->getFSValue(iv_tyFS, internal::gs_tySofaRefFeature);
    int annSofaNum = iv_cas->getHeap()->getIntValue(annSofaAddr, internal::gs_tySofaNumFeature);
    return iv_cas->getViewBySofaNum(annSofaNum);
  }

  size_t AnnotationFS::getBeginPosition( void ) const {
    assert(EXISTS(iv_cas));
    assert(iv_cas->getHeap()->getIntValue(iv_tyFS, uima::internal::gs_tyBeginPosFeature) >= 0);
    return(size_t)iv_cas->getHeap()->getIntValue(iv_tyFS, uima::internal::gs_tyBeginPosFeature);
  }


  size_t AnnotationFS::getEndPosition( void ) const {
    assert(EXISTS(iv_cas));
    assert(iv_cas->getHeap()->getIntValue(iv_tyFS, uima::internal::gs_tyEndPosFeature) >= 0);
    return(size_t)iv_cas->getHeap()->getIntValue(iv_tyFS, uima::internal::gs_tyEndPosFeature);
  }


  UnicodeStringRef AnnotationFS::getCoveredText( void ) const {
    if (isValid()) {
      AnnotationFS* grr = const_cast<AnnotationFS*>(this);
      CAS * myCas = grr->getView();
      if (myCas->iv_uiDocumentLength > 0) {
        assert( getEndPosition() <= myCas->iv_uiDocumentLength );
        size_t uiBegin = getBeginPosition();
        UChar const * puc = myCas->iv_cpDocument;
        puc += uiBegin;
        return UnicodeStringRef( puc, getEndPosition()-uiBegin );
      }
    }

    UIMA_EXC_THROW_NEW(InvalidFSObjectException,
                       UIMA_ERR_INVALID_FS_OBJECT,
                       UIMA_MSG_ID_EXC_INVALID_FS_OBJECT,
                       ErrorMessage(UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE),
                       ErrorInfo::recoverable
                      );
  }

  AnnotationFS AnnotationFS::getFirstCoveringAnnotation( Type ofType ) const {
    if (!(isValid() && ofType.isValid())) {
      return AnnotationFS(); //will be invalid
    }
    assert(EXISTS(iv_cas));
    /* Let's create an Index of Type typeOfType */
    ANIterator itOfType(getCAS().getAnnotationIndex(ofType).iterator());
    /* Declare an object instance for the covering annotation we are looking for
     If we find one, it will be initialized below, if we don't find one,
     we return the invalid (because uninitialized) variable */
    AnnotationFS fsCoveringAn;
    size_t uiBegPosFrom = getBeginPosition();
    size_t uiEndPosFrom = getEndPosition();

    /* Iterate over all annotations in the ofTypeIdx */
    AnnotationFS fsOfTypeAn;
    for (itOfType.moveToFirst(); itOfType.isValid(); itOfType.moveToNext()) {
      fsOfTypeAn = itOfType.get();
      size_t uiBegPosOf   = fsOfTypeAn.getBeginPosition();
      size_t uiEndPosOf   = fsOfTypeAn.getEndPosition();
      if ((uiBegPosOf <= uiBegPosFrom )  &&
          (uiEndPosOf >= uiEndPosFrom)) {
        if (fsOfTypeAn.getType() != getType()) {
          fsCoveringAn = fsOfTypeAn;
        }
      } else {
        if (uiBegPosOf > uiBegPosFrom) {
          return fsCoveringAn;  //shortcut - avoids unecessary iteration
        }
      }
    };
    return fsCoveringAn;
  }

  ANIterator AnnotationFS::subIterator( Type const & crType, EnIteratorAmbiguity enAmbiguous ) const {
    return getCAS().getAnnotationIndex(crType).subIterator(*this, enAmbiguous);
  }


  /*****************************************************************************/
  /*  DocumentFS                                                               */
  /*****************************************************************************/
  DocumentFS::DocumentFS(FeatureStructure const & fs)
      : AnnotationFS(fs) {
    assert( EXISTS(iv_cas) );
    assert( iv_cas->getHeap()->getTypeSystem().subsumes(
              iv_cas->getHeap()->getTypeSystem().getTypeByName(CAS::TYPE_NAME_DOCUMENT_ANNOTATION),
              iv_cas->getHeap()->getType(internal::FSPromoter::demoteFS(fs)) ) );
  }

  Language DocumentFS::getLanguage() const {
    assert(EXISTS(iv_cas));
    assert( iv_cas->getHeap()->isValid(iv_tyFS) );

    if ( iv_cas->getHeap()->getTypeSystem().isValidFeature(getCAS().iv_utDocumentLangAsIntFeat) ) {
      int iLangID = iv_cas->getHeap()->getIntValue(iv_tyFS, getCAS().iv_utDocumentLangAsIntFeat);
      if (iLangID != 0) {
        return Language( iLangID );
      }
    }
    assert( iv_cas->getHeap()->getTypeSystem().isValidFeature(getCAS().iv_utDocumentLangAsStrFeat) );
    UnicodeStringRef ustrrefLang = iv_cas->getHeap()->getStringValue(iv_tyFS, getCAS().iv_utDocumentLangAsStrFeat);
    return Language( ustrrefLang );
  }

  void DocumentFS::setLanguage(Language const & lang) {
    assert(EXISTS(iv_cas));
    assert( iv_cas->getHeap()->isValid(iv_tyFS) );

    if ( iv_cas->getHeap()->getTypeSystem().isValidFeature(getCAS().iv_utDocumentLangAsIntFeat) ) {
      iv_cas->getHeap()->setIntValue(iv_tyFS, getCAS().iv_utDocumentLangAsIntFeat, lang.asNumber());
    }
    assert( iv_cas->getHeap()->getTypeSystem().isValidFeature(getCAS().iv_utDocumentLangAsStrFeat) );
    int ref = iv_cas->getHeap()->addString( lang.asUnicodeString() );
    iv_cas->getHeap()->setStringValue(iv_tyFS, getCAS().iv_utDocumentLangAsStrFeat, ref);
  }


  /*****************************************************************************/
  /*  SubIterator                                                              */
  /*****************************************************************************/

  namespace lowlevel {

    class SubIterator : public IndexIterator {
    protected:
      uima::lowlevel::FSHeap * iv_heap;
      IndexIterator* iv_pIterator;
      size_t iv_uiBegPos;
      size_t iv_uiEndPos;
      TyFS   iv_tyMoveToFirstFS;
    public:
      SubIterator(uima::lowlevel::FSHeap & heap, size_t uiBeginPos, size_t uiEndPos, IndexIterator* pIterator):
          iv_heap(&heap),
          iv_pIterator(pIterator),
          iv_uiBegPos(uiBeginPos),
          iv_uiEndPos(uiEndPos),
          iv_tyMoveToFirstFS(0) {
        assert( EXISTS(iv_heap) );
        TyFSType    tyTAN  = iv_heap->getTypeSystem().getTypeByName(CAS::TYPE_NAME_ANNOTATION);
        // create the longest possible AN starting at uiBeginPos (must be first of all starting there)
        iv_tyMoveToFirstFS = iv_heap->createFS(tyTAN);
        iv_heap->setIntValue(iv_tyMoveToFirstFS, uima::internal::gs_tyBeginPosFeature, (int)iv_uiBegPos );
        iv_heap->setIntValue(iv_tyMoveToFirstFS, uima::internal::gs_tyEndPosFeature, INT_MAX );
      }

      SubIterator(uima::lowlevel::FSHeap & heap, TyFS tyAn, IndexIterator* pIterator):
          iv_heap(&heap),
          iv_pIterator(pIterator),
          iv_uiBegPos(iv_heap->getIntValue(tyAn, uima::internal::gs_tyBeginPosFeature)),
          iv_uiEndPos(iv_heap->getIntValue(tyAn, uima::internal::gs_tyEndPosFeature)),
          iv_tyMoveToFirstFS(tyAn) {}

      virtual ~SubIterator() {
        assert( EXISTS(iv_pIterator) );
        assert( EXISTS(iv_heap) );
        delete iv_pIterator;
      }

      void moveToFirst() {
        UIMA_TPRINT("lowlevel::SubIterator::moveToFirst() entered");
        assert( EXISTS(iv_pIterator) );
        assert( EXISTS(iv_heap) );
        iv_pIterator->moveTo( iv_tyMoveToFirstFS );
        if (isValid()) {
          if (iv_pIterator->get() == iv_tyMoveToFirstFS) {
            iv_pIterator->moveToNext();
          }
        }
      }

      void moveToNext() {
        UIMA_TPRINT("lowlevel::SubIterator::moveToNext() entered");
        assert( isValid() );
        iv_pIterator->moveToNext();
        if (isValid()) {
          if (iv_pIterator->get() == iv_tyMoveToFirstFS) {
            iv_pIterator->moveToNext();
          }
        }

      }

      void moveToPrevious() {
        assert( isValid() );
        iv_pIterator->moveToPrevious();
        if (isValid()) {
          if (iv_pIterator->get() == iv_tyMoveToFirstFS) {
            iv_pIterator->moveToPrevious();
          }
        }
      }

      void moveToLast() {
        UIMA_TPRINT("moveToLast() entered");
        moveToFirst();
        if (!isValid()) {
          return;
        }
        assert( isValid() );
        uima::lowlevel::TyFS lastValidFS = uima::lowlevel::FSHeap::INVALID_FS;
        while (isValid()) {
          lastValidFS = get();
          moveToNext();
        }
        assert( ! isValid() );
        assert(lastValidFS != uima::lowlevel::FSHeap::INVALID_FS);

        moveTo(lastValidFS);
        assert( isValid() );

        if (isValid()) {
          if (iv_pIterator->get() == iv_tyMoveToFirstFS) {
            iv_pIterator->moveToPrevious();
          }
        }


#ifdef DEBUG_VERBOSE
        UIMA_TPRINT("last FS passed over: ");
        iv_pFSSystem->getLowlevelFSHeap().printFS(cerr, lastValidFS);
        UIMA_TPRINT("last FS: ");
        iv_pFSSystem->getLowlevelFSHeap().printFS(cerr, get());
#endif
      }

      TyFS get() const {
          assert( isValid() );
          return iv_pIterator->get();
        }


      TyFSType getTyFSType() const {
        assert( isValid() );
        return iv_pIterator->getTyFSType();
      }

      bool isValid() const {
        assert( EXISTS(iv_heap) );
        assert( EXISTS(iv_pIterator) );
        if (!iv_pIterator->isValid()) {
          return false;
        }
        TyFS fsCurrent = iv_pIterator->get();
        size_t uiCurrBegPos = iv_heap->getIntValue(fsCurrent, uima::internal::gs_tyBeginPosFeature);
        // true if current begin is between begin and end pos of our span
        return(uiCurrBegPos >= iv_uiBegPos && uiCurrBegPos < iv_uiEndPos);
      }

      IndexIterator* clone() const {
        UIMA_TPRINT("lowlevel::SubIterator::clone() entered");
        assert( EXISTS(iv_heap) );
        assert( EXISTS(iv_pIterator) );
        // don't call 4-arg constructor here! Pass on the FS to be moved to
        IndexIterator * pResult = new SubIterator(*iv_heap, iv_tyMoveToFirstFS, iv_pIterator->clone());
        assert( EXISTS(pResult) );

        assert( isValid() == pResult->isValid() );
#ifndef NDEBUG
        if (isValid()) {
          assert( get() == pResult->get() );
        }
#endif
        return pResult;
      }

      bool moveTo(TyFS fs) {
        assert( EXISTS(iv_heap) );
        assert( EXISTS(iv_pIterator) );
        return iv_pIterator->moveTo(fs);
      }

    }
    ; // class SubIterator


    class UnAmbiguousSubIterator : public SubIterator {
    private:
    public:
      UnAmbiguousSubIterator(uima::lowlevel::FSHeap & heap, size_t uiBeginPos, size_t uiEndPos, IndexIterator* pIterator):
          SubIterator(heap, uiBeginPos, uiEndPos, pIterator) {}

      UnAmbiguousSubIterator(uima::lowlevel::FSHeap & heap, TyFS tyAn, IndexIterator* pIterator):
          SubIterator(heap, tyAn, pIterator) {}

      void moveToNext() {
        /*
        assert( isValid() );
        size_t uiLastEndPos = (size_t)iv_pFSSystem->getLowlevelFSHeap().getIntValue(SubIterator::get(), uima::internal::gs_tyEndPosFeature);
        size_t uiCurrBeginPos;
        do {
           SubIterator::moveToNext();
           if (!isValid()) {
              return;
           }
           uiCurrBeginPos = iv_pFSSystem->getLowlevelFSHeap().getIntValue(SubIterator::get(), uima::internal::gs_tyBeginPosFeature);
        } while (uiCurrBeginPos < uiLastEndPos);
        */
        assert( EXISTS(iv_heap) );
        assert( isValid() );

        size_t uiLastEndPos = (size_t)iv_heap->getIntValue(iv_pIterator->get(), uima::internal::gs_tyEndPosFeature);
        size_t uiCurrBeginPos;
        do {
          iv_pIterator->moveToNext();
          if (!isValid()) {
            return;
          }
          uiCurrBeginPos = iv_heap->getIntValue(iv_pIterator->get(), uima::internal::gs_tyBeginPosFeature);
        } while (uiCurrBeginPos < uiLastEndPos);

      }

      IndexIterator* clone() const {
        assert( EXISTS(iv_heap) );
        assert( EXISTS(iv_pIterator) );
        return new UnAmbiguousSubIterator(*iv_heap, iv_tyMoveToFirstFS, iv_pIterator->clone());
      }

    }
    ; // class UnAmbiguousSubIterator

    class UnambiguousIterator : public IndexIterator {
    protected:
      uima::lowlevel::FSHeap* iv_heap;
      IndexIterator* iv_pIterator;
    public:
      UnambiguousIterator(uima::lowlevel::FSHeap & heap, IndexIterator* pIterator):
          iv_heap(&heap),
          iv_pIterator(pIterator) {
        assert( EXISTS(iv_heap) );
      }

      virtual ~UnambiguousIterator() {
        assert( EXISTS(iv_pIterator) );
        assert( EXISTS(iv_heap) );
        delete iv_pIterator;
      }

      void moveToFirst() {
        assert( EXISTS(iv_pIterator) );
        assert( EXISTS(iv_heap) );
        iv_pIterator->moveToFirst();
      }

      void moveToNext() {
        assert( isValid() );
        size_t uiLastEndPos = (size_t)iv_heap->getIntValue(iv_pIterator->get(), uima::internal::gs_tyEndPosFeature);
        size_t uiCurrBeginPos;
        do {
          iv_pIterator->moveToNext();
          if (!isValid()) {
            return;
          }
          uiCurrBeginPos = iv_heap->getIntValue(iv_pIterator->get(), uima::internal::gs_tyBeginPosFeature);
        } while (uiCurrBeginPos < uiLastEndPos);
      }

      void moveToPrevious() {
        assertWithMsg(false, "Not implemented yet!");
        UIMA_EXC_THROW_NEW(NotYetImplementedException,
                           UIMA_ERR_NOT_YET_IMPLEMENTED,
                           UIMA_MSG_ID_EXC_NOT_YET_IMPLEMENTED,
                           ErrorMessage(UIMA_MSG_ID_EXCON_UNKNOWN_CONTEXT),
                           ErrorInfo::unrecoverable
                          );
      }

      void moveToLast() {
        assertWithMsg(false, "Not implemented yet!");
        UIMA_EXC_THROW_NEW(NotYetImplementedException,
                           UIMA_ERR_NOT_YET_IMPLEMENTED,
                           UIMA_MSG_ID_EXC_NOT_YET_IMPLEMENTED,
                           ErrorMessage(UIMA_MSG_ID_EXCON_UNKNOWN_CONTEXT),
                           ErrorInfo::unrecoverable
                          );
      }

      TyFS get() const {
          assert( isValid() );
          return iv_pIterator->get();
        }

      TyFSType getTyFSType() const {
        assert( isValid() );
        return iv_pIterator->getTyFSType();
      }

      bool isValid() const {
        assert( EXISTS(iv_heap) );
        assert( EXISTS(iv_pIterator) );
        if (!iv_pIterator->isValid()) {
          return false;
        }
        return true;
      }

      IndexIterator* clone() const {
        assert( EXISTS(iv_heap) );
        assert( EXISTS(iv_pIterator) );
        return new UnambiguousIterator(*iv_heap, iv_pIterator->clone());
      }

      bool moveTo(TyFS fs) {
        assert( EXISTS(iv_heap) );
        assert( EXISTS(iv_pIterator) );
        return iv_pIterator->moveTo(fs);
      }

    }
    ; // class UnambiguousIterator
  } // namespace lowlevel


  ANIterator ANIndex::subIterator( AnnotationFS const & an, EnIteratorAmbiguity enAmbiguous ) const {
    checkValidity();
    lowlevel::IndexIterator* pitBase = iv_pIndex->createIterator();
    assert( EXISTS(pitBase) );

    lowlevel::IndexIterator* pitSub;
    if (enAmbiguous == enUnambiguous) {
      pitSub = new lowlevel::UnAmbiguousSubIterator( iv_indexRepository->getFSHeap(), internal::FSPromoter::demoteFS(an), pitBase);
    } else {
      assert(enAmbiguous == enAmbiguous);
      pitSub = new lowlevel::SubIterator(iv_indexRepository->getFSHeap(), internal::FSPromoter::demoteFS(an), pitBase);
    }
    assert( EXISTS(pitSub) );

    return ANIterator(pitSub, &iv_indexRepository->getCas());
  }

  ANIterator ANIndex::unambiguousIterator() const {
    checkValidity();
    lowlevel::IndexIterator* pitBase = iv_pIndex->createIterator();
    assert( EXISTS(pitBase) );

    lowlevel::IndexIterator* pitSub;
    pitSub = new lowlevel::UnambiguousIterator(iv_indexRepository->getFSHeap(), pitBase);
    assert( EXISTS(pitSub) );

    return ANIterator(pitSub, &iv_indexRepository->getCas());
  }

}

/* ----------------------------------------------------------------------- */





