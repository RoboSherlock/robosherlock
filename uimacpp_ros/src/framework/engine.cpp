/** \file engine.cpp .
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
#define UIMA_ENGINE_MAIN_CPP

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas

#include <memory>

#include <uima/macros.h>

#include <uima/engine.hpp>
#include <uima/strconvert.hpp>
#include <uima/internal_aggregate_engine.hpp>
#include <uima/internal_primitive_engine.hpp>
#include <uima/internal_jedii_engine.hpp>
#include <uima/taespecifierbuilder.hpp>
#include <uima/internal_casimpl.hpp>
#include <uima/resmgr.hpp>
#include <uima/msg.h>
#include <uima/casdefinition.hpp>
#include "xercesc/framework/LocalFileInputSource.hpp"
#include "xercesc/framework/MemBufInputSource.hpp"
#include <uima/stltools.hpp>
XERCES_CPP_NAMESPACE_USE

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

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
namespace uima {

  UIMA_EXC_CLASSIMPLEMENT(UnknownTypeException, Exception);
  UIMA_EXC_CLASSIMPLEMENT(UnknownFeatureException, Exception);
  UIMA_EXC_CLASSIMPLEMENT(UnknownRangeTypeException, Exception);
  UIMA_EXC_CLASSIMPLEMENT(IncompatibleRangeTypeException, Exception);
  UIMA_EXC_CLASSIMPLEMENT(AllowedStringValuesIncompatibleException, Exception);
  UIMA_EXC_CLASSIMPLEMENT(TypePriorityConflictException, Exception);
  UIMA_EXC_CLASSIMPLEMENT(IncompatibleIndexDefinitionsException, Exception);
  UIMA_EXC_CLASSIMPLEMENT(IncompatibleParentTypesException, Exception);
  UIMA_EXC_CLASSIMPLEMENT(CASIncompatibilityException, Exception);

  
  AnalysisEngineMetaData const & AnalysisEngine::getAnalysisEngineMetaData() const {
    assert( EXISTS(getAnnotatorContext().getTaeSpecifier().getAnalysisEngineMetaData()) );
    return *getAnnotatorContext().getTaeSpecifier().getAnalysisEngineMetaData();
  }

  TyErrorId AnalysisEngine::process(CAS & tcas) {
    return ((AnalysisEngine*) this)->process(tcas);
  }

  TyErrorId AnalysisEngine::process(CAS & tcas, ResultSpecification const & resultSpec) {
    return ((AnalysisEngine*) this)->process(tcas, resultSpec);
  }

  CASIterator  AnalysisEngine::processAndOutputNewCASes(CAS & tcas) {
    TyErrorId rc = ((AnalysisEngine*) this)->process(tcas);
    if (rc != UIMA_ERR_NONE) {
      UIMA_EXC_THROW_NEW(CASIteratorException,
                         UIMA_ERR_ENGINE_INVALID_CALLING_SEQUENCE,
                         UIMA_MSG_ID_EXCON_CALLING_ANNOTATOR_FUNCTION,
                         UIMA_MSG_ID_EXCON_CALLING_ANNOTATOR_FUNCTION,
                         ErrorInfo::unrecoverable);
    }
    return CASIterator(this);
  }

  /* static */ const char * AnalysisEngine::getVersionInfo(void)
  /* ----------------------------------------------------------------------- */
  {
    return(UIMA_STRINGIFY(UIMA_VERSION));
  }

  /* static */ const char * AnalysisEngine::getLevelInfo(void)
  /* ----------------------------------------------------------------------- */
  {
    return(TextAnalysisEngine::getVersionInfo());
  }

  /* static */ const char * AnalysisEngine::getErrorIdAsCString(TyErrorId utErrorId)
  /* ----------------------------------------------------------------------- */
  {
    const StErrorId2StringMapping * cpstErrorId2StringMapping = gs_astErrorId2StringMapping;
    size_t u;

    /* originally, we had a real big switch/case statement but this caused an
       "internal error" with the SUN compiler.
       now we perform a linear search - this method is not considered time critical */
    for (u = 0; u < NUMBEROF(gs_astErrorId2StringMapping); u++) {
      assert(EXISTS(cpstErrorId2StringMapping));
      if (utErrorId == cpstErrorId2StringMapping->iv_utErrorId) {
        assert(EXISTS(cpstErrorId2StringMapping->iv_cpszErrorId));
        return(cpstErrorId2StringMapping->iv_cpszErrorId);
      }
      ++cpstErrorId2StringMapping;
    }
#ifdef NDEBUG
    return(_TEXT("UIMA_ERR_UNKNOWN"));
#else
    string *            pstrRetVal = new string(_TEXT("??? UNKNOWN UIMACPP ERROR ID:")); /* this memory gets wasted - but this is only NDEBUG code!!! */
    string              strValue;

    long2String((long) utErrorId, strValue);
    assert(EXISTS(pstrRetVal));
    *pstrRetVal += strValue;
    *pstrRetVal += _TEXT(" - Please update file '" __FILE__ "' ???");
    /* this memory gets wasted - but this is only NDEBUG code!!! */
    return(pstrRetVal->c_str());  //lint !e429: Custodial pointer 'pstrRetVal' (line 596) has not been freed or returned
#endif
  }

  /* static */ void AnalysisEngine::printErrorIdTable(ostream & rclOutStream)
  /* ----------------------------------------------------------------------- */
  {
    const StErrorId2StringMapping * cpstErrorId2StringMapping = gs_astErrorId2StringMapping;
    size_t                     u;

    /* originally, we had a real big switch/case statement but this caused an
       "internal error" with the SUN compiler.
       now we perform a linear search - this method is not considered time critical */
    for (u = 0; u < NUMBEROF(gs_astErrorId2StringMapping); u++) {
      assert(EXISTS(cpstErrorId2StringMapping));
      assert(EXISTS(cpstErrorId2StringMapping->iv_cpszErrorId));
      rclOutStream << cpstErrorId2StringMapping->iv_utErrorId
      << " = "
      << cpstErrorId2StringMapping->iv_cpszErrorId
      << "\n";
      ++cpstErrorId2StringMapping;
    }
  }


  /**  
   * All variants of creating a TAE will eventually be mapped to this call.
   * Depending on the context, memory ownership over the different objects passed to this 
   * method may vary, thus the boolean parameters.
   * Toplevel TAEs alwasy own the ANC and the CAS Definition but if they are created
   * with an external TAESpec (see API calls for createTextAnalysisEngine()), they don't 
   * own this object. On the other hand, delegate AEs never own neither the ANC, the TAESpec
   * or the CASDefinition. 
   */
  /*static*/ AnalysisEngine * Framework::createAnalysisEngine(AnnotatorContext & rANC,
      bool bOwnsANC,
      bool bOwnsTAESpecifier,
      uima::internal::CASDefinition & casDefinition,
      bool ownsCASDefintion,
      ErrorInfo & rErrorInfo) {
    AnalysisEngine * pResult = NULL;
    assert( rErrorInfo.getErrorId() == UIMA_ERR_NONE );
    try {
      if (!bOwnsANC) {
        assert( ! bOwnsTAESpecifier );
      }

      // create the engine depending on the framework (UIMACPP or JEDII) or if it is primitive or aggregate.
      AnalysisEngineDescription const & crTAESpecifier = rANC.getTaeSpecifier();
      if (crTAESpecifier.getFrameworkImplName() == AnalysisEngineDescription::JAVA) {
        pResult = new uima::internal::JEDIIEngine( rANC, bOwnsANC, bOwnsTAESpecifier, casDefinition, ownsCASDefintion );
      } else {
        if (crTAESpecifier.isPrimitive()) {
          pResult = new uima::internal::PrimitiveEngine( rANC, bOwnsANC, bOwnsTAESpecifier, casDefinition, ownsCASDefintion );
        } else {
          pResult = new uima::internal::AggregateEngine( rANC, bOwnsANC, bOwnsTAESpecifier, casDefinition, ownsCASDefintion );
        }
      }

      if (pResult == NULL) {
        rErrorInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
        return NULL;
      }

      assert( EXISTS(pResult) );
      TyErrorId utErrorID = pResult->initialize( crTAESpecifier );
      if (utErrorID != UIMA_ERR_NONE) {
        ErrorInfo const & crLastError = pResult->getAnnotatorContext().getLogger().getLastError();
        if (crLastError.getErrorId() != UIMA_ERR_NONE) {
          rErrorInfo = crLastError;
        }
        // overwrite the error ID
        rErrorInfo.setErrorId( utErrorID );
        delete pResult;
        return NULL;
      }

      assert( EXISTS(pResult) );

      return  pResult;
    } catch (Exception & rExc) {
      rErrorInfo = rExc.getErrorInfo();
    }
    assert( rErrorInfo.getErrorId() != UIMA_ERR_NONE );
    if (pResult != NULL) {
      delete pResult;
    }
    return NULL;
  }


  TextAnalysisEngine * TextAnalysisEngine::createTAE(bool bIsFilename,
      icu::UnicodeString const & crString,
      ErrorInfo & rErrorInfo) {
		  return (TextAnalysisEngine*) Framework::createAnalysisEngine(bIsFilename,
			  crString, rErrorInfo);
	  }

    /*static*/
  /**   
   * create a TAE from a file name or an XML buffer (depending on first argument).
   */
  AnalysisEngine * Framework::createAnalysisEngine(bool bIsFilename,
      icu::UnicodeString const & crString,
      ErrorInfo & rErrorInfo) {
    try {
      rErrorInfo.reset();
      if (! ResourceManager::hasInstance()) {
        rErrorInfo.setErrorId(UIMA_ERR_ENGINE_RESMGR_NOT_INITIALIZED);
        return NULL;
      }

      /*
      We use quite a lot of auto_ptrs here because many of those methods can go wrong
      and throw exceptions but we still want to clean up all the memory we used thus far.
      */
      
	  XMLParser builder;
      auto_ptr<AnalysisEngineDescription> apTAESpecifier( new AnalysisEngineDescription() );
      if (apTAESpecifier.get() == NULL) {
        rErrorInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
        return NULL;
      }

      if (bIsFilename) {
        builder.parseAnalysisEngineDescription(* apTAESpecifier.get(), crString);
      } else {
        /*
           Some XML4C routines which have to do wiht parsing an in-memory 
           buffer don't work, so this is why we don't call
             builder.buildTaeFromMemory(* apTAESpecifier.get(), crString);
           here but rather create a temp file.
        */
        // Use what appears to be a thread-safe routine on Linux & Windows
        char* tmpFileName = tempnam(NULL, "TMP");

        ofstream ofs(tmpFileName);
        uima::operator<<(ofs, crString);
        ofs << endl;
        ofs.close();
        UIMA_TPRINT("Wrote descriptor to temp file:");
        UIMA_TPRINT(tmpFileName);
        builder.parseAnalysisEngineDescription(* apTAESpecifier.get(), tmpFileName);

        remove(tmpFileName);                // or unlink ??
        free(tmpFileName);                  // Free storage allocated by tempnam
      }

      apTAESpecifier->validate();
      apTAESpecifier->commit();

      auto_ptr<AnnotatorContext> apANC( new AnnotatorContext(apTAESpecifier.get()) );
      if (apANC.get() == NULL) {
        rErrorInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
        return NULL;
      }

      assert( EXISTS(apANC.get()) );

      auto_ptr<uima::internal::CASDefinition> apCASDef( uima::internal::CASDefinition::createCASDefinition(*apANC.get()) );
      // release auto_ptrs here because the createTAE transfers ownership to the engine
      //  Warning: this could cause a memory leak if the createTAE() method coudl not create
      //           the actual engine object.     suhre 02/11/03
      apTAESpecifier.release();
      AnalysisEngine * pResult = createAnalysisEngine(* apANC.release(), true,
                                     true,
                                     * apCASDef.release(), true,
                                     rErrorInfo);
           
      return pResult;
    } catch (Exception & rExc) {
      rErrorInfo = rExc.getErrorInfo();
    }
    assert( rErrorInfo.getErrorId() != UIMA_ERR_NONE );
    return NULL;
  }


     /*static*/
  TextAnalysisEngine * TextAnalysisEngine::createTextAnalysisEngine(char const * cpFileName, ErrorInfo & rErrorInfo) {
    icu::UnicodeString usFileName( cpFileName );
	return (TextAnalysisEngine*)Framework::createAnalysisEngine(cpFileName, rErrorInfo);
  }
    /*static*/
  AnalysisEngine * Framework::createAnalysisEngine(char const * cpFileName, ErrorInfo & rErrorInfo) {
    icu::UnicodeString usFileName( cpFileName );
    return createAnalysisEngine(true, usFileName, rErrorInfo);
  }


    /*static*/
  TextAnalysisEngine * TextAnalysisEngine::createTextAnalysisEngine(UChar const * cpBuffer, size_t uiLength, ErrorInfo & rErrorInfo) {
    
	  return (TextAnalysisEngine*)Framework::createAnalysisEngine(cpBuffer,uiLength, rErrorInfo);
  }

    /*static*/
  AnalysisEngine * Framework::createAnalysisEngine(UChar const * cpBuffer, 
				size_t uiLength, ErrorInfo & rErrorInfo) {
    // read-only constructor of unicode string
    icu::UnicodeString usXMLBuffer(false, cpBuffer, uiLength);
    return createAnalysisEngine(false, usXMLBuffer, rErrorInfo);
  }

    /*static*/
  TextAnalysisEngine * TextAnalysisEngine::createTextAnalysisEngine(AnalysisEngineDescription & crAEDesc, 
									ErrorInfo & rErrorInfo) {
		return (TextAnalysisEngine*) Framework::createAnalysisEngine(crAEDesc, rErrorInfo);
  }

    /*static*/
  AnalysisEngine * Framework::createAnalysisEngine(AnalysisEngineDescription & crTAESpec, ErrorInfo & rErrorInfo) {
    try {
      rErrorInfo.reset();
      if (! ResourceManager::hasInstance()) {
        rErrorInfo.setErrorId(UIMA_ERR_ENGINE_RESMGR_NOT_INITIALIZED);
        return NULL;
      }

      auto_ptr<AnnotatorContext> apANC( new AnnotatorContext(& crTAESpec) );
      if (apANC.get() == NULL) {
        rErrorInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
        return NULL;
      }

      assert( EXISTS(apANC.get()) );

      auto_ptr<uima::internal::CASDefinition> apCASDef( uima::internal::CASDefinition::createCASDefinition(*apANC.get()) );
      // release auto_ptrs here because the createTAE transfers ownership to the engine
      //  Warning: this could cause a memory leak if the createTAE() method coudl not create (construct)
      //           the actual engine object.     suhre 02/11/03
       AnalysisEngine * pResult = createAnalysisEngine(*apANC.release(), true,
                                     false,
                                     *apCASDef.release(), true,
                                     rErrorInfo);
      return pResult;
    } catch (Exception & rExc) {
      rErrorInfo = rExc.getErrorInfo();
    }
    return NULL;
  }




  /* ------------------------------------------------
   *  uima::Framework methods
   * ----------------------------------------------- */
  //create typesystem and set type priorities
  TypeSystem * Framework::createTypeSystem(AnalysisEngineMetaData const & ae, ErrorInfo& rErrorInfo) {
    rErrorInfo.reset();
    if (! ResourceManager::hasInstance()) {
      rErrorInfo.setErrorId(UIMA_ERR_ENGINE_RESMGR_NOT_INITIALIZED);
      return NULL;
    }

    uima::TypeSystem * pTs =  uima::internal::CASDefinition::createTypeSystem(ae);

    return pTs;
  }

  //create typesystem from description
  TypeSystem * Framework::createTypeSystem(TypeSystemDescription const & tsDesc,
      icu::UnicodeString const & creatorID,
      ErrorInfo& rErrorInfo) {
    rErrorInfo.reset();
    if (! ResourceManager::hasInstance()) {
      rErrorInfo.setErrorId(UIMA_ERR_ENGINE_RESMGR_NOT_INITIALIZED);
      return NULL;
    }

    uima::TypeSystem * pTs =  uima::internal::CASDefinition::createTypeSystem(tsDesc,creatorID);

    return pTs;
  }

  //create typesystem from description and set type priorities
  TypeSystem * Framework::createTypeSystem(TypeSystemDescription const & tsDesc,
      uima::AnalysisEngineMetaData::TyVecpTypePriorities const & typePriorities,
      icu::UnicodeString const & creatorID,
      ErrorInfo& rErrorInfo) {
    rErrorInfo.reset();
    if (! ResourceManager::hasInstance()) {
      rErrorInfo.setErrorId(UIMA_ERR_ENGINE_RESMGR_NOT_INITIALIZED);
      return NULL;
    }

    uima::TypeSystem * pTs =  uima::internal::CASDefinition::createTypeSystem(tsDesc,typePriorities,creatorID);

    return pTs;
  }


  //read in descriptor from and create TypeSystem per the specification
  TypeSystem * Framework::createTypeSystem(char const * crFileName, ErrorInfo& rErrorInfo) {

    XMLParser builder;
    AnalysisEngineMetaData * pAe = new AnalysisEngineMetaData();
	TypeSystemDescription * tsDesc = new TypeSystemDescription();
	
	UnicodeString ufn(crFileName);
    size_t uiLen = ufn.length();
    auto_array<UChar> arBuffer( new UChar[uiLen + 1] );
    assert( EXISTS(arBuffer.get()));
    ufn.extract(0, uiLen, arBuffer.get());
    (arBuffer.get())[uiLen] = 0; // terminate the buffer with 0

	LocalFileInputSource fileIS((XMLCh const *) arBuffer.get());

	builder.parseTypeSystemDescription(*tsDesc, fileIS);
    pAe->setTypeSystemDescription(tsDesc);

    auto_ptr<AnalysisEngineMetaData> apSpecifier(pAe  );
    if (apSpecifier.get() == NULL) {
      rErrorInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
      return NULL;
    }

    uima::TypeSystem * pTs =  uima::internal::CASDefinition::createTypeSystem(*apSpecifier.get());
    return pTs;
  }


  //read in xml descriptor from buffer and create typesystem per those specifications
  TypeSystem * Framework::createTypeSystem(UChar const * cpBuffer,
      size_t uiLength,
      ErrorInfo& rErrorInfo) {
    UnicodeStringRef usRef(cpBuffer, uiLength);
    XMLParser builder;
    AnalysisEngineMetaData * pAe = new AnalysisEngineMetaData();
	TypeSystemDescription * tsDesc = new TypeSystemDescription();
    MemBufInputSource memIS((XMLByte const *)usRef.asUTF8().c_str(),
		                    usRef.asUTF8().length(),
							"sysID");
	builder.parseTypeSystemDescription(*tsDesc,memIS);
    pAe->setTypeSystemDescription(tsDesc);

    auto_ptr<AnalysisEngineMetaData> apSpecifier(pAe  );
    if (apSpecifier.get() == NULL) {
      rErrorInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
      return NULL;
    }

    uima::TypeSystem * pTs =  uima::internal::CASDefinition::createTypeSystem(*apSpecifier.get());
    return pTs;

  }

  //read in xml descriptor from buffer and create typesystem per those specifications
  TypeSystem * Framework::createTypeSystemFromXMLBuffer( char const * cpBuffer,
      ErrorInfo& rErrorInfo) {
    XMLParser builder;
    AnalysisEngineMetaData * pAe = new AnalysisEngineMetaData();
	MemBufInputSource memIS((XMLByte const *) cpBuffer, strlen(cpBuffer), "sysID");
	TypeSystemDescription * tsDesc = new TypeSystemDescription();
	builder.parseTypeSystemDescription(*tsDesc, memIS);
    pAe->setTypeSystemDescription(tsDesc);

    auto_ptr<AnalysisEngineMetaData> apSpecifier(pAe  );
    if (apSpecifier.get() == NULL) {
      rErrorInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
      return NULL;
    }

    uima::TypeSystem * pTs =  uima::internal::CASDefinition::createTypeSystem(*apSpecifier.get());
    return pTs;

  }



  //create CAS with specified typesystem and only built in indices
  CAS * Framework::createCAS(TypeSystem & typesystem, ErrorInfo& rErrorInfo) {
    auto_ptr<uima::internal::CASDefinition> apCASDef( uima::internal::CASDefinition::createCASDefinition(typesystem) );
    if (apCASDef.get() == NULL) {
      rErrorInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
      return NULL;
    }

    CAS * pCas = uima::internal::CASImpl::createCASImpl(*apCASDef.release(), true);

    return pCas->getInitialView();
  }



  //create CAS with specified typesystem and indices/typePriorities defined in the AE descriptor
  CAS * Framework::createCAS(TypeSystem & typesystem, AnalysisEngineMetaData & aeDesc, ErrorInfo & rErrorInfo) {
    auto_ptr<uima::internal::CASDefinition> apCASDef( uima::internal::CASDefinition::createCASDefinition(typesystem, aeDesc) );
    if (apCASDef.get() == NULL) {
      rErrorInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
      return NULL;
    }
    CAS * pCas = uima::internal::CASImpl::createCASImpl(*apCASDef.release(), true);
    return pCas->getInitialView();
  }


  CAS * Framework::createCAS(TypeSystem & typesystem,
                             AnalysisEngineMetaData::TyVecpFSIndexDescriptions & fsIndexDesc,
                             AnalysisEngineMetaData::TyVecpTypePriorities  & prioDesc,
                             ErrorInfo & rErrorInfo)  {
    auto_ptr<uima::internal::CASDefinition>
    apCASDef( uima::internal::CASDefinition::createCASDefinition(typesystem, fsIndexDesc, prioDesc) );
    if (apCASDef.get() == NULL) {
      rErrorInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
      return NULL;
    }
    CAS * pCas = uima::internal::CASImpl::createCASImpl(*apCASDef.release(), true);
    return pCas->getInitialView();


  }

  //create TypeSystemDescription object from xml spec in buffer
  TypeSystemDescription * Framework::createTypeSystemDescription(UChar const * cpBuffer, size_t uiLength) {
    UnicodeStringRef usRef(cpBuffer, uiLength);
    XMLParser builder;
    TypeSystemDescription * tsDesc = new TypeSystemDescription();
    MemBufInputSource memIS((XMLByte const *) usRef.asUTF8().c_str(), 
		                     usRef.asUTF8().length(), "sysID");
	builder.parseTypeSystemDescription(*tsDesc,memIS);
    return tsDesc;
  }

  //read in xml from a file and create a TypeSystemDescription object
  TypeSystemDescription * Framework::createTypeSystemDescription(char const * crFileName) {
    XMLParser builder;
	LocalFileInputSource fileIS((XMLCh const *)crFileName);
	TypeSystemDescription * tsDesc = new TypeSystemDescription();
    builder.parseTypeSystemDescription(*tsDesc,fileIS);
	return tsDesc;
  }


  CAS * Framework::createCAS(uima::internal::CASDefinition & casdef, ErrorInfo & rErrorInfo) {
    CAS * pCas = uima::internal::CASImpl::createCASImpl(casdef, false);
    if (pCas == NULL) {
      rErrorInfo.setErrorId(UIMA_ERR_ENGINE_OUT_OF_MEMORY);
      return NULL;
    }
    return pCas->getInitialView();
  }






}


/* ----------------------------------------------------------------------- */





