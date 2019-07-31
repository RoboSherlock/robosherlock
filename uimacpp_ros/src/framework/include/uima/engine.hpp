/** \file engine.hpp .
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

    \brief Contains Engine the central UIMACPP Engine object

-----------------------------------------------------------------------------


   4/27/1999  Initial creation

-------------------------------------------------------------------------- */

#ifndef UIMA_ENGINE_HPP
#define UIMA_ENGINE_HPP

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> //must be included first to disable warnings
#include <vector>
#include <uima/err_ids.h>
#include <uima/annotator_timing.hpp>
#include "unicode/uchar.h"
#include "unicode/unistr.h"
#include <uima/exceptions.hpp>
#include <uima/taespecifier.hpp>
#include <uima/casdefinition.hpp>
#include <uima/casiterator.hpp>
#include <uima/consoleui.hpp>

/** \mainpage UIMA C++ API Documentation
   @htmlonly
   <table cellpadding="10" >
   <tr>
   <td valign="top" bgcolor="lavender" width="20%">
   @endhtmlonly
   <strong>Useful Links</strong>
   <p>
   <small>
   Annotator interfaces:<br>
   uima::Annotator <br>
   <br>
   Configuration parameters access:<br>
   uima::AnnotatorContext <br>
   <br>
   CAS and TypeSystem APIs:<br>
   uima::CAS <br>
   uima::TypeSystem <br>
   <br>
   Generic feature structure access:<br>
   uima::FSIndex <br>
   uima::FSIterator <br>
   uima::FeatureStructure <br>
   <br>
   Specalized annotation access:<br>
   uima::ANIndex <br>
   uima::ANIterator <br>
   uima::AnnotationFS <br>
   uima::DocumentFS <br>
   <br>
   Sofa access:<br>
   uima::SofaFS <br>
   uima::SofaDataStream <br>
   <br>
   Application:<br>
   uima::ResourceManager <br>
   uima::Framework <br>
   uima::AnalysisEngine <br>
   <br>
   </small>
   @htmlonly
   </td>
   <td valign="top">
   @endhtmlonly
   The <strong>C++ Enablement for UIMA</strong> provides transparent interoperability for UIMA C++ analytics to run with
   the Java UIMA SDK platform. Supported components include primitive and aggregate Analysis Engines
   and Cas Consumers.
   <br>
   <br>
   Please refer to the <strong>UIMA SDK User's Guide and Reference </strong> for an introduction to
   the UIMA SDK and as well as instructions on how to create, run and manage UIMA analytics. The
   important APIs that support development of UIMA C++ analytics are described below.
   <br>
   <br>
   The <strong> Annotator </strong> ( uima::Annotator ) class defines the interface that must be implemented
   by primitive UIMA C++ components. The C++ component descriptor must specify the framework implementation
   as follows:
   <br>
   <br>
   @code 
   <frameworkImplementation>org.apache.uima.cpp</frameworkImplementation> 
   @endcode   
   <br>
   An <strong>application</strong> that runs a UIMA analysis engine must instantiate a uima::AnalysisEngine from a
   UIMA component descriptor and call its process method using APIs shown below:
   <br>
   @code
 (void) ResourceManager::createInstance("My UIMA Application"); //singleton
 AnalysisEngine * pEngine =
  Framework::createAnalysisEngine(descriptorfn, errorInfo); //engine
    CAS* cas = pEngine-newCAS(); //create a CAS
 //use CAS APIs to add data to be analysed
 TyErrorId utErrorId = pEngine->process(*cas);//call process
 cas->reset(); //reset CAS before reuse
 utErrorId = pEngine->destroy();  //free resources
   @endcode
   Example annotator and application code can be found in the docs/example directory of the
   UIMACPP distribution.
   <br>
   <br>
   The <strong> AnnotatorContext </strong> ( uima::AnnotatorContext ) class associated with an AnalysisEngine
   provides access to the configuration parameters and other metadata about the Analysis Engine.
   <br>
   <br>
   The <strong>CAS</strong> ( uima::CAS ) object consists of the data structure and set of APIs to support the
   representation of the <strong> TypeSystem </strong> and instances of these types <strong>FeatureStructures</strong>.
   The CAS APIs are virtually identical to those in the Java implementation. To support interoperability,
   the CAS is serialized into the native environment through the JNI.
   <br>
   <br>
   The <strong>Subject of Analysis or Sofa </strong> capability is supported by the uima::SofaFS class.
   Use uima::CAS::createView() to create a view of a Sofa and uima::CAS::getView() to access the
   view. 
   <br>
   <br>The <strong> SofaDataStream </strong> ( uima::SofaDataStream ) class provides stream access
   to Sofa data.  Use the method uima::SofaFS::getSofaDataStream() to get a handle to a uima::SofaDataStream object.  The support for
   stream access to local sofa data is built into the UIMACPP library. Handlers for other URI schemes must be
   registered with the framework. The handler for the file URI scheme can be found in the UIMACPP distribution.
   To develop custom URI schemes, implement the interface defined in sofastreamhandler.hpp and
   build a DLL.  To make the handlers available to the framework, register
   the handler by setting the UIMACPP_STREAMHANDLERS environment variable as follows:
   <br>
   <br>
   @code
   Windows:
 set UIMACPP_STREAMHANDLERS=file:SofaStreamHandlerFile myscheme:mylibrary
   Linux:
 export UIMACPP_STREAMHANDLERS=file:SofaStreamHandlerFile myscheme:mylibrary
   @endcode
   <br>
   @htmlonly
   </td>
   <td valign="top" halign="center">
   @endhtmlonly
   @htmlonly
   </td>
   </tr>
   </table>
   @endhtmlonly
 *
 */

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/**
 * This is the main namespace we use for all of our published API.
 *
 * The only exception to this are defines which due to restrictions of the
 * C++ language can not be put in namespaces (to make up for the missing
 * namespace all defines are prefixed with <code>UIMA</code>)
 */
namespace uima {
  class UnicodeStringRef;
  class AnalysisEngineDescription;
  class AnalysisEngineDescription;
  class ResultSpecification;
  class AnalysisEngineMetaData;
  class AnnotatorContext;
  class Capability;
  class Language;
  class Timer;
  class ErrorInfo;
  class CAS;
  class TCAS;
  class LogFacility;
  class TypeSystem;
  class TypeSystemDescription;
  class TyVecpTypePriorities;
  class CASIterator;
  class JNIInstance;

  /**
   * This namespace is only to be used internally to UIMACPP.
   *
   * Any use of classes and functions from this namespace even if exported
   * by the UIMACPP DLL is discouraged
   *
   * @internal
   */
  namespace internal {
    class DeprecatedTAEDecorator;
    class AnnotatorManager;
    class CASDefinition;
  }
  /**
   * This namespace contains lowlevel access functions not conforming to
   * UIMA API and not covered by standard documentation and tutorial.
   *
   * You should use functions from this namespace only if performance is
   * absolutely critical.
   *
   * @internal
   */
  namespace lowlevel {}
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  UIMA_EXC_CLASSDECLARE(UnknownTypeException, Exception);
  UIMA_EXC_CLASSDECLARE(UnknownFeatureException, Exception);
  UIMA_EXC_CLASSDECLARE(UnknownRangeTypeException, Exception);
  UIMA_EXC_CLASSDECLARE(IncompatibleRangeTypeException, Exception);
  UIMA_EXC_CLASSDECLARE(AllowedStringValuesIncompatibleException, Exception);
  UIMA_EXC_CLASSDECLARE(TypePriorityConflictException, Exception);
  UIMA_EXC_CLASSDECLARE(IncompatibleIndexDefinitionsException, Exception);
  UIMA_EXC_CLASSDECLARE(IncompatibleParentTypesException, Exception);
  UIMA_EXC_CLASSDECLARE(CASIncompatibilityException, Exception);
  UIMA_EXC_CLASSDECLARE(UimaAnalysisComponentException, Exception);

  /**
   * An <code>AnalysisEngine</code> can be used for analyzing all kinds of unstructured
   * information.
   * It is mainly designed to work well with text but it should also work
   * with other kinds of unstructured information like life sciences data.
   *
   * An analysis engine is a mediator between an application interested
   * in the analysis results for some piece of unstructured input data and
   * the so called annotators that actually produce this analysis result.
   *
   * All results -intermediate and final- are passed through the
   * Common Analysis Structure (CAS) linking the application and the
   * annotators.
   *
   * An AnalysisEngine hides the complexities of computing the results and
   * the flow of control between all annotators working on the results from
   * the application.
   *
   *
   * For text analysis, use uima::TextAnalysisEngine.
   *
   * @nosubgrouping
   * @see uima::CAS
   */
  class UIMA_LINK_IMPORTSPEC AnalysisEngine {
    friend class JNIInstance;
    friend class uima::CASIterator;
    friend class uima::internal::DeprecatedTAEDecorator;
	friend class uima::internal::AnnotatorManager;

    
  protected:
	/**
       * Returns whether this engine will return a new CAS.
       * @return true or false.
       */
    virtual bool hasNext() = 0;

    /**
        * Returns a new CAS distinct from the input CAS.
        * @return true or false.
        */
    virtual CAS & next() = 0;


    /**
        * Returns the maximum number of CAS instances that this AnalysisComponent expects to 
        * use at the same time.  This only applies to CasMultipliers.  Most CasMultipliers will 
        * only need one CAS at a time.  Only if there is a clear need should this be overridden 
        * to return something greater than 1.
        * @return true or false.
        */
    virtual int getCasInstancesRequired() = 0;

  public:
   
	 virtual ~AnalysisEngine() {}
    /**
     * create a new CAS which can be used to process documents and other data with this AnalysisEngine.
     * Memory ownership is transferred to the caller. The returned CAS is only
     * valid for the lifetime as the engine it was obtained from.
     */
    virtual CAS * newCAS() const = 0;

	/**
     * Returns true if this is not an aggregate engine
     */
    virtual bool isPrimitive() const = 0;

    /** Returns the AnnotatorContext for this engine */
    virtual AnnotatorContext & getAnnotatorContext() = 0;

	/** Returns the AnnotatorContext for this engine  const version */
    virtual AnnotatorContext const & getAnnotatorContext() const = 0;
   
	/**
     * get an uima::AnalysisEngineMetaData describing all kinds of meta data
     * about this engine, e.g., if it is primitive, which annotators it uses, etc.
     */
    //replace virtual AnalysisEngineMetaData const & getAnalysisEngineMetaData() const = 0;
	virtual AnalysisEngineMetaData const & getAnalysisEngineMetaData() const;

    /**
     * Returns the result specification that is specified in the
     * configuration file for this engine.
     * An application may copy the result of this function, remove some
     * elements from the copy and pass it back to the process function.
     */
    virtual ResultSpecification const & getCompleteResultSpecification() const = 0;


	virtual TyErrorId initialize(AnalysisEngineDescription const & ) = 0;
    virtual bool isInitialized() const = 0;

    /**
     * invoke this engine's analysis logic.
     * @return an error code
     */
    virtual TyErrorId process(CAS & cas) = 0;

    /**
     * invoke this engine's analysis logic where <code>resultSpec</code>
     * constrains what kinds on results are needed by the application.
     * @return an error code
     */
    virtual TyErrorId process(CAS & cas, ResultSpecification const & resultSpec) = 0;

    /**
     * trigger a reconfigure call to all annotators of this engine
     * @return an error code
     */
    virtual TyErrorId reconfigure() = 0;

    /**
     * de-initialize the engine.
     * @return an error code
     */
    virtual TyErrorId destroy() = 0;

    /**
     * Completes the processing of a batch. 
     * A collection of artifacts to be analyzed may be divided into one or more batches 
     * - it is up to the CollectionProcessingManager or the application to determine 
     * the number and size of batches. 
     * @return an error code
     */
    virtual TyErrorId batchProcessComplete() = 0;

    /**
     * Completes the processing of a collection.
     * The CollectionProcessingManager or the application invokes this
     * method when after all artifacts in the collection are processed. 
     * @return an error code
     */
    virtual TyErrorId collectionProcessComplete() = 0;


    /**Processes a <code>CAS</code>, possibly producing multiple CASes as a result.
        * The application uses the {@link CASIterator } interface to step through the 
        * output CASes. 
        * <p>
        * If this Analysis Engine does not produce output CASes, then the 
        * <code>CASIterator</code> will return no elements. 
        * <p>
        * Once this method is called, the AnalysisEngine "owns" <code>aCAS</code>
        * until such time as the {@link CASIterator#hasNext() } method returns false.
        * That is, the caller should not attempt to modify or access the input CAS until
        * it has read all of the elements from the CasIterator.  If the caller wants to 
        * abort the processing before having read all of the output CASes, it may call 
        * {@link uima::CASIterator#release()}, which will stop further processing from 
        * occurring, and ownership of <code>aCAS</code> will revert to the caller.
     */
    virtual CASIterator processAndOutputNewCASes(CAS &) =0;

    
#ifdef UIMA_DEBUG_ANNOTATOR_TIMING

    /** @name Timer Access Functions
       Note: The time returned accumulates over all calls to the functions.
       So getProcessDocumentTime() will return the time spent processing
       documents since the instance (this Annotator object), was
       created.
    */
    /*@{*/
    /** Return the time spent in Annotator DLL load(). */
    virtual const Timer &              getLoadTimer(void) const = 0;
    /** Return the time spent in initialize(). */
    virtual const Timer &              getInitTimer(void) const = 0;
    /** Return the time spent in destroy(). */
    virtual const Timer &              getDeInitTimer(void) const = 0;
    /** Return the time spent in reconfigure(). */
    virtual const Timer &              getConfigTimer(void) const = 0;
    /** Return the time spent in process(). */
    virtual const Timer &              getProcessDocumentTimer(void) const = 0;
    virtual void displayTimingData( util::ConsoleUI const &, bool bVerbose = false ) const = 0;

    /*@}*/
#endif
        /** @name Static Info Functions */
    /*@{*/
    /** Return a static pointer to a string representation of the specified
        error id. Can be used to produce more readable error output*/
    static const char *     getErrorIdAsCString(TyErrorId utErrorId);
    /** Prints a table of ERRID = ERRSTRING to <TT>rclOutStream</TT>.*/
    static void             printErrorIdTable(std::ostream & rclOutStream);
    /** Return the engine version information. */
    static const char *     getVersionInfo(void);
    /** Return the engine level information. */
    static const char *     getLevelInfo(void);
  };


  /**
   * @deprecated
   * A TextAnalysisEngine is an AnalysisEngine specialized for Text Analysis.
   * It adds text specific support like language aware processing.
   * It contains a specialized version of the CAS for text analysis, a uima::TCAS.
   * <br>
   * This class also provides methods to instantiate an AnalysisEngine from
   * an XML descriptor. 
   *
   */
  class UIMA_LINK_IMPORTSPEC TextAnalysisEngine : public AnalysisEngine {
    friend class uima::internal::DeprecatedTAEDecorator;
    friend class uima::internal::AnnotatorManager;
  protected:

  public:
    virtual ~TextAnalysisEngine() {}
    /** @name Creation functions */
    /*@{*/
    /**@deprecated
     * create a TextAnalysisEngine from a TAESpecifier. Returns NULL if creation failed.
     * In such a case, <code>errorInfo</code> contains information about possible errors
     * (output parameter).
     */
    static TextAnalysisEngine * createTextAnalysisEngine(AnalysisEngineDescription &, ErrorInfo& errorInfo);

    /**@deprecated
     * create a TextAnalysisEngine from the name of configuration file. Returns NULL if creation failed.
     * In such a case, <code>errorInfo</code> contains information about possible errors
     * (output parameter).
     */
    static TextAnalysisEngine * createTextAnalysisEngine(char const * cpConfigFileName, ErrorInfo& errorInfo);

    /**@deprecated
     * create a TextAnalysisEngine from an in-memory XML Buffer. Returns NULL if creation failed.
     * In such a case, <code>errorInfo</code> contains information about possible errors
     * (output parameter).
     */
    static TextAnalysisEngine * createTextAnalysisEngine(UChar const * cpBuffer, size_t uiLength, ErrorInfo& errorInfo);


    /**@deprecated
     * A lower level API for creating a TextAnalysisEngine
     */
    static TextAnalysisEngine * createTAE(AnnotatorContext & rANC, bool bOwnsANC,
                                          bool bOwnsTAESpecifier,
                                          uima::internal::CASDefinition & casDefinition,
                                          bool ownsCASDefintion,
                                          ErrorInfo &);
    /**@deprecated
     * Creates a TAE from a file name if the first argument is true or 
     * an XML buffer if false.
     */
    static TextAnalysisEngine * createTAE(bool isFile,
                                          icu::UnicodeString const &,
                                          ErrorInfo & );


    /*@}*/
   
  };




  /**
   * 
   */
  class UIMA_LINK_IMPORTSPEC Framework {
    friend class uima::internal::DeprecatedTAEDecorator;
    friend class uima::internal::AnnotatorManager;
  protected:

  public:
    /** @name Creation functions */
    /*@{*/
    /**
     * create a TypeSystem from the input specifications. Returns NULL if creation failed.
     * In such a case, <code>errorInfo</code> contains information about possible errors
     * (output parameter).
     * The returned TypeSystem object is committed and cannot be modified.
     */

    static TypeSystem * createTypeSystem(TypeSystemDescription const &,
                                         icu::UnicodeString const &,
                                         ErrorInfo & );
    static TypeSystem * createTypeSystem(TypeSystemDescription const &,
                                         uima::AnalysisEngineMetaData::TyVecpTypePriorities const &,
                                         icu::UnicodeString const &,
                                         ErrorInfo & );

    static TypeSystem * createTypeSystem(char const * tsSpecFileName, ErrorInfo& );

    static TypeSystem * createTypeSystem(AnalysisEngineMetaData const & ae, ErrorInfo& rErrorInfo);
    /**
    * create a TypeSystem object from a null terminated char buffer containing
    * the XML typesystem descriptor.
    */
    static TypeSystem * createTypeSystem(UChar const * cpBuffer, size_t, ErrorInfo& );
    /**
    * create a TypeSystem object from a null terminated char buffer containing
    * the XML typesystem descriptor.
    */
    static TypeSystem * createTypeSystemFromXMLBuffer(char const * cpBuffer,  ErrorInfo& rErrorInfo);

    /**
     * create a Cas using input TypeSystem and default indices. Returns NULL if creation failed.
     * In such a case, <code>errorInfo</code> contains information about possible errors
     * (output parameter).
     */
    static CAS * createCAS(TypeSystem &, ErrorInfo&);
    /**
     * create a CAS with input TypeSystem and indices defined in the descriptor. 
     * Returns NULL if creation failed.
     * In such a case, <code>errorInfo</code> contains information about possible errors
     * (output parameter). 
     *
     */
    static CAS * createCAS(TypeSystem &, AnalysisEngineMetaData & , ErrorInfo &);

    /**
     * create a CAS from TypeSystem and index definition and type priorities
     * Returns NULL if creation failed.
     * In such a case, <code>errorInfo</code> contains information about possible errors
     * (output parameter). 
     *
     */
    static CAS * createCAS(TypeSystem &,
                           AnalysisEngineMetaData::TyVecpFSIndexDescriptions &,
                           AnalysisEngineMetaData::TyVecpTypePriorities &,
                           ErrorInfo &);


    /**
     * create a CAS from CASDefinition. Return NULL if creation failed.
     * In such a case, <code>errorInfo</code> contains information about possible errors
     * (output parameter). 
     *
     */
    static CAS * createCAS(uima::internal::CASDefinition  &,
                           ErrorInfo &) ;



    /**
     * Create Description objects from xml
     */
    static TypeSystemDescription * createTypeSystemDescription(char const * tsSpecFileName);
    static TypeSystemDescription * createTypeSystemDescription(UChar const * cpBuffer, size_t uiLength);


    /*@}*/
    

    /**
     * create a AnalysisEngine from an AnalysisEngineDescription. Returns NULL if creation failed.
     * In such a case, <code>errorInfo</code> contains information about possible errors
     * (output parameter).
     * The createAnalysisEngine functions could cause a memory leak if it 
     * fails because these methods use auto ptrs which are released prior
     * to engine initialization.
     */
    static AnalysisEngine * createAnalysisEngine(AnalysisEngineDescription &, ErrorInfo& errorInfo);

    /**
     * create a AnalysisEngine from the name of configuration file. Returns NULL if creation failed.
     * In such a case, <code>errorInfo</code> contains information about possible errors
     * (output parameter).
     * The createAnalysisEngine functions could cause a memory leak if it 
     * fails because these methods use auto ptrs which are released prior
     * to engine initialization.
     */
    static AnalysisEngine * createAnalysisEngine(char const * cpConfigFileName, ErrorInfo& errorInfo);

    /**
     * create a TextAnalysisEngine from an in-memory XML Buffer. Returns NULL if creation failed.
     * In such a case, <code>errorInfo</code> contains information about possible errors
     * (output parameter).
     * The createAnalysisEngine functions could cause a memory leak if it 
     * fails because these methods use auto ptrs which are released prior
     * to engine initialization.
     */
    static AnalysisEngine * createAnalysisEngine(UChar const * cpBuffer, size_t uiLength, ErrorInfo& errorInfo);


	/**
     * Creates a TAE from a file name if the first argument is true or 
     * an XML buffer if false.
     * The createAnalysisEngine functions could cause a memory leak if it 
     * fails because these methods use auto ptrs which are released prior
     * to engine initialization.
     */
    static AnalysisEngine * createAnalysisEngine(bool isFile,
                                          icu::UnicodeString const &,
                                          ErrorInfo & );

    /**
     * A lower level API for creating a TextAnalysisEngine
     * The createAnalysisEngine functions could cause a memory leak if it 
     * fails because these methods use auto ptrs which are released prior
     * to engine initialization.
     */
    static AnalysisEngine * createAnalysisEngine(AnnotatorContext & rANC, bool bOwnsANC,
                                          bool bOwnsTAESpecifier,
                                          uima::internal::CASDefinition & casDefinition,
                                          bool ownsCASDefintion,
                                          ErrorInfo &);
   

  };


}

/* ----------------------------------------------------------------------- */
#endif /* UIMA_ENGINE_HPP */

/* <EOF> */



