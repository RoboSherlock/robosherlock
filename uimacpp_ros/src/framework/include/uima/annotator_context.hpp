/** \file annotator_context.hpp .
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

-----------------------------------------------------------------------------

    \brief Contains class uima::AnnotatorContext
	 Description: The AnnotatorContext object provides access to the 
	              configuration parameters for the Annotator.
                  It also provides access to the framework log facility.
				   Use uima::Annotator::getAnnotatorContext() to instantiate a CAS

-------------------------------------------------------------------------- */

#ifndef UIMA_ANNOTATOR_CONTEXT_HPP
#define UIMA_ANNOTATOR_CONTEXT_HPP

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> //must be included first to disable warnings

#include <uima/types.h>
#include <uima/err_ids.h>
#include <uima/trace.hpp>        /* For "first" component id */
#include <uima/comp_ids.h>
#include <uima/engine.hpp>
#include <uima/internal_engine_base.hpp>
#include <uima/taespecifier.hpp>
#include <uima/featurestructure.hpp>
#include <uima/sofaid.hpp>
#include <uima/cas.hpp>
#include <uima/caspool.hpp>
#include <set>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
  class CAS;
  class TCAS;
  class LogFacility;
  class Language;
  class NameValuePair;
  class CASPool;

  namespace internal {
    class CASDefinition;
    class EngineBase;
  }

}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {


  /**
   * The AnnotatorContext is the interface to the configuration settings
   * and other metadata of an AnalysisEngine. Moreover, it gives access to the logging 
  * facility to be used by Annotators.
   * <code>AnnotatorContext</code> defines various <code>extractValue</code> and
   * <code>assignValue</code> methods. These methods are used by the application and
   * annotators to access (or change) configuration parameter values.
  * <br>
   */
  class UIMA_LINK_IMPORTSPEC AnnotatorContext {
  public:
    typedef std::map < icu::UnicodeString, AnnotatorContext * > TyMapDelegateAnCs;
    typedef std::map < icu::UnicodeString, SofaID * > TyMapSofaMappings;



    //------------------------
    //
    //   Constructor
    //
    //------------------------
    AnnotatorContext(void) {
      iv_pTaeSpecifier=NULL;
      bOwnsTaeSpecifier=false;
      iv_pLogger=NULL;
      iv_pParentAnC=NULL;
    }

    /**
    * Note: This object will NOT assume memory ownership of <code>taeSpec</code>.
    * It is the responsibility of the caller to delete <code>taeSpec</code>.
    **/
    AnnotatorContext(AnalysisEngineDescription * taeSpec);

    ~AnnotatorContext(void);
    /** @{ */
    /**
     * Return string config params as UTF-8--should be only if you know this is will work with
     * you are dealing with single byte char set. 
     */
    TyErrorId extractValue(const icu::UnicodeString & paramName, std::string & value) const;
    /** returns string parameter values as UTF-8 string
     * Caller assumes ownership of objects in the vector
     */
    TyErrorId extractValue(const icu::UnicodeString & paramName, std::vector<std::string*> & returnValues) const;

    TyErrorId extractValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, std::string & value) const;
    /** returns string parameter values as UTF-8 string
     * Caller assumes ownership of objects in the vector
     */
    TyErrorId extractValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, std::vector<std::string*> & returnValues) const;
    /** @} */


    //end for externalization of configuration object
    //-----------------------------------------------------





    /**
     * Use this method to obtain a SofaID which is a handle to a Sofa in the CAS.  
     * This method looks up the SofaID in the Sofa name mapping provided for
     * this AnalysisEngine.  If there is no mapping for the specified Sofa name,
     * it maps to itself.
     * Note: This AnnotatorContext still has memory ownership of the returned
     * object.
     **/
    const SofaID & mapToSofaID(const icu::UnicodeString & componentSofaName)  {
      TyMapSofaMappings::const_iterator ite = iv_mapSofaMappings.find(componentSofaName);
      if (ite != iv_mapSofaMappings.end()) {
        return *(ite->second);
      } else {
        SofaID * pSofaid = new SofaID();
        pSofaid->setComponentSofaName(componentSofaName);
        pSofaid->setSofaId(componentSofaName);
        icu::UnicodeString astr = pSofaid->getComponentSofaName();
        iv_mapSofaMappings[astr] = pSofaid;
        return *pSofaid;
      }
    }


    /**
     * Returns the component sofaname to absolute sofa name mapping for this context
     *     
     * Note: This AnnotatorContext still has memory ownership of the returned
     * object.
     **/
    TyMapSofaMappings  getSofaMappings() const {
      return iv_mapSofaMappings;
    }

    /**
      * Defines the CAS pool that this AnnotatorContext must support. 
         * This method must be called before {@link AnnotatorContext#getEmptyCAS()}
      * may be called. A CASPool is defined by the framework when the components 
      * getCASInstancesRequired() returns a value greater than 0,
      */
    TyErrorId defineCASPool(size_t numInstances);

    /**
    * Returns an empty CAS from the CASPool. This AnnotatorContext retains ownership of 
    * the CAS instance. To return the CAS to the pool, call releaseCAS.
    */
    CAS & getEmptyCAS();


    /** Return the CAS to the pool. This will first call CAS.reset().
    */
    void releaseCAS(CAS &);

    /** @name Getters  for Configuration Parameter Values
    * These getters should be used as follows:
    * <ol>
    * <li> If there are no groups defined, use the methods with two arguments </li>
    * <li> If there is a default group defined, the methods with two arguments will lookup the value in the defailt group </li>
    * <li> The methods with three arguments will return a value found in the specified group or in one of the fallback groups, as defined in the TextAnalysisEngine specifier.</li>
    * </ol>
    * If parameter name (and group name) are valid, but no value can be found, the methods return
    * UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND. In this case, the value parameter passed into the method is not changed.
    * All methods check whether type and multiplicity of the value match the definition for <code>paramName</code>.
    * If not, they return an error code. In this case, the value parameter passed into the method is not changed.
    * If the <code>AnnotatorContext<code> belongs to a delegate TextAnalysisEngine, these methods will search in the
    * parent TAE first, as the parent may override values of the delegates.
    **/
    /*@{*/
    TyErrorId extractValue(const icu::UnicodeString & paramName, bool & value) const;
    TyErrorId extractValue(const icu::UnicodeString & paramName, std::vector<bool> & returnValues) const;
    TyErrorId extractValue(const icu::UnicodeString & paramName, int & value) const;
    TyErrorId extractValue(const icu::UnicodeString & paramName, size_t & value) const;
    TyErrorId extractValue(const icu::UnicodeString & paramName, std::vector<int> & returnValues) const;
    TyErrorId extractValue(const icu::UnicodeString & paramName, float & value) const;
    TyErrorId extractValue(const icu::UnicodeString & paramName, std::vector<float> & returnValues) const;
    TyErrorId extractValue(const icu::UnicodeString & paramName, icu::UnicodeString & value) const;
    TyErrorId extractValue(const icu::UnicodeString & paramName, std::vector<icu::UnicodeString> & returnValues) const;

    TyErrorId extractValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, bool & value) const;
    TyErrorId extractValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, std::vector<bool> & returnValues) const;
    TyErrorId extractValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, int & value) const;
    TyErrorId extractValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, size_t & value) const;
    TyErrorId extractValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, std::vector<int> & returnValues) const;
    TyErrorId extractValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, float & value) const;
    TyErrorId extractValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, std::vector<float> & returnValues) const;
    TyErrorId extractValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, icu::UnicodeString & value) const;
    TyErrorId extractValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, std::vector<icu::UnicodeString> & returnValues) const;
    /** @} */

    /** Release contents of string container filled by extractValue.
     * Useful when caller and callee use different heaps, 
     * e.g. when debug code uses a release library.
     */
    void release(std::string & value) const;
    /** Release contents of vector container filled by extractValue.
     * Useful when caller and callee use different heaps, 
     * e.g. when debug code uses a release library.
     */
    void release(std::vector<std::string*> & returnValues) const;

    /**
     * Returns true iff the parameter paramName is defined for this
     * TAE specifier
     */
    bool isParameterDefined(const icu::UnicodeString & paramName) const;

    /**
     * Returns true iff the parameter paramName is defined in the
     * group groupName for this TAE specifier
     */
    bool isParameterDefined(const icu::UnicodeString & groupName,
                            const icu::UnicodeString & paramName) const;



    /**
     * Returns this AnC's AnalysisEngineDescription
     **/
    const AnalysisEngineDescription & getTaeSpecifier() const {
      assert( EXISTS(iv_pTaeSpecifier) );
      return *iv_pTaeSpecifier;
    }

    /**
    * Returns NULL if no delegate for key can be found.
    * Note: This AnnotatorContext still has memory ownership of the returned
    * object.
    **/
    AnnotatorContext * getDelegate(const icu::UnicodeString & key) const;

    /**
    * Returns NULL if no delegate for key can be found.
    * NOTE: This AnnotatorContext will transfer memory ownership of the returned object
    * to the caller. The returned object is removed from the list of delegate
    * AnnotatorContexts
    **/
    AnnotatorContext * extractDelegate(const icu::UnicodeString & key);

    /**
    * Returns the delegates of this AnnotatorContext. Each delegate is mapped to
    * a unique key.
    * Note: This AnnotatorContext still has memory ownership of the returned
    * objects.
    **/
    const TyMapDelegateAnCs & getDelegates() const {
      return iv_mapDelegateAncs;
    }

    /**
    * returns the LogFacility to be used for logging
    **/
    LogFacility & getLogger() const {
      assert( EXISTS(iv_pLogger) );
      return *iv_pLogger;
    }

    size_t getTraceCompId() const {
      return UIMA_TRACE_COMPID_ANNOTATOR_DEFAULT;
    }


    /**
     * returns the index descriptions of this AnnotatorContext and all of its delegates
     **/
    const AnalysisEngineMetaData::TyVecpFSIndexDescriptions getFSIndexDescriptions() const;

    /**
    * @return A set containing the names of all groups that define <code>paramName</code>.
    * This includes the groups in this AnC and all its parents.
    * Calls to <code>extractValue</code> for each of these group names will succeed (albeit
    * it is not guarenteed that each call will return a value).
    * If the AnC doesn't contain groups, but defines the parameter, a special group name
    * will be returned that can be used in the calls to <code>extractValue</code>.
    * In this case, no group names of any parents will be included in the result,
    * as <code>extractValue</code> on this AnC with these group names would fail.
    **/
    const std::set <icu::UnicodeString> getGroupNamesForParameter(const icu::UnicodeString & paramName) const;



    /** @name Setters for Configuration Parameter Values
    * These setters should be used as follows:
    * <ol>
    * <li> If there are no groups defined, use the methods with two arguments. </li>
    * <li> If there is a default group defined, the methods without the group name will try to set the value on the default group </li>
    * <li> To set the values for a certain group, use the methods with group names. Note that there is no fallback strategy: if the group does not exist, the method will return an error</li>
    * </ol>
    * All methods check whether type and multiplicity of the value match the definition for <code>paramName</code>.
    * If not, they return an error code.
    * If the <code>TyErrorId</code> returned is not UIMA_ERR_NONE, the value of the configuration parameter will not have been changed.
    **/
    /*@{*/

    TyErrorId assignValue(const icu::UnicodeString & paramName, const bool & value);
    TyErrorId assignValue(const icu::UnicodeString & paramName, const std::vector< bool > & value);

    TyErrorId assignValue(const icu::UnicodeString & paramName, const size_t & value);
    TyErrorId assignValue(const icu::UnicodeString & paramName, const std::vector< size_t > & value);

    TyErrorId assignValue(const icu::UnicodeString & paramName, const int & value);
    TyErrorId assignValue(const icu::UnicodeString & paramName, const std::vector< int > & value);

    TyErrorId assignValue(const icu::UnicodeString & paramName, const float & value);
    TyErrorId assignValue(const icu::UnicodeString & paramName, const std::vector< float > & value);

    TyErrorId assignValue(const icu::UnicodeString & paramName, const double & value);
    TyErrorId assignValue(const icu::UnicodeString & paramName, const std::vector< double > & value);

    TyErrorId assignValue(const icu::UnicodeString & paramName, const icu::UnicodeString & value);
    TyErrorId assignValue(const icu::UnicodeString & paramName, const std::vector< icu::UnicodeString > & value);

    TyErrorId assignValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, const bool & value);
    TyErrorId assignValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, const std::vector< bool > & value);

    TyErrorId assignValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, const int & value);
    TyErrorId assignValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, const std::vector< int > & value);

    TyErrorId assignValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, const size_t & value);
    TyErrorId assignValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, const std::vector< size_t > & value);

    TyErrorId assignValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, const float & value);
    TyErrorId assignValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, const std::vector< float > & value);

    TyErrorId assignValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, const double & value);
    TyErrorId assignValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, const std::vector< double > & value);


    TyErrorId assignValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, const icu::UnicodeString & value);
    TyErrorId assignValue(const icu::UnicodeString & groupName, const icu::UnicodeString & paramName, const std::vector< icu::UnicodeString > & value);

    /** @} */



  protected:
    /* --- functions --- */
    //AnnotatorContext(void);
  private:
    friend class uima::internal::EngineBase;

    AnalysisEngineDescription * getTAESpec() {
      return iv_pTaeSpecifier;
    }

    AnnotatorContext(AnalysisEngineDescription * delegateSpec,
                     AnnotatorContext * parent,
                     const icu::UnicodeString & ancKey);


    AnnotatorContext(AnalysisEngineDescription * delegateSpec,
                     AnnotatorContext * parent,
                     const icu::UnicodeString & ancKey,
                     const AnalysisEngineDescription::TyVecpSofaMappings & sofaMappings);


    void buildDelegateAnCs();

    /* COPY CONSTRUCTOR NOT SUPPORTED */
    AnnotatorContext(const AnnotatorContext & );
    /* ASSIGNMENT OPERATOR NOT SUPPORTED */
    AnnotatorContext & operator=(const AnnotatorContext & crclObject);

    NameValuePair const * findNameValuePair(
      const icu::UnicodeString & groupName,
      const icu::UnicodeString & paramName,
      AnalysisEngineMetaData::EnSearchStrategy strategy,
      const icu::UnicodeString & ancKey="") const;

    NameValuePair const * findNameValuePair(const icu::UnicodeString & paramName,
                                            const icu::UnicodeString & ancKey="") const;

    /**
    * Sets nvPair in the parent (or one of its ancestors), if possible.
    * If not, it tries to set it in its own AnalysisEngineDescription.
    * @see AnalysisEngineDescription::setNameValuePair
    **/
    TyErrorId setNameValuePair(const NameValuePair & nvPair);

    /**
    * Sets nvPair in group groupName of the parent (or one of its ancestors), if possible.
    * If not, it tries to set it in its own AnalysisEngineDescription.
    * @see AnalysisEngineDescription::setNameValuePair
    **/
    TyErrorId setNameValuePair(const icu::UnicodeString & groupName,
                               const NameValuePair & nvPair);

    LogFacility * iv_pLogger;
    AnnotatorContext * iv_pParentAnC;
    //the key to this AnC in the parent AnC
    icu::UnicodeString iv_AnCKey;
    AnalysisEngineDescription * iv_pTaeSpecifier;
    TyMapDelegateAnCs iv_mapDelegateAncs;
    TyMapSofaMappings iv_mapSofaMappings;
    CASPool *   iv_pCasPool;
    //BSI
    bool bOwnsTaeSpecifier;
  };
  /* AnnotatorContext */



} //namespace uima

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

namespace uima {

  /**inline
   AnnotatorContext::AnnotatorContext(void)
      {}
      **/
  /* The following extractValue methods must be inline
     (at least for the overloads with vector of values under Win/MS compiler)
     because the central assignment of the value
     (e.g. returnValues = pResult->extractBoolValues();)
     must be interpreted in the context of the exploiting annotator or app.
     Otherwise the new values for the vector returnValues would be added
     using the memory management of the UIMACPP engine DLL (typically the
     memory mgmt of MS ship/release C++ runtime). If the exploiting
     annotator or app is built with in debug mode the values will be
     freed (e.g. after returnValues has expired) under the memory mgmt of
     the MS debug C++ runtime). This results in an exception
     _BLOCK_TYPE_IS_VALID(pHead->nBlockUse)

     To put it in a nutshell:
     If it's not inline you can't mix a ship engine library with a
     debug exploiter.
   */
  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & paramName,
      bool & returnValue) const {
    NameValuePair const * pResult = findNameValuePair(paramName);
    if (pResult != NULL) {// a value has been found
      returnValue = pResult->extractBoolValue();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & paramName,
					   std::vector<bool> & returnValues) const {
    NameValuePair const * pResult = findNameValuePair(paramName);
    if (pResult != NULL) {// a value has been found
      returnValues = pResult->extractBoolValues();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & paramName,
      int & returnValue) const {
    NameValuePair const * pResult = findNameValuePair(paramName);
    if (pResult != NULL) {// a value has been found
      returnValue = pResult->extractIntValue();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & paramName,
      size_t & returnValue) const {
    NameValuePair const * pResult = findNameValuePair(paramName);
    if (pResult != NULL) {// a value has been found
      returnValue = (size_t)pResult->extractIntValue();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & paramName,
					   std::vector<int> & returnValues) const {
    NameValuePair const * pResult = findNameValuePair(paramName);
    if (pResult != NULL) {// a value has been found
      returnValues = pResult->extractIntValues();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & paramName,
      float & returnValue) const {
    NameValuePair const * pResult = findNameValuePair(paramName);
    if (pResult != NULL) {// a value has been found
      returnValue = pResult->extractFloatValue();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & paramName,
					   std::vector<float> & returnValues) const {
    NameValuePair const * pResult = findNameValuePair(paramName);
    if (pResult != NULL) {// a value has been found
      returnValues = pResult->extractFloatValues();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & paramName,
      icu::UnicodeString & returnValue) const {
    NameValuePair const * pResult = findNameValuePair(paramName);
    if (pResult != NULL) {// a value has been found
      returnValue = pResult->extractStringValue();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & paramName,
					   std::vector<icu::UnicodeString> & returnValues) const {
    NameValuePair const * pResult = findNameValuePair(paramName);
    if (pResult != NULL) {// a value has been found
      returnValues = pResult->extractStringValues();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & groupName,
      const icu::UnicodeString & paramName,
      bool & returnValue) const {
    NameValuePair const * pResult =
      findNameValuePair(groupName, paramName, iv_pTaeSpecifier->getSearchStrategy());
    if (pResult != NULL) {// a value has been found
      returnValue = pResult->extractBoolValue();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & groupName,
      const icu::UnicodeString & paramName,
					   std::vector<bool> & returnValues) const {
    NameValuePair const * pResult =
      findNameValuePair(groupName, paramName, iv_pTaeSpecifier->getSearchStrategy());
    if (pResult != NULL) {// a value has been found
      returnValues = pResult->extractBoolValues();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & groupName,
      const icu::UnicodeString & paramName,
      int & returnValue) const {
    NameValuePair const * pResult =
      findNameValuePair(groupName, paramName, iv_pTaeSpecifier->getSearchStrategy());
    if (pResult != NULL) {// a value has been found
      returnValue = pResult->extractIntValue();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & groupName,
      const icu::UnicodeString & paramName,
      size_t & returnValue) const {
    NameValuePair const * pResult =
      findNameValuePair(groupName, paramName, iv_pTaeSpecifier->getSearchStrategy());
    if (pResult != NULL) {// a value has been found
      returnValue = (size_t)pResult->extractIntValue();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & groupName,
      const icu::UnicodeString & paramName,
					   std::vector<int> & returnValues) const {
    NameValuePair const * pResult =
      findNameValuePair(groupName, paramName, iv_pTaeSpecifier->getSearchStrategy());
    if (pResult != NULL) {// a value has been found
      returnValues = pResult->extractIntValues();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & groupName,
      const icu::UnicodeString & paramName,
      float & returnValue) const {
    NameValuePair const * pResult =
      findNameValuePair(groupName, paramName, iv_pTaeSpecifier->getSearchStrategy());
    if (pResult != NULL) {// a value has been found
      returnValue = pResult->extractFloatValue();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & groupName,
      const icu::UnicodeString & paramName,
					   std::vector<float> & returnValues) const {
    NameValuePair const * pResult =
      findNameValuePair(groupName, paramName, iv_pTaeSpecifier->getSearchStrategy());
    if (pResult != NULL) {// a value has been found
      returnValues = pResult->extractFloatValues();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & groupName,
      const icu::UnicodeString & paramName,
      icu::UnicodeString & returnValue) const {
    NameValuePair const * pResult =
      findNameValuePair(groupName, paramName, iv_pTaeSpecifier->getSearchStrategy());
    if (pResult != NULL) {// a value has been found
      returnValue = pResult->extractStringValue();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & groupName,
      const icu::UnicodeString & paramName,
					   std::vector<icu::UnicodeString> & returnValues) const {
    NameValuePair const * pResult =
      findNameValuePair(groupName, paramName, iv_pTaeSpecifier->getSearchStrategy());
    if (pResult != NULL) {// a value has been found
      returnValues = pResult->extractStringValues();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }


  //UTF-8 strings
  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & groupName,
      const icu::UnicodeString & paramName,
      std::string & returnValue) const {
    NameValuePair const * pResult =
      findNameValuePair(groupName, paramName, iv_pTaeSpecifier->getSearchStrategy());
    if (pResult != NULL) {// a value has been found
      returnValue = pResult->extractSingleByteStringValue();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & groupName,
      const icu::UnicodeString & paramName,
					   std::vector<std::string*> & returnValues) const {
    NameValuePair const * pResult =
      findNameValuePair(groupName, paramName, iv_pTaeSpecifier->getSearchStrategy());
    if (pResult != NULL) {// a value has been found
      pResult->extractSingleByteStringValues(returnValues);
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & paramName,
      std::string & returnValue) const {
    NameValuePair const * pResult = findNameValuePair(paramName);
    if (pResult != NULL) {// a value has been found
      returnValue = pResult->extractSingleByteStringValue();
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

  inline
  TyErrorId AnnotatorContext::extractValue(const icu::UnicodeString & paramName,
					   std::vector<std::string*> & returnValues) const {
    NameValuePair const * pResult = findNameValuePair(paramName);
    if (pResult != NULL) {// a value has been found
      pResult->extractSingleByteStringValues(returnValues);
      return UIMA_ERR_NONE;
    }
    return UIMA_ERR_CONFIG_NAME_VALUE_PAIR_NOT_FOUND;
  }

} // namespace uima
/* ----------------------------------------------------------------------- */
#endif /* UIMA_ANNOTATOR_CONTEXT_HPP */

/* <EOF> */

