#ifndef UIMA_CAS_HPP
#define UIMA_CAS_HPP
/** \file cas.hpp .
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

    \brief  Contains class uima::CAS

   Description: The CAS object provides access to the type system, to indexes,
       iterators and filters (constraints).
       It also lets you create new annotations, Sofas and other data
       structures.

             Use uima::AnalysisEngine::newCAS() to instantiate a CAS

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp>
#include <uima/lowlevel_typedefs.hpp>
#include <uima/featurestructure.hpp>
#include <uima/language.hpp>
#include <uima/sofaid.hpp>
#include <uima/sofastream.hpp>
#include <uima/fsindex.hpp>
#include <map>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */
/* option names for engine */
#define UIMA_ENGINE_CONFIG_OPTION_PREDEF_TYPES               _TEXT("PredefinedTypes")

#define UIMA_ENGINE_CONFIG_OPTION_FSHEAP_PAGESIZE            _TEXT("FSHeapPageSize")
#define UIMA_ENGINE_CONFIG_OPTION_STRINGHEAP_PAGESIZE        _TEXT("StringHeapPageSize")
#define UIMA_ENGINE_CONFIG_OPTION_STRINGREFHEAP_PAGESIZE     _TEXT("StringRefHeapPageSize")

const size_t                  UIMA_ENGINE_CONFIG_DEFAULT_FSHEAP_PAGESIZE = 10000; // this value seems good for search engine-style tokenization, old value = 128 * 1024
const size_t                  UIMA_ENGINE_CONFIG_DEFAULT_STRINGHEAP_PAGESIZE = UIMA_ENGINE_CONFIG_DEFAULT_FSHEAP_PAGESIZE;
const size_t                  UIMA_ENGINE_CONFIG_DEFAULT_STRINGREFHEAP_PAGESIZE = UIMA_ENGINE_CONFIG_DEFAULT_STRINGHEAP_PAGESIZE;


/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  class FeatureStructure;
  class FSIndexRepository;
  class FSFilterBuilder;
  class ListFS;
  class SofaFS;
  class AnnotationFS;
  class DocumentFS;
  class FSIterator;
  class ANIterator;
  class SofaDataStream;
  class LocalSofaDataStream;
  class XCASDeserializerHandler;
  class XmiDeserializerHandler;
  class XCASWriter;
  class XmiWriter;
  class ComponentInfo;
  class AnnotatorContext;
  class SofaDataStream;
  class Framework;
  class CASPool;
  class ANIndex;
  namespace lowlevel {
    class IndexRepository;
    class IndexRepositoryDefinition;
    class FSHeap;
    class TypeSystem;
    class DefaultFSIterator;
  }
  namespace internal {
    class CASDefinition;
    class EngineBase;
    class CASSerializer;
    class CASDeserializer;
    class CASImpl;
    void fromHeapCellTempl(lowlevel::TyHeapCell, uima::CAS &, FeatureStructure &);
  }
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {

  UIMA_EXC_CLASSDECLARE(CouldNotCreateFSOfFinalTypeException, CASException);
  UIMA_EXC_CLASSDECLARE(DuplicateSofaNameException, CASException);
  UIMA_EXC_CLASSDECLARE(InvalidBaseCasMethod, CASException);

  /**
   * The CAS (Common Analysis System)
   * is the container where all feature structures are stored and
   * maintained.
   * The CAS object provides access to the type system, to indexes, 
   *   iterators and filters (constraints). 
   *   It also lets you create new annotations, Sofas and other data 
   *  structures.
   * <br>
   * Use uima::AnalysisEngine::newCAS() to instantiate a CAS.
   * <br>
   * <br>
   * The CAS APIs ( uima::CAS ) provide access to the following:
   * <ol>
   * <li>The creation of <strong>feature structures</strong> ( uima::FeatureStructure ).<br>
   *   These have one or more feature values which are
   *   like public data members of a feature structure.<br>
   *   The feature structures are organized in
   *   <strong>indexes</strong> ( uima::FSIndex ).
   *   <strong>Iterators</strong> ( uima::FSIterator ) over indexes are used to
   *   access individual feature structures.
   * <li>The <strong>Type System</strong> ( uima::TypeSystem )
   *   where all possible feature structures are defined in terms of
   *   <strong>@link uima::Type types@endlink</strong>
   *   and <strong>@link uima::Feature features@endlink</strong>.
   * <li>The creation of and access to Sofas ( uima::SofaFS )  and CAS views ( uima::CAS ) .   
   * <li>A set of @link PreDefTypes predefined types@endlink,
   *   @link PreDefFeatures features@endlink,
   *   @link PreDefIndexes indexes@endlink and
   *   @link UtilFuncts Utility Functions@endlink
   *   for text analysis.
   * </ol>
   */
  class UIMA_LINK_IMPORTSPEC CAS {
    friend class uima::internal::FSPromoter;
    friend class uima::lowlevel::FSHeap;
    friend class ListFS;
    friend class FeatureStructure;
    friend class SofaFS;
    friend class uima::internal::EngineBase;
    friend class uima::internal::CASSerializer;
    friend class uima::internal::CASDeserializer;
    friend class uima::XCASWriter;
	friend class uima::XmiWriter;
    friend class uima::XCASDeserializerHandler;
    friend class uima::XmiDeserializerHandler;
    friend class uima::Framework;
    friend class uima::CASPool;
    friend class AnnotationFS;
    friend class DocumentFS;
    friend void uima::internal::fromHeapCellTempl(lowlevel::TyHeapCell, uima::CAS &, FeatureStructure &);

  private:
    CAS(void);
    CAS(CAS const &);
    CAS & operator=(CAS const &);

    // Remove a View from sofaMap
    void dropView(int sofaNum);

    // private calls for internal use
    CAS* getInitialView();
    SofaFS createLocalSofa(const char* sofaName, const char* mimeType);
    SofaFS createSofa(UnicodeStringRef const sofaName, UnicodeStringRef const mimeType);
    SofaFS getSofa(int sofaNum);
    SofaFS getSofa(char* sofaName);
    SofaFS getSofa(UnicodeStringRef sofaName);
    uima::lowlevel::IndexRepository * getIndexRepositoryForSofa(SofaFS sofa);
    void bumpSofaCount();	
    void invalidBaseCasMethod();

    void registerView(SofaFS);
    void updateDocumentAnnotation( );
    void copyDocumentString(UnicodeStringRef);
    void refreshCachedTypes();
    void createDocumentAnnotation( void );
    void pickupDocumentAnnotation( );
    void setDocTextFromDeserializtion(UnicodeStringRef);

    inline int getNumViews() {
      return iv_baseCas->iv_sofaCount;
    }

    CAS & getCasForTyFS(lowlevel::TyHeapCell);

//   void updateAndIndexDocumentAnnotation();

  protected:
    int iv_sofaNum;
    uima::internal::CASDefinition * iv_casDefinition;
    bool bOwnsCASDefinition;
    TypeSystem * iv_typeSystem;
    uima::lowlevel::FSHeap * iv_heap;
    uima::lowlevel::IndexRepository * iv_indexRepository;

    uima::FSFilterBuilder * iv_filterBuilder;

    // absolute Sofa counter
    int iv_sofaCount;
    // maps sofaNum to Cas views
    std::map<int, CAS*> iv_sofa2tcasMap;
    // maps sofaNum to index repository
    std::vector<uima::lowlevel::IndexRepository *> iv_sofa2indexMap;
    // reference to one-and-only base CAS
    CAS* iv_baseCas;
    // reference to one-and-only initial View
    CAS* iv_initialView;
    bool isbaseCas;
    bool initialSofaCreated;
    bool isDeletingViews;  //set this flag to true when destroying CAS
    AnnotatorContext *iv_componentInfo;

    uima::lowlevel::TyFSType     iv_utDocumentType;
    uima::lowlevel::TyFSFeature  iv_utDocumentLangAsIntFeat;
    uima::lowlevel::TyFSFeature  iv_utDocumentLangAsStrFeat;
    uima::lowlevel::TyFS         iv_tyDocumentAnnotation;
    UChar const *               iv_cpDocument;
    size_t                      iv_uiDocumentLength;
    UChar *                     iv_copyOfDocument;

    /**
     * Add a copy of <code>crString</code> to the string heap.
     * @param crString the string to add
     * @return an LString pointing to the new copy
     */
    int addString(icu::UnicodeString const & crString);

    /**
     * Add a copy of <code>cpString</code> to the string heap.
     * @param cpString a pointer to the string to copy (maynot be zero terminated)
     * @param uiLen number of Unicode code units (not bytes!) to copy
     * @return an LString pointing to the new copy
     */
    int addString(UChar const * cpString, size_t uiLen);
    /**
     * Add a copy of <code>uls</code> to the string heap.
     * @param uls a pointer to the string to copy (maynot be zero terminated)
     * @return an LString pointing to the new copy
     */
    int addString(const UnicodeStringRef & uls );

    /**
     * Get a copy of <code>crString</code> from the string heap.
     * @param strRef the offset in the strRefHeap
     * @return a UnicodeStringRef to the data in the stringHeap
     */
    UnicodeStringRef getString(int strRef);

    /**
     * Construct a CAS.
     * @param uiFSHeapPageSize        the number of heap cells a heap page should contain.
     * @param uiStringHeapPageSize    the number of bytes a string heap page should contain.
     * @param uiStringRefHeapPageSize the number of bytes a string-ref heap page should contain.
     */
    CAS(uima::internal::CASDefinition &,
        size_t uiFSHeapPageSize,
        size_t uiStringHeapPageSize,
        size_t uiStringRefHeapPageSize);

    /**
     * Construct a CAS and specify ownership of CASDefinition object.
     */
    CAS(uima::internal::CASDefinition &,
        bool bOwnsCASDefinition,
        size_t uiFSHeapPageSize,
        size_t uiStringHeapPageSize,
        size_t uiStringRefHeapPageSize);

    CAS(CAS* baseCas, SofaFS aSofa);

    /**
     * Only for internal use. Return a View for given Sofa on heap
     */
    CAS* getViewBySofaNum(int sofaNum);

    /**
     * Only for internal use.
     */
    void registerInitialSofa();

    /**
     * Only for internal use.
     */
    bool isInitialSofaCreated();

    /**
     * Only for internal use.
     */
    SofaFS createInitialSofa(UnicodeStringRef const mimeType);

  public:
    virtual ~CAS();


    /**
     * Returns the Sofa number
     */
    inline int getSofaNum() {
      return iv_sofaNum;
    }

    /**
     * True if a CAS view
     */
    bool isBackwardCompatibleCas();

    /**
     * If a View, returns the base CAS, else returns itself.
     */
    inline CAS* getBaseCas() {
      return iv_baseCas;
    }

    /**
     * Returns true if a CAS view.
     * @deprecated
     */
    inline bool isView() {
      return !isbaseCas;
    }

    /**
     * Resets the CAS. In particular, all data structures meant only to
     * live throughout a document are deleted.
     */
    TyErrorId reset(void);

    /**
     * get the FSFilterBuilder associated with this CAS.
     * @see FSFilterBuilder
     */
    FSFilterBuilder const & getFSFilterBuilder() const;

    /**
     * Get the TypeSystem (const version).
     */
    TypeSystem const & getTypeSystem(void) const;

    /**
     * Get the FSHeap pointer
     */
    uima::lowlevel::FSHeap * getHeap();


    /**
     * create an FS of type <code>crType</code>
     * @param crType the type of FS to create
     * @throws CouldNotCreateFSOfFinalTypeException
     * @return the created feature structure
     */
    FeatureStructure createFS(Type const & crType);

    /**
     * create a feature structure of type FS Array.
     * Each of the uiSize elements in the array is a FeatureStructure.
     * @param uiSize        the size of the array
     */
    ArrayFS createArrayFS(size_t uiSize);

    /**
     * create a feature structure of type FS Array.
     * Each of the uiSize elements in the array is a float.
     * @param uiSize        the size of the array
     */
    FloatArrayFS createFloatArrayFS(size_t uiSize );

    /**
     * create a feature structure of type FS Array.
     * Each of the uiSize elements in the array is a int.
     * @param uiSize        the size of the array
     */
    IntArrayFS createIntArrayFS(size_t uiSize );

    /**
     * create a feature structure of type FS Array.
     * Each of the uiSize elements in the array is a string.
     * @param uiSize        the size of the array
     */
    StringArrayFS createStringArrayFS(size_t uiSize );

    /**
     * create a feature structure of type empty FS list (list length is zero)
     * Each element in the list is a FeatureStructure.
     */
    ListFS createListFS();

    /**
     * create a feature structure of type empty float list (list length is zero).
     * Each element in the list is a float.
     */
    FloatListFS createFloatListFS();

    /**
     * create a feature structure of type empty int list (list length is zero).
     * Each element in the list is a int.
     */
    IntListFS createIntListFS();

    /**
     * create a feature structure of type empty string list (list length is zero).
     * Each element in the list is a string.
     */
    StringListFS createStringListFS();


    /**
     * create a feature structure of type FS Array.
     * Each of the uiSize elements in the array is a byte.
     * @param uiSize        the size of the array
     */
    BooleanArrayFS createBooleanArrayFS(size_t uiSize );


    /**
     * create a feature structure of type FS Array.
     * Each of the uiSize elements in the array is a byte.
     * @param uiSize        the size of the array
     */
    ByteArrayFS createByteArrayFS(size_t uiSize );


    /**
     * create a feature structure of type FS Array.
     * Each of the uiSize elements in the array is a short.
     * @param uiSize        the size of the array
     */
    ShortArrayFS createShortArrayFS(size_t uiSize );


    /**
     * create a feature structure of type FS Array.
     * Each of the uiSize elements in the array is a long.
     * @param uiSize        the size of the arrayt
     */
    LongArrayFS createLongArrayFS(size_t uiSize );

    /**
     * create a feature structure of type FS Array.
     * Each of the uiSize elements in the array is a long.
     * @param uiSize        the size of the array
     */
    DoubleArrayFS createDoubleArrayFS(size_t uiSize );





    /**
     * Return the index repository for index use
     * @throws CASException
     *           if View is the base CAS
     */
    FSIndexRepository & getIndexRepository( void );

    /**
     * Return the index repository for index use (const-version)
     */
    FSIndexRepository const & getIndexRepository( void ) const;

    /**
        * Get the lowlevel IndexRepository
        */
    uima::lowlevel::IndexRepository & getLowlevelIndexRepository(void) const;

    /**
     * Return the base index repository: for internal use only
     */
    FSIndexRepository &
    getBaseIndexRepository( void );


    /**
      * Create a SofaFS
    * @deprecated As of v2.0, use createView() instead.
      */
    SofaFS createSofa(const SofaID & sofaName, const char* mimeType);
    /**
      * Retrieve the SofaFS as identified by the SofaID
    * @deprecated As of v2.0, use getView() instead. From the view you can access the Sofa data, or
    *             call getSofa() if you truly need to access the SofaFS object.
      */
    SofaFS getSofa(const SofaID & sofaName);

    /**
     * Get the Sofa feature structure associated with this CAS view.
     * 
     * @return The SofaFS associated with this View.
     */
    SofaFS getSofa();


    /**
     * Get the view for a Sofa (subject of analysis). The view provides access to the Sofa data and
     * the index repository that contains metadata (annotations and other feature structures)
     * pertaining to that Sofa.
     * 
     * @param localViewName
     *          the local name, before any sofa name mapping is done, for this view (note: this is the
     *          same as the associated Sofa name).
     * 
     * @return The view corresponding to this local name.
     * @throws CASException
     *           if no View with this name exists in this CAS
     */
    CAS* getView(const icu::UnicodeString & localViewName);

    /**
     * Get the view for a Sofa (subject of analysis). The view provides access to the Sofa data and
     * the index repository that contains metadata (annotations and other feature structures)
     * pertaining to that Sofa.
     * 
     * @param aSofa
     *          a Sofa feature structure in the CAS
     * 
     * @return The view for the given Sofa
     */
    CAS* getView(SofaFS aSofa);

    /**
     * Create a view and its underlying Sofa (subject of analysis). The view provides access to the
     * Sofa data and the index repository that contains metadata (annotations and other feature
     * structures) pertaining to that Sofa.
     * 
     * @param localViewName
     *          the local name, before any sofa name mapping is done, for this view (note: this is the
     *          same as the associated Sofa name).
     * 
     * @return The view corresponding to this local name.
     * @throws CASException
     *           if a View with this name already exists in this CAS
     */
    CAS * createView(icu::UnicodeString const & localViewName);

    /**
     * Get the view name. The view name is the same as the name of the view's Sofa, retrieved by
     * getSofa().getSofaID(), except for the initial View before its Sofa has been created.
     * 
     * @return The name of the view
     */
    UnicodeStringRef getViewName();

    /**
     * Set the document text. Once set, Sofa data is immutable, and cannot be set again until the CAS
     * has been reset.
     * 
     * @param text
     *          The text to be analyzed.
     * @exception CASException
     *              If the Sofa data has already been set, or View is base CAS.
     */
    void setDocumentText(UnicodeStringRef const text);

    /**
     * Set the document text, old style.
     * @deprecated Use the new style
     */
    void setDocumentText(UChar const * cpBuffer, size_t uiLength, bool bCopyToCAS = false);

    /**
     * Set the document text. Once set, Sofa data is immutable, and cannot be set again until the CAS
     * has been reset.
     * 
     * @param text
     *          The text to be analyzed.
     * @param mimetype
     *          The mime type of the data
     * @exception CASException
     *              If the Sofa data has already been set, or View is base CAS.
     */
    virtual void setSofaDataString(UnicodeStringRef const text, icu::UnicodeString const & mimetype);

    /**
     * Get the document text.
     * 
     * @return The text being analyzed.
     */
    virtual UnicodeStringRef getDocumentText() const;

    /**
     * Returns the FeaturesStructure of type document annotation representing
     * the current document.
     */
    virtual DocumentFS const getDocumentAnnotation() const;
    virtual DocumentFS getDocumentAnnotation();

    /**
     * Set the Sofa data as an ArrayFS. Once set, Sofa data is immutable, and cannot be set again
     * until the CAS has been reset.
     * 
     * @param array
     *          The ArrayFS to be analyzed.
     * @param mime
     *          The mime type of the data
     * @exception CASException
     *              If the Sofa data has already been set, or View is base CAS.
     */
    virtual void setSofaDataArray(FeatureStructure array, icu::UnicodeString const & mime);

    /**
     * Get the Sofa data array.
     * 
     * @return The Sofa Data being analyzed.
     */
    virtual FeatureStructure getSofaDataArray();

    /**
     * Set the Sofa data as a URI. Once set, Sofa data is immutable, and cannot be set again until the
     * CAS has been reset.
     * 
     * @param uri
     *          The URI of the data to be analyzed.
     * @param mime
     *          The mime type of the data
     * @exception CASException
     *              If the Sofa data has already been set, or View is base CAS.
     */
    virtual void setSofaDataURI(icu::UnicodeString const & uri, icu::UnicodeString const & mime);

    /**
     * Get the Sofa data array.
     * 
     * @return The Sofa Data being analyzed.
     */
    virtual UnicodeStringRef getSofaDataURI();

    /**
     * Get the Sofa data as a byte stream.
     * 
     * @returns a SofaDataStream.
     */
    virtual SofaDataStream * getSofaDataStream();

    /**
    * Informs the CAS of relevant information about the component that is currently procesing it.
    * This is called by the framework automatically; users do not need to call it.
     */
    void setCurrentComponentInfo(AnnotatorContext* info);

    /**
     * Get an iterator over all SofaFS
     * 
     * @returns a FSIterator
     */
    FSIterator getSofaIterator();

    /**
     * create an iterator over all the feature structures in this CAS.
     * @throws InvalidIndexObjectException
     */
    FSIterator iterator() ;

    
	/** @defgroup UtilFuncts functions 
     * CAS Utility functions for Annotations
	 * @{ */
    /**
     * convenience function for creating an annotation.
     * <code>type</code> must be a subtype of type Annotation.
     * Note that this will not commit the new feature structure to the
     * appropriate indexes. To do this CAS::commitFS() has to be called.
     * @throws CASException
     *           if specified type is not an annotation, or View is base CAS.
     */
    virtual AnnotationFS createAnnotation( Type const & type, size_t uiBeginPos, size_t uiEndPos );


    /**
     * CAS Utility functions for Annotation Iterators
     * sets the index iterator to the begin position of <code>crFromAnn</code>.
     *
     * @param itOfType  The iterator to move
     * @param crFromAnn The annotation defining the begin position
     *
     * @return true if <code>beginPositioncrFromAnn</code> is also a valid
     *         begin position of the Annotation the iterator points to and
     *         the movement was successful, false otherwise.
     *         Note that both arguments, i.e. the index iterator and the
     *         FeatureStructure must be annotations.
     */
    bool moveToBeginPosition (ANIterator & itOfType, AnnotationFS const & crFromAnn);
	/** @}  defgroup */

    /** @defgroup PreDefIndexes Predefined Indexes
     * Only very few applications and annotators will need to create specific
     * indexes that are optimized for special iteration needs.
     *
     * Most applications and annotators will be just use some some
     * "natural" way to iterate over the data in the feature structures
     * in the FSStore.
     *
     * For annotation feature structures this is the linear text order
     * with some special sorting for annotations that start at the same
     * position in the text.
     *
     * For vocabulary feature structures a sort order based on lemma string
     * and part of speech is the most accepted one.
     *
     * Those "natural" indexes are predefined in the CAS and can be
     * accessed using the functions described here.
     *
     * For example to access tokens using the index over token annotations
     * write:
     * <pre>
     * Type tokType = cas.getTypeSystem().getType(TT::TYPE_NAME_TOKEN_ANNOTATION);
     * ANIndex tokIdx = cas.getAnnotationIndex(tokType);
     * ANIterator tokIter = tokIdx.iterator();
     * while(tokIter.isValid()) {
     *    AnnotationFS tokFS = tokIter.get();
     *    // ... do some processing with tokFS ...
     *    tokIter.moveToNext();
     * }
     * </pre>
     *  @{
     */
    /**
     * @name Access to predefined indexes
     * @{ */

    /**
     * get the index over all annotations.
     * The index is ordered where an annotation a1 is considered "less-than"
     * another annotation a2 if the begin position of a1 is less then the begin
     * position of a2, or, if they are equal, a1 is shorter than a2.
     *
     * @throws InvalidFSTypeObjectException
     */
    virtual ANIndex getAnnotationIndex();
	  /**
     * get the index over annotations of type <code>crType</code>.
     * The index is ordered where an annotation a1 is considered "less-than"
     * another annotation a2 if the begin position of a1 is less then the begin
     * position of a2, or, if they are equal, a1 is shorter than a2.
     *
     * @param  crType The returned index will return only annoations of type
     *         <code>crType</code>.
     *         <code>crType</code> must be derived from our annotation type.
     *
     * @throws InvalidFSTypeObjectException
     */
    virtual ANIndex getAnnotationIndex(Type const & crType);
    virtual ANIndex const getAnnotationIndex(Type const & crType) const;

    /** @} */
    /** @} */

    /**
     * @name Access to predefined index IDs.
     * This should only be needed to get access to low level indexes.
     * @{ */
    icu::UnicodeString getAnnotationIndexID() const {
      return CAS::INDEXID_ANNOTATION;
    }
    /** @} */

    /** @defgroup PreDefTypes Predefined Types
     *
     * For each predefined type the system provides a string constant
     * for the names of the type.
     * Programmers are strongly encouraged to use them instead of string literals.
     * E.g. write <code>uima::TT::TYPE_NAME_TOKEN_ANNOTATION</code> instead of
     * <code>"uima.tt.TokenAnnotation"</code>
     *
     * @see @link PreDefFeatures Predefined Features@endlink
     * @{
     */
    /** @name CAS string constants for the names of predefined basic types.
     *  @{
     */

    /// Use this instead of a string literal
    static char const * NAME_SPACE_UIMA_CAS;

    /// Use this instead of a string literal
    static char const * TYPE_NAME_TOP;
    /// Use this instead of a string literal
    static  char const * TYPE_NAME_INTEGER;
    /// Use this instead of a string literal
    static char const * TYPE_NAME_STRING;
    /// Use this instead of a string literal
    static char const * TYPE_NAME_FLOAT;
    /// Use this instead of a string literal
    static char const * TYPE_NAME_ARRAY_BASE;
    /// Use this instead of a string literal
    static char const * TYPE_NAME_FS_ARRAY;
    /// Use this instead of a string literal
    static char const * TYPE_NAME_FLOAT_ARRAY;
    /// Use this instead of a string literal
    static char const * TYPE_NAME_INTEGER_ARRAY;
    /// Use this instead of a string literal
    static char const * TYPE_NAME_STRING_ARRAY;
    /// Use this instead of a string literal
    static char const * TYPE_NAME_LIST_BASE;
    /// Use this instead of a string literal
    static char const * TYPE_NAME_FS_LIST;
    /// Use this instead of a string literal
    static char const * TYPE_NAME_EMPTY_FS_LIST;
    /// Use this instead of a string literal
    static char const * TYPE_NAME_NON_EMPTY_FS_LIST;
    /// Use this instead of a string literal
    static char const * TYPE_NAME_FLOAT_LIST;
    /// Use this instead of a string literal
    static char const * TYPE_NAME_NON_EMPTY_FLOAT_LIST;
    /// Use this instead of a string literal
    static   char const * TYPE_NAME_EMPTY_FLOAT_LIST;
    /// Use this instead of a string literal
    static   char const * TYPE_NAME_INTEGER_LIST;
    /// Use this instead of a string literal
    static   char const * TYPE_NAME_NON_EMPTY_INTEGER_LIST;
    /// Use this instead of a string literal
    static   char const * TYPE_NAME_EMPTY_INTEGER_LIST;
    /// Use this instead of a string literal
    static   char const * TYPE_NAME_STRING_LIST;
    /// Use this instead of a string literal
    static   char const * TYPE_NAME_NON_EMPTY_STRING_LIST;
    /// Use this instead of a string literal
    static   char const * TYPE_NAME_EMPTY_STRING_LIST;
    /// Use this instead of a string literal
    static   char const * TYPE_NAME_SOFA;
    /// Use this instead of a string literal
    static   char const * TYPE_NAME_LOCALSOFA;
    /// Use this instead of a string literal
    static   char const * TYPE_NAME_REMOTESOFA;
    /// Use this instead of a string literal
    static   char const * NAME_DEFAULT_TEXT_SOFA;
    /// Use this instead of a string literal
    static   char const * NAME_DEFAULT_SOFA;

    /// Use this instead of a string literal
    static   char const * TYPE_NAME_ANNOTATION_BASE;
    /// Use this instead of a string literal
    static   char const * TYPE_NAME_ANNOTATION;
    /// Use this instead of a string literal
    static   char const * TYPE_NAME_DOCUMENT_ANNOTATION;
    /// Use this instead of a string literal
    static   char const * INDEXID_ANNOTATION;

    /// Use this instead of a string literal
    static   char const * FEATURE_BASE_NAME_SOFA;
    /// Use this instead of a string literal
    static   char const * FEATURE_FULL_NAME_SOFA;
    /// Use this instead of a string literal
    static   char const * FEATURE_BASE_NAME_BEGIN;
    /// Use this instead of a string literal
    static   char const * FEATURE_FULL_NAME_BEGIN;
    /// Use this instead of a string literal
    static   char const * FEATURE_BASE_NAME_END;
    /// Use this instead of a string literal
    static   char const * FEATURE_FULL_NAME_END;
    /// Use this instead of a string literal
    static   char const * FEATURE_BASE_NAME_LANGUAGE;
    /// Use this instead of a string literal
    static   char const * FEATURE_FULL_NAME_LANGUAGE;


    static   char const * TYPE_NAME_BOOLEAN;
    static   char const * TYPE_NAME_BYTE;
    static   char const * TYPE_NAME_SHORT;
    static   char const * TYPE_NAME_LONG;
    static   char const * TYPE_NAME_DOUBLE;

    static   char const * TYPE_NAME_BOOLEAN_ARRAY;
    static   char const * TYPE_NAME_BYTE_ARRAY;
    static   char const * TYPE_NAME_SHORT_ARRAY;
    static   char const * TYPE_NAME_LONG_ARRAY;
    static   char const * TYPE_NAME_DOUBLE_ARRAY;

    /** @} */
    /** @} defgroup*/

    /** @defgroup PreDefFeatures Predefined Features
     * For each predefined feature the system provides a string constant
     * for the names of the feature.
     * Programmers are strongly encouraged to use them instead of string literals.
     * E.g. write <code>CAS::FEATURENAME_LEMMAENTRIES</code> instead of
     * <code>"UIMA_Feature_LemmaEntries"</code>
     * @see @link PreDefTypes Predefined Types@endlink
     * @{
     */
    /** @name CAS string constants for the names of predefined basic features.
     *  @{
     */
    /// Use this instead of a string literal
    static   char const * FEATURE_BASE_NAME_HEAD;
    /// Use this instead of a string literal
    static   char const * FEATURE_BASE_NAME_TAIL;
    /// Use this instead of a string literal
    static   char const * FEATURE_FULL_NAME_FS_LIST_HEAD;
    /// Use this instead of a string literal
    static   char const * FEATURE_FULL_NAME_FS_LIST_TAIL;
    /// Use this instead of a string literal
    ////static   char const * FEATURE_FULL_NAME_FLOAT_LIST_HEAD;
    /// Use this instead of a string literal
    ////static   char const * FEATURE_FULL_NAME_FLOAT_LIST_TAIL;
    /// Use this instead of a string literal
    ////static   char const * FEATURE_FULL_NAME_INTEGER_LIST_HEAD;
    /// Use this instead of a string literal
    ////static   char const * FEATURE_FULL_NAME_INTEGER_LIST_TAIL;
    /// Use this instead of a string literal
    ////static   char const * FEATURE_FULL_NAME_STRING_LIST_HEAD;
    /// Use this instead of a string literal
    ////static   char const * FEATURE_FULL_NAME_STRING_LIST_TAIL;
    /// Use this instead of a string literal
    static   char const * FEATURE_BASE_NAME_SOFANUM;
    /// Use this instead of a string literal
    static   char const * FEATURE_BASE_NAME_SOFAID;
    /// Use this instead of a string literal
    static   char const * FEATURE_BASE_NAME_SOFAMIME;
    /// Use this instead of a string literal
    static   char const * FEATURE_BASE_NAME_SOFAURI;
    /// Use this instead of a string literal
    static   char const * FEATURE_BASE_NAME_SOFASTRING;
    /// Use this instead of a string literal
    static   char const * FEATURE_BASE_NAME_SOFAARRAY;
    /// Use this instead of a string literal
    static   char const * FEATURE_FULL_NAME_SOFANUM;
    /// Use this instead of a string literal
    static   char const * FEATURE_FULL_NAME_SOFAID;
    /// Use this instead of a string literal
    static   char const * FEATURE_FULL_NAME_SOFAMIME;
    /// Use this instead of a string literal
    static   char const * FEATURE_FULL_NAME_SOFAURI;
    /// Use this instead of a string literal
    static   char const * FEATURE_FULL_NAME_SOFASTRING;
    /// Use this instead of a string literal
    static   char const * FEATURE_FULL_NAME_SOFAARRAY;
    /** @} */
    /** @} defgroup*/


    /// @if internal
    /**
     * @name Access to predefined index IDs.
     * This should only be needed to get access to low level indexes.
     * Use functions like getLemmaIndex() to access high level indexes.
     * @{ */
    /// Use this instead of a string literal
    static   char const * INDEXID_SOFA;
    /** @} */
    /// @endif internal


    /**
     * @defgroup CreatorIDs Creator IDs
     * Creator IDs are used with
     * uima::Type::getCreatorID() and uima::Feature::getCreatorID()
     * The ID shows which component has created a type or feature.
     * They are intended for information/debug/display purposes.
     * Currently two creator IDs are predefined:
     * - CAS::ustrCREATOR_ID_CAS for the core types/features created by the CAS (e.g. Integer)
     * - CAS::ustrCREATOR_ID_CAS for the basic linguistic types/features created by the CAS (e.g. Annotation)
     *
     * For types/features not created by CAS the creator id
     * is the name of the component/annotator
     * @{ */
    /**
     * The @link CreatorIDs creator id@endlink constant for types/features created by the CAS (e.g. Integer).
     * See @link CreatorIDs separate section @endlink for details
     */
    static icu::UnicodeString const ustrCREATOR_ID_CAS;
    /** @} defgroup */
  };


  /**
   * The Sofa is implemented as a feature structure of type uima.cas.Sofa.
   * Class SofaFS derives from a standard FeatureStrucure and extends
   * with convenience functions specific for feature structures of type
   * uima.cas.Sofa.
   * <br>
   * The generic CAS Apis must never be used to set/get features
   * of a Sofa.  Instead SofaFS methods must be used to set/get features
   * of a Sofa.
   * <br>
   * The features of the Sofa type include:
   * <ul>
   * <li><strong> SofaID </strong> : Every Sofa in a CAS must have a unique SofaID.
   *         SofaIDs are the primary handle or access.
   * <li><strong> Mime type </strong> : This string feature can be used to describe the type of the data represented by the Sofa.
   * <li><strong>Sofa Data</strong> : The data itself. This data can be resident in the CAS or it can be a reference to data outside the CAS.
   * </ul>
   * To create a Sofa FS, use CAS.createSofa().
   * <br>
   * <br>
   * Sofa data can be contained locally in the CAS itself or it can be
   * remote from CAS. To set the local Sofa data in the Sofa FS use:
   * uima::SofaFS::setLocalSofaData(). If the data is remote from the CAS use:
   * uima::SofaFS::setRemoteSofaURI().
   *
   *
   */
  class UIMA_LINK_IMPORTSPEC SofaFS : public FeatureStructure {
    friend class LocalSofaDataStream;
    friend class CAS;

  protected:

    /**
     * Set the Sofa mime type.
     */
    void setSofaMime(icu::UnicodeString const & aString);

  public:
    /**
     * Default constructor. Creates an invalid SofaFS.
     */
    SofaFS();

    /**
     * Promote a generic FeatureStructure object to an SofaFS.
     * <code>fs</code> must be of type sofa.
     * If <code>fs</code> is not of type sofa the operation will not
     * immediately fail but access to member functions of SofaFS
     * will result in exceptions beeing thrown.
     * This is done by value like all FS API but SofaFS and FS are
     * shallow objects.
     */
    explicit SofaFS(FeatureStructure const & fs);

    /**
     * Set the Local Subject of Analysis to be a predefined ArrayFS.
     */
    void setLocalSofaData(FeatureStructure aFS);

    /**
     * Set the Local Subject of Analysis to be a String.
     */
    void setLocalSofaData(UnicodeStringRef const aString);

    /**
     * Set the Remote Subject of Analysis URI
     */
    void setRemoteSofaURI(const char* aURI);
    void setRemoteSofaURI(icu::UnicodeString const & aString);

    /**
     * Get the Sofa name.
     * @returns String
     */
    UnicodeStringRef getSofaID();

    /**
     * Get the Sofa mime type.
     * @returns String
     */
    UnicodeStringRef getSofaMime();

    /**
     * Get the Sofa URI value.
     * @returns String or null if not valid
     */
    UnicodeStringRef getSofaURI();

    /**
     * Get the Sofa Ref value.
     * @returns the integer identifier for this Sofa
     */
    int getSofaRef();

    /**
     * Get the Sofa FSArray value.
     * @returns FeatureStructure (invalid FS if not valid)
     */
    FeatureStructure getLocalFSData();

    /**
     * Get the Sofa FSArray value.
     * @returns String or null if not valid
     */
    UnicodeStringRef getLocalStringData();

    /**
     * Get the Sofa Data Stream for this SofA fs.
     * Values for one of these features -- sofaString, SofaArray,
     * sofaURI -- must have been set.  
     * If the sofaURI has a valid value, a valid SofaDataStream 
     * is returned only if there is a stream handler registered 
     * for the URI scheme. 
     * @returns a SofaDataStream or null if not valid
     */
    SofaDataStream * getSofaDataStream();

    /**
     * Get the Sofa Data Stream for the give Sofa FeatureStructure.
     * @param fs a valid SofA feature structure with values for one of 
     * these feature -- sofaString, sofaArray, or sofaURI -- set.
     * If the sofaURI feature has a valid value, a valid SofaDataStream 
     * is returned only if there is a stream handler registered  
     * for the URI scheme. 
     * @returns a SofaDataStream or null if not valid
     */
    static SofaDataStream * getSofaDataStream(FeatureStructure & fs);

  protected:
  }
  ; // class SofaFS

  /* ----------------------------------------------------------------------- */
  /*       Implementation                                                    */
  /* ----------------------------------------------------------------------- */

  inline SofaFS::SofaFS() :
      FeatureStructure() {}


  /**
   * This enum is used for annotation iterators to determine there iteration
   * behaviour.
   * This is used in ANIndex::subIterator(), AnnotationFS::subIterator()
   * and ANIterator::subIterator()
   * @see ANIndex
   * @see ANIterator
   * @see AnnotationFS
   */
  typedef enum EnIteratorAmbiguity_ {
    enAmbiguous,   /// Default behaviour: return all annotations, even if several are available for a postion.
    enUnambiguous, /// return only one annotation for a given position.
    enNumberOfIteratorAmbiguityElems // must be last in enum
  } EnIteratorAmbiguity;

  /**
   * Class AnnotationFS derives from a standard FeatureStrucure and extends
   * with convenience functions specific for feature structures of type
   * annotation.
   * The most important functions deal with access to the features begin position
   * and end position as well as access to the text covered by the annotation.
   */
  class UIMA_LINK_IMPORTSPEC AnnotationFS : public FeatureStructure {
  public:
    /**
     * Default constructor. Creates an invalid AnnotationFS.
     */
    AnnotationFS();

    /**
     * Promote a generic FeatureStructure object to an AnnotationFS.
     * <code>fs</code> must be of type annotation.
     * If <code>fs</code> is not of type annotation the operation will not
     * immediately fail but access to member functions of AnnotationFS
     * will result in exceptions beeing thrown.
     * This is done by value like all FS API but AnnotationFS and FS are
     * shallow objects.
     */
    explicit AnnotationFS(FeatureStructure const & fs);

    /**
     * Returns the CAS object in which this feature structure lives.
     * This just saves a cast over the methode CAS::getCAS() inherited
     * from uima::CAS
     */
     CAS & getCAS();
     CAS const & getCAS() const;

    /**
     * Gets the CAS view associated with the Sofa that this Annotation is over.
     * 
     * @return the CAS view associated with the Annotation's Sofa
     */
    CAS * getView();

    /**
     * Get the start position of this annotation feature structure.
     */
    size_t getBeginPosition( void ) const;

    /**
     * Get the end position of this annotation feature structure.
     */
    size_t getEndPosition( void ) const;

    /**
     * Get the number of characters covered by this annotation feature structure.
     * This is just a shortcut for getEndPosition()-getBeginPosition()
     */
    size_t getLength( void ) const;
    /**
     * Get a reference to the text spanned by this annotation feature structure.
     * @throws CASException
     *           if FS is not a valid annotation.
     */
    UnicodeStringRef getCoveredText( void ) const;

    /**
     * Get the first annotation A that covers this annotation, i.e.
     * that has a begin position(A) <= begin position(this) and an
     * end position(A) >= end position(this).
     *
     * Note that the covering relation here is reflexive:
     * getFirstCoveringAnnotation(x, t) == x if t is the type of x
     *
     * If several annotations fulfill these conditions, the one with the
     * starting position nearest begin position relative to this annotation is
     * returned.
     * Of covering annotations with the same starting positions, the one
     * with the biggest end position relative to to this annotation is chosen.
     *
     * @return The next covering features structure as defined above
     *         (!isValid() if none found)
     */
    AnnotationFS getFirstCoveringAnnotation ( Type ofType ) const;

    /**
     * create an iterator over an index of annoations of type crType
     * only returning Annotations with begin position >= this.getBeginPosition()
     * and begin positon < this.getEndPosition()
     *
     * @param crType      The type of annotation over which to iterate.
                          crType must be subsumed by type annotation.
     * @param enAmbiguous If set to CAS::enAmbiguous calling
     *                    moveToNext/moveToPrevious will alway move the resulting
     *                    interator to an annotation that is no longer covered
     *                    by the current annotation.
     *                    This means that:
     *                    moveToNext will always return an annotation with a
     *                    begin position > than the current end position.
     *                    moveToPrevious will always return an annotation with a
     *                    end position < than the current begin position.
     *                    In a situation like this:<br>
     *                    <tt>|--------- Name1 ------||-------- Name2 -------|</tt><br>
     *                    <tt>|-- Tok1 --||-- Tok2 --||-- Tok3 --||-- Tok4 --|</tt><br>
     *                    A normal iterator starting out with Name1 would return:
     *                    Name1, Tok1, Tok2, Name2, Tok3, Tok4
     *                    A unambibous iterator starting out with Name1 would return:
     *                    Name1, Name2
     *                    (This assumes that the types Name and Tok are subsumed
     *                    by crType and no other subusumed annotations cover the
     *                    area.)
     *
     * @throws InvalidIndexObjectException
     */
    ANIterator subIterator( Type const & crType, EnIteratorAmbiguity enAmbiguous = enAmbiguous ) const;
  protected:
  }
  ; // class AnnotationFS

  /* ----------------------------------------------------------------------- */
  /*       Implementation                                                    */
  /* ----------------------------------------------------------------------- */

  inline AnnotationFS::AnnotationFS() :
      FeatureStructure() {}


  inline size_t AnnotationFS::getLength() const {
    return getEndPosition()-getBeginPosition();
  }

  /* ----------------------------------------------------------------------- */
  /*       AnnotationIterator                                                */
  /* ----------------------------------------------------------------------- */
  /**
   * Iterator over AnnotationFS objects in a CAS annotation index (ANIndex).
   * Iterators are created by calling ANIndex.iterator()
   * @see AnnotationFS
   * @see ANIndex
   */
  class UIMA_LINK_IMPORTSPEC ANIterator : public FSIterator {
    friend class ANIndex;
  protected:
    ANIterator(uima::lowlevel::IndexIterator*, uima::CAS*);
  public:
    /// Default CTOR
    ANIterator();
    /// upgrade/conversion CTOR
    explicit ANIterator( FSIterator const & );
    /// retrieve the current element in the index for this iterator.
    AnnotationFS get() const;
  }
  ; // class ANIterator

  /* ----------------------------------------------------------------------- */
  /*       ANIterator Implementation                                         */
  /* ----------------------------------------------------------------------- */

  inline ANIterator::ANIterator(uima::lowlevel::IndexIterator* pIt, uima::CAS * cas) :
      FSIterator(pIt, cas) {
    assert(sizeof(ANIterator) == sizeof(FSIterator));//no additonal data members
  }
  inline ANIterator::ANIterator() :
      FSIterator() {
    assert(sizeof(ANIterator) == sizeof(FSIterator));//no additonal data members
  }

  inline ANIterator::ANIterator(FSIterator const & crOther) :
      FSIterator(crOther) {
    assert(sizeof(ANIterator) == sizeof(FSIterator));//no additonal data members
  }

  inline AnnotationFS ANIterator::get() const {
      return (AnnotationFS)FSIterator::get();
    }

  /* ----------------------------------------------------------------------- */
  /*       AnnotationIndex                                                   */
  /* ----------------------------------------------------------------------- */

  /**
   * This class represents a single index over feature structures of
   * type annotation.
   * @see AnnotationFS
   * @see ANIterator
   */
  class UIMA_LINK_IMPORTSPEC ANIndex : public FSIndex {
  public:
    /**
     * Default constructor. Creates an invalid ANIndex object.
     */
    ANIndex();

    /**
     * Upgrade/Conversion constructor from a standard index.
     */
    explicit ANIndex( FSIndex const & );

    /**
     * create an iterator over all the feature structures in this index.
     * @throws InvalidIndexObjectException
     */
    ANIterator iterator() const;

    /**
     * create an iterator over this index with the filter <code>cpFilter</code>,
     * @see FSFilter
     * @throws InvalidIndexObjectException
     */
    ANIterator filteredIterator(FSFilter const * cpFilter) const;

    /**
     * create an iterator over this index only returning Annotations with
     * begin position >= an.BeginPos and begin positon < an.EndPos
     *
     * @param an          The annotatation "under" which the subiterator
     *                    iterates
     * @param enAmbiguous If set to CAS::enAmbiguous calling
     *                    moveToNext/moveToPrevious will alway move the resulting
     *                    interator to an annotation that is no longer covered
     *                    by the current annotation.
     *                    This means that:
     *                    moveToNext will always return an annotation with a
     *                    begin position > than the current end position.
     *                    moveToPrevious will always return an annotation with a
     *                    end position < than the current begin position.
     *                    In a situation like this:<br>
     *                    <tt>|--------- Name1 ------||-------- Name2 -------|</tt><br>
     *                    <tt>|-- Tok1 --||-- Tok2 --||-- Tok3 --||-- Tok4 --|</tt><br>
     *                    A normal iterator starting out with Name1 would return:
     *                    Name1, Tok1, Tok2, Name2, Tok3, Tok4
     *                    A unambibous iterator starting out with Name1 would return:
     *                    Name1, Name2
     *                    (This assumes that the types Name and Tok are subsumed
     *                    by the type for this index and no other subusumed
     *                    annotations cover the area.)
     * @throws InvalidIndexObjectException
     */
    ANIterator subIterator( AnnotationFS const & an, EnIteratorAmbiguity enAmbiguous = enAmbiguous ) const;

    /**
     * create an iterator over this index such that calling
     * moveToNext/moveToPrevious will alway move the resulting iterator to
     * an annotation that is no longer covered by the current annotation.
     * This means that:
     * moveToNext will always return an annotation with a
     * begin position > than the current end position.
     * moveToPrevious will always return an annotation with a
     * end position < than the current begin position.
     * In a situation like this:<br>
     * <tt>|--------- Name1 ------||-------- Name2 -------|</tt><br>
     * <tt>|-- Tok1 --||-- Tok2 --||-- Tok3 --||-- Tok4 --|</tt><br>
     * A normal iterator starting out with Name1 would return:
     * Name1, Tok1, Tok2, Name2, Tok3, Tok4
     * A unambibous iterator starting out with Name1 would return:
     * Name1, Name2
     * (This assumes that the types Name and Tok are subsumed
     * by the type for this index and no other subusumed annotations cover the
     * area.)
     * @throws InvalidIndexObjectException
     */
    ANIterator unambiguousIterator() const;
  };

  /* ----------------------------------------------------------------------- */
  /*       Implementation                                                    */
  /* ----------------------------------------------------------------------- */

  inline ANIndex::ANIndex() :
      FSIndex() {}

  inline ANIndex::ANIndex( FSIndex const & crFSIndex )
      :  FSIndex(crFSIndex) {
    assert(sizeof(ANIndex) == sizeof(FSIndex)); // no additional data members
  }

  inline ANIterator ANIndex::iterator() const {
    return (ANIterator)FSIndex::iterator();
  }

  inline ANIterator ANIndex::filteredIterator(FSFilter const * cpFilter) const {
    return (ANIterator)FSIndex::filteredIterator(cpFilter);
  }


  /**
   * Class DocumentFS derives from AnnotationFS and extends
   * with convenience functions specific for the single feature structures
   * of type Document-Annotation that is present for each input document.
   * The most important additional functions deal with access to the
   * features document language and the document text.
   * To get an easy access to this annotation just call the function
   * CAS::getDocumentAnnotation()
   */
  class UIMA_LINK_IMPORTSPEC DocumentFS : public AnnotationFS {
  public:
    /**
     * Default constructor. Creates an invalid DocumentFS.
     */
    DocumentFS();

    /**
     * Promote a generic FeatureStructure object to an DocumentFS.
     * <code>fs</code> must be of document annotation.
     * If <code>fs</code> is not of document annotation the operation will not
     * immediately fail but access to member functions of AnnotationFS
     * will result in exceptions beeing thrown.
     */
    explicit DocumentFS(FeatureStructure const & fs);

    /**
     * Returns the language of the current document.
     * The return value may be invalid (<TT>!Language.isValid()</TT>) if
     * this function is called when no valid document is present.
     * The return value may be unspecified (<TT>== CosEnLanguage_Unspecified</TT>)
     * If no language was specified for the document and no language
     * detection annotator has run.
     */
    Language getLanguage() const;

    /**
     * Set the language of the current document.
     */
    void setLanguage(Language const &);

  private:

    friend class CAS;

  }
  ; // class DocumentFS

  /* ----------------------------------------------------------------------- */
  /*       Implementation                                                    */
  /* ----------------------------------------------------------------------- */

  inline DocumentFS::DocumentFS() :
      AnnotationFS() {}

}

#endif



