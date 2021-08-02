/** \file xcasdeserializer_handler.hpp .


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


    \brief  SAX2 handler for reading XCAS into a CAS.

-------------------------------------------------------------------------- */
#ifndef __UIMA_XCASDESER_HANDLER_HPP
#define __UIMA_XCASDESER_HANDLER_HPP


// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include <uima/pragmas.hpp> //must be first to surpress warnings
#include <map>
#include <stack>
#include <utility>
#include "xercesc/sax2/DefaultHandler.hpp"
#include <uima/internal_casimpl.hpp>

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  class FeatureStructure;
  class FSIndexRepository;
  class SofaFS;
  class CAS;
  class AnnotationFS;
  class AnnotatorContext;
  namespace lowlevel {
    class IndexRepository;
    class FSHeap;
    class TypeSystem;
  }
  namespace internal {
    class CASImpl;
  }
}

XERCES_CPP_NAMESPACE_USE


namespace uima {

  /**
     The class <TT>XCASDeserializerHandler</TT> implements a SAX2 handler for XCAS format
  */

#define DOC_STATE                       0
#define FS_STATE                        1
#define FEAT_STATE                      2
#define CONTENT_STATE                   3
#define FEAT_CONTENT_STATE              4
#define ARRAY_ELE_CONTENT_STATE         5
#define ARRAY_ELE_STATE                 6
#define DOC_TEXT_STATE                  7

  class UIMA_LINK_IMPORTSPEC FSInfo  {
  public:
    int addr;
    std::vector<int>* indexRep;

    FSInfo(int addr, std::vector<int>* indexRep)  {
      this->addr = addr;
      this->indexRep = indexRep;
    }
  };

  class UIMA_LINK_IMPORTSPEC XCASDeserializerHandler : public DefaultHandler {
  public:
    // -----------------------------------------------------------------------
    //  Constructors and Destructor
    // -----------------------------------------------------------------------

    XCASDeserializerHandler(CAS & acas, AnnotatorContext  * const ctx);

    ~XCASDeserializerHandler();

    void startDocument();
    void startElement(const   XMLCh* const    uri,
                      const   XMLCh* const    localname,
                      const   XMLCh* const    qname,
                      const Attributes& attrs);
    void characters(const XMLCh* const chars,
                    const unsigned int length);
    void endDocument();
    void endElement(const XMLCh* const uri,
                    const XMLCh* const localname,
                    const XMLCh* const qname
                   );
    void ignorableWhitespace(const XMLCh* const chars,
                             const unsigned int length);
    void setDocumentLocator(const Locator* const locator);


    void warning(const SAXParseException& exception);
    void error(const SAXParseException& exception);
    void fatalError(const SAXParseException& exception);


  private:

    void readFS(icu::UnicodeString & qualifiedName, const Attributes & attrs);
    void readFS(lowlevel::TyFS addr, const Attributes  & attrs, bool toIndex);
    void handleFeature(lowlevel::TyFS addr, icu::UnicodeString & featName, icu::UnicodeString & featVal, bool lenient);
    void handleFeature(Type & type, lowlevel::TyFS addr, icu::UnicodeString & featName, icu::UnicodeString & featVal,
                       bool lenient);
    void finalizeFS(FSInfo & fsInfo);
    void readArray(Type & type, const Attributes & attrs);
    void readArrayElement(icu::UnicodeString & qualifiedName, const Attributes & attrs);
    void addArrayElement(icu::UnicodeString & buffer);
    void finalizeArray(Type & type, lowlevel::TyFS addr, FSInfo & fsInfo);

    const Locator *  iv_locator;
    CAS * iv_cas;
    internal::CASImpl & iv_casimpl;
    const lowlevel::TypeSystem * iv_typesystem;
    int iv_state;
    icu::UnicodeString buffer;

    // The address of the most recently created FS.  Needed for array elements
    // and embedded feature values.
    lowlevel::TyFS currentAddr;

    // The name of the content feature, if we've seen one.
    icu::UnicodeString currentContentFeat;

    // The current position when parsing array elements.
    size_t arrayPos;

    // The type of the array we're currently reading.  Needed for proper
    // treatment of array element values.
    lowlevel::TyFS arrayType;

    // for processing old style TCAS
    CAS * oldTcas;

    // SofaFS type
    int sofaTypeCode;

    // Store IndexRepositories in a vector;
    std::vector<uima::lowlevel::IndexRepository *> indexRepositories;

    // Store CAS Views in a vector
    std::vector<CAS*> tcasInstances;

    //store FS information for end of document processing
    std::map<int, FSInfo*> fsTree;
    std::vector<FSInfo*> idLess;

    //maps for deserializing v1.x format XCAS documents
    std::vector<int> sofaRefMap;
    std::vector<int> indexMap;
    int nextIndex;


    AnnotatorContext * const iv_ctx;

    static char const  * CASTAGNAME;
    static char const  * DEFAULT_DOC_TYPE_NAME ;
    static char const  * DEFAULT_DOC_TEXT_FEAT;
    static char const  * INDEXED_ATTR_NAME;
    static char const  * REF_PREFIX;
    static char const  * ID_ATTR_NAME;
    static char const  * CONTENT_ATTR_NAME;
    static char const  * ARRAY_SIZE_ATTR;
    static char const  * ARRAY_ELEMENT_TAG;
    static char const  * TRUE_VALUE;
    static char const  * DEFAULT_CONTENT_FEATURE;


  };




} // namespace uima

#endif //__UIMA_XMLPARSE_HANDLERS_HPP

