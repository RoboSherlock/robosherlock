/** \file xmideserializer_handler.hpp .


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


    \brief  SAX2 handler for reading XMI into a CAS.

-------------------------------------------------------------------------- */
#ifndef __UIMA_XMIDESER_HANDLER_HPP
#define __UIMA_XMIDESER_HANDLER_HPP


// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include <uima/pragmas.hpp> //must be first to surpress warnings
#include <map>
#include <stack>
#include <utility>
#include "xercesc/sax2/DefaultHandler.hpp"
#include <uima/internal_casimpl.hpp>
#include "xmishareddata.hpp"

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
     The class <TT>XmiDeserializerHandler</TT> implements a SAX2 handler for XMI format
  */

#define DOC_STATE                       0
#define FS_STATE                        1
#define FEAT_STATE                      2
#define FEAT_CONTENT_STATE              3
#define IGNORING_XMI_ELEMENTS_STATE     4
           
  class  XmiDeserializerHandler : public DefaultHandler {
  public:
    // -----------------------------------------------------------------------
    //  Constructors and Destructor
    // -----------------------------------------------------------------------  
    XmiDeserializerHandler(CAS & cas, XmiSerializationSharedData * xmiSharedData, bool lenient=true);   
		~XmiDeserializerHandler();

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
  
    //void readFS(UnicodeString & qualifiedName, const Attributes & attrs);
    void readFS(UnicodeString & nsUri,
		        UnicodeString & localName,
				    UnicodeString & qualifiedName, 
				    const Attributes & attrs);
    void readFS(lowlevel::TyFS addr, const Attributes  & attrs, bool toIndex);
    void handleFeature(lowlevel::TyFS addr, 
		         UnicodeString & featName, 
					   UnicodeString & featVal, 
					   bool lenient);
	  void handleFeature(Type & type, lowlevel::TyFS addr, 
					   lowlevel::TyFSFeature featCode, 
					   UnicodeString & featVal,
             bool lenient);

    void finalizeFS(int addr);
    void finalizeArray(Type & type, lowlevel::TyFS addr); 
  
    const Locator *  iv_locator;
    CAS * iv_cas;
    internal::CASImpl & iv_casimpl;
    const lowlevel::TypeSystem * iv_typesystem;
    int iv_state;
    UnicodeString buffer;

    // The address of the most recently created FS.  Needed for array elements
    // and embedded feature values.
    lowlevel::TyFS currentAddr;

    // The name of the content feature, if we've seen one.
    UnicodeString currentContentFeat;

    // The current position when parsing array elements.
    size_t arrayPos;

    // The type of the array we're currently reading.  Needed for proper
    // treatment of array element values.
    lowlevel::TyFS arrayType;

    // SofaFS type
    int sofaTypeCode;

    // Store IndexRepositories in a vector;
    std::vector<uima::lowlevel::IndexRepository *> indexRepositories;

    // Store CAS Views in a vector
    std::vector<CAS*> tcasInstances;

    int nextIndex;

		UnicodeString xmiElementName2uimaTypeName(UnicodeString& nameSpaceURI, UnicodeString& localName);
		int createByteArray(UnicodeString& currentArrayElements, int currentArrayId);
		void remapFSListHeads(int addr);

		void tokenize(UnicodeString&, std::vector<std::string>&);
		int createIntList(  std::vector<std::string>& featVal);
		int createFloatList( std::vector<std::string>& featVal);
		int createStringList(  std::vector<std::string>& featVal);
		int createFSList(  std::vector<std::string>& featVal);

		void addArrayElement(lowlevel::TyFS addr,lowlevel::TyFSType arrayType, 
											int arrayPos, std::string & buffer);

		void handleFeature(lowlevel::TyFS addr, UnicodeString & featName,
												std::vector<std::string> & featVal);

		void handleFeature(lowlevel::TyFS addr, lowlevel::TyFSFeature featCode,
								lowlevel::TyFSType rangeTypeCode,std::vector<std::string> & featVal);

		int createArray(  lowlevel::TyFSType typeCode,
								std::vector<std::string>& featVal, int xmiID);

		void processView(int sofaXmiId, UnicodeString & membersString) ;
		int getFsAddrForXmiId(int xmiId);
		void addToOutOfTypeSystemData(XmlElementName * xmlElementName, const Attributes & attrs);
		void addOutOfTypeSystemFeature(OotsElementData * ootsElem, 
					UnicodeString & featName, std::vector<UnicodeString> & featVals);

		// container for data shared between the XmiCasSerialier and
		// XmiDeserializer, to support things such as consistency of IDs across
		// multiple serializations.  This is also where the map from xmi:id to
		// FS address is stored.
		XmiSerializationSharedData * sharedData;  
		bool ownsSharedData;

		//Current out-of-typesystem element, if any
		OotsElementData * outOfTypeSystemElement;

		// Store address of every FS we've deserialized, since we need to back
		// and apply fix-ups afterwards.
		std::vector<int> deserializedFsAddrs;

		// map from namespace prefixes to URIs.
		std::map<UnicodeString, UnicodeString> nsPrefixToUriMap;
		// map from xmi namespace  to uima namespace 
		std::map<UnicodeString, UnicodeString> xmiNamespaceToUimaNamespaceMap;

		//typename - values
		std::map<UnicodeString, std::vector<UnicodeString>* > multiValuedFeatures; 
		int ignoreDepth;

		// The type of the most recently created FS. Needed for arrays, also
		// useful for embedded feature values.
		Type currentType;

		// the ID and values of arrays are stored on startElement, then used on
		// endElement to actually create the array. This is because in the case of
		// String arrays serialized with the values as child elements, we can't create
		// the array until we've seen all of the child elements.
		int currentArrayId;
		UnicodeString currentArrayElements;

		int nextSofaNum; //number of sofas found so far

		// Store a separate vector of FSList nodes that were deserialized 
		// from multivalued properties.
		// These are special because their "head" feature needs remapping but their "tail" feature
		// doesn't.
		std::vector<int> fsListNodesFromMultivaluedProperties;
		bool lenient;

		static char const  * XMI_ID_ATTR_NAME;
		static char const  * TRUE_VALUE;
		static char const  * DEFAULT_CONTENT_FEATURE;
		static char const  * DEFAULT_NAMESPACE_URI;

  };

} // namespace uima

#endif //__UIMA_XMIDESER_HANDLER_HPP

