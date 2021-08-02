/** @name xmideserializer_handler.cpp
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


   10/18/2005  Initial creation

-------------------------------------------------------------------------- */

//TODO support multiple indexed FS

// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include <uima/pragmas.hpp>
#include <iostream>
#include <sstream>
#include <algorithm>
using namespace std;

#include "xercesc/sax2/Attributes.hpp"
#include "xercesc/sax/SAXParseException.hpp"
#include "xercesc/sax/SAXException.hpp"
#include <uima/msg.h>
#include <uima/exceptions.hpp>
#include <uima/lowlevel_typesystem.hpp>
#include <uima/lowlevel_indexrepository.hpp>

#include <uima/xmideserializer_handler.hpp>
#include <uima/internal_fspromoter.hpp>
#include <uima/internal_typeshortcuts.hpp>
#include <uima/internal_casimpl.hpp>
#include <uima/fsindexrepository.hpp>
#include <uima/arrayfs.hpp>
#include <uima/listfs.hpp>
#include <uima/annotator_context.hpp>
#include <uima/resmgr.hpp>


namespace uima {

// ---------------------------------------------------------------------------
//  XmiDeserialiserHandler: Constructors and Destructor
// ---------------------------------------------------------------------------

	XmiDeserializerHandler::XmiDeserializerHandler(CAS & cas,
		XmiSerializationSharedData * xmiSharedData, bool len) : iv_cas(cas.getBaseCas() ),
		iv_locator(NULL), iv_casimpl( uima::internal::CASImpl::promoteCAS(*iv_cas)),
		sharedData(xmiSharedData), ownsSharedData(false), outOfTypeSystemElement(NULL) {
	        lenient = len;
			if (this->sharedData==NULL) {
				this->sharedData = new XmiSerializationSharedData();
				ownsSharedData=true;
			} else {
				lenient=true;
			}
			//cout << " XmiDeserializerHandler::constructor " << endl;
			currentContentFeat.append(DEFAULT_CONTENT_FEATURE);
			sofaTypeCode = uima::internal::gs_tySofaType;
			FSIndexRepository * fsidx = &iv_cas->getBaseIndexRepository();
			indexRepositories.push_back((lowlevel::IndexRepository*)fsidx);
			// There should always be another index for the Initial View
			fsidx = &iv_cas->getView(CAS::NAME_DEFAULT_SOFA)->getIndexRepository();
			indexRepositories.push_back((lowlevel::IndexRepository*)fsidx);

			// get temp heap handle for checking if an FS is an annotation
			lowlevel::FSHeap const & crHeap = iv_casimpl.getHeap();
			//       uima::lowlevel::FSHeap::TyFSHeap const & tyTempHeap = crHeap.iv_clTemporaryHeap;
			iv_typesystem = &crHeap.getTypeSystem();

			ignoreDepth=0;
			nextSofaNum=2;
		}

		XmiDeserializerHandler::~XmiDeserializerHandler()   {
			//cout << " XmiDeserializerHandler::destructor " << endl;

			if (ownsSharedData)  {
				delete sharedData;
			}
			//cout << " XmiDeserializerHandler::destructor done " << endl;
		}


// ---------------------------------------------------------------------------
//  XmiDeserializerHandler: Implementation of the SAX2 ContentHandler interface
// ---------------------------------------------------------------------------

  void  XmiDeserializerHandler::setDocumentLocator(const Locator* const locator) {
    //cout << " XmiDeserializerHandler::setDocumentLocator() " << endl;
    iv_locator = locator;
  }

  void XmiDeserializerHandler::startDocument() {
    //cout << " XmiDeserializerHandler::startDocument() " << endl;
    iv_state = DOC_STATE;
  }

	void XmiDeserializerHandler::startElement(const   XMLCh* const    uri,
		const   XMLCh* const    localname,
		const   XMLCh* const    qname,
		const Attributes & attrs) {
			//cout << " XmiDeserializerHandler::startElement() qname " <<icu::UnicodeString((UChar*)qname, XMLString::stringLen(qname)) << endl;
			//cout << "startElement localname " << icu::UnicodeString(localname) << " uri " << icu::UnicodeString(uri) << endl;
			//cout << "startElement attrs " << attrs.getLength() << endl;


			assert(sizeof(XMLCh) == sizeof(UChar));

			icu::UnicodeString qualifiedName(qname);
			buffer.remove(); 

			switch (iv_state) {
		case DOC_STATE: {
			//cout << "startElement DOC_STATE " <<  attrs.getLength() << endl;
			// allow any root element name
			// extract xmlns:prefix=uri attributes into a map, which we can use to
			// resolve the prefixes even with a non-namespace-aware parser
			if (attrs.getLength() != 0) {
				for (size_t i = 0; i < attrs.getLength(); i++) {
					icu::UnicodeString attrName(attrs.getQName(i));
					//cout << "xmlns attrName " << attrName << endl;
					if (attrName.indexOf("xmlns:") > -1 ) {
						icu::UnicodeString prefix;
						attrName.extract(6, attrName.length()-6, prefix);
						icu::UnicodeString uri(attrs.getValue(i));
						nsPrefixToUriMap[prefix]= uri;
					}
				}
			}
			iv_state = FS_STATE;
			break;
										}
		case FS_STATE: {
			// ignore elements with XMI prefix (such as XMI annotations)  
			if (qualifiedName.indexOf("xmi") > 0) {
				this->iv_state = IGNORING_XMI_ELEMENTS_STATE;
				this->ignoreDepth++;
				return;
			}

			icu::UnicodeString unsuri(uri);
			icu::UnicodeString ulocalname(localname);
			// parser not namespace-enabled, so try to resolve NS ourselves
			// TODO test with non namespace-enabled
			/**
			int colonIndex = qualifiedName.indexOf(":");
			if (colonIndex != -1) {
			icu::UnicodeString prefix;
			qualifiedName.extract(0, colonIndex,prefix);
			map<icu::UnicodeString,UnicodeString>::iterator uriite = nsPrefixToUriMap.find(prefix);
			if (uriite != nsPrefixToUriMap.end()) { 
			nameSpaceURI = uriite->second;
			} else {
			// unbound namespace. Rather than failing, just assume a reasonable default.
			nameSpaceURI.append("http:///");
			nameSpaceURI.append(prefix);
			nameSpaceURI.append(".ecore");
			}
			colonIndex++;
			qualifiedName.extract(colonIndex, qualifiedName.length()-colonIndex,localName );
			} else { // no prefix. Use default URI 
			nameSpaceURI = DEFAULT_NAMESPACE_URI;
			}
			**/
			//cout << "startElement FS_STATE calling readFS " << typeName << endl; 
			//readFS(typeName, attrs);
			readFS(unsuri, ulocalname, qualifiedName, attrs);

			map<icu::UnicodeString, vector<icu::UnicodeString>*>::iterator mite;
            for (mite=multiValuedFeatures.begin();
				mite != multiValuedFeatures.end(); mite++) {
					if (mite->second != NULL) {
						delete mite->second;
					}
			}
			multiValuedFeatures.clear();
			iv_state = FEAT_STATE;
			break;	   
									 }
		case FEAT_STATE: {
			iv_state = FEAT_CONTENT_STATE;
			break;				 
										 }
		case IGNORING_XMI_ELEMENTS_STATE: {
			ignoreDepth++;
			break;								 
																			}
		default: {
			// If we're not in an element expecting state, raise an error.
			ErrorInfo errInfo;
			errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
			ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
			assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
			msg.addParam( qualifiedName );
			errInfo.setMessage(msg);
			errInfo.setSeverity(ErrorInfo::unrecoverable);
			ExcIllFormedInputError exc(errInfo);
			throw exc;
						 }
			} //switch
		}

	void XmiDeserializerHandler::characters(
					const   XMLCh* const  cpwsz,
					const unsigned int    uiLength) {
		assert(sizeof(XMLCh) == sizeof(UChar));

		if (this->iv_state == FEAT_CONTENT_STATE) {
			buffer.append( (UChar const *) cpwsz, 0, uiLength );
		}
		/**
		switch (this->iv_state)  {
			case FEAT_CONTENT_STATE:
				buffer.append( (UChar const *) cpwsz, 0, uiLength );
				break;
			default:
				break;
		}**/
	}

	void XmiDeserializerHandler::endElement(const XMLCh* const nsuri,
						const XMLCh* const localname,
						const XMLCh* const qname) {
		/**
		cout << " XmiDeserializerHandler::endElement() qname " 
		<< icu::UnicodeString( (UChar*) qname, XMLString::stringLen(qname) ) << endl;
		cout << " XmiDeserializerHandler::endElement() uri " 
		<< icu::UnicodeString( (UChar*) nsuri, XMLString::stringLen(nsuri) ) << endl;
		**/

		assert(sizeof(XMLCh) == sizeof(UChar));
		icu::UnicodeString qualifiedName( (UChar const *) qname, XMLString::stringLen(qname));
		//cout << "endElement qualifiedname " << qualifiedName << endl;
		switch (iv_state) {
		case DOC_STATE: {
			// Do nothing.
			break; 
		}
		case FS_STATE: {
			iv_state = DOC_STATE;
			break;
		}
		case FEAT_CONTENT_STATE: {
			// We have just processed one of possibly many values for a feature.
			// Store this value in the multiValuedFeatures map for later use.
			//cout << "endELement FEAT_CONTENT_STATE " << buffer << endl;
			map<icu::UnicodeString, vector<icu::UnicodeString>*>::iterator ite =
				multiValuedFeatures.find(qualifiedName);
			vector<icu::UnicodeString> * valuesList=0; 
			if (ite == multiValuedFeatures.end()) {
				valuesList = new vector<icu::UnicodeString>;
				multiValuedFeatures[qualifiedName] = valuesList;
			} else {
				valuesList = ite->second;
			}
			if (valuesList==0) {
				cout << "endELement()FEAT_CONTENT_STATE valuesList not created" << endl;
			}
			else valuesList->push_back(buffer);

			// go back to the state where we're expecting a feature
			iv_state = FEAT_STATE;
			break;
		}
		case FEAT_STATE: {
			// end of FS. Process multi-valued features or array elements that were
			// encoded as subelements
			if (this->outOfTypeSystemElement != NULL) {
				if (this->multiValuedFeatures.size() > 0) {
					map<icu::UnicodeString,vector<icu::UnicodeString>*>::iterator ite;
					for (ite=multiValuedFeatures.begin(); ite != multiValuedFeatures.end();ite++) {
						icu::UnicodeString featName = ite->first;
						vector<icu::UnicodeString>* featVals = ite->second;
						addOutOfTypeSystemFeature(outOfTypeSystemElement, featName, *featVals);
					}
				}
				this->outOfTypeSystemElement = NULL;
			}
			//process the multivalued feature or array elements that 
			//were encoded as subelements.
			//cout << "endElement FEAT_STATE " << qualifiedName << endl;
			else if (currentType.isValid()) {
				int typecode = internal::FSPromoter::demoteType(currentType);
				if ( iv_cas->getTypeSystem().isArrayType(typecode) && 
					typecode != internal::gs_tyByteArrayType) {
						// create the array now. elements may have been provided either as
						// attributes or child elements, but not both.
						// BUT - not byte arrays! They are created immediately, to avoid
						// the overhead of parsing into a String array first
						vector<string> featVals;
						// cout << "endElement FEAT_STATE currentArrayElements " <<
						//	    currentArrayElements.length() << endl;
						if (currentArrayElements.length()==0) // were not specified as attributes
						{
							map<icu::UnicodeString, vector<icu::UnicodeString>*>::iterator ite =
								multiValuedFeatures.find(icu::UnicodeString("elements"));
							if (ite != multiValuedFeatures.end()) {
								vector<icu::UnicodeString>* vals = ite->second;
								for (size_t i=0; i<vals->size(); i++) {
									featVals.push_back( ((UnicodeStringRef)vals->at(i)).asUTF8()); 
								}
							}
						} else {
							tokenize(currentArrayElements,featVals);
						}         
						createArray(internal::FSPromoter::demoteType(currentType), featVals, currentArrayId);
					} else {
						map<icu::UnicodeString,vector<icu::UnicodeString>*>::iterator ite;
						for (ite=multiValuedFeatures.begin(); ite != multiValuedFeatures.end();ite++) {
							icu::UnicodeString featName = ite->first;
							vector<icu::UnicodeString>* featVals = ite->second;
							vector<string> stringList;
							for (size_t i=0; i< featVals->size();i++) {
								stringList.push_back( ((UnicodeStringRef)featVals->at(i)).asUTF8());
							}
							handleFeature(currentAddr, featName, stringList);
						}
					}
			}
			iv_state = FS_STATE;
			break;
		} 
		case IGNORING_XMI_ELEMENTS_STATE: {
			ignoreDepth--;
			if (ignoreDepth == 0) {
				iv_state = FS_STATE;
			}
			break;
		}
	}
}


	void XmiDeserializerHandler::endDocument() {

		//cout << " XmiDeserializerHandler::endDocument() " << endl;

		//fix up deserialized FSs
		for (size_t i = 0; i < this->deserializedFsAddrs.size(); i++) {
			//cout << "finalize fs " << deserializedFsAddrs.at(i) << endl;
			finalizeFS(deserializedFsAddrs.at(i));
		}

		//fix up lists
		for (size_t i = 0; i < fsListNodesFromMultivaluedProperties.size(); i++) {
			this->remapFSListHeads(fsListNodesFromMultivaluedProperties.at(i));
		}

		//cout << " XmiDeserializerHandler::endDocument() tcasInstance " << endl;
		//update document annotation info in tcas
		for (size_t i = 0; i < tcasInstances.size(); i++) {
			CAS * tcas = (CAS *) tcasInstances[i];
			if (tcas != 0) {
				tcas->pickupDocumentAnnotation();
			}
		}
		//cout << " XmiDeserializerHandler::endDocument() " << endl;
	}


  void XmiDeserializerHandler::ignorableWhitespace(const  XMLCh* const cpwsz,
      const unsigned int length) {
    //cout << " XmiDeserializerHandler::ignorableWhitespace() " << endl;
  }


/**
   * Converts an XMI element name to a UIMA-style dotted type name.
   * 
   */
icu::UnicodeString XmiDeserializerHandler::xmiElementName2uimaTypeName(icu::UnicodeString& nsUri, icu::UnicodeString& localName) {
		// check map first to see if we've already computed the namespace mapping
		map<icu::UnicodeString,icu::UnicodeString>::iterator ite = xmiNamespaceToUimaNamespaceMap.find(nsUri);
		icu::UnicodeString uimaNamespace;
		if (ite != xmiNamespaceToUimaNamespaceMap.end()) {
			uimaNamespace = ite->second;
		} else {
			// check for the special "no-namespace" URI, which is used for UIMA types with no namespace
			if (nsUri.compare(DEFAULT_NAMESPACE_URI) == 0) {
				//uimaNamespace = "";
			} else {
				// Our convention is that the UIMA namespace is the URI path format e.g:
				// http:///uima/cas.ecore.
				// remove http:/// and trailing .ecore
				// replace remaining slashes with dot.
				nsUri.extractBetween(8, nsUri.length()-6, uimaNamespace); 
				//cout << "uimanamespace " << uimaNamespace << endl;
				uimaNamespace.findAndReplace("/", ".");
				uimaNamespace.append("."); // include trailing dot for convenience
			}
			xmiNamespaceToUimaNamespaceMap[nsUri]= uimaNamespace;
		}
		//cout << "uimaNamespace final " << uimaNamespace << endl;
		uimaNamespace.append(localName);
		return uimaNamespace;
	}


// Create a new FS.
	void XmiDeserializerHandler::readFS(icu::UnicodeString & nsUri, icu::UnicodeString & localName,
		icu::UnicodeString & qualifiedName, const Attributes & attrs) {
			icu::UnicodeString typeName = xmiElementName2uimaTypeName(nsUri, localName);
			Type type = iv_cas->getTypeSystem().getType(typeName);
			currentType=type;
			
			if (!type.isValid()) {
				if (typeName.compare(icu::UnicodeString("uima.cas.NULL"))==0) {
					//cout << "readFS ignore " << typeName << endl;
					return; //ignore
				}
				if (typeName.compare(icu::UnicodeString("uima.cas.View"))==0) {
					//cout << "readFS  " << typeName << endl;
					icu::UnicodeString attrName; 
					int sofaXmiId=0;
					icu::UnicodeString members;
					for (size_t i = 0; i < attrs.getLength(); i++) {
						attrName =  attrs.getQName(i);
						if (attrName.compare(CAS::FEATURE_BASE_NAME_SOFA) == 0) {
							icu::UnicodeString ustr(attrs.getValue(i));
							sofaXmiId = atoi( ((UnicodeStringRef)ustr).asUTF8().c_str());
						} else if (attrName.compare("members") == 0) {
							members = attrs.getValue(i);
						} 
					}
					processView(sofaXmiId, members);
					return;
				} 
				// type is not in our type system
				if (!lenient) {
					ErrorInfo errInfo;
					errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
					ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
					assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
					msg.addParam(typeName);
					errInfo.setMessage(msg);
					errInfo.setSeverity(ErrorInfo::unrecoverable);
					ExcIllFormedInputError exc(errInfo);
					throw exc; 
				} else {
					this->addToOutOfTypeSystemData(
						new XmlElementName( ((UnicodeStringRef)nsUri).asUTF8(),
						((UnicodeStringRef)localName).asUTF8(),
						((UnicodeStringRef)qualifiedName).asUTF8()), attrs );   
					return;
				}
			} else if (iv_cas->getTypeSystem().isArrayType(internal::FSPromoter::demoteType(type)) ) {
				
				icu::UnicodeString attrName; 
				int xmiId=0;
				icu::UnicodeString elements;
				for (size_t i = 0; i < attrs.getLength(); i++) {
						attrName =  attrs.getQName(i);
						if (attrName.compare(XMI_ID_ATTR_NAME) == 0) {
							icu::UnicodeString ustr(attrs.getValue(i));
							currentArrayId = atoi( ((UnicodeStringRef)ustr).asUTF8().c_str());
						} else if (attrName.compare("elements") == 0) {
							currentArrayElements = attrs.getValue(i);
						} 
				}

        //cout << "xmiId " << currentArrayId << " type=" << typeName << " elements=" << currentArrayElements << endl;
				if (internal::FSPromoter::demoteType(type) == internal::gs_tyByteArrayType) {
					int addr = createByteArray(currentArrayElements, currentArrayId);
				} 

			} else {
				//cout << "readFS() create FS and read attributes " << typeName << endl;
				uima::lowlevel::TyFS addr = uima::internal::FSPromoter::demoteFS(iv_cas->createFS(type));
				readFS(addr, attrs, true);
			}

		}



	/**
	* Handles the processing of a cas:View element in the XMI. The cas:View element encodes indexed
	* FSs.
	* 
	* @param sofa
	*          xmi:id of the sofa for this view, null indicates base CAS "view"
	* @param membersString
	*          whitespace-separated string of FS addresses. Each FS is to be added to the specified
	*          sofa's index repository
	*/
	void XmiDeserializerHandler::processView(int sofaXmiId, icu::UnicodeString & members) {
		// TODO: this requires View to come AFTER all of its members
		//cout << "processView start " << sofaXmiId << "members=" << membersString << endl;
		if (members.length() > 0) { 
			// a view with no Sofa will be added to the 1st, _InitialView, index
			int sofaNum = 1;
			if (sofaXmiId != 0) {
				// translate sofa's xmi:id into its sofanum
				//cout << __LINE__ << " calling getFsAddrForXmiId " << sofaXmiId << endl;
				int sofaAddr = getFsAddrForXmiId(sofaXmiId);

				sofaNum = iv_cas->getHeap()->getIntValue(sofaAddr, internal::gs_tySofaNumFeature);
			}
			lowlevel::IndexRepository * indexRep =  indexRepositories.at(sofaNum);

			vector<string> memberList;
			tokenize(members, memberList);

			for (size_t i = 0; i < memberList.size(); i++) {
				// have to map each ID to its "real" address
				int addr=0;
        int amember = atoi(memberList.at(i).c_str());
				try {
					//cout << __LINE__ << " calling getFsAddrForXmiId " << members.at(i) << endl;
					addr = getFsAddrForXmiId(amember);
					indexRep->addFS(internal::FSPromoter::promoteFS(addr,*iv_cas));
				} catch (Exception e) {
					if (!lenient) {
						throw e;
					}
					else {
						//unknown view member may be an OutOfTypeSystem FS
						//cout << "calling sharedData->addOutOfTypeSystemViewMember" << endl;
						this->sharedData->addOutOfTypeSystemViewMember(sofaXmiId, amember);
					}
				}
			}
		}
	}


	int XmiDeserializerHandler::createByteArray(icu::UnicodeString& currentArrayElements, int currentArrayId) {
		string elemStr = ( (UnicodeStringRef) currentArrayElements).asUTF8();
		int arrayLen = elemStr.length() / 2;
		ByteArrayFS fs = iv_cas->createByteArrayFS(arrayLen);
		size_t j=0;
		for (int i = 0; i < arrayLen; i++) {
			char hex[5], *stop;
			hex[0] = '0';
			hex[1] = 'x';
			if (j < elemStr.length() ) {
				hex[2] = elemStr.at(j++);
				if (j < elemStr.length()) {
					hex[3] = elemStr.at(j++);
					hex[4] = 0;
					char val = strtol(hex, &stop, 16);
					fs.set(i,val);
				}
			}
		}  
		int arrayAddr = internal::FSPromoter::demoteFS(fs);
		deserializedFsAddrs.push_back(arrayAddr);
		if (currentArrayId > 0) {
			sharedData->addIdMapping(arrayAddr,currentArrayId);
		}

		return arrayAddr;
	}

	void XmiDeserializerHandler::readFS(lowlevel::TyFS addr, const Attributes  & attrs, bool toIndex) {
		// Hang on address for setting content feature
		currentAddr = addr;

		int id = -1;
		//       int sofaRef = -1; // 0 ==> baseCas indexRepository
		////vector<int>* sofaRef = new vector<int>;
		icu::UnicodeString attrName;
		icu::UnicodeString attrValue;
		bool nameMapping = false;
		UChar ubuff[256];
		UErrorCode errorCode = U_ZERO_ERROR;
		int thisSofaNum;
		lowlevel::TyFS heapValue = iv_casimpl.getHeap().getType(addr);


		if (sofaTypeCode == heapValue) {
			int extsz = icu::UnicodeString(CAS::FEATURE_BASE_NAME_SOFAID).extract(ubuff, 256, errorCode);
			if (extsz > 256) {
				cout << "ACK!" << endl;
			}
			const UChar* sofaID = attrs.getValue(ubuff);

			if (0==UnicodeStringRef(sofaID).compare(icu::UnicodeString("_DefaultTextSofaName"))) {  
				// initial view Sofa always has sofaNum = 1
				thisSofaNum = 1;
			} else if (0==UnicodeStringRef(sofaID).compare(icu::UnicodeString(CAS::NAME_DEFAULT_SOFA))) {   
				thisSofaNum = 1;
			}  else {
				thisSofaNum = this->nextSofaNum++;
			}
		}

		Type type = uima::internal::FSPromoter::promoteType(heapValue, iv_cas->getTypeSystem().getLowlevelTypeSystem());
		for (size_t i = 0; i < attrs.getLength(); i++) {
			assertWithMsg( sizeof(XMLCh) == sizeof(UChar), "Port required!");
			attrName = (UChar*)attrs.getQName(i);
			attrValue = (UChar*)attrs.getValue(i);

			if (attrName.compare(icu::UnicodeString(XMI_ID_ATTR_NAME)) == 0) {
				id = atoi(UnicodeStringRef(attrValue).asUTF8().c_str());
				//cout << "got " << XMI_ID_ATTR_NAME << " " << id << endl;
			} else {       
				if (sofaTypeCode == heapValue && attrName.compare(CAS::FEATURE_BASE_NAME_SOFAID)==0) {
					if (attrValue.compare(icu::UnicodeString("_DefaultTextSofaName"))==0 ) {
						// First change old default Sofa name into the new one
						attrValue =icu::UnicodeString(CAS::NAME_DEFAULT_SOFA);
					}
				} else if (sofaTypeCode == heapValue 
					&& attrName.compare(icu::UnicodeString(CAS::FEATURE_BASE_NAME_SOFANUM))==0) {
						stringstream str;
						str << thisSofaNum << endl;
						attrValue =icu::UnicodeString(str.str().c_str());
					}
					//cout << "readFS calling handleFeature " << attrName << " attrvalue= "
					//	<< attrValue << endl;
					handleFeature(addr, attrName, attrValue, true);
			}
		}

		if (sofaTypeCode == heapValue) {
			// If a Sofa, create CAS view to get new indexRepository
			SofaFS sofa = (SofaFS) uima::internal::FSPromoter::promoteFS(addr, *iv_cas);
			//also add to indexes so we can retrieve the Sofa later
			iv_cas->getBaseIndexRepository().addFS(sofa);
			CAS * tcas = iv_cas->getView(sofa);
			assert ( EXISTS(tcas) );
			if (sofa.getSofaRef() == 1) {
				iv_cas->registerInitialSofa();
			} else {
				// add indexRepo for views other than the initial view
				lowlevel::IndexRepository * indexRep = iv_cas->getIndexRepositoryForSofa(sofa);
				assert ( EXISTS(indexRep) );
				indexRepositories.push_back(indexRep);
			}
			tcasInstances.push_back(tcas);
		}

		deserializedFsAddrs.push_back(addr);
		if (id > 0) {
			sharedData->addIdMapping(addr, id);
		}

	}

  

  
  void XmiDeserializerHandler::addArrayElement(lowlevel::TyFS addr,
											lowlevel::TyFSType arrayType, 
											int arrayPos, 
											string & buffer) {

    if (arrayPos >= (int) iv_casimpl.getHeap().getArraySize(addr) ) {
      ErrorInfo errInfo;
      errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
      ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
      assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
      msg.addParam("Invalid array FS in the CAS" );
      errInfo.setMessage(msg);
      errInfo.setSeverity(ErrorInfo::unrecoverable);
      ExcIllFormedInputError exc(errInfo);
      throw exc;
    }

    FeatureStructure fs = uima::internal::FSPromoter::promoteFS(addr, *iv_cas);

    switch (arrayType) {
    case internal::gs_tyIntArrayType: {
		//cout << "add intarray element at " << arrayPos << " " << buffer << endl;
        int val = atoi(buffer.c_str());
        IntArrayFS intFS(fs);
        intFS.set( (size_t) arrayPos, val);
        break;
      }
    case internal::gs_tyFloatArrayType: {
        float val = atof(buffer.c_str());
        FloatArrayFS floatFS(fs);
        floatFS.set( (size_t) arrayPos, val);
        break;
      }
    case internal::gs_tyStringArrayType: {
        //add the striug
				StringArrayFS strFS(fs);
				icu::UnicodeString strval(buffer.c_str());
				strFS.set( (size_t) arrayPos,strval);
				break;
    }
    case internal::gs_tyByteArrayType: {
        short intval = atoi(buffer.c_str());
        char charval[2];
        sprintf(charval,"%c",intval);
        ByteArrayFS byteFS(fs);
        byteFS.set( (size_t) arrayPos, charval[0]);
        break;
    }
    case internal::gs_tyBooleanArrayType: {
        BooleanArrayFS booleanFS(fs);
        if (buffer.compare("true") == 0)  {
          booleanFS.set( (size_t) arrayPos, true);
          //cout << "bool buffer " << buffer << " val= " << val << "set " << true << endl;
        } else {
          booleanFS.set ( (size_t) arrayPos, false);
          //cout << arrayPos << " bool buffer " << buffer << " val= " << val << "set " << false << endl;
        }
        break;
      }
    case internal::gs_tyShortArrayType: {
        short val;
        //string strval;
        //UnicodeStringRef(buffer).extractUTF8(strval);
        stringstream s;
        s << buffer.c_str();
        s >> val;
        ShortArrayFS shortFS(fs);
        shortFS.set( (size_t) arrayPos, val);
        break;
      }
    case internal::gs_tyLongArrayType: {
        INT64 val;
        stringstream s;
        s << buffer;
        s >> val;
        LongArrayFS longFS(fs);
        longFS.set( (size_t) arrayPos, val);
        break;
      }
    case internal::gs_tyDoubleArrayType: {
        DoubleArrayFS doubleFS(fs);
        stringstream s;
        s << buffer;
        long double doubleval;
        s >> doubleval;
        doubleFS.set((size_t) arrayPos, doubleval);
        break;
      }
    default: {    //array of FSs
      lowlevel::TyFS fsid = atoi(buffer.c_str());
      FeatureStructure fsitem(fsid, *iv_cas);
      ArrayFS fsArrayfs(fs);
      fsArrayfs.set((size_t) arrayPos, fsitem);
    }
    } //swithch
  }



	// Create a feature value from a string representation.
	void XmiDeserializerHandler::handleFeature(lowlevel::TyFS addr,icu::UnicodeString & featName, icu::UnicodeString & featVal, bool lenient) {
		lowlevel::TyFSType fstype = iv_casimpl.getHeap().getType(addr);
		Type type = uima::internal::FSPromoter::promoteType(fstype, iv_cas->getTypeSystem().getLowlevelTypeSystem());
		Feature feat = type.getFeatureByBaseName(featName);
		if (!feat.isValid()) {
			if (!lenient) {
				ErrorInfo errInfo;
				errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
				ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
				assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
				msg.addParam("Unknown Feature");
				msg.addParam(featName);
				errInfo.setMessage(msg);
				errInfo.setSeverity(ErrorInfo::unrecoverable);
				ExcIllFormedInputError exc(errInfo);
				throw exc; 
			}
			else {
				sharedData->addOutOfTypeSystemAttribute(addr, featName, featVal);
			}
			return;
		}

		lowlevel::TyFSFeature featCode = internal::FSPromoter::demoteFeature(feat);
		handleFeature(type, addr, featCode, featVal, lenient);
	}

  void XmiDeserializerHandler::handleFeature(Type & type, 
    lowlevel::TyFS addr,
    lowlevel::TyFSFeature featCode, 
    icu::UnicodeString & featVal,
    bool lenient) {

    FeatureStructure fs = uima::internal::FSPromoter::promoteFS(addr, *iv_cas);
    if (!fs.isValid() ) {
      cerr << "handle feature of Invalid FS " << type.getName() << endl;
      ErrorInfo errInfo;
      errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
      ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
      assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
      msg.addParam("Invalid FeatureStructure");
      msg.addParam(type.getName());
      errInfo.setMessage(msg);
      errInfo.setSeverity(ErrorInfo::unrecoverable);
      ExcIllFormedInputError exc(errInfo);
      throw exc; 
    }

    ///Feature feat = type.getFeatureByBaseName(featName);
	  Feature feat =  internal::FSPromoter::promoteFeature(featCode, *iv_typesystem);

    Type rtype;
    feat.getRangeType(rtype);
    lowlevel::TyFSType rangeType = uima::internal::FSPromoter::demoteType(rtype);
    switch (rangeType) {
      case internal::gs_tyIntegerType: {
        if (featVal.length()>0) {
          if (featCode == internal::gs_tySofaRefFeature) {
            // special handling for "sofa" feature of annotation. Need to change
            // it from a sofa reference into a sofa number
            int sofaXmiId = atoi(UnicodeStringRef(featVal).asUTF8().c_str());
            //cout << __LINE__ << " calling getFsAddrForXmiId " << sofaXmiId << endl; 
            int sofaAddr = getFsAddrForXmiId(sofaXmiId); 
            int sofaNum = iv_cas->getHeap()->getFSValue(sofaAddr, internal::gs_tySofaNumFeature);
            iv_cas->getHeap()->setFSValue(addr,featCode,sofaNum);

          } else {             
            fs.setIntValue(feat, atoi(UnicodeStringRef(featVal).asUTF8().c_str()));
          }
        }
        break;
      }
      case internal::gs_tyFloatType: { 
        if ( featVal.length() > 0)  {
          fs.setFloatValue(feat, atof(UnicodeStringRef(featVal).asUTF8().c_str()));
        }
        break;
      }
      case internal::gs_tyStringType: {
       // if (featVal.length() > 0) {
          fs.setStringValue(feat, featVal);
        //}
        break;
      }
      case internal::gs_tyByteType: {
        if (featVal.length() > 0) {
          string val = UnicodeStringRef(featVal).asUTF8();
          short intval = atoi(val.c_str());
          char charval[2];
          sprintf(charval,"%c",intval);
          fs.setByteValue(feat, charval[0] );
        }
        break;
      }
      case internal::gs_tyBooleanType: {
        if (featVal.length() > 0) {
          string val = UnicodeStringRef(featVal).asUTF8();
          if (val.compare("1")==0 || val.compare("true") == 0)
            fs.setBooleanValue(feat, true );
          else fs.setBooleanValue(feat, false);
        }
        break;
      }
      case internal::gs_tyShortType: {
        if (featVal.length() > 0) {
          string strval = UnicodeStringRef(featVal).asUTF8();
          short shortval;
          stringstream s;
          s << strval.c_str();
          s >> shortval;
          fs.setShortValue(feat, shortval);
        }
        break;
      }
      case internal::gs_tyLongType: {
        if (featVal.length() > 0) {
          string strval = UnicodeStringRef(featVal).asUTF8();
          INT64 longval;
          stringstream s;
          s << strval.c_str();
          s >> longval;
          fs.setLongValue(feat, longval);
        }
        break;
      }
      case internal::gs_tyDoubleType: {
        if (featVal.length() > 0) {
          string strval = UnicodeStringRef(featVal).asUTF8();
          long double doubleval;
          stringstream s;
          s << strval.c_str();
          s >> doubleval;
          fs.setDoubleValue(feat, doubleval );
        }
        break;
      }	  
      case internal::gs_tyBooleanArrayType:
      case internal::gs_tyByteArrayType:
      case internal::gs_tyIntArrayType: 
      case internal::gs_tyFloatArrayType: 
      case internal::gs_tyStringArrayType: 
      case internal::gs_tyLongArrayType:
      case internal::gs_tyShortArrayType:
      case internal::gs_tyDoubleArrayType: 
      case internal::gs_tyFSArrayType: {
        //cout << "handleFeature " << feat.getName() << " " << featVal << endl;
        if (feat.isMultipleReferencesAllowed()) {
          // do the usual FS deserialization
          //cout << "  multiplerefsallowed " << endl;
          if (featVal.length() > 0) {
            int val = atoi(UnicodeStringRef(featVal).asUTF8().c_str());
            //cout << " setting fsvalue " << "fsaddr " << addr << " value "<< val << endl;
            iv_cas->getHeap()->setFeatureInternal(addr,featCode,val);
          }
        } else {
          // Do the multivalued property deserialization.
          // However, byte arrays have a special serialization (as hex digits)
          //cout << " not multiplerefsallowed " << endl;
          if (rangeType == internal::gs_tyByteArrayType) {
            int arrayAddr = createByteArray(featVal, -1);
            iv_cas->getHeap()->setFSValue(addr,featCode,arrayAddr);
          } else {
            //cout << "tokenizing array values " << endl;
            vector<string> stringList;
            tokenize(featVal, stringList);
            handleFeature(addr, featCode, rangeType, stringList);
          }
        }
        break;
      }
      case internal::gs_tyFloatListType:
      case internal::gs_tyIntListType:
      case internal::gs_tyStringListType: 
      case internal::gs_tyFSListType: {
        //cout << "GOT A LIST FEATURE " << endl;
        if (feat.isMultipleReferencesAllowed()) {
          // do the usual FS deserialization
          if (featVal.length() > 0) {
            int val = atoi(UnicodeStringRef(featVal).asUTF8().c_str());
            iv_cas->getHeap()->setFeatureInternal(addr,featCode,val);
          }
        } else {
          // Do the multivalued property deserialization.
          ////handleFeature(addr,featCode,featVal);
          vector<string> stringList;
          tokenize(featVal, stringList);
          handleFeature(addr, featCode, rangeType, stringList);
        }
        break;
      }
      default: {
        if (rtype.isStringSubType()) {
          if (featVal.length() > 0) {
            fs.setStringValue(feat, featVal);
          }
        } else if (featVal.length() > 0) {
          lowlevel::TyFS val = (lowlevel::TyFS) atoi(UnicodeStringRef(featVal).asUTF8().c_str());
          iv_casimpl.getHeap().setFeatureInternal(addr, uima::internal::FSPromoter::demoteFeature(feat), val);
        }
        break;
      }
    }
  }
 
  void XmiDeserializerHandler::handleFeature(lowlevel::TyFS addr, 
                          icu::UnicodeString & featName,
                          vector<string> & featVal) {
    lowlevel::TyFSType fstype = iv_casimpl.getHeap().getType(addr);
    Type type = uima::internal::FSPromoter::promoteType(fstype, iv_cas->getTypeSystem().getLowlevelTypeSystem());
    Feature feat = type.getFeatureByBaseName(featName);
    if (!feat.isValid()) {
      if (!lenient) {
        ErrorInfo errInfo;
        errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
        ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
        assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
        msg.addParam("Unknown Feature");
        msg.addParam(featName);
        errInfo.setMessage(msg);
        errInfo.setSeverity(ErrorInfo::unrecoverable);
        ExcIllFormedInputError exc(errInfo);
        throw exc; 
      }
      else {
        sharedData->addOutOfTypeSystemChildElements(addr, ( (UnicodeStringRef)featName).asUTF8(), featVal);
      }
      return;
    }
    lowlevel::TyFSFeature featCode = internal::FSPromoter::demoteFeature(feat);
    Type rtype;
    feat.getRangeType(rtype);
    handleFeature(addr, featCode,internal::FSPromoter::demoteType(rtype), featVal);
}

void XmiDeserializerHandler::handleFeature(lowlevel::TyFS addr, 
		                lowlevel::TyFSFeature featCode,
					          lowlevel::TyFSType rangeTypeCode,
					          vector<string> & featVals)  {
    //cout << "handleFeature array/list  " << featVals.size() << endl;
	 switch(rangeTypeCode) {
	   case internal::gs_tyBooleanArrayType:
	   case internal::gs_tyByteArrayType:
	   case internal::gs_tyIntArrayType: 
	   case internal::gs_tyFloatArrayType: 
	   case internal::gs_tyLongArrayType:
	   case internal::gs_tyShortArrayType:
	   case internal::gs_tyDoubleArrayType:
	   case internal::gs_tyFSArrayType: {
		   int arrayFS = createArray(rangeTypeCode, featVals, -1);  
		   iv_cas->getHeap()->setFSValue(addr,featCode,arrayFS);
		   break;
	   }
	   case internal::gs_tyStringArrayType: {
		   //cout << "handleFeature of type string array" << endl;
		   int arrayFS = createArray(rangeTypeCode, featVals, -1);  
		   iv_cas->getHeap()->setFSValue(addr,featCode,arrayFS);
		   break;
	   }
	   case internal::gs_tyIntListType: {
		   int arrayFS = createIntList(featVals);
           iv_cas->getHeap()->setFSValue(addr,featCode,arrayFS);
		   break;
	   }
	   case internal::gs_tyFloatListType: {
		   int arrayFS = createFloatList(featVals);
           iv_cas->getHeap()->setFSValue(addr,featCode,arrayFS);
		   break;
	   }
	   case internal::gs_tyStringListType: {
		   int arrayFS = createStringList(featVals);
           iv_cas->getHeap()->setFSValue(addr,featCode,arrayFS);
		   break;
	   }
	   case internal::gs_tyFSListType: {
		   int arrayFS = createFSList(featVals);
           iv_cas->getHeap()->setFSValue(addr,featCode,arrayFS);
		   break;
	   }   
	   default: {
		//scalar and FS type
		   if (featVals.size() != 1) {
				 //TODO log
			   cerr << "one feature value expected " << endl;
		   } else {
			   Type type = internal::FSPromoter::promoteType(rangeTypeCode,
				   iv_cas->getTypeSystem().getLowlevelTypeSystem());
			   icu::UnicodeString val(featVals.at(0).c_str());
			   handleFeature(type,		   
				   addr, featCode,val, true);
		   }
	  	break;	
	   }
	  }
  }

  void XmiDeserializerHandler::tokenize(icu::UnicodeString & ustr, vector<string> & stringList ) {

	  string str = (UnicodeStringRef(ustr)).asUTF8();
	  string::size_type lastPos = str.find_first_not_of(" ", 0); 
	  string::size_type pos = str.find_first_of(" ",lastPos);
    
	  while (string::npos != pos || string::npos != lastPos) {
        // Found a token, add it to the vector.
        stringList.push_back(str.substr(lastPos, pos - lastPos));
        // Skip blanks and find next non blank
        lastPos = str.find_first_not_of(" ", pos);
        // Find next blank
        pos = str.find_first_of(" ",lastPos);
    }
  }


  int XmiDeserializerHandler::createArray(  lowlevel::TyFSType typeCode,
		               vector<string>& stringList,
					          int xmiId)		{
    int arrayAddr;
  
	  switch (typeCode) {
      case internal::gs_tyBooleanArrayType:
	    case internal::gs_tyByteArrayType:
	    case internal::gs_tyIntArrayType: 
	    case internal::gs_tyFloatArrayType: 
	    case internal::gs_tyLongArrayType:
	    case internal::gs_tyShortArrayType:
	    case internal::gs_tyDoubleArrayType:
	    case internal::gs_tyStringArrayType:
	    case internal::gs_tyFSArrayType: {
	      //cout << "createArray() type " << typeCode << " size " << stringList.size() << endl;
		    arrayAddr = iv_cas->getHeap()->createArrayFS(typeCode, stringList.size());
		    //cout << "created array FS now adding element values at address " << arrayAddr << endl;
		    for (size_t i=0; i < stringList.size();i++) {		
			    addArrayElement(arrayAddr, typeCode,i,stringList.at(i));
		    }		 
		    break;
		  }        
	    default: {
	      cerr << "Invalid Array type" << endl;
		    ErrorInfo errInfo;
		    errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
		    ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
		    assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
		    msg.addParam("createArray failed.");
		    stringstream str;
		    str << "xmiId=" << xmiId << " typecode= " << typeCode << endl;
		    msg.addParam(str.str().c_str());
		    errInfo.setMessage(msg);
		    errInfo.setSeverity(ErrorInfo::unrecoverable);
		    ExcIllFormedInputError exc(errInfo);
		    throw exc; 
		    break;
	    }
	  }
		//cout << "createArray " << xmiId << " addr=" << arrayAddr << endl;
	  deserializedFsAddrs.push_back(arrayAddr);
	  if (xmiId > 0) {
		  sharedData->addIdMapping(arrayAddr, xmiId);
	  }
	  return arrayAddr;
  }
    
  int XmiDeserializerHandler::createIntList(vector<string>& stringList)	{
	  IntListFS listFS =   iv_cas->createIntListFS();
    for (size_t i = 0; i < stringList.size(); i++ ) {
      int value = atoi (stringList.at(i).c_str()); 
	    listFS.addLast(value);
    }
	  return internal::FSPromoter::demoteFS(listFS);
  }

  int XmiDeserializerHandler::createFloatList(vector<string>& stringList)	{
	  FloatListFS listFS =   iv_cas->createFloatListFS();
    for (size_t i = 0; i < stringList.size(); i++ ) {
      float value = atof (stringList.at(i).c_str()); 
	    listFS.addLast(value);
    }
	  return internal::FSPromoter::demoteFS(listFS);
  }

  int XmiDeserializerHandler::createFSList(vector<string>& stringList)	{
	  int first = iv_cas->getHeap()->createFS(internal::gs_tyEListType);

	  size_t i = stringList.size();
	  for  (;i > 0;i--) {
		  int value = atoi(stringList.at(i-1).c_str());
		  int node = iv_cas->getHeap()->createFS(internal::gs_tyNEListType);
		  fsListNodesFromMultivaluedProperties.push_back(node);
		  iv_cas->getHeap()->setFeatureInternal(node, internal::gs_tyHeadFeature, value);
		  iv_cas->getHeap()->setFeatureInternal(node, internal::gs_tyTailFeature, first);
		  first = node;
	  }
	  return first;
  }

  int XmiDeserializerHandler::createStringList(vector<string>& stringList)	{   
    StringListFS listFS =   iv_cas->createStringListFS();
    for (size_t i = 0; i < stringList.size(); i++ ) {
	    icu::UnicodeString value(stringList.at(i).c_str()); //use xmiId to look up addr
	    listFS.addLast(value);
    }
	  return internal::FSPromoter::demoteFS(listFS);
  }

  void XmiDeserializerHandler::remapFSListHeads(int addr) {
    int type = iv_cas->getHeap()->getType(addr);
	  if (type != internal::gs_tyIntListType &&
		  type != internal::gs_tyFloatListType &&
		  type != internal::gs_tyStringListType &&
		  type != internal::gs_tyFSListType &&
		  type != internal::gs_tyNEListType) {
      return;
	  }

	  int headFeat = internal::gs_tyHeadFeature;
    int featVal = iv_cas->getHeap()->getFeatureInternal(addr, headFeat);
    if (featVal != 0) {
      int fsValAddr = 0;
	  	try {
		  	  //cout << __LINE__ << " remap calling getFsAddrForXmiId " << featVal << endl;
		  	  fsValAddr = getFsAddrForXmiId(featVal);
		  } catch (Exception e) {
			  if (!lenient) {
			    throw e;
			  } else {
				  stringstream str;
				  str << featVal;
				  this->sharedData->addOutOfTypeSystemAttribute(addr, CAS::FEATURE_BASE_NAME_HEAD, str.str());
			  }
		  }
		  iv_cas->getHeap()->setFeatureInternal(addr, headFeat, fsValAddr);
    }
  }


  void XmiDeserializerHandler::finalizeFS(int deserializedfsaddr) {
    lowlevel::TyFS addr = deserializedfsaddr; 
    FeatureStructure fs = uima::internal::FSPromoter::promoteFS(addr, *iv_cas);
    Type type = fs.getType();
    if (iv_cas->getTypeSystem().isArrayType(uima::internal::FSPromoter::demoteType(type)) ) {    
      finalizeArray(type, addr); 
      return;
    }

    //update heap value of features that are references to other FS.
    vector<Feature> feats;
    type.getAppropriateFeatures(feats);

    for (size_t i = 0; i < feats.size(); i++) {
      Feature feat = (Feature) feats[i];
      Type rangeType;
      feat.getRangeType(rangeType);
      if (rangeType.isValid()) {
        lowlevel::TyFSType  rangetypecode = uima::internal::FSPromoter::demoteType(rangeType);
        lowlevel::TyFSFeature featcode = uima::internal::FSPromoter::demoteFeature(feat);

        //if not primitive
        if (iv_cas->getTypeSystem().isFSType(rangetypecode)  ||
          (iv_cas->getTypeSystem().isArrayType(rangetypecode) && 
          feat.isMultipleReferencesAllowed() ) || 
          (iv_cas->getTypeSystem().isListType(rangetypecode) && 
          feat.isMultipleReferencesAllowed()) ) {
            //get the current feature value which is the id
            lowlevel::TyFS featVal = iv_casimpl.getHeap().getFeatureInternal(addr, featcode);
            if (featVal != 0) {
              int fsValAddr = 0;
              try {
                //cout << __LINE__ << feat.getName() << " calling getFsAddrForXmiId " << featVal << endl;
                fsValAddr = getFsAddrForXmiId(featVal);		 
              } catch (Exception e) {
                if (!lenient) {
                  throw e;
                }
                else {
                  //this may be a reference to an out-of-typesystem FS
                  stringstream str;
                  str << featVal;
                  this->sharedData->addOutOfTypeSystemAttribute(addr, 
                      ((UnicodeStringRef)feat.getName()).asUTF8(), str.str());
                }       
              }
              iv_casimpl.getHeap().setFSValue(addr, featcode, fsValAddr);
            }
         }
      }
    }
  }


  void XmiDeserializerHandler::finalizeArray(Type & type, lowlevel::TyFS addr) {

    lowlevel::TyFSType typecode = uima::internal::FSPromoter::demoteType(type);
    if (!iv_cas->getTypeSystem().isFSArrayType(typecode)) {
      return;
    }
    // *** WARNING ***  *** WARNING ***  *** WARNING ***  *** WARNING ***
    // if implementation of ArrayFS on the heap changes, this code will be invalid
    int size = (int)iv_cas->getHeap()->getHeap().getHeapValue(addr + 1);
	
    for (int i=0; i<size; i++) {
      lowlevel::TyFS arrayVal = iv_cas->getHeap()->getHeap().getHeapValue(addr + 2 + i);
      if (arrayVal != 0) {
        int arrayValAddr = 0;
        try {
          //cout << __LINE__ << " calling getFsAddrForXmiId " << arrayVal << endl;
          arrayValAddr = getFsAddrForXmiId(arrayVal);
        } catch (Exception e) {
          if (!lenient) {
            throw e;
          }
          else {  
            // the array element may be out of typesystem.  In that case set it
            // to null, but record the id so we can add it back on next serialization.
            this->sharedData->addOutOfTypeSystemArrayElement(addr, i, arrayVal);
          }
        }
        iv_cas->getHeap()->getHeap().setHeapValue(addr + 2 + i, arrayValAddr);
      }
    }
  }

   /**
     * Gets the FS address into which the XMI element with the given ID
     * was deserialized.  This method supports merging multiple XMI documents
     * into a single CAS, by checking the XmiSerializationSharedData
     * structure to get the address of elements that were skipped during this
     * deserialization but were deserialized during a previous deserialization.
     * 
     * @param xmiId
     * @return
     */
  int XmiDeserializerHandler::getFsAddrForXmiId(int xmiId) {
    int addr = sharedData->getFsAddrForXmiId(xmiId);
		//cout << "xmiid=" << xmiId << "fsaddr=" << addr << endl;
    if (addr > 0)
        return addr;
	   else {  
      //cerr << __FILE__<<__LINE__ <<  " throw exc No such xmiid " << xmiId << endl;
		  ErrorInfo errInfo;
		  errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
		  ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
		  assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
		  msg.addParam("getFsAddrForXmiId");
		  msg.addParam(xmiId);
		  errInfo.setMessage(msg);
		  errInfo.setSeverity(ErrorInfo::unrecoverable);
		  ExcIllFormedInputError exc(errInfo);
		  throw exc; 
	  }
  }

	/**
   * Adds a feature sturcture to the out-of-typesystem data.  Also sets the
   * this->outOfTypeSystemElement field, which is referred to later if we have to
   * handle features recorded as child elements.
   */
  void XmiDeserializerHandler::addToOutOfTypeSystemData(XmlElementName * xmlElementName, const Attributes & attrs) {
    this->outOfTypeSystemElement = new OotsElementData();
    //this->outOfTypeSystemElement->elementName = xmlElementName->qualifiedName;
	  this->outOfTypeSystemElement->elementName = xmlElementName;
	  icu::UnicodeString attrName;
	  icu::UnicodeString attrValue;
    for (size_t i = 0; i < attrs.getLength(); i++) {
      attrName = attrs.getQName(i);
      attrValue = attrs.getValue(i);
      if (attrName.compare(icu::UnicodeString(XMI_ID_ATTR_NAME))==0) {
		    UnicodeStringRef uref(attrValue);
        this->outOfTypeSystemElement->xmiId = atoi(uref.asUTF8().c_str());
      }
      else {
        this->outOfTypeSystemElement->attributes.push_back(
                  new XmlAttribute(attrName, attrValue));
      }
    }
    this->sharedData->addOutOfTypeSystemElement(this->outOfTypeSystemElement);
  }    

    /**
     * Adds a feature to the out-of-typesystem features list.
     * @param ootsElem object to which to add the feature
     * @param featName name of feature
     * @param featVals feature values, as a list of strings
     */
  void XmiDeserializerHandler::addOutOfTypeSystemFeature(OotsElementData * ootsElem, 
		icu::UnicodeString & featName, 
		vector<icu::UnicodeString> & featVals) {
	  vector<string> * pVals = new vector<string>;
    for (size_t i=0;i<featVals.size();i++) {
		  pVals->push_back(  ((UnicodeStringRef)featVals.at(i)).asUTF8());    
    }
	  ootsElem->childElements[ ((UnicodeStringRef)featName).asUTF8()] = pVals;
  } 
// ---------------------------------------------------------------------------
//  XmiDeserializerHandler: Overrides of the SAX ErrorHandler interface
// ---------------------------------------------------------------------------
  void XmiDeserializerHandler::error(const SAXParseException& e) {
    ErrorInfo errInfo;
    errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
    ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_ERROR);
    assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
    msg.addParam((UChar const *)e.getSystemId());
    msg.addParam(e.getLineNumber());
    msg.addParam(e.getColumnNumber());
    msg.addParam((UChar const *) e.getMessage());
    errInfo.setMessage(msg);
    errInfo.setSeverity(ErrorInfo::unrecoverable);
    ExcIllFormedInputError exc(errInfo);
    throw exc;
  }

  void XmiDeserializerHandler::fatalError(const SAXParseException& e) {
    ErrorInfo errInfo;
    errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
    ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
    assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
    msg.addParam((UChar const *)e.getSystemId());
    msg.addParam(e.getLineNumber());
    msg.addParam(e.getColumnNumber());
    msg.addParam((UChar const *) e.getMessage());
    errInfo.setMessage(msg);
    errInfo.setSeverity(ErrorInfo::unrecoverable);
    ExcIllFormedInputError exc(errInfo);
    throw exc;
  }

  void XmiDeserializerHandler::warning(const SAXParseException& e) {
    ErrorInfo errInfo;
    errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
    ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_WARNING);
    assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
    msg.addParam((UChar const *)e.getSystemId());
    msg.addParam(e.getLineNumber());
    msg.addParam(e.getColumnNumber());
    msg.addParam((UChar const *) e.getMessage());
    errInfo.setMessage(msg);
    errInfo.setSeverity(ErrorInfo::unrecoverable);
    ExcIllFormedInputError exc(errInfo);
    throw exc;
  }

  char const * XmiDeserializerHandler::XMI_ID_ATTR_NAME = "xmi:id";
  char const * XmiDeserializerHandler::TRUE_VALUE = "true";
  char const * XmiDeserializerHandler::DEFAULT_CONTENT_FEATURE = "value";
  char const * XmiDeserializerHandler::DEFAULT_NAMESPACE_URI = "http:///uima/noNamespace.ecore";


} // namespace uima



