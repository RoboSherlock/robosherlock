/** @name xcasdeserializer_handler.cpp
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

#include <uima/xcasdeserializer_handler.hpp>
#include <uima/internal_fspromoter.hpp>
#include <uima/internal_typeshortcuts.hpp>
#include <uima/internal_casimpl.hpp>
#include <uima/fsindexrepository.hpp>
#include <uima/arrayfs.hpp>
#include <uima/annotator_context.hpp>
#include <uima/resmgr.hpp>


namespace uima {

// ---------------------------------------------------------------------------
//  XCASDeserialiserHandler: Constructors and Destructor
// ---------------------------------------------------------------------------

  XCASDeserializerHandler::XCASDeserializerHandler(CAS & cas, AnnotatorContext * const ctx) : iv_cas(cas.getBaseCas() ),
      iv_locator(NULL), iv_ctx(ctx),
      iv_casimpl( uima::internal::CASImpl::promoteCAS(*iv_cas)
            //    ,iv_typesystem(iv_casimpl.getHeap().getTypeSystem())
                ) {

    //cout << " XCASDeserializerHandler::constructor " << endl;
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

    // add entry for baseCAS ... point non-compliant annotations at first Sofa
    sofaRefMap.push_back(1);
    // add entry for baseCAS ... _indexed=0 stays in 0
    indexMap.push_back(0);
  }

  XCASDeserializerHandler::~XCASDeserializerHandler()   {
    //cout << " XCASDeserializerHandler::destructor " << endl;
    for (size_t i = 0; i < fsTree.size(); i++) {
      FSInfo * fsinfo = (FSInfo*) fsTree[i];
      if (fsinfo != 0) {
        delete fsinfo->indexRep;
        delete fsinfo;
      }
    }

    for (size_t i = 0; i < idLess.size(); i++) {
      FSInfo * fsinfo = (FSInfo*) idLess[i];
      if (fsinfo != 0) {
        delete fsinfo->indexRep;
        delete fsinfo;
      }
    }

    // free some storage
    fsTree.clear();
    sofaRefMap.clear();
    indexMap.clear();
  }


// ---------------------------------------------------------------------------
//  XCASDeserializerHandler: Implementation of the SAX2 ContentHandler interface
// ---------------------------------------------------------------------------

  void  XCASDeserializerHandler::setDocumentLocator(const Locator* const locator) {
    //cout << " XCASDeserializerHandler::setDocumentLocator() " << endl;
    iv_locator = locator;
  }

  void XCASDeserializerHandler::startDocument() {
    //cout << " XCASDeserializerHandler::startDocument() " << endl;
    iv_state = DOC_STATE;
  }

  void XCASDeserializerHandler::startElement(const   XMLCh* const    uri,
      const   XMLCh* const    localname,
      const   XMLCh* const    qname,
      const Attributes & attrs) {
    //cout << " XCASDeserializerHandler::startElement() " <<icu::UnicodeString((UChar*)qname, XMLString::stringLen(qname)) << endl;
    assert(sizeof(XMLCh) == sizeof(UChar));

    icu::UnicodeString qualifiedName( (UChar const *) qname, XMLString::stringLen(qname));
    buffer.remove();

    switch (iv_state) {
    case DOC_STATE: {
      if (qualifiedName.compare(CASTAGNAME) != 0) {
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
      iv_state = FS_STATE;
      break;
    }
    case FS_STATE: {
      currentContentFeat = DEFAULT_CONTENT_FEATURE;
      if (qualifiedName.compare(DEFAULT_DOC_TYPE_NAME) == 0) {
        iv_state = DOC_TEXT_STATE;
      } else {
        readFS(qualifiedName, attrs);
      }
      break;
    }
    case ARRAY_ELE_STATE: {
      readArrayElement(qualifiedName, attrs);
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
    }
  }

  void XCASDeserializerHandler::characters(
    const   XMLCh* const  cpwsz,
    const unsigned int    uiLength) {
    //cout << "XCASDeserializerHandler::characters: \"" << icu::UnicodeString(cpwsz, uiLength) << "\"" << endl;

    assert(sizeof(XMLCh) == sizeof(UChar));

    switch (this->iv_state)  {
    case DOC_TEXT_STATE:
    case CONTENT_STATE:
    case ARRAY_ELE_CONTENT_STATE:
    case FEAT_CONTENT_STATE:
      buffer.append( (UChar const *) cpwsz, 0, uiLength );
      break;
    default:
      break;
    }

  }

  void XCASDeserializerHandler::endElement(const XMLCh* const uri,
      const XMLCh* const localname,
      const XMLCh* const qname) {
    //cout << " XCASDeserializerHandler::endElement() " << icu::UnicodeString( (UChar*) qname, XMLString::stringLen(qname) ) << endl;
    icu::UnicodeString qualifiedName( (UChar const *) qname, XMLString::stringLen(qname));
    assert(sizeof(XMLCh) == sizeof(UChar));
    switch (iv_state) {
    case DOC_STATE: {
      // Do nothing.
      break;
    }
    case FS_STATE: {
      iv_state = DOC_STATE;
      break;
    }
    case FEAT_STATE: {
      iv_state = FS_STATE;
      break;
    }
    case CONTENT_STATE: {
      // Set the value of the content feature.
      //if (!isAllWhitespace(buffer))
      //{
      handleFeature(currentAddr, currentContentFeat, buffer, true);
      //}            }
      iv_state = FS_STATE;
      break;
    }
    case FEAT_CONTENT_STATE: {
      // Create a feature value from an element.
      handleFeature(currentAddr, qualifiedName, buffer, false);
      iv_state = FEAT_STATE;
      break;
    }
    case ARRAY_ELE_CONTENT_STATE: {
      // Create an array value.
      addArrayElement(buffer);
      iv_state = ARRAY_ELE_STATE;
      break;
    }
    case ARRAY_ELE_STATE: {
      iv_state = FS_STATE;
      break;
    }
    case DOC_TEXT_STATE: {
      // Assume old style TCAS with one text Sofa
      SofaFS newSofa = iv_cas->createInitialSofa(icu::UnicodeString("text"));
      CAS* cas = iv_cas->getInitialView();
      cas->registerView(newSofa);
      // Set the document text without creating a documentAnnotation
      cas->setDocTextFromDeserializtion(UnicodeStringRef(buffer.getBuffer(), buffer.length()));

      // and assume the new Sofa is at location 1!
      int addr = 1;
      int id = 1;
      sofaRefMap.push_back(id);

      // and register the id for this Sofa
      FSInfo * fsInfo = new FSInfo(addr, new vector<int>);
//           FSInfo * fsInfo = new FSInfo(addr, -1); //??? Should be 0 or -1 ???
      fsTree[id] =  fsInfo;

      iv_state = FS_STATE;
      break;
    }
    }

  }


  void XCASDeserializerHandler::endDocument() {

    //cout << " XCASDeserializerHandler::endDocument() " << endl;

    //update features that are FSs
    for (size_t i = 0; i < fsTree.size(); i++) {
      FSInfo * fsinfo = (FSInfo*) fsTree[i];
      if (fsinfo != 0)
        finalizeFS(*fsinfo);
    }
    //update features that are FSs
    for (size_t i = 0; i < idLess.size(); i++) {
      FSInfo * fsinfo = (FSInfo*) idLess[i];
      if (fsinfo != 0)
        finalizeFS(*fsinfo);
    }

    //update document annotation info in tcas
    for (size_t i = 0; i < tcasInstances.size(); i++) {
      CAS * tcas = (CAS *) tcasInstances[i];
      if (tcas != 0) {
        tcas->pickupDocumentAnnotation();
      }
    }

  }


  void XCASDeserializerHandler::ignorableWhitespace(const  XMLCh* const cpwsz,
      const unsigned int length) {
    cout << " XCASDeserializerHandler::ignorableWhitespace() " << endl;

  }



// Create a new FS.
  void XCASDeserializerHandler::readFS(icu::UnicodeString & qualifiedName, const Attributes & attrs) {
    icu::UnicodeString typeName(qualifiedName);
    Type type = iv_cas->getTypeSystem().getType(typeName);
    uima::lowlevel::TyFSType typecode =  uima::internal::FSPromoter::demoteType(type);

    if (!type.isValid() ) {
      cout << "INFO: invalid type " << typeName << endl;
      iv_state = CONTENT_STATE;
    } else {
      if (iv_cas->getTypeSystem().isArrayType(typecode)) {
        readArray(type, attrs);
        return;
      }
      uima::lowlevel::TyFS addr = uima::internal::FSPromoter::demoteFS(iv_cas->createFS(type));
      readFS(addr, attrs, true);
    }
  }

  void XCASDeserializerHandler::readFS(lowlevel::TyFS addr, const Attributes  & attrs, bool toIndex) {
    // Hang on address for setting content feature
    currentAddr = addr;

    int id = -1;
//       int sofaRef = -1; // 0 ==> baseCas indexRepository
    vector<int>* sofaRef = new vector<int>;
    icu::UnicodeString attrName;
    icu::UnicodeString attrValue;
    bool nameMapping = false;
    UChar ubuff[256];
    UErrorCode errorCode = U_ZERO_ERROR;
    lowlevel::TyFS heapValue = iv_casimpl.getHeap().getType(addr);

    // Special handling for Sofas
    if (sofaTypeCode == heapValue) {
      // create some maps to handle v1 format XCAS ...
      // ... where the sofa feature of annotations was an int not a ref

      // determine if this is the one and only initial view Sofa
      bool isInitialView = false;
      int extsz = icu::UnicodeString(CAS::FEATURE_BASE_NAME_SOFAID).extract(ubuff, 256, errorCode);
      if (extsz > 256) {
        cout << "ACK!" << endl;
      }
      const UChar* sofaID = attrs.getValue(ubuff);
      if (0==UnicodeStringRef(sofaID).compare(icu::UnicodeString("_DefaultTextSofaName"))) {
        sofaID = ubuff;
      }
//   no Sofa mapping for now
//   if (iv_ctx != NULL) {
//           // Map incoming SofaIDs
//           sofaID = iv_ctx->mapToSofaID(sofaID).getSofaId();
//         }
      if (0==UnicodeStringRef(sofaID).compare(icu::UnicodeString(CAS::NAME_DEFAULT_SOFA))) {
        isInitialView = true;
      }
      // get the sofaNum
      extsz =icu::UnicodeString(CAS::FEATURE_BASE_NAME_SOFANUM).extract(ubuff, 256, errorCode);
      if (extsz > 256) {
        cout << "ACK!" << endl;
      }
      const UChar* aString = attrs.getValue(ubuff);
      int thisSofaNum = atoi(UnicodeStringRef(aString).asUTF8().c_str());

      // get the sofa's FeatureStructure id
      icu::UnicodeString(ID_ATTR_NAME).extract(ubuff,256, errorCode);
      aString = attrs.getValue(ubuff);
      int sofaFsId = atoi(UnicodeStringRef(aString).asUTF8().c_str());

      // for v1 and v2 formats, create the index map
      // ***we assume Sofas are always received in Sofanum order***
      // Two scenarios ... the initial view is the first sofa, or not.
      // If not, the _indexed values need to be remapped to leave room for the initial view,
      // which may or may not be in the received CAS.
      if (indexMap.size() == 1) {
        if (isInitialView) {
          // the first Sofa an initial view
          if (thisSofaNum == 2) {
            // this sofa was mapped to the initial view
            indexMap.push_back(-1); // for this CAS, there should not be a sofanum = 1
            indexMap.push_back(1); // map 2 to 1
            nextIndex = 2;
          } else {
            indexMap.push_back(1);
            nextIndex = 2;
          }
        } else {
          if (thisSofaNum > 1) {
            // the first Sofa not initial, but sofaNum > 1
            // must be a v2 format, and sofaNum better be 2
            indexMap.push_back(1);
            assert (thisSofaNum == 2);
            indexMap.push_back(2);
            nextIndex = 3;
          } else {
            // must be v1 format
            indexMap.push_back(2);
            nextIndex = 3;
          }
        }
      } else {
        // if the new Sofa is the initial view, always map to 1
        if (isInitialView) {
          // the initial view is not the first
          // if v2 format, space already reserved in mapping
          if (indexMap.size() == thisSofaNum) {
            // v1 format, add mapping for initial view
            indexMap.push_back(1);
          }
        } else {
          indexMap.push_back(nextIndex);
          nextIndex++;
        }
      }

      // Now update the mapping from annotation int to ref values
      if (sofaRefMap.size() == thisSofaNum) {
        // Sofa received in sofaNum order, add new one
        sofaRefMap.push_back(sofaFsId);
      } else if ((int)sofaRefMap.size() > thisSofaNum) {
        // new Sofa has lower sofaNum than last one
        sofaRefMap[thisSofaNum] =  sofaFsId;
      } else {
        // new Sofa has skipped ahead more than 1
        sofaRefMap.resize(thisSofaNum + 1);
        sofaRefMap[thisSofaNum] = sofaFsId;
      }
    }

    Type type = uima::internal::FSPromoter::promoteType(heapValue, iv_cas->getTypeSystem().getLowlevelTypeSystem());

    for (size_t i = 0; i < attrs.getLength(); i++) {
      assertWithMsg( sizeof(XMLCh) == sizeof(UChar), "Port required!");
      attrName = (UChar*)attrs.getQName(i);
      attrValue = (UChar*)attrs.getValue(i);
      if (attrName.startsWith("_")) {
        if (attrName.compare(ID_ATTR_NAME) == 0) {
          id = atoi(UnicodeStringRef(attrValue).asUTF8().c_str());
        } else if (attrName.compare(CONTENT_ATTR_NAME) == 0) {
          currentContentFeat = attrValue;
        } else if (attrName.compare(INDEXED_ATTR_NAME)== 0) {
//             if (toIndex)
//             { // suppress indexing of document annotation if old CAS
//               if (attrValue.compare(TRUE_VALUE) == 0)
//                 sofaRef = 1;
//               else if (!attrValue.compare("false") == 0)
//                 sofaRef = atoi(uniStr2SingleByteStr(attrValue,"UTF-8").c_str());
//             }
          char indexes[256];
          // we have a problem here if number of indexed views is ridiculously big
          strcpy(indexes, UnicodeStringRef(attrValue).asUTF8().c_str());
          char* ptr = strtok (indexes," ");
          while (ptr != NULL) {
            sofaRef->push_back(atoi(ptr));
            ptr = strtok (NULL, " ");
          }
        } else {
          handleFeature(type, addr, attrName, attrValue, false);
        }
      } else {
        if (nameMapping && attrName.compare(CAS::FEATURE_BASE_NAME_SOFAID) == 0) {
          if (iv_ctx != NULL) {
            attrValue = iv_ctx->mapToSofaID(attrValue).getSofaId();
          }
        }
        handleFeature(type, addr, attrName, attrValue, false);
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

    // sofaRef.size()==0 means not indexed
    FSInfo * fsInfo = new FSInfo(addr, sofaRef);
    if (id < 0) {
      idLess.push_back(fsInfo);
    } else {
      fsTree[id] =  fsInfo;
    }
    iv_state = CONTENT_STATE;

  }

  void XCASDeserializerHandler::readArray(Type & type, const Attributes  & attrs) {

    vector<int>* indexRep = new vector<int>;
    int id = -1;
    int size=0;
    icu::UnicodeString attrName;
    icu::UnicodeString attrValue;

    for (size_t i = 0; i < attrs.getLength(); i++) {
      assertWithMsg( sizeof(XMLCh) == sizeof(UChar), "Port required!");
      attrName = (UChar*)attrs.getQName(i);
      attrValue = (UChar*)attrs.getValue(i);

      if (attrName.compare(ID_ATTR_NAME) == 0) {
        id = atoi(UnicodeStringRef(attrValue).asUTF8().c_str());
      } else if (attrName.compare(ARRAY_SIZE_ATTR) == 0) {
        size = atoi(UnicodeStringRef(attrValue).asUTF8().c_str());
      } else if (attrName.compare(INDEXED_ATTR_NAME)== 0) {
//             // suppress indexing of document annotation if old CAS
//               if (attrValue.compare(TRUE_VALUE) == 0)
//                 indexRep = 1;
//               else if (!attrValue.compare("false") == 0)
//                 indexRep = atoi(uniStr2SingleByteStr(attrValue,"UTF-8").c_str());
        char indexes[256];
        // we have a problem here if number of indexed views is ridiculously big
        strcpy(indexes, UnicodeStringRef(attrValue).asUTF8().c_str());
        char* ptr = strtok (indexes," ");
        while (ptr != NULL) {
          indexRep->push_back(atoi(ptr));
          ptr = strtok (NULL, " ");
        }
      } else {
        ErrorInfo errInfo;
        errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
        ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
        assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
        msg.addParam( attrName );
        msg.addParam( attrValue );
        errInfo.setMessage(msg);
        errInfo.setSeverity(ErrorInfo::unrecoverable);
        ExcIllFormedInputError exc(errInfo);
        throw exc;
      }
    }


    arrayType = uima::internal::FSPromoter::demoteType(type);
    currentAddr = iv_casimpl.getHeap().createArrayFS(arrayType, size);

    arrayPos=0;

    // indexRep.size()==0 means not indexed
    FSInfo * fsInfo = new FSInfo(currentAddr, indexRep);
    if (id < 0) {
      idLess.push_back(fsInfo);
    } else {
      fsTree[id] =  fsInfo;
    }
    iv_state = ARRAY_ELE_STATE;

  }

  void XCASDeserializerHandler::readArrayElement(icu::UnicodeString & qualifiedName, const Attributes & attrs) {
    if (qualifiedName.compare(ARRAY_ELEMENT_TAG) != 0) {
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
    if (attrs.getLength() > 0) {
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
    iv_state = ARRAY_ELE_CONTENT_STATE;
  }

  void XCASDeserializerHandler::addArrayElement(icu::UnicodeString & buffer) {

    if (arrayPos >= iv_casimpl.getHeap().getArraySize(currentAddr) ) {
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

    FeatureStructure fs = uima::internal::FSPromoter::promoteFS(currentAddr, *iv_cas);

    switch (arrayType) {
    case internal::gs_tyIntArrayType: {
        int val = atoi(UnicodeStringRef(buffer).asUTF8().c_str());
        IntArrayFS intFS(fs);
        intFS.set( (size_t) arrayPos, val);
        break;
      }
    case internal::gs_tyFloatArrayType: {
        float val = atof(UnicodeStringRef(buffer).asUTF8().c_str());
        FloatArrayFS floatFS(fs);
        floatFS.set( (size_t) arrayPos, val);
        break;
      }
    case internal::gs_tyStringArrayType: {
        //add the striug
        int stringoffset = iv_cas->getHeap()->addString(buffer);
        //set the array value in fs heap
        lowlevel::TyFS  stringref =  iv_cas->getHeap()->getStringAsFS(stringoffset);
        lowlevel::TyHeapCell * fsarray = iv_cas->getHeap()->getCArrayFromFS(currentAddr);
        fsarray[arrayPos] = stringref;
        break;
      }
    case internal::gs_tyByteArrayType: {
        short intval = atoi(UnicodeStringRef(buffer).asUTF8().c_str());
        char charval[2];
        sprintf(charval,"%c",intval);
        ByteArrayFS byteFS(fs);
        byteFS.set( (size_t) arrayPos, charval[0]);
        break;
      }
    case internal::gs_tyBooleanArrayType: {
        string val = UnicodeStringRef(buffer).asUTF8();
        BooleanArrayFS booleanFS(fs);
        if (val.compare("1")==0)  {
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
        string strval;
        UnicodeStringRef(buffer).extractUTF8(strval);
        stringstream s;
        s << strval.c_str();
        s >> val;
        ShortArrayFS shortFS(fs);
        shortFS.set( (size_t) arrayPos, val);
        break;
      }
    case internal::gs_tyLongArrayType: {
        INT64 val;
        stringstream s;
        s << UnicodeStringRef(buffer).asUTF8();
        s >> val;
        LongArrayFS longFS(fs);
        longFS.set( (size_t) arrayPos, val);
        break;
      }
    case internal::gs_tyDoubleArrayType: {
        DoubleArrayFS doubleFS(fs);
        stringstream s;
        s << UnicodeStringRef(buffer).asUTF8();
        long double doubleval;
        s >> doubleval;
        doubleFS.set((size_t) arrayPos, doubleval);
        break;
      }
    default: {    //array of FSs
      lowlevel::TyFS fsid = atoi(UnicodeStringRef(buffer).asUTF8().c_str());
      FeatureStructure fsitem(fsid, *iv_cas);
      ArrayFS fsArrayfs(fs);
      fsArrayfs.set((size_t) arrayPos, fsitem);
    }
    }

    ++arrayPos;
  }



  // Create a feature value from a string representation.
  void XCASDeserializerHandler::handleFeature(lowlevel::TyFS addr, icu::UnicodeString & featName, icu::UnicodeString & featVal, bool lenient) {
    lowlevel::TyFSType fstype = iv_casimpl.getHeap().getType(addr);
    Type type = uima::internal::FSPromoter::promoteType(fstype, iv_cas->getTypeSystem().getLowlevelTypeSystem());
    handleFeature(type, addr, featName, featVal, lenient);
  }

  void XCASDeserializerHandler::handleFeature(Type & type, lowlevel::TyFS addr, icu::UnicodeString & featName, icu::UnicodeString & featVal,
      bool lenient) {
    char charFeatVal[10];

    // handle v1.x format annotations, mapping int to ref values
    lowlevel::TyFSType fstype = iv_casimpl.getHeap().getType(addr);
    if (0==featName.compare("sofa") &&
        iv_typesystem->subsumes(internal::gs_tyAnnotationBaseType, fstype)) {
      int ifeatval = atoi(UnicodeStringRef(featVal).asUTF8().c_str());
      sprintf(charFeatVal, "%d", sofaRefMap[ifeatval]);
      featVal.setTo(icu::UnicodeString(charFeatVal));
    }

    // handle v1.x sofanum values, remapping so that _InitialView always == 1
    if (0==featName.compare(CAS::FEATURE_BASE_NAME_SOFAID)
        && sofaTypeCode == fstype) {
      int sofaNum = iv_casimpl.getHeap().getIntValue(addr, internal::gs_tySofaNumFeature);
      iv_casimpl.getHeap().setIntValue(addr, internal::gs_tySofaNumFeature, indexMap[sofaNum]);
    }

    icu::UnicodeString prefix(REF_PREFIX);
    if (featName.startsWith(REF_PREFIX)) {
      featName.remove(0,prefix.length());             // Delete prefix
    }
    FeatureStructure fs = uima::internal::FSPromoter::promoteFS(addr, *iv_cas);
    Feature feat = type.getFeatureByBaseName(featName);
    //    System.out.println("DEBUG - Feature map result: " + featName + " = " + feat.getName());
    if (!feat.isValid()) { //feature does not exist in typesystem;
      //Out of typesystem data not supported.
      //we skip this feature
      /**ErrorInfo errInfo;
      errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
      ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
      assertWithMsg(sizeof(XMLCh) == sizeof(UChar), "Port required");
      msg.addParam(type.getName());
      msg.addParam(featName);
      errInfo.setMessage(msg);
      errInfo.setSeverity(ErrorInfo::unrecoverable);
      ExcIllFormedInputError exc(errInfo);
      throw exc; **/
    } else  {
      Type rtype;
      feat.getRangeType(rtype);
      lowlevel::TyFSType rangeType = uima::internal::FSPromoter::demoteType(rtype);
      switch (rangeType) {
      case internal::gs_tyIntegerType: {
          if (featVal.length()>0) {
            fs.setIntValue(feat, atoi(UnicodeStringRef(featVal).asUTF8().c_str()));
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
          if (featVal.length() > 0) {
            fs.setStringValue(feat, featVal);
          }
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
            if (val.compare("1")==0)
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
      default: {
        if (rtype.isStringSubType()) {
          if (featVal.length() > 0)
            fs.setStringValue(feat, featVal);
        } else if (featVal.length() > 0) {
          lowlevel::TyFS val = (lowlevel::TyFS) atoi(UnicodeStringRef(featVal).asUTF8().c_str());
          iv_casimpl.getHeap().setFeatureInternal(addr, uima::internal::FSPromoter::demoteFeature(feat), val);
        }
        break;
      }
      }
    }
  }

  void XCASDeserializerHandler::finalizeFS(FSInfo & fsInfo) {
    lowlevel::TyFS addr = fsInfo.addr;
    FeatureStructure fs = uima::internal::FSPromoter::promoteFS(addr, *iv_cas);
    Type type = fs.getType();

    if (fsInfo.indexRep->size() >= 0) {
      // Now add FS to all specified index repositories
      for (int i = 0; i < (int)fsInfo.indexRep->size(); i++) {
        lowlevel::IndexRepository *  pIndexRep;
        if (indexMap.size() == 1) {
          pIndexRep = indexRepositories[fsInfo.indexRep->at(i)];
        } else {
          pIndexRep = indexRepositories[indexMap[fsInfo.indexRep->at(i)]];
        }
        assert(EXISTS(pIndexRep));
        pIndexRep->add(addr);
      }
    }


    if (iv_cas->getTypeSystem().isArrayType(uima::internal::FSPromoter::demoteType(type)) ) {
      finalizeArray(type, addr, fsInfo);
      return;
    }


    //update heap value of features that are references to other FS.
    vector<Feature> feats;
    type.getAppropriateFeatures(feats);

    FSInfo * fsValInfo;
    for (size_t i = 0; i < feats.size(); i++) {
      Feature feat = (Feature) feats[i];
      Type rangeType;
      feat.getRangeType(rangeType);

      if (rangeType.isValid()) {
        lowlevel::TyFSType  rangetypecode = uima::internal::FSPromoter::demoteType(rangeType);
        lowlevel::TyFSFeature featcode = uima::internal::FSPromoter::demoteFeature(feat);

        //if not primitive
        if (!iv_cas->getTypeSystem().isPrimitive(rangetypecode)) {
          //get the current feature value which is the id
          lowlevel::TyFS featVal = iv_casimpl.getHeap().getFeatureInternal(addr, featcode);
          //get the FSInfo object for that id
          fsValInfo = (FSInfo*) fsTree[featVal];
          //if there is a FSInfo
          //set the feature value of this feature to the
          //address in FSInfo else set it to NULL;
          if (fsValInfo == NULL) {
            //nothing to do, reference value already = 0!
            //iv_casimpl.getHeap().setFSValue(addr, featcode, (lowlevel::TyFS) 0);
          } else {
            iv_casimpl.getHeap().setFSValue(addr, featcode, fsValInfo->addr);
          }
        }
      }
    }
  }


  void XCASDeserializerHandler::finalizeArray(Type & type, lowlevel::TyFS addr, FSInfo & fsInfo) {

    lowlevel::TyFSType typecode = uima::internal::FSPromoter::demoteType(type);
    if (!iv_cas->getTypeSystem().isFSArrayType(typecode)) {
      return;
    }

    // *** WARNING ***  *** WARNING ***  *** WARNING ***  *** WARNING ***
    // if implementation of ArrayFS on the heap changes, this code will be invalid
    int size = (int)iv_cas->getHeap()->getHeap().getHeapValue(addr + 1);
    FSInfo * fsValInfo;
    for (int i=0; i<size; i++) {
      lowlevel::TyFS id = iv_cas->getHeap()->getHeap().getHeapValue(addr + 2 + i);
      fsValInfo = fsTree[id];
      if (fsValInfo != NULL) {
        iv_cas->getHeap()->getHeap().setHeapValue(addr + 2 + i, fsValInfo->addr);
      }
    }

  }


// ---------------------------------------------------------------------------
//  XCASDeserializerHandler: Overrides of the SAX ErrorHandler interface
// ---------------------------------------------------------------------------
  void XCASDeserializerHandler::error(const SAXParseException& e) {
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

  void XCASDeserializerHandler::fatalError(const SAXParseException& e) {
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

  void XCASDeserializerHandler::warning(const SAXParseException& e) {
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

  char const * XCASDeserializerHandler::CASTAGNAME = "CAS";
  char const * XCASDeserializerHandler::DEFAULT_DOC_TYPE_NAME = "uima.tcas.Document";
  char const * XCASDeserializerHandler::DEFAULT_DOC_TEXT_FEAT = "text";
  char const * XCASDeserializerHandler::INDEXED_ATTR_NAME = "_indexed";
  char const * XCASDeserializerHandler::REF_PREFIX = "_ref_";
  char const * XCASDeserializerHandler::ID_ATTR_NAME = "_id";
  char const * XCASDeserializerHandler::CONTENT_ATTR_NAME = "_content";
  char const * XCASDeserializerHandler::ARRAY_SIZE_ATTR = "size";
  char const * XCASDeserializerHandler::ARRAY_ELEMENT_TAG = "i";
  char const * XCASDeserializerHandler::TRUE_VALUE = "true";
  char const * XCASDeserializerHandler::DEFAULT_CONTENT_FEATURE = "value";



} // namespace uima


