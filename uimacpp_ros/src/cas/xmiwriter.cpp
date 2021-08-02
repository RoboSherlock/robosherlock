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

#include <uima/xmiwriter.hpp>
#include <uima/arrayfs.hpp>
#include <uima/listfs.hpp>
#include <uima/lowlevel_indexrepository.hpp>
#include <uima/lowlevel_indexiterator.hpp>
#include <uima/internal_casimpl.hpp>
#include <uima/internal_fspromoter.hpp>
#include <uima/fsindexrepository.hpp>

#include <uima/resmgr.hpp>
#include <uima/location.hpp>
#include <sstream>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */
//#define TAB_INCREMENT 0

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

using namespace std;
namespace uima {

  ////////////////////////////////////////////////////////////////////////
  // XmiWriter

  XmiWriter::XmiWriter(CAS const & cas, bool addDocument)
      : XMLWriterABase(cas, addDocument),  sharedData(0),
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

  XmiWriter::XmiWriter(CAS const & cas, bool addDocument, XmiSerializationSharedData * serdata)
      : XMLWriterABase(cas, addDocument),  sharedData(serdata),
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

  XmiWriter::~XmiWriter() {
    for (size_t i = 0; i < enqueuedFS.size(); i++) {
      vector<int> * indexes = (vector<int>*) enqueuedFS[i];
      if (indexes != 0)
        delete indexes;
    }
    for (size_t i = 0; i < xmiTypeNames.size(); i++) {
      if (xmiTypeNames.at(i) != 0) {
        delete xmiTypeNames.at(i);
      }
    }
  }

 /**
  * Populates nsUriToPrefixMap and xmiTypeNames structures based on CAS type system.
  */
 void XmiWriter::initTypeAndNamespaceMappings() {
   nsUriToPrefixMap[XMI_NS_URI] = XMI_NS_PREFIX;

   //Add any namespace prefix mappings used by out of type system data.
   //Need to do this before the in-typesystem namespaces so that the prefix
   //used here are reserved and won't be reused for any in-typesystem namespaces.
   if (this->sharedData !=  NULL) {
     vector<OotsElementData*> & ootsElements = this->sharedData->getOutOfTypeSystemElements();
     for(size_t i=0;i<ootsElements.size();i++) {
       OotsElementData * oed = ootsElements.at(i);
       string nsUri = oed->elementName->nsUri;
       string qname = oed->elementName->qualifiedName;
       string localName = oed->elementName->shortName;
       string prefix = qname.substr(0, oed->elementName->qualifiedName.find_first_of(":"));
       nsUriToPrefixMap[nsUri] = prefix;
       nsPrefixesUsed.insert(prefix);
     }
   }

   vector<Type>  allTypes;
   iv_cas.getTypeSystem().getAllTypes(allTypes);
   xmiTypeNames.resize(allTypes.size() + 1);
   //leave out 0th element.
   XmlElementName * invalidType = new XmlElementName("INVALID_TYPE",
     "INVALID_TYPE",
     "INVALID_TYPE");

   xmiTypeNames[0] = invalidType;

   for (size_t i = 0; i < allTypes.size() ; i++) {
     Type type = allTypes.at(i);
     lowlevel::TyFSType typecode = uima::internal::FSPromoter::demoteType(type);
     //cout << "Mapping " << type.getName() << endl;
     UnicodeStringRef name = type.getName();
     xmiTypeNames[typecode] = uimaTypeName2XmiElementName(name);
     // this also populats the nsUriToPrefix map
   }
 }

 /**
 * Converts a UIMA-style dotted type name to the element name that should be used in the XMI
 * serialization. The XMI element name consists of three parts - the Namespace URI, the Local
 * Name, and the QName (qualified name).
 *
 * @param uimaTypeName
 *          a UIMA-style dotted type name
 * @return a data structure holding the three components of the XML element name
 */
 XmlElementName * XmiWriter::uimaTypeName2XmiElementName(UnicodeStringRef & uimaTypeName) {
   // split uima type name into namespace and short name
   string nameSpace;
   string shortName;
   string nsUri;
   string qName;
   string typeName = uimaTypeName.asUTF8();
   int lastDotIndex = typeName.find_last_of(".");

   if (lastDotIndex == -1) // no namespace
   {
     shortName = typeName;
     nsUri = DEFAULT_NAMESPACE_URI;
   } else {
     nameSpace = typeName.substr(0, lastDotIndex);
     shortName = typeName.substr(lastDotIndex + 1);
     nsUri = "http:///";
     nsUri.append(nameSpace);
     std::replace(nsUri.begin(), nsUri.end(), '.', '/');
     nsUri.append(".ecore");
     //cout << nsUri << endl;
   }

   // determine what namespace prefix to use
   map<string, string>::iterator ite = nsUriToPrefixMap.find(nsUri);
   string prefix;
   if  (ite != nsUriToPrefixMap.end()) {
     prefix =  ite->second;
   }

   //create a prefix and associate nameSpace with prefix
   if (prefix.length() == 0) {
     if (nameSpace.length() != 0) {
       int secondLastDotIndex = nameSpace.find_last_of(".");
       //cout << nameSpace << " secondlastdot " << secondLastDotIndex << endl;
       prefix = nameSpace.substr(secondLastDotIndex + 1);
     } else {
       prefix = "noNamespace";
     }
     //cout << "prefix " << prefix << endl;
     // make sure this prefix hasn't already been used for some other namespace
     set<string>::iterator prefixIte = nsPrefixesUsed.find(prefix);
     int num=1;
     while (prefixIte != nsPrefixesUsed.end()) {
       num++;
       stringstream basePrefix;
       basePrefix << prefix << num << endl;
       prefixIte = nsPrefixesUsed.find(basePrefix.str());
       if (prefixIte == nsPrefixesUsed.end() ) {
         prefix = basePrefix.str();
         break;
       }
     }
     nsPrefixesUsed.insert(prefix);
     nsUriToPrefixMap[nsUri] = prefix;
   }
   qName = prefix;
   qName.append(":");
   qName.append(shortName);
   return new XmlElementName(nsUri, shortName, qName);

 }

 void XmiWriter::writeViews(ostream & os, CAS const & cas)  {
   // Get indexes for each SofaFS in the CAS
   int numViews = cas.iv_baseCas->iv_sofaCount; 
   //const CAS * baseCas = cas.getBaseCas();
   int sofaXmiId=0;
   lowlevel::TyFS  sofaAddr;

   for (int sofaNum = 1; sofaNum <= numViews; sofaNum++) {
     vector<lowlevel::TyFS> indexedFS;
     SofaFS sofa = cas.iv_baseCas->getSofa(sofaNum); 

     lowlevel::IndexRepository * loopIR = cas.iv_baseCas->iv_sofa2indexMap[sofaNum];

     if (sofaNum != 1 || cas.iv_baseCas->isInitialSofaCreated()) {
       sofaAddr = internal::FSPromoter::demoteFS(sofa);
       sofaXmiId = this->getXmiId(sofaAddr);
     }
     if (loopIR != NULL) { 
       loopIR->getIndexedFSs(indexedFS);
       writeView(os, sofaXmiId, indexedFS);
     }
   }
 }

 void XmiWriter::writeView(ostream & os,int sofaXmiId, vector<lowlevel::TyFS>& members) {
   icu::UnicodeString viewType(icu::UnicodeString("uima.cas.View"));
   UnicodeStringRef uref(viewType.getBuffer(), viewType.length());
   XmlElementName * elemName = uimaTypeName2XmiElementName(uref);

   stringstream membersString;
   for (size_t i = 0; i < members.size(); i++) {
     membersString << getXmiId(members.at(i)) << " ";
   }

   //check for out-of-typesystem members
   if (this->sharedData != NULL) {
     vector<int> ootsMembers;
     this->sharedData->getOutOfTypeSystemViewMembers(sofaXmiId, ootsMembers);
     if (ootsMembers.size() != 0) {
       for (size_t i=0; i < ootsMembers.size(); i++) {
         membersString << ootsMembers.at(i) << " ";
       }
     }
   }

   //remove leading and trailing blanks
   string outstr = membersString.str();
   size_t startpos = outstr.find_first_not_of(" ");
   size_t endpos = outstr.find_last_not_of(" ");
   if (string::npos != startpos && string::npos != endpos) {
     outstr = outstr.substr(startpos, endpos-startpos+1);
   } 

   if (sofaXmiId !=0 || outstr.size() > 0) {
     os << "<" << elemName->qualifiedName;
     if (sofaXmiId != 0) {
       os << " sofa=\"" << sofaXmiId << "\"";
     }
     if (outstr.size() > 0) {
       os << " members=\"" << outstr << "\"";
     }
     os << "/>";
   }   
     delete elemName;
 }

  bool XmiWriter::isReferenceType(Type const & t) const {
    return !( t.getTypeSystem().isPrimitive(uima::internal::FSPromoter::demoteType(t)) );
  }

  void XmiWriter::writeFeatureValue(ostream & os, FeatureStructure const & fs, Feature const & f) {
    assert( fs.isValid() );
    assert( f.isValid() );
    Type t;
    f.getRangeType(t); 
    assert( t.isValid() );
    if ( t == iv_stringType || t.isStringSubType() ) {
      if (!fs.isUntouchedFSValue(f) ) {
      UnicodeStringRef ref = fs.getStringValue(f);
      if (ref.getBuffer() != NULL) {
        icu::UnicodeString us;
        normalize( ref, us );
        os << " " << f.getName() << "=\"";
        os << us << "\"";
      }
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
      if (fs.getBooleanValue(f))
        os << "true" << "\"";
      else 
        os << "false" << "\"";
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
        os << " " ;
        os << f.getName() << "=\"";
        //ptrdiff_t val = uima::internal::CASImpl::promoteCAS(iv_cas).getHeap().getUniqueID(lolFS);
        int val = getXmiId(lolFS);
        os << val << "\"";
      }
    }
  }

  template<class Array>
  void writeArray(ostream & os, Array const & array, char const * tag, int xmiid) {
    size_t i;
    if (array.size() > 0) {
      //XCAS os << " size=\"" << array.size() << "\">" << endl;
      os << " <" << tag;
            os << " " << XmiWriter::ID_ATTR_NAME << "=\"" << xmiid << "\"";
      os << " elements=\"" ;
      os << arrayToString(array, tag);    
      os << "\"/>" << endl;
    } else {
      //XCAS os << " size=\"0\"/>" << endl;
    }
  }

  void XmiWriter::writeArray(ostream & os, 
                            FeatureStructure const & array, 
                            char const * tag, int xmiid) { 
      os << " <" << tag;
      os << " " << ID_ATTR_NAME << "=\"" << xmiid << "\"";
      os << " elements=\"" ;
      os << arrayToString(array, tag);
      os << "\"/>" << endl;
  }

 string XmiWriter::arrayToString(FeatureStructure const & fs, char const * tag) {  
   stringstream str;

   int typecode = internal::FSPromoter::demoteType(fs.getType());
   switch (typecode) {
      case internal::gs_tyIntArrayType: {
        IntArrayFS arrayfs(fs);
        size_t n = arrayfs.size();
        for (size_t i=0; i < n;i++) {
          str << arrayfs.get(i);
          if (i+1 < n) {         
            str << " ";
          }
        }         
        break;
                                        }
      case internal::gs_tyFloatArrayType: {
        FloatArrayFS arrayfs(fs);
        size_t n = arrayfs.size();
        for (size_t i=0; i < n;i++) {
          str << arrayfs.get(i);
          if (i+1 < n) {         
            str << " ";
          }
        }         
        break;
                                          }
      case internal::gs_tyBooleanArrayType: {
        BooleanArrayFS arrayfs(fs);
        size_t n = arrayfs.size();
        for (size_t i=0; i < n;i++) {
          if (arrayfs.get(i)) {
            str << "true";
          } else {
            str << "false";
          }
          if (i+1 < n) {         
            str << " ";
          }
        }         
        break;
                                            }
      case internal::gs_tyByteArrayType: {
        ByteArrayFS arrayfs(fs);
        size_t n = arrayfs.size();
                char * out = new char[3];
                memset(out,0,3);
        for (size_t i=0; i < n;i++) {      
          sprintf(out,"%02X",0xFF & arrayfs.get(i)); 
          //printf ("itoahexadecimal: %d %d\n",i, arrayfs.get(i));
          str << out[0] << out[1];            
        }         
            delete[] out;
        break;
                                         }
      case internal::gs_tyShortArrayType: {
        ShortArrayFS arrayfs(fs);
        size_t n = arrayfs.size();
        for (size_t i=0; i < n;i++) {
          str << arrayfs.get(i);
          if (i+1 < n) {         
            str << " ";
          }
        }         
        break;
                                          }
      case internal::gs_tyLongArrayType: {
        LongArrayFS arrayfs(fs);
        size_t n = arrayfs.size();
        for (size_t i=0; i < n;i++) {
          str << arrayfs.get(i);
          if (i+1 < n) {         
            str << " ";
          }
        }         
        break;
                                         }
      case internal::gs_tyDoubleArrayType: {
        DoubleArrayFS arrayfs(fs);
        size_t n = arrayfs.size();
        for (size_t i=0; i < n;i++) {
          str << arrayfs.get(i);
          if (i+1 < n) {         
            str << " ";
          }
        }         
        break;
                                           }
      case internal::gs_tyFSArrayType: {
        ArrayFS arrayfs(fs);
        size_t size = arrayfs.size();

        vector<XmiArrayElement*> * ootsList = NULL;
        if (this->sharedData != NULL) {
          ootsList = this->sharedData->getOutOfTypeSystemArrayElements(internal::FSPromoter::demoteFS(arrayfs));
        }
        size_t ootsIndex = 0;
        for (size_t j=0; j < size; j++) {
          int xmiId = getXmiId(uima::internal::FSPromoter::demoteFS(arrayfs.get(j)));//convert to xmiid; 0 if not set
          if (xmiId == 0) {  //if 0 check if its an Oots FS 
            // However, this null array element might have been a reference to an 
            //out-of-typesystem FS, so check the ootsArrayElementsList
            if (ootsList != NULL) {
              while (ootsIndex < ootsList->size()) {
                XmiArrayElement * arel = ootsList->at(ootsIndex++);
                if (arel->index == j) {
                  str << arel->xmiId;
                  break;
                }                
              }
            }
          } else {
            str << xmiId;
          }
          if (j+1 < size) {         
            str << " ";
          }
        }         
        break;
                                       }
      case internal::gs_tyStringArrayType: {
        StringArrayFS arrayfs(fs);
        size_t n = arrayfs.size();
         icu::UnicodeString ustr;       
        for (size_t i=0; i < n;i++) {
          ustr.setTo("");
          normalize( arrayfs.get(i), ustr );
          str << "<" << tag << ">" << ustr << "</" << tag << ">";
         
        }         
        break;
                                           }
      default: {
        cerr << "arrayToString() type not supported " << typecode << endl;
        ErrorInfo errInfo;
        errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
        ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
        msg.addParam("arrayToString() type not supported.");
        msg.addParam(typecode);
        errInfo.setMessage(msg);
        errInfo.setSeverity(ErrorInfo::unrecoverable);
        ExcIllFormedInputError exc(errInfo);
        throw exc; 
      }
    }
    return str.str();
  }

  void XmiWriter::writeBooleanArray(ostream & os, BooleanArrayFS const & array, char const * tag, int xmiid) {
    
    if (array.size() > 0) {
      os << " <" << tag;
      os << " " << ID_ATTR_NAME << "=\"" << xmiid << "\"";
      os << " elements=\"" ;
        os << arrayToString(array, tag).c_str();
        os << "\"/>" << endl;
    } 
  }

  void XmiWriter::writeStringArray(ostream & os, StringArrayFS const & array, char const * tag, int xmiid) {
    if (array.size() > 0) {
      //XCAS os << " size=\"" << array.size() << "\">" << endl;
      os << " <" << tag;
      os << " " << ID_ATTR_NAME << "=\"" << xmiid << "\">";
        os << arrayToString(array, "elements").c_str();
        os << "</" << tag << ">" << endl;
    } 
  }

  void XmiWriter::writeFSFlat(ostream & os,
                               FeatureStructure const & fs,
                               vector<int>* indexInfo) {
    uima::internal::CASImpl const & crCASImpl = uima::internal::CASImpl::promoteCAS(iv_cas);
    assert( fs.isValid() );
    Type t = fs.getType();
    lowlevel::TyFSType  typecode = uima::internal::FSPromoter::demoteType(t);
    XmlElementName * xmlElementName = xmiTypeNames[typecode];
    bool insidelist = fs.getCAS().getTypeSystem().isListType(typecode);
    int xmiId = /**crCASImpl.getHeap().getUniqueID**/
      getXmiId( uima::internal::FSPromoter::demoteFS(fs) );
   
    // if array
    if ( iv_arrayType.subsumes(t) ) {
      const CAS* ccasp = &iv_cas;
      CAS* casp = const_cast<CAS*> (ccasp);
        if (casp->getHeap()->getArraySize(internal::FSPromoter::demoteFS(fs)) >0) {
            if ( t == iv_intArrayType ) {
                writeArray( os, IntArrayFS(fs), xmlElementName->qualifiedName.c_str(), xmiId);
            } else if ( t == iv_floatArrayType ) {
                writeArray( os, FloatArrayFS(fs), xmlElementName->qualifiedName.c_str(), xmiId);
            } else if ( t == iv_stringArrayType ) {
                //cout << "got a string array " << endl;
                writeStringArray( os, StringArrayFS(fs), xmlElementName->qualifiedName.c_str(), xmiId);
            } else if ( t == iv_byteArrayType ) {
                writeArray(os, ByteArrayFS(fs), xmlElementName->qualifiedName.c_str(), xmiId);
            } else if ( t == iv_booleanArrayType ) {
                writeArray( os, BooleanArrayFS(fs), xmlElementName->qualifiedName.c_str(), xmiId);
            } else if ( t == iv_shortArrayType ) {
                writeArray( os, ShortArrayFS(fs),xmlElementName->qualifiedName.c_str(), xmiId );
            } else if ( t == iv_longArrayType ) {
                writeArray( os, LongArrayFS(fs), xmlElementName->qualifiedName.c_str(), xmiId);
            } else if ( t == iv_doubleArrayType ) {
                writeArray( os, DoubleArrayFS(fs), xmlElementName->qualifiedName.c_str(), xmiId);
            } else {
                assert( t == iv_cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_FS_ARRAY) );
                if (internal::FSPromoter::demoteType(t) == internal::gs_tyFSArrayType) {
                    writeArray( os, ArrayFS(fs), xmlElementName->qualifiedName.c_str(), xmiId);
                } else {
                    cerr << "writeFSFlat unknown array type " << t.getName() << endl;
                }
            }
        }
    } else {
      vector<Feature> features;
      fs.getType().getAppropriateFeatures(features);
      size_t i;
        stringstream strcontent;
        os << " <" << xmlElementName->qualifiedName;
        os << " " << ID_ATTR_NAME << "=\"" << getXmiId( uima::internal::FSPromoter::demoteFS(fs) ) << "\"";
      bool containsElements=false;
        for (i=0; i<features.size(); ++i) {
        uima::Feature const & f = features[i];
        Type range;
        f.getRangeType(range); 
            int typecode = internal::FSPromoter::demoteType(range);
            //cout << "writeFSFlat() " << range.getName() << endl;
        switch (typecode) {
        case internal::gs_tyBooleanType:
        case internal::gs_tyByteType:
        case internal::gs_tyIntegerType:
        case internal::gs_tyFloatType:
        case internal::gs_tyStringType:
        case internal::gs_tyShortType:
        case internal::gs_tyLongType:
        case internal::gs_tyDoubleType: {
          writeFeatureValue(os, fs, f);
          break;
        }
        case internal::gs_tyStringArrayType: {
          if (f.isMultipleReferencesAllowed() ) {
            writeFeatureValue(os, fs, f);
          } else {
            StringArrayFS arrayFS = fs.getStringArrayFSValue(f);
            if (arrayFS.isValid() && arrayFS.size() > 0) {
              strcontent << arrayToString(arrayFS, f.getName().asUTF8().c_str());
            }
          }
          break;                             
        }
        case internal::gs_tyIntArrayType:
        case internal::gs_tyFloatArrayType:
        case internal::gs_tyByteArrayType:
        case internal::gs_tyBooleanArrayType:
        case internal::gs_tyShortArrayType:
        case internal::gs_tyLongArrayType:
        case internal::gs_tyDoubleArrayType:
        case internal::gs_tyFSArrayType:   {
          if (f.isMultipleReferencesAllowed() ) {
            writeFeatureValue(os, fs, f);
          } else {
            if (fs.getFSValue(f).isValid()) {
              string str = arrayToString(fs.getFSValue(f), f.getName().asUTF8().c_str());          
              os << " " << f.getName() << "=\"" << str << "\" ";
            }
          }
          break;                               
        }
        case internal::gs_tyIntListType:
        case internal::gs_tyEIntListType:
        case internal::gs_tyNEIntListType:
        case internal::gs_tyFloatListType:
        case internal::gs_tyEFloatListType:
        case internal::gs_tyNEFloatListType:
        case internal::gs_tyFSListType:  {
          if (f.isMultipleReferencesAllowed() || insidelist) {
            writeFeatureValue(os, fs, f);
          } else {
            if (fs.getFSValue(f).isValid()) {
              string str = listToString(fs.getFSValue(f), f.getName().asUTF8().c_str());                  
              os << " " << f.getName() << "=\"" << str << "\" ";  
              //cout << "   " << range.getName() << str << endl;
            }
          }
          break;
        }
        case internal::gs_tyStringListType:
        case internal::gs_tyEStringListType:
        case internal::gs_tyNEStringListType: {
          FeatureStructure listFS = fs.getFSValue(f);
          if (listFS.isValid()) {
            if (f.isMultipleReferencesAllowed() ) {
              writeFeatureValue(os, fs, f);
            } else {
              if (fs.getFSValue(f).isValid()) {
                string str = listToString(fs.getFSValue(f), f.getName().asUTF8().c_str());          
                if (listFS.isValid() && str.length() > 0) {
                  strcontent << str;
                }
              }
            }
          }
          break;
        }
        default: { 
          writeFeatureValue(os, fs, f);
          break;
        }
     }      
    }
      
  //add out-of-typesystem features, if any
  if (this->sharedData != NULL) {
    int fsaddr = internal::FSPromoter::demoteFS(fs);
    OotsElementData * oed = this->sharedData->getOutOfTypeSystemFeatures(fsaddr);
    if (oed != NULL) {
      //attributes
      for (size_t a=0; a < oed->attributes.size(); a++) {
        XmlAttribute * attr = oed->attributes.at(a);
        icu::UnicodeString us;
        icu::UnicodeString av(attr->value.c_str());
        normalize( av, us );
        os << " " << attr->name << "=\"" << us << "\"";
      }
      //child elements
      map<string,vector<string>*>::iterator ite ;
      for (ite = oed->childElements.begin();
        ite != oed->childElements.end(); ite++) {
          vector<string> * values = ite->second;
          for (size_t v=0; v < values->size();v++) {
            icu::UnicodeString us;
            icu::UnicodeString av(values->at(v).c_str());
            normalize( av, us );
            strcontent << " <" << ite->first 
              << ">" << us
              << "</" << ite->first << ">";
          }
        }
      }
    }
    // write stringArray and/or stringList Features if any
    if (strcontent.str().length() > 0) {
      os << ">" << strcontent.str();
      os << "</" << xmlElementName->qualifiedName << ">" << endl;
      strcontent.clear();
    } else {
      os << "/>" << endl;
    } 
  } 
}


void XmiWriter::findReferencedFSs(FeatureStructure const & fs, bool check) {
    

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
    //cout << "findReferencedFSs() " << t.getName() << endl;
    int tcode = internal::FSPromoter::demoteType(t);
    bool insideListNode=fs.getCAS().getTypeSystem().isListType(tcode);
    assert( t.isValid() );
    size_t i;
    if ( iv_arrayType.subsumes(t) ) {
      if ( t == iv_cas.getTypeSystem().getType(uima::CAS::TYPE_NAME_FS_ARRAY) ) {
        ArrayFS array(fs);
        for (i=0; i<array.size(); ++i) {
          findReferencedFSs(array.get(i));
        }
      }
    } 
    else {
      vector<Feature> features;
      fs.getType().getAppropriateFeatures(features);

      for (i=0; i<features.size(); ++i) {
        uima::Feature const & f = features[i];
        Type range;
        f.getRangeType(range); 
        if (isReferenceType(range)) {
          //cout << __LINE__ << "a referenced feature " <<
                //  f.getName() << endl;
                int typecode = internal::FSPromoter::demoteType(range);
                switch (typecode) {
                    case internal::gs_tyFSArrayType:  {
                      findReferencedFSs(fs.getFSValue(f));
                      break;
                  }
          case internal::gs_tyFSListType:  {
            // we only enqueue lists as first-class objects if the feature has
            // multipleReferencesAllowed = true
            // OR if we're already inside a list node (this handles the tail feature correctly)
            if (f.isMultipleReferencesAllowed() || insideListNode) {
              findReferencedFSs(fs.getFSValue(f));
            } else  {
              // enqueue any FSs reachable from an FSList
              enqueueFSListElements(fs.getFSValue(f));
            }
            break;
          }
                  case internal::gs_tyBooleanArrayType:
                  case internal::gs_tyByteArrayType:
                  case internal::gs_tyIntArrayType:
                  case internal::gs_tyFloatArrayType:
                  case internal::gs_tyStringArrayType:
                  case internal::gs_tyShortArrayType:
                  case internal::gs_tyLongArrayType:
                  case internal::gs_tyDoubleArrayType: {
                      if (f.isMultipleReferencesAllowed ()) {
                          findReferencedFSs(fs.getFSValue(f));
                    }
                      break;
                  }
                  case internal::gs_tyIntListType:
                  case internal::gs_tyFloatListType:
                  case internal::gs_tyStringListType:   {
                    if (f.isMultipleReferencesAllowed () || insideListNode) {
                          findReferencedFSs(fs.getFSValue(f));
                    }
                      break;
                  }
                  default: { //FS Ref
                      findReferencedFSs(fs.getFSValue(f));
                  }
            }
      }
    }
  }
}

void XmiWriter::write(ostream & os) {

  //build xmlTypeNames and nsUri to prefix mapping
  initTypeAndNamespaceMappings();
  //enqueue every FS that was deserialized into this CAS
  enqueueIncoming();
  const uima::lowlevel::IndexRepository * ixRep;
  const CAS* ccasp = &iv_cas;
  CAS* casp = const_cast<CAS*> (ccasp);
  CAS* tcas;
  uima::internal::CASImpl const & crCASImpl = uima::internal::CASImpl::promoteCAS(*casp->getBaseCas());
  ixRep = &crCASImpl.getIndexRepository();

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
    vector<FeatureStructure> indexedFSs;
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

  os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
  /////XCAS os << "<CAS>" << endl;
  os  << "<xmi:XMI " ;
  //write nsURI and prefix
  map<string, string>::iterator ite;
  for (ite=nsUriToPrefixMap.begin(); ite !=  nsUriToPrefixMap.end(); ite++) {
    os << "xmlns:" << ite->second << "=\"" << ite->first << "\" " ;
  }
  os << " xmi:version=\"2.0\">";
  // TODO: add schemaLocation if specified

  //Write a special instance of dummy type uima.cas.NULL, having xmi:id=0.
  //This is needed to represent nulls in multi-valued references, which 
  //aren't natively supported in Ecore.
  //?? XmlElementName * nullElement = uimaTypeName2XmiElementName("uima.cas.NULL"); //?
  os << "<cas:NULL xmi:id=\"0\"/>";

  // write out all enqueued FS
  map<int, vector<int>*>::iterator it;
  for (it = enqueuedFS.begin(); it != enqueuedFS.end(); it++) {
    FeatureStructure fs = internal::FSPromoter::promoteFS((*it).first, iv_cas);
    writeFSFlat(os, fs, (*it).second);
  }
  serializeOutOfTypeSystemElements(os); //encodes sharedData.getOutOfTypeSystemElements().size() elements
  writeViews(os,iv_cas);
  os << "</xmi:XMI>" << endl;
}

  // return true if the fs was not previously enqueued
  bool XmiWriter::enqueueUnindexed(FeatureStructure const &fs) {
    
    vector<int> * indexes;
    size_t tyfs = uima::internal::FSPromoter::demoteFS(fs);
    indexes = enqueuedFS[tyfs];
    if (NULL != indexes) {
      return false;
    }
    //cout << "enqueue new fs " << tyfs << " " << fs.getType().getName() << endl;
    // new FS, enqueue it
    indexes = new vector<int>;
    enqueuedFS[tyfs] = indexes;
    return true;
  }
  

  // return true if the fs was not previously enqueued
  bool XmiWriter::enqueueUnindexed(int id) {
    
    vector<int> * indexes;
    indexes = enqueuedFS[id];
    if (NULL != indexes) {
      return false;
    }
    //cout << "enqueue new fs " << tyfs << " " << fs.getType().getName() << endl;
    // new FS, enqueue it
    indexes = new vector<int>;
    enqueuedFS[id] = indexes;
    return true;
  }

  // return true if the fs was not previously enqueued
  bool XmiWriter::enqueueIndexed(FeatureStructure const &fs, int sofa) {
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
      // and look for references
      // currently this is done for every FS.
      // This could be done more efficiently 
      // when enqueueing the incoming FS.
      findReferencedFSs(fs, false);
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

  /**
  * Enqueues all FS that are stored in the XmiSerializationSharedData's id map.
  * This map is populated during the previous deserialization.  This method
  * is used to make sure that all incoming FS are echoed in the next
  * serialization.
  */
  void XmiWriter::enqueueIncoming() {
    if (this->sharedData == NULL)
      return;
    vector<int> fsAddrs;
    this->sharedData->getAllFsAddressesInIdMap(fsAddrs);
    for (size_t i = 0; i < fsAddrs.size(); i++) {
      enqueueUnindexed(fsAddrs.at(i));
    }
  }

  /**
  * Get the XMI ID to use for an FS.
  * 
  * @param addr
  *          address of FS
  * @return XMI ID. If addr == CASImpl.NULL, returns null
  */
  int XmiWriter::getXmiId(int addr) {

    if (addr == 0) return 0;
    if (sharedData != NULL) {
      return this->sharedData->getXmiId(addr);
    } else {
      return addr;    // in the absence of outside information, just use the FS address
    }   
  }

  void XmiWriter::enqueueFSListElements(FeatureStructure const & fs) {
    if (!fs.isValid()) {
      return;
    }

    ListFS curNode(fs);
    map<int,int> visited;
    while (!curNode.isEmpty()) { 
      int addr = internal::FSPromoter::demoteFS(curNode);
      if (visited[addr] == addr) {
        cerr << "Found Cycle truncating " << endl;
        ResourceManager::getInstance().getLogger().logWarning("Found cycle in FSlist. Truncating.");
        break;
      }
      visited[addr] = addr;
      FeatureStructure head = curNode.getHead();
      if (head.isValid()) {
        this->findReferencedFSs(head);
      }

      curNode = curNode.getTail();        
    }
  }


  string XmiWriter::listToString(FeatureStructure const & fs, char const * tag) {
    stringstream str;
    map<int,int> visited;
    if (!fs.isValid()) { 
      return str.str();
    }
    int typecode = internal::FSPromoter::demoteType(fs.getType());
    //cout << __LINE__ << " listToString() " << fs.getType().getName() << endl;
    switch (typecode) {
    case internal::gs_tyNEIntListType: {
      IntListFS curNode(fs);
      while (!curNode.isEmpty()) { 
        int head = curNode.getHead();
        str << head;
        curNode = curNode.getTail();
        int addr = internal::FSPromoter::demoteFS(curNode);
        if (visited[addr] == addr) {
          cerr << "Found Cycle truncating " << endl;
          ResourceManager::getInstance().getLogger().logWarning("Found cycle in Intlist. Truncating.");
          break;
        }
        visited[addr] = addr;
        if ( !curNode.isEmpty() ) {
          str << " " ;
        }
      }
      //cout << "intList contents " << str.str() << endl;
      break;
                                       }
    case internal::gs_tyNEFloatListType: {
      FloatListFS curNode(fs);
      while (!curNode.isEmpty()) { 
        str << curNode.getHead();
        curNode = curNode.getTail();
        int addr = internal::FSPromoter::demoteFS(curNode);
        if (visited[addr] == addr) {
          cerr << "Found Cycle truncating " << endl;
          ResourceManager::getInstance().getLogger().logWarning("Found cycle in FloatListFS. Truncating.");
          break;
        }
        visited[addr] = addr;
        if ( !curNode.isEmpty() ) {
          str << " " ;
        }
      }
      break;
                                         }
    case internal::gs_tyEListType: {
      //cout << "listToString() Empty FSList SKIP"  << endl;
      break;
                                   }
    case internal::gs_tyFSListType: {
      //cout << "listToString() FSList SKIP"  << endl;
      break;
                                    }
    case internal::gs_tyNEListType: {
      //cout << "listToString() NonEmpty FSList"  << endl;
      ListFS curNode(fs);
      while (!curNode.isEmpty()) { 
        int head = internal::FSPromoter::demoteFS(curNode.getHead());
        if (head == 0) {
          //null value in list.  Represent with "0".
          // this may be null because the element was originally a reference to an 
          // out-of-typesystem FS, so chck the XmiSerializationSharedData
          if (sharedData != NULL) {
            int addr = internal::FSPromoter::demoteFS(curNode);
            OotsElementData * oed = sharedData->getOutOfTypeSystemFeatures(addr);
            if (oed != NULL) {
              assert(oed->attributes.size() == 1); //only the head feature can possibly be here
              XmlAttribute *  attr = oed->attributes.at(0);
              assert(attr->name.compare(CAS::FEATURE_BASE_NAME_HEAD)==0);
              str << attr->value;
            }
          } else {
            str << head;
          }
        } else {
          int addr = internal::FSPromoter::demoteFS(curNode);
          if (visited[addr] == addr) {
            cerr << "Found Cycle truncating " << endl;
            ResourceManager::getInstance().getLogger().logWarning("Found cycle in NEListFS. Truncating.");
            break;
          }
          visited[addr] = addr;
          str << getXmiId(head);
        }

        curNode = curNode.getTail();
        if ( !curNode.isEmpty() ) {
          str << " " ;
        }
      }
      //cout << "FSListType  content " << str.str() << endl;
      break;
                                    }
    case internal::gs_tyNEStringListType:  {
      StringListFS curNode(fs);
      icu::UnicodeString ustr;
      while (!curNode.isEmpty()) { 
        ///string head = curNode.getHead().asUTF8();
        ustr.setTo("");
        normalize(curNode.getHead(), ustr);
        str << "<" << tag << ">" << ustr << "</" << tag << ">";
        curNode = curNode.getTail();
        int addr = internal::FSPromoter::demoteFS(curNode);
        if (visited[addr] == addr) {
          cerr << "Found Cycle truncating " << endl;
          ResourceManager::getInstance().getLogger().logWarning("Found cycle in StringListFS. Truncating.");
          break;
        }
        visited[addr] = addr;
        if ( !curNode.isEmpty() ) {
          str << " " ;
        }
      }
      break;
                                           }
    default: {
      cerr << "listToString() Invalid type " << fs.getType().getName() << endl;
      ErrorInfo errInfo;
      errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
      ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
      msg.addParam("listToString Invalid List type.");
      msg.addParam(tag);
      errInfo.setMessage(msg);
      errInfo.setSeverity(ErrorInfo::unrecoverable);
      ExcIllFormedInputError exc(errInfo);
      throw exc; 
    }
  }
  return str.str();
}


  /**
     * Serializes all of the out-of-typesystem elements that were recorded
     * in the XmiSerializationSharedData during the last deserialization.
     */
void XmiWriter::serializeOutOfTypeSystemElements(ostream & os)  {
  if (this->sharedData == NULL)
    return;
  vector<OotsElementData*> & ootsList = this->sharedData->getOutOfTypeSystemElements();
  for (size_t i=0; i < ootsList.size();i++) {
    OotsElementData * oed = ootsList.at(i);
    os << " <" << oed->elementName->qualifiedName;
    os << " " << ID_ATTR_NAME << "=\"" << oed->xmiId << "\"";
    // Add other attributes
    for (size_t a=0; a < oed->attributes.size();a++) {
      XmlAttribute * attr = oed->attributes.at(a);
      icu::UnicodeString us;
      icu::UnicodeString av(attr->value.c_str());
      normalize( av, us );
      os << " " << attr->name << "=\"" << us << "\"";
    }
    if (oed->childElements.size() > 0) {
      os << ">";
      //serialize features encoded as child elements
      map<string, vector<string>*>::iterator ite;
      for (ite = oed->childElements.begin(); ite != oed->childElements.end();ite++ ) {
        vector<string> * values = ite->second;
        for (size_t v=0; v < values->size(); v++) {
	        icu::UnicodeString us;
			icu::UnicodeString av(values->at(v).c_str());
			normalize( av, us );
          os << " <" << ite->first << ">";
          os << us;
          os << "</" << ite->first << ">";
        }
      }
      os << "</" << oed->elementName->qualifiedName << ">";
    } else {
      os << "/>";
    }
  }

}

char const *  XmiWriter::XMI_NS_URI =     "http://www.omg.org/XMI";
char const *  XmiWriter::XMI_NS_PREFIX = "xmi";
char const *  XmiWriter::DEFAULT_NAMESPACE_URI = "http:///uima/noNamespace.ecore";

char const *  XmiWriter::XSI_NS_URI= "http://www.w3.org/2001/XMLSchema-instance";
char const *  XmiWriter::XMI_TAG_LOCAL_NAME = "XMI";
char const *  XmiWriter::XMI_TAG_QNAME = "xmi:XMI";
char const *  XmiWriter::INDEXED_ATTR_NAME = "_indexed";
char const *  XmiWriter::ID_ATTR_NAME = "xmi:id";
char const *  XmiWriter::XMI_VERSION_LOCAL_NAME = "version";
char const *  XmiWriter::XMI_VERSION_QNAME = "xmi:version";
char const *  XmiWriter::XMI_VERSION_VALUE = "2.0";

}


/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */








