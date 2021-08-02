#ifndef UIMA_XMISHAREDDATA_HPP
#define UIMA_XMISHAREDDATA_HPP
/** \file xmlwriter.hpp .
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

   \brief Used to output the CAS in XCAS format

-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp>

#include <iostream>
#include <vector>
#include <set>
#include <sstream>
#include <map>

#include <uima/msg.h>
#include <uima/exceptions.hpp>
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */
namespace uima {

class XmlElementName {
  public:
	  std::string nsUri;
	  std::string shortName;
	  std::string qualifiedName;
	  XmlElementName(std::string ns, std::string sname, std::string qname) :
    nsUri(ns), shortName(sname), qualifiedName(qname) {}
  };

/**
 * Data structure representing an XML attribute.
 */
class XmlAttribute {
public:
 std::string name;
  std::string value;
  
  XmlAttribute(icu::UnicodeString name, icu::UnicodeString value) {
    this->name = ((UnicodeStringRef)name).asUTF8();
    this->value = ((UnicodeStringRef)value).asUTF8();
  }
  XmlAttribute(std::string name, std::string value) {
    this->name = name;
    this->value = value;
  }
};

 /**
   * Data structure holding all information about an XMI element
   * containing an out-of-typesystem FS.
   */
  class OotsElementData {
  public:
    /**
     * xmi:id of the element
     */
    int xmiId;

    /**
     * Name of the element, including XML namespace.
     */
    XmlElementName * elementName;

    /**
     * List of XmlAttribute objects each holding name and value of an attribute.
     */
    std::vector<XmlAttribute*>  attributes;
    
    /**
     * Map qualified name of an attribute to a list of strings that contain
	 * the values of the element. Use to store the
     * child elements representing features of this out-of-typesystem element.
     */
    std::map<std::string, std::vector<std::string>* > childElements;

	OotsElementData() : elementName(0) {}

	~OotsElementData() {
	  if (elementName != NULL) {
	    delete elementName;
		elementName=0;
	  }
	  for (size_t i=0; i < attributes.size();i++) {
		if (attributes.at(i) != NULL) {
	      delete attributes.at(i);
		}
	  }
	  std::map<std::string, std::vector<std::string>* >::iterator ite;
	  for (ite=childElements.begin(); ite != childElements.end();ite++) {
	    if (ite->second != NULL) {
		  delete ite->second;
		}
	  }
    }
  };
  
  /** 
   * Data structure holding the index and the xmi:id of an array or list element that
   * is a reference to an out-of-typesystem FS.
   */
  class XmiArrayElement {
  public:
    int index;
    int xmiId;

    XmiArrayElement(int index, int xmiId) {
      this->index = index;
      this->xmiId = xmiId;
    }
  };
/**
 * Holds information that is shared between the XmiCasSerializer and the XmiCasDeserializer. This
 * allows consistency of XMI IDs across serializations, and also provides the ability to filter out
 * some FSs during serialization (e.g. to send to a service) and then reintegrate those FSs during
 * the next deserialization.
 * 
 */
class UIMA_LINK_IMPORTSPEC XmiSerializationSharedData {
  private:
  /**
   * A map from FeatureStructure address to XMI ID. This is built during deserialization, then used
   * by the next serialization to ensure consistent IDs.
   */
    std::map<int, int> fsAddrToXmiIdMap;
   /** 
   * A map from xmi:id to FeatureStructure address.  This is populated whenever
   * an XMI element is serialized or deserialized.  It is used by the
   * getFsAddrForXmiId() method, necessary to support merging multiple XMI
   * CASes into the same CAS object.
   **/
   std::map<int,int> xmiIdToFsAddrMap;
  
  /**
   * List of OotsElementData objects, each of which captures information about
   * incoming XMI elements that did not correspond to any type in the type system.
   */
  std::vector<OotsElementData*> ootsFs;
  
  /**
   * Map that from the xmi:id  Sofa to a List of xmi:id's  for
   * the out-of-typesystem FSs that are members of that Sofa's view.
   */
  std::map<int,std::vector<int>*> ootsViewMembers;

  /** Map from Feature Structure address (Integer) to OotsElementData object, capturing information 
   * about out-of-typesystem features that were part of an in-typesystem FS.  These include both
   * features not defined in the typesystem and features that are references to out-of-typesystem
   * elements.  This information needs to be included when the FS is subsequently serialized.
   */
  std::map<int, OotsElementData*> ootsFeatures;
  
  /** Map from Feature Structure address (Integer) of an FSArray to a list of 
   * {@link XmiArrayElement} objects, each of which holds an index and an xmi:id
   * for an out-of-typesystem array element.
   */
  std::map<int,std::vector<XmiArrayElement*>*> ootsArrayElements;
  

  /**
   * The maximum XMI ID used in the serialization. Used to generate unique IDs if needed.
   */
    int maxXmiId;

public:
  XmiSerializationSharedData() : maxXmiId(0) {
  }
  ~XmiSerializationSharedData() {
    for (size_t i=0; i < ootsFs.size(); i++) {
      if (ootsFs.at(i) != NULL) {
        delete ootsFs.at(i);
      }
    }

    std::map<int,std::vector<XmiArrayElement*>*>::iterator ite;
    for (ite = ootsArrayElements.begin(); ite != ootsArrayElements.end();ite++) {
      if (ite->second != NULL) {
        std::vector<XmiArrayElement*>* vec =  ite->second;
        for (size_t i=0; i < vec->size(); i++) {
          if (vec->at(i) != NULL) {
            delete vec->at(i);
          }
        }
        delete vec;
      }
    }

	std::map<int,std::vector<int>*>::iterator viewite;
	for (viewite = ootsViewMembers.begin(); 
		viewite != ootsViewMembers.end();viewite++) {
		if (viewite->second != NULL) {
			delete viewite->second;
		}
    }
	std::map<int, OotsElementData*>::iterator featite;
	for (featite = ootsFeatures.begin(); 
		featite != ootsFeatures.end();featite++) {
		if (featite->second != NULL) {
			delete featite->second;
		}
    }
  }

  void addIdMapping(int fsAddr, int xmiId) { 
    fsAddrToXmiIdMap[fsAddr] = xmiId;
    xmiIdToFsAddrMap[xmiId] = fsAddr;
    if (xmiId > maxXmiId)
      maxXmiId = xmiId;
  }

  int getXmiId(int fsAddr) {
    // see if we already have a mapping
    std::map<int,int>::iterator ite = fsAddrToXmiIdMap.find(fsAddr);
    if (ite != fsAddrToXmiIdMap.end()) {
      return ite->second;
    } else { // no mapping for this FS. Generate a unique ID
      // to be sure we get a unique Id, increment maxXmiId and use that
      fsAddrToXmiIdMap[fsAddr] = ++maxXmiId;
      xmiIdToFsAddrMap[maxXmiId] = fsAddr;
      return maxXmiId;
    }
  }
   /**
   * Gets the FS address that corresponds to the given xmi:id, in the most
   * recent serialization or deserialization.
   *   
   * @param xmiId an xmi:id from the most recent XMI CAS that was serialized
   *   or deserialized.
   * @return the FS address of the FeatureStructure corresponding to that
   *   xmi:id, -1 if none.
   */
  int getFsAddrForXmiId(int xmiId) {
	  std::map<int,int>::iterator ite =  xmiIdToFsAddrMap.find(xmiId);
	  if (ite != xmiIdToFsAddrMap.end() ) 
		  return ite->second;
	  else  {
		  //cout << __FILE__ << __LINE__ << " getFsAddrForXmiId() could not find xmiid " << xmiId << endl; 
      return -1;	   
			/**ErrorInfo errInfo;
		  errInfo.setErrorId((TyErrorId)UIMA_ERR_RESOURCE_CORRUPTED);
		  ErrorMessage msg(UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR);
		  msg.addParam("getFSAddrForXmiId Unknown xmiId");
		  msg.addParam(xmiId);
		  errInfo.setMessage(msg);
		  errInfo.setSeverity(ErrorInfo::unrecoverable);
		  ExcIllFormedInputError exc(errInfo);
		  throw exc;**/ 
	  }
  }
  
 

  /**
   * Records information about an XMI element that was not an instance of any type in the type system.
   * @param elemData information about the out-of-typesystem XMI element
   */
  void addOutOfTypeSystemElement(OotsElementData * elemData) {
    this->ootsFs.push_back(elemData);
    //check if we need to update max ID
    int xmiId = elemData->xmiId;
    if (elemData->xmiId > maxXmiId)
      maxXmiId = elemData->xmiId;
  }

  /**
   * Gets a List of {@link OotsElementData} objects, each of which describes an
   * incoming XMI element that did not correspond to a Type in the TypeSystem.
   * @return List of {@link OotsElementData} objects
   */
  std::vector<OotsElementData *> & getOutOfTypeSystemElements() {
    return this->ootsFs;
  }
  
  /**
   * Records that an out-of-typesystem XMI element should be a member of the
   * specified view.
   * @param sofaXmiId xmi:id of a Sofa
   * @param memberXmiId xmi:id of an out-of-typesystem element that should be
   *   a member of the view for the given Sofa
   */
  void addOutOfTypeSystemViewMember(int sofaXmiId, int memberXmiId) {
    std::vector<int> * memberList = 0; 
    std::map<int,std::vector<int>* >::iterator ite = this->ootsViewMembers.find(sofaXmiId);
    if (ite == ootsViewMembers.end() ) {
      memberList = new std::vector<int>;
      ootsViewMembers[sofaXmiId] = memberList;
    } else {
      memberList = ite->second;
    }
    memberList->push_back(memberXmiId);
  }
  
  /**
   * Gets a List of xmi:id's (Strings) of all out-of-typesystem XMI elements
   * that are members of the view with the given id.
   * @param sofaXmiId xmi:id of a Sofa
   * @return List of xmi:id's of members of the view for the given Sofa.
   */
  void getOutOfTypeSystemViewMembers(int sofaXmiId, std::vector<int>& tofill) {
    std::map<int, std::vector<int> *>::iterator ite = this->ootsViewMembers.find(sofaXmiId); 
    std::vector<int> * memberList = NULL;
    if (ite != ootsViewMembers.end()) {
      memberList = ite->second;
      for (size_t i=0;i < memberList->size();i++) {
        tofill.push_back(memberList->at(i));
      }
    }

  }
  
  /**
   * Records an out-of-typesystem attribute that belongs to an in-typesystem FS.
   * This will be added to the attributes when that FS is reserialized.
   * @param addr CAS address of the FS 
   * @param featName name of the feature
   * @param featVal value of the feature, as a string
   */
  void addOutOfTypeSystemAttribute(int addr, std::string featName, std::string featVal) {
    std::map<int, OotsElementData*>::iterator ite = this->ootsFeatures.find(addr);
    OotsElementData * oed = NULL;
    if (ite != ootsFeatures.end() )
      oed = ite->second;
    if (oed == NULL) {
      oed = new OotsElementData();
      this->ootsFeatures[addr] = oed;
    }
    oed->attributes.push_back(new XmlAttribute(featName, featVal));
  }  

  void addOutOfTypeSystemAttribute(int addr, icu::UnicodeString& featName, 
    icu::UnicodeString & featVal) {
      //cout << "addOotsAttribute " << featName << "=" << featVal << endl;
      addOutOfTypeSystemAttribute(addr, ( (UnicodeStringRef)featName).asUTF8(),
        ( (UnicodeStringRef)featVal).asUTF8());
  }

  /**
   * Records out-of-typesystem child elements that belong to an in-typesystem FS.
   * These will be added to the child elements when that FS is reserialized.
   * @param addr CAS address of the FS 
   * @param featName name of the feature (element tag name)
   * @param featVal values of the feature, as a List of strings
   */
    void addOutOfTypeSystemChildElements(int addr, std::string featName, std::vector<std::string> featVals) {
      std::map<int, OotsElementData*>::iterator ite = this->ootsFeatures.find(addr);
      OotsElementData * oed = NULL;
      if (ite != ootsFeatures.end() )
        oed = ite->second;
      if (oed == NULL) {
        oed = new OotsElementData();
        this->ootsFeatures[addr] = oed;
      } 

      std::map<std::string, std::vector<std::string>*>::iterator ite2 = oed->childElements.find(featName);
      std::vector<std::string> * pVals = NULL;
      if (ite2 ==	oed->childElements.end() ) {
        pVals = new std::vector<std::string>;
        oed->childElements[featName] = pVals;
      } else {
        pVals = ite2->second;
      }

      for (size_t i = 0; i < featVals.size();i++) {
        pVals->push_back(featVals.at(i));
      }

    }  
  
  /**
   * Gets information about out-of-typesystem features that belong to an
   * in-typesystem FS.
   * @param addr CAS address of the FS
   * @return object containing information about out-of-typesystem features
   *   (both attributes and child elements)
   */
  OotsElementData * getOutOfTypeSystemFeatures(int addr) {
	  std::map<int, OotsElementData*>::iterator ite = this->ootsFeatures.find(addr);
	  if (ite != this->ootsFeatures.end())
		  return ite->second;
      else return NULL; 
  }
  
  /**
   * Get all FS Addresses that have been added to the id map.
   * @return an array containing all the FS addresses
   */
  void getAllFsAddressesInIdMap( std::vector<int> & tofill) {    
	  std::map<int,int>::iterator ite;
	  for (ite = this->fsAddrToXmiIdMap.begin(); ite != fsAddrToXmiIdMap.end();ite++) {
		  tofill.push_back(ite->first);
	  }
  }  
  
  /**
   * Gets information about out-of-typesystem array elements.
   * @param addr the CAS address of an FSArray
   * @return a List of {@link XmiArrayElement} objects, each of which
   *   holds the index and xmi:id of an array element that is a
   *   reference to an out-of-typesystem FS.
   */
  std::vector<XmiArrayElement*> * getOutOfTypeSystemArrayElements(int addr) {
	  std::map<int, std::vector<XmiArrayElement*>*>::iterator ite=this->ootsArrayElements.find(addr);
      if (ite != ootsArrayElements.end()) 
		  return ite->second;
	  else return NULL;
  }
  

  /**
   * Records an out-of-typesystem array element in the XmiSerializationSharedData.
   * @param addr CAS address of FSArray
   * @param index index into array 
   * @param xmiId xmi:id of the out-of-typesystem element that is the value at the given index
   */
  void addOutOfTypeSystemArrayElement(int addr, int index, int xmiId) {
    std::map<int, std::vector<XmiArrayElement*>* >::iterator ite = this->ootsArrayElements.find(addr);
    std::vector<XmiArrayElement*> * oed = NULL;
    if (ite != ootsArrayElements.end() )
      oed  = ite->second;
    else {
      oed = new std::vector<XmiArrayElement*>;
      this->ootsArrayElements[addr] = oed;
    }
    oed->push_back(new XmiArrayElement(index, xmiId));
  }


  void clearIdMap() {
      fsAddrToXmiIdMap.clear();
      xmiIdToFsAddrMap.clear();
      maxXmiId = 0;
  }

  void print() {
    std::map<int,OotsElementData *>::iterator ite;
    for (size_t i=0; i < ootsFs.size() ;i++ ) {
       OotsElementData * oed = ootsFs.at(i);
       std::cout << "**OotsFS " << i << oed->elementName->qualifiedName << std::endl;
       std::cout << "NUM CHILDELEMENTS " << oed->childElements.size() << std::endl;
       for (size_t i=0; i < oed->attributes.size();i++) {
         std::cout << oed->attributes.at(i)->name << " " << oed->attributes.at(i)->value << std::endl;
       }    
    }
  }

  /**
   * For debugging purposes only.
   */
  
  /**
  string toString() {
    
    stringstream str; 
 
    int[] keys = fsAddrToXmiIdMap;
    for (int i = 0; i < keys.length; i++) {
      buf.append(keys[i]).append(": ").append(fsAddrToXmiIdMap.get(keys[i])).append('\n');
    }
    return str.str();;
  }**/

};

}


/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif
