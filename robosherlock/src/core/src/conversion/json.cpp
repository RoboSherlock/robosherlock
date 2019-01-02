/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *         Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *         Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// STL
#include <map>

// UNICODE STRING
#include <unicode/unistr.h>

// RS
#include <rs/conversion/conversion.h>
#include <rs/conversion/json.h>
#include <rs/utils/output.h>

namespace rs
{
namespace conversion
{

///******************************************************************************
// * Conversion:: Definitions
// *****************************************************************************/

//#define FIELD_ID     "_id"
#define FIELD_ID_CAS "id"
//#define FIELD_TYPE   "_type"
#define FIELD_DATA   "_data"
//#define FIELD_PARENT "_parent"

static int ARRAY_SIZE_LIMIT = {20};
//#define ARRAY_NATIVE_LIMIT 20 // Store array with a size smaller than ARRAY_NATIVE_LIMIT as native BSON arrays else as binary

///******************************************************************************
// * Conversion:: Type Lookup Maps
// *****************************************************************************/

typedef void (*FromBasicTypeFunction)(const uima::FeatureStructure &fs, const uima::Feature &feature, const std::string &name, rapidjson::Document &document, rapidjson::MemoryPoolAllocator<> &allocator);
//typedef void (*ToBasicTypeFunction)(uima::CAS &cas, uima::FeatureStructure &fs, const uima::Feature &feature, const mongo::BSONElement &elem);

typedef std::map<uima::Type, FromBasicTypeFunction> FromBasicTypes;
//typedef std::map<uima::Type, ToBasicTypeFunction> ToBasicTypes;

static FromBasicTypes fromBasicTypes;
//static ToBasicTypes toBasicTypes;

typedef void (*FromArrayTypeFunction)(const uima::FeatureStructure &fs, const std::string &fieldName, rapidjson::Document &builder, rapidjson::MemoryPoolAllocator<> &allocator);
//typedef uima::FeatureStructure(*ToArrayTypeFunction)(uima::CAS &cas, const mongo::BSONElement &elem);

typedef std::map<uima::Type, FromArrayTypeFunction> FromArrayTypes;
//typedef std::map<uima::Type, ToArrayTypeFunction> ToArrayTypes;

static FromArrayTypes fromArrayTypes;
//static ToArrayTypes toArrayTypes;

void initFunctionMaps(const uima::CAS &cas);

///******************************************************************************
// * Conversion:: Prototypes
// *****************************************************************************/

void fromFeatureStructureAux(const uima::FeatureStructure &fs, rapidjson::Document &parent,  rapidjson::MemoryPoolAllocator<> &allocator);
//uima::FeatureStructure toFeatureStructureAux(uima::CAS &cas, const mongo::BSONObj &object);

///******************************************************************************
// * Conversion:: Basic Feature
// *****************************************************************************/

#define DATA_FEATURE_IMPL(_TYPE_, _NAME_, _CAST_)\
void fromFeature##_NAME_(const uima::FeatureStructure &fs, const uima::Feature &feature, const std::string &name, rapidjson::Document &document, rapidjson::MemoryPoolAllocator<> &allocator)\
{\
  rapidjson::Value val, key(name, allocator);\
  val.Set##_CAST_((fs.get##_NAME_##Value(feature)));\
  document.AddMember(key, val, allocator);\
}

//void toFeature##_NAME_(uima::CAS &cas, uima::FeatureStructure &fs, const uima::Feature &feature, const mongo::BSONElement &elem)\
//{\
//  _CAST_ tmp;\
//  elem.Val(tmp);\
//  fs.set##_NAME_##Value(feature, (_TYPE_)tmp);\
//}

DATA_FEATURE_IMPL(bool, Boolean, Bool)
DATA_FEATURE_IMPL(char, Byte, Int)
DATA_FEATURE_IMPL(short, Short, Int)
DATA_FEATURE_IMPL(int, Int, Int)
DATA_FEATURE_IMPL(INT64, Long, Int64)
DATA_FEATURE_IMPL(float, Float, Float)
DATA_FEATURE_IMPL(double, Double, Double)

///******************************************************************************
// * Conversion:: String Feature
// *****************************************************************************/

void fromFeatureString(const uima::FeatureStructure &fs, const uima::Feature &feature, const std::string &name, rapidjson::Document &document, rapidjson::MemoryPoolAllocator<> &allocator)
{
  std::string stringValue = fs.getStringValue(feature).asUTF8();

  rapidjson::Value key;
  char buffer[name.length()];
  int len = sprintf(buffer, name.c_str());
  key.SetString(buffer, len, allocator);

  rapidjson::Value val;
  char buffer2[stringValue.length()];
  int len2 = sprintf(buffer2, stringValue.c_str());
  val.SetString(buffer2, len2, allocator);
  document.AddMember(key.Move(), val.Move(), allocator);

  //  rapidjson::StringBuffer rBuffer;
  //  rapidjson::Writer<rapidjson::StringBuffer> writer(rBuffer);
  //  document.Accept(writer);
  //  std::cerr << __FILE__ << "::" << __LINE__ << rBuffer.GetString() << std::endl;

}

//void toFeatureString(uima::CAS &cas, uima::FeatureStructure &fs, const uima::Feature &feature, const mongo::BSONElement &elem)
//{
//  UnicodeString value = UnicodeString::fromUTF8(elem.String());
//  fs.setStringValue(feature, value);
//}

///******************************************************************************
// * Conversion:: FS Feature
// *****************************************************************************/
void fromFeatureFeatureStructure(const uima::FeatureStructure &fs, const uima::Feature &feature, const std::string &name, rapidjson::Document &document, rapidjson::MemoryPoolAllocator<> &allocator)
{
  rapidjson::Value fieldNameV(name, allocator);

  rapidjson::Document newDoc(rapidjson::kObjectType);
  fromFeatureStructureAux(fs.getFSValue(feature), newDoc, allocator);
  document.AddMember(fieldNameV, newDoc, allocator);
  //  builder.append(name, fromFeatureStructureAux(fs.getFSValue(feature), oid.OID()));
}

//void toFeatureFeatureStructure(uima::CAS &cas, uima::FeatureStructure &fs, const uima::Feature &feature, const mongo::BSONElement &elem)
//{
//  uima::FeatureStructure subFS = toFeatureStructureAux(cas, elem.Obj());
//  if(subFS.isValid()) {
//    fs.setFSValue(feature, subFS);
//  }
//}

///******************************************************************************
// * Conversion:: Feature Type selection
// *****************************************************************************/

void fromFeature(const uima::FeatureStructure &fs, const uima::Feature &feature, rapidjson::Document &builder, rapidjson::MemoryPoolAllocator<> &allocator)
{
  uima::Type featureType;
  feature.getRangeType(featureType);
  const uima::UnicodeStringRef featureName = feature.getName();
  const std::string &name = featureName.asUTF8();

  FromBasicTypes::const_iterator itB = fromBasicTypes.find(featureType);
  if(itB != fromBasicTypes.end()) {
    (*itB->second)(fs, feature, name, builder, allocator);
    return;
  }

  FromArrayTypes::const_iterator itA = fromArrayTypes.find(featureType);
  if(itA != fromArrayTypes.end()) {
    const uima::FeatureStructure &subFS = fs.getFSValue(feature);
    if(subFS.isValid()) {
      (*itA->second)(subFS, name, builder, allocator);
    }
    return;
  }
  fromFeatureFeatureStructure(fs, feature, name, builder, allocator);
}

//void toFeature(uima::CAS &cas, uima::FeatureStructure fs, const uima::Type &fsType, const mongo::BSONElement &elem)
//{
//  const UnicodeString name = UnicodeString::fromUTF8(elem.fieldName());
//  uima::Feature feature = fsType.getFeatureByBaseName(name);
//  uima::Type featureType;
//  feature.getRangeType(featureType);

//  ToBasicTypes::const_iterator itB = toBasicTypes.find(featureType);
//  if(itB != toBasicTypes.end()) {
//    (*itB->second)(cas, fs, feature, elem);
//    return;
//  }

//  ToArrayTypes::const_iterator itA = toArrayTypes.find(featureType);
//  if(itA != toArrayTypes.end()) {
//    if(!elem.isABSONObj() || !elem.Obj().isEmpty()) {
//      fs.setFSValue(feature, (*itA->second)(cas, elem));
//    }
//    return;
//  }

//  toFeatureFeatureStructure(cas, fs, feature, elem);
//}

///******************************************************************************
// * Conversion:: Basic Feature Structure
// *****************************************************************************/

void fromBasicFeatureStructure(const uima::FeatureStructure &fs, const uima::Type &fsType, const std::string &type,
                               rapidjson::Document &parent, rapidjson::MemoryPoolAllocator<> &allocator)
{
  std::vector<uima::Feature> features;
  fsType.getAppropriateFeatures(features);

  uima::Feature idFeature;
  std::string id;
  try {
    idFeature = fsType.getFeatureByBaseName(FIELD_ID_CAS);
    if(fsType.isAppropriateFeature(idFeature)) {
      id = fs.getStringValue(idFeature).asUTF8();
    }
  }
  catch(const uima::InvalidFSTypeObjectException) {
  }
  for(size_t i = 0; i < features.size(); ++i) {
    const uima::Feature &feature = features[i];
    if(feature != idFeature) {
      fromFeature(fs, feature, parent, allocator);
    }
  }
  //  rapidjson::StringBuffer buffer;
  //  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  //  parent.Accept(writer);
  //  outInfo(buffer.GetString());
}

//uima::FeatureStructure toBasicFeatureStructure(uima::CAS &cas, const uima::Type &fsType, const mongo::BSONObj &object)
//{
//  uima::FeatureStructure newFS = cas.createFS(fsType);

//  try {
//    const uima::Feature &idFeature = fsType.getFeatureByBaseName(FIELD_ID_CAS);
//    if(fsType.isAppropriateFeature(idFeature)) {
//      mongo::BSONElement elem;
//      if(object.getObjectID(elem)) {
//        newFS.setStringValue(idFeature, UnicodeString::fromUTF8(elem.OID().toString()));
//      }
//    }
//  }
//  catch(const uima::InvalidFSTypeObjectException) {
//  }

//  std::vector<mongo::BSONElement> elems;
//  object.elems(elems);

//  for(size_t i = 0; i < elems.size(); ++i) {
//    const mongo::BSONElement &elem = elems[i];
//    if(elem.fieldName()[0] != '_') {
//      toFeature(cas, newFS, fsType, elem);
//    }
//  }
//  return newFS;
//}


#define DATA_ARRAY_IMPL(_TYPE_, _NAME_, _RJSON_)\
void fromArrayFS##_NAME_(const uima::FeatureStructure &fs, const std::string &fieldName, rapidjson::Document &document, rapidjson::MemoryPoolAllocator<> &allocator)\
{\
  uima::_NAME_##ArrayFS arrayFS(fs);\
  rapidjson::Value key(fieldName, allocator);\
  size_t size = 0;\
  if(arrayFS.isValid())\
  {\
    size = arrayFS.size();\
  }\
  rapidjson::Document arrayVal(rapidjson::kArrayType);\
  if(size < ARRAY_SIZE_LIMIT)\
  {\
    for(size_t i = 0; i < size; ++i)\
    {\
      _TYPE_ val = arrayFS.get(i);\
      rapidjson::Value atom;\
      atom.Set##_RJSON_(val);\
      arrayVal.PushBack(atom, allocator);\
    }\
  }\
  document.AddMember(key, arrayVal, allocator);\
}
//uima::FeatureStructure toArrayFS##_NAME_(uima::CAS &cas, const mongo::BSONElement &elem)\
//{\
//  switch(elem.type())\
//  {\
//  case mongo::BinData:\
//    return toArrayFS##_NAME_##Binary(cas, elem);\
//  case mongo::Array:\
//    return toArrayFS##_NAME_##Native(cas, elem);\
//  default:\
//    outError("unsupported element type!");\
//    return uima::FeatureStructure();\
//  }\
//}

DATA_ARRAY_IMPL(bool, Boolean, Bool)
DATA_ARRAY_IMPL(char, Byte, Int)
DATA_ARRAY_IMPL(short, Short, Int)
DATA_ARRAY_IMPL(int, Int, Int)
DATA_ARRAY_IMPL(INT64, Long, Int64)
DATA_ARRAY_IMPL(float, Float, Float)
DATA_ARRAY_IMPL(double, Double, Float)

///******************************************************************************
// * Conversion:: String Array Feature Structure
// *****************************************************************************/

void fromArrayFSString(const uima::FeatureStructure &fs, const std::string &fieldName, rapidjson::Document &document, rapidjson::MemoryPoolAllocator<> &allocator)
{
  uima::StringArrayFS arrayFS(fs);
  size_t size = 0;
  if(arrayFS.isValid()) {
    size = arrayFS.size();
  }
  rapidjson::Document arrayVal(rapidjson::kArrayType);
  for(size_t i = 0; i < size && size < ARRAY_SIZE_LIMIT; ++i) {
    uima::UnicodeStringRef ref = arrayFS.get(i);
    rapidjson::Value atom;
    atom.SetString(ref.asUTF8(), allocator);
    arrayVal.PushBack(atom, allocator);
  }
  rapidjson::Value key(fieldName, allocator);
  document.AddMember(key, arrayVal, allocator);
}

//uima::FeatureStructure toArrayFSString(uima::CAS &cas, const mongo::BSONElement &elem)
//{
//  std::vector<std::string> data;
//  //  elem.Obj().Vals(data);

//  mongo::BSONObjIterator i(elem.Obj());
//  while(i.more()) {
//    std::string t;
//    i.next().Val(t);
//    data.push_back(t);
//  }
//  uima::StringArrayFS arrayFS = cas.createStringArrayFS(data.size());
//  for(size_t i = 0; i < data.size(); ++i) {
//    UnicodeString str = UnicodeString::fromUTF8(data[i]);
//    arrayFS.set(i, uima::UnicodeStringRef(str));
//  }
//  return arrayFS;
//}

///******************************************************************************
// * Conversion:: FS Array Feature Structure Implementation
// *****************************************************************************/

void fromArrayFSFeatureStructure(const uima::FeatureStructure &fs, const std::string &fieldName, rapidjson::Document &document, rapidjson::MemoryPoolAllocator<> &allocator)
{
  uima::ArrayFS arrayFS(fs);
  size_t size = 0;
  if(arrayFS.isValid()) {
    size = arrayFS.size();
  }
  std::vector<mongo::BSONObj> data(size);
  rapidjson::Document arrayVal(rapidjson::kArrayType);

  for(size_t i = 0; i < size && size< ARRAY_SIZE_LIMIT; ++i) {
    rapidjson::Document doc(rapidjson::kObjectType);
    fromFeatureStructureAux(arrayFS.get(i), doc, allocator);
    arrayVal.PushBack(doc.Move(), allocator);
  }

  rapidjson::Value key(fieldName, allocator);
  document.AddMember(key, arrayVal, allocator);
}

//uima::FeatureStructure toArrayFSFeatureStructure(uima::CAS &cas, const mongo::BSONElement &elem)
//{
//  std::vector<mongo::BSONObj> objects;
//  //elem.Obj().Vals(objects);
//  mongo::BSONObjIterator i(elem.Obj());
//  while(i.more()) {
//    mongo::BSONObj t;
//    i.next().Val(t);
//    objects.push_back(t);
//  }
//  uima::ArrayFS arrayFS = cas.createArrayFS(objects.size());
//  for(size_t i = 0; i < objects.size(); ++i) {
//    arrayFS.set(i, toFeatureStructureAux(cas, objects[i]));
//  }
//  return arrayFS;
//}

///******************************************************************************
// * Conversion:: Basic List Feature Structure with casting
// *****************************************************************************/

//#define DATA_LIST_CAST_IMPL(_TYPE_, _NAME_, _CAST_)\
//void fromListFS##_NAME_##Native(uima::_NAME_##ListFS &listFS, const size_t &size, const std::string &fieldName, mongo::BSONObjBuilder &builder, const mongo::OID &parent)\
//{\
//  std::vector<_CAST_> data(size);\
//  for(size_t i = 0; i < size; ++i)\
//  {\
//    data[i] = (_CAST_)listFS.getHead();\
//    listFS.moveToNext();\
//  }\
//  builder.append(fieldName, data);\
//}\
//uima::FeatureStructure toListFS##_NAME_##Native(uima::CAS &cas, const mongo::BSONElement &elem)\
//{\
//  std::vector<_CAST_> data;\
//  mongo::BSONObjIterator i(elem.Obj());\
//  while( i.more() )\
//  {\
//    _CAST_ t;\
//    i.next().Val(t);\
//    data.push_back(t);\
//  }\
//  uima::_NAME_##ListFS listFS = cas.create##_NAME_##ListFS();\
//  for(int i = data.size() - 1; i >= 0; --i)\
//  {\
//    listFS.addFirst((_TYPE_)data[i]);\
//  }\
//  return listFS;\
//}

//DATA_LIST_CAST_IMPL(int, Int, int)
//DATA_LIST_CAST_IMPL(float, Float, double)

///******************************************************************************
// * Conversion:: Basic List Feature Structure implementation selection
// *****************************************************************************/

#define DATA_LIST_IMPL(_NAME_, _TYPE_)\
void fromListFS##_NAME_(const uima::FeatureStructure &fs, const std::string &fieldName, rapidjson::Document &document, rapidjson::MemoryPoolAllocator<> &allocator)\
{\
  uima::_NAME_##ListFS listFS(fs);\
  size_t size = 0;\
  if(listFS.isValid())\
  {\
    size = listFS.getLength();\
  }\
  rapidjson::Value key (fieldName,allocator);\
  rapidjson::Document arrayVal(rapidjson::kArrayType);\
  for(size_t i = 0; i < size && size < ARRAY_SIZE_LIMIT; ++i)\
  {\
    _TYPE_ head = (_TYPE_)listFS.getHead();\
    rapidjson::Value item(head);\
    arrayVal.PushBack(item, allocator);\
    listFS.moveToNext();\
  }\
  document.AddMember(key, arrayVal,allocator);\
}\
//uima::FeatureStructure toListFS##_NAME_(uima::CAS &cas, const mongo::BSONElement &elem)\
//{\
//  switch(elem.type())\
//  {\
//  case mongo::BinData:\
//    return toListFS##_NAME_##Binary(cas, elem);\
//  case mongo::Array:\
//    return toListFS##_NAME_##Native(cas, elem);\
//  default:\
//    outError("unsupported element type!");\
//    return uima::FeatureStructure();\
//  }\
//}

DATA_LIST_IMPL(Int, int)
DATA_LIST_IMPL(Float, float)

///******************************************************************************
// * Conversion:: String List Feature Structure
// *****************************************************************************/

void fromListFSString(const uima::FeatureStructure &fs, const std::string &fieldName,  rapidjson::Document &document, rapidjson::MemoryPoolAllocator<> &allocator)
{
  uima::StringListFS listFS(fs);
  size_t size = 0;
  if(listFS.isValid()) {
    size = listFS.getLength();
  }

  rapidjson::Document arrayVal(rapidjson::kArrayType);
  for(size_t i = 0; i < size && size < ARRAY_SIZE_LIMIT; ++i) {
    uima::UnicodeStringRef ref = listFS.getHead();
    rapidjson::Value value;
    value.SetString(ref.asUTF8(),allocator);
    arrayVal.PushBack(value, allocator);
    listFS.moveToNext();
  }

  rapidjson::Value key (fieldName,allocator);
  document.AddMember(key, arrayVal, allocator);
}

//uima::FeatureStructure toListFSString(uima::CAS &cas, const mongo::BSONElement &elem)
//{
//  std::vector<std::string> data;
//  //elem.Obj().Vals(data);
//  mongo::BSONObjIterator i(elem.Obj());
//  while(i.more()) {
//    std::string t;
//    i.next().Val(t);
//    data.push_back(t);
//  }
//  uima::StringListFS listFS = cas.createStringListFS();
//  for(int i = data.size() - 1; i >= 0; --i) {
//    UnicodeString str = UnicodeString::fromUTF8(data[i]);
//    listFS.addFirst(uima::UnicodeStringRef(str));
//  }
//  return listFS;
//}

///******************************************************************************
// * Conversion:: FS List Feature Structure
// *****************************************************************************/

void fromListFSFeatureStructure(const uima::FeatureStructure &fs, const std::string &fieldName, rapidjson::Document &document, rapidjson::MemoryPoolAllocator<> &allocator)
{
  uima::ListFS listFS(fs);
  size_t size = 0;
  if(listFS.isValid()) {
    size = listFS.getLength();
  }

  rapidjson::Value key(fieldName, allocator);
  rapidjson::Document arrayVal(rapidjson::kArrayType);
  for(size_t i = 0; i < size && size < ARRAY_SIZE_LIMIT; ++i) {
    rapidjson::Document doc(rapidjson::kObjectType);
    uima::FeatureStructure elem = listFS.getHead();
    rapidjson::Value elemKey (elem.getType().getName().asUTF8(),allocator);
    fromFeatureStructureAux(listFS.getHead(), doc, allocator);
    arrayVal.PushBack(doc.Move(),allocator);
    listFS.moveToNext();
  }
  document.AddMember(key, arrayVal, allocator);
}

//uima::FeatureStructure toListFSFeatureStructure(uima::CAS &cas, const mongo::BSONElement &elem)
//{
//  std::vector<mongo::BSONObj> objects;
//  //elem.Obj().Vals(objects);
//  mongo::BSONObjIterator i(elem.Obj());
//  while(i.more()) {
//    mongo::BSONObj t;
//    i.next().Val(t);
//    objects.push_back(t);
//  }
//  uima::ListFS listFS = cas.createListFS();
//  for(int i = objects.size() - 1; i >= 0; --i) {
//    try {
//      listFS.addFirst(toFeatureStructureAux(cas, objects[i]));
//    }
//    catch(const uima::InvalidFSTypeObjectException) {
//      outError(objects[i].getField("_type").toString() << "mongo entry could not be converted to UIMA type;");
//    }

//  }
//  return listFS;
//}

/******************************************************************************
 * Conversion:: FeatureStructure Type selection
 *****************************************************************************/

void fromFeatureStructureAux(const uima::FeatureStructure &fs, rapidjson::Document &parent, rapidjson::MemoryPoolAllocator<> &allocator)
{
  if(!fs.isValid()) {
    return;// rapidjson::Document();
  }
  initFunctionMaps(fs.getCAS());

  const uima::Type &fsType = fs.getType();
  std::string type = fsType.getName().asUTF8();

  rapidjson::Document doc(rapidjson::kObjectType);

  //  mongo::BSONObjBuilder builder;
  FromArrayTypes::const_iterator itA = fromArrayTypes.find(fsType);
  if(itA != fromArrayTypes.end()) {
    (*itA->second)(fs, FIELD_DATA, doc, allocator);
  }
  else {
    fromBasicFeatureStructure(fs, fsType, type, doc, allocator);
  }


  rapidjson::Value typeName(type, allocator);
  parent.AddMember(typeName, doc.Move(), allocator);

  //  rapidjson::StringBuffer buffer;
  //  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  //  parent.Accept(writer);
  //  std::cerr << __FILE__ << "::" << __LINE__ << ":" << buffer.GetString() << std::endl;

}

uima::FeatureStructure toFeatureStructureAux(uima::CAS &cas, const rapidjson::Document &object)
{
  initFunctionMaps(cas);
  //  if(object.Empty()) {
  //    return uima::FeatureStructure();
  //  }

  //  const UnicodeString type = UnicodeString::fromUTF8(object.getStringField(FIELD_TYPE));
  //  const uima::TypeSystem &typeSys = cas.getTypeSystem();
  //  const uima::Type &fsType = typeSys.getType(type);

  //  ToArrayTypes::const_iterator itA = toArrayTypes.find(fsType);
  //  if(itA != toArrayTypes.end()) {
  //    return (*itA->second)(cas, object.getField(FIELD_DATA));
  //  }
  //  outWarn("Converision from json to uima Types is not supported");
  throw rs::conversion::ConversionException("Json to FeatureStructure not supported!");
  //  return uima::FeatureStructure();//toBasicFeatureStructure(cas, fsType, object);
}

/******************************************************************************
 * Conversion:: FeatureStructure Type selection
 *****************************************************************************/

void fromFeatureStructure(const uima::FeatureStructure &fs, rapidjson::Document &parent)
{
  if(!fs.isValid()) {
    return;// rapidjson::Document(rapidjson::kObjectType);
  }
  initFunctionMaps(fs.getCAS());
  fromFeatureStructureAux(fs, parent, parent.GetAllocator());

  //  rapidjson::StringBuffer buffer;
  //  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  //  parent.Accept(writer);
  //  std::cerr << __FILE__ << "::" << __LINE__ << ":" << buffer.GetString() << std::endl;
}

uima::FeatureStructure toFeatureStructure(uima::CAS &cas, const rapidjson::Document &object)
{
  //  initMaps(cas);
  return toFeatureStructureAux(cas, object);
}

/******************************************************************************
 * Conversion:: Initialize Type Lookup Maps
 *****************************************************************************/

#define ADD_BASIC_TYPE(_TS_, _NAME_, _UPPER_)\
  fromBasicTypes[_TS_.getType(uima::CAS::TYPE_NAME_##_UPPER_)] = &fromFeature##_NAME_;\
//  toBasicTypes[_TS_.getType(uima::CAS::TYPE_NAME_##_UPPER_)] = &toFeature##_NAME_;

#define ADD_ARRAY_TYPE(_TS_, _NAME_, _UPPER_)\
  fromArrayTypes[_TS_.getType(uima::CAS::TYPE_NAME_##_UPPER_##_ARRAY)] = &fromArrayFS##_NAME_;\
//  toArrayTypes[_TS_.getType(uima::CAS::TYPE_NAME_##_UPPER_##_ARRAY)] = &toArrayFS##_NAME_;

#define ADD_LIST_TYPE(_TS_, _NAME_, _UPPER_)\
  fromArrayTypes[_TS_.getType(uima::CAS::TYPE_NAME_##_UPPER_##_LIST)] = &fromListFS##_NAME_;\
  fromArrayTypes[_TS_.getType(uima::CAS::TYPE_NAME_EMPTY_##_UPPER_##_LIST)] = &fromListFS##_NAME_;\
  fromArrayTypes[_TS_.getType(uima::CAS::TYPE_NAME_NON_EMPTY_##_UPPER_##_LIST)] = &fromListFS##_NAME_;\
//  toArrayTypes[_TS_.getType(uima::CAS::TYPE_NAME_##_UPPER_##_LIST)] = &toListFS##_NAME_;\
//  toArrayTypes[_TS_.getType(uima::CAS::TYPE_NAME_EMPTY_##_UPPER_##_LIST)] = &toListFS##_NAME_;\
//  toArrayTypes[_TS_.getType(uima::CAS::TYPE_NAME_NON_EMPTY_##_UPPER_##_LIST)] = &toListFS##_NAME_;

void initFunctionMaps(const uima::CAS &cas)
{
  static bool isInitialized = false;

  if(isInitialized) {
    return;
  }

  const uima::TypeSystem &ts = cas.getTypeSystem();

  ADD_BASIC_TYPE(ts, Boolean, BOOLEAN);
  ADD_BASIC_TYPE(ts, Byte, BYTE);
  ADD_BASIC_TYPE(ts, Short, SHORT);
  ADD_BASIC_TYPE(ts, Int, INTEGER);
  ADD_BASIC_TYPE(ts, Long, LONG);
  ADD_BASIC_TYPE(ts, Float, FLOAT);
  ADD_BASIC_TYPE(ts, Double, DOUBLE);
  ADD_BASIC_TYPE(ts, String, STRING);

  ADD_ARRAY_TYPE(ts, Boolean, BOOLEAN);
  ADD_ARRAY_TYPE(ts, Byte, BYTE);
  ADD_ARRAY_TYPE(ts, Short, SHORT);
  ADD_ARRAY_TYPE(ts, Int, INTEGER);
  ADD_ARRAY_TYPE(ts, Long, LONG);
  ADD_ARRAY_TYPE(ts, Float, FLOAT);
  ADD_ARRAY_TYPE(ts, Double, DOUBLE);
  ADD_ARRAY_TYPE(ts, String, STRING);
  ADD_ARRAY_TYPE(ts, FeatureStructure, FS);

  ADD_LIST_TYPE(ts, Int, INTEGER);
  ADD_LIST_TYPE(ts, Float, FLOAT);
  ADD_LIST_TYPE(ts, String, STRING);
  ADD_LIST_TYPE(ts, FeatureStructure, FS);

  isInitialized = true;
}

} // namespace conversion

} // namespace rs
