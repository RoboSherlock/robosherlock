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
#include <rs/conversion/bson.h>
#include <rs/utils/output.h>

namespace rs
{
namespace conversion
{

/******************************************************************************
 * Conversion:: Definitions
 *****************************************************************************/

#define FIELD_ID     "_id"
#define FIELD_ID_CAS "id"
#define FIELD_TYPE   "_type"
#define FIELD_DATA   "_data"
#define FIELD_PARENT "_parent"

#define ARRAY_NATIVE_LIMIT 20 // Store array with a size smaller than ARRAY_NATIVE_LIMIT as native BSON arrays else as binary

/******************************************************************************
 * Conversion:: Type Lookup Maps
 *****************************************************************************/

typedef void (*FromBasicTypeFunction)(const uima::FeatureStructure &fs, const uima::Feature &feature, const std::string &name, mongo::BSONObjBuilder &builder);
typedef void (*ToBasicTypeFunction)(uima::CAS &cas, uima::FeatureStructure &fs, const uima::Feature &feature, const mongo::BSONElement &elem);
typedef std::map<uima::Type, FromBasicTypeFunction> FromBasicTypes;
typedef std::map<uima::Type, ToBasicTypeFunction> ToBasicTypes;

static FromBasicTypes fromBasicTypes;
static ToBasicTypes toBasicTypes;

typedef void (*FromArrayTypeFunction)(const uima::FeatureStructure &fs, const std::string &fieldName, mongo::BSONObjBuilder &builder, const mongo::OID &parent);
typedef uima::FeatureStructure(*ToArrayTypeFunction)(uima::CAS &cas, const mongo::BSONElement &elem);
typedef std::map<uima::Type, FromArrayTypeFunction> FromArrayTypes;
typedef std::map<uima::Type, ToArrayTypeFunction> ToArrayTypes;

static FromArrayTypes fromArrayTypes;
static ToArrayTypes toArrayTypes;

void initMaps(const uima::CAS &cas);

/******************************************************************************
 * Conversion:: Prototypes
 *****************************************************************************/

mongo::BSONObj fromFeatureStructureAux(const uima::FeatureStructure &fs, const mongo::OID &parent);
uima::FeatureStructure toFeatureStructureAux(uima::CAS &cas, const mongo::BSONObj &object);

/******************************************************************************
 * Conversion:: Basic Feature
 *****************************************************************************/

#define DATA_FEATURE_IMPL(_TYPE_, _NAME_, _CAST_)\
void fromFeature##_NAME_(const uima::FeatureStructure &fs, const uima::Feature &feature, const std::string &name, mongo::BSONObjBuilder &builder)\
{\
  builder.append(name, (_CAST_)fs.get##_NAME_##Value(feature));\
}\
void toFeature##_NAME_(uima::CAS &cas, uima::FeatureStructure &fs, const uima::Feature &feature, const mongo::BSONElement &elem)\
{\
  _CAST_ tmp;\
  elem.Val(tmp);\
  fs.set##_NAME_##Value(feature, (_TYPE_)tmp);\
}

DATA_FEATURE_IMPL(bool, Boolean, bool)
DATA_FEATURE_IMPL(char, Byte, int)
DATA_FEATURE_IMPL(short, Short, int)
DATA_FEATURE_IMPL(int, Int, int)
DATA_FEATURE_IMPL(INT64, Long, long long int)
DATA_FEATURE_IMPL(float, Float, double)
DATA_FEATURE_IMPL(double, Double, double)

/******************************************************************************
 * Conversion:: String Feature
 *****************************************************************************/

void fromFeatureString(const uima::FeatureStructure &fs, const uima::Feature &feature, const std::string &name, mongo::BSONObjBuilder &builder)
{
  uima::UnicodeStringRef ref = fs.getStringValue(feature);
  builder.append(name, ref.asUTF8());
}

void toFeatureString(uima::CAS &cas, uima::FeatureStructure &fs, const uima::Feature &feature, const mongo::BSONElement &elem)
{
  UnicodeString value = UnicodeString::fromUTF8(elem.String());
  fs.setStringValue(feature, value);
}

/******************************************************************************
 * Conversion:: FS Feature
 *****************************************************************************/

void fromFeatureFeatureStructure(const uima::FeatureStructure &fs, const uima::Feature &feature, const std::string &name, mongo::BSONObjBuilder &builder)
{
  mongo::BSONElement oid;
  builder.asTempObj().getObjectID(oid);
  builder.append(name, fromFeatureStructureAux(fs.getFSValue(feature), oid.OID()));
}

void toFeatureFeatureStructure(uima::CAS &cas, uima::FeatureStructure &fs, const uima::Feature &feature, const mongo::BSONElement &elem)
{
  uima::FeatureStructure subFS = toFeatureStructureAux(cas, elem.Obj());
  if(subFS.isValid())
  {
    fs.setFSValue(feature, subFS);
  }
}

/******************************************************************************
 * Conversion:: Feature Type selection
 *****************************************************************************/

void fromFeature(const uima::FeatureStructure &fs, const uima::Feature &feature, mongo::BSONObjBuilder &builder)
{
  uima::Type featureType;
  feature.getRangeType(featureType);
  const uima::UnicodeStringRef featureName = feature.getName();
  const std::string &name = featureName.asUTF8();

  FromBasicTypes::const_iterator itB = fromBasicTypes.find(featureType);
  if(itB != fromBasicTypes.end())
  {
    (*itB->second)(fs, feature, name, builder);
    return;
  }

  FromArrayTypes::const_iterator itA = fromArrayTypes.find(featureType);
  if(itA != fromArrayTypes.end())
  {
    const uima::FeatureStructure &subFS = fs.getFSValue(feature);
    if(!subFS.isValid())
    {
      builder.append(name, mongo::BSONObj());
    }
    else
    {
      mongo::BSONElement oid;
      builder.asTempObj().getObjectID(oid);
      (*itA->second)(subFS, name, builder, oid.OID());
    }
    return;
  }

  fromFeatureFeatureStructure(fs, feature, name, builder);
}

void toFeature(uima::CAS &cas, uima::FeatureStructure fs, const uima::Type &fsType, const mongo::BSONElement &elem)
{
  const UnicodeString name = UnicodeString::fromUTF8(elem.fieldName());
  uima::Feature feature = fsType.getFeatureByBaseName(name);
  uima::Type featureType;
  feature.getRangeType(featureType);

  ToBasicTypes::const_iterator itB = toBasicTypes.find(featureType);
  if(itB != toBasicTypes.end())
  {
    (*itB->second)(cas, fs, feature, elem);
    return;
  }

  ToArrayTypes::const_iterator itA = toArrayTypes.find(featureType);
  if(itA != toArrayTypes.end())
  {
    if(!elem.isABSONObj() || !elem.Obj().isEmpty())
    {
      fs.setFSValue(feature, (*itA->second)(cas, elem));
    }
    return;
  }

  toFeatureFeatureStructure(cas, fs, feature, elem);
}

/******************************************************************************
 * Conversion:: Basic Feature Structure
 *****************************************************************************/

void fromBasicFeatureStructure(const uima::FeatureStructure &fs, const uima::Type &fsType, const std::string &type, mongo::BSONObjBuilder &builder, const mongo::OID &parent)
{
  std::vector<uima::Feature> features;
  fsType.getAppropriateFeatures(features);

  uima::Feature idFeature;
  std::string id;
  try
  {
    idFeature = fsType.getFeatureByBaseName(FIELD_ID_CAS);
    if(fsType.isAppropriateFeature(idFeature))
    {
      id = fs.getStringValue(idFeature).asUTF8();
    }
  }
  catch(const uima::InvalidFSTypeObjectException)
  {
  }

  if(!id.empty())
  {
    builder.append(FIELD_ID, mongo::OID(id));
  }
  else
  {
    builder.genOID();
  }
  builder.append(FIELD_PARENT, parent);
  builder.append(FIELD_TYPE, type);

  for(size_t i = 0; i < features.size(); ++i)
  {
    const uima::Feature &feature = features[i];
    if(feature != idFeature)
    {
      fromFeature(fs, feature, builder);
    }
  }
}

uima::FeatureStructure toBasicFeatureStructure(uima::CAS &cas, const uima::Type &fsType, const mongo::BSONObj &object)
{
  uima::FeatureStructure newFS = cas.createFS(fsType);

  try
  {
    const uima::Feature &idFeature = fsType.getFeatureByBaseName(FIELD_ID_CAS);
    if(fsType.isAppropriateFeature(idFeature))
    {
      mongo::BSONElement elem;
      if(object.getObjectID(elem))
      {
        newFS.setStringValue(idFeature, UnicodeString::fromUTF8(elem.OID().toString()));
      }
    }
  }
  catch(const uima::InvalidFSTypeObjectException)
  {
  }

  std::vector<mongo::BSONElement> elems;
  object.elems(elems);

  for(size_t i = 0; i < elems.size(); ++i)
  {
    const mongo::BSONElement &elem = elems[i];
    if(elem.fieldName()[0] != '_')
    {
      toFeature(cas, newFS, fsType, elem);
    }
  }
  return newFS;
}

/******************************************************************************
 * Conversion:: Basic Array Feature Structure with casting
 *****************************************************************************/

#define DATA_ARRAY_CAST_IMPL(_TYPE_, _NAME_, _CAST_)\
void fromArrayFS##_NAME_##Native(const uima::_NAME_##ArrayFS &arrayFS, const size_t &size, const std::string &fieldName, mongo::BSONObjBuilder &builder, const mongo::OID &parent)\
{\
  std::vector<_CAST_> data(size);\
  if(size)\
  {\
    _TYPE_ *rawData = new _TYPE_[size];\
    arrayFS.copyToArray(0, size, rawData, 0);\
    for(size_t i = 0; i < size; ++i)\
    {\
      data[i] = (_CAST_)rawData[i];\
    }\
    delete[] rawData;\
  }\
  builder.append(fieldName, data);\
}\
uima::FeatureStructure toArrayFS##_NAME_##Native(uima::CAS &cas, const mongo::BSONElement &elem)\
{\
  std::vector<_CAST_> data;\
  elem.Obj().Vals(data);\
  uima::_NAME_##ArrayFS arrayFS = cas.create##_NAME_##ArrayFS(data.size());\
  if(!data.empty())\
  {\
    _TYPE_ *rawData = new _TYPE_[data.size()];\
    for(size_t i = 0; i < data.size(); ++i)\
    {\
      rawData[i] = (_TYPE_)data[i];\
    }\
    arrayFS.copyFromArray(rawData, 0, data.size(), 0);\
    delete[] rawData;\
  }\
  return arrayFS;\
}

DATA_ARRAY_CAST_IMPL(bool, Boolean, bool)
DATA_ARRAY_CAST_IMPL(char, Byte, int)
DATA_ARRAY_CAST_IMPL(short, Short, int)
DATA_ARRAY_CAST_IMPL(float, Float, double)

/******************************************************************************
 * Conversion:: Basic Array Feature Structure as native types
 *****************************************************************************/

#define DATA_ARRAY_NATIVE_IMPL(_TYPE_, _NAME_, _CAST_)\
void fromArrayFS##_NAME_##Native(const uima::_NAME_##ArrayFS &arrayFS, const size_t &size, const std::string &fieldName, mongo::BSONObjBuilder &builder, const mongo::OID &parent)\
{\
  std::vector<_CAST_> data(size);\
  if(size)\
  {\
    arrayFS.copyToArray(0, size, (_TYPE_*)data.data(), 0);\
  }\
  builder.append(fieldName, data);\
}\
uima::FeatureStructure toArrayFS##_NAME_##Native(uima::CAS &cas, const mongo::BSONElement &elem)\
{\
  std::vector<_CAST_> data;\
  elem.Obj().Vals(data);\
  uima::_NAME_##ArrayFS arrayFS = cas.create##_NAME_##ArrayFS(data.size());\
  if(!data.empty())\
  {\
    arrayFS.copyFromArray((_TYPE_*)data.data(), 0, data.size(), 0);\
  }\
  return arrayFS;\
}

DATA_ARRAY_NATIVE_IMPL(int, Int, int)
DATA_ARRAY_NATIVE_IMPL(INT64, Long, long long int)
DATA_ARRAY_NATIVE_IMPL(double, Double, double)

/******************************************************************************
 * Conversion:: Basic Array Feature Structure as binary
 *****************************************************************************/

#define DATA_ARRAY_BIN_IMPL(_TYPE_, _NAME_)\
void fromArrayFS##_NAME_##Binary(const uima::_NAME_##ArrayFS &arrayFS, const size_t &size, const std::string &fieldName, mongo::BSONObjBuilder &builder, const mongo::OID &parent)\
{\
  _TYPE_ *rawData = NULL;\
  if(size)\
  {\
    rawData = new _TYPE_[size];\
    arrayFS.copyToArray(0, size, rawData, 0);\
  }\
  builder.appendBinData(fieldName, size * sizeof(_TYPE_), mongo::BinDataGeneral, rawData);\
  if(rawData)\
  {\
    delete[] rawData;\
  }\
}\
uima::FeatureStructure toArrayFS##_NAME_##Binary(uima::CAS &cas, const mongo::BSONElement &elem)\
{\
  int len = 0;\
  const _TYPE_ *rawData = (const _TYPE_*)elem.binData(len);\
  size_t size = len / sizeof(_TYPE_);\
  uima::_NAME_##ArrayFS arrayFS = cas.create##_NAME_##ArrayFS(size);\
  if(size)\
  {\
    arrayFS.copyFromArray(rawData, 0, size, 0);\
  }\
  return arrayFS;\
}

DATA_ARRAY_BIN_IMPL(bool, Boolean)
DATA_ARRAY_BIN_IMPL(char, Byte)
DATA_ARRAY_BIN_IMPL(short, Short)
DATA_ARRAY_BIN_IMPL(int, Int)
DATA_ARRAY_BIN_IMPL(INT64, Long)
DATA_ARRAY_BIN_IMPL(float, Float)
DATA_ARRAY_BIN_IMPL(double, Double)

/******************************************************************************
 * Conversion:: Basic Array Feature Structure implementation selection
 *****************************************************************************/

#define DATA_ARRAY_IMPL(_NAME_)\
void fromArrayFS##_NAME_(const uima::FeatureStructure &fs, const std::string &fieldName, mongo::BSONObjBuilder &builder, const mongo::OID &parent)\
{\
  uima::_NAME_##ArrayFS arrayFS(fs);\
  size_t size = 0;\
  if(arrayFS.isValid())\
  {\
    size = arrayFS.size();\
  }\
  if((int64_t)size < ARRAY_NATIVE_LIMIT)\
  {\
    fromArrayFS##_NAME_##Native(arrayFS, size, fieldName, builder, parent);\
  }\
  else\
  {\
    fromArrayFS##_NAME_##Binary(arrayFS, size, fieldName, builder, parent);\
  }\
}\
uima::FeatureStructure toArrayFS##_NAME_(uima::CAS &cas, const mongo::BSONElement &elem)\
{\
  switch(elem.type())\
  {\
  case mongo::BinData:\
    return toArrayFS##_NAME_##Binary(cas, elem);\
  case mongo::Array:\
    return toArrayFS##_NAME_##Native(cas, elem);\
  default:\
    outError("unsupported element type!");\
    return uima::FeatureStructure();\
  }\
}

DATA_ARRAY_IMPL(Boolean)
DATA_ARRAY_IMPL(Byte)
DATA_ARRAY_IMPL(Short)
DATA_ARRAY_IMPL(Int)
DATA_ARRAY_IMPL(Long)
DATA_ARRAY_IMPL(Float)
DATA_ARRAY_IMPL(Double)

/******************************************************************************
 * Conversion:: String Array Feature Structure
 *****************************************************************************/

void fromArrayFSString(const uima::FeatureStructure &fs, const std::string &fieldName, mongo::BSONObjBuilder &builder, const mongo::OID &parent)
{
  uima::StringArrayFS arrayFS(fs);
  size_t size = 0;
  if(arrayFS.isValid())
  {
    size = arrayFS.size();
  }
  std::vector<std::string> data(size);
  for(size_t i = 0; i < size; ++i)
  {
    uima::UnicodeStringRef ref = arrayFS.get(i);
    data[i] = ref.asUTF8();
  }
  builder.append(fieldName, data);
}

uima::FeatureStructure toArrayFSString(uima::CAS &cas, const mongo::BSONElement &elem)
{
  std::vector<std::string> data;
  elem.Obj().Vals(data);
  uima::StringArrayFS arrayFS = cas.createStringArrayFS(data.size());
  for(size_t i = 0; i < data.size(); ++i)
  {
    UnicodeString str = UnicodeString::fromUTF8(data[i]);
    arrayFS.set(i, uima::UnicodeStringRef(str));
  }
  return arrayFS;
}

/******************************************************************************
 * Conversion:: FS Array Feature Structure Implementation
 *****************************************************************************/

void fromArrayFSFeatureStructure(const uima::FeatureStructure &fs, const std::string &fieldName, mongo::BSONObjBuilder &builder, const mongo::OID &parent)
{
  uima::ArrayFS arrayFS(fs);
  size_t size = 0;
  if(arrayFS.isValid())
  {
    size = arrayFS.size();
  }
  std::vector<mongo::BSONObj> data(size);
  for(size_t i = 0; i < size; ++i)
  {
    data[i] = fromFeatureStructureAux(arrayFS.get(i), parent);
  }
  builder.append(fieldName, data);
}

uima::FeatureStructure toArrayFSFeatureStructure(uima::CAS &cas, const mongo::BSONElement &elem)
{
  std::vector<mongo::BSONObj> objects;
  elem.Obj().Vals(objects);
  uima::ArrayFS arrayFS = cas.createArrayFS(objects.size());
  for(size_t i = 0; i < objects.size(); ++i)
  {
    arrayFS.set(i, toFeatureStructureAux(cas, objects[i]));
  }
  return arrayFS;
}

/******************************************************************************
 * Conversion:: Basic List Feature Structure with casting
 *****************************************************************************/

#define DATA_LIST_CAST_IMPL(_TYPE_, _NAME_, _CAST_)\
void fromListFS##_NAME_##Native(uima::_NAME_##ListFS &listFS, const size_t &size, const std::string &fieldName, mongo::BSONObjBuilder &builder, const mongo::OID &parent)\
{\
  std::vector<_CAST_> data(size);\
  for(size_t i = 0; i < size; ++i)\
  {\
    data[i] = (_CAST_)listFS.getHead();\
    listFS.moveToNext();\
  }\
  builder.append(fieldName, data);\
}\
uima::FeatureStructure toListFS##_NAME_##Native(uima::CAS &cas, const mongo::BSONElement &elem)\
{\
  std::vector<_CAST_> data;\
  elem.Obj().Vals(data);\
  uima::_NAME_##ListFS listFS = cas.create##_NAME_##ListFS();\
  for(int i = data.size() - 1; i >= 0; --i)\
  {\
    listFS.addFirst((_TYPE_)data[i]);\
  }\
  return listFS;\
}

DATA_LIST_CAST_IMPL(int, Int, int)
DATA_LIST_CAST_IMPL(float, Float, double)

/******************************************************************************
 * Conversion:: Basic List Feature Structure as binary
 *****************************************************************************/

#define DATA_LIST_BIN_IMPL(_TYPE_, _NAME_)\
void fromListFS##_NAME_##Binary(uima::_NAME_##ListFS &listFS, const size_t &size, const std::string &fieldName, mongo::BSONObjBuilder &builder, const mongo::OID &parent)\
{\
  _TYPE_ *rawData = NULL;\
  if(size)\
  {\
    rawData = new _TYPE_[size];\
    for(size_t i = 0; i < size; ++i)\
    {\
      rawData[i] = listFS.getHead();\
      listFS.moveToNext();\
    }\
  }\
  builder.appendBinData(fieldName, size * sizeof(_TYPE_), mongo::BinDataGeneral, rawData);\
  if(rawData)\
  {\
    delete[] rawData;\
  }\
}\
uima::FeatureStructure toListFS##_NAME_##Binary(uima::CAS &cas, const mongo::BSONElement &elem)\
{\
  int len = 0;\
  const _TYPE_ *rawData = (const _TYPE_*)elem.binData(len);\
  size_t size = len / sizeof(_TYPE_);\
  uima::_NAME_##ListFS listFS = cas.create##_NAME_##ListFS();\
  for(int i = size - 1; i >= 0; --i)\
  {\
    listFS.addFirst(rawData[i]);\
  }\
  return listFS;\
}

DATA_LIST_BIN_IMPL(int, Int)
DATA_LIST_BIN_IMPL(float, Float)

/******************************************************************************
 * Conversion:: Basic List Feature Structure implementation selection
 *****************************************************************************/

#define DATA_LIST_IMPL(_NAME_)\
void fromListFS##_NAME_(const uima::FeatureStructure &fs, const std::string &fieldName, mongo::BSONObjBuilder &builder, const mongo::OID &parent)\
{\
  uima::_NAME_##ListFS listFS(fs);\
  size_t size = 0;\
  if(listFS.isValid())\
  {\
    size = listFS.getLength();\
  }\
  if((int64_t)size < ARRAY_NATIVE_LIMIT)\
  {\
    fromListFS##_NAME_##Native(listFS, size, fieldName, builder, parent);\
  }\
  else\
  {\
    fromListFS##_NAME_##Binary(listFS, size, fieldName, builder, parent);\
  }\
}\
uima::FeatureStructure toListFS##_NAME_(uima::CAS &cas, const mongo::BSONElement &elem)\
{\
  switch(elem.type())\
  {\
  case mongo::BinData:\
    return toListFS##_NAME_##Binary(cas, elem);\
  case mongo::Array:\
    return toListFS##_NAME_##Native(cas, elem);\
  default:\
    outError("unsupported element type!");\
    return uima::FeatureStructure();\
  }\
}

DATA_LIST_IMPL(Int)
DATA_LIST_IMPL(Float)

/******************************************************************************
 * Conversion:: String List Feature Structure
 *****************************************************************************/

void fromListFSString(const uima::FeatureStructure &fs, const std::string &fieldName, mongo::BSONObjBuilder &builder, const mongo::OID &parent)
{
  uima::StringListFS listFS(fs);
  size_t size = 0;
  if(listFS.isValid())
  {
    size = listFS.getLength();
  }
  std::vector<std::string> data(size);
  for(size_t i = 0; i < size; ++i)
  {
    uima::UnicodeStringRef ref = listFS.getHead();
    data[i] = ref.asUTF8();
    listFS.moveToNext();
  }
  builder.append(fieldName, data);
}

uima::FeatureStructure toListFSString(uima::CAS &cas, const mongo::BSONElement &elem)
{
  std::vector<std::string> data;
  elem.Obj().Vals(data);
  uima::StringListFS listFS = cas.createStringListFS();
  for(int i = data.size() - 1; i >= 0; --i)
  {
    UnicodeString str = UnicodeString::fromUTF8(data[i]);
    listFS.addFirst(uima::UnicodeStringRef(str));
  }
  return listFS;
}

/******************************************************************************
 * Conversion:: FS List Feature Structure
 *****************************************************************************/

void fromListFSFeatureStructure(const uima::FeatureStructure &fs, const std::string &fieldName, mongo::BSONObjBuilder &builder, const mongo::OID &parent)
{
  uima::ListFS listFS(fs);
  size_t size = 0;
  if(listFS.isValid())
  {
    size = listFS.getLength();
  }
  std::vector<mongo::BSONObj> data(size);
  for(size_t i = 0; i < size; ++i)
  {
    data[i] = fromFeatureStructureAux(listFS.getHead(), parent);
    listFS.moveToNext();
  }
  builder.append(fieldName, data);
}

uima::FeatureStructure toListFSFeatureStructure(uima::CAS &cas, const mongo::BSONElement &elem)
{
  std::vector<mongo::BSONObj> objects;
  elem.Obj().Vals(objects);
  uima::ListFS listFS = cas.createListFS();
  for(int i = objects.size() - 1; i >= 0; --i)
  {
    listFS.addFirst(toFeatureStructureAux(cas, objects[i]));
  }
  return listFS;
}

/******************************************************************************
 * Conversion:: FeatureStructure Type selection
 *****************************************************************************/

mongo::BSONObj fromFeatureStructureAux(const uima::FeatureStructure &fs, const mongo::OID &parent)
{
  if(!fs.isValid())
  {
    return mongo::BSONObj();
  }
  initMaps(fs.getCAS());

  const uima::Type &fsType = fs.getType();
  const std::string type = fsType.getName().asUTF8();

  mongo::BSONObjBuilder builder;

  FromArrayTypes::const_iterator itA = fromArrayTypes.find(fsType);
  if(itA != fromArrayTypes.end())
  {
    builder.genOID();
    builder.append(FIELD_TYPE, type);
    builder.append(FIELD_PARENT, parent);
    (*itA->second)(fs, FIELD_DATA, builder, parent);
  }
  else
  {
    fromBasicFeatureStructure(fs, fsType, type, builder, parent);
  }
  return builder.obj();
}

uima::FeatureStructure toFeatureStructureAux(uima::CAS &cas, const mongo::BSONObj &object)
{
  initMaps(cas);
  if(object.isEmpty())
  {
    return uima::FeatureStructure();
  }

  const UnicodeString type = UnicodeString::fromUTF8(object.getStringField(FIELD_TYPE));
  const uima::TypeSystem &typeSys = cas.getTypeSystem();
  const uima::Type &fsType = typeSys.getType(type);

  ToArrayTypes::const_iterator itA = toArrayTypes.find(fsType);
  if(itA != toArrayTypes.end())
  {
    return (*itA->second)(cas, object.getField(FIELD_DATA));
  }
  return toBasicFeatureStructure(cas, fsType, object);
}

/******************************************************************************
 * Conversion:: FeatureStructure Type selection
 *****************************************************************************/

mongo::BSONObj fromFeatureStructure(const uima::FeatureStructure &fs, const mongo::OID &parent)
{
  if(!fs.isValid())
  {
    return mongo::BSONObj();
  }
  initMaps(fs.getCAS());

  return fromFeatureStructureAux(fs, parent);
}

uima::FeatureStructure toFeatureStructure(uima::CAS &cas, const mongo::BSONObj &object)
{
  initMaps(cas);
  return toFeatureStructureAux(cas, object);
}

/******************************************************************************
 * Conversion:: Initialize Type Lookup Maps
 *****************************************************************************/

#define ADD_BASIC_TYPE(_TS_, _NAME_, _UPPER_)\
  fromBasicTypes[_TS_.getType(uima::CAS::TYPE_NAME_##_UPPER_)] = &fromFeature##_NAME_;\
  toBasicTypes[_TS_.getType(uima::CAS::TYPE_NAME_##_UPPER_)] = &toFeature##_NAME_;

#define ADD_ARRAY_TYPE(_TS_, _NAME_, _UPPER_)\
  fromArrayTypes[_TS_.getType(uima::CAS::TYPE_NAME_##_UPPER_##_ARRAY)] = &fromArrayFS##_NAME_;\
  toArrayTypes[_TS_.getType(uima::CAS::TYPE_NAME_##_UPPER_##_ARRAY)] = &toArrayFS##_NAME_;

#define ADD_LIST_TYPE(_TS_, _NAME_, _UPPER_)\
  fromArrayTypes[_TS_.getType(uima::CAS::TYPE_NAME_##_UPPER_##_LIST)] = &fromListFS##_NAME_;\
  fromArrayTypes[_TS_.getType(uima::CAS::TYPE_NAME_EMPTY_##_UPPER_##_LIST)] = &fromListFS##_NAME_;\
  fromArrayTypes[_TS_.getType(uima::CAS::TYPE_NAME_NON_EMPTY_##_UPPER_##_LIST)] = &fromListFS##_NAME_;\
  toArrayTypes[_TS_.getType(uima::CAS::TYPE_NAME_##_UPPER_##_LIST)] = &toListFS##_NAME_;\
  toArrayTypes[_TS_.getType(uima::CAS::TYPE_NAME_EMPTY_##_UPPER_##_LIST)] = &toListFS##_NAME_;\
  toArrayTypes[_TS_.getType(uima::CAS::TYPE_NAME_NON_EMPTY_##_UPPER_##_LIST)] = &toListFS##_NAME_;

void initMaps(const uima::CAS &cas)
{
  static bool isInitialized = false;

  if(isInitialized)
  {
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
