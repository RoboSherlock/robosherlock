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

// SYSTEM
#include <dirent.h>
#include <sys/stat.h>

// UNICODE STRING
#include <unicode/unistr.h>

// RS
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/io/Storage.h>
#include <rs/conversion/bson.h>

//#undef OUT_LEVEL
//#define OUT_LEVEL OUT_LEVEL_DEBUG

using namespace rs;

/******************************************************************************
 * Defines
 *****************************************************************************/
#define DB_CAS "cas"
#define DB_SCRIPTS "system.js"
#define DB_CAS_TIME "_timestamp"
#define SCRIPT_EXT ".js"

/******************************************************************************
 * Storage
 *****************************************************************************/

Storage::Storage()
  : dbHost(DB_HOST)
  , dbName(DB_NAME)
  , dbBase(dbName + ".")
  , dbCAS(dbBase + DB_CAS)
  , dbScripts(dbBase + DB_SCRIPTS)
  , first(true)
{
}

Storage::Storage(const Storage& other)
{
  this->operator=(other);
}

Storage::Storage(const std::string& dbHost, const std::string& dbName, const bool clear, const bool setupScripts)
  : dbHost(dbHost), dbName(dbName), dbBase(dbName + "."), dbCAS(dbBase + DB_CAS), dbScripts(dbBase + DB_SCRIPTS)
{
  db.connect(dbHost);

  if (clear)
  {
    std::list<std::string> names = db.getCollectionNames(dbName);
    std::list<std::string>::const_iterator it = names.begin(), end = names.end();
    for (; it != end; ++it)
    {
      if (it->find(dbBase + "system") != 0)
      {
        outDebug("removing collection '" << *it << "' from mongoDB...");
        db.dropCollection(dbBase + *it);
      }
    }
  }
}

Storage::~Storage()
{
}

Storage& Storage::operator=(const Storage& other)
{
  dbHost = other.dbHost;
  dbName = other.dbName;
  dbBase = other.dbBase;
  dbCAS = other.dbCAS;
  dbScripts = other.dbScripts;
  storeViews = other.storeViews;
  loadViews = other.loadViews;
  db.connect(dbHost);
  return *this;
}

bool Storage::readArrayFS(uima::FeatureStructure fs, mongo::BSONObjBuilder& builderCAS, const mongo::OID& casOID,
                          const std::string& sofaId, const std::string& dbCollection)
{
  uima::ArrayFS array;
  try
  {
    array = uima::ArrayFS(fs);
  }
  catch (...)
  {
    return false;
  }

  std::vector<mongo::BSONObj> objects(array.size());
  std::vector<mongo::OID> objectIds(array.size());

  for (size_t i = 0; i < array.size(); ++i)
  {
    mongo::BSONObj object = rs::conversion::fromFeatureStructure(array.get(i), casOID);
    objects[i] = object;

    mongo::BSONElement elem;
    object.getObjectID(elem);
    objectIds[i] = elem.OID();
  }
  builderCAS.append(sofaId, objectIds);

  outDebug("storing sofas to " << dbCollection << ".");
  db.insert(dbCollection, objects);
  return true;
}

bool Storage::readFS(uima::FeatureStructure fs, mongo::BSONObjBuilder& builderCAS, const mongo::OID& casOID,
                     const std::string& sofaId, const std::string& dbCollection)
{
  mongo::BSONObj object = rs::conversion::fromFeatureStructure(fs, casOID);

  if (first)
  {
    mongo::BSONElement elem;
    object.getObjectID(elem);
    builderCAS.append(sofaId, elem.OID());
    if (sofaId.compare(0, std::string("camera_info").length(), "camera_info") == 0)
    {
      camInfoOIDs[sofaId] = elem.OID();
    }
    outDebug("storing sofas to " << dbCollection << ".");
    db.insert(dbCollection, object);
    return true;
  }

  if (sofaId.compare(0, std::string("camera_info").length(), "camera_info") == 0)
  {
    builderCAS.append(sofaId, camInfoOIDs[sofaId]);
  }
  else
  {
    mongo::BSONElement elem;
    object.getObjectID(elem);
    builderCAS.append(sofaId, elem.OID());
    outDebug("storing sofas to " << dbCollection << ".");
    db.insert(dbCollection, object);
  }

  return true;
}

void Storage::loadView(uima::CAS& cas, const mongo::BSONElement& elem)
{
  std::string field_name = elem.fieldName();
  std::string viewName = field_name;
  if (viewName.find(".") == std::string::npos)
    viewName = "cam0." + viewName;

  uima::CAS* view = nullptr;
  try
  {
    outDebug("try to get view " << viewName);
    view = cas.getView(UnicodeString::fromUTF8(viewName));
  }
  catch (...)
  {
    outDebug("create view " << viewName);
    view = cas.createView(UnicodeString::fromUTF8(viewName));
  }

  outDebug("getting referenced object...");
  uima::FeatureStructure fs;
  if (elem.isSimpleType())
  {
    fs = loadFS(view, field_name, elem.OID());
  }
  else
  {
    const std::vector<mongo::BSONElement>& elems = elem.Array();
    std::vector<mongo::OID> ids(elems.size());
    for (size_t i = 0; i < elems.size(); ++i)
    {
      ids[i] = elems[i].OID();
    }

    fs = loadArrayFS(view, viewName, ids);
  }

  const std::string mime = "application/x-" + viewName;
  view->setSofaDataArray(fs, UnicodeString::fromUTF8(mime));
}

uima::FeatureStructure Storage::loadArrayFS(uima::CAS* view, const std::string& viewName,
                                            const std::vector<mongo::OID>& ids)
{
  mongo::Query query(BSON("_id" << BSON("$in" << ids)));
  std::auto_ptr<mongo::DBClientCursor> cursor = db.query(dbBase + viewName, query, ids.size());

  uima::ArrayFS array = view->createArrayFS(ids.size());
  size_t i = 0;
  while (cursor->more())
  {
    array.set(i++, rs::conversion::to(*view, cursor->next()));
  }

  outAssert(i == ids.size(), "Returned objects (" << i << ") do not match stored OIDs (" << ids.size() << ").");
  return array;
}

uima::FeatureStructure Storage::loadFS(uima::CAS* view, const std::string& viewName, const mongo::OID& id)
{
  mongo::Query query(BSON("_id" << id));
  std::auto_ptr<mongo::DBClientCursor> cursor = db.query(dbBase + viewName, query, 1);

  if (cursor->more())
  {
    return rs::conversion::to(*view, cursor->next());
  }
  return uima::FeatureStructure();
}

void Storage::removeView(const mongo::BSONElement& elem)
{
  const std::string& viewName = elem.fieldName();
  const std::string dbCollection = dbBase + viewName;

  if (elem.isSimpleType())
  {
    db.remove(dbCollection, mongo::Query(BSON("_id" << elem.OID())));
  }
  else
  {
    const std::vector<mongo::BSONElement>& elems = elem.Array();
    std::vector<mongo::OID> ids(elems.size());
    for (size_t i = 0; i < elems.size(); ++i)
    {
      ids[i] = elems[i].OID();
    }

    db.remove(dbCollection, mongo::Query(BSON("_id" << BSON("$in" << ids))));
  }
}

void Storage::enableViewStoring(const std::string& viewName, const bool enable)
{
  storeViews[viewName] = enable;
}

void Storage::enableViewLoading(const std::string& viewName, const bool enable)
{
  loadViews[viewName] = enable;
}

void Storage::getScenes(std::vector<uint64_t>& timestamps)
{
  timestamps.clear();

  std::auto_ptr<mongo::DBClientCursor> cursor = db.query(dbCAS, mongo::Query());

  while (cursor->more())
  {
    const mongo::BSONObj& object = cursor->next();
    std::vector<mongo::BSONElement> elems;
    object.elems(elems);

    for (size_t i = 0; i < elems.size(); ++i)
    {
      const mongo::BSONElement& elem = elems[i];
      const std::string& name = elem.fieldName();
      if (name == DB_CAS_TIME)
      {
        timestamps.push_back(elem.Long());
      }
    }
  }
}

bool Storage::storeScene(uima::CAS& cas, const uint64_t& timestamp, bool multi_cam)
{
  outDebug("converting CAS Views to BSON and writing to mongoDB...");
  mongo::BSONObjBuilder builder;
  builder.genOID();
  builder.append(DB_CAS_TIME, (long long)timestamp);
  mongo::BSONElement elemOID;
  builder.asTempObj().getObjectID(elemOID);
  const mongo::OID& casOID = elemOID.OID();

  uima::FSIterator it = cas.getSofaIterator();
  for (; it.isValid(); it.moveToNext())
  {
    uima::SofaFS sofa(it.get());

    std::string sofaId = sofa.getSofaID().asUTF8();

    if (!storeViews[sofaId])
    {
      outInfo("skipping sofa \"" << sofaId << "\".");
      continue;
    }

    std::string sofaName = sofaId;
    if (!multi_cam)
      sofaId = sofaId.substr(sofaId.find(".") + 1);


    const std::string dbCollection = dbBase + sofaId;

    outDebug("converting sofa \"" << sofaId << "\".");

    uima::FeatureStructure fs = sofa.getLocalFSData();

    readArrayFS(fs, builder, casOID, sofaId, dbCollection) || readFS(fs, builder, casOID, sofaId, dbCollection);
  }

  outDebug("storing CAS information to " << DB_CAS << ".");
  db.insert(dbCAS, builder.obj());
  if (first)
  {
    first = false;
  }
  return true;
}

bool Storage::removeScene(const uint64_t& timestamp, bool multi_cam)
{
  mongo::Query query(BSON(DB_CAS_TIME << (long long)timestamp));
  std::auto_ptr<mongo::DBClientCursor> cursor = db.query(dbCAS, query, 1);

  if (cursor->more())
  {
    const mongo::BSONObj& object = cursor->next();
    std::vector<mongo::BSONElement> elems;
    object.elems(elems);

    for (size_t i = 0; i < elems.size(); ++i)
    {
      const mongo::BSONElement& elem = elems[i];
      std::string name = elem.fieldName();
      if (name[0] != '_')
      {
        if (!multi_cam)
          name = name.substr(name.find(".") + 1);
        outDebug("removing view: " << name);

        removeView(elem);
      }
    }
    db.remove(dbCAS, query);
  }
  else
  {
    return false;
  }
  return true;
}

bool Storage::updateScene(uima::CAS& cas, const uint64_t& timestamp, bool multi_cam)
{
  removeScene(timestamp, multi_cam);
  return storeScene(cas, timestamp,multi_cam);
}

bool Storage::loadScene(uima::CAS& cas, const uint64_t& timestamp)
{
  const bool loadAll = loadViews.empty();
  mongo::Query query(BSON(DB_CAS_TIME << (long long)timestamp));
  std::auto_ptr<mongo::DBClientCursor> cursor = db.query(dbCAS, query, 1);

  if (cursor->more())
  {
    const mongo::BSONObj& object = cursor->next();
    std::vector<mongo::BSONElement> elems;
    object.elems(elems);

    for (size_t i = 0; i < elems.size(); ++i)
    {
      const mongo::BSONElement& elem = elems[i];
      const std::string& name = elem.fieldName();
      if ((loadAll && name[0] != '_') || (!loadAll && loadViews[name]))
      {
        outDebug("loading view: " << name);
        loadView(cas, elem);
      }
    }
  }
  else
  {
    return false;
  }
  return true;
}

void Storage::removeCollection(const std::string& collection)
{
  outDebug("removing collection '" << collection << "' from mongoDB...");
  const std::string dbCollection = dbBase + collection;
  db.dropCollection(dbCollection);
}

void Storage::storeCollection(uima::CAS& cas, const std::string& view, const std::string& collection, int cam_id)
{
  outDebug("storing CAS View as Collection to mongoDB...");
  std::stringstream view_name;
  view_name<<"cam"<<cam_id<<"."<<view;
  mongo::BSONObjBuilder builder;
  const mongo::OID casOID;
  const std::string dbCollection = dbBase + collection;

  db.remove(dbCollection, mongo::Query());
  try
  {
    uima::CAS* _view = cas.getView(UnicodeString::fromUTF8(view_name.str()));
    uima::FeatureStructure fs = _view->getSofaDataArray();
    readArrayFS(fs, builder, casOID, view_name.str(), dbCollection) || readFS(fs, builder, casOID, view_name.str(), dbCollection);
  }
  catch (uima::CASException e)
  {
    outInfo("No Sofa named: " << view_name.str());
  }
}

void Storage::loadCollection(uima::CAS& cas, const std::string& view, const std::string& collection, int cam_id)
{
  const std::string dbCollection = dbBase + collection;
  mongo::Query query;
  std::auto_ptr<mongo::DBClientCursor> cursor = db.query(dbCollection, query);
  std::vector<mongo::OID> ids;

  std::stringstream view_name;
  view_name<<"cam"<<cam_id<<"."<<view;
  while (cursor->more())
  {
    const mongo::BSONObj& object = cursor->next();
    mongo::BSONElement elem;
    object.getObjectID(elem);
    ids.push_back(elem.OID());
  }

  uima::CAS* _view = nullptr;
  try
  {
    outDebug("try to get view " << view);
    _view = cas.getView(UnicodeString::fromUTF8(view_name.str()));
  }
  catch (...)
  {
    outDebug("create view " << view);
    _view = cas.createView(UnicodeString::fromUTF8(view_name.str()));
  }

  outDebug("getting referenced object...");
  uima::FeatureStructure fs = loadArrayFS(_view, collection, ids);

  const std::string mime = "application/x-" + view;
  _view->setSofaDataArray(fs, UnicodeString::fromUTF8(mime));
}

std::vector<rs::ObjectHypothesis> Storage::getClusters(uima::CAS& cas, const std::string& collection,
                                                       std::vector<std::string> ids)
{
  const std::string dbCollection = dbBase + collection;
  std::auto_ptr<mongo::DBClientCursor> cursor = db.query(dbCollection, mongo::Query());

  std::vector<rs::ObjectHypothesis> clusters;

  while (cursor->more())
  {
    const mongo::BSONObj& object = cursor->next();
    std::vector<mongo::BSONElement> elements = object["identifiables"].Array();

    for (std::vector<mongo::BSONElement>::iterator it = elements.begin(); it != elements.end(); ++it)
    {
      mongo::BSONObj identifiable = it->embeddedObject();
      mongo::BSONElement elem;
      identifiable.getObjectID(elem);

      if (std::any_of(ids.begin(), ids.end(), std::bind2nd(std::equal_to<std::string>(), elem.OID().toString())))
      {
        clusters.push_back(rs::ObjectHypothesis(rs::conversion::toFeatureStructure(cas, identifiable)));
      }
    }
  }
  return clusters;
}
