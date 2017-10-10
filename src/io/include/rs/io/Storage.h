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

#ifndef __STORAGE_H__
#define __STORAGE_H__

// STL
#include <unordered_map>
#include <vector>

// MONGO
#include <mongo/client/dbclient.h>

// UIMA
#include <uima/api.hpp>

// RS
#include <rs/types/core_types.h>
#include <rs/types/scene_types.h>


namespace rs
{

/******************************************************************************
 * Defines
 *****************************************************************************/

#define DB_HOST "localhost"
#define DB_NAME "Scenes"

/******************************************************************************
 * Storage
 *****************************************************************************/

class Storage
{
private:
  ::mongo::DBClientConnection db;

  std::string dbHost;
  std::string dbName;
  std::string dbBase;
  std::string dbCAS;
  std::string dbScripts;

  std::unordered_map<std::string, bool> storeViews;
  std::unordered_map<std::string, bool> loadViews;

  std::unordered_map<std::string,mongo::OID> camInfoOIDs;
  bool first;

  void setupDBScripts();

  bool readArrayFS(uima::FeatureStructure fs, ::mongo::BSONObjBuilder &builderCAS, const ::mongo::OID &casOID, const std::string &sofaId, const std::string &dbCollection);
  bool readFS(uima::FeatureStructure fs, ::mongo::BSONObjBuilder &builderCAS, const ::mongo::OID &casOID, const std::string &sofaId, const std::string &dbCollection);

  void loadView(uima::CAS &cas, const ::mongo::BSONElement &elem);
  uima::FeatureStructure loadArrayFS(uima::CAS *view, const std::string &viewName, const std::vector< ::mongo::OID> &ids);
  uima::FeatureStructure loadFS(uima::CAS *view, const std::string &viewName, const ::mongo::OID &id);

  void removeView(const ::mongo::BSONElement &elem);


public:
  Storage();
  Storage(const Storage &other);
  Storage(const std::string &dbHost, const std::string &dbName, const bool clear = false, const bool setupScripts = true);
  virtual ~Storage();

  Storage &operator=(const Storage &other);

  void enableViewStoring(const std::string &viewName, const bool enable);
  void enableViewLoading(const std::string &viewName, const bool enable);

  void getScenes(std::vector<uint64_t> &timestamps);

  bool storeScene(uima::CAS &cas, const uint64_t &timestamp);
  bool removeScene(const uint64_t &timestamp);
  bool updateScene(uima::CAS &cas, const uint64_t &timestamp);
  bool loadScene(uima::CAS &cas, const uint64_t &timestamp);

  void removeCollection(const std::string &collection);
  void storeCollection(uima::CAS &cas, const std::string &view, const std::string &collection);
  void loadCollection(uima::CAS &cas, const std::string &view, const std::string &collection);

  std::vector<Cluster> getClusters(uima::CAS &cas, const std::string &collection, std::vector<std::string> ids);

  uint64_t prevTS;
};

} // namespace rs

#endif //__STORAGE_H__
