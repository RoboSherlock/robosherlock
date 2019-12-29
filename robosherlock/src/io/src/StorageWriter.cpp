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

// UIMA
#include <uima/api.hpp>

// RS
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>
#include <robosherlock/utils/output.h>
#include <robosherlock/io/Storage.h>

using namespace uima;

class StorageWriter : public Annotator
{
private:
  std::string host;
  std::string db;
  rs::Storage storage;
  bool multi_cam_;

public:
  StorageWriter() : host(DB_HOST), db(DB_NAME), multi_cam_(true)
  {
  }

  TyErrorId initialize(AnnotatorContext& ctx)
  {
    outInfo("initialize");
    std::vector<std::string*> enableViews;
    bool clearStorageOnStart = false;
    bool unique = false;

    if (ctx.isParameterDefined("enableViews"))
    {
      ctx.extractValue("enableViews", enableViews);
    }
    if (ctx.isParameterDefined("host"))
    {
      ctx.extractValue("host", host);
    }
    if (ctx.isParameterDefined("storagedb"))
    {
      ctx.extractValue("storagedb", db);
    }
    if (ctx.isParameterDefined("clearStorageOnStart"))
    {
      ctx.extractValue("clearStorageOnStart", clearStorageOnStart);
    }
    if (ctx.isParameterDefined("newUniqueDB"))
    {
      ctx.extractValue("newUniqueDB", unique);
    }
    if (ctx.isParameterDefined("multi_cam"))
    {
      ctx.extractValue("multi_cam", multi_cam_);
    }

    if (unique)
    {
      std::ostringstream oss;
      oss << db << '_' << ros::Time::now().toNSec();
      db = oss.str();
    }
    try
    {
      storage = rs::Storage(host, db, clearStorageOnStart);
    }
    catch (std::exception& e)
    {  // This catches the seg fault when it cannot connect to MongoDB
      // Please ensure a safe return or kill the program after you write the error message
      // Usually it will end itself since it is SIGSEV
      outError("The Mongodb is not valid");
      outError(e.what());
      return 5;
    }

    outInfo("Setting db to: " << db);
    outInfo("Views stored:");
    for (size_t i = 0; i < enableViews.size(); ++i)
    {
      std::string temp_str = *enableViews[i];
      if (temp_str.find(".") == std::string::npos)
        temp_str = "cam0." + temp_str;

      storage.enableViewStoring(temp_str, true);
      outInfo(i << " : " << temp_str);
    }

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS& tcas, ResultSpecification const& res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");

    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    const uint64_t timestamp = (uint64_t)scene.timestamp();

    if (scene.id().empty())
    {
      storage.storeScene(*tcas.getBaseCas(), timestamp, multi_cam_);
    }
    else
    {
      storage.updateScene(*tcas.getBaseCas(), timestamp, multi_cam_);
    }

    if (cas.hasObjets())
    {
      outDebug("store persistent objects");
      storage.storeCollection(tcas, VIEW_OBJECTS, "persistent_objects");
    }

    return UIMA_ERR_NONE;
  }
};

MAKE_AE(StorageWriter)
