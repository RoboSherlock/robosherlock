/* Copyright (c) 2013, Thiemo Wiedemeyer  <wiedemeyer@informatik.uni-bremen.de>
 * All rights reserved.
*/

// UIMA
#include <uima/api.hpp>

// RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/io/Storage.h>

using namespace uima;

class StorageWriter : public Annotator
{
private:
  std::string host;
  std::string db;
  rs::Storage storage;

public:
  StorageWriter() : host(DB_HOST), db(DB_NAME)
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    std::vector<std::string *> enableViews;
    bool clearStorageOnStart = false;

    if(ctx.isParameterDefined("enableViews"))
    {
      ctx.extractValue("enableViews", enableViews);
    }
    if(ctx.isParameterDefined("host"))
    {
      ctx.extractValue("host", host);
    }
    if(ctx.isParameterDefined("storagedb"))
    {
      ctx.extractValue("storagedb", db);
      outInfo("Setting db to: "<<db);
    }
    if(ctx.isParameterDefined("clearStorageOnStart"))
    {
      ctx.extractValue("clearStorageOnStart", clearStorageOnStart);
    }
    storage = rs::Storage(host, db, clearStorageOnStart);

    outInfo("Views stored:");
    for(size_t i = 0; i < enableViews.size(); ++i)
    {
      storage.enableViewStoring(*enableViews[i], true);
      outInfo(i<<" : "<<*enableViews[i]);
    }

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");

    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    const uint64_t timestamp = (uint64_t)scene.timestamp();

    if(scene.id().empty())
    {
      storage.storeScene(*tcas.getBaseCas(), timestamp);
    }
    else
    {
      storage.updateScene(*tcas.getBaseCas(), timestamp);
    }

    if(cas.has(VIEW_OBJECTS))
    {
      outDebug("store persistent objects");
      storage.storeCollection(tcas, VIEW_OBJECTS, "persistent_objects");
    }

    return UIMA_ERR_NONE;
  }
};

MAKE_AE(StorageWriter)

