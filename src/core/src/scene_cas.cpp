/*
 * scene_cas.cpp
 *
 *  Created on: Mar 29, 2012
 *      Author: christiankerl
 */

#include <rs/scene_cas.h>
#include <rs/utils/output.h>

// Force disable debug output
#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_INFO

namespace rs
{

SceneCas::SceneCas(uima::CAS &cas) :
  cas(cas)
{
}

SceneCas::~SceneCas()
{
}

bool SceneCas::getView(const char *name, uima::CAS *&view)
{
  try
  {
    view = cas.getView(name);
    outDebug("Got view: " << name);
    return true;
  }
  catch(const uima::CASException &e)
  {
    outDebug("There is no view: " << name);
  }
  return false;
}

rs::Scene SceneCas::getScene()
{
  uima::FeatureStructure fs;
  if(!getFS(VIEW_SCENE, fs))
  {
    rs::Scene scene = rs::create<rs::Scene>(cas);
    setFS(VIEW_SCENE, (uima::FeatureStructure)scene);
    return scene;
  }
  return rs::Scene(fs);
}

bool SceneCas::has(const char *name)
{
  uima::CAS *view;
  return getView(name, view);
}

bool SceneCas::getFS(const char *name, uima::FeatureStructure &fs)
{
  uima::CAS *view;
  if(!getView(name, view))
  {
    outDebug("View '" << name << "' does not exist!");
    return false;
  }
  fs = view->getSofaDataArray();
  return true;
}

void SceneCas::setFS(const char *name, const uima::FeatureStructure &fs)
{
  uima::CAS *view;
  if(!getView(name, view))
  {
    view = cas.createView(name);
  }
  const std::string mime = std::string("application/x-") + name;

  view->setSofaDataArray(fs, UnicodeString::fromUTF8(mime));
}

}
