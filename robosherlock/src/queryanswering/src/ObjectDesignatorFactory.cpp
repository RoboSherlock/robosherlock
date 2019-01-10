#include <rs/queryanswering/ObjectDesignatorFactory.h>
#include "rapidjson/pointer.h"

namespace rs
{

ObjectDesignatorFactory::ObjectDesignatorFactory()
{
  tcas = NULL;
}

ObjectDesignatorFactory::~ObjectDesignatorFactory()
{
}

ObjectDesignatorFactory::ObjectDesignatorFactory(uima::CAS *cas) : tcas(cas)
{
  mode = ObjectDesignatorFactory::Mode::CLUSTER;
}


void ObjectDesignatorFactory::setCAS(uima::CAS *cas)
{
  tcas = cas;
}

void ObjectDesignatorFactory::setMode(ObjectDesignatorFactory::Mode m)
{
  mode = m;
}

bool ObjectDesignatorFactory::getObjectDesignators(std::vector<std::string> &objectDesignators)
{
  if(!tcas) {
    std::cout << "NULL Pointer in DesignatorWrapper::getDesignatorResponse. tcas is not set! Use DesignatorWrapper::setCAS before calling this method" << std::endl;
    return false;
  }

  rs::SceneCas cas(*tcas);
  rs::Scene scene = cas.getScene();
  now = scene.timestamp();

  if(mode == ObjectDesignatorFactory::Mode::CLUSTER) {
    std::vector<rs::ObjectHypothesis> clusters;
    scene.identifiables.filter(clusters);
    std::vector<double> lastSeen;
    lastSeen.resize(clusters.size(), 0.0);
    process(clusters, objectDesignators, lastSeen);
  }
  else {
    std::vector<rs::Object> allObjects, objects;
    cas.get(VIEW_OBJECTS, allObjects);
    outWarn("objects found: " << allObjects.size());
    std::vector<double> lastSeen;
    for(size_t i = 0; i < allObjects.size(); ++i) {
      rs::Object &object = allObjects[i];
      lastSeen.push_back((now - (uint64_t)object.lastSeen()) / 1000000000.0);
      //      if(lastSeen == 0)
      //      {
      //        objects.push_back(object);
      //      }
    }
    process(allObjects, objectDesignators, lastSeen);
  }
  return true;
}

template<class T> void ObjectDesignatorFactory::process(std::vector<T> &elements, std::vector<std::string> &objectDesignators, std::vector<double> lastSeen)
{
  for(size_t i = 0; i < elements.size(); ++i) {
    outDebug("reading object: " << i);

    if(lastSeen[i] != 0.0) {
      objectDesignators.push_back("");
      continue;
    }
    T &element = elements[i];
    rapidjson::Document objectDesignator(rapidjson::kObjectType);
    double seconds = now / 1000000000;
    double nsec = (now % 1000000000) / 1000000000.0;

    outDebug("Seconds: " << std::setprecision(12) << seconds);
    outDebug("NanoSecs: " << std::setprecision(12) << nsec);
    double time = seconds + nsec;

    outDebug("time: " << std::setprecision(20) << time);
    objectDesignator.AddMember("timestamp", time, objectDesignator.GetAllocator());
    for(auto annotation : element.annotations) {
      rapidjson::Document annotAsJson(rapidjson::kObjectType);
      rs::conversion::from(annotation, annotAsJson);

      for(rapidjson::Value::MemberIterator it = annotAsJson.MemberBegin(); it != annotAsJson.MemberEnd(); ++it) {
        outInfo(it->name.GetString());
        rapidjson::Value key(it->name.GetString(), objectDesignator.GetAllocator());
        rapidjson::Document val(rapidjson::kObjectType);
        val.CopyFrom(it->value, objectDesignator.GetAllocator());
        if(!objectDesignator.HasMember(it->name.GetString())) {
          rapidjson::Document array(rapidjson::kArrayType);
          array.PushBack(val, objectDesignator.GetAllocator());
          objectDesignator.AddMember(key, array.Move(), objectDesignator.GetAllocator());
        }
        else {
            objectDesignator[it->name.GetString()].PushBack(val, objectDesignator.GetAllocator());
        }

      }
    }

    if(objectDesignator.MemberCount() > 0) {
      outDebug("Object as json:");
      outDebug(jsonToString(objectDesignator, true));
      objectDesignators.push_back(jsonToString(objectDesignator));
    }
  }
}

void ObjectDesignatorFactory::mergeJson(rapidjson::Document &destination, rapidjson::Document &source, std::string fieldName)
{
  rapidjson::Value fieldNameV(fieldName, destination.GetAllocator());
  destination.AddMember(fieldNameV, source, destination.GetAllocator());
}

template<class T> std::string ObjectDesignatorFactory::jsonToString(T &res, bool pretty)
{
  rapidjson::StringBuffer buffer;
  if(pretty) {
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    writer.SetFormatOptions(rapidjson::PrettyFormatOptions::kFormatSingleLineArray);
    res.Accept(writer);
  }
  else {
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    res.Accept(writer);
  }
  return buffer.GetString();
}

template void ObjectDesignatorFactory::process<rs::ObjectHypothesis>(std::vector<rs::ObjectHypothesis> &elements,
    std::vector<std::string> &objectDesignators, std::vector<double> lastSeen);

template void ObjectDesignatorFactory::process<rs::Object>(std::vector<rs::Object> &elements,
    std::vector<std::string> &objectDesignators, std::vector<double> lastSeen);

template std::string ObjectDesignatorFactory::jsonToString<rapidjson::Document>(rapidjson::Document &res, bool pretty);
template std::string ObjectDesignatorFactory::jsonToString<rapidjson::Value>(rapidjson::Value &res, bool pretty);

}
