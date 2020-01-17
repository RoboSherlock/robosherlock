#pragma once
#ifndef DESIGNATORSTORAGE_H
#define DESIGNATORSTORAGE_H


#include <uima/api.hpp>

#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/output.h>

#include <robosherlock/types/all_types.h>

#include <boost/bind.hpp>
#include <boost/signals2.hpp>
#include <map>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/allocators.h>
#include <rapidjson/prettywriter.h>

namespace rs
{


/**
 * @brief The ObjectDesignatorFactory class for generating json description of objects or object hypotheses.
 */
class ObjectDesignatorFactory
{
public:

  enum class Mode
  {
    CLUSTER = 0,
    OBJECT
  };

  ObjectDesignatorFactory::Mode mode;

  uint64_t now;
  uima::CAS *tcas;

  ObjectDesignatorFactory();

  ObjectDesignatorFactory(uima::CAS *cas);

  ObjectDesignatorFactory(uima::CAS *cas,rs::ObjectDesignatorFactory::Mode m);

  void setCAS(uima::CAS *cas);
  void setMode(ObjectDesignatorFactory::Mode m);

  virtual ~ObjectDesignatorFactory();

  bool getObjectDesignators(std::vector<std::string> &);

private:

  template<class T> void process(std::vector<T> &elements, std::vector<std::string> &objectDesignators, std::vector<double> lastSeen);
  static void mergeJson (rapidjson::Document &destination, rapidjson::Document &source, std::string fieldName);
  template<class T> std::string jsonToString(T &res, bool pretty=false);


};

}

#endif // DESIGNATORSTORAGE_H
