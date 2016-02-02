#ifndef RSANALYSISENGINE_H
#define RSANALYSISENGINE_H

#include <rs/utils/output.h>
#include <rs/utils/time.h>
#include <rs/utils/exception.h>

#include <uima/api.hpp>

class RSAnalysisEngine
{
private:
  std::string name;

  uima::AnalysisEngine *engine;
  uima::CAS *cas;

public:

  RSAnalysisEngine();

  ~RSAnalysisEngine();

  void init(const std::string &file);

  void stop();

  void process();

};
#endif // RSANALYSISENGINE_H
