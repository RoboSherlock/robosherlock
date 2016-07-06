#ifndef RSANALYSISENGINE_H
#define RSANALYSISENGINE_H

#include <rs/utils/output.h>
#include <rs/utils/time.h>
#include <rs/utils/exception.h>

#include <uima/api.hpp>

class RSAnalysisEngine
{
public:
  std::string name;

protected:
  uima::AnalysisEngine *engine;
  uima::CAS *cas;

public:

  RSAnalysisEngine();

  ~RSAnalysisEngine();

  virtual void init(const std::string &file);

  void stop();

  virtual void process();

  inline void resetCas()
  {
    cas->reset();
  }

};
#endif // RSANALYSISENGINE_H
