#ifndef RSOBJDESCRIPTIONREFINER_H
#define RSOBJDESCRIPTIONREFINER_H

#include <rs_queryanswering/RSControledAnalysisEngine.h>

class RSObjectsRefiner
{
private:
  RSControledAnalysisEngine engine;
  std::vector<designator_integration::Designator> queryQeue;
public:
  RSObjectsRefiner()
  {

  }
  ~RSObjectsRefiner()
  {
  }

  void init(const std::string &xmlFile)
  {
    //xmlFile is a copy of the one set for the main engine but reading from a DB instead
    //owerwrite a field in the AE or create a copy????
    engine.init(xmlFile, std::vector<std::string>());
  }

  void addQuery(designator_integration::Designator q)
  {
      queryQeue.push_back(q);
  }
  void getNextAction()
  {
      //rostopic get next planned action
      //openease get average duration
      //if conditions permit
      //engine.process
  }
};


#endif // RSOBJDESCRIPTIONREFINER_H
