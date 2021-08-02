//
// Created by pmania on 08.01.20.
//

#include "robosherlock/CASConsumerContext.h"


using namespace rs;

// Add a new CAS that should be accessible for the CAS Consumers
void CASConsumerContext::addCAS(std::string identifier, uima::CAS* cas)
{
  CASes[identifier] = cas;
}
// Remove a CAS by its identifier. Returns true if key was presented
bool CASConsumerContext::removeCAS(std::string identifier)
{
  // return true if more than 0 elements have been deleted.
  return CASes.erase(identifier) > 0;
}
// Get CAS by the identifier you've used in addCAS. Returns nullptr if identifier couldn't be found.
uima::CAS* CASConsumerContext::getCAS(std::string identifier)
{
  if(CASes.count(identifier) > 0){
    return CASes.at(identifier);
  }

  return nullptr;
}

// Remove all stored CAS pointers in this CASConsumerContext
void CASConsumerContext::clearCASes()
{
  CASes.clear();
}

std::vector<std::string> CASConsumerContext::getCASIdentifiers() {
    std::vector<std::string> ret;
    for (auto const& element : CASes) {
        ret.push_back(element.first);
    }
    return ret;
}
