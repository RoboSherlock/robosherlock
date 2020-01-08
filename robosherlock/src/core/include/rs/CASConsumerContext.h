#ifndef ROBOSHERLOCK_CASCONSUMERCONTEXT_H
#define ROBOSHERLOCK_CASCONSUMERCONTEXT_H
#

#include <map>
#include "uima/api.hpp"


namespace rs
{
/**
 * This class is an interface for CAS Consumers. It enables them to access CASes that have been produced
 * by analysis engines that ran before the CAS consumers.
 *
 * In it's current state, it's implemented as a singleton to be able to access the CASes from Annotators.
 */
class CASConsumerContext
{
  static CASConsumerContext& getInstance()
  {
    // Since it's a static variable, if the class has already been created,
    // it won't be created again.
    // And it **is** thread-safe in C++11.
    static CASConsumerContext cccInstance;

    // Return a reference to our instance.
    return cccInstance;
  }

  // delete copy and move ctors and assign operators
  CASConsumerContext(CASConsumerContext const&) = delete;
  CASConsumerContext(CASConsumerContext&&) = delete;
  CASConsumerContext& operator=(CASConsumerContext const&) = delete;
  CASConsumerContext& operator=(CASConsumerContext&&) = delete;

protected:
  CASConsumerContext() = default;
  ~CASConsumerContext() = default;

  std::map<std::string, uima::CAS*> CASes;

public:
  /** Add a new CAS that should be accessible for the CAS Consumers **/
  void addCAS(std::string identifier, uima::CAS* cas);
  /** Remove a CAS by its identifier. Returns true if key was presented **/
  bool removeCAS(std::string identifier);
  /** Get CAS by the identifier you've used in addCAS. Returns nullptr if identifier couldn't be found. **/
  uima::CAS* getCAS(std::string identifier);
  /** Remove all stored CAS pointers in this CASConsumerContext **/
  void clearCASes();
};
} // End of namespace 'rs'
#endif  // ROBOSHERLOCK_CASCONSUMERCONTEXT_H
