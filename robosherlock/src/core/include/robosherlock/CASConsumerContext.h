#ifndef ROBOSHERLOCK_CASCONSUMERCONTEXT_H
#define ROBOSHERLOCK_CASCONSUMERCONTEXT_H
#

#include <map>
#include "uima/api.hpp"
#include "uima/xmlwriter.hpp"

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
public:
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

  /**
   * Get a list of every identifier that is currently presented in the CASConsumerContext.
   * The order of the identifiers in the list is alphabetically (std::Map default) and
   * _NOT_ in the order they have been added.
   *
   * @return List of identifiers
   */
  std::vector<std::string> getCASIdentifiers();

  /**
   * Convert the CAS to std::string, optional all the ByteArray and Integer Array are removed (point clouds and depth)
   * @param CAS that should be converted
   * @param saveWithPointCloud, if TRUE then the whole point clouds are converted too, if FALSE point clouds are removed
   * @return std::string that contains the cas
   */
  std::string getCAStoString(uima::CAS &tcas, bool saveWithPointCloud);

  /**
   * Convert the CAS to std::string and saving it as xml file, optional all the ByteArray and Integer Array are removed
   * (point clouds and depth)
   * @param CAS that should be converted
   * @param saveWithPointCloud, if TRUE then the whole point clouds are converted too, if FALSE point clouds are removed
   * @param strOutDir the output directory fr the file
   * @param docnum the number that the nme is containing (iteration 0, 1, 2..)
   */
  void saveCASToXML(uima::CAS &tcas, std::string strOutDir, bool saveWithPointCloud, int docnum);
};
} // End of namespace 'rs'
#endif  // ROBOSHERLOCK_CASCONSUMERCONTEXT_H
