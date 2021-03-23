//
// Created by pmania on 08.01.20.
//

#include "robosherlock/CASConsumerContext.h"
#include <algorithm>

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

std::string CASConsumerContext::getCAStoString(uima::CAS &tcas, bool saveWithPointCloud)
{
  uima::XCASWriter writer(tcas, true);

  std::stringstream casAsStringStream;
  // writing whole cas to stringstream
  writer.write(casAsStringStream);

  // define parameter for point clouds stuff
  std::string byteArrayB = "<uima.cas.ByteArray";
  std::string byteArrayE = " </uima.cas.ByteArray>";

  std::string intArrayB = "<uima.cas.IntegerArray";
  std::string intArrayE = "</uima.cas.IntegerArray>";

  std::string removedPointsString = casAsStringStream.str();

  if (!saveWithPointCloud)
  {
    while (removedPointsString.find(byteArrayB) != std::string::npos)
    {
      // find the specific beginning and ending of uima.cas.ByteArray child
      std::string::size_type ByteABegin = removedPointsString.find(byteArrayB);
      std::string::size_type ByteAEnd = removedPointsString.find(byteArrayE);

      // remove ByteArray from CAS
      removedPointsString = removedPointsString.erase(ByteABegin, ((ByteAEnd - ByteABegin) + byteArrayE.length()));

      if (removedPointsString.find(intArrayB) != std::string::npos)
      {
        // find the specific beginning and ending of uima.cas.IntegerArray child
        std::string::size_type IntegerABegin = removedPointsString.find(intArrayB);
        std::string::size_type IntegerAEnd = removedPointsString.find(intArrayE);

        // remove IntegerArray from CAS
        removedPointsString =
            removedPointsString.erase(IntegerABegin, ((IntegerAEnd - IntegerABegin) + intArrayE.length()));
      }
    }
  }

  return removedPointsString;
}

void CASConsumerContext::saveCASToXML(uima::CAS &tcas, std::string strOutDir, bool saveWithPointCloud, int docnum)
{
  std::string removedPointsString = getCAStoString(tcas, saveWithPointCloud);

  std::string createFileName = "" + strOutDir + "/doc" + std::to_string(docnum++) + ".xml";
  std::ofstream file(createFileName);
  file << removedPointsString;
}
