#ifndef __MONGODB_BRIDGE_H__
#define __MONGODB_BRIDGE_H__

// RS
#include <rs/io/CamInterface.h>
#include <rs/io/Storage.h>

class MongoDBBridge : public CamInterface
{
private:
  std::string host;
  std::string db;
  rs::Storage storage;

  std::vector<uint64_t> frames;
  size_t actualFrame;
  bool continual;
  bool loop;

  void readConfig(const boost::property_tree::ptree &pt);

public:
  MongoDBBridge(const boost::property_tree::ptree &pt);
  ~MongoDBBridge();

  bool setData(uima::CAS &tcas);
  bool setData(uima::CAS &tcas, uint64_t ts = 0);
};

#endif // __MONGODB_BRIDGE_H__
