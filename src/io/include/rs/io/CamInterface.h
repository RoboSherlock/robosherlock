#ifndef __CAM_INTERFACE_H__
#define __CAM_INTERFACE_H__

// UIMA
#include <uima/api.hpp>

// Boost
#include <boost/property_tree/ptree.hpp>

class CamInterface
{
protected:
  bool _newData;

  CamInterface(const boost::property_tree::ptree &pt) : _newData(false) {}

public:
  virtual ~CamInterface() {}

  bool newData() const
  {
    return _newData;
  }

  virtual bool setData(uima::CAS &tcas, uint64_t ts = 0) = 0;
};

#endif // __CAM_INTERFACE_H__
