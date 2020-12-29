/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *         Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *         Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __UNREAL_VISION_BRIDGE_H__
#define __UNREAL_VISION_BRIDGE_H__

// STD
#include <string>
#include <vector>
#include <thread>
#include <mutex>

// RS
#include <robosherlock/io/CamInterface.h>

//TF
#include <tf/transform_broadcaster.h>


class IUVBDINotification{
public:
  virtual void notifyNewDataAvailable() = 0;
};

class UnrealVisionBridgeDataInterface{
public:

  struct Vector
  {
    float x;
    float y;
    float z;
  };

  struct Quaternion
  {
    float x;
    float y;
    float z;
    float w;
  };

  struct PacketHeader
  {
    uint32_t size;
    uint32_t sizeHeader;
    uint32_t mapEntries;
    uint32_t width;
    uint32_t height;
    uint64_t timestampCapture;
    uint64_t timestampSent;
    float fieldOfViewX;
    float fieldOfViewY;
    Vector translation;
    Quaternion rotation;
  };

  struct MapEntry
  {
    uint32_t size;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    char firstChar;
  };

  struct Packet
  {
    PacketHeader header;
    uint8_t *pColor, *pDepth, *pObject, *pMap;
    size_t sizeColor, sizeDepth, sizeObject;
  } packet;

  const size_t sizeRGB;
  const size_t sizeFloat;

  std::thread receiver;
  std::mutex lockBuffer;
  bool running, isConnected, _newData;

  std::string address;
  uint16_t port;
  int connection;

  std::vector<uint8_t> bufferComplete, bufferActive, bufferInUse;

  uint32_t mantissaTable[2048];
  uint32_t exponentTable[64];
  uint16_t offsetTable[64];

  std::vector<IUVBDINotification*> data_observer_;

  UnrealVisionBridgeDataInterface(std::string address, uint16_t port) :  sizeRGB(3 * sizeof(uint8_t)), sizeFloat(sizeof(uint16_t)),
                                                                         running(false), isConnected(false), address(address), port(port)
  {
    const size_t bufferSize = 1024 * 1024 * 10;
    bufferComplete.resize(bufferSize);
    bufferActive.resize(bufferSize);
    bufferInUse.resize(bufferSize);

    outInfo("starting receiver and transmitter threads.");
    running = true;
    receiver = std::thread(&UnrealVisionBridgeDataInterface::receive, this);
    createLookupTables();
  }

  ~UnrealVisionBridgeDataInterface()
  {
    running = false;
    receiver.join();
  }

  void convertDepth(const uint16_t *in, __m128 *out) const;
  void convertDepth(const uint16_t *in, uint32_t *out) const;
  void connectToServer();
  void receive();

  void createLookupTables();
  uint32_t convertMantissa(const uint32_t i) const;
};


class UnrealVisionBridge : public CamInterface, IUVBDINotification
{
private:

  bool advertiseTf;
  bool lookUpViewpoint, onlyStableViewpoints;

  std::string address;
  uint16_t port;

  std::string tfFrom, tfTo;

  tf::TransformBroadcaster broadcaster;

  std::shared_ptr<UnrealVisionBridgeDataInterface> uvb_datainterface_;


  void readConfig(const boost::property_tree::ptree &pt);

public:
  UnrealVisionBridge(const boost::property_tree::ptree &pt);
  ~UnrealVisionBridge();

  bool setData(uima::CAS &tcas, u_int64_t = 0);

//  bool newData() const;
  void notifyNewDataAvailable() override;
};

#endif // __UNREAL_VISION_BRIDGE_H__
