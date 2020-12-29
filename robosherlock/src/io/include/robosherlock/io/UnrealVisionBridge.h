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

/**
 * Interface class for objects that need to be notified when UnrealVisionBridgeDataInterface
 * receives new data. Please refer to UnrealVisionBridgeDataInterface::data_observer_ if you
 * want to register an observer.
 */
class IUnrealVisionBridgeDataInterfaceNotification{
public:
  virtual void notifyNewDataAvailable() = 0;
};

/**
 * Communication related class to gather data from URoboVision.
 * Interested classes can register as observers and get notified when new data is available.
 * Currently, this class is only used by the UnrealVisionBridge camera interface.
 *
 * Make sure to call init() once before using the class.
 *
 * Please note that this class has been designed under the assumption that there will always be only ONE
 * class per data receipt that is actually retrieving the data.
 * Having more classes that need to retreive the data at the same time requires a redesign in the mutex and copy handling
 * of the observer classes. However, for RS, Pipelines(AAEs) usually begin with a CollectionReader that
 * reads in data from a single URoboVision. This is supported.
 * However, if you need to run parallel AAEs or have two URoboVisions in the same AAE, it might not work.
 */
class UnrealVisionBridgeDataInterface{
public:
  static UnrealVisionBridgeDataInterface& getInstance()
  {
    // Since it's a static variable, if the class has already been created,
    // it won't be created again.
    // And it **is** thread-safe in C++11.
    static UnrealVisionBridgeDataInterface instance;

    // Return a reference to our instance.
    return instance;
  }

  // delete copy and move ctors and assign operators
  UnrealVisionBridgeDataInterface(UnrealVisionBridgeDataInterface const&) = delete;
  UnrealVisionBridgeDataInterface(UnrealVisionBridgeDataInterface&&) = delete;
  UnrealVisionBridgeDataInterface& operator=(UnrealVisionBridgeDataInterface const&) = delete;
  UnrealVisionBridgeDataInterface& operator=(UnrealVisionBridgeDataInterface&&) = delete;

  UnrealVisionBridgeDataInterface() : sizeRGB(3 * sizeof(uint8_t)), sizeFloat(sizeof(uint16_t)),
  running(false), isConnected(false), init_called_(false)
  {
    outInfo("UnrealVisionBridgeDataInterface Singleton instantiated");
  }



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
  bool running, isConnected, _newData, init_called_;

  std::string address;
  uint16_t port;
  int connection;

  std::vector<uint8_t> bufferComplete, bufferActive, bufferInUse;

  uint32_t mantissaTable[2048];
  uint32_t exponentTable[64];
  uint16_t offsetTable[64];

  std::vector<IUnrealVisionBridgeDataInterfaceNotification*> data_observer_;

  void init(std::string address, uint16_t port)
  {
    if(init_called_)
      return;

    outInfo("UnrealVisionBridgeDataInterface received first init() call.");

    init_called_ = true;

    this->address = address;
    this->port = port;
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

/**
 * Camera Interface to gather data from the URoboVision camera currently hosted at
 * https://github.com/robcog-iai/URoboVision
 * This class access a separate singleton class called UnrealVisionBridgeDataInterface
 * which handles the communication between the URoboVision Plugin and RoboSherlock.
 */
class UnrealVisionBridge : public CamInterface, IUnrealVisionBridgeDataInterfaceNotification
{
private:

  bool advertiseTf;
  bool lookUpViewpoint, onlyStableViewpoints;

  std::string address;
  uint16_t port;

  std::string tfFrom, tfTo;

  tf::TransformBroadcaster broadcaster;

  void readConfig(const boost::property_tree::ptree &pt);

public:
  UnrealVisionBridge(const boost::property_tree::ptree &pt);
  ~UnrealVisionBridge();

  bool setData(uima::CAS &tcas, u_int64_t = 0);

  /**
   * This class is called when UnrealVisionBridgeDataInterface receives new data.
   */
  void notifyNewDataAvailable() override;
};

#endif // __UNREAL_VISION_BRIDGE_H__
