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
  double playbackSpeed;
  uint64_t lastTimestamp, lastRun, simTimeLast;

  void readConfig(const boost::property_tree::ptree &pt);

public:
  MongoDBBridge(const boost::property_tree::ptree &pt);
  ~MongoDBBridge();

  bool setData(uima::CAS &tcas);
  bool setData(uima::CAS &tcas, uint64_t ts = std::numeric_limits<uint64_t>::max());
};

#endif // __MONGODB_BRIDGE_H__
