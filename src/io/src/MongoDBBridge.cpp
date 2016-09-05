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
// RS
#include <rs/io/MongoDBBridge.h>
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/utils/exception.h>

#include <sys/stat.h>
#include <thread>

MongoDBBridge::MongoDBBridge(const boost::property_tree::ptree &pt) : CamInterface(pt)
{
  readConfig(pt);

  outInfo("initialize");

  storage = rs::Storage(host, db);

  actualFrame = 0;

  storage.getScenes(frames);

  if(continual)
  {
    actualFrame = frames.size();
  }
  else
  {
    if(frames.empty())
    {
      outError("No frames in database found!");
      throw_exception_message("no frames found.")
    }
    outInfo("found " << frames.size() << " frames in database.");
    lastTimestamp = 0x7FFFFFFFFFFFFFFF;
  }
  _newData = true;
}

MongoDBBridge::~MongoDBBridge()
{
}

void MongoDBBridge::readConfig(const boost::property_tree::ptree &pt)
{
  host = pt.get<std::string>("mongodb.host");
  db = pt.get<std::string>("mongodb.db");

  continual = pt.get<bool>("mongodb.continual");
  loop = pt.get<bool>("mongodb.loop");
  playbackSpeed = pt.get<double>("mongodb.playbackSpeed", 0.0);

  outInfo("DB host:   " FG_BLUE << host);
  outInfo("DB name:   " FG_BLUE << db);
  outInfo("continual: " FG_BLUE << (continual ? "ON" : "OFF"));
  outInfo("looping:   " FG_BLUE << (loop ? "ON" : "OFF"));

  char *host_env = getenv("MONGO_PORT_27017_TCP_ADDR");
  char *port_env = getenv("MONGO_PORT_27017_TCP_PORT");

  if(host_env != NULL && port_env != NULL)
  {
    outInfo("Mongo host stet to: " + std::string(host_env) + ":" + std::string(port_env));
    host  = std::string(host_env) + ":" + std::string(port_env);
  }
}

bool MongoDBBridge::setData(uima::CAS &tcas, uint64_t timestamp)
{
  MEASURE_TIME;
  const bool isNextFrame = timestamp == 0;

  if(actualFrame >= frames.size())
  {
    if(continual)
    {
      storage.getScenes(frames);
      if(actualFrame >= frames.size())
      {
        return false;
      }
    }
    else if(loop)
    {
      actualFrame = 0;
    }
    else
    {
      outInfo("last frame. shuting down.");
      cv::waitKey();
      ros::shutdown();
      return false;
    }
  }
  if(isNextFrame)
  {
    outInfo("default behaviour");
    timestamp = frames[actualFrame];
    outInfo("setting data from frame: " << actualFrame << " (" << timestamp << ")");
  }
  else
  {
    outInfo("setting data from frame with timestamp: (" << timestamp << ")");
  }

  if(!storage.loadScene(*tcas.getBaseCas(), timestamp))
  {
    if(timestamp == 0)
    {
      outInfo("loading frame failed. shuting down.");
      ros::shutdown();
      return false;
    }
    else
    {
      outInfo("No frame with that timestamp");
      return false;
    }
  }


  if(playbackSpeed > 0.0 && isNextFrame)
  {
    if(lastTimestamp > timestamp)
    {
      lastTimestamp = timestamp;
      simTimeLast = frames[actualFrame];
      lastRun = ros::Time::now().toNSec();
    }

    uint64_t now = ros::Time::now().toNSec();
    uint64_t simTime = (uint64_t)((now - lastRun) * playbackSpeed) + simTimeLast;
    if(simTime <= timestamp)
    {
      uint64_t sleepTime = (timestamp - simTime) / playbackSpeed;
      outDebug("waiting for " << sleepTime / 1000000.0 << " ms.");
      std::this_thread::sleep_for(std::chrono::nanoseconds(sleepTime));
    }

    now = ros::Time::now().toNSec();
    simTimeLast = (uint64_t)((now - lastRun) * playbackSpeed) + simTimeLast;
    lastRun = now;
  }


  ++actualFrame;
  if(!continual && !loop && actualFrame == frames.size())
  {
    _newData = false;
  }
  return true;
}
