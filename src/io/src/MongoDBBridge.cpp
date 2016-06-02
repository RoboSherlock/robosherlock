// RS
#include <rs/io/MongoDBBridge.h>
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/utils/exception.h>

#include <sys/stat.h>

MongoDBBridge::MongoDBBridge(const boost::property_tree::ptree &pt) : CamInterface(pt)
{
  readConfig(pt);

  outInfo("initialize");

  storage = rs::Storage(host, db);

  actualFrame = 0;
  _newData = true;

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
  }
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

  outInfo("DB host:   " FG_BLUE << host);
  outInfo("DB name:   " FG_BLUE << db);
  outInfo("continual: " FG_BLUE << (continual ? "ON" : "OFF"));
  outInfo("looping:   " FG_BLUE << (loop ? "ON" : "OFF"));

  char* host_env = getenv("MONGO_PORT_27017_TCP_ADDR");
  char* port_env = getenv("MONGO_PORT_27017_TCP_PORT");

  if(host_env!=NULL && port_env!=NULL)
  {
    outInfo("Mongo host stet to: "+ std::string(host_env)+ ":"+std::string(port_env));
    host  = std::string(host_env)+":"+std::string(port_env);
  }
}

bool MongoDBBridge::setData(uima::CAS &tcas, uint64_t timestamp)
{
  MEASURE_TIME;

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
  if(timestamp == 0)
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

  ++actualFrame;
  return true;
}
