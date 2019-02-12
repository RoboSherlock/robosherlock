
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Trigger.h>

//#include <SWI-Prolog.h>
//#include <SWI-cpp.h>

#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <algorithm>

#include <pcl/point_types.h>
#include <rs/queryanswering/SWIPLInterface.h>
#include <iostream>


class ROSInterface
{


public:
  //  typedef std::shared_ptr<PlEngine> PlEnginePtr;

  std::shared_ptr<rs::SWIPLInterface> plEngine_;
  ros::ServiceServer srv_;
  ros::NodeHandle *nh_;
  ros::AsyncSpinner spinner;
  std::mutex mutex;

  ROSInterface(): spinner(1)
  {
    plEngine_ = std::make_shared<rs::SWIPLInterface>();
    plEngine_->simple_query();
    nh_ = new ros::NodeHandle("ROS_SWIPL");
    srv_ = nh_->advertiseService("trigger", &ROSInterface::trigger_service_cb_, this);
    spinner.start();
  }
  ~ROSInterface()
  {
    spinner.stop();
  }

  bool trigger_service_cb_(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    std::lock_guard<std::mutex> lock(mutex);
    plEngine_->assertValueForKey("key", "value");
    res.success = true;
    res.message = "Trigger successfull";
    return true;
  }
  void run()
  {
    for(; ros::ok();)
    {
     std::lock_guard<std::mutex> lock(mutex);
//      ros::spinOnce();
      usleep(100000);
    }
  }
};

int main(int argc, char **argv)
{
  std::cerr << argv[0] << std::endl;
  ros::init(argc, argv, "test_swi");
  ROSInterface ri;
//  ri.run();
  ros::waitForShutdown();
  return 0;
}
