
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
  ros::ServiceServer srv1_, srv2_;
  ros::NodeHandle *nh_;
  ros::AsyncSpinner spinner;
  std::mutex mutex;

  ROSInterface(): spinner(4)
  {
    plEngine_ = std::make_shared<rs::SWIPLInterface>();
    nh_ = new ros::NodeHandle("ROS_SWIPL");
    srv1_ = nh_->advertiseService("trigger1", &ROSInterface::trigger_service_cb1_, this);
    srv2_ = nh_->advertiseService("trigger2", &ROSInterface::trigger_service_cb2_, this);
    spinner.start();
  }
  ~ROSInterface()
  {
    spinner.stop();
  }

  bool trigger_service_cb1_(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    std::lock_guard<std::mutex> lock(mutex);
    plEngine_->assertValueForKey("key", "value");
    res.success = true;
    res.message = "Trigger successfull";
    return true;
  }

  bool trigger_service_cb2_(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    std::lock_guard<std::mutex> lock(mutex);
    plEngine_->retractQueryLanguage();
    res.success = true;
    res.message = "Trigger successfull";
    return true;
  }
  void run()
  {
    for(; ros::ok();)
    {
      {
        std::lock_guard<std::mutex> lock(mutex);
        plEngine_->retractQueryKvPs();
      }
      sleep(1);
    }
  }
};

int main(int argc, char **argv)
{
  std::cerr << argv[0] << std::endl;
  ros::init(argc, argv, "test_swi");
  ROSInterface ri;
  ri.run();
  //  ros::waitForShutdown();
  return 0;
}
