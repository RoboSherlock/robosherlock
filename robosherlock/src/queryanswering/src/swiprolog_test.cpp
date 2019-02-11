#include <iostream>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

//#include <SWI-Prolog.h>
//#include <SWI-cpp.h>
#include <memory>
#include <thread>
#include <mutex>
#include <ros/package.h>
#include <algorithm>

#include <rs/queryanswering/SWIPLInterface.h>

//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/PointIndices.h>
//#include <pcl/ModelCoefficients.h>

//#include <opencv2/opencv.hpp>




class ROSInterface
{


public:
  //  typedef std::shared_ptr<PlEngine> PlEnginePtr;

  std::shared_ptr<rs::SWIPLInterface> plEngine_;
  ros::ServiceServer srv_;
  ros::NodeHandle *nh_;

  ROSInterface()
  {
    plEngine_ = std::make_shared<rs::SWIPLInterface>();
    nh_ = new ros::NodeHandle("ROS_SWIPL");
    srv_ = nh_->advertiseService("trigger", &ROSInterface::trigger_service_cb_, this);
  }

  bool trigger_service_cb_(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    plEngine_->simple_query();
    res.success = true;
    res.message = "Trigger successfull";
    return true;
  }
  void run()
  {
    while(ros::ok()) {
      ros::spinOnce();
      usleep(100000);
    }
  }
};



int main(int argc, char **argv)
{

  ros::init(argc, argv, "test_swi");
  ROSInterface ri;
  ri.run();
  return 0;
}
