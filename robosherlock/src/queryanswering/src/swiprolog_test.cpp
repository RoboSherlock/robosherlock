
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

#include <rs/queryanswering/SWIPLInterface.h>
#include <rs/queryanswering/ObjectDesignatorFactory.h>
#include <pcl/point_types.h>

#include <iostream>

#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>

class ROSInterface
{


public:
  //  typedef std::shared_ptr<PlEngine> PlEnginePtr;

  std::shared_ptr<rs::SWIPLInterface> plEngine_;
  ros::ServiceServer srv1_, srv2_;
  ros::NodeHandle *nh_;
  ros::AsyncSpinner spinner;
  std::mutex mutex;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
  ROSInterface(): spinner(4)
  {
    plEngine_ = std::make_shared<rs::SWIPLInterface>();
    nh_ = new ros::NodeHandle("test_swi");
    srv1_ = nh_->advertiseService("trigger1", &ROSInterface::trigger_service_cb1_, this);
    srv2_ = nh_->advertiseService("trigger2", &ROSInterface::trigger_service_cb2_, this);
    plEngine_->assertTestPipelnie();
    cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    pcl::io::loadPCDFile("/home/ferenc/work/rs_ws/src/robosherlock/robosherlock/clud_filtered_no_nan.pcd",*cloud_);
    spinner.start();
  }
  ~ROSInterface()
  {
    spinner.stop();
  }

  bool trigger_service_cb1_(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<std::string> pipeline;
    std::vector<std::string> keywords = {"shape"};
    plEngine_->q_subClassOf("KoellnMuesliKnusperHonigNuss","Drink");

    res.success = true;
    res.message = "Trigger successfull";
    return true;
  }


  bool trigger_service_cb2_(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    plEngine_->assertTestPipelnie();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_no_nan(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::SACSegmentation<pcl::PointXYZRGBA> plane_segmentation;

    pcl::PointIndicesPtr plane_inliers(new pcl::PointIndices);
    pcl::ModelCoefficientsPtr plane_coefficients(new pcl::ModelCoefficients);
    plane_segmentation.setOptimizeCoefficients(true);
    plane_segmentation.setModelType(pcl::SACMODEL_PLANE);
    plane_segmentation.setMethodType(pcl::SAC_RANSAC);
    plane_segmentation.setDistanceThreshold(0.01);
    plane_segmentation.setMaxIterations(50);
    plane_segmentation.setInputCloud(cloud_);
    plane_segmentation.segment(*plane_inliers, *plane_coefficients);

   pcl::io::savePCDFile("clud_plane.pcd", *cloud_, plane_inliers->indices);

    outInfo("plane inliers: " << plane_inliers->indices.size());
    if(plane_inliers->indices.size() < 5000)
    {
      outWarn("not enough inliers!");
      return false;
    }

    std::sort(plane_inliers->indices.begin(), plane_inliers->indices.end());
    if(plane_inliers->indices.size() == 0)
    {
      return false;
    }
    outDebug("Number of inliers in plane:" << plane_inliers->indices.size());
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
