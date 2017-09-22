#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/common.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <ctime>

#define LOGNAME_FORMAT "%Y%m%d_%H%M%S"

using namespace uima;


class TestDataWriter : public Annotator
{
public:

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  void writeViewpointIni(tf::StampedTransform viewpoint, std::string path, std::string filename){
      boost::property_tree::ptree pt;
      tf::Quaternion q = viewpoint.getRotation();
      tf::Vector3 trans = viewpoint.getOrigin();

      pt.put("translation.x",trans.getX());
      pt.put("translation.y",trans.getY());
      pt.put("translation.z",trans.getZ());
      pt.put("rotation.x",q.getX());
      pt.put("rotation.y",q.getY());
      pt.put("rotation.z",q.getZ());
      pt.put("rotation.w",q.getW());
      pt.put("frame.child_frame",viewpoint.child_frame_id_);
      pt.put("frame.frame",viewpoint.frame_id_);
      pt.put("viewpoint.stamp",viewpoint.stamp_);
      std::stringstream ss;
      ss << path << "/samples/viewpoints/" << filename << ".ini";
      write_ini( ss.str(), pt );
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_CLOUD,*cloud_ptr);
    cv::Mat colorImage;
    cas.get(VIEW_COLOR_IMAGE_HD, colorImage);

    char filename[20];
    time_t now = time(0);
    strftime(filename, sizeof(filename), LOGNAME_FORMAT, localtime(&now));

    std::string path = ros::package::getPath("robosherlock");

    if(scene.viewPoint.has()){
        tf::Stamped<tf::Pose> tf_stamped_pose;
        rs::conversion::from(scene.viewPoint.get(), tf_stamped_pose);
        writeViewpointIni(tf::StampedTransform(tf_stamped_pose, tf_stamped_pose.stamp_, tf_stamped_pose.frame_id_,
                                               "head_mount_kinect_rgb_optical_frame"), path, filename);
    }
    std::stringstream ssIm;
    ssIm << path << "/samples/images/" << filename << ".png";
    cv::imwrite(ssIm.str(),colorImage);
    std::stringstream ssCloud;
    ssCloud << path << "/samples/clouds/" <<filename << ".pcd";
    pcl::io::savePCDFileASCII (ssCloud.str(), *cloud_ptr);

    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(TestDataWriter)
