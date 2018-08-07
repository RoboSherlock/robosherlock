#include <string.h>
#include <gtest/gtest.h>
#include <errno.h>
#include <sys/stat.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include <uima/fsfilterbuilder.hpp>

#include <ros/ros.h>
#include <rs/flowcontrol/RSProcessManager.h>
#include <rs/flowcontrol/RSAnalysisEngineManager.h>
#include <rs/utils/common.h>
#include <rs/io/Storage.h>

#include <rs/conversion/conversion.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <rs/scene_cas.h>

#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG
#include "../main.h"

bool exist_plane_pcl = true;
bool exist_plane_mps = true;
//Boolean checks if there are over 10 000 points in the plane;
bool number_of_points_pcl = false;
bool number_of_points_mps = false;
typedef pcl::PointXYZRGBA PointT;

int planeTest()
{
  pcl::PointCloud<PointT>::Ptr cloud_ptr;
  cloud_ptr = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
  cv::Mat color;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  std::vector<std::string> engineList = {"CollectionReader","ImagePreprocessor","NormalEstimator","PlaneAnnotator"};
  engine.getPipelineManager()->setPipelineOrdering(engineList);

  engine.process();
  
  cas = engine.getCas();
  rs::SceneCas sceneCas(*cas);

  sceneCas.get(VIEW_CLOUD, *cloud_ptr);
  sceneCas.get(VIEW_NORMALS, *cloud_normals);
  sceneCas.get(VIEW_COLOR_IMAGE_HD, color);

    
  rs::SceneCas cas1(*cas);

  std::vector<rs::Plane> planes;
  rs::Scene scene = cas1.getScene();
  
  pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices());
  
  scene.annotations.filter(planes);

  if(planes.empty())
    {
      outInfo("NO PLANE COEFFICIENTS SET!! RUN A PLANE ESIMTATION BEFORE!!!");
      exist_plane_pcl = false;
    }

  plane_coefficients->values = planes[0].model();
  plane_inliers->indices = planes[0].inliers();
  
  
  if(plane_coefficients->values.empty())
	{ 
 	   exist_plane_pcl = false;
        }
  else
  	{
           if (plane_inliers->indices.size() > 10000 ) number_of_points_pcl = true;
           else number_of_points_pcl = false;
  	}


  engine.overwriteParam("PlaneAnnotator","plane_estimation_mode",std::string("MPS"));
  engine.reconfigure();

  engine.process();
  cas = engine.getCas();
  rs::SceneCas sceneCas2(*cas);

  sceneCas2.get(VIEW_CLOUD, *cloud_ptr);
  sceneCas2.get(VIEW_NORMALS, *cloud_normals);
  sceneCas2.get(VIEW_COLOR_IMAGE_HD, color);

    
  rs::SceneCas cas2(*cas);
  scene = cas2.getScene();
  scene.annotations.filter(planes);

  if(planes.empty())
    {
      outInfo("NO PLANE COEFFICIENTS SET!! RUN A PLANE ESIMTATION BEFORE!!!");
      exist_plane_mps = false;
    }

  plane_coefficients->values = planes[0].model();
  plane_inliers->indices = planes[0].inliers();
  
  
  if(plane_coefficients->values.empty())
	{ 
 	   exist_plane_mps = false;
        }
  else
  	{
           if (plane_inliers->indices.size() > 10000 ) number_of_points_mps = true;
           else number_of_points_mps = false;
  	}


  engine.getAnnotatorContext().releaseCAS(*cas);
  engine.collectionProcessComplete();
}
TEST(UnitTest, CheckExistingPlanePCL)
{
   planeTest();
   EXPECT_EQ(1,exist_plane_pcl);
}
TEST(UnitTest, CheckPlaneNumberOfPointsPCL)
{
   EXPECT_EQ(1,number_of_points_pcl);
}
TEST(UnitTest, CheckExistingPlaneMPS)
{
   EXPECT_EQ(1,exist_plane_mps);
}
TEST(UnitTest, CheckPlaneNumberOfPointsMPS)
{
   EXPECT_EQ(1,number_of_points_mps);
}
