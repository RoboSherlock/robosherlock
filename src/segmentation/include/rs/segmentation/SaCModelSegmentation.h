#pragma once

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


namespace rs
{
namespace rs_pcl
{
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class SacModelSegmentation
{
public:

  class Config
  {
  public:
    bool useNormals;
    bool downsample;

    int sacModel;
    int minPoints;

    float minOpeningAngle;
    float maxOpeningAngle;
    float radiusLimit1;
    float radiusLimit2;
    float distanceWeight;
    float distanceThreshold;
    float downsampleLeafSize;
  };


  std::vector< pcl::PointIndices> modelsIndices;
  Config config;

  SacModelSegmentation()
  {

  }

  void configure(const Config &config);

  /**
   * \brief this method detect the shapes of multiple point clouds
   * \param inputCluster cluster of point clouds
   * \param cloud_normals the normals of the inputCloud
   */
  bool detect(PointCloudT::Ptr inputCloud,
              pcl::PointCloud< pcl::Normal>::Ptr cloud_normals);

private:

  pcl::SACSegmentationFromNormals< PointT, pcl::Normal> segNormal;
  pcl::SACSegmentation< PointT> seg;
  // Datasets
  pcl::ModelCoefficients::Ptr coefficients;
  pcl::ExtractIndices< PointT> extractIndices;
  pcl::ExtractIndices< pcl::Normal> extractIndicesFromNormals;

  /**
   * \brief calculate the normals
   * \param cloud point cloud
   * \param cloud_normals resulting normals
   */
  bool estimateNormals(const PointCloudT::Ptr &cloud,
                       pcl::PointCloud< pcl::Normal>::Ptr &cloud_normals);

};
}
}

