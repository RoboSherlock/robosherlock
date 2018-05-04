#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/shot_omp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

typedef pcl::SHOT352 FeatureT;

class FeatureCloud
{
public:
  // A bit of shorthand
  typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;
  typedef pcl::PointCloud<pcl::Normal> SurfaceNormal;
  typedef pcl::PointCloud<FeatureT> LocalFeatures;
  typedef pcl::search::KdTree<pcl::PointXYZRGBA> SearchMethod;

  FeatureCloud();
  ~FeatureCloud();

  // Process the given cloud
  void setInputCloud(PointCloud::Ptr xyz);
  // Load and process the cloud in the given PCD file
  void loadInputCloud(const std::string &pcd_file);
  // Get a pointer to the cloud 3D points
  PointCloud::Ptr getPointCloud() const;
  // Get a pointer to the cloud of 3D surface normals
  pcl::PointCloud<pcl::Normal>::Ptr getSurfaceNormals() const;
  // Get a pointer to the cloud of feature descriptors
  LocalFeatures::Ptr getLocalFeatures() const;

protected:
  // Compute the surface normals and local features
  void processInput();
  //downsample the input
  void downsample();
  // Compute the surface normals
  void computeSurfaceNormals();
  // Compute the local feature descriptors
  void computeLocalFeatures();

private:
  // Point cloud data
  PointCloud::Ptr cloud_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  LocalFeatures::Ptr features_;
  SearchMethod::Ptr search_method_;

  // Parameters
  float normal_radius_;
  float feature_radius_;
};

class TemplateAlignment
{
public:
  // A struct for storing alignment results
  struct Result
  {
    float fitness_score;
    Eigen::Matrix4f final_transformation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  TemplateAlignment();
  ~TemplateAlignment();

  // Set the given cloud as the target to which the templates will be aligned
  void setTargetCloud(FeatureCloud &target_cloud);
  // Add the given cloud to the list of template clouds
  void addTemplateCloud(FeatureCloud &template_cloud);
  // Align the given template cloud to the target specified by setTargetCloud ()
  void align(FeatureCloud &template_cloud, TemplateAlignment::Result &result);
  // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
  void alignAll(std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results);
  // Align all of template clouds to the target cloud to find the one with best alignment score
  int findBestAlignment(TemplateAlignment::Result &result);

private:
  // A list of template clouds and the target to which they will be aligned
  std::vector<FeatureCloud> templates_;
  FeatureCloud target_;

  // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGBA, pcl::PointXYZRGBA, FeatureT> sac_ia_;
  float min_sample_distance_;
  float max_correspondence_distance_;
  int nr_iterations_;
};
