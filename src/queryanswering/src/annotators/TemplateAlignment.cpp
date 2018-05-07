#include <rs_queryanswering/annotators/TemplateAlignment.h>

FeatureCloud::FeatureCloud() :
  search_method_(new SearchMethod),
  normal_radius_(0.01f),
  feature_radius_(0.04f)
{
}

FeatureCloud::~FeatureCloud()
{
}

// Process the given cloud
void
FeatureCloud::setInputCloud(PointCloud::Ptr xyz)
{
  cloud_ = xyz;
  processInput();
}

// Load and process the cloud in the given PCD file
void
FeatureCloud::loadInputCloud(const std::string &pcd_file)
{
  cloud_ = PointCloud::Ptr(new PointCloud);
  pcl::io::loadPCDFile(pcd_file, *cloud_);
  processInput();
}

// Get a pointer to the cloud 3D points
FeatureCloud::PointCloud::Ptr FeatureCloud::getPointCloud() const
{
  return (cloud_);
}

// Get a pointer to the cloud of 3D surface normals
pcl::PointCloud<pcl::Normal>::Ptr FeatureCloud::getSurfaceNormals() const
{
  return (normals_);
}

// Get a pointer to the cloud of feature descriptors
FeatureCloud::LocalFeatures::Ptr FeatureCloud::getLocalFeatures() const
{
  return (features_);
}

// Compute the surface normals and local features
void FeatureCloud::processInput()
{
//  downsample();
  computeSurfaceNormals();
  computeLocalFeatures();
}

void FeatureCloud::downsample()
{
    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setInputCloud(cloud_);
    vg.setLeafSize(0.005f,0.005f,0.005f);
    vg.filter(*cloud_);
}

// Compute the surface normals
void FeatureCloud::computeSurfaceNormals()
{
  normals_ = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

  pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> norm_est;
  norm_est.setInputCloud(cloud_);
  norm_est.setSearchMethod(search_method_);
  norm_est.setRadiusSearch(normal_radius_);
  norm_est.compute(*normals_);
}

// Compute the local feature descriptors
void FeatureCloud::computeLocalFeatures()
{
  features_ = LocalFeatures::Ptr(new LocalFeatures);
  pcl::SHOTEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, FeatureT> shot_est;
  shot_est.setInputCloud(cloud_);
  shot_est.setInputNormals(normals_);
  shot_est.setSearchMethod(search_method_);
  shot_est.setRadiusSearch(feature_radius_);
  shot_est.compute(*features_);
}

TemplateAlignment::TemplateAlignment() :
  min_sample_distance_(0.001f),
  max_correspondence_distance_(0.005f * 0.005f),
  nr_iterations_(500)
{
  // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
  sac_ia_.setMinSampleDistance(min_sample_distance_);
  sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
  sac_ia_.setMaximumIterations(nr_iterations_);
  sac_ia_.setCorrespondenceRandomness(1);
}

TemplateAlignment::~TemplateAlignment()
{
}

// Set the given cloud as the target to which the templates will be aligned
void TemplateAlignment::setTargetCloud(FeatureCloud &target_cloud)
{
  target_ = target_cloud;
  sac_ia_.setInputTarget(target_cloud.getPointCloud());
  sac_ia_.setTargetFeatures(target_cloud.getLocalFeatures());
}

// Add the given cloud to the list of template clouds
void TemplateAlignment::addTemplateCloud(FeatureCloud &template_cloud)
{
  templates_.push_back(template_cloud);
}

// Align the given template cloud to the target specified by setTargetCloud ()
void TemplateAlignment::align(FeatureCloud &template_cloud, TemplateAlignment::Result &result)
{
  sac_ia_.setInputSource(template_cloud.getPointCloud());
  sac_ia_.setSourceFeatures(template_cloud.getLocalFeatures());

  pcl::PointCloud<pcl::PointXYZRGBA> registration_output;
  sac_ia_.align(registration_output);

  result.fitness_score = (float) sac_ia_.getFitnessScore(max_correspondence_distance_);
  result.final_transformation = sac_ia_.getFinalTransformation();
}

// Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
void TemplateAlignment::alignAll(std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
{
  results.resize(templates_.size());
  for(size_t i = 0; i < templates_.size(); ++i)
  {
    align(templates_[i], results[i]);
  }
}

// Align all of template clouds to the target cloud to find the one with best alignment score
int TemplateAlignment::findBestAlignment(TemplateAlignment::Result &result)
{
  // Align all of the templates to the target cloud
  std::vector<Result, Eigen::aligned_allocator<Result> > results;
  alignAll(results);

  // Find the template with the best (lowest) fitness score
  float lowest_score = std::numeric_limits<float>::infinity();
  int best_template = 0;
  for(size_t i = 0; i < results.size(); ++i)
  {
    const Result &r = results[i];
    if(r.fitness_score < lowest_score)
    {
      lowest_score = r.fitness_score;
      best_template = (int) i;
    }
  }

  // Output the best alignment
  result = results[best_template];
  return (best_template);
}
