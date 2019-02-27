//
// Created by alexander on 20.09.18.
//
/**
 * Author: Alexander Link <link@uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// UIMA
#include <uima/api.hpp>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

// RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/DrawingAnnotator.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>

#include <pcl/io/pcd_io.h>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/impl/tracking.hpp>

using namespace cv;
using namespace std;
using namespace uima;
using namespace pcl::tracking;

typedef pcl::PointXYZ RefPointType;
typedef ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<RefPointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

CloudPtr cloud_pass_;
CloudPtr cloud_pass_downsampled_;
CloudPtr target_cloud;

boost::mutex mtx_;
boost::shared_ptr<ParticleFilter> tracker_;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

class PCLParticleTrackingAnnotator : public DrawingAnnotator {
private:
  CloudPtr input_cloud;
  double pointSize;
  int counter = 0;
public:
  PCLParticleTrackingAnnotator() : DrawingAnnotator(__func__), pointSize(1) {
    //cv::initModule_nonfree();
  }

  //Filter along a specified dimension
  void filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
  {
    pcl::PassThrough<RefPointType> pass;
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 10.0);
    pass.setKeepOrganized (false);
    pass.setInputCloud (cloud);
    pass.filter (result);
  }

  void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size)
  {
    pcl::ApproximateVoxelGrid<RefPointType> grid;
    grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
    grid.setInputCloud (cloud);
    grid.filter (result);
  }

  void track(const CloudConstPtr &input_cloud)
  {
    boost::mutex::scoped_lock lock (mtx_);
    cloud_pass_.reset (new Cloud);
    cloud_pass_downsampled_.reset (new Cloud);
    filterPassThrough (input_cloud, *cloud_pass_);
    gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, 0.005);
    outInfo("Input cloud size after filtering is " + std::to_string(cloud_pass_downsampled_->size()));

    if(counter < 0){ // Changed 10 to 0 for testing
      counter++;
    }else{
      //Track the object
      tracker_->setInputCloud (cloud_pass_downsampled_);
      tracker_->compute();
    }
  }

  TyErrorId initialize(AnnotatorContext &ctx) {
    outInfo("initialize");
    target_cloud.reset(new Cloud());
    if (pcl::io::loadPCDFile("/home/alex/tracking/SeverinPancakeMaker_5mm.pcd", *target_cloud) == -1) {
      outError(".pcd-file not found!");
      return UIMA_ERR_NONE;
    }
    std::vector<double> default_step_covariance = std::vector<double>(6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;

    std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);

    boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
            (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>(8));

    ParticleT bin_size;
    bin_size.x = 0.1f;
    bin_size.y = 0.1f;
    bin_size.z = 0.1f;
    bin_size.roll = 0.1f;
    bin_size.pitch = 0.1f;
    bin_size.yaw = 0.1f;


    //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
    tracker->setMaximumParticleNum(1000);
    tracker->setDelta(0.99);
    tracker->setEpsilon(0.2);
    tracker->setBinSize(bin_size);

    //Set all parameters for  ParticleFilter
    tracker_ = tracker;
    tracker_->setTrans(Eigen::Affine3f::Identity());
    tracker_->setStepNoiseCovariance(default_step_covariance);
    tracker_->setInitialNoiseCovariance(initial_noise_covariance);
    tracker_->setInitialNoiseMean(default_initial_mean);
    tracker_->setIterationNum(1);
    tracker_->setParticleNum(600);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal(false);


    //Setup coherence object for tracking
    ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
            (new ApproxNearestPairPointCloudCoherence<RefPointType>());

    boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
            = boost::shared_ptr<DistanceCoherence<RefPointType> >(new DistanceCoherence<RefPointType>());
    coherence->addPointCoherence(distance_coherence);

    boost::shared_ptr<pcl::search::Octree<RefPointType> > search(new pcl::search::Octree<RefPointType>(0.01));
    coherence->setSearchMethod(search);
    coherence->setMaximumDistance(0.01);

    tracker_->setCloudCoherence(coherence);

    //prepare the model of tracker's target
    Eigen::Vector4f c;
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    CloudPtr transed_ref(new Cloud);
    CloudPtr transed_ref_downsampled (new Cloud);

    pcl::compute3DCentroid<RefPointType>(*target_cloud, c);
    trans.translation().matrix() = Eigen::Vector3f(c[0], c[1], c[2]);
    pcl::transformPointCloud<RefPointType>(*target_cloud, *transed_ref, trans.inverse());
    // Downsampling with gridSampleApprox should not be necessary because the pcd is already donsampled.
    gridSampleApprox (transed_ref, *transed_ref_downsampled, 0.005);
    outInfo("Target cloud size is " + std::to_string(transed_ref->size()));

    //set reference model and trans
    tracker_->setReferenceCloud(transed_ref_downsampled);
    tracker_->setTrans(trans);

    return UIMA_ERR_NONE;
  }

  TyErrorId reconfigure()
  {
    outInfo("Reconfiguring");
    AnnotatorContext &ctx = getAnnotatorContext();
    initialize(ctx);
    return UIMA_ERR_NONE;
  }

  // Destroys annotator
  TyErrorId destroy()
  {
    outInfo("destroy");

    return UIMA_ERR_NONE;
  }

  // Processes a frame
  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    rs::StopWatch clock;
    outInfo("process begins");
    rs::SceneCas cas(tcas);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGBA>); // Input data for 3D tracking
    CloudPtr input_cloud_nan(new Cloud); // Input data for 3D tracking
    CloudPtr input_cloud(new Cloud); // Input data for 3D tracking
    cas.get(VIEW_CLOUD, *input_cloud_rgb); // Fill input data for 3D tracking
    pcl::copyPointCloud(*input_cloud_rgb, *input_cloud_nan);
    std::vector<int> input_indices;
    pcl::removeNaNFromPointCloud(*input_cloud_nan, *input_cloud, input_indices);
    if(!input_cloud->size() > 0) {
      outError("Input cloud is empty.");
    }
    else {
      outInfo("Input cloud size is " + std::to_string(input_cloud->size()));
      if (!target_cloud->size() > 0) {
        outError("Target cloud is empty.");
      }
      else {
        CloudConstPtr test_ref = tracker_->getReferenceCloud();
        outInfo("Target cloud size is " + std::to_string(test_ref->size()));
        outInfo(input_cloud->points[0].x);
        outInfo(input_cloud->points[0].y);
        outInfo(input_cloud->points[0].z);
        outInfo(input_cloud->points[4].x);
        outInfo(input_cloud->points[4].y);
        outInfo(input_cloud->points[4].z);
        outInfo(input_cloud->points[20].x);
        outInfo(input_cloud->points[20].y);
        outInfo(input_cloud->points[20].z);
        outInfo(input_cloud->points[40].x);
        outInfo(input_cloud->points[40].y);
        outInfo(input_cloud->points[40].z);

        track(input_cloud);


        // ------------------------------------------------------------------------------- //
        ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles();
        if (particles && input_cloud) {
          //Set pointCloud with particle's points
          pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
          for (size_t i = 0; i < particles->points.size(); i++) {
            pcl::PointXYZ point;

            point.x = particles->points[i].x;
            point.y = particles->points[i].y;
            point.z = particles->points[i].z;
            particle_cloud->points.push_back(point);
          }

          const std::string &cloudname = this->name;
          outInfo("Amount of points in result particle cloud: " + std::to_string(particle_cloud->size()));
        }
        // ------------------------------------------------------------------------------- //
      }
    }
    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp){
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun) {
    ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
    if (!particles->size() > 0) { // TODO: Before going in counter loop: Segfault here because particles is null
      outError("Particle result cloud is empty.");
    }
    else {
      outInfo("Particle result cloud is fine.");
      pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      for (size_t i = 0; i < particles->points.size(); i++) {
        pcl::PointXYZ point;

        point.x = particles->points[i].x;
        point.y = particles->points[i].y;
        point.z = particles->points[i].z;
        particle_cloud->points.push_back(point);
      }

      pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> result_color(particle_cloud, 255, 255, 255);
      pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> cloud_color(particle_cloud, 255, 40, 20);
      //if (!visualizer.updatePointCloud (particle_cloud, red_color, "particle cloud"))
      //  visualizer.addPointCloud (particle_cloud, red_color, "particle cloud");


      const std::string &cloudname = this->name;
      /**
      outInfo("Attempting to update visualizer...");
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_xyz;
      outInfo("1");
      for(int n = 0; n < input_cloud->size(); n++){ // TODO: Once the PCL error spam occurs: Segfault here, smth wrong with input_cloud
        outInfo("2.1");
        pcl::PointXYZ point;
        outInfo("2.2");
        point.x = input_cloud->points[n].x;
        point.y = input_cloud->points[n].y;
        point.z = input_cloud->points[n].z;
        outInfo("2.3");
        input_cloud_xyz->push_back(point);
      }
      //pcl::copyPointCloud(*input_cloud, *input_cloud_xyz);
       **/
      outInfo("test");
      if (firstRun) {
        visualizer.addPointCloud(particle_cloud, result_color, cloudname);
        visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize,
                                                    cloudname);
        //visualizer.addPointCloud(input_cloud_xyz, cloud_color, "tracking_input_cloud");
        //visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize,
        //                                            "tracking_input_cloud");
      } else {
        pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*particle_cloud, *particle_cloud_filtered, indices);
        outInfo("Updating visualizer cloud with " + std::to_string(particle_cloud_filtered->size()) + " points!");
        outInfo(particle_cloud_filtered->points[0].x);
        outInfo(particle_cloud_filtered->points[0].y);
        outInfo(particle_cloud_filtered->points[0].z);
        outInfo(particle_cloud_filtered->points[4].x);
        outInfo(particle_cloud_filtered->points[4].y);
        outInfo(particle_cloud_filtered->points[4].z);
        visualizer.updatePointCloud(particle_cloud_filtered, result_color, cloudname);
        //visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
        //visualizer.updatePointCloud(input_cloud_xyz, cloud_color, "tracking_input_cloud");
      }
    }
    return;
  }
};

MAKE_AE(PCLParticleTrackingAnnotator)
