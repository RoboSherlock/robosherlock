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


#include <robosherlock_msgs/RSObjectDescriptions.h>

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
#include <pcl/filters/extract_indices.h>

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

class PCLParticleTrackingAnnotator : public DrawingAnnotator
{
private:
  CloudPtr input_cloud;
  double point_size;
  bool first_execution = true;
  std::vector <rs::ObjectHypothesis> clusters;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud_rgb;
  ParticleFilter::PointCloudStatePtr particles;
  CloudPtr particle_cloud;
public:
  PCLParticleTrackingAnnotator() : DrawingAnnotator(__func__), point_size(1), input_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGBA>), particle_cloud(new Cloud())
  {
    //cv::initModule_nonfree();
  }

  ros::NodeHandle nh_;
  ros::Publisher result_pub;

  //Filter along a specified dimension
  void filterPassThrough (const CloudConstPtr &CLOUD, Cloud &result)
  {
    pcl::PassThrough<RefPointType> pass;
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 10.0);
    pass.setKeepOrganized (false);
    pass.setInputCloud (CLOUD);
    pass.filter (result);
  }

  void gridSampleApprox (const CloudConstPtr &CLOUD, Cloud &result, double leaf_size)
  {
    pcl::ApproximateVoxelGrid<RefPointType> grid;
    grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
    grid.setInputCloud (CLOUD);
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

    tracker_->setInputCloud (cloud_pass_downsampled_);
    tracker_->compute();
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    return UIMA_ERR_NONE;
  }

  TyErrorId reconfigure()
  {
    outInfo("Reconfiguring");
    first_execution = true;
    return UIMA_ERR_NONE;
  }

  // Destroys annotator
  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  // Processes a frame
  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &RES_SPEC)
  {

    rs::StopWatch clock;
    outInfo("process begins");
    rs::SceneCas cas(tcas);

    CloudPtr input_cloud(new Cloud); // Input data for 3D tracking
    cas.get(VIEW_CLOUD, *input_cloud_rgb); // Fill input data for 3D tracking
    pcl::copyPointCloud(*input_cloud_rgb, *input_cloud);
    std::vector<int> input_indices;
    if(!input_cloud->size() > 0)
    {
      outError("Input cloud is empty.");
    }
    else
    {
      outInfo("Input cloud size is " + std::to_string(input_cloud->size()));
      if (first_execution)
      {
        ros::NodeHandle nh_("~"); // Set correct namespace
        result_pub = nh_.advertise<robosherlock_msgs::RSObjectDescriptions>(std::string("result_advertiser"), 1);

        target_cloud.reset(new Cloud());
        rs::Scene scene = cas.getScene();
        scene.identifiables.filter(clusters);

        rs::Size s = rs::create<rs::Size>(tcas); // a hack to get a simple integer (the object ID) from the cas.
        outInfo(FG_GREEN << "GETTING OBJ_TO_TRACK");
        if (!cas.getFS("OBJ_ID_TRACK", s))
        {
          outError("Please set OBJ_TO_TRACK before processing with KCFTrackingAnnotator for the first time.");
          return UIMA_ERR_NONE;
        }
        int obj_id = s.height.get();

        if (clusters.size() <= obj_id)
        {
          outError("An object of id " + std::to_string(obj_id) + " does not exist. "
                                                                 "There are only " + std::to_string(clusters.size())
                   + " potential objects in the scene.");
          return UIMA_ERR_NONE;
        }

        // Get current 3D view from CAS

        // Make point cloud from target object hypothesis
        rs::ObjectHypothesis &cluster = clusters[obj_id];
        if (!cluster.points.has())
        {
          outError("Target cluster has no points.");
          return UIMA_ERR_NONE;
        }
        pcl::PointIndicesPtr indices(new pcl::PointIndices());
        rs::conversion::from(static_cast<rs::ReferenceClusterPoints>(cluster.points.get()).indices.get(), *indices);
        pcl::ExtractIndices <RefPointType> ei;
        ei.setInputCloud(input_cloud);
        ei.setIndices(indices);
        ei.filter(*target_cloud);

        std::vector<double> default_step_covariance = std::vector<double>(6, 0.015 * 0.015);
        default_step_covariance[3] *= 40.0;
        default_step_covariance[4] *= 40.0;
        default_step_covariance[5] *= 40.0;

        std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.00001);
        std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);

        boost::shared_ptr <KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>> tracker
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

        boost::shared_ptr <DistanceCoherence<RefPointType>> distance_coherence
                = boost::shared_ptr < DistanceCoherence < RefPointType > > (new DistanceCoherence<RefPointType>());
        coherence->addPointCoherence(distance_coherence);

        boost::shared_ptr <pcl::search::Octree<RefPointType>> search(new pcl::search::Octree<RefPointType>(0.01));
        coherence->setSearchMethod(search);
        coherence->setMaximumDistance(0.01);

        tracker_->setCloudCoherence(coherence);

        //prepare the model of tracker's target
        Eigen::Vector4f c;
        Eigen::Affine3f trans = Eigen::Affine3f::Identity();
        CloudPtr transed_ref(new Cloud);
        CloudPtr transed_ref_downsampled(new Cloud);

        pcl::compute3DCentroid<RefPointType>(*target_cloud, c);
        trans.translation().matrix() = Eigen::Vector3f(c[0], c[1], c[2]);
        pcl::transformPointCloud<RefPointType>(*target_cloud, *transed_ref, trans.inverse());
        // Downsampling with gridSampleApprox should not be necessary because the pcd is already donsampled.
        gridSampleApprox(transed_ref, *transed_ref_downsampled, 0.005);
        outInfo("Target cloud size is " + std::to_string(transed_ref->size()));

        //set reference model and trans
        tracker_->setReferenceCloud(transed_ref_downsampled);
        tracker_->setTrans(trans);
        first_execution = false;
      }
      else
      {
        if (!target_cloud->size() > 0)
        {
          outError("Target cloud is empty.");
        }
        else
        {
          CloudConstPtr test_ref = tracker_->getReferenceCloud();
          outInfo("Target cloud size is " + std::to_string(test_ref->size()));

          track(input_cloud);

          particles = tracker_->getParticles ();
          if(particles && input_cloud)
          {
            particle_cloud->clear();
            for (size_t i = 0; i < particles->points.size(); i++)
            {
              pcl::PointXYZ point;
              point.x = particles->points[i].x;
              point.y = particles->points[i].y;
              point.z = particles->points[i].z;
              particle_cloud->points.push_back(point);
            }

            outInfo("Amount of points in result particle cloud: " + std::to_string(particle_cloud->size()));

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*particle_cloud, centroid);

            robosherlock_msgs::RSObjectDescriptions result_message;
            std::vector<std::string> result_response;
            result_response.push_back("x: " + std::to_string(centroid[0]));
            result_response.push_back("y: " + std::to_string(centroid[1]));
            result_response.push_back("z: " + std::to_string(centroid[2]));
            result_message.obj_descriptions = result_response;
            result_pub.publish(result_message);
          }
        }
      }
    }
    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool FIRST_RUN)
  {
    if (!particles->size() > 0)
    {
      outError("Particle result cloud is empty.");
    }
    else
    {
      pcl::visualization::PointCloudColorHandlerCustom<RefPointType> result_color(particle_cloud, 255, 255, 255);

      const std::string &CLOUDNAME = this->name;
      if (FIRST_RUN)
      {
        visualizer.addPointCloud(particle_cloud, result_color, CLOUDNAME);

        visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size,
                                                    CLOUDNAME);
        visualizer.addPointCloud(input_cloud_rgb, "original_cloud");
        visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size,"original_cloud");
      }
      else
      {
        outInfo("Updating visualizer cloud with " + std::to_string(particle_cloud->size()) + " points!");
        visualizer.updatePointCloud(particle_cloud, result_color, CLOUDNAME);
        visualizer.updatePointCloud(input_cloud_rgb, "original_cloud");
      }
    }
    return;
  }
};

MAKE_AE(PCLParticleTrackingAnnotator)
