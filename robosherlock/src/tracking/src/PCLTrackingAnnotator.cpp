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

typedef pcl::PointXYZRGBA RefPointType;
typedef ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

class TrackingAnnotator : public DrawingAnnotator
{
private:
    bool pclStarted;
    cv::Mat depthImage;
    boost::shared_ptr<ParticleFilterTracker<pcl::PointXYZRGBA, ParticleXYZRPY>> tracker_;
    CloudPtr cloud_pass_;
    CloudPtr cloud_pass_downsampled_;
    Cloud::Ptr target_cloud;
    Cloud::Ptr cloud; // Input data for 3D tracking
    Cloud::Ptr objectCloud; // Loaded PCD file for 3D tracking
    cv::Mat frame; // Input data for 2D tracking
    cv::Rect roi; // Region of interest
    Rect2d bbox; // Could later be used for the bounding box query parameter.
    boost::mutex mtx_;
    bool new_cloud_;
    double downsampling_grid_size_;
    int counter;
public:
    TrackingAnnotator() : DrawingAnnotator(__func__)
    {
        //cv::initModule_nonfree();
    }

    /*
     * Initializes annotator
     */
    TyErrorId initialize(AnnotatorContext &ctx) {
        outInfo("initialize");

        /**
        // TODO: Check and extract ctx parameters

        #if (CV_MINOR_VERSION < 3)
        {
            tracker = Tracker::create(KCF);
        }
        #else
        {
            tracker = TrackerKCF::create();
        }
        #endif

         **/
        
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
        MEASURE_TIME;
        outInfo("process begins");

        rs::SceneCas cas(tcas);
        cas.get(VIEW_CLOUD, *cloud); // Fill input data for 3D tracking

        PCLTracker();

        return UIMA_ERR_NONE;
    }

    //Filter along a specified dimension
    void filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
    {
        pcl::PassThrough<pcl::PointXYZRGBA> pass;
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, 10.0);
        pass.setKeepOrganized (false);
        pass.setInputCloud (cloud);
        pass.filter (result);
    }

    void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size)
    {
        pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
        grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
        grid.setInputCloud (cloud);
        grid.filter (result);
    }

    //OpenNI Grabber's cloud Callback function
    void
    cloud_cb (const CloudConstPtr &cloud)
    {
        boost::mutex::scoped_lock lock (mtx_);
        cloud_pass_.reset (new Cloud);
        cloud_pass_downsampled_.reset (new Cloud);
        filterPassThrough (cloud, *cloud_pass_);
        gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);

        if(counter < 10){
            counter++;
        }else{
            //Track the object
            tracker_->setInputCloud (cloud_pass_downsampled_);
            tracker_->compute ();
            new_cloud_ = true;
        }
    }

    // TODO: Where to get the .pcd file from?
    bool PCLTracker()
    {
        if(!pclStarted) {
            objectCloud.reset(new Cloud());
            if(pcl::io::loadPCDFile ("path/to/pcd/file", *objectCloud) == -1){
                std::cout << "pcd file not found" << std::endl;
                return false;
            }


            int counter = 0;

            //Set parameters
            bool new_cloud_  = false;
            float downsampling_grid_size_ =  0.002;

            std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
            default_step_covariance[3] *= 40.0;
            default_step_covariance[4] *= 40.0;
            default_step_covariance[5] *= 40.0;

            std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
            std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

            boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBA, ParticleXYZRPY> > tracker
                    (new KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBA, ParticleXYZRPY> (8));

            ParticleXYZRPY bin_size;
            bin_size.x = 0.1f;
            bin_size.y = 0.1f;
            bin_size.z = 0.1f;
            bin_size.roll = 0.1f;
            bin_size.pitch = 0.1f;
            bin_size.yaw = 0.1f;


            //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
            tracker->setMaximumParticleNum (1000);
            tracker->setDelta (0.99);
            tracker->setEpsilon (0.2);
            tracker->setBinSize (bin_size);

            //Set all parameters for  ParticleFilter
            tracker_ = tracker;
            tracker_->setTrans (Eigen::Affine3f::Identity ());
            tracker_->setStepNoiseCovariance (default_step_covariance);
            tracker_->setInitialNoiseCovariance (initial_noise_covariance);
            tracker_->setInitialNoiseMean (default_initial_mean);
            tracker_->setIterationNum (1);
            tracker_->setParticleNum (600);
            tracker_->setResampleLikelihoodThr(0.00);
            tracker_->setUseNormal (false);


            //Setup coherence object for tracking
            ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBA>::Ptr coherence = ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBA>::Ptr
                    (new ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBA> ());

            boost::shared_ptr<DistanceCoherence<pcl::PointXYZRGBA> > distance_coherence
                    = boost::shared_ptr<DistanceCoherence<pcl::PointXYZRGBA> > (new DistanceCoherence<pcl::PointXYZRGBA> ());
            coherence->addPointCoherence (distance_coherence);

            boost::shared_ptr<pcl::search::Octree<pcl::PointXYZRGBA> > search (new pcl::search::Octree<pcl::PointXYZRGBA> (0.01));
            coherence->setSearchMethod (search);
            coherence->setMaximumDistance (0.01);

            tracker_->setCloudCoherence (coherence);

            //prepare the model of tracker's target
            Eigen::Vector4f c;
            Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
            Cloud::Ptr transed_ref (new Cloud);
            Cloud::Ptr transed_ref_downsampled (new Cloud);

            pcl::compute3DCentroid<pcl::PointXYZRGBA> (*target_cloud, c);
            trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
            pcl::transformPointCloud<pcl::PointXYZRGBA> (*target_cloud, *transed_ref, trans.inverse());
            gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);

            //set reference model and trans
            tracker_->setReferenceCloud (transed_ref_downsampled);
            tracker_->setTrans (trans);

            //Setup OpenNIGrabber and viewer
            pcl::visualization::CloudViewer* viewer_ = new pcl::visualization::CloudViewer("PCL OpenNI Tracking Viewer");
            pcl::Grabber* interface = new pcl::OpenNIGrabber ("#1");
            boost::function<void (const CloudConstPtr&)> f =
                    boost::bind (&cloud_cb, _1); // TODO: taking an address & isn't allowed inside of a parenthesis
            interface->registerCallback (f);

            //Start viewer and object tracking
            interface->start();
            //interface->stop();



            pclStarted = true;
        }
    }
};

MAKE_AE(TrackingAnnotator)
