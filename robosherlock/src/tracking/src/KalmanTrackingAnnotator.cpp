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

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

class TrackingAnnotator : public DrawingAnnotator
{
private:
    Ptr<Tracker> tracker = TrackerKCF::create();
    bool kcfStarted;
    bool pclStarted;
    bool hasDepth;
    cv::Mat depthImage;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud; // Input data for 3D tracking
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objectCloud; // Loaded PCD file for 3D tracking
    cv::Mat frame; // Input data for 2D tracking
    cv::Rect roi; // Region of interest
    Rect2d bbox; // Could later be used for the bounding box query parameter.
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
        cas.get(VIEW_COLOR_IMAGE, frame); // Fill input data for 2D tracking

        //rs::ImageROI image_rois = clusters[idx].rois.get(); TODO
        //rs::conversion::from(image_rois.roi_hires(), roi);


        // TODO: Don't run PCL tracker if we don't have an object .pcd file.
        if(!kcfStarted && !pclStarted) {
            hasDepth = cas.get(VIEW_DEPTH_IMAGE, depthImage);
            if (hasDepth) {
                PCLTracker();
            }
            else {
                KCFTracker();
            }
        }

        return UIMA_ERR_NONE;
    }

    bool KCFTracker()
    {
        if(!kcfStarted) {
            // Define bounding box. Could later be overriden by parameter.
            Rect2d bbox(0, 0, 200, 200);

            // Initializes tracker
            tracker->init(frame, bbox);

            kcfStarted = true;
        }

        // Update the tracking result
        tracker->update(frame, bbox);
    }

    // TODO: Where to get the .pcd file from?
    bool PCLTracker()
    {
        if(!pclStarted) {
            objectCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
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
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transed_ref (new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transed_ref_downsampled (new pcl::PointCloud<pcl::PointXYZRGBA>);

            pcl::compute3DCentroid<pcl::PointXYZRGBA> (*target_cloud, c);
            trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
            pcl::transformPointCloud<pcl::PointXYZRGBA> (*target_cloud, *transed_ref, trans.inverse());
            gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);

            //set reference model and trans
            tracker_->setReferenceCloud (transed_ref_downsampled);
            tracker_->setTrans (trans);

            //Setup OpenNIGrabber and viewer
            pcl::visualization::CloudViewer* viewer_ = new pcl::visualization::CloudViewer("PCL OpenNI Tracking Viewer");
            pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id);
            boost::function<void (const CloudConstPtr&)> f =
                    boost::bind (&cloud_cb, _1);
            interface->registerCallback (f);

            viewer_->runOnVisualizationThread (boost::bind(&viz_cb, _1), "viz_cb");

            //Start viewer and object tracking
            interface->start();
            while (!viewer_->wasStopped ())
                boost::this_thread::sleep(boost::posix_time::seconds(1));
            interface->stop();



            pclStarted = true;
        }
    }
};

MAKE_AE(TrackingAnnotator)
