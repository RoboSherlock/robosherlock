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
    cv::Mat depthImage;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud; // Input data for 3D tracking
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objectCloud; // Loaded PCD file for 3D tracking
    cv::Mat frame; // Input data for 2D tracking
    cv::Rect roi; // Region of interest
    Rect2d bbox; // Could later be used for the bounding box query parameter.
    std::vector<rs::Cluster> clusters;
    std::vector<cv::Rect> clusterRois;
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
        rs::Scene scene = cas.getScene();
        cas.get(VIEW_COLOR_IMAGE, frame); // Fill input data for 2D tracking

        // Get regions of interest. TODO: Which one do we need now?
        scene.identifiables.filter(clusters);
        clusterRois.resize(clusters.size());
        for(size_t idx = 0; idx < clusters.size(); ++idx) {
            rs::ImageROI image_rois = clusters[idx].rois.get();

            cv::Rect roi;
            rs::conversion::from(image_rois.roi_hires(), roi);

            clusterRois[idx] = roi;
        }

        KCFTracker(clusterRois[0]); // Use the first object for now.


        /** TODO: Now that the PCL tracker has its own Annotator, decision on which one to run has to happen
         * somewhere else, most likely in Prolog (?)
        if(!kcfStarted && !pclStarted) {
            hasDepth = cas.get(VIEW_DEPTH_IMAGE, depthImage);
            if (hasDepth) {
                PCLTracker();
            }
            else {
                KCFTracker();
            }
        }
         **/

        return UIMA_ERR_NONE;
    }

    bool KCFTracker(cv::Rect roi) {
        if (!kcfStarted) {
            // Define bounding box. Could later be overriden by parameter.
            Rect2d bbox(0, 0, 200, 200);

            // Initializes tracker
            tracker->init(frame, roi);

            kcfStarted = true;
        }

        // Update the tracking result
        tracker->update(frame, bbox);
    }
};

MAKE_AE(TrackingAnnotator)
