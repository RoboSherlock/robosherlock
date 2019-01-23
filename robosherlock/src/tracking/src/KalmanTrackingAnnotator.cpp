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
    cv::Mat depthImage;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud; // Input data for 3D tracking
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objectCloud; // Loaded PCD file for 3D tracking
    cv::Mat frame; // Input data for 2D tracking
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

        // TODO: Check and extract ctx parameters

        // Default bounding box. Could be overridden by parameter. Is later overridden using region of interest.
        Rect2d bbox(0, 0, 200, 200);

        // Initializes tracker
        tracker->init(frame, bbox);
        
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

        outInfo("Receiving data from cas");
        rs::SceneCas cas(tcas);
        rs::Scene scene = cas.getScene();
        std::vector<rs::ObjectHypothesis> clusters;
        scene.identifiables.filter(clusters);
        cas.get(VIEW_COLOR_IMAGE, frame); // Fill input data for 2D tracking

        outInfo("Get the ROI of the object that is to be tracked.");
        if(clusters.size() < 1){
            outError("An object of id " + std::to_string(0) + " does not exist.");
            return UIMA_ERR_NONE;
        }
        else{
            if(clusters.size() > 1) {
                outWarn("Found more than one object in the scene. "
                        "It is recommended to run ClosestHypothesisFilter before running KalmanTrackingAnnotator. "
                        "Now tracking the object hypothesis of ID 0 by default...");
            }
            rs::ImageROI image_roi = clusters[0].rois.get();
            cv::Rect roi;
            rs::conversion::from(image_roi.roi_hires(), roi);
            Rect2d bbox(roi.x, roi.y, roi.width, roi.height); // Manual Rect to Rect2d conversion

            outInfo("Updating the tracker...");
            tracker->update(frame, bbox);
            outInfo("...tracker updated successfully!");
        }





        /** TODO: Now that the PCL tracker has its own Annotator, decision on which one to run has to happen
         *  TODO: somewhere else, most likely in Prolog (?)
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
};

MAKE_AE(TrackingAnnotator)
