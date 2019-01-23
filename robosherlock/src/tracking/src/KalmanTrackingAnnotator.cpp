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
    bool firstExecution = true;
    int debugCounter = 0;
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

            if(firstExecution){
                outInfo("Initializing tracker using current object hypothesis in the scene...");
                tracker->init(frame, bbox);
                outInfo("...tracker initialized successfully!");
                firstExecution = false;
            }
            else {
                outInfo("Updating the tracker...");
                tracker->update(frame, bbox);
                outInfo("...tracker updated successfully!");

                if(debugCounter < 3){
                    debugCounter++;
                }
                else{
                    rectangle( frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
                    imshow("tracker",frame);
                    if(waitKey(1)==27)return UIMA_ERR_NONE;
                }
                // Result visualization
                /**
                rectangle( frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
                imshow("tracker",frame);
                if(waitKey(1)==27)return UIMA_ERR_NONE;
                 **/

                // bbox position debug output
                outInfo("x: " + std::to_string(bbox.x));
                outInfo("y: " + std::to_string(bbox.y));
                outInfo("width: " + std::to_string(bbox.width));
                outInfo("height: " + std::to_string(bbox.height));
            }
        }
        return UIMA_ERR_NONE;
    }
};

MAKE_AE(TrackingAnnotator)
