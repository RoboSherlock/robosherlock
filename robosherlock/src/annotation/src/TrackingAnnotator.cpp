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
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr; // Input data for 3D tracking
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

        #if (CV_MINOR_VERSION < 3)
        {
            tracker = Tracker::create(KCF);
        }
        #else
        {
            tracker = TrackerKCF::create();
        }
        #endif
        
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

        // create necessary pcl objects
        cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

        rs::SceneCas cas(tcas);
        cas.get(VIEW_CLOUD, *cloud_ptr); // Fill input data for 3D tracking
        cas.get(VIEW_COLOR_IMAGE, frame); // Fill input data for 2D tracking

        KCFTracker(frame);

        return UIMA_ERR_NONE;
    }


    // This runs on repeat, tracking the current object.
    // Depending on if it's a good idea to have a single Annotator run for so long or not,
    // this could instead be changed to just do a single iteration, but instead be called
    // repeatedly.
    bool KCFTracker(cv::Mat frame)
    {
        // Define bounding box. Could later be overriden by parameter.
        Rect2d bbox(0, 0, 200, 200);

        // Initializes tracker
        tracker->init(frame, bbox);

        while(true)
        {
            // Start timer
            double timer = (double)getTickCount();

            // Update the tracking result
            bool ok = tracker->update(frame, bbox);

            // Calculate Frames per second (FPS)
            float fps = getTickFrequency() / ((double)getTickCount() - timer);

            if (ok)
            {
                // Tracking success : Draw the tracked object
                rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
            }
            else
            {
                // Tracking failure detected.
                putText(frame, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
            }

            // Display tracker type on frame
            putText(frame, "KCF Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);

            // Display FPS on frame
            putText(frame, "FPS : " + SSTR(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);

            // Display frame.
            imshow("Tracking", frame);

            // Exit if ESC pressed.
            int k = waitKey(1);
            if(k == 27)
            {
                break;
            }

        }
    }
};

MAKE_AE(TrackingAnnotator)