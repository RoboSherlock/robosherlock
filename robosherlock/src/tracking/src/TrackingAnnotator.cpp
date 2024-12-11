#include <uima/api.hpp>

#include <pcl/point_types.h>

//RS
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>
#include <robosherlock/utils/common.h>
#include <robosherlock/types/all_types.h>
#include <robosherlock/DrawingAnnotator.h>

//OCV
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

using namespace uima;
using namespace cv;
using namespace std;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

class TrackingAnnotator : public DrawingAnnotator
{
private:
  float test_param;
  cv::Mat annotatorView;
  cv::Mat color;
  bool first_pass = true;
  Ptr<Tracker> tracker;
  string trackerType;
  Rect2d bbox;

public:

  TrackingAnnotator() : DrawingAnnotator(__func__) {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("test_param", test_param);

    // List of tracker types in OpenCV 3.4.1
    string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
    // vector <string> trackerTypes(types, std::end(types));

    // Create a tracker
    trackerType = trackerTypes[2];

    #if (CV_MINOR_VERSION < 3)
    {
        tracker = Tracker::create(trackerType);
    }
    #else
    {
        if (trackerType == "BOOSTING")
            tracker = TrackerBoosting::create();
        if (trackerType == "MIL")
            tracker = TrackerMIL::create();
        if (trackerType == "KCF")
            tracker = TrackerKCF::create();
        if (trackerType == "TLD")
            tracker = TrackerTLD::create();
        if (trackerType == "MEDIANFLOW")
            tracker = TrackerMedianFlow::create();
        if (trackerType == "GOTURN")
            tracker = TrackerGOTURN::create();
        if (trackerType == "MOSSE")
            tracker = TrackerMOSSE::create();
        //if (trackerType == "CSRT")
        //    tracker = TrackerCSRT::create();
    }
    #endif

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    std::vector<rs::ObjectHypothesis> clusters;
    scene.identifiables.filter(clusters);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

    cas.get(VIEW_CLOUD,*cloud_ptr);
    cas.get(VIEW_COLOR_IMAGE, color);
    cas.get(VIEW_COLOR_IMAGE, annotatorView);
    outInfo(clusters.size());

    for(int a = 0; a < 1 && a < clusters.size(); a++)
    {
        rs::ImageROI imageRoi(clusters[0].rois());
        cv::Rect rect;
        rs::conversion::from(imageRoi.roi(), rect);

        // Read frame
        Mat frame = color.clone();

        if(first_pass)
        {
            // Define initial bounding box
            bbox = Rect2d(rect.x, rect.y, rect.width, rect.height);

            // Display bounding box.
            rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1);
            rectangle(annotatorView, bbox, Scalar( 255, 0, 0 ), 2, 1);

            tracker->init(frame, bbox);
            first_pass = false;
        }
        else
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
                outInfo("Tracking successful");
                rectangle(annotatorView, bbox, Scalar( 255, 0, 0 ), 2, 1);
            }
            else
            {
                // Tracking failure detected.
                putText(annotatorView, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
            }

            // Display tracker type on frame
            putText(annotatorView, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);

            // Display FPS on frame
            putText(annotatorView, "FPS : " + SSTR(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);
        }
        outInfo("Tracking End");
    }


    outInfo("Cloud size: " << cloud_ptr->points.size());
    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp) override
  {
    disp = annotatorView.clone();
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(TrackingAnnotator)