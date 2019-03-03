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

// ROS
#include <robosherlock_msgs/RSObjectDescriptions.h>

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

class KCFTrackingAnnotator : public DrawingAnnotator
{
private:
  Ptr<Tracker> tracker = TrackerKCF::create();
  cv::Mat frame; // Input data for 2D tracking
  Rect2d bbox;
  bool first_execution = true;
  std::vector <rs::ObjectHypothesis> clusters;
  bool ok = false;
public:
  KCFTrackingAnnotator() : DrawingAnnotator(__func__)
  {
    //cv::initModule_nonfree();
  }

  ros::NodeHandle nh_;
  ros::Publisher result_pub = nh_.advertise<robosherlock_msgs::RSObjectDescriptions>(std::string("RoboSherlock/tracking_info"), 1);

  /*
   * Initializes annotator
   */
  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    return UIMA_ERR_NONE;
  }

  TyErrorId reconfigure()
  {
    outInfo("Reconfiguring");
    tracker = TrackerKCF::create();
    first_execution = true;
    cv::Mat frame_temp = frame;
    frame = Mat(frame_temp.rows, frame_temp.cols, CV_32F, 5.0);
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
    rs::Scene scene = cas.getScene();
    cas.get(VIEW_COLOR_IMAGE_HD, frame); // Fill input data

    if (first_execution)
    {
      scene.identifiables.filter(clusters);
      if (!frame.rows > 0)
      {
        outError("Visual input is empty. Has VIEW_COLOR_IMAGE been filled?");
        return UIMA_ERR_NONE;
      }

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

      rs::ImageROI image_roi = clusters[obj_id].rois.get();
      cv::Rect roi;
      rs::conversion::from(image_roi.roi_hires(), roi);
      Rect2d bbox(roi.x, roi.y, roi.width, roi.height); // Manual Rect to Rect2d conversion

      bool init_ok = tracker->init(frame, bbox);
      if (init_ok)
      {
        outInfo("Tracker initialized successfully!");
      }
      else
      {
        outError("Tracker initialization failed!");
      }
      first_execution = false;
    }
    else
    {
      outInfo("Updating the tracker...");
      ok = tracker->update(frame, bbox);
      robosherlock_msgs::RSObjectDescriptions result_message;
      if (ok)
      {
        outInfo("Tracker updated successfully!");

        // TODO: The output is supposed to happen through RSProcessManager's result_pub. To do this, put an
        // TODO: ObjectHypothesis with a set roi parameter into the scene. For now, this Annotator just has its own
        // TODO: publisher.
        /**
        cv::Rect roi(bbox.x, bbox.y, bbox.width, bbox.height);
        rs::ObjectHypothesis tracked_cluster = rs::create<rs::ObjectHypothesis>(tcas);
        tracked_cluster.rois.set(roi);
        scene.identifiables.append(tracked_cluster);
         **/

        std::vector<std::string> result_response;
        result_response.push_back("x: " + std::to_string(bbox.x));
        result_response.push_back("y: " + std::to_string(bbox.y));
        result_response.push_back("width: " + std::to_string(bbox.width));
        result_response.push_back("height: " + std::to_string(bbox.height));
        result_message.obj_descriptions = result_response;
        result_pub.publish(result_message);
      }
      else
      {
        outError("Tracking failed!");
        std::vector<std::string> result_response;
        result_response.push_back("Object lost!");
        result_message.obj_descriptions = result_response;
        result_pub.publish(result_message);
      }
    }
    // bbox position debug output
    outInfo("x: " + std::to_string(bbox.x));
    outInfo("y: " + std::to_string(bbox.y));
    outInfo("width: " + std::to_string(bbox.width));
    outInfo("height: " + std::to_string(bbox.height));
    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = frame.clone();
    if(ok)
    {
      rectangle(disp, bbox, Scalar(255, 0, 0), 2, 1);
    }
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool FIRST_RUN)
  {
  }

};

MAKE_AE(KCFTrackingAnnotator)
