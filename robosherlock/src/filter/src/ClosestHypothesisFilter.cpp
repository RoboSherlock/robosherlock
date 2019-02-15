#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>

using namespace uima;

class ClosestHypothesisFilter : public DrawingAnnotator
{
private:
  bool firstExecution = true;
  int xPos;
  int yPos;

public:
  ClosestHypothesisFilter() : DrawingAnnotator(__func__)
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    //ctx.extractValue("test_param", test_param);
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    std::vector<rs::ObjectHypothesis> clusters;
    scene.identifiables.filter(clusters);

    /**
     * Logic of the first process call:
     * Using OBJ_ID_TRACK which has to be set before executing ClosestHypothesisFilter for the first time,
     * the ID of the object of interest in the current scene is passed. The 2D position of this object is
     * saved for future process calls.
     */
    if(firstExecution) {
      rs::Size s = rs::create<rs::Size>(tcas); // a hack to get a simple integer (the object ID) from the cas.
      if (!cas.getFS("OBJ_ID_TRACK", s)) {
        outError("Please set OBJ_TO_TRACK before processing with ClosestHypothesisFilter for the first time.");
        return UIMA_ERR_NONE;
      }
      int obj_id = s.height.get();
      if(clusters.size() > obj_id){
        rs::ImageROI image_roi = clusters[obj_id].rois.get();
        cv::Rect roi;
        rs::conversion::from(image_roi.roi_hires(), roi);
        xPos = roi.x + (roi.width / 2);
        yPos = roi.y + (roi.height / 2);
        std::vector<rs::Identifiable> finalClusterVector;
        finalClusterVector.push_back(clusters[obj_id]);
        scene.identifiables.set(finalClusterVector);
        outInfo("Successfully extracted the object hypothesis of ID " + std::to_string(obj_id) +
        " in the scene and saved its position. "
        "Following process calls with attempt to segment the same object hypothesis.");
      }
      else{
        outError("An object of id " + std::to_string(obj_id) + " does not exist.");
        return UIMA_ERR_NONE;
      }
      firstExecution = false;
    }
    /**
     * Logic of successive process calls:
     * Determine the distance from where our object hypothesis was last seen to the first object hypothesis
     * for reference. Then, determine the distance of every successive object hypothesis, compare it to the
     * closest yet and only save the closest. After iterating through all clusters, push only the closest
     * cluster back into the scene identifiables.
     */
    else{
      outInfo("This is a successive execution.");
      int targetID = 0;
      int xTargetPos;
      int yTargetPos;
      if(!clusters.size() > 0){
        outError("There are no clusters in the scene.");
        return UIMA_ERR_NONE;
      }
      else{
        //outInfo("There is at least one cluster in this scene.");
        rs::ImageROI rs_target_roi = clusters[0].rois.get();
        cv::Rect target_roi;
        rs::conversion::from(rs_target_roi.roi_hires(), target_roi);
        xTargetPos = target_roi.x + (target_roi.width / 2);
        yTargetPos = target_roi.y + (target_roi.height / 2);
        double targetDistance = sqrt((xTargetPos - xPos) * (xTargetPos - xPos)) +
                ((yTargetPos - yPos) * (yTargetPos - yPos));

        for(int n = 1; n < clusters.size(); n++) {
          //outInfo("Checking distance of object hypothesis " + std::to_string(n));
          rs::ImageROI rs_current_roi = clusters[n].rois.get();
          cv::Rect current_roi;
          rs::conversion::from(rs_current_roi.roi_hires(), current_roi);
          int xCurrentPos = current_roi.x + (current_roi.width / 2);
          int yCurrentPos = current_roi.y + (current_roi.height / 2);
          double currentDistance = sqrt((xCurrentPos - xPos) * (xCurrentPos - xPos)) +
                                 ((yCurrentPos - yPos) * (yCurrentPos - yPos));
          if(currentDistance < targetDistance){
            targetID = n;
            targetDistance = currentDistance;
            xTargetPos = xCurrentPos;
            yTargetPos = yCurrentPos;
          }
        }
      }
      std::vector<rs::Identifiable> finalClusterVector;
      finalClusterVector.push_back(clusters[targetID]);
      scene.identifiables.set(finalClusterVector);
      double finalMovementAmount = sqrt((xTargetPos - xPos) * (xTargetPos - xPos)) +
                                   ((yTargetPos - yPos) * (yTargetPos - yPos));
      outInfo("Object has moved by " + std::to_string(finalMovementAmount) + " since last process call");
      xPos = xTargetPos;
      yPos = yTargetPos;
    }

    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ClosestHypothesisFilter)