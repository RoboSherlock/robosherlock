#include <iterator>

#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>

// CV
#include <opencv2/opencv.hpp>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>

#include <rs/segmentation/ImageSegmentation.h>

using namespace uima;


class TransparentSegmentator : public DrawingAnnotator
{
private:
  int open_iterations = 6;
  int close_iterations = 12;
  int gc_erode_iterations = 6;
  int gc_dilate_iterations = 12;
  int grabcut_iterations = 2;
  int morph_kernel_size = 3;
  bool convex_hull_segments = false;
  int min_contour_area = 100;

  cv::Mat image_rgb;
  cv::Mat image_depth;
  cv::Mat viz_image;

  cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F);
  cv::Mat distCoefficients = cv::Mat(1, 8, CV_64F);

  std::vector<ImageSegmentation::Segment> segments;


public:
  TransparentSegmentator(): DrawingAnnotator(__func__) {

  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    ctx.extractValue("open_iterations", open_iterations);
    ctx.extractValue("close_iterations", close_iterations);
    ctx.extractValue("gc_erode_iterations", gc_erode_iterations);
    ctx.extractValue("gc_dilate_iterations", gc_dilate_iterations);
    ctx.extractValue("grabcut_iterations", grabcut_iterations);
    ctx.extractValue("morph_kernel_size", morph_kernel_size);
    ctx.extractValue("convex_hull_segments", convex_hull_segments);
    ctx.extractValue("min_contour_area", min_contour_area);

    viz_image = cv::Mat::zeros(480, 640, CV_8UC3);

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec) override
  {
    outInfo("process start");
    rs::SceneCas cas(tcas);

    if (!cas.get(VIEW_MASK, image_depth)) {
      outError("No view mask image");
      return UIMA_ERR_NONE;
    }

    if (!cas.get(VIEW_COLOR_IMAGE, image_rgb)) {
      outError("No color image");
      return UIMA_ERR_NONE;
    }

    rs::Scene scene = cas.getScene();

    std::vector<rs::Plane> planes;
    scene.annotations.filter(planes);

    if(planes.empty())
      return UIMA_ERR_ANNOTATOR_MISSING_INFO;

    rs::Plane &plane = planes[0];
    std::vector<float> model = plane.model();

    if(model.empty() || model.size() != 4)
      outInfo("no plane found!");
    else
    {
      sensor_msgs::CameraInfo camInfo;
      cas.get(VIEW_CAMERA_INFO_HD, camInfo);
      readCameraInfo(camInfo);

      cv::Mat planeNormal(3, 1, CV_64F);
      planeNormal.at<double>(0) = model[0];
      planeNormal.at<double>(1) = model[1];
      planeNormal.at<double>(2) = model[2];
      const double planeDistance = model[3];

      cv::Mat planeMask = getPlaneMaskClosed(plane, camInfo); 

      // TODO: use raw depth mask - the one before ImagePreprocessor
      cv::Mat depth_failed_mask = (image_depth == 255) & planeMask;

      preprocessFailedMask(depth_failed_mask);

      auto roi_masks = getProbableCGRegions(depth_failed_mask);

      cv::Mat refined_mask(depth_failed_mask.size(), CV_8UC1, cv::GC_BGD);

      for (auto &region : roi_masks)
        refineRegion(region, refined_mask, image_rgb);

      refined_mask = ((refined_mask == cv::GC_FGD)
        | (refined_mask == cv::GC_PR_FGD));

      this->segments.clear();

      int min_hole_size = 50;
      ImageSegmentation::segment(refined_mask, segments, min_contour_area, min_hole_size,
          cv::Rect(0, 0, refined_mask.cols, refined_mask.rows));
      ImageSegmentation::computePose(segments, cameraMatrix, distCoefficients, planeNormal, planeDistance);

      // TODO: add appropriate annotations, based on segments
    }

    return UIMA_ERR_NONE;
  }

protected:
  using Contour = std::vector<cv::Point>;

  virtual void drawImageWithLock(cv::Mat &disp) override {
    disp = image_rgb*0.3;
    ImageSegmentation::drawSegments2D(disp, segments, {});
  }

  cv::Mat getPlaneMaskClosed(rs::Plane &plane, sensor_msgs::CameraInfo &camInfo) {
      cv::Mat planeMask, mask;
      cv::Rect planeRoi;

      rs::conversion::from(plane.mask(), mask);
      rs::conversion::from(plane.roi(), planeRoi);

      planeMask = cv::Mat::zeros(camInfo.height / 2, camInfo.width / 2, CV_8U);
      mask.copyTo(planeMask(planeRoi));

      std::vector<Contour> contours;
      std::vector<cv::Vec4i> hierarchy;

      cv::findContours(planeMask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

      auto plane_it = std::max_element(contours.begin(), contours.end(),
          [](const Contour &a, const Contour &b) {
            return a.size() < b.size();});

      Contour hull;
      cv::convexHull(*plane_it, hull);

      std::vector<Contour> contour_list{hull};
      cv::fillPoly(planeMask, contour_list, cv::Scalar(255, 255, 255));

      return planeMask;
  }

  void preprocessFailedMask(cv::Mat &failed_depth) {
    cv::Mat mkernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morph_kernel_size, morph_kernel_size));

    cv::morphologyEx(failed_depth, failed_depth, cv::MORPH_CLOSE, mkernel, cv::Point(-1,-1), close_iterations);
    cv::morphologyEx(failed_depth, failed_depth, cv::MORPH_OPEN, mkernel, cv::Point(-1,-1), open_iterations);

    std::vector<Contour> contours;
    cv::Mat contours_mat = failed_depth.clone();
    std::vector<Contour> hulls;
    cv::findContours(contours_mat, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    for (auto &contour : contours) {
      std::vector<Contour> contour_list;

      if (convex_hull_segments) {
        Contour hull;
        cv::convexHull(contour, hull);
        contour_list.push_back(hull);
      }
      else
        contour_list.push_back(contour);

      cv::fillPoly(failed_depth, contour_list, cv::Scalar(255, 255, 255));
    }

    // TODO: apply sensor dead-zone mask
  }

  std::vector<cv::Mat> getProbableCGRegions(cv::Mat &src) {
    cv::Mat mkernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morph_kernel_size, morph_kernel_size));

    cv::Mat eroded, dilated;
    cv::erode(src, eroded, mkernel, cv::Point(-1, -1), gc_erode_iterations);
    cv::dilate(src, dilated, mkernel, cv::Point(-1, -1), gc_dilate_iterations);

    std::vector<Contour> contours;
    cv::Mat contours_mat = src.clone();
    cv::findContours(contours_mat, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    std::vector<cv::Rect> rois;
    std::vector<cv::Mat> roi_masks;
    for (auto &contour : contours) {
      if (cv::contourArea(contour) < min_contour_area)
        continue;

      constexpr int margin = 20;
      cv::Rect roi = cv::boundingRect(contour);
      roi.x = std::max(0, roi.x - margin);
      roi.y = std::max(0, roi.y - margin);
      roi.width = std::min(src.cols - roi.x, roi.width + 2*margin);
      roi.height = std::min(src.rows - roi.y, roi.height + 2*margin);

      cv::Mat c_mask(src.size(), CV_8UC1, cv::GC_BGD);
      c_mask(roi).setTo(cv::GC_PR_BGD, dilated(roi));
      c_mask(roi).setTo(cv::GC_PR_FGD, src(roi));
      c_mask(roi).setTo(cv::GC_FGD, eroded(roi));

      roi_masks.push_back(c_mask(roi));
    }

    return roi_masks;
  }

  void refineRegion(cv::Mat &gc_region_mask, cv::Mat &refined_mask, cv::Mat &rgb) {
    cv::Size size;
    cv::Point offset;

    gc_region_mask.locateROI(size, offset);
    cv::Rect roi(offset, gc_region_mask.size());

    cv::Mat bgd_model, fgd_model;
    cv::grabCut(rgb(roi), gc_region_mask, cv::Rect(),
      bgd_model, fgd_model, grabcut_iterations, cv::GC_INIT_WITH_MASK);

    cv::Mat refined_depth_roi = refined_mask(roi);
    cv::Mat copy_mask = (gc_region_mask != cv::GC_BGD) & (gc_region_mask != cv::GC_PR_BGD);

    gc_region_mask.copyTo(refined_depth_roi, copy_mask);
  }

  void readCameraInfo(const sensor_msgs::CameraInfo &camInfo)
  {
    double *it = cameraMatrix.ptr<double>(0);
    *it++ = camInfo.K[0];
    *it++ = camInfo.K[1];
    *it++ = camInfo.K[2];
    *it++ = camInfo.K[3];
    *it++ = camInfo.K[4];
    *it++ = camInfo.K[5];
    *it++ = camInfo.K[6];
    *it++ = camInfo.K[7];
    *it++ = camInfo.K[8];

    distCoefficients = cv::Mat(1, camInfo.D.size(), CV_64F);
    it = distCoefficients.ptr<double>(0);
    for(size_t i = 0; i < camInfo.D.size(); ++i, ++it)
    {
      *it = camInfo.D[i];
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(TransparentSegmentator)