/* Copyright (c) 2013, Thiemo Wiedemeyer  <wiedemeyer@informatik.uni-bremen.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// UIMA
#include <uima/api.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// RS
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>

#include <rs/segmentation/ImageSegmentation.h>

using namespace uima;

class ImageSegmentationAnnotator : public DrawingAnnotator
{

private:
  cv::Mat color, hsv, grey, bin, mask, maskPlane;
  std::vector<ImageSegmentation::Segment> segments;
  float threshold;
  float hsvThreshold;
  bool hsvFilter;
  size_t minSize, minHoleSize;
  int hbins, sbins, hdivide, sdivide;

  cv::Mat cameraMatrix;
  cv::Mat distCoefficients;
  bool foundPlane;

  tf::StampedTransform camToWorld, worldToCam;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

  enum
  {
    POSE,
    SEGMENTS,
    BIN,
    GREY,
    COLOR,
    HUE,
    SATURATION,
    MASK
  } displayMode;

public:
  ImageSegmentationAnnotator() : DrawingAnnotator(__func__), threshold(100), hsvThreshold(150)
    , hsvFilter(false), minHoleSize(50), hbins(64), sbins(64), foundPlane(false), displayMode(SEGMENTS)
  {
    cameraMatrix = cv::Mat(3, 3, CV_64F);
    distCoefficients = cv::Mat(1, 8, CV_64F);
  }

  /*
   * Initializes annotator
   */
  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    if(ctx.isParameterDefined("threshold"))
    {
      ctx.extractValue("threshold", threshold);
    }
    if(ctx.isParameterDefined("hsvThreshold"))
    {
      ctx.extractValue("hsvThreshold", hsvThreshold);
    }
    if(ctx.isParameterDefined("minSize"))
    {
      ctx.extractValue("minSize", minSize);
    }
    if(ctx.isParameterDefined("minHoleSize"))
    {
      ctx.extractValue("minHoleSize", minHoleSize);
    }
    if(ctx.isParameterDefined("hsvFilter"))
    {
      ctx.extractValue("hsvFilter", hsvFilter);
    }

    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    hdivide = 180 / hbins;
    sdivide = 256 / sbins;

    return UIMA_ERR_NONE;
  }

  /*
   * Destroys annotator
   */
  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  bool callbackKey(const int key, const Source source)
  {
    switch(key)
    {
    case 'b':
    case 'B':
      displayMode = BIN;
      return true;
    case 'g':
    case 'G':
      displayMode = GREY;
      return true;
    case 'c':
    case 'C':
      displayMode = COLOR;
      return true;
    case 'h':
    case 'H':
      displayMode = HUE;
      return true;
    case 'j':
    case 'J':
      displayMode = SATURATION;
      return true;
    case 'p':
    case 'P':
      displayMode = POSE;
      return true;
    case 's':
    case 'S':
      displayMode = SEGMENTS;
      return true;
    case 'm':
    case 'M':
      displayMode = MASK;
      return true;
    }
    return false;
  }

private:
  /*
   * Processes a frame
   */
  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    cas.get(VIEW_COLOR_IMAGE_HD, color);
    cas.get(VIEW_CLOUD, *cloud);

    cv::cvtColor(color, hsv, cv::COLOR_BGR2HSV_FULL);
    cv::cvtColor(color, grey, cv::COLOR_BGR2GRAY);

    segments.clear();
    bin = cv::Mat::zeros(color.rows, color.cols, CV_8U);
    mask = cv::Mat::zeros(color.rows, color.cols, CV_8U);

    camToWorld.setIdentity();
    if(scene.viewPoint.has())
    {
      rs::conversion::from(scene.viewPoint.get(), camToWorld);
    }
    else
    {
      outInfo("No camera to world transformation!!!");
    }
    worldToCam = tf::StampedTransform(camToWorld.inverse(), camToWorld.stamp_, camToWorld.child_frame_id_, camToWorld.frame_id_);
    std::vector<rs::Plane> planes;
    scene.annotations.filter(planes);

    if(planes.empty())
    {
      return UIMA_ERR_ANNOTATOR_MISSING_INFO;
    }
    rs::Plane &plane = planes[0];

    std::vector<float> model = plane.model();

    if(model.empty() || model.size() != 4)
    {
      foundPlane = false;
      outInfo("no plane found!");
    }
    else
    {
      foundPlane = true;

      cv::Mat planeNormal(3, 1, CV_64F);
      planeNormal.at<double>(0) = model[0];
      planeNormal.at<double>(1) = model[1];
      planeNormal.at<double>(2) = model[2];
      const double planeDistance = model[3];

      outInfo("Plane normal: " << planeNormal);
      outInfo("Plane distance: " << planeDistance);

      sensor_msgs::CameraInfo camInfo;
      cas.get(VIEW_CAMERA_INFO_HD, camInfo);
      readCameraInfo(camInfo);

      cv::Mat planeMask, mask;
      cv::Rect planeRoi, planeRoiHires;
      rs::conversion::from(plane.mask(), mask);
      rs::conversion::from(plane.roi(), planeRoi);

      planeMask = cv::Mat::zeros(camInfo.height / 2, camInfo.width / 2, CV_8U);
      maskPlane = cv::Mat::zeros(camInfo.height / 2, camInfo.width / 2, CV_8U);
      mask.copyTo(planeMask(planeRoi));

      std::vector<std::vector<cv::Point>> contours;
      std::vector<cv::Vec4i> hierarchy;

      cv::findContours(planeMask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
      int biggest = 0;
      for(size_t i = 1; i < contours.size(); ++i)
      {
        if(contours[i].size() > contours[biggest].size())
        {
          biggest = i;
        }
      }
      planeMask.setTo(255);
      cv::drawContours(planeMask, contours, biggest, CV_RGB(0, 0, 0), CV_FILLED);
      cv::drawContours(maskPlane, contours, biggest, CV_RGB(255, 255, 255), CV_FILLED);
      cv::dilate(planeMask, planeMask, cv::Mat(), cv::Point(-1, -1), 10);
      cv::resize(planeMask, mask, cv::Size(camInfo.width, camInfo.height), 0, 0, cv::INTER_NEAREST);
      planeRoi = cv::boundingRect(contours[biggest]);
      planeRoiHires = planeRoi;
      planeRoiHires.x *= 2;
      planeRoiHires.y *= 2;
      planeRoiHires.width *= 2;
      planeRoiHires.height *= 2;


      ImageSegmentation::thresholding(grey, bin, threshold, cv::THRESH_BINARY);
      bin.setTo(0, mask);
      ImageSegmentation::segment(bin, segments, minSize, minHoleSize, planeRoiHires);
      ImageSegmentation::computePose(segments, cameraMatrix, distCoefficients, planeNormal, planeDistance);
      std::vector<ImageSegmentation::Segment> additional_segments;

      if(hsvFilter)
      {
        std::vector<cv::Mat> channels;
        cv::split(hsv, channels);
        ImageSegmentation::thresholding(channels[1], bin, hsvThreshold, cv::THRESH_BINARY);
        bin.setTo(0, mask);
        ImageSegmentation::segment(bin, additional_segments, minSize, minHoleSize, planeRoiHires);
        ImageSegmentation::computePose(additional_segments, cameraMatrix, distCoefficients, planeNormal, planeDistance);
        segments.insert(segments.end(), additional_segments.begin(), additional_segments.end());
      }
      addClusters(tcas, scene);
    }

    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    switch(displayMode)
    {
    case BIN:
      cv::cvtColor(bin, disp, CV_GRAY2BGR);
      break;
    case GREY:
      cv::cvtColor(grey, disp, CV_GRAY2BGR);
      break;
    case POSE:
      disp = color.clone();
      if(foundPlane)
      {
        ImageSegmentation::drawSegments3D(disp, segments, cameraMatrix, distCoefficients, std::vector<std::string>());
      }
      break;
    case SEGMENTS:
      cv::cvtColor(grey, disp, CV_GRAY2BGR);
      ImageSegmentation::drawSegments2D(disp, segments, std::vector<std::string>());
      break;

    case COLOR:
      disp = color.clone();
      break;
    case HUE:
      {
        std::vector<cv::Mat> channels;
        cv::split(hsv, channels);
        cv::cvtColor(channels[0], disp, CV_GRAY2BGR);
        break;
      }
    case SATURATION:
      {
        std::vector<cv::Mat> channels;
        cv::split(hsv, channels);
        cv::cvtColor(channels[1], disp, CV_GRAY2BGR);
        break;
      }
    case MASK:
      {
        cv::cvtColor(maskPlane, disp, CV_GRAY2BGR);
        break;
      }
    }
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

  void addClusters(CAS &tcas, rs::Scene &scene)
  {
    for(size_t i = 0; i < segments.size(); ++i)
    {
      ImageSegmentation::Segment &seg = segments[i];
      cv::Rect roi_lowres(seg.rect.x / 2, seg.rect.y / 2, seg.rect.width / 2, seg.rect.height / 2);
      cv::Mat mask, mask_hires;
      mask_hires = cv::Mat::zeros(seg.rect.height, seg.rect.width, CV_8U);

      ImageSegmentation::drawSegment(mask_hires, CV_RGB(255, 255, 255), CV_RGB(0, 0, 0), seg, 0, 1, true, 4, -seg.rect.tl());
      cv::resize(mask_hires, mask, roi_lowres.size(), 0, 0, cv::INTER_NEAREST);

      rs::ImageROI roi = rs::create<rs::ImageROI>(tcas);
      roi.roi(rs::conversion::to(tcas, roi_lowres));
      roi.roi_hires(rs::conversion::to(tcas, seg.rect));
      roi.mask(rs::conversion::to(tcas, mask));
      roi.mask_hires(rs::conversion::to(tcas, mask_hires));

      maskPlane(roi_lowres).setTo(0, mask);

      rs::ReferenceClusterPoints points = rs::create<rs::ReferenceClusterPoints>(tcas);
      pcl::PointIndices indices;
      indices.header = cloud->header;

      for(int r = 0; r < mask.rows; ++r)
      {
        size_t index = (roi_lowres.y + r) * cloud->width + roi_lowres.x;
        const uint8_t *itM = mask.ptr<uint8_t>(r);
        const pcl::PointXYZRGBA *itP = &cloud->points[index];

        for(int c = 0; c < mask.cols; ++c, ++itM, ++itP, ++index)
        {
          if(*itM > 0 && itP->z > 0)
          {
            indices.indices.push_back(index);
          }
        }
      }
      points.indices(rs::conversion::to(tcas, indices));

      rs::Cluster cluster = rs::create<rs::Cluster>(tcas);
      cluster.rois(roi);
      cluster.points(points);
      cluster.annotations.append(rs::conversion::to(tcas, seg));
      cluster.annotations.append(getPose(tcas, seg, scene.timestamp()));
      cluster.annotations.append(buildDesciptor(tcas, seg));
      scene.identifiables.append(cluster);
    }
  }

  rs::Features buildDesciptor(CAS &tcas, const ImageSegmentation::Segment &seg)
  {
    cv::Mat histogram;
    std::vector<std::vector<cv::Point>> points;
    points.push_back(seg.contour);
    cv::Mat mask = cv::Mat(seg.rect.height, seg.rect.width, CV_8U);
    cv::drawContours(mask, points, 0, CV_RGB(255, 255, 255), CV_FILLED, 8, cv::noArray(), INT_MAX, -seg.rect.tl());
    computeColorHist(color(seg.rect), mask, histogram);

    cv::Mat descriptor(1, 14 + histogram.cols, CV_32F);
    float *it = descriptor.ptr<float>(0);
    *it++ = (float)seg.moments.nu02;
    *it++ = (float)seg.moments.nu03;
    *it++ = (float)seg.moments.nu11;
    *it++ = (float)seg.moments.nu12;
    *it++ = (float)seg.moments.nu20;
    *it++ = (float)seg.moments.nu21;
    *it++ = (float)seg.moments.nu30;

    for(size_t i = 0; i < 7; ++i, ++it)
    {
      *it = (float)seg.huMoments[i];
    }
    //hack for now;
    //*it++ = (float)seg.area / (1920 * 1080);
    //*it++ = seg.childrenArea / (float)seg.area;

    const float *itI = histogram.ptr<float>(0);
    for(size_t i = 0; i < histogram.cols; ++i, ++it, ++itI)
    {
      *it = *itI;
    }
    //    cv::normalize(hitogram);
    rs::Features features = rs::create<rs::Features>(tcas);
    features.descriptors(rs::conversion::to(tcas, descriptor));
    features.descriptorType("numerical");
    //features.detector("AcatSegmentation");
    features.extractor("ImageSegmentationAnnotator");
    return features;
  }
  void computeColorHist(const cv::Mat &color, const cv::Mat &mask, cv::Mat &histogram)
  {
    const int hbins = 4, sbins = 4;
    const int histSize[] = {hbins, sbins};
    const float hranges[] = { 0, 180 };
    const float sranges[] = { 0, 256 };
    const float *ranges[] = { hranges, sranges };
    const int channels[] = {0, 1};

    cv::Mat hsv, hist;
    cv::cvtColor(color, hsv, CV_BGR2HSV);

    cv::calcHist(&hsv, 1, channels, mask, hist, 2, histSize, ranges, true, false);

    // Creating descriptor mat
    size_t descriptorSize = hbins * sbins;
    histogram = cv::Mat(1, descriptorSize, CV_32FC1);

    float *itO = histogram.ptr<float>(0);
    const float *itI = hist.ptr<float>(0);
    for(int32_t i = 0; i < descriptorSize; ++i, ++itI, ++itO)
    {
      *itO = *itI;
    }
    cv::normalize(histogram, histogram, 0, 1, cv::NORM_MINMAX);
  }

  void combineDescriptors(cv::Mat &descriptors, const std::vector<cv::Mat> &histograms)
  {
    if(histograms.empty() || descriptors.empty())
    {
      outError("Error: nothing to combine.");
      return;
    }

    size_t descriptorSize = descriptors.cols + histograms[0].cols;
    cv::Mat newDescriptors = cv::Mat(descriptors.rows, descriptorSize, CV_32FC1);
    descriptors.copyTo(newDescriptors(cv::Rect(0, 0, descriptors.cols, descriptors.rows)));

    for(int32_t r = 0; r < descriptors.rows; ++r)
    {
      const cv::Mat &histogram = histograms[r];
      const float *itI = histogram.ptr<float>(0);
      float *itO = newDescriptors.ptr<float>(r, descriptors.cols);
      for(int32_t c = 0; c < histogram.cols; ++c, ++itO, ++itI)
      {
        *itO = *itI;
      }
    }
    descriptors = newDescriptors;
  }

  rs::PoseAnnotation getPose(CAS &tcas, const ImageSegmentation::Segment &seg, const INT64 timestamp)
  {
    const cv::Mat &r = seg.rotation;
    const cv::Mat &t = seg.translation;

    tf::Vector3 trans(t.at<double>(0), t.at<double>(1), t.at<double>(2));
    tf::Matrix3x3 rot;
    rot.setValue(r.at<double>(0, 0), r.at<double>(0, 1), r.at<double>(0, 2),
                 r.at<double>(1, 0), r.at<double>(1, 1), r.at<double>(1, 2),
                 r.at<double>(2, 0), r.at<double>(2, 1), r.at<double>(2, 2));

    tf::Transform transform;
    transform.setOrigin(trans);
    transform.setBasis(rot);

    tf::Stamped<tf::Pose> camera(transform, camToWorld.stamp_, camToWorld.child_frame_id_);
    tf::Stamped<tf::Pose> world(transform * camToWorld, camToWorld.stamp_, camToWorld.frame_id_);

    rs::PoseAnnotation poseAnnotation = rs::create<rs::PoseAnnotation>(tcas);
    poseAnnotation.camera.set(rs::conversion::to(tcas, camera));
    poseAnnotation.world.set(rs::conversion::to(tcas, world));

    return poseAnnotation;
  }
};

MAKE_AE(ImageSegmentationAnnotator)
