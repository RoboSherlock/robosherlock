#ifndef __IMAGE_SEGMENTATION_H__
#define __IMAGE_SEGMENTATION_H__

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

class ImageSegmentation
{
public:
  struct Segment
  {
    // Segment attributes
    std::vector<cv::Point> contour;
    std::vector<Segment> children;

    size_t area;
    size_t childrenArea;
    size_t holes;

    cv::Rect rect;
    cv::Moments moments;
    std::vector<double> huMoments;

    // 2D attributes
    cv::Point2d center;
    cv::Point2d axisX;
    cv::Point2d axisY;
    double alpha;

    // 3D attributes
    cv::Mat rotation;
    cv::Mat translation;
    double lengthX;
    double lengthY;
  };

  static void thresholding(const cv::Mat &grey, cv::Mat &bin, const double threshold, const int method = cv::THRESH_BINARY, const cv::Rect roi = cv::Rect(0, 0, 0, 0));

  static void segment(const cv::Mat &bin, std::vector<Segment> &segments, const size_t minSize, const size_t minHoleSize, const cv::Rect &roi);

  static void computePose(std::vector<Segment> &segments, const cv::Mat &cameraMatrix, const cv::Mat &distCoefficients, const cv::Mat &planeNormal, const double planeDistance);

  static void backProject(const cv::Mat &planeNormal, const double planeDistance, const cv::Point2d &pointImage, cv::Mat &pointWorld);

  static void drawSegments2D(cv::Mat &disp, const std::vector<Segment> &segments, const std::vector<std::string> &text,
                             const int sizeLine = 1, const double sizeText = 1.0, const int font = cv::FONT_HERSHEY_SIMPLEX);

  static void drawSegments3D(cv::Mat &disp, const std::vector<Segment> &segments, const cv::Mat &cameraMatrix, const cv::Mat &distCoefficients,
                             const std::vector<std::string> &text, const int sizeLine = 1, const double sizeText = 1.0, const int font = cv::FONT_HERSHEY_SIMPLEX);

  static void drawSegment(cv::Mat &disp, const cv::Scalar &colorOut, const cv::Scalar &colorIn, const Segment &seg, const int32_t level = 0, const int32_t levels = 1, const bool fill = true, const int lineSize = 4, const cv::Point &offset = cv::Point(0, 0));

private:
  ImageSegmentation();
  ~ImageSegmentation();

  static bool toSegment(const std::vector<std::vector<cv::Point>> &contours, const std::vector<cv::Vec4i> &hierarchy, const size_t index,
                        Segment &seg, const size_t minSize, const size_t minHoleSize, const cv::Rect &roi);

  static void computeMoments(const std::vector<std::vector<cv::Point>> &contours, const std::vector<cv::Vec4i> &hierarchy, const size_t index, Segment &seg);

  static void compute2DAttributes(Segment &seg);
};

#endif //__IMAGE_SEGMENTATION_H__
