#ifndef DEPTH_IMAGE_PROCESSING_H_
#define DEPTH_IMAGE_PROCESSING_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/gpu/gpumat.hpp>

namespace rs
{
namespace DepthImageProcessing
{

void fillHoles(cv::Mat &image);
void project(const cv::Mat &depth, const cv::Mat &color, const cv::Mat &alpha, const cv::Mat &lookupX, const cv::Mat &lookupY, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

}
} // end namespace

#endif /* DEPTH_IMAGE_PROCESSING_H_ */
