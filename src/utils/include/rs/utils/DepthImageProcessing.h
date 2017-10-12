/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *         Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *         Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
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

#ifndef DEPTH_IMAGE_PROCESSING_H_
#define DEPTH_IMAGE_PROCESSING_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>
//#include <opencv2/gpu/gpu.hpp>
//#include <opencv2/gpu/gpumat.hpp>

namespace rs
{
namespace DepthImageProcessing
{

void fillHoles(cv::Mat &image);
void project(const cv::Mat &depth, const cv::Mat &color, const cv::Mat &alpha, const cv::Mat &lookupX, const cv::Mat &lookupY, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

}
} // end namespace

#endif /* DEPTH_IMAGE_PROCESSING_H_ */
