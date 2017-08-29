/**
 * Copyright 2017 University of Bremen, Institute for Artificial Intelligence
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

#include <rs/utils/Drawing.h>


namespace Drawing {
//////////////////////////////////////////////////
void drawMeshDepth(
    cv::Mat &dst_32fc1,
    const std::vector<cv::Point3f> &points,
    const std::vector<std::vector<int>> &indices,
    const cv::Vec3f rot,
    const cv::Vec3f trans,
    const cv::Mat &cam,
    const std::vector<float> &ks) {
  cv::Mat cam_sp_transform = GeometryCV::poseRTToAffine({rot, trans});

  std::vector<cv::Point3f> vertice = GeometryCV::transform(cam_sp_transform, points);

  std::vector<cv::Point2f> vertice_2d;
  cv::projectPoints(vertice, cv::Vec3f(0, 0, 0), cv::Vec3f(0, 0, 0), cam, ks, vertice_2d);

  cv::Mat none;
  std::vector<float> vals;

  for (const auto &tri : indices) {
    std::vector<float> depth {
      vertice[tri[0]].z,
      vertice[tri[1]].z,
      vertice[tri[2]].z};

    std::vector<cv::Point3f> poly{
        cv::Vec3f(vertice_2d[tri[0]].x, vertice_2d[tri[0]].y, depth[0]),
        cv::Vec3f(vertice_2d[tri[1]].x, vertice_2d[tri[1]].y, depth[1]),
        cv::Vec3f(vertice_2d[tri[2]].x, vertice_2d[tri[2]].y, depth[2])};

    drawTriangleInterp(dst_32fc1, none, poly, vals);
  }
}

//////////////////////////////////////////////////
void drawMeshNormals(
    cv::Mat &dst_depth_32fc1,
    cv::Mat &dst_32fc3,
    const std::vector<cv::Point3f> &points,
    const std::vector<cv::Vec3f> &normals,
    const std::vector<std::vector<int>> &indices,
    const cv::Vec3f rot,
    const cv::Vec3f trans,
    const cv::Mat &cam,
    const std::vector<float> &ks,
    const bool flat) {
  cv::Mat cam_sp_transform = GeometryCV::poseRTToAffine({rot, trans});
  cv::Mat cam_sp_transform_rot = GeometryCV::poseRTToAffine({rot, cv::Vec3f(0, 0, 0)});

  std::vector<cv::Point3f> vertice = GeometryCV::transform(cam_sp_transform, points);
  std::vector<cv::Vec3f> normals_cs = GeometryCV::transform(cam_sp_transform_rot, normals);

  std::vector<cv::Point2f> vertice_2d;
  cv::projectPoints(vertice, cv::Vec3f(0, 0, 0), cv::Vec3f(0, 0, 0), cam, ks, vertice_2d);

  for (const auto &tri : indices) {
    std::vector<float> depth {
      vertice[tri[0]].z,
      vertice[tri[1]].z,
      vertice[tri[2]].z};

    std::vector<cv::Vec3f> norms {
      normals_cs[tri[0]],
      normals_cs[tri[1]],
      normals_cs[tri[2]]};

    std::vector<cv::Point3f> poly{
        cv::Vec3f(vertice_2d[tri[0]].x, vertice_2d[tri[0]].y, depth[0]),
        cv::Vec3f(vertice_2d[tri[1]].x, vertice_2d[tri[1]].y, depth[1]),
        cv::Vec3f(vertice_2d[tri[2]].x, vertice_2d[tri[2]].y, depth[2])};

    drawTriangleInterp(dst_depth_32fc1, dst_32fc3, poly, norms);
  }
}

//////////////////////////////////////////////////
cv::Mat drawHistogram(const std::vector<double> &data, const int bin_width, const int height, const double level) {
  int bins_num = data.size();
  cv::Mat hist = cv::Mat::zeros(height, bins_num*bin_width, CV_8UC3);

  double max_val = *std::max_element(data.begin(), data.end());

  int i = 0;
  for (auto &bin : data) {
    float x0 = i * bin_width;
    float y0 = (1 - bin/max_val) * height;
    auto color = (bin > level ? cv::Scalar(0, 128, 0) : cv::Scalar(0, 0, 128));
    cv::rectangle(hist, cv::Rect(x0, y0, bin_width, height - y0 + 1), color, -1);
    i++;
  }

  return hist;
}

}