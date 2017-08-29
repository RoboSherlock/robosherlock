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

#include <fstream>

#include <sensor_msgs/CameraInfo.h>

#include <rs/segmentation/ImageSegmentation.h>
#include <rs/utils/SimilarityRanking.h>

#include <rs/annotation/ContourFittingClassifier.h>


//////////////////////////////////////////////////
::Mesh Mesh::readFromFile(std::string const &filename) {
  std::vector<cv::Point3f> points;
  std::vector<cv::Vec3f> normals;
  std::vector<std::vector<int>> triangles;

  std::string path = ros::package::getPath("robosherlock") + filename;
  std::ifstream ifs(path);

  if (!ifs.good())
    throw std::invalid_argument("File '" + path + "' not found");

  enum class PLYSection : int { HEADER=0, VERTEX, FACE};
  std::map<PLYSection, int> counts;

  auto minf = std::numeric_limits<float>::min();
  auto maxf = std::numeric_limits<float>::max();
  cv::Point3f min_pt(maxf, maxf, maxf);
  cv::Point3f max_pt(minf, minf, minf);

  PLYSection cur_section = PLYSection::HEADER;
  for (std::string line; std::getline(ifs, line);) {
    if (cur_section == PLYSection::HEADER) {
      if (line.find("element face") == 0)
        counts[PLYSection::FACE] = std::atoi(line.substr(line.rfind(" ")).c_str());
      if (line.find("element vertex") == 0)
        counts[PLYSection::VERTEX] = std::atoi(line.substr(line.rfind(" ")).c_str());
      if (line.find("end_header") == 0) {
        cur_section = PLYSection::VERTEX;
        outInfo("Vertices / normals: " << counts[PLYSection::VERTEX]);
        outInfo("Faces: " << counts[PLYSection::FACE]);
      }
    }
    else if (cur_section == PLYSection::VERTEX) {
      if (0 < counts[cur_section]) {
        std::istringstream iss(line);

        cv::Point3f pt;
        cv::Point3f nrm;
        iss >> pt.x >> pt.y >> pt.z >> nrm.x >> nrm.y >> nrm.z;

        min_pt.x = std::min(pt.x, min_pt.x);
        min_pt.y = std::min(pt.y, min_pt.y);
        min_pt.z = std::min(pt.z, min_pt.z);

        max_pt.x = std::max(pt.x, max_pt.x);
        max_pt.y = std::max(pt.y, max_pt.y);
        max_pt.z = std::max(pt.z, max_pt.z);

        points.push_back(pt);
        normals.push_back(nrm);
        --counts[cur_section];
      }
      else
        cur_section = PLYSection::FACE;
    }
    if (cur_section == PLYSection::FACE) {
      if (0 == counts[cur_section]--)
        break;

      std::istringstream iss(line);

      int n_verts, i1, i2, i3;
      iss >> n_verts >> i1 >> i2 >> i3;
      assert(n_verts == 3);

      triangles.push_back({i1, i2, i3});
    }
  }

  assert(counts[PLYSection::VERTEX] == 0);
  assert(counts[PLYSection::FACE] == 0);

  float scale = 0.122f/0.135f;

  cv::Point3f offset = max_pt - min_pt;
  for (auto &pt : points) {
    // pt -= offset;
    pt *= scale;
  }

  cv::Point3f origin = offset*scale;

  return {points, normals, triangles, origin};
}

//////////////////////////////////////////////////
std::vector<cv::Point2f> getCannyEdges(const cv::Mat &grayscale, const cv::Rect &input_roi) {
  auto roi = input_roi;

  constexpr int margin = 5;
  roi.x -= margin;
  roi.y -= margin;
  roi.width += 2*margin;
  roi.height += 2*margin;

  cv::Mat gray_roi = grayscale(roi);

  // Compute optimal thresh value
  cv::Mat not_used;
  double otsu_thresh_val = cv::threshold(gray_roi, not_used, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
  cv::Canny(gray_roi, gray_roi, otsu_thresh_val/3, otsu_thresh_val);

  std::vector<cv::Point2f> result;

  for(int row = 0; row < gray_roi.rows; ++row) {
    uint8_t *ptr = gray_roi.ptr(row);
    for(int col = 0; col < gray_roi.cols; ++col, ++ptr) {
      if (*ptr != 0)
        result.push_back(cv::Point2f(col + roi.x, row + roi.y));
    }
  }

  return result;
}

//////////////////////////////////////////////////
void saveToFile(const std::string filename, const std::vector<cv::Point3f> &pts) {
  std::ofstream ofs(filename);

  ofs << "VERSION .7\n"
      << "FIELDS x y z\n"
      << "SIZE 4 4 4\n"
      << "TYPE F F F\n"
      << "COUNT 1 1 1\n"
      << "WIDTH " << pts.size() << "\n"
      << "HEIGHT 1\n"
      << "VIEWPOINT 0 0 0 1 0 0 0\n"
      << "POINTS " << pts.size() << "\n"
      << "DATA ascii\n";

  for (const auto &pt : pts)
    ofs << pt.x << " " << pt.y << " " << pt.z << "\n";

  ofs.close();
}

//////////////////////////////////////////////////
void checkViewCloudLookup(const ::Camera &camera, const cv::Size size, cv::Mat &lookupX, cv::Mat &lookupY)
{
  if (!lookupX.empty() && !lookupY.empty())
    return;

  const float fx = 1.0f / camera.matrix.at<float>(0, 0);
  const float fy = 1.0f / camera.matrix.at<float>(1, 1);
  const float cx = camera.matrix.at<float>(0, 2);
  const float cy = camera.matrix.at<float>(1, 2);
  float *it;

  lookupY = cv::Mat(1, size.height, CV_32F);
  it = lookupY.ptr<float>();
  for(size_t r = 0; r < size.height; ++r, ++it)
  {
    *it = (r - cy) * fy;
  }

  lookupX = cv::Mat(1, size.width, CV_32F);
  it = lookupX.ptr<float>();
  for(size_t c = 0; c < size.width; ++c, ++it)
  {
    *it = (c - cx) * fx;
  }
}

//////////////////////////////////////////////////
std::tuple<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, std::vector<pcl::Vertices>>
  meshToPCLMesh(const ::Mesh &mesh, const cv::Mat &trans, const float tint) {
  std::vector<pcl::Vertices> polygons;

  for (const auto &tri : mesh.triangles) {
    pcl::Vertices vtc;
    vtc.vertices.push_back(tri[0]);
    vtc.vertices.push_back(tri[1]);
    vtc.vertices.push_back(tri[2]);

    polygons.push_back(vtc);
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZRGBA>};
  cloud->width = mesh.points.size();
  cloud->height = 1;
  cloud->is_dense = false;

  cloud->points.resize(cloud->width * cloud->height);

  for (size_t i = 0; i < mesh.points.size(); ++i) {
    auto pt = GeometryCV::transform(trans, mesh.points[i]);
    auto &cpt = cloud->points[i];
    cpt.x = -pt.x;
    cpt.y = -pt.y;
    cpt.z = -pt.z;
    cpt.r = 255*tint;
    cpt.g = 255*tint;
    cpt.b = 255*tint;
    cpt.a = 255;
  }

  return std::tie(cloud, polygons);
}

//////////////////////////////////////////////////
double distanceToScore(const double distance) noexcept {
  return -std::log(distance);
}

//////////////////////////////////////////////////
MeshFootprint::MeshFootprint(const ::Mesh &mesh, const ::PoseRT &pose,
    ::Camera &camera, const int im_size, const bool offset) {
  std::vector<cv::Point3f> points_3d;
  for (auto &pt : mesh.points) {
    if (offset)
      points_3d.push_back(pt - mesh.origin);
    else
      points_3d.push_back(pt);
  }

  std::vector<cv::Point2f> points_2d = GeometryCV::projectPoints(points_3d, pose, camera);

  cv::Rect_<float> b_rect = GeometryCV::getBoundingRect(points_2d);

  float rect_size = std::max(b_rect.width, b_rect.height);
  auto rate = im_size / rect_size;
  cv::Size fp_mat_size(b_rect.width*rate + 1, b_rect.height*rate + 1);

  // cv::Mat depth_img = cv::Mat::ones(fp_mat_size, CV_16UC1) * max_depth;
  // cv::Mat normal_img = cv::Mat::zeros(fp_mat_size, CV_32FC1);
  cv::Mat footprint_img = cv::Mat::zeros(fp_mat_size, CV_8UC1);

  for (auto &point : points_2d) {
    cv::Point2i xy = (point - b_rect.tl())*rate;

    assert(xy.x >= 0);
    assert(xy.y >= 0);
    assert(xy.x <= footprint_img.cols);
    assert(xy.y <= footprint_img.rows);

    xy.x = std::min(xy.x, footprint_img.cols-1);
    xy.y = std::min(xy.y, footprint_img.rows-1);

    point = xy;
  }

  for (const auto &tri : mesh.triangles) {
    std::vector<cv::Point2i> poly{
        points_2d[tri[0]],
        points_2d[tri[1]],
        points_2d[tri[2]]};

    cv::fillConvexPoly(footprint_img, poly, cv::Scalar(255));
    // TODO: Normal map for inner edges
    // Drawing::drawTriangleInterp(depth_img, normal_img, );
  }

  constexpr size_t marg_size = 3;
  cv::copyMakeBorder(footprint_img, footprint_img,
    marg_size, marg_size, marg_size, marg_size,
    cv::BORDER_CONSTANT | cv::BORDER_ISOLATED, cv::Scalar(0));

  cv::Mat tmp = footprint_img.clone();
  std::vector<std::vector<cv::Point2i>> contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::findContours(tmp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  assert(contours.size() == 1);

  cv::Mat bounding_matrix_inv = (cv::Mat_<float>(3, 3) << 1/rate, 0, b_rect.x - marg_size/rate,
                                                          0, 1/rate, b_rect.y - marg_size/rate, 0 , 0, 1);
  std::vector<cv::Point2f> contour = GeometryCV::transform(bounding_matrix_inv, contours[0]);

  this->outerEdge = contour;
  this->pose = pose;

  // cv::Canny(); // TODO
  // this->innerEdges
}

//////////////////////////////////////////////////
void MeshEdgeModel::addSampledFootprints(const size_t rotationAxisSamples, const size_t rotationAngleSamples, const size_t size) {
  auto it = std::max_element(this->mesh.points.cbegin(), this->mesh.points.cend(),
    [](const cv::Point3f &a, const cv::Point3f &b) {
      return cv::norm(a) < cv::norm(b); });
  float mlen = cv::norm(*it);

  const auto pi = 3.1415926f;
  for (int r_ax_i = 0; r_ax_i < rotationAxisSamples; ++r_ax_i) {
    float axis_inc = pi * r_ax_i / rotationAxisSamples;
    cv::Vec3f axis{std::cos(axis_inc), std::sin(axis_inc), 0};

    for (int r_ang_i = 0; r_ang_i < rotationAngleSamples; ++r_ang_i ) {
      // outInfo("Training sample (" << r_ax_i << ";" << r_ang_i << ")");

      float theta = pi * r_ang_i / rotationAngleSamples;
      auto rodrigues = axis*theta;

      ::PoseRT mesh_pose{rodrigues, {0, 0, -mlen*3.f}};

      this->addFootprint(mesh_pose, size);
    }
  }
}

//////////////////////////////////////////////////
void MeshEdgeModel::saveToFile(const std::string filename) {
  throw std::runtime_error("Not implemented");
}

//////////////////////////////////////////////////
bool MeshEdgeModel::loadFromFile(const std::string filename) {
  throw std::runtime_error("Not implemented");
}

//////////////////////////////////////////////////
TyErrorId ContourFittingClassifier::initialize(AnnotatorContext &ctx) {
  outInfo("initialize");

  ctx.extractValue("repairPointCloud", this->repairPointCloud);
  ctx.extractValue("rotationAxisSamples", this->rotation_axis_samples);
  ctx.extractValue("rotationAngleSamples", this->rotation_angle_samples);
  ctx.extractValue("footprintImageSize", this->footprint_image_size);
  ctx.extractValue("icp2d3dIterationsLimit", this->icp2d3dIterationsLimit);

  ctx.extractValue("performICPPoseRefinement", this->performICPPoseRefinement);
  ctx.extractValue("applySupportPlaneAssumption", this->applySupportPlaneAssumption);

  ctx.extractValue("rejectScoreLevel", this->rejectScoreLevel);
  ctx.extractValue("normalizedAcceptScoreLevel", this->normalizedAcceptScoreLevel);

  std::vector<std::string*> filenames;
  ctx.extractValue("referenceMeshes", filenames);

  for (auto &fname : filenames) {
    try {
      ::MeshEdgeModel edge_model;

      edge_model.mesh = ::Mesh::readFromFile(*fname);
      edge_model.addSampledFootprints(this->rotation_axis_samples,
                                      this->rotation_angle_samples,
                                      this->footprint_image_size);

      outInfo("Trained " << edge_model.items.size() << " samples");

      this->edge_models.emplace(*fname, edge_model);
    } catch (const std::exception &ex) {
      outError(ex.what());
    }
    delete fname;
  }

  if (this->edge_models.empty())
    outError("No valid meshes found");

  return UIMA_ERR_NONE;
}

//////////////////////////////////////////////////
TyErrorId ContourFittingClassifier::destroy()
{
  outInfo("destroy");
  return UIMA_ERR_NONE;
}

//////////////////////////////////////////////////
TyErrorId ContourFittingClassifier::processWithLock(CAS &tcas, ResultSpecification const &res_spec)
{
  outInfo("process start");
  rs::StopWatch clock;
  rs::SceneCas cas(tcas);

  // Read CAS Data
  cv::Mat cas_image_rgb;
  cv::Mat cas_image_depth;
  cv::Mat image_grayscale;

  if (!cas.get(VIEW_DEPTH_IMAGE, cas_image_depth)) {
    outError("No depth image");
    return UIMA_ERR_NONE;
  }

  if (!cas.get(VIEW_COLOR_IMAGE, cas_image_rgb)) {
    outError("No color image");
    return UIMA_ERR_NONE;
  }

  cv::resize(cas_image_rgb, this->image_rgb, cas_image_depth.size());
  cv::cvtColor(image_rgb, image_grayscale, CV_BGR2GRAY);

  if (!cas.get(VIEW_CLOUD, *view_cloud)) {
    outError("No view point cloud");
    return UIMA_ERR_NONE;
  }

  sensor_msgs::CameraInfo camInfo;
  cas.get(VIEW_CAMERA_INFO, camInfo);
  this->camera.setFromMsgs(camInfo);

  rs::Scene scene = cas.getScene();
  std::vector<rs::TransparentSegment> t_segments;
  scene.identifiables.filter(t_segments);
  outInfo("Found " << t_segments.size() << " transparent segments");

  this->segments.clear();
  this->fitted_silhouettes.clear();
  this->debug_points_red.clear();
  this->debug_points_blue.clear();
  this->labels.clear();
  this->pose_hypotheses.clear();
  this->histograms.clear();

  for (auto &t_segment : t_segments) {
    rs::Segment segment = t_segment.segment.get();

    rs::Plane plane = t_segment.supportPlane.get();
    std::vector<float> planeModel = plane.model();
    if(planeModel.size() != 4) {
      outError("No plane found!");
      continue;
    }

    cv::Vec3f plane_normal(planeModel[0], planeModel[1], planeModel[2]);
    double plane_distance = planeModel[3];

    std::vector<cv::Point2f> contour;
    for (auto &rs_pt : segment.contour.get()) {
      cv::Point2i cv_pt;
      rs::conversion::from(rs_pt, cv_pt);

      contour.push_back(cv::Point2f(cv_pt.x, cv_pt.y));
    }

    outInfo("\tContour of " << contour.size() << " points");

    ImageSegmentation::Segment i_segment;
    rs::conversion::from(segment, i_segment);

    std::vector<cv::Point2f> innerEdges = getCannyEdges(image_grayscale, i_segment.rect);
    if (innerEdges.size() == 0)
      continue;

    // this->debug_points_red.push_back(innerEdges);

    ::SimilarityRanking<PoseHypothesis> poseRanking;

    // will be reinitialised by first chamfer distance call
    cv::Mat dist_transform;
    GeometryCV::getChamferDistance(contour, contour, cas_image_depth.size(), dist_transform);
    double contour_stddev;
    std::tie(std::ignore, contour_stddev) = GeometryCV::getMeanAndStdDev(contour);

    for (const auto &kv : this->edge_models) {
      auto &mesh = kv.second.mesh;
      assert(mesh.points.size() >= 4);

      #pragma omp parallel for
      for (auto it = kv.second.items.cbegin(); it < kv.second.items.cend(); ++it) {
        ::PoseHypothesis hypothesis(kv.first, std::distance(kv.second.items.cbegin(), it), 0);

        cv::Mat pr_trans;
        std::tie(pr_trans, std::ignore) = GeometryCV::fitProcrustes2d(it->outerEdge, contour);

        std::vector<cv::Point3f> points_3d;
        for (auto &pt : mesh.points)
          points_3d.push_back(pt - mesh.origin);

        std::vector<cv::Point2f> points_2d = GeometryCV::projectPoints(points_3d, it->pose, this->footprint_camera);
        points_2d = GeometryCV::transform(pr_trans, points_2d);
        // #pragma omp critical
        // this->fitted_silhouettes.push_back(points_2d);

        cv::Mat r_vec(3, 1, cv::DataType<double>::type);
        cv::Mat t_vec(3, 1, cv::DataType<double>::type);

        cv::solvePnP(mesh.points, points_2d, this->camera.matrix, this->camera.distortion, r_vec, t_vec);

        ::PoseRT pose_3d;

        for (int i = 0; i < 3; ++i) {
          pose_3d.rot(i) = r_vec.at<double>(i);
          pose_3d.trans(i) = t_vec.at<double>(i);
        }

        hypothesis.pose = pose_3d;

        double distance, confidence;
        auto trans_mesh_fp = ::MeshFootprint(mesh, pose_3d, this->camera, this->footprint_image_size, false);
        std::tie(distance, confidence) = GeometryCV::getChamferDistance(trans_mesh_fp.outerEdge, contour, cas_image_depth.size(), dist_transform);

        distance /= contour_stddev; // All sane values should be in (0,1] range
        hypothesis.setScore(distanceToScore(distance));

        #pragma omp critical
        poseRanking.addElement(hypothesis);
      }
    }

    poseRanking.supressNonMaximum(1);

    poseRanking.filter(this->rejectScoreLevel);

    if (poseRanking.size() == 0) {
      outInfo("Low probable silhouette => rejecting");
      continue;
    }

    auto max_score = poseRanking.getMaxScore();

    auto histogram = poseRanking.getHistogram();
    cv::Mat hist_img = Drawing::drawHistogram(histogram, 3, 72, max_score*this->normalizedAcceptScoreLevel);
    this->histograms.push_back(hist_img);

    poseRanking.filter(max_score * this->normalizedAcceptScoreLevel);

    outInfo("Surface edges contains: " << innerEdges.size() << " points");

    // Refine obtained poses
    for (auto &hyp : poseRanking) {
      ::PoseRT new_pose;
      double distance = 0;
      cv::Mat jacobian;

      ::Mesh &mesh = this->edge_models[hyp.getClass()].mesh;

      if (this->performICPPoseRefinement) {
        auto edge_points_3d = getMeshSurfaceEdgesAtPose(mesh, hyp.pose, this->camera, cas_image_depth.size());

        outInfo("Running 2d-3d ICP ... ");
        std::tie(new_pose, distance, jacobian) = GeometryCV::fit2d3d(edge_points_3d, hyp.pose, innerEdges, this->camera, this->icp2d3dIterationsLimit);
        outInfo("\tdone: distance = " << distance);
        assert(distance < 10 && distance >= 0);
        if (std::isnormal(distance)) {
          // find new score
          ::MeshFootprint new_fp(mesh, hyp.pose, this->camera, this->footprint_image_size, false);
          std::tie(distance, std::ignore) = GeometryCV::getChamferDistance(new_fp.outerEdge, contour, cas_image_depth.size(), dist_transform);
          hyp.setScore(distanceToScore(distance));
          hyp.pose = new_pose;
        }

        if (this->applySupportPlaneAssumption) {
          hyp.pose = alignObjectsPoseWithPlane(hyp.pose, cv::Vec3f(0, 0, 0), plane_normal, plane_distance, this->camera, jacobian);
          edge_points_3d = getMeshSurfaceEdgesAtPose(mesh, hyp.pose, this->camera, cas_image_depth.size());

          outInfo("Running 2d-3d ICP2 ... ");
          std::tie(new_pose, distance, std::ignore) = GeometryCV::fit2d3d(edge_points_3d, hyp.pose, innerEdges, this->camera, this->icp2d3dIterationsLimit, plane_normal);
          if (std::isnormal(distance)) {
            // find new score
            ::MeshFootprint new_fp(mesh, hyp.pose, this->camera, this->footprint_image_size, false);
            std::tie(distance, std::ignore) = GeometryCV::getChamferDistance(new_fp.outerEdge, contour, cas_image_depth.size(), dist_transform);
            hyp.setScore(distanceToScore(distance));
            hyp.pose = new_pose;
          }
        }
      }
      else {
        if (this->applySupportPlaneAssumption) {
          hyp.pose = alignObjectsPoseWithPlane(hyp.pose, cv::Vec3f(0, 0, 0), plane_normal, plane_distance, this->camera, jacobian);

          // find new score
          ::MeshFootprint new_fp(mesh, hyp.pose, this->camera, this->footprint_image_size, false);
          std::tie(distance, std::ignore) = GeometryCV::getChamferDistance(new_fp.outerEdge, contour, cas_image_depth.size(), dist_transform);

          hyp.setScore(distanceToScore(distance / contour_stddev));
        }
      }
    }

    outInfo("Has " << poseRanking.size() << " hypotheses for the segment");
    auto top_hypotheses = poseRanking.getTop(1);
    if (this->repairPointCloud && (top_hypotheses.size() > 0)) {
      const auto mesh = this->edge_models[top_hypotheses[0].getClass()].mesh;
      auto points_2d = GeometryCV::projectPoints(mesh.points, top_hypotheses[0].pose, this->camera);
      this->fitted_silhouettes.push_back(points_2d);

      this->segments.push_back(i_segment);
      this->labels.push_back(top_hypotheses[0].getClass());

      this->drawHypothesisToCAS(tcas, cas_image_depth, view_cloud, top_hypotheses[0], this->camera);
    }

    // cv::imwrite("/tmp/color.png", cas_image_rgb);
    // cv::imwrite("/tmp/depth.png", cas_image_depth);
    // break;
  }

  outInfo("took: " << clock.getTime() << " ms.");
  return UIMA_ERR_NONE;
}

//////////////////////////////////////////////////
void ContourFittingClassifier::drawImageWithLock(cv::Mat &disp) {
  cv::Mat gray;
  cv::cvtColor(this->image_rgb, gray, CV_BGR2GRAY);

  for (auto seg : this->segments) {
    auto roi = seg.rect;

    constexpr int margin = 5;
    roi.x -= margin;
    roi.y -= margin;
    roi.width += 2*margin;
    roi.height += 2*margin;

    cv::Mat gray_roi = gray(roi);

    // Compute optimal thresh value
    cv::Mat not_used;
    double otsu_thresh_val = cv::threshold(gray_roi, not_used, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    cv::Canny(gray_roi, gray_roi, otsu_thresh_val/3, otsu_thresh_val);
  }

  cv::cvtColor(gray, disp, CV_GRAY2BGR);

  ImageSegmentation::drawSegments2D(disp, this->segments, this->labels, 1, 0.5);

  for (const auto &sil : this->fitted_silhouettes) {
    for (const auto &pt : sil)
      cv::circle(disp, pt, 1, cv::Scalar(0, 0, 255), -1);
  }

  cv::Mat transpR = cv::Mat::zeros(disp.size(), CV_8UC3);
  cv::Mat transpB = cv::Mat::zeros(disp.size(), CV_8UC3);
  for (const auto &sil : this->debug_points_red) {
    for (auto &pt : sil)
      cv::circle(transpR, pt, 1, cv::Scalar(0, 0, 255), -1);
  }

  for (const auto &sil : this->debug_points_blue) {
    for (auto &pt : sil)
      cv::circle(transpB, pt, 1, cv::Scalar(255, 0, 0), -1);
  }


  for (size_t i = 0; i < this->segments.size(); ++i) {
    auto segment = segments[i];
    auto &seg_center = segment.center;
    cv::Rect hist_dst_rect(cv::Point(seg_center.x, seg_center.y + 5 + segment.rect.height/2), histograms[i].size());
    hist_dst_rect = hist_dst_rect & cv::Rect(cv::Point(), disp.size());
    disp(hist_dst_rect) *= 0.5;
    cv::Rect hist_src_rect = (hist_dst_rect - hist_dst_rect.tl()) & cv::Rect(cv::Point(), histograms[i].size());
    disp(hist_dst_rect) += histograms[i](hist_src_rect);
  }

  cv::Mat depth_map;
  distance_mat.convertTo(depth_map, CV_8UC1, 255/1500.0, 0);
  cv::cvtColor(depth_map, depth_map, CV_GRAY2BGR);
  cv::resize(depth_map, depth_map, cv::Size(), 0.25, 0.25);

  cv::Rect depth_roi(disp.cols - depth_map.cols, disp.rows - depth_map.rows,
      depth_map.cols, depth_map.rows);
  depth_map.copyTo(disp(depth_roi));

  disp += transpR + transpB;
}

//////////////////////////////////////////////////
void ContourFittingClassifier::fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
{
  const std::string &cloudname = "ContourFittingClassifier";
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outCloud;

  outCloud = this->view_cloud;

  if (firstRun)
    visualizer.addPointCloud(outCloud, cloudname);
  else {
    visualizer.updatePointCloud(outCloud, cloudname);
    visualizer.removeAllShapes();
  }

  if (this->visualizeSolidMeshes) {
    int i = 0;
    for (const auto &seg_hyp : this->pose_hypotheses) {
      for (const auto &hyp : seg_hyp) {
        std::string pcl_mesh_name = "_" + std::to_string(i);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
        std::vector<pcl::Vertices> polygons;

        cv::Mat affine_3d_transform = GeometryCV::poseRTToAffine(hyp.pose);
        std::tie(cloud, polygons) = meshToPCLMesh(this->edge_models[hyp.getClass()].mesh,
                                                  affine_3d_transform, (hyp.getScore() - 0.85) / 0.15);

        visualizer.removeShape(pcl_mesh_name);

        auto result = visualizer.addPolygonMesh<pcl::PointXYZRGBA>(cloud, polygons, pcl_mesh_name);
        assert(result);

        ++i;
      }
    }
  }
}

//////////////////////////////////////////////////
void ContourFittingClassifier::drawHypothesisToCAS(CAS &tcas, cv::Mat &cas_image_depth, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cas_view_cloud, const ::PoseHypothesis &hypothesis, const ::Camera &camera) {
  rs::SceneCas cas(tcas);

  auto &mesh = this->edge_models[hypothesis.getClass()].mesh;

  cv::Mat depth_map = cv::Mat::ones(cas_image_depth.size(), CV_32FC1) * Drawing::max_depth_32f;
  Drawing::drawMeshDepth(depth_map, mesh.points, mesh.triangles, hypothesis.pose.rot, hypothesis.pose.trans, camera.matrix, camera.distortion);

  cv::Mat depth_u16;
  depth_map.convertTo(depth_u16, CV_16UC1, 1000);
  depth_u16.copyTo(cas_image_depth, (depth_map != Drawing::max_depth_32f));

  checkViewCloudLookup(camera, depth_map.size(), this->lookupX, this->lookupY);

  int counter = 0;
  for (int i = 0; i < cas_view_cloud->width; ++i) {
    for (int j = 0; j < cas_view_cloud->height; ++j) {
      int id = j*cas_view_cloud->width + i;
      if (depth_map.at<float>(j, i) != Drawing::max_depth_32f) {
        float depth_value = depth_map.at<float>(j, i);

        cas_view_cloud->points[id].x = depth_value * this->lookupX.at<float>(i);
        cas_view_cloud->points[id].y = depth_value * this->lookupY.at<float>(j);
        cas_view_cloud->points[id].z = depth_value;

        auto rgb = this->image_rgb.at<cv::Vec<uint8_t, 3>>(j, i);

        cas_view_cloud->points[id].r = rgb(0);
        cas_view_cloud->points[id].g = rgb(1);
        cas_view_cloud->points[id].b = rgb(2);
        cas_view_cloud->points[id].a = 255;

        ++counter;
      }
    }
  }

  outInfo("PC updated with " << counter << " points");

  {
    rs::Scene scene = cas.getScene();

    std::stringstream sstr;

    rs::Classification rsClassification  = rs::create<rs::Classification>(tcas);
    rsClassification.classification_type.set("");
    rsClassification.classname.set(hypothesis.getClass());
    rsClassification.featurename.set("outer silhouette and canny edges");
    rsClassification.classifier.set("NearestNeighbor");

    sstr << "rotationAngleSamples: " << this->rotation_angle_samples << ";"
     << "rotationAxisSamples: " << this->rotation_axis_samples << ";"
     << "footprintImageSize: " << this->footprint_image_size << ";"
     << "rejectScoreLevel: " << this->rejectScoreLevel << ";"
     << "normalizedAcceptScoreLevel: " << this->normalizedAcceptScoreLevel << ";"
     << "performICPPoseRefinement: " << this->performICPPoseRefinement << ";"
     << "icp2d3dIterationsLimit: " << this->icp2d3dIterationsLimit << ";"
     << "applySupportPlaneAssumption: " << this->applySupportPlaneAssumption << ";";
    rsClassification.parameters.set(sstr.str());

    sstr.clear();
    sstr.str(std::string());
    for (const auto &em : this->edge_models)
      sstr << em.first << ";";
    rsClassification.model.set(sstr.str());

    rs::ClassConfidence rsClassConfidence = rs::create<rs::ClassConfidence>(tcas);
    rsClassConfidence.name.set(hypothesis.getClass());
    rsClassConfidence.score.set(1.f); // FIXME
    rsClassification.confidences.append(rsClassConfidence);

    // FIXME: annotate cluster
    scene.identifiables.append(rsClassification);

    // Submit pose
    tf::Matrix3x3 rot;
    tf::Vector3 trans;

    cv::Mat cvRot;
    cv::Rodrigues(hypothesis.pose.rot, cvRot);

    rot.setValue(cvRot.at<float>(0),
                 cvRot.at<float>(1),
                 cvRot.at<float>(2),
                 cvRot.at<float>(3),
                 cvRot.at<float>(4),
                 cvRot.at<float>(5),
                 cvRot.at<float>(6),
                 cvRot.at<float>(7),
                 cvRot.at<float>(8));
    trans.setValue(hypothesis.pose.trans(0),
                   hypothesis.pose.trans(1),
                   hypothesis.pose.trans(2));

    std::string frameId("idk");
    auto tfStampedPose = tf::Stamped<tf::Pose>(tf::Pose(rot, trans),
        ros::Time(/*pose.timestamp.get()*/0 / 1000000000, /*pose.timestamp.get()*/0 % 1000000000),
        frameId);

    rs::PoseAnnotation rsPoseAnnotation = rs::create<rs::PoseAnnotation>(tcas);
    rsPoseAnnotation.camera.set(rs::conversion::to(tcas, tfStampedPose));

    // FIXME: annotate cluster
    scene.identifiables.append(rsPoseAnnotation);
  }

  cas.set(VIEW_DEPTH_IMAGE, cas_image_depth);
  cas.set(VIEW_CLOUD, *cas_view_cloud);

  cas_image_depth.copyTo(this->distance_mat);
}

//////////////////////////////////////////////////
::PoseRT ContourFittingClassifier::alignObjectsPoseWithPlane(const ::PoseRT &initial_pose,const cv::Vec3f mesh_anchor_point,const cv::Vec3f support_plane_normal, const float support_plane_distance, ::Camera &camera, cv::Mat &jacobian) {
  auto spd = support_plane_distance;

  // find anchor point
  auto objects_anchor_point_camspace = GeometryCV::transform(initial_pose, mesh_anchor_point);
  auto view_ray = objects_anchor_point_camspace / cv::norm(objects_anchor_point_camspace);

  // find anchor point offset along view ray
  auto lambda = spd / support_plane_normal.ddot(view_ray);
  auto anchor_offset = lambda*view_ray - objects_anchor_point_camspace;

  // find axes rotation transformation to align object's up to plane normal
  cv::Vec3f objects_up_local(0, 1, 0);
  ::PoseRT object_to_camera_rotation {initial_pose.rot, cv::Vec3f(0, 0, 0)};
  auto objects_up_camspace = GeometryCV::transform(object_to_camera_rotation, objects_up_local);

  double phi = std::acos(support_plane_normal.ddot(objects_up_camspace));
  // phi = std::min(phi, M_PI - phi);

  cv::Vec3f up_to_n_rot_axis = objects_up_camspace.cross(support_plane_normal);
  cv::Vec3f rodrigues_up_to_n = up_to_n_rot_axis * (phi / cv::norm(up_to_n_rot_axis));

  ::PoseRT aligned_to_plane_objects_pose;

#define LOCAL_AMBIGUITY_RESOLUTION 0
#if LOCAL_AMBIGUITY_RESOLUTION
  if (jacobian.empty()) {
    outError("Jacobian is empty, returning original pose");
    return initial_pose;
  }

  using namespace GeometryCV;

  cv::Mat E = (cv::Mat_<double>(4, 6) <<
    0, 0, 0, support_plane_normal(0), support_plane_normal(1), support_plane_normal(2),
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0);

  cv::Mat Et = E.t();

  cv::Mat d = (cv::Mat_<double>(4, 1) <<
    cv::norm(anchor_offset),
    rodrigues_up_to_n(0),
    rodrigues_up_to_n(1),
    rodrigues_up_to_n(2));

  cv::Mat jacobian_d;
  jacobian.convertTo(jacobian_d, CV_64FC1);

  cv::Mat Q = jacobian_d.t() * jacobian_d;

  cv::Mat A = cv::Mat::eye(10, 10, CV_64FC1);
  Q.copyTo(A(cv::Rect(0, 0, 6, 6)));
  E.copyTo(A(cv::Rect(0, 6, E.cols, E.rows)));
  Et.copyTo(A(cv::Rect(6, 0, Et.cols, Et.rows)));

  cv::Mat b = cv::Mat::zeros(10, 1, CV_64FC1);
  d.copyTo(b(cv::Rect(0, 6, 1, 4)));

  cv::Mat x;
  cv::solve(A, b, x, cv::DECOMP_SVD);

  outDebug("Support plane pose delta: " << x.t());

  ::PoseRT align_to_plane_objects_pose {
    cv::Vec3f{static_cast<float>(x.at<double>(0)),
              static_cast<float>(x.at<double>(1)),
              static_cast<float>(x.at<double>(2))},
    cv::Vec3f{static_cast<float>(x.at<double>(3)),
              static_cast<float>(x.at<double>(4)),
              static_cast<float>(x.at<double>(5))}};

  aligned_to_plane_objects_pose = initial_pose + align_to_plane_objects_pose;
#else
  cv::Mat up_to_n_rotation;
  cv::Rodrigues(rodrigues_up_to_n, up_to_n_rotation);

  aligned_to_plane_objects_pose.trans = initial_pose.trans + anchor_offset;

  cv::Mat initial_rotation;
  cv::Rodrigues(initial_pose.rot, initial_rotation);
  cv::Mat final_rotation = up_to_n_rotation * initial_rotation;

  cv::Rodrigues(final_rotation, aligned_to_plane_objects_pose.rot);
#endif

  return aligned_to_plane_objects_pose;
}

//////////////////////////////////////////////////
std::vector<cv::Point3f> ContourFittingClassifier::getMeshSurfaceEdgesAtPose(const ::Mesh &mesh, const ::PoseRT &pose, const ::Camera &camera, const cv::Size image_size) {
  cv::Mat z_buffer = cv::Mat::ones(image_size, CV_32FC1) * Drawing::max_depth_32f;
  cv::Mat normal_map = cv::Mat::zeros(image_size, CV_32FC3);

  Drawing::drawMeshNormals(z_buffer, normal_map, mesh.points, mesh.normals, mesh.triangles,
      pose.rot, pose.trans, camera.matrix, camera.distortion);

  cv::Mat dot_map = cv::Mat::zeros(normal_map.size(), CV_32FC1);

  cv::Vec3f cam_vec(0, 0, -1);
  for (int i = 0; i < image_size.height; ++i)
    for (int j = 0; j < image_size.width; ++j) {
      float val = cam_vec.dot(normal_map.at<cv::Vec3f>(i, j));
      dot_map.at<float>(i, j) = val;
    }

  cv::normalize(dot_map, dot_map, 0, 1, cv::NORM_MINMAX);
  dot_map.convertTo(dot_map, CV_8UC1, 255);
  cv::Canny(dot_map, dot_map, 60, 128);

  std::vector<cv::Point3f> points_3d;

  checkViewCloudLookup(camera, image_size, this->lookupX, this->lookupY);

  for (int i = 0; i < image_size.height; ++i)
    for (int j = 0; j < image_size.width; ++j) {
      if ((dot_map.at<uint8_t>(i, j) == 255) && (z_buffer.at<float>(i, j) != Drawing::max_depth_32f)) {
        cv::Point3f pt3;

        pt3.z = -z_buffer.at<float>(i, j);

        pt3.x = this->lookupX.at<float>(j) * pt3.z;
        pt3.y = this->lookupY.at<float>(i) * pt3.z;

        points_3d.push_back(pt3);
      }
    }

  for (auto &pt : points_3d) {
    pt.x -= pose.trans(0);
    pt.y -= pose.trans(1);
    pt.z -= pose.trans(2);
  }

  PoseRT to_origin_rot{-pose.rot, {0., 0., 0.}};
  cv::Mat to_origin = GeometryCV::poseRTToAffine(to_origin_rot);
  auto result = GeometryCV::transform(to_origin, points_3d);

  return result;
}

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ContourFittingClassifier)
