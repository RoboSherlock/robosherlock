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

  outInfo("Loading mesh: " << path);

  if (!ifs.good())
    throw std::invalid_argument("File '" + path + "' not found");

  enum class PLYSection : int { HEADER = 0, VERTEX, FACE};
  std::map<PLYSection, int> counts;

  auto minf = std::numeric_limits<float>::min();
  auto maxf = std::numeric_limits<float>::max();
  cv::Point3f minPoint(maxf, maxf, maxf);
  cv::Point3f maxPoint(minf, minf, minf);

  PLYSection readingSection = PLYSection::HEADER;
  for (std::string line; std::getline(ifs, line);) {
    if (readingSection == PLYSection::HEADER) {
      if (line.find("element face") == 0)
        counts[PLYSection::FACE] =
            std::atoi(line.substr(line.rfind(" ")).c_str());
      if (line.find("element vertex") == 0)
        counts[PLYSection::VERTEX] =
            std::atoi(line.substr(line.rfind(" ")).c_str());
      if (line.find("end_header") == 0) {
        readingSection = PLYSection::VERTEX;
        outInfo("Vertices / normals: " << counts[PLYSection::VERTEX]);
        outInfo("Faces: " << counts[PLYSection::FACE]);
      }
    }
    else if (readingSection == PLYSection::VERTEX) {
      if (0 < counts[readingSection]) {
        std::istringstream iss(line);

        cv::Point3f point;
        cv::Point3f normal;
        iss >> point.x >> point.y >> point.z
            >> normal.x >> normal.y >> normal.z;

        minPoint.x = std::min(point.x, minPoint.x);
        minPoint.y = std::min(point.y, minPoint.y);
        minPoint.z = std::min(point.z, minPoint.z);

        maxPoint.x = std::max(point.x, maxPoint.x);
        maxPoint.y = std::max(point.y, maxPoint.y);
        maxPoint.z = std::max(point.z, maxPoint.z);

        points.push_back(point);
        normals.push_back(normal);
        --counts[readingSection];
      }
      else
        readingSection = PLYSection::FACE;
    }
    if (readingSection == PLYSection::FACE) {
      if (0 == counts[readingSection]--)
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

  cv::Point3f offset = maxPoint - minPoint;
  for (auto &point : points) {
    // point -= offset;
    point *= scale;
  }

  cv::Point3f origin = offset*scale;

  return {points, normals, triangles, origin};
}

//////////////////////////////////////////////////
std::vector<cv::Point2f> getCannyEdges(const cv::Mat &grayscale,
                                       const cv::Rect &inputROI) {
  auto roi = inputROI;

  constexpr int margin = 5;
  roi.x -= margin;
  roi.y -= margin;
  roi.width += 2*margin;
  roi.height += 2*margin;

  cv::Mat grayROI = grayscale(roi);

  // Compute optimal thresh value
  cv::Mat notUsed;
  double otsuThreshold = cv::threshold(grayROI, notUsed, 0, 255,
                                       CV_THRESH_BINARY | CV_THRESH_OTSU);
  cv::Canny(grayROI, grayROI, otsuThreshold/3, otsuThreshold);

  std::vector<cv::Point2f> result;

  for(int row = 0; row < grayROI.rows; ++row) {
    uint8_t *ptr = grayROI.ptr(row);
    for(int col = 0; col < grayROI.cols; ++col, ++ptr) {
      if (*ptr != 0)
        result.push_back(cv::Point2f(col + roi.x, row + roi.y));
    }
  }

  return result;
}

//////////////////////////////////////////////////
void saveToFile(const std::string filename,
                const std::vector<cv::Point3f> &points) {
  std::ofstream ofs(filename);

  ofs << "VERSION .7\n"
      << "FIELDS x y z\n"
      << "SIZE 4 4 4\n"
      << "TYPE F F F\n"
      << "COUNT 1 1 1\n"
      << "WIDTH " << points.size() << "\n"
      << "HEIGHT 1\n"
      << "VIEWPOINT 0 0 0 1 0 0 0\n"
      << "POINTS " << points.size() << "\n"
      << "DATA ascii\n";

  for (const auto &point : points)
    ofs << point.x << " " << point.y << " " << point.z << "\n";

  ofs.close();
}

//////////////////////////////////////////////////
void checkViewCloudLookup(const ::Camera &camera, const cv::Size size,
                          cv::Mat &lookupX, cv::Mat &lookupY)
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
    *it = (r - cy) * fy;

  lookupX = cv::Mat(1, size.width, CV_32F);
  it = lookupX.ptr<float>();
  for(size_t c = 0; c < size.width; ++c, ++it)
    *it = (c - cx) * fx;
}

//////////////////////////////////////////////////
std::tuple<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, std::vector<pcl::Vertices>>
  meshToPCLMesh(const ::Mesh &mesh, const cv::Mat &trans, const float tint) {
  std::vector<pcl::Vertices> polygons;

  for (const auto &triangle : mesh.triangles) {
    pcl::Vertices vertices;
    vertices.vertices.push_back(triangle[0]);
    vertices.vertices.push_back(triangle[1]);
    vertices.vertices.push_back(triangle[2]);

    polygons.push_back(vertices);
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud{
      new pcl::PointCloud<pcl::PointXYZRGBA>};
  cloud->width = mesh.points.size();
  cloud->height = 1;
  cloud->is_dense = false;

  cloud->points.resize(cloud->width * cloud->height);

  for (size_t i = 0; i < mesh.points.size(); ++i) {
    auto pt = GeometryCV::transform(trans, mesh.points[i]);
    auto &cloudPoint = cloud->points[i];
    cloudPoint.x = -pt.x;
    cloudPoint.y = -pt.y;
    cloudPoint.z = -pt.z;
    cloudPoint.r = 255*tint;
    cloudPoint.g = 255*tint;
    cloudPoint.b = 255*tint;
    cloudPoint.a = 255;
  }

  return std::tie(cloud, polygons);
}

//////////////////////////////////////////////////
double distanceToScore(const double distance) noexcept {
  return -std::log(distance);
}

//////////////////////////////////////////////////
MeshFootprint::MeshFootprint(const ::Mesh &mesh, const ::PoseRT &pose,
    ::Camera &camera, const int imageSize, const bool offset) {
  std::vector<cv::Point3f> points3d;
  for (auto &point : mesh.points) {
    if (offset)
      points3d.push_back(point - mesh.origin);
    else
      points3d.push_back(point);
  }

  std::vector<cv::Point2f> projectedPoints =
      GeometryCV::projectPoints(points3d, pose, camera);

  cv::Rect_<float> projectedPointsBoundingRect =
      GeometryCV::getBoundingRect(projectedPoints);

  float projectedRectSize = std::max(projectedPointsBoundingRect.width,
                                     projectedPointsBoundingRect.height);
  auto scalingRate = imageSize / projectedRectSize;
  cv::Size fp_mat_size(projectedPointsBoundingRect.width*scalingRate + 1,
                       projectedPointsBoundingRect.height*scalingRate + 1);

  // cv::Mat depth_img = cv::Mat::ones(fp_mat_size, CV_16UC1) * max_depth;
  // cv::Mat normal_img = cv::Mat::zeros(fp_mat_size, CV_32FC1);
  cv::Mat footprintImage = cv::Mat::zeros(fp_mat_size, CV_8UC1);

  for (auto &point : projectedPoints) {
    cv::Point2i xy = (point - projectedPointsBoundingRect.tl())*scalingRate;

    assert(xy.x >= 0);
    assert(xy.y >= 0);
    assert(xy.x <= footprintImage.cols);
    assert(xy.y <= footprintImage.rows);

    xy.x = std::min(xy.x, footprintImage.cols-1);
    xy.y = std::min(xy.y, footprintImage.rows-1);

    point = xy;
  }

  for (const auto &triangle : mesh.triangles) {
    std::vector<cv::Point2i> poly{
        projectedPoints[triangle[0]],
        projectedPoints[triangle[1]],
        projectedPoints[triangle[2]]};

    cv::fillConvexPoly(footprintImage, poly, cv::Scalar(255));
    // TODO: Normal map for inner edges
    // Drawing::drawTriangleInterp(depth_img, normal_img, );
  }

  constexpr size_t marg_size = 3;
  cv::copyMakeBorder(footprintImage, footprintImage,
    marg_size, marg_size, marg_size, marg_size,
    cv::BORDER_CONSTANT | cv::BORDER_ISOLATED, cv::Scalar(0));

  cv::Mat tmp = footprintImage.clone();
  std::vector<std::vector<cv::Point2i>> contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::findContours(tmp, contours, hierarchy,
                   CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  assert(contours.size() == 1);

  cv::Mat shiftScaleBackTransformation = (cv::Mat_<float>(3, 3) <<
      1/scalingRate, 0, projectedPointsBoundingRect.x - marg_size/scalingRate,
      0, 1/scalingRate, projectedPointsBoundingRect.y - marg_size/scalingRate,
      0 , 0, 1);
  std::vector<cv::Point2f> contour =
      GeometryCV::transform(shiftScaleBackTransformation, contours[0]);

  this->outerEdge = contour;
  this->pose = pose;

  // cv::Canny(); // TODO
  // this->innerEdges
}

//////////////////////////////////////////////////
void MeshEdgeModel::addSampledFootprints(const size_t rotationAxisSamples,
                                         const size_t rotationAngleSamples,
                                         const size_t size) {
  auto it = std::max_element(
    this->mesh.points.cbegin(), this->mesh.points.cend(),
    [](const cv::Point3f &a, const cv::Point3f &b) {
      return cv::norm(a) < cv::norm(b); });
  float meshRadius = cv::norm(*it);

  for (int r_ax_i = 0; r_ax_i < rotationAxisSamples; ++r_ax_i) {
    float axisAngle = M_PI * r_ax_i / rotationAxisSamples;
    cv::Vec3f axis{std::cos(axisAngle), std::sin(axisAngle), 0};

    for (int r_ang_i = 0; r_ang_i < rotationAngleSamples; ++r_ang_i ) {
      // outInfo("Training sample (" << r_ax_i << ";" << r_ang_i << ")");

      float theta = M_PI * r_ang_i / rotationAngleSamples;
      auto rodrigues = axis*theta;

      ::PoseRT mesh_pose{rodrigues, {0, 0, -meshRadius*3.f}};

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
  ctx.extractValue("rotationAxisSamples", this->rotationAxisSamples);
  ctx.extractValue("rotationAngleSamples", this->rotationAngleSamples);
  ctx.extractValue("footprintImageSize", this->footprintImageSize);
  ctx.extractValue("icp2d3dIterationsLimit", this->icp2d3dIterationsLimit);

  ctx.extractValue("performICPPoseRefinement", this->performICPPoseRefinement);
  ctx.extractValue("applySupportPlaneAssumption",
      this->applySupportPlaneAssumption);

  ctx.extractValue("rejectScoreLevel", this->rejectScoreLevel);
  ctx.extractValue("normalizedAcceptScoreLevel",
      this->normalizedAcceptScoreLevel);

  std::vector<std::string*> filenames;
  ctx.extractValue("referenceMeshes", filenames);

  for (auto &fname : filenames) {
    try {
      ::MeshEdgeModel edgeModel;

      edgeModel.mesh = ::Mesh::readFromFile(*fname);
      edgeModel.addSampledFootprints(this->rotationAxisSamples,
                                      this->rotationAngleSamples,
                                      this->footprintImageSize);

      outInfo("Trained " << edgeModel.items.size() << " samples");

      this->edgeModels.emplace(*fname, edgeModel);
    } catch (const std::exception &ex) {
      outError(ex.what());
    }
    delete fname;
  }

  if (this->edgeModels.empty())
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
TyErrorId ContourFittingClassifier::processWithLock(CAS &tcas,
    ResultSpecification const &res_spec)
{
  outInfo("process start");
  rs::StopWatch clock;
  rs::SceneCas cas(tcas);

  // Read CAS Data
  cv::Mat casImageRGB;
  cv::Mat casImageDepth;
  cv::Mat imageGrayscale;

  if (!cas.get(VIEW_DEPTH_IMAGE, casImageDepth)) {
    outError("No depth image");
    return UIMA_ERR_NONE;
  }

  if (!cas.get(VIEW_COLOR_IMAGE, casImageRGB)) {
    outError("No color image");
    return UIMA_ERR_NONE;
  }

  cv::resize(casImageRGB, this->imageRGB, casImageDepth.size());
  cv::cvtColor(imageRGB, imageGrayscale, CV_BGR2GRAY);

  if (!cas.get(VIEW_CLOUD, *viewCloud)) {
    outError("No view point cloud");
    return UIMA_ERR_NONE;
  }

  sensor_msgs::CameraInfo camInfo;
  cas.get(VIEW_CAMERA_INFO, camInfo);
  this->camera.setFromMsgs(camInfo);

  rs::Scene scene = cas.getScene();
  std::vector<rs::TransparentSegment> transparentSegments;
  scene.identifiables.filter(transparentSegments);
  outInfo("Found " << transparentSegments.size() << " transparent segments");

  this->segments.clear();
  this->fittedSilhouettes.clear();
  this->debugPointsRed.clear();
  this->debugPointsBlue.clear();
  this->labels.clear();
  this->poseHypotheses.clear();
  this->histograms.clear();

  for (auto &tSegment : transparentSegments) {
    rs::Segment segment = tSegment.segment.get();

    rs::Plane plane = tSegment.supportPlane.get();
    std::vector<float> planeModel = plane.model();
    if(planeModel.size() != 4) {
      outError("No plane found!");
      continue;
    }

    cv::Vec3f plane_normal(planeModel[0], planeModel[1], planeModel[2]);
    double distanceToPlaneSigned = planeModel[3];

    std::vector<cv::Point2f> contour;
    for (auto &rsPoint : segment.contour.get()) {
      cv::Point2i cvPoint;
      rs::conversion::from(rsPoint, cvPoint);

      contour.push_back(cv::Point2f(cvPoint.x, cvPoint.y));
    }

    outInfo("\tContour of " << contour.size() << " points");

    ImageSegmentation::Segment segmentationSegment;
    rs::conversion::from(segment, segmentationSegment);

    std::vector<cv::Point2f> innerEdges = getCannyEdges(imageGrayscale, segmentationSegment.rect);
    if (innerEdges.size() == 0)
      continue;

    // this->debugPointsRed.push_back(innerEdges);

    ::SimilarityRanking<PoseHypothesis> poseRanking;

    // will be reinitialised by first chamfer distance call
    cv::Mat distanceTransformCache;
    GeometryCV::getChamferDistance(contour, contour, casImageDepth.size(), distanceTransformCache);
    double contourStdDev;
    std::tie(std::ignore, contourStdDev) = GeometryCV::getMeanAndStdDev(contour);

    for (const auto &kv : this->edgeModels) {
      auto &mesh = kv.second.mesh;
      assert(mesh.points.size() >= 4);

      #pragma omp parallel for
      for (auto it = kv.second.items.cbegin(); it < kv.second.items.cend(); ++it) {
        ::PoseHypothesis hypothesis(kv.first, std::distance(kv.second.items.cbegin(), it), 0);

        cv::Mat procrustesTransformation;
        std::tie(procrustesTransformation, std::ignore) = GeometryCV::fitProcrustes2d(it->outerEdge, contour);

        std::vector<cv::Point3f> points3d;
        for (auto &point : mesh.points)
          points3d.push_back(point - mesh.origin);

        std::vector<cv::Point2f> projectedPoints = GeometryCV::projectPoints(points3d, it->pose, this->footprintCamera);
        projectedPoints = GeometryCV::transform(procrustesTransformation, projectedPoints);
        // #pragma omp critical
        // this->fittedSilhouettes.push_back(projectedPoints);

        cv::Mat rotVec(3, 1, cv::DataType<double>::type);
        cv::Mat transVec(3, 1, cv::DataType<double>::type);

        cv::solvePnP(mesh.points, projectedPoints, this->camera.matrix, this->camera.distortion, rotVec, transVec);

        ::PoseRT pose;

        for (int i = 0; i < 3; ++i) {
          pose.rot(i) = rotVec.at<double>(i);
          pose.trans(i) = transVec.at<double>(i);
        }

        hypothesis.pose = pose;

        double distance, confidence;
        auto tmpFootprint = ::MeshFootprint(mesh, pose, this->camera, this->footprintImageSize, false);
        std::tie(distance, confidence) = GeometryCV::getChamferDistance(tmpFootprint.outerEdge, contour, casImageDepth.size(), distanceTransformCache);

        distance /= contourStdDev; // All sane values should be in (0,1] range
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

    auto maxScore = poseRanking.getMaxScore();

    auto histogram = poseRanking.getHistogram();
    cv::Mat histogramImage = Drawing::drawHistogram(histogram, 3, 72, maxScore*this->normalizedAcceptScoreLevel);
    this->histograms.push_back(histogramImage);

    poseRanking.filter(maxScore * this->normalizedAcceptScoreLevel);

    outInfo("Surface edges contains: " << innerEdges.size() << " points");

    // Refine obtained poses
    for (auto &hyp : poseRanking) {
      ::PoseRT newPose;
      double distance = 0;
      cv::Mat jacobian;

      ::Mesh &mesh = this->edgeModels[hyp.getClass()].mesh;

      if (this->performICPPoseRefinement) {
        auto edgePoints3d = getMeshSurfaceEdgesAtPose(mesh, hyp.pose, this->camera, casImageDepth.size());

        outInfo("Running 2d-3d ICP ... ");
        std::tie(newPose, distance, jacobian) = GeometryCV::fit2d3d(edgePoints3d, hyp.pose, innerEdges, this->camera, this->icp2d3dIterationsLimit);
        outInfo("\tdone: distance = " << distance);
        assert(distance < 10 && distance >= 0);
        if (std::isnormal(distance)) {
          // find new score
          ::MeshFootprint tmpFootprint(mesh, hyp.pose, this->camera, this->footprintImageSize, false);
          std::tie(distance, std::ignore) = GeometryCV::getChamferDistance(tmpFootprint.outerEdge, contour, casImageDepth.size(), distanceTransformCache);
          hyp.setScore(distanceToScore(distance));
          hyp.pose = newPose;
        }

        if (this->applySupportPlaneAssumption) {
          hyp.pose = alignObjectsPoseWithPlane(hyp.pose, cv::Vec3f(0, 0, 0), plane_normal, distanceToPlaneSigned, this->camera, jacobian);
          edgePoints3d = getMeshSurfaceEdgesAtPose(mesh, hyp.pose, this->camera, casImageDepth.size());

          outInfo("Running 2d-3d ICP2 ... ");
          std::tie(newPose, distance, std::ignore) = GeometryCV::fit2d3d(edgePoints3d, hyp.pose, innerEdges, this->camera, this->icp2d3dIterationsLimit, plane_normal);
          if (std::isnormal(distance)) {
            // find new score
            ::MeshFootprint tmpFootprint(mesh, hyp.pose, this->camera, this->footprintImageSize, false);
            std::tie(distance, std::ignore) = GeometryCV::getChamferDistance(tmpFootprint.outerEdge, contour, casImageDepth.size(), distanceTransformCache);
            hyp.setScore(distanceToScore(distance));
            hyp.pose = newPose;
          }
        }
      }
      else {
        if (this->applySupportPlaneAssumption) {
          hyp.pose = alignObjectsPoseWithPlane(hyp.pose, cv::Vec3f(0, 0, 0), plane_normal, distanceToPlaneSigned, this->camera, jacobian);

          // find new score
          ::MeshFootprint tmpFootprint(mesh, hyp.pose, this->camera, this->footprintImageSize, false);
          std::tie(distance, std::ignore) = GeometryCV::getChamferDistance(tmpFootprint.outerEdge, contour, casImageDepth.size(), distanceTransformCache);

          hyp.setScore(distanceToScore(distance / contourStdDev));
        }
      }
    }

    outInfo("Has " << poseRanking.size() << " hypotheses for the segment");
    auto topHypotheses = poseRanking.getTop(1);
    if (this->repairPointCloud && (topHypotheses.size() > 0)) {
      const auto mesh = this->edgeModels[topHypotheses[0].getClass()].mesh;
      auto projectedPoints = GeometryCV::projectPoints(mesh.points, topHypotheses[0].pose, this->camera);
      this->fittedSilhouettes.push_back(projectedPoints);

      this->segments.push_back(segmentationSegment);
      this->labels.push_back(topHypotheses[0].getClass());

      this->drawHypothesisToCAS(tcas, casImageDepth, viewCloud, topHypotheses[0], this->camera);
    }

    // cv::imwrite("/tmp/color.png", casImageRGB);
    // cv::imwrite("/tmp/depth.png", casImageDepth);
    // break;
  }

  outInfo("took: " << clock.getTime() << " ms.");
  return UIMA_ERR_NONE;
}

//////////////////////////////////////////////////
void ContourFittingClassifier::drawImageWithLock(cv::Mat &disp) {
  cv::Mat gray;
  cv::cvtColor(this->imageRGB, gray, CV_BGR2GRAY);

  for (auto segment : this->segments) {
    auto roi = segment.rect;

    constexpr int margin = 5;
    roi.x -= margin;
    roi.y -= margin;
    roi.width += 2*margin;
    roi.height += 2*margin;

    cv::Mat grayROI = gray(roi);

    // Compute optimal thresh value
    cv::Mat notUsed;
    double otsuThreshold = cv::threshold(grayROI, notUsed, 0, 255,
                                         CV_THRESH_BINARY | CV_THRESH_OTSU);
    cv::Canny(grayROI, grayROI, otsuThreshold/3, otsuThreshold);
  }

  cv::cvtColor(gray, disp, CV_GRAY2BGR);

  ImageSegmentation::drawSegments2D(disp, this->segments, this->labels, 1, 0.5);

  for (const auto &sil : this->fittedSilhouettes) {
    for (const auto &pt : sil)
      cv::circle(disp, pt, 1, cv::Scalar(0, 0, 255), -1);
  }

  cv::Mat transpR = cv::Mat::zeros(disp.size(), CV_8UC3);
  cv::Mat transpB = cv::Mat::zeros(disp.size(), CV_8UC3);
  for (const auto &points : this->debugPointsRed) {
    for (auto &pt : points)
      cv::circle(transpR, pt, 1, cv::Scalar(0, 0, 255), -1);
  }

  for (const auto &points : this->debugPointsBlue) {
    for (auto &pt : points)
      cv::circle(transpB, pt, 1, cv::Scalar(255, 0, 0), -1);
  }


  for (size_t i = 0; i < this->segments.size(); ++i) {
    auto segment = segments[i];
    cv::Point histogramTopLeft(segment.center.x,
                               segment.center.y + 5 + segment.rect.height/2);
    cv::Rect histogramDestinationRect(histogramTopLeft, histograms[i].size());
    histogramDestinationRect =
        histogramDestinationRect & cv::Rect(cv::Point(), disp.size());
    disp(histogramDestinationRect) *= 0.5;
    cv::Rect histogramSourceRect =
        (histogramDestinationRect - histogramDestinationRect.tl())
        & cv::Rect(cv::Point(), histograms[i].size());
    disp(histogramDestinationRect) += histograms[i](histogramSourceRect);
  }

  cv::Mat depthMap;
  this->depthMap.convertTo(depthMap, CV_8UC1, 255/1500.0, 0);
  cv::cvtColor(depthMap, depthMap, CV_GRAY2BGR);
  cv::resize(depthMap, depthMap, cv::Size(), 0.25, 0.25);

  cv::Rect depthROI(disp.cols - depthMap.cols, disp.rows - depthMap.rows,
      depthMap.cols, depthMap.rows);
  depthMap.copyTo(disp(depthROI));

  disp += transpR + transpB;
}

//////////////////////////////////////////////////
void ContourFittingClassifier::fillVisualizerWithLock(
    pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
{
  const std::string &cloudname = "ContourFittingClassifier";
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outCloud;

  outCloud = this->viewCloud;

  if (firstRun)
    visualizer.addPointCloud(outCloud, cloudname);
  else {
    visualizer.updatePointCloud(outCloud, cloudname);
    visualizer.removeAllShapes();
  }

  if (this->visualizeSolidMeshes) {
    int i = 0;
    for (const auto &segmentHypothesis : this->poseHypotheses) {
      for (const auto &hyp : segmentHypothesis) {
        std::string pcl_mesh_name = "_" + std::to_string(i);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
        std::vector<pcl::Vertices> polygons;

        cv::Mat affine3DTransform = GeometryCV::poseRTToAffine(hyp.pose);
        std::tie(cloud, polygons) =
            meshToPCLMesh(this->edgeModels[hyp.getClass()].mesh,
                          affine3DTransform, (hyp.getScore() - 0.85) / 0.15);

        visualizer.removeShape(pcl_mesh_name);

        auto result = visualizer.addPolygonMesh<pcl::PointXYZRGBA>(
            cloud, polygons, pcl_mesh_name);
        assert(result);

        ++i;
      }
    }
  }
}

//////////////////////////////////////////////////
void ContourFittingClassifier::drawHypothesisToCAS(
  CAS &tcas, cv::Mat &casImageDepth,
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr casViewCloud,
  const ::PoseHypothesis &hypothesis, const ::Camera &camera) {
  rs::SceneCas cas(tcas);

  auto &mesh = this->edgeModels[hypothesis.getClass()].mesh;

  cv::Mat depthMap = cv::Mat::ones(casImageDepth.size(),
                                   CV_32FC1) * Drawing::max_depth_32f;
  Drawing::drawMeshDepth(depthMap, mesh.points, mesh.triangles,
      hypothesis.pose.rot, hypothesis.pose.trans,
      camera.matrix, camera.distortion);

  cv::Mat depthU16;
  depthMap.convertTo(depthU16, CV_16UC1, 1000);
  depthU16.copyTo(casImageDepth, (depthMap != Drawing::max_depth_32f));

  checkViewCloudLookup(camera, depthMap.size(), this->lookupX, this->lookupY);

  int counter = 0;
  for (int i = 0; i < casViewCloud->width; ++i) {
    for (int j = 0; j < casViewCloud->height; ++j) {
      int id = j*casViewCloud->width + i;
      if (depthMap.at<float>(j, i) != Drawing::max_depth_32f) {
        float depth_value = depthMap.at<float>(j, i);

        casViewCloud->points[id].x = depth_value * this->lookupX.at<float>(i);
        casViewCloud->points[id].y = depth_value * this->lookupY.at<float>(j);
        casViewCloud->points[id].z = depth_value;

        auto rgb = this->imageRGB.at<cv::Vec<uint8_t, 3>>(j, i);

        casViewCloud->points[id].r = rgb(0);
        casViewCloud->points[id].g = rgb(1);
        casViewCloud->points[id].b = rgb(2);
        casViewCloud->points[id].a = 255;

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

    sstr << "rotationAngleSamples:" << this->rotationAngleSamples << ";"
     << "rotationAxisSamples:" << this->rotationAxisSamples << ";"
     << "footprintImageSize:" << this->footprintImageSize << ";"
     << "rejectScoreLevel:" << this->rejectScoreLevel << ";"
     << "normalizedAcceptScoreLevel:" << this->normalizedAcceptScoreLevel << ";"
     << "performICPPoseRefinement:" << this->performICPPoseRefinement << ";"
     << "icp2d3dIterationsLimit:" << this->icp2d3dIterationsLimit << ";"
     << "applySupportPlaneAssumption:" << this->applySupportPlaneAssumption
     << ";";
    rsClassification.parameters.set(sstr.str());

    sstr.clear();
    sstr.str(std::string());
    for (const auto &em : this->edgeModels)
      sstr << em.first << ";";
    rsClassification.model.set(sstr.str());

    rs::ClassConfidence rsClassConfidence =
        rs::create<rs::ClassConfidence>(tcas);
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
        ros::Time(/*pose.timestamp.get()*/0 / 1000000000,
                  /*pose.timestamp.get()*/0 % 1000000000),
        frameId);

    rs::PoseAnnotation rsPoseAnnotation = rs::create<rs::PoseAnnotation>(tcas);
    rsPoseAnnotation.camera.set(rs::conversion::to(tcas, tfStampedPose));

    // FIXME: annotate cluster
    scene.identifiables.append(rsPoseAnnotation);
  }

  cas.set(VIEW_DEPTH_IMAGE, casImageDepth);
  cas.set(VIEW_CLOUD, *casViewCloud);

  casImageDepth.copyTo(this->depthMap);
}

//////////////////////////////////////////////////
::PoseRT ContourFittingClassifier::alignObjectsPoseWithPlane(
    const ::PoseRT &initialPose, const cv::Vec3f meshAnchorPoint,
    const cv::Vec3f supportPlaneNormal, const float supportPlaneDistance,
    const ::Camera &camera, const cv::Mat &jacobian) {
  // find anchor point
  auto objectsAnchorPointCamspace =
      GeometryCV::transform(initialPose, meshAnchorPoint);
  auto viewRay = objectsAnchorPointCamspace /
                    cv::norm(objectsAnchorPointCamspace);

  // find anchor point offset along view ray
  auto lambda = supportPlaneDistance / supportPlaneNormal.ddot(viewRay);
  auto anchorOffset = lambda*viewRay - objectsAnchorPointCamspace;

  // find axes rotation transformation to align object's up to plane normal
  cv::Vec3f objectsUpLocal(0, 1, 0);
  ::PoseRT objectToCameraRotation {initialPose.rot, cv::Vec3f(0, 0, 0)};
  auto objectsUpCamspace =
      GeometryCV::transform(objectToCameraRotation, objectsUpLocal);

  double phi = std::acos(supportPlaneNormal.ddot(objectsUpCamspace));
  // phi = std::min(phi, M_PI - phi);

  cv::Vec3f upToNRotationAxis = objectsUpCamspace.cross(supportPlaneNormal);
  cv::Vec3f rodriguesUpToN =
      upToNRotationAxis * (phi / cv::norm(upToNRotationAxis));

  ::PoseRT alignedToPlaneObjectsPose;

#define LOCAL_AMBIGUITY_RESOLUTION 0
#if LOCAL_AMBIGUITY_RESOLUTION
  if (jacobian.empty()) {
    outError("Jacobian is empty, returning original pose");
    return initialPose;
  }

  using namespace GeometryCV;

  cv::Mat E = (cv::Mat_<double>(4, 6) <<
    0, 0, 0, supportPlaneNormal(0), supportPlaneNormal(1), supportPlaneNormal(2),
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0);

  cv::Mat Et = E.t();

  cv::Mat d = (cv::Mat_<double>(4, 1) <<
    cv::norm(anchorOffset),
    rodriguesUpToN(0),
    rodriguesUpToN(1),
    rodriguesUpToN(2));

  cv::Mat jacobianDouble;
  jacobian.convertTo(jacobianDouble, CV_64FC1);

  cv::Mat Q = jacobianDouble.t() * jacobianDouble;

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

  alignedToPlaneObjectsPose = initialPose + align_to_plane_objects_pose;
#else
  cv::Mat upToNRotation;
  cv::Rodrigues(rodriguesUpToN, upToNRotation);

  alignedToPlaneObjectsPose.trans = initialPose.trans + anchorOffset;

  cv::Mat initialRotation;
  cv::Rodrigues(initialPose.rot, initialRotation);
  cv::Mat finalRotation = upToNRotation * initialRotation;

  cv::Rodrigues(finalRotation, alignedToPlaneObjectsPose.rot);
#endif

  return alignedToPlaneObjectsPose;
}

//////////////////////////////////////////////////
std::vector<cv::Point3f> ContourFittingClassifier::getMeshSurfaceEdgesAtPose(
    const ::Mesh &mesh, const ::PoseRT &pose,
    const ::Camera &camera, const cv::Size imageSize) {
  cv::Mat zBuffer = cv::Mat::ones(imageSize, CV_32FC1) * Drawing::max_depth_32f;
  cv::Mat normalMap = cv::Mat::zeros(imageSize, CV_32FC3);

  Drawing::drawMeshNormals(zBuffer, normalMap,
      mesh.points, mesh.normals, mesh.triangles,
      pose.rot, pose.trans, camera.matrix, camera.distortion);

  cv::Vec3f cameraForwardVec(0, 0, -1);
  cv::Mat dotProductMap = cv::Mat::zeros(normalMap.size(), CV_32FC1);

  for (int i = 0; i < imageSize.height; ++i)
    for (int j = 0; j < imageSize.width; ++j) {
      float val = cameraForwardVec.dot(normalMap.at<cv::Vec3f>(i, j));
      dotProductMap.at<float>(i, j) = val;
    }

  cv::normalize(dotProductMap, dotProductMap, 0, 1, cv::NORM_MINMAX);
  dotProductMap.convertTo(dotProductMap, CV_8UC1, 255);
  cv::Canny(dotProductMap, dotProductMap, 60, 128);

  std::vector<cv::Point3f> points3d;

  checkViewCloudLookup(camera, imageSize, this->lookupX, this->lookupY);

  for (int i = 0; i < imageSize.height; ++i)
    for (int j = 0; j < imageSize.width; ++j) {
      if ((dotProductMap.at<uint8_t>(i, j) == 255)
           && (zBuffer.at<float>(i, j) != Drawing::max_depth_32f)) {
        cv::Point3f pt3;

        pt3.z = -zBuffer.at<float>(i, j);

        pt3.x = this->lookupX.at<float>(j) * pt3.z;
        pt3.y = this->lookupY.at<float>(i) * pt3.z;

        points3d.push_back(pt3);
      }
    }

  for (auto &point : points3d) {
    point.x -= pose.trans(0);
    point.y -= pose.trans(1);
    point.z -= pose.trans(2);
  }

  PoseRT toOriginRotation{-pose.rot, {0., 0., 0.}};
  cv::Mat makePoseDefault = GeometryCV::poseRTToAffine(toOriginRotation);
  auto result = GeometryCV::transform(makePoseDefault, points3d);

  return result;
}

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ContourFittingClassifier)
