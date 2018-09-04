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

// UIMA
#include <uima/api.hpp>

#include <opencv2/opencv.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// RS
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/utils/common.h>
#include <rs/utils/exception.h>

//ROS
#include <ros/package.h>

#define DEBUG_OUTPUT 0

using namespace uima;

class PlaneAnnotator : public DrawingAnnotator
{
private:
  enum
  {
    PCL,
    BOARD,
    MPS,
    FILE
  } mode;

  // BOARD
  cv::Mat cameraMatrix;
  cv::Mat distortionCoefficients;
  cv::Mat rotation;
  cv::Mat translation;
  cv::Mat planeNormal;
  double planeDistance;

  // PCL
  pcl::PointIndices::Ptr plane_inliers;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr display;
  std::vector<int> mapping_indices;

  bool useNonNANCloud;

  // MPS
  std::vector<pcl::PlanarRegion<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBA>>> regions;
  std::vector<pcl::ModelCoefficients> modelCoefficients;
  std::vector<pcl::PointIndices> inlierIndices;

  //Drawing
  bool foundPlane, saveToFile;
  cv::Mat image_;
  double pointSize;

  //params
  int min_plane_inliers,
      max_iterations;
  float distance_threshold,
        max_curvature,
        angular_threshold_deg;

  std::string pathToModelFile;

public:
  PlaneAnnotator() : DrawingAnnotator(__func__), mode(BOARD), display(new pcl::PointCloud<pcl::PointXYZRGBA>()),
    saveToFile(false), pointSize(1)
  {
    pathToModelFile = ros::package::getPath("robosherlock") + "/config/plane_model.xml";
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    if(ctx.isParameterDefined("plane_estimation_mode"))
    {
      std::string sMode;
      ctx.extractValue("plane_estimation_mode", sMode);
      outInfo("mode set to: " << sMode);
      if(sMode == "BOARD")
      {
        mode = BOARD;
      }
      else if(sMode == "PCL")
      {
        mode = PCL;
      }
      else if(sMode == "MPS")
      {
        mode = MPS;
      }
      else if(sMode == "FILE")
      {
        mode = FILE;
      }
    }
    if(ctx.isParameterDefined("use_non_nan_cloud"))
    {
      ctx.extractValue("use_non_nan_cloud", useNonNANCloud);
    }
    if(ctx.isParameterDefined("min_plane_inliers"))
    {
      ctx.extractValue("min_plane_inliers", min_plane_inliers);
    }
    if(ctx.isParameterDefined("max_iterations"))
    {
      ctx.extractValue("max_iterations", max_iterations);
    }
    if(ctx.isParameterDefined("distance_threshold"))
    {
      ctx.extractValue("distance_threshold", distance_threshold);
    }
    if(ctx.isParameterDefined("max_curvature"))
    {
      ctx.extractValue("max_curvature", max_curvature);
    }
    if(ctx.isParameterDefined("angular_threshold_deg"))
    {
      ctx.extractValue("angular_threshold_deg", angular_threshold_deg);
    }
    if(ctx.isParameterDefined("save_to_file"))
    {
      ctx.extractValue("save_to_file", saveToFile);
    }



    return UIMA_ERR_NONE;
  }
  
    TyErrorId reconfigure()
  {
    outInfo("Reconfiguring");
    AnnotatorContext &ctx = getAnnotatorContext();
    initialize(ctx);
    if(ctx.isParameterDefined("plane_estimation_mode"))
    {
      std::string sMode;
      ctx.extractValue("plane_estimation_mode", sMode);
      outInfo("mode set to: " << sMode);
    }
    return UIMA_ERR_NONE;
  }


  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

private:
  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");

    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    cas.get(VIEW_COLOR_IMAGE, image_);

    foundPlane = false;

    switch(mode)
    {
    case BOARD:
      outInfo("Estimating form board");
      estimateFromBoard(tcas, scene);
      break;
    case PCL:
      outInfo("Estimating form Point Cloud");
      estimateFromPCL(tcas, scene);
      break;
    case MPS:
      outInfo("Estimating form MPS");
      estimateFromMPS(tcas, scene);
      break;
    case FILE:
      outInfo("Loading from File");
      loadPlaneModel(tcas, scene);
      break;
    }

    if(!foundPlane)
    {
      outWarn("no plane found, no further processing!");
      throw rs::FrameFilterException();
    }

    return UIMA_ERR_NONE;
  }

  void estimateFromBoard(CAS &tcas, rs::Scene &scene)
  {
    rs::SceneCas cas(tcas);

    sensor_msgs::CameraInfo camInfo;
    cas.get(VIEW_CAMERA_INFO, camInfo);
    readCameraInfo(camInfo);

    std::vector<rs::Board> boards;
    scene.annotations.filter(boards);

    if(boards.empty())
    {
      foundPlane = false;
      outInfo("no board found!");
      return;
    }

    foundPlane = true;
    rs::Board &board = boards[0];
    cv::Mat pointsWorld, pointsImage, rotW2C;
    rs::conversion::from(board.pointsImage(), pointsImage);
    rs::conversion::from(board.pointsWorld(), pointsWorld);

    cv::solvePnPRansac(pointsWorld, pointsImage, cameraMatrix, distortionCoefficients, rotation, translation, !rotation.empty() && !translation.empty(), 100, 1.0);
    cv::Rodrigues(rotation, rotW2C);

    planeNormal = cv::Mat(3, 1, CV_64F);
    planeNormal.at<double>(0) = 0;
    planeNormal.at<double>(1) = 0;
    planeNormal.at<double>(2) = 1;
    planeNormal = rotW2C * planeNormal;
    planeDistance = planeNormal.dot(translation);

    std::vector<float> planeModel(4);
    planeModel[0] = -planeNormal.at<double>(0);
    planeModel[1] = -planeNormal.at<double>(1);
    planeModel[2] = -planeNormal.at<double>(2);
    planeModel[3] = -planeDistance;

    rs::Plane plane = rs::create<rs::Plane>(tcas);
    plane.model(planeModel);
    plane.source("CheckerBoard");
    //TODO: add empty?
    //    plane.inliers(plane_inliers->indices);
    //    plane.roi(rs::conversion::to(tcas, roi));
    //    plane.mask(rs::conversion::to(tcas, mask));
    scene.annotations.append(plane);
  }

  void getMask(const pcl::PointIndices &inliers, const cv::Size &size, cv::Mat &mask, cv::Rect &roi)
  {
    cv::Mat tmp = cv::Mat::zeros(size.height, size.width, CV_8U);
    int minX = size.width, maxX = 0;
    int minY = size.height, maxY = 0;

    //#pragma omp parallel for
    for(size_t i = 0; i < inliers.indices.size(); ++i)
    {
      const int index = inliers.indices[i];
      const int x = index % size.width;
      const int y = index / size.width;
      tmp.at<uint8_t>(y, x) = 255;

      minX = std::min(minX, x);
      maxX = std::max(maxX, x);
      minY = std::min(minY, y);
      maxY = std::max(maxY, y);
    }

    roi = cv::Rect(minX, minY, (maxX - minX) + 1, (maxY - minY) + 1);
    tmp(roi).copyTo(mask);
  }

  void estimateFromMPS(CAS &tcas, rs::Scene &scene)
  {
    outInfo("estimating plane form Point Cloud data");
    rs::SceneCas cas(tcas);

    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    plane_inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> mps;
    pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>());
    std::vector<pcl::PointIndices> labelIndices;
    std::vector<pcl::PointIndices> boundaryIndices;

    regions.clear();
    modelCoefficients.clear();
    inlierIndices.clear();

    cas.get(VIEW_CLOUD, *cloud);
    cas.get(VIEW_NORMALS, *normals);

    mps.setMinInliers(min_plane_inliers);
    mps.setMaximumCurvature(max_curvature);
    mps.setAngularThreshold(angular_threshold_deg * M_PI / 180);
    mps.setDistanceThreshold(distance_threshold);
    mps.setProjectPoints(false);
    mps.setInputNormals(normals);
    mps.setInputCloud(cloud);

    mps.segmentAndRefine(regions, modelCoefficients, inlierIndices, labels, labelIndices, boundaryIndices);

    size_t biggest = 0;
    std::vector<float> biggest_planeModel(4);

    for(size_t i = 0; i < regions.size(); ++i)
    {
      const pcl::PlanarRegion<pcl::PointXYZRGBA> &region = regions[i];
      const Eigen::Vector4f model = region.getCoefficients();
      std::vector<float> planeModel(4);

      if(model[3] < 0)
      {
        planeModel[0] = model[0];
        planeModel[1] = model[1];
        planeModel[2] = model[2];
        planeModel[3] = model[3];
      }
      else
      {
        planeModel[0] = -model[0];
        planeModel[1] = -model[1];
        planeModel[2] = -model[2];
        planeModel[3] = -model[3];
      }

      cv::Mat mask;
      cv::Rect roi;
      getMask(inlierIndices[i], cv::Size(cloud->width, cloud->height), mask, roi);

      rs::Plane plane = rs::create<rs::Plane>(tcas);
      plane.model(planeModel);
      plane.inliers(inlierIndices[i].indices);
      plane.roi(rs::conversion::to(tcas, roi));
      plane.mask(rs::conversion::to(tcas, mask));
      plane.source("MPS");
      scene.annotations.append(plane);

      if(region.getCount() > regions[biggest].getCount())
      {
        biggest = i;
        biggest_planeModel = planeModel;
      }

      plane_inliers->indices.insert(plane_inliers->indices.end(), inlierIndices[i].indices.begin(), inlierIndices[i].indices.end());
    }

    if(!regions.empty())
    {
      foundPlane = true;
    }
    else
    {
      outInfo("No plane found in the cloud");
    }
  }

  void estimateFromPCL(CAS &tcas, rs::Scene &scene)
  {
    outInfo("Estimating plane form Point Cloud data");
    rs::SceneCas cas(tcas);

    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);

    if(useNonNANCloud)
    {
      rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
      cas.get(VIEW_CLOUD_NON_NAN, rcp);
      rs::conversion::from(rcp.cloud(), *cloud);
    }
    else
    {
      cas.get(VIEW_CLOUD, *cloud);
    }

    if(cloud->size() == 0)
    {
      outError("No PointCloud present;");
    }

    std::vector<float> planeModel(4);
    if(process_cloud(plane_coefficients))
    {
      foundPlane = true;

      if(plane_coefficients->values[3] < 0)
      {
        planeModel[0] = plane_coefficients->values[0];
        planeModel[1] = plane_coefficients->values[1];
        planeModel[2] = plane_coefficients->values[2];
        planeModel[3] = plane_coefficients->values[3];
      }
      else
      {
        planeModel[0] = -plane_coefficients->values[0];
        planeModel[1] = -plane_coefficients->values[1];
        planeModel[2] = -plane_coefficients->values[2];
        planeModel[3] = -plane_coefficients->values[3];
      }

      if(saveToFile)
      {
        outInfo("Saving Plane to file: " << pathToModelFile);
        cv::Mat coeffs = cv::Mat_<float>(4, 1);
        for(size_t i = 0; i < planeModel.size(); ++i)
        {
          coeffs.at<float>(i) = planeModel[i];
        }
        cv::FileStorage fs;
        fs.open(pathToModelFile, cv::FileStorage::WRITE);
        fs << "PlaneModel" << coeffs;
        fs.release();
      }

      pcl::PointIndices::Ptr temp(new pcl::PointIndices());
      temp->indices.resize(plane_inliers->indices.size());
      for(size_t i = 0; i < plane_inliers->indices.size(); ++i)
      {
        temp->indices[i] = mapping_indices[plane_inliers->indices[i]];
      }
      plane_inliers.swap(temp);

      cv::Mat mask;
      cv::Rect roi;
      getMask(*plane_inliers, cv::Size(cloud->width, cloud->height), mask, roi);

      rs::Plane plane = rs::create<rs::Plane>(tcas);
      plane.model(planeModel);
      plane.inliers(plane_inliers->indices);
      plane.roi(rs::conversion::to(tcas, roi));
      plane.mask(rs::conversion::to(tcas, mask));
      plane.source("RANSAC");
      scene.annotations.append(plane);
    }
    else
    {
      outInfo("No plane found in the cloud");
    }
  }

  void loadPlaneModel(CAS &tcas, rs::Scene &scene)
  {
    outInfo("loading plane from model file");
    rs::SceneCas cas(tcas);
    plane_inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud <pcl::PointXYZRGBA>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cas.get(VIEW_CLOUD, *cloud);

    cv::Mat planeCoeffs;
    cv::FileStorage fs;

    if(fs.open(pathToModelFile, cv::FileStorage::READ))
    {
      outInfo("plane model file found");
    }
    else
    {
      outWarn("Could not load plane model. Are you sure you want to load a plane from model file?");
      outWarn("Have You saved a plane_model file in the config folder?");
      exit(1);
    }
    fs["PlaneModel"] >> planeCoeffs;
    std::vector<float> planeModel(4);
    foundPlane = true;

    planeModel[0] = planeCoeffs.at<float>(0);
    planeModel[1] = planeCoeffs.at<float>(1);
    planeModel[2] = planeCoeffs.at<float>(2);
    planeModel[3] = planeCoeffs.at<float>(3);

    //TODO find inliers for plane;
    std::vector<int> mappingIndices;
    pcl::removeNaNFromPointCloud(*cloud, *cloudFiltered, mappingIndices);

    for(size_t i = 0; i < cloudFiltered->points.size(); ++i)
    {
      float num =
        fabs(planeModel[0] * cloudFiltered->points[i].x +
             planeModel[1] * cloudFiltered->points[i].y +
             planeModel[2] * cloudFiltered->points[i].z +
             planeModel[3]);
      float denum =
        std::sqrt(std::pow(planeModel[0], 2) +
                  std::pow(planeModel[1], 2) +
                  std::pow(planeModel[2], 2));

      float dist = num / denum;
      if(fabs(dist) < 0.015)
      {
        plane_inliers->indices.push_back(mappingIndices[i]);
      }
    }
    outInfo("No. of inliers found: " << plane_inliers->indices.size());

    cv::Mat mask;
    cv::Rect roi;
    getMask(*plane_inliers, cv::Size(cloud->width, cloud->height), mask, roi);
    rs::Plane plane = rs::create<rs::Plane>(tcas);
    plane.model(planeModel);
    plane.inliers(plane_inliers->indices);
    plane.roi(rs::conversion::to(tcas, roi));
    plane.mask(rs::conversion::to(tcas, mask));
    plane.source("offline estimation");
    scene.annotations.append(plane);
  }

  bool process_cloud(pcl::ModelCoefficients::Ptr &plane_coefficients)
  {
    plane_inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(cloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_no_nan(new pcl::PointCloud<pcl::PointXYZRGBA>);

    mapping_indices.clear();

    pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered_no_nan, mapping_indices);

    // find the major plane
    pcl::SACSegmentation<pcl::PointXYZRGBA> plane_segmentation;

    plane_segmentation.setOptimizeCoefficients(true);
    plane_segmentation.setModelType(pcl::SACMODEL_PLANE);
    plane_segmentation.setMethodType(pcl::SAC_RANSAC);
    plane_segmentation.setDistanceThreshold(distance_threshold);
    plane_segmentation.setMaxIterations(max_iterations);
    plane_segmentation.setInputCloud(cloud_filtered_no_nan);
    plane_segmentation.segment(*plane_inliers, *plane_coefficients);

    if(plane_inliers->indices.size() < min_plane_inliers)
    {
      outWarn("not enough inliers!");
      return false;
    }

    std::sort(plane_inliers->indices.begin(), plane_inliers->indices.end());
    if(plane_inliers->indices.size() == 0)
    {
      return false;
    }
    outDebug("Number of inliers in plane:" << plane_inliers->indices.size());

#if DEBUG_OUTPUT
    pcl::PCDWriter writer;
    outInfo("Size of input cloud: " << cloud->points.size());
    outInfo("Filtered cloud size: " << cloud_filtered_no_nan->points.size());
    outInfo("Downsampled cloud size: " << cloud_downsampled->points.size());
    if(cloud_filtered_no_nan->points.size() > 0)
    {
      writer.writeASCII("original.pcd", *cloud_filtered_no_nan);
    }
    if(plane_inliers->indices.size() > 0)
    {
      writer.writeASCII("plane.pcd", *cloud_filtered_no_nan, plane_inliers->indices);
    }

#endif
    return true;
  }

  void readCameraInfo(const sensor_msgs::CameraInfo &camInfo)
  {
    cameraMatrix = cv::Mat(3, 3, CV_64F);
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

    distortionCoefficients = cv::Mat(1, camInfo.D.size(), CV_64F);
    it = distortionCoefficients.ptr<double>(0);
    for(size_t i = 0; i < camInfo.D.size(); ++i, ++it)
    {
      *it = camInfo.D[i];
    }
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    if(!foundPlane)
    {
      disp = cv::Mat::zeros(image_.rows, image_.cols, CV_8UC3);
      return;
    }
    else if(foundPlane && image_.size().area() != cloud->size())
    {
      disp = cv::Mat::zeros(480, 640, CV_8UC3);
      return;
    }
    std::vector<cv::Point2f> pointsImage;
    std::vector<cv::Point3f> axis(4);

    switch(mode)
    {
    case BOARD:
      disp = image_.clone();

      axis[0] = cv::Point3f(0, 0, 0);
      axis[1] = cv::Point3f(0.02, 0, 0);
      axis[2] = cv::Point3f(0, 0.02, 0);
      axis[3] = cv::Point3f(0, 0, 0.02);

      cv::projectPoints(axis, rotation, translation, cameraMatrix, distortionCoefficients, pointsImage);

      //draw the axes on the colored image
      cv::line(disp, pointsImage[0], pointsImage[1], CV_RGB(255, 0, 0), 2, CV_AA);
      cv::line(disp, pointsImage[0], pointsImage[2], CV_RGB(0, 255, 0), 2, CV_AA);
      cv::line(disp, pointsImage[0], pointsImage[3], CV_RGB(0, 0, 255), 2, CV_AA);
      break;
    case PCL:
    case FILE:
    case MPS:
      disp = cv::Mat::zeros(cloud->height, cloud->width, CV_8UC3);
      #pragma omp parallel for
      for(size_t i = 0; i < plane_inliers->indices.size(); ++i)
      {
        const size_t index = plane_inliers->indices[i];
        const size_t r = index / disp.cols;
        const size_t c = index % disp.cols;
        const pcl::PointXYZRGBA &point = cloud->at(index);
        disp.at<cv::Vec3b>(r, c) = cv::Vec3b(point.b, point.g, point.r);
      }
      break;
    }
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = this->name;
    pcl::ExtractIndices<pcl::PointXYZRGBA> ei;
    //    uint32_t colors[6] = {0xFFFF0000, 0xFF00FF00, 0xFF0000FF, 0xFFFFFF00, 0xFFFF00FF, 0xFF00FFFF};
    const pcl::PointCloud<pcl::PointXYZRGBA>::VectorType &origPoints = this->cloud->points;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output;
    switch(mode)
    {
    case BOARD:
      output = cloud;
      break;
    case FILE:
    case PCL:
      output.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
      ei.setInputCloud(cloud);
      ei.setIndices(plane_inliers);
      //      ei.setKeepOrganized(true);
      ei.filter(*output);
      break;
    case MPS:
      output.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
      for(size_t i = 0; i < inlierIndices.size(); ++i)
      {
        const pcl::PointIndices &indices = this->inlierIndices[i];
        const size_t outIndex = output->points.size();
        output->points.resize(outIndex + indices.indices.size());
        uint32_t rgba = rs::common::colors[i % rs::common::numberOfColors];

        #pragma omp parallel for
        for(size_t j = 0; j < indices.indices.size(); ++j)
        {
          const size_t index = indices.indices[j];
          output->points[outIndex + j] = origPoints[index];
          output->points[outIndex + j].rgba = rgba;
        }
      }
      output->width = output->points.size();
      output->height = 1;
      output->is_dense = 1;
      break;
    }

    if(firstRun)
    {
      visualizer.addPointCloud(output, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(output, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }

};

MAKE_AE(PlaneAnnotator)
