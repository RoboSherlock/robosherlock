/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *            Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
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

//UIMA
#include <uima/api.hpp>

//RS
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/utils/exception.h>
#include <rs/utils/common.h>
#include <rs/types/all_types.h>

//STD
#include <iostream>
#include <typeinfo>
#include <stdio.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/vfh.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/features/esf.h>
#include <pcl/features/gfpfh.h>

using namespace uima;

enum GlobalDescriptor
{
  VFH = 0,
  CVFH,
  OURCVFH,
  ESF,
  GFPFH,
  NIL
};

const std::string globalDescriptorNames[] =
{
  "VFH",
  "CVFH",
  "OUR-CVFH",
  "ESF",
  "GFPFH",
  "NIL"
};


class PCLDescriptorExtractor : public DrawingAnnotator
{
private:  
  typedef pcl::PointXYZ PointT;
  typedef pcl::Histogram<135> ROPS135;
  typedef pcl::Histogram<153> SpinImage;
  typedef pcl::Histogram<32> RIFT32;

  const cv::Size diagramSize;
  const uint32_t legendHeight;
  double pointSize;

  GlobalDescriptor descriptorType;

  cv::Mat color;
  std::vector<cv::Rect> clusterRois;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PointCloud<pcl::Normal>::Ptr normals;

  std::vector<pcl::PointCloud<PointT>::Ptr> extractedClusters;
  std::vector<pcl::PointCloud<pcl::Normal>::Ptr> extractedNormals;

  std::vector<pcl::ESFSignature640> descriptorsESF;
  std::vector<pcl::VFHSignature308> descriptorsVFH;
  std::vector<pcl::GFPFHSignature16> descriptorsGFPFH;

public:
  PCLDescriptorExtractor() : DrawingAnnotator(__func__), diagramSize(1600, 600), legendHeight(300), pointSize(1),
    cloud(new pcl::PointCloud<pcl::PointXYZRGBA>),
    normals(new pcl::PointCloud<pcl::Normal>)
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    std::string descType;

    if(!ctx.isParameterDefined("descriptorType"))
    {
      outError("Mandatory parameter 'descriptorType' not defined!");
      return UIMA_ERR_CONFIG_NO_VALUE_FOR_MANDATORY_PARAM;
    }

    ctx.extractValue("descriptorType", descType);
    /**
     * Global Descriptors:
     */
    if(descType == "VFH")
    {
      descriptorType = GlobalDescriptor::VFH;
    }
    else if(descType == "CVFH")
    {
      descriptorType = GlobalDescriptor::CVFH;
    }
    else if(descType == "OUR-CVFH")
    {
      descriptorType = GlobalDescriptor::OURCVFH;
    }
    else if(descType == "ESF")
    {
      descriptorType = GlobalDescriptor::ESF;
    }
    else if(descType == "GFPFH")
    {
      descriptorType = GlobalDescriptor::GFPFH;
    }
    return UIMA_ERR_NONE;
  }

  TyErrorId typeSystemInit(TypeSystem const &type_system)
  {
    outInfo("typeSystemInit");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process start");
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    std::vector<rs::Cluster> clusters;

    cas.get(VIEW_CLOUD, *cloud);
    cas.get(VIEW_NORMALS, *normals);
    cas.get(VIEW_COLOR_IMAGE, color);

    scene.identifiables.filter(clusters);
    extractClustersAndNormals(clusters);

    switch(descriptorType)
    {
    case GlobalDescriptor::VFH:
      computeVFH();
      storeDescriptors(descriptorsVFH, clusters, tcas);
      break;
    case GlobalDescriptor::CVFH:
      computeCVFH();
      storeDescriptors(descriptorsVFH, clusters, tcas);
      break;
    case GlobalDescriptor::OURCVFH:
      computeOURCVFH();
      storeDescriptors(descriptorsVFH, clusters, tcas);
      break;
    case GlobalDescriptor::ESF:
      computeESF();
      storeDescriptors(descriptorsESF, clusters, tcas);
      break;
    case GlobalDescriptor::GFPFH:
      computeGFPFH();
      storeDescriptors(descriptorsGFPFH, clusters, tcas);
      break;
    default:
      throw_exception_message("Descriptor not specified!");
      break;
    }

    return UIMA_ERR_NONE;
  }
  void drawImageWithLock(cv::Mat &disp)
  {
    switch(descriptorType)
    {
    case GlobalDescriptor::VFH:
      drawHistograms(disp, descriptorsVFH, "Viewpoint Feature Histogram (VFH)");
      break;
    case GlobalDescriptor::CVFH:
      drawHistograms(disp, descriptorsVFH, "Clustered Viewpoint Feature Histogram (CVFH)");
      break;
    case GlobalDescriptor::OURCVFH:
      drawHistograms(disp, descriptorsVFH, "Oriented, Unique and Repeatable Clustered Viewpoint Feature Histogram (OUR-CVFH)");
      break;
    case GlobalDescriptor::ESF:
      drawHistograms(disp, descriptorsESF, "Ensemble of Shape Functions (ESF)");
      break;
    case GlobalDescriptor::GFPFH:
      drawHistograms(disp, descriptorsGFPFH, "Global Fast Point Feature Histogram (GFPFH)");
      break;
    default:
      break;
    }
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    if(firstRun)
    {
      visualizer.addPointCloud(cloud, "cloudname");
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloudname");
    }
    else
    {
      visualizer.updatePointCloud(cloud, "cloudname");
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloudname");
      visualizer.removeAllShapes();
    }
  }

  /**
  * @brief extractClustersAndNormals-func to extract clusters from input cloud
  * @param inputCloud-a single point cloud
  * @return extractedClusters-vector of clusters extracted
  */
  void extractClustersAndNormals(std::vector<rs::Cluster> &clusters)
  {
    //first, empty the vector of clusters and the vector of normals
    extractedClusters.clear();
    extractedNormals.clear();
    clusterRois.clear();

    //next, iterate over clusters
    for(size_t i = 0; i < clusters.size(); ++i)
    {
      rs::Cluster &cluster = clusters[i];
      if(!cluster.points.has())
      {
        outWarn("skipping cluster without points.");
        continue;
      }

      pcl::PointIndicesPtr indices(new pcl::PointIndices());
      rs::conversion::from(((rs::ReferenceClusterPoints)cluster.points.get()).indices.get(), *indices);
      pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>());
      pcl::PointCloud<pcl::Normal>::Ptr clusterNormals(new pcl::PointCloud<pcl::Normal>);
      pcl::copyPointCloud(*cloud, *indices, *cluster_cloud);
      extractedClusters.push_back(cluster_cloud);

      pcl::ExtractIndices<pcl::Normal> eiNormal;
      eiNormal.setInputCloud(normals);
      eiNormal.setIndices(indices);
      eiNormal.filter(*clusterNormals);
      extractedNormals.push_back(clusterNormals);

      cv::Rect roi;
      rs::conversion::from(cluster.rois().roi(), roi);
      clusterRois.push_back(roi);
    }
  }

  /**
  * @brief computeESF
  * Function that computeulates ESF (Ensemble of Shape Functions)
  * ~~Global Descriptor~~
  */
  void computeESF()
  {
    descriptorsESF.resize(extractedClusters.size());
    #pragma omp parallel for
    for(size_t i = 0; i < extractedClusters.size(); ++i)
    {
      pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor(new pcl::PointCloud<pcl::ESFSignature640>);
      pcl::ESFEstimation<PointT, pcl::ESFSignature640> esf;
      esf.setInputCloud(extractedClusters[i]);
      esf.compute(*descriptor);
      descriptorsESF[i] = descriptor->points[0];
    }
  }

  /**
  * @brief computeVFH
  * Function that computeulates VFH (Viewpoint Feature Histogram)
  * ~~Global Descriptor~~
  */
  void computeVFH()
  {
    descriptorsVFH.resize(extractedClusters.size());
    #pragma omp parallel for
    for(size_t i = 0; i < extractedClusters.size(); ++i)
    {
      pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);
      pcl::VFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> vfh;
      vfh.setInputCloud(extractedClusters[i]);
      vfh.setInputNormals(extractedNormals[i]);
      vfh.setNormalizeBins(true);
      vfh.setNormalizeDistance(true);
      vfh.compute(*descriptor);
      descriptorsVFH[i] = descriptor->points[0];
    }
  }

  /**
  * @brief computeCVFH
  * Function that computeulates CVFH (Clustered Viewpoint Feature Histogram)
  * ~~Global Descriptor~~
  */
  void computeCVFH()
  {
    descriptorsVFH.resize(extractedClusters.size());
    #pragma omp parallel for
    for(size_t i = 0; i < extractedClusters.size(); ++i)
    {
      pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);
      pcl::CVFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> cvfh;
      cvfh.setInputCloud(extractedClusters[i]);
      cvfh.setInputNormals(extractedNormals[i]);
      cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); //5 deg
      cvfh.setCurvatureThreshold(1.0);
      cvfh.setNormalizeBins(true);
      cvfh.compute(*descriptor);
      descriptorsVFH[i] = descriptor->points[0];
    }
  }

  void computeOURCVFH()
  {
    descriptorsVFH.resize(extractedClusters.size());
    #pragma omp parallel for
    for(size_t i = 0; i < extractedClusters.size(); ++i)
    {
      pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);
      pcl::OURCVFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> ourcvfh;
      ourcvfh.setInputCloud(extractedClusters[i]);
      ourcvfh.setInputNormals(extractedNormals[i]);
      //ourcvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); //5 deg
      //ourcvfh.setCurvatureThreshold(1.0);
      //ourcvfh.setAxisRatio(0.8);
      ourcvfh.setNormalizeBins(true);
      ourcvfh.compute(*descriptor);
      descriptorsVFH[i] = descriptor->points[0];
    }
  }

  /**
  * @brief computeGFPFH
  * Function that computeulates GFPFH (Global Fast Point Feature Histogram)
  * ~~Global Descriptor~~
  */
  void computeGFPFH()
  {
    descriptorsGFPFH.resize(extractedClusters.size());
    #pragma omp parallel for
    for(size_t i = 0; i < extractedClusters.size(); ++i)
    {
      pcl::PointCloud<pcl::GFPFHSignature16>::Ptr descriptor(new pcl::PointCloud<pcl::GFPFHSignature16>);
      pcl::PointCloud<pcl::PointXYZL>::Ptr object(new pcl::PointCloud<pcl::PointXYZL>);
      pcl::copyPointCloud(*(extractedClusters[i]), *object);

      for(size_t j = 0; j < object->points.size(); ++j)
      {
        object->points[j].label = 1 + j % 4;
      }
      pcl::GFPFHEstimation<pcl::PointXYZL, pcl::PointXYZL, pcl::GFPFHSignature16> gfpfh;
      gfpfh.setInputCloud(object);
      gfpfh.setInputLabels(object);
      gfpfh.setOctreeLeafSize(0.01);
      gfpfh.setNumberOfClasses(16);
      gfpfh.compute(*descriptor);
      descriptorsGFPFH[i] = descriptor->points[0];
    }
  }

  template<typename DescriptorSignature>
  void storeDescriptors(const std::vector<DescriptorSignature> &descriptors, std::vector<rs::Cluster> &clusters, CAS &tcas)
  {
    for(size_t i = 0; i < clusters.size(); ++i)
    {
      rs::Cluster &cluster = clusters[i];
      const DescriptorSignature &descriptor = descriptors[i];
      const size_t size = sizeof(descriptor.histogram) / sizeof(descriptor.histogram[0]);
      const std::vector<float> histogram(descriptor.histogram, descriptor.histogram + size);

      rs::PclFeature annotation = rs::create<rs::PclFeature>(tcas);
      annotation.feat_type.set(globalDescriptorNames[descriptorType]);
      annotation.feature.set(histogram);
      cluster.annotations.append(annotation);
    }
  }

  /**
  * @brief drawHistograms -OpenCV histogram drawing
  * @param descriptors -descriptor vector containing the data to draw histograms
  * @param title -the string to be displayed on the histogram (descriptor name)
  */
  template<typename DescriptorSignature>
  void drawHistograms(cv::Mat &disp, const std::vector<DescriptorSignature> &descriptors, const std::string &title)
  {
    const cv::Scalar &backgroundColor = CV_RGB(0, 0, 0);
    const cv::Scalar &forgroundColor = CV_RGB(255, 255, 255);

    const uint32_t topBorder = 25;
    const uint32_t sideBorder = 5;

    disp = cv::Mat(diagramSize.height + legendHeight + topBorder, diagramSize.width + 2 * sideBorder, CV_8UC3, backgroundColor);

    //cv::rectangle(disp, cv::Rect(sideBorder - 1, topBorder - 1, diagramSize.width + 2, diagramSize.height + 2), borderColor, 1);
    cv::line(disp, cv::Point(sideBorder, topBorder + diagramSize.height), cv::Point(sideBorder + diagramSize.width, topBorder + diagramSize.height), forgroundColor, 1);
    cv::putText(disp, title, cvPoint(300, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, forgroundColor, 1, CV_AA);

    // drawing the diagram
    float maxValue = 0;
    const size_t histogramSize = sizeof(descriptors[0].histogram) / sizeof(descriptors[0].histogram[0]);

    for(size_t i = 0; i < descriptors.size(); ++i)
    {
      const DescriptorSignature &descriptor = descriptors[i];
      for(size_t j = 1; j < histogramSize; ++j)
      {
        if(descriptor.histogram[j] > maxValue)
        {
          maxValue = descriptor.histogram[j];
        }
      }
    }

    for(size_t i = 0; i < descriptors.size(); ++i)
    {
      const DescriptorSignature &descriptor = descriptors[i];
      std::vector<cv::Point> points(histogramSize);
      const float step = diagramSize.width / (float)(histogramSize - 1);

      for(size_t j = 0; j < histogramSize; ++j)
      {
        cv::Point &p = points[j];
        p.x = sideBorder + step * j;
        p.y = (topBorder + diagramSize.height) - diagramSize.height * (descriptor.histogram[j] / maxValue);
      }
      cv::polylines(disp, points, false, rs::common::cvScalarColors[i % rs::common::numberOfColors], 1, CV_AA);
    }

    // thumbnails for clusters
    const uint32_t clusterBorder = 3;
    const cv::Size thumbnailSize(diagramSize.width / clusterRois.size(), legendHeight);
    const cv::Size maxThumbnailSize(thumbnailSize.width - 2 * clusterBorder, thumbnailSize.height - 2 * clusterBorder);

    for(size_t i = 0, start = sideBorder; i < clusterRois.size(); ++i, start += thumbnailSize.width)
    {
      const cv::Rect &roi = clusterRois[i];
      cv::Mat thumbnail;

      if(roi.width < maxThumbnailSize.width && roi.height < maxThumbnailSize.height)
      {
        thumbnail = color(roi);
      }
      else
      {
        const double factor = std::max(maxThumbnailSize.width / (double)roi.width, maxThumbnailSize.height / (double)roi.height);
        cv::resize(color(roi), thumbnail, cv::Size(), factor, factor, cv::INTER_AREA);
      }

      const cv::Point topLeft(start + (thumbnailSize.width - thumbnail.cols) / 2, diagramSize.height + topBorder + (thumbnailSize.height - thumbnail.rows) / 2);
      const cv::Point topLeftBorder = topLeft - cv::Point(clusterBorder, clusterBorder);

      const cv::Rect border(topLeftBorder, cv::Size(thumbnail.cols + 2 * clusterBorder, thumbnail.rows + 2 * clusterBorder));
      cv::rectangle(disp, border, rs::common::cvScalarColors[i % rs::common::numberOfColors], CV_FILLED);

      cv::Rect roiCpy(topLeft, thumbnail.size());
      thumbnail.copyTo(disp(roiCpy));
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(PCLDescriptorExtractor)
