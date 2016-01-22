//UIMA
#include <uima/api.hpp>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/output.h>
#include <rs/types/all_types.h>

//STD
#include <iostream>
#include <typeinfo>
#include <stdio.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

//global descriptors:

#include <pcl/features/esf.h>
#include <pcl/features/vfh.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/gfpfh.h>

#ifdef PCL_FROM_SOURCE
#include <pcl/features/our_cvfh.h>
#include <pcl/features/grsd.h>
#endif

//local descriptors:
#include<pcl/features/pfh.h>
#include<pcl/features/fpfh.h>
#include<pcl/features/3dsc.h>
#include<pcl/features/usc.h>
#include<pcl/features/shot.h>
#include<pcl/features/spin_image.h>
//RIFT
#include<pcl/features/rift.h>
#include<pcl/point_types_conversion.h>
#include<pcl/features/intensity_gradient.h>

//NARF
#include<pcl/range_image/range_image.h>
//#include<pcl/visualization/range_image_visualizer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::Histogram<135> ROPS135;
typedef pcl::Histogram<153> SpinImage;
typedef pcl::Histogram<32> RIFT32;

using namespace uima;

static const cv::Scalar colors[] =
{
  cv::Scalar(255, 0, 0),
  cv::Scalar(0, 255, 0),
  cv::Scalar(0, 0, 255),
  cv::Scalar(255, 255, 0),
  cv::Scalar(255, 0, 255),
  cv::Scalar(0, 255, 255),
  cv::Scalar(191, 0, 0),
  cv::Scalar(0, 191, 0),
  cv::Scalar(0, 0, 191),
  cv::Scalar(191, 191, 0),
  cv::Scalar(191, 0, 191),
  cv::Scalar(0, 191, 191),
  cv::Scalar(127, 0, 0),
  cv::Scalar(0, 127, 0),
  cv::Scalar(0, 0, 127),
  cv::Scalar(127, 127, 0),
  cv::Scalar(127, 0, 127),
  cv::Scalar(0, 127, 127)
};
static const size_t numberOfColors = sizeof(colors) / sizeof(colors[0]);

enum GlobalDescriptor
{
  VFH = 0,
  CVFH,
  ESF,
  GFPFH,
#ifdef PCL_FROM_SOURCE
  OURCVFH,
  GRSD,
#endif
  NIL
};

const std::string globalDescriptorNames[] =
{
  "VFH",
  "CVFH",
  "ESF",
  "GFPFH",
  #ifdef PCL_FROM_SOURCE
  "OUR-CVFH",
  "GRSD",
  #endif
  "NIL"
};

enum class LocalDescriptor
{
  PFH = 0,
  FPFH,
  _3DSC,
  SHOT,
  SI,
  RIFT,
  NIL
};
//RSD, USC, NARF, ROPS -not func (TO-DO)

class PCLDescriptorExtractor : public DrawingAnnotator
{
private:
  const cv::Size diagramSize;
  const uint32_t legendHeight;

  double pointSize;
  pcl::PointCloud<PointT>::Ptr cloud;
  pcl::PointCloud<pcl::Normal>::Ptr normals;

  std::vector<pcl::PointCloud<PointT>::Ptr> extractedClusters;
  std::vector<pcl::PointCloud<pcl::Normal>::Ptr> extractedNormals;

  std::vector<pcl::ESFSignature640> descVectESF;
  std::vector<pcl::VFHSignature308> descVectVFH;
  std::vector<pcl::GFPFHSignature16> descVectGFPFH;
#ifdef PCL_FROM_SOURCE
  std::vector<pcl::GRSDSignature21> descVectGRSD;
#endif

  pcl::search::KdTree<PointT>::Ptr kdtree;

  GlobalDescriptor descriptorType;
  LocalDescriptor localD;

  cv::Mat color;
  std::vector<cv::Rect> clusterRois;

public:
  PCLDescriptorExtractor() : DrawingAnnotator(__func__), diagramSize(1600, 600), legendHeight(300), pointSize(1),
    cloud(new pcl::PointCloud<pcl::PointXYZRGBA>),
    normals(new pcl::PointCloud<pcl::Normal>)
  {}

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
    if(descType == "ESF")
    {
      descriptorType = GlobalDescriptor::ESF;
      localD = LocalDescriptor::NIL;
    }
    else if(descType == "VFH")
    {
      descriptorType = GlobalDescriptor::VFH;
      localD = LocalDescriptor::NIL;
    }
    else if(descType == "CVFH")
    {
      descriptorType = GlobalDescriptor::CVFH;
      localD = LocalDescriptor::NIL;
    }
#ifdef PCL_FROM_SOURCE
    else if(descType == "OUR-CVFH")
    {
      descriptorType = GlobalDescriptor::OURCVFH;
      localD = LocalDescriptor::NIL;
    }
    else if(descType == "GRSD")
    {
      descriptorType = GlobalDescriptor::GRSD;
      localD = LocalDescriptor::NIL;
    }
#endif
    else if(descType == "GFPFH")
    {
      descriptorType = GlobalDescriptor::GFPFH;
      localD = LocalDescriptor::NIL;
    }

    /**
     * Local Descriptors:
     */
    else if(descType == "PFH")
    {
      descriptorType = GlobalDescriptor::NIL;
      localD = LocalDescriptor::PFH;
    }
    else if(descType == "FPFH")
    {
      descriptorType = GlobalDescriptor::NIL;
      localD = LocalDescriptor::FPFH;
    }
    else if(descType == "3DSC")
    {
      descriptorType = GlobalDescriptor::NIL;
      localD = LocalDescriptor::_3DSC;
    }
    else if(descType == "SHOT")
    {
      descriptorType = GlobalDescriptor::NIL;
      localD = LocalDescriptor::SHOT;
    }
    else if(descType == "SI")
    {
      descriptorType = GlobalDescriptor::NIL;
      localD = LocalDescriptor::SI;
    }
    else if(descType == "RIFT")
    {
      descriptorType = GlobalDescriptor::NIL;
      localD = LocalDescriptor::RIFT;
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

    //retrieve the point cloud to be processed:
    cas.get(VIEW_CLOUD, *cloud);
    cas.get(VIEW_NORMALS, *normals);
    cas.get(VIEW_COLOR_IMAGE, color);

    //2.filter out clusters into array
    scene.identifiables.filter(clusters);
    outInfo("Number of clusters:" << clusters.size());

    //extract clusters into a vector
    extractClustersAndNormals(clusters);

    switch(localD)
    {
    case LocalDescriptor::PFH:
      calcPFH();
      break;
    case LocalDescriptor::FPFH:
      calcFPFH();
      break;
      //      //TO-DO:case RSD
    case LocalDescriptor::_3DSC:
      calc3DSC();
      break;
      //      //TO-DO:case USC
    case LocalDescriptor::SHOT:
      calcSHOT();
      break;
    case LocalDescriptor::SI:
      calcSI();
      break;
    case LocalDescriptor::RIFT:
      calcRIFT();
      break;
      //      //TO-DO:case NARF
      //      //TO-DO:case ROPS
    default:
      break;
    }

    switch(descriptorType)
    {
    case GlobalDescriptor::VFH:
      descVectVFH.clear();
      calcVFH();
      storeGlobalDescrToCas(descVectVFH, clusters, tcas);
      break;
    case GlobalDescriptor::CVFH:
      descVectVFH.clear();
      calcCVFH();
      storeGlobalDescrToCas(descVectVFH, clusters, tcas);
      break;
#ifdef PCL_FROM_SOURCE
    case GlobalDescriptor::OURCVFH:
      descVectVFH.clear();
      calcOURCVFH();
      storeGlobalDescrToCas(descVectVFH, clusters, tcas);
      break;
    case GlobalDescriptor::GRSD:
      descVectGRSD.clear();
      calcGRSD();
      storeGlobalDescrToCas(descVectGRSD, clusters, tcas);
      break;
#endif
    case GlobalDescriptor::ESF:
      descVectESF.clear();
      calcESF();
      storeGlobalDescrToCas(descVectESF, clusters, tcas);
      break;
    case GlobalDescriptor::GFPFH:
      descVectGFPFH.clear();
      calcGFPFH();
      storeGlobalDescrToCas(descVectGFPFH, clusters, tcas);
      break;
      //      //TO-DO:case GRSD
    default:
      break;
    }

    return UIMA_ERR_NONE;
  }
  void drawImageWithLock(cv::Mat &disp)
  {
    // Visualization only for global descriptors
    switch(descriptorType)
    {
    case GlobalDescriptor::VFH:
      drawHistograms(disp, descVectVFH, "Viewpoint Feature Histogram (VFH)");
      break;
    case GlobalDescriptor::CVFH:
      drawHistograms(disp, descVectVFH, "Clustered Viewpoint Feature Histogram (CVFH)");
      break;
#ifdef PCL_FROM_SOURCE
    case GlobalDescriptor::OURCVFH:
      drawHistograms(disp, descVectVFH, "Oriented, Unique and Repeatable Clustered Viewpoint Feature Histogram (OUR-CVFH)");
      break;
    case GlobalDescriptor::GRSD:
      drawHistograms(disp, descVectGRSD, "Radius-based Surface Descriptor (GRSD)");
      break;
#endif
    case GlobalDescriptor::ESF:
      drawHistograms(disp, descVectESF, "Ensemble of Shape Functions (ESF)");
      break;
    case GlobalDescriptor::GFPFH:
      drawHistograms(disp, descVectGFPFH, "Global Fast Point Feature Histogram (GFPFH)");
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
      pcl::ExtractIndices<PointT> ei;
      ei.setInputCloud(cloud);
      ei.setIndices(indices);
      ei.filter(*cluster_cloud);
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
  * @brief calcESF
  * Function that calculates ESF (Ensemble of Shape Functions)
  * ~~Global Descriptor~~
  */
  void calcESF()
  {
    for(size_t i = 0; i < extractedClusters.size(); ++i)
    {
      //Object for storing the ESF descriptor
      pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor(new pcl::PointCloud<pcl::ESFSignature640>);
      //ESF estimation object
      pcl::ESFEstimation<pcl::PointXYZRGBA, pcl::ESFSignature640> esf;
      esf.setInputCloud(extractedClusters[i]);
      esf.compute(*descriptor);
      descVectESF.push_back(descriptor->points[0]);
    }
  }

  /**
  * @brief calcVFH
  * Function that calculates VFH (Viewpoint Feature Histogram)
  * ~~Global Descriptor~~
  */
  void calcVFH()
  {
    //Object for storing the VFH descriptor
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);
    for(size_t i = 0; i < extractedClusters.size(); ++i)
    {
      // VFH estimation object.
      pcl::VFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> vfh;
      vfh.setInputCloud(extractedClusters[i]);
      vfh.setInputNormals(extractedNormals[i]);
      vfh.setSearchMethod(kdtree);
      // Optionally, we can normalize the bins of the resulting histogram,
      // using the total number of points.
      vfh.setNormalizeBins(true);
      // Also, we can normalize the SDC with the maximum size found between
      // the centroid and any of the cluster's points.
      vfh.setNormalizeDistance(true);
      vfh.compute(*descriptor);
      descVectVFH.push_back(descriptor->points[0]);
    }
  }

  /**
  * @brief calcCVFH
  * Function that calculates CVFH (Clustered Viewpoint Feature Histogram)
  * ~~Global Descriptor~~
  */
  void calcCVFH()
  {
    //Object for storing the VFH descriptor
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);
    for(size_t i = 0; i < extractedClusters.size(); ++i)
    {
      //CVFH estimation object.
      pcl::CVFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> cvfh;
      cvfh.setInputCloud(extractedClusters[i]);
      cvfh.setInputNormals(extractedNormals[i]);
      cvfh.setSearchMethod(kdtree);
      //set maximum allowable derivation of the normals,
      //for the region segmentation step.
      //cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); //5 deg
      //Set the curvature threshol (maximum disparity between curvatures),
      //for the region segmentation step.
      //cvfh.setCurvatureThreshold(1.0);
      //Set to true to normalize the bins of the resulting hist,
      //using the total number of points. Note:enabling it will make
      //CVFH invariant to scale, just like VFH, but the authors encourage
      //the opposite.
      //cvfh.setRadiusSearch(0.5);
      cvfh.setNormalizeBins(true);
      cvfh.compute(*descriptor);
      descVectVFH.push_back(descriptor->points[0]);
    }
  }

#ifdef PCL_FROM_SOURCE
  void calcOURCVFH()
  {
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);
    for(size_t i = 0; i < extractedClusters.size(); ++i)
    {
      pcl::OURCVFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> ourcvfh;
      ourcvfh.setInputCloud(extractedClusters[i]);
      ourcvfh.setInputNormals(extractedNormals[i]);
      ourcvfh.setSearchMethod(kdtree);
      //ourcvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); //5 deg
      //ourcvfh.setCurvatureThreshold(1.0);
      //ourcvfh.setRadiusSearch(0.05);
      ourcvfh.setNormalizeBins(true);
      ourcvfh.compute(*descriptor);
      descVectVFH.push_back(descriptor->points[0]);
    }
  }

  void calcGRSD()
  {
    pcl::PointCloud<pcl::GRSDSignature21>::Ptr descriptor(new pcl::PointCloud<pcl::GRSDSignature21>);
    for(size_t i = 0; i < extractedClusters.size(); ++i)
    {
      pcl::GRSDEstimation<PointT, pcl::Normal, pcl::GRSDSignature21> grsd;
      grsd.setInputCloud(extractedClusters[i]);
      grsd.setInputNormals(extractedNormals[i]);
      grsd.setSearchMethod(kdtree);
      grsd.setRadiusSearch(0.05);
      grsd.compute(*descriptor);
      descVectGRSD.push_back(descriptor->points[0]);
    }
  }
#endif

  /**
  * @brief calcGFPFH
  * Function that calculates GFPFH (Global Fast Point Feature Histogram)
  * ~~Global Descriptor~~
  */
  void calcGFPFH()
  {
    for(size_t i = 0; i < extractedClusters.size(); ++i)
    {
      //Object for storing the GFPFH descriptor.
      pcl::PointCloud<pcl::GFPFHSignature16>::Ptr descriptor(new pcl::PointCloud<pcl::GFPFHSignature16>);
      //Note: you should have performed preprocessing to cluster out the object
      //from the cloud, and save it to an individual file.

      // Cloud for storing the object.
      pcl::PointCloud<pcl::PointXYZL>::Ptr object(new pcl::PointCloud<pcl::PointXYZL>);
      // Note: you should now perform classification on the cloud's points. See the
      // original paper for more details. For this example, we will now consider 4
      // different classes, and randomly label each point as one of them.

      pcl::copyPointCloud(*(extractedClusters[i]), *object);

      for(size_t j = 0; j < object->points.size(); ++j)
      {
        object->points[j].label = 1 + j % 4;
      }
      //ESF estimation object;
      pcl::GFPFHEstimation<pcl::PointXYZL, pcl::PointXYZL, pcl::GFPFHSignature16> gfpfh;
      gfpfh.setInputCloud(object);
      //Set the object that contains the labels for each point. Thanks to the
      //PointXYZL type, we can use the same object we store the cloud in.
      gfpfh.setInputLabels(object);
      //Set the size of the octree leaves to 1cm(cubic)
      gfpfh.setOctreeLeafSize(0.01);
      //Set the number of classes the cloud has been labeled with
      //(default is 16)
      gfpfh.setNumberOfClasses(4);
      gfpfh.compute(*descriptor);
      descVectGFPFH.push_back(descriptor->points[0]);
    }
  }

  /**
  * @brief calcPFH
  * Function that calculates PFH(Point Feature Histogram)
  * ~~Local Descriptor~~
  */
  void calcPFH()
  {
    for(size_t i = 0; i < extractedClusters.size(); ++i)
    {
      //Object for storing the PFH descriptors for each point.
      pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors(new pcl::PointCloud<pcl::PFHSignature125>());
      // Note: you would usually perform downsampling now. It has been omitted here
      // for simplicity, but be aware that computation can take a long time.
      //PFH estimation object.
      pcl::PFHEstimation<PointT, pcl::Normal, pcl::PFHSignature125> pfh;
      pfh.setInputCloud(extractedClusters[i]);
      pfh.setInputNormals(extractedNormals[i]);
      pfh.setSearchMethod(kdtree);
      //Search radius, to look for neighbours. Note: the value given here has
      //to be larger than the radius used to estimate the normals.
      pfh.setRadiusSearch(0.05);
      pfh.compute(*descriptors);
    }
  }

  /**
  * @brief calcFPFH
  * Function that calculates PFH(Fast Point Feature Histogram)
  * ~~Local Descriptor~~
  */
  void calcFPFH()
  {
    for(size_t i = 0; i < extractedClusters.size(); ++i)
    {
      //Object for storing the FPFH descriptors for each point.
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
      // Note: you would usually perform downsampling now. It has been omitted here
      // for simplicity, but be aware that computation can take a long time.
      //FPFH estimation object.
      pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
      fpfh.setInputCloud(extractedClusters[i]);
      fpfh.setInputNormals(extractedNormals[i]);
      fpfh.setSearchMethod(kdtree);
      //Search radius, to look for neighbours. Note: the value given here has to be
      //larger thatn the radius used to estimate the normals.
      fpfh.setRadiusSearch(0.05);
      fpfh.compute(*descriptors);
    }
  }

  /**
  * @brief calc3DSC
  * Function that calculates 3DSC (3D Shape Context)
  * ~~Local Descriptor~~
  */
  void calc3DSC()
  {
    for(size_t i = 0; i < extractedClusters.size(); ++i)
    {
      //Object for storing the 3DSC descriptors for each point.
      pcl::PointCloud<pcl::ShapeContext1980>::Ptr descriptors(new pcl::PointCloud<pcl::ShapeContext1980>());
      //3DSC estmation object.
      pcl::ShapeContext3DEstimation<PointT, pcl::Normal, pcl::ShapeContext1980> sc3d;
      sc3d.setInputCloud(extractedClusters[i]);
      sc3d.setInputNormals(extractedNormals[i]);
      sc3d.setSearchMethod(kdtree);
      //Search radius, to look for neighbours. It will also be the radius of the
      //support shphere
      sc3d.setRadiusSearch(0.05);
      //The minimal radius value for each sphere, to avoid being too sensitive
      //in bins close to the sphere center.
      sc3d.setMinimalRadius(0.05 / 10.0);
      //Radius used to compute the local point density for the neighbours
      //(the density is the number of points within that radius).
      sc3d.setPointDensityRadius(0.05 / 5.0);
      sc3d.compute(*descriptors);
    }
  }

  /**
  * @brief calcSHOT
  * Function that calculates SHOT (Signature of Histograms of Orientations)
  * ~~Local Descriptor~~
  */
  void calcSHOT()
  {
    for(size_t i = 0; i < extractedClusters.size(); ++i)
    {
      //Object for storing the SHOT descriptors for each point.
      pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());
      // Note: you would usually perform downsampling now. It has been omitted here
      // for simplicity, but be aware that computation can take a long time.
      //SHOT estimation object.
      pcl::SHOTEstimation<PointT, pcl::Normal, pcl::SHOT352> shot;
      shot.setInputCloud(extractedClusters[i]);
      shot.setInputNormals(extractedNormals[i]);
      //The radius that defines which of the keypoint's neighbours are described.
      //If too large, there may be clutter, and if too small, not enough points may be found.
      shot.setRadiusSearch(0.02);
      shot.compute(*descriptors);
    }
  }

  /**
  * @brief calcSI
  * Function that calculates SI (Spin Image)
  * ~~Local Descriptor~~
  */
  void calcSI()
  {
    for(size_t i = 0; i < extractedClusters.size(); ++i)
    {
      //Object for storing the Spin Image for each point.
      pcl::PointCloud<SpinImage>::Ptr descriptors(new pcl::PointCloud<SpinImage>());
      // Note: you would usually perform downsampling now. It has been omitted here
      // for simplicity, but be aware that computation can take a long time.
      //Spin Image estimation object.
      pcl::SpinImageEstimation<PointT, pcl::Normal, SpinImage> si;
      si.setInputCloud(extractedClusters[i]);
      si.setInputNormals(extractedNormals[i]);
      //Radius of the support cylinder.
      si.setRadiusSearch(0.02);
      //Set the resolution of the spin image
      //(the number of bins along one dimension).
      //Note:you must change the output histogram size to reflect this.
      si.setImageWidth(8);
      si.compute(*descriptors);
    }
  }

  /**
  * @brief calcRIFT
  * Function that calculates RIFT (Rotation-Invariant Feature Transform)
  * ~~Local Descriptor~~
  */
  void calcRIFT()
  {
    for(size_t i = 0; i < extractedClusters.size(); ++i)
    {
      //Object for storing the point cloud with color information.
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudColor(new pcl::PointCloud<pcl::PointXYZRGB>);
      //Object for storing the point cloud with intensity value.
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIntensity(new pcl::PointCloud<pcl::PointXYZI>);
      //Object for storing the intensity gradients.
      pcl::PointCloud<pcl::IntensityGradient>::Ptr gradients(new pcl::PointCloud<pcl::IntensityGradient>);
      //Object for storing the RIFT descriptor for each point.
      pcl::PointCloud<RIFT32>::Ptr descriptors(new pcl::PointCloud<RIFT32>());

      // Note: you would usually perform downsampling now. It has been omitted here
      // for simplicity, but be aware that computation can take a long time.

      //need to convert to XYZRGB ! (for data type consistency)

      pcl::copyPointCloud(*(extractedClusters[i]), *cloudColor);

      //Convert the RGB to intensity.
      pcl::PointCloudXYZRGBtoXYZI(*cloudColor, *cloudIntensity);

      pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);

      //compute the intensity gradients.
      pcl::IntensityGradientEstimation<pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient, pcl::common::IntensityFieldAccessor<pcl::PointXYZI>> ge;
      ge.setInputCloud(cloudIntensity);
      ge.setInputNormals(extractedNormals[i]);
      ge.setRadiusSearch(0.03);
      ge.compute(*gradients);
      //RIFT estimation object.
      pcl::RIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, RIFT32> rift;
      rift.setInputCloud(cloudIntensity);
      rift.setSearchMethod(kdtree);
      //Set the intensity gradients to use.
      rift.setInputGradient(gradients);
      //Radius, to get all the neighbours within.
      rift.setRadiusSearch(0.02);
      //Set the number of bins to use in the distance dimension.
      rift.setNrDistanceBins(4);
      //Set the number of bins to use in the gradient orientation dimension.
      rift.setNrGradientBins(8);
      //Note:you must change the output histogram size to reflect the previous values.
      rift.compute(*descriptors);
    }
  }

#define notestStoreCAS
  template<typename DescriptorSignature>
  void storeGlobalDescrToCas(const std::vector<DescriptorSignature> &descriptors, std::vector<rs::Cluster> &clusters, CAS &tcas)
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
  * @param descVect -descriptor vector containing the data to draw histograms
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
      cv::polylines(disp, points, false, colors[i % numberOfColors], 1, CV_AA);
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
      cv::rectangle(disp, border, colors[i % numberOfColors], CV_FILLED);

      cv::Rect roiCpy(topLeft, thumbnail.size());
      thumbnail.copyTo(disp(roiCpy));
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(PCLDescriptorExtractor)
