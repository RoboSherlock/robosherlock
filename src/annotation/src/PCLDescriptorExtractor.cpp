#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/output.h>

//st
#include <iostream>
#include <typeinfo>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

#include <pcl/impl/point_types.hpp>

//global descriptors:

#include <pcl/features/esf.h>
#include <pcl/features/vfh.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/features/gfpfh.h>
//GRSD
//#include <pcl/features/impl/grsd.h>

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
#include<pcl/visualization/range_image_visualizer.h>

#define PCL_SOURCE

typedef pcl::PointXYZRGBA PointT;
typedef pcl::Histogram<135> ROPS135;
typedef pcl::Histogram<153> SpinImage;
typedef pcl::Histogram<32> RIFT32;
////////////////////////////////////
typedef std::vector<pcl::PointCloud<PointT>::Ptr> ClusterVector;
typedef std::vector<pcl::PointCloud<pcl::Normal>::Ptr> NormalVector;
////////////////////////
using namespace std;
using namespace cv;
using namespace uima;

static const Scalar colors[] =
{
  Scalar(255, 0, 0), //BLUE
  Scalar(50, 255, 50), //GREEN
  Scalar(0, 0, 255), //RED
  Scalar(255, 0, 255), //MAGENTA
  Scalar(247, 131, 30), //DARK TURQOUISE
  Scalar(0, 242, 255), //YELLOW
  Scalar(98, 156, 38) //DARK GREEN
};

enum class GlobalDescriptor
{
  VFH, CVFH, ESF, GFPFH, NIL
};
//OUR-CVFH, GRSD -not func (TO-DO)
enum class LocalDescriptor
{
  PFH, FPFH, _3DSC, SHOT, SI, RIFT, NIL
};
//RSD, USC, NARF, ROPS -not func (TO-DO)

static const size_t nbOfColors = sizeof(colors) / sizeof(colors[0]);


class PCLDescriptorExtractor : public DrawingAnnotator
{
private:

  int histWidthPixels = 1650;
  int hist_h = 480;
  int widthEachPicture, widthForPictures;

  std::string descType;


  double pointSize;
  /**
  * @brief cloud_ptr-The point cloud extracted from CAS
  */
  pcl::PointCloud<PointT>::Ptr cloud_ptr;
  pcl::PointCloud<pcl::Normal>::Ptr normalCloud;
  float test_param;
  std::vector<rs::Cluster> clusters;
  //std::vectors with clusters and with normals
  ClusterVector extractedClusters;
  NormalVector extractedNormals;
  /**
  * Arrays of Global Descriptors:
  */
  std::vector<pcl::ESFSignature640> descVectESF;
  std::vector<pcl::VFHSignature308> descVectVFH;
  std::vector<pcl::VFHSignature308> descVectCVFH;
  std::vector<pcl::GFPFHSignature16> descVectGFPFH;
  //the much needed KdTree structure
  pcl::search::KdTree<PointT>::Ptr kdtree;
  cv::Mat hists;

  GlobalDescriptor globalD;
  LocalDescriptor localD;


  cv::Mat color;
  std::vector<cv::Rect> clusterRois;

public:
  PCLDescriptorExtractor() : DrawingAnnotator(__func__), pointSize(1),
    cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>),
    normalCloud(new pcl::PointCloud<pcl::Normal>)
  {}

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    //std::string descType;
    ctx.extractValue("descriptorType", descType);
    /**
     * Global Descriptors:
     */
    if(descType == "ESF")
    {
      globalD = GlobalDescriptor::ESF;
      localD = LocalDescriptor::NIL;
    }
    else if(descType == "VFH")
    {
      globalD = GlobalDescriptor::VFH;
      localD = LocalDescriptor::NIL;
    }
    else if(descType == "CVFH")
    {
      globalD = GlobalDescriptor::CVFH;
      localD = LocalDescriptor::NIL;
    }
    else if(descType == "GFPFH")
    {
      globalD = GlobalDescriptor::GFPFH;
      localD = LocalDescriptor::NIL;
    }


    /**
     * Local Descriptors:
     */
    else if(descType == "PFH")
    {
      globalD = GlobalDescriptor::NIL;
      localD = LocalDescriptor::PFH;
    }
    else if(descType == "FPFH")
    {
      globalD = GlobalDescriptor::NIL;
      localD = LocalDescriptor::FPFH;
    }
    else if(descType == "3DSC")
    {
      globalD = GlobalDescriptor::NIL;
      localD = LocalDescriptor::_3DSC;
    }
    else if(descType == "SHOT")
    {
      globalD = GlobalDescriptor::NIL;
      localD = LocalDescriptor::SHOT;
    }
    else if(descType == "SI")
    {
      globalD = GlobalDescriptor::NIL;
      localD = LocalDescriptor::SI;
    }
    else if(descType == "RIFT")
    {
      globalD = GlobalDescriptor::NIL;
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

  /**
  * @brief drawHistograms -OpenCV histogram drawing
  * @param descVect -descriptor vector containing the data to draw histograms
  * @param title -the string to be displayed on the histogram (descriptor name)
  */
  template<typename globalDescSign>
  void drawHistograms(const std::vector<globalDescSign> &descVect, std::string title)
  {
    //each cluster's decriptor should have same nb of hist values
    int nbOfEntries = sizeof(descVect.at(0).histogram) / sizeof(float);
    cv::Mat histImage(hist_h + 400, histWidthPixels, CV_8UC3, Scalar(170, 170, 170));
    //Draw axes:
    //horizontal axis///////////////////////////////
    //body
    line(histImage, Point(9, hist_h + 41),
         Point(1630, hist_h + 41),
         Scalar(0, 0, 0), 1, 8, 0);
    //its arrow
    //upper part
    line(histImage, Point(1620, hist_h + 36),
         Point(1630, hist_h + 41),
         Scalar(0, 0, 0), 1, 8, 0);
    //lower part
    line(histImage, Point(1620, hist_h + 46),
         Point(1630, hist_h + 41),
         Scalar(0, 0, 0), 1, 8, 0);
    //////////////////////////////////////////////////

    //vertical axis///////////////////////////////////
    line(histImage, Point(9, hist_h + 41),
         Point(9, 20),
         Scalar(0, 0, 0), 1, 8, 0);
    //its arrow
    //left part
    line(histImage, Point(4, 30),
         Point(9, 20),
         Scalar(0, 0, 0), 1, 8, 0);
    //right part
    line(histImage, Point(14, 30),
         Point(9, 20),
         Scalar(0, 0, 0), 1, 8, 0);
    ///////////////////////////////////////////////////

    /////////title of the histogram (descriptor name)/////////////////////////
    putText(histImage, title, cvPoint(30, 30),
            FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(102, 0, 102), 1, CV_AA);
    //////////////////////////////////////////////////////////////////////////

    for(int i = 0; i < clusters.size(); i++)
    {
      //the key to the diagram////////////////////////////////////////////////////
      cv::rectangle(
        histImage,
        cv::Point(10, hist_h + 60 + 20 * i),
        cv::Point(40, hist_h + 60 + 20 * i + 10),
        colors[i % nbOfColors],
        CV_FILLED
      );
      putText(histImage, "Cluster#" + std::to_string(i), cvPoint(40, hist_h + 70 + 20 * i),
              FONT_HERSHEY_COMPLEX_SMALL, 0.95, colors[i % nbOfColors], 1, CV_AA);
      ////////////////////////////////////////////////////////////////////////////
      float maxHist = 0;
      for(int j = 1; j < nbOfEntries; j++)
      {
        if(descVect.at(i).histogram[j] > maxHist)
        {
          maxHist = descVect.at(i).histogram[j];
        }
      }
      for(int j = 1; j < nbOfEntries; j++)
      {
        float heightOfEntry;
        float pastEntryHeight;
        heightOfEntry = hist_h * (descVect.at(i).histogram[j]) / maxHist;
        pastEntryHeight = hist_h * (descVect.at(i).histogram[j - 1] / maxHist);
        line(histImage, Point(10 + ((histWidthPixels) / nbOfEntries)*j, hist_h - heightOfEntry + 40),
             Point(10 + ((histWidthPixels) / nbOfEntries) * (j - 1), hist_h - pastEntryHeight + 40),
             colors[i % nbOfColors], 1, 8, 0);
      }
    }
    hists = histImage.clone();
  }


  /**
  * @brief extractClustersAndNormals-func to extract clusters from input cloud
  * @param inputCloud-a single point cloud
  * @return extractedClusters-vector of clusters extracted
  */
  void extractClustersAndNormals()
  {
    //first, empty the vector of clusters and the vector of normals
    extractedClusters.clear();
    extractedNormals.clear();
    //next, iterate over clusters
    for(int i = 0; i < clusters.size(); ++i)
    {
      rs::Cluster &cluster = clusters[i];
      if(!cluster.points.has())
      {
        continue;
      }
      pcl::PointIndicesPtr indices(new pcl::PointIndices());
      rs::conversion::from(((rs::ReferenceClusterPoints)cluster.points.get()).indices.get(), *indices);
      pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>());
      pcl::PointCloud<pcl::Normal>::Ptr clusterNormals(new pcl::PointCloud<pcl::Normal>);
      pcl::ExtractIndices<PointT> ei;
      ei.setInputCloud(cloud_ptr);
      ei.setIndices(indices);
      ei.filter(*cluster_cloud);
      extractedClusters.push_back(cluster_cloud);

      pcl::ExtractIndices<pcl::Normal> eiNormal;
      eiNormal.setInputCloud(normalCloud);
      eiNormal.setIndices(indices);
      eiNormal.filter(*clusterNormals);
      extractedNormals.push_back(clusterNormals);
    }
  }

  /**
  * @brief calcESF
  * Function that calculates ESF (Ensemble of Shape Functions)
  * ~~Global Descriptor~~
  */
  void calcESF()
  {
    for(int i = 0; i < clusters.size(); ++i)
    {
      //Object for storing the ESF descriptor
      pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor(new pcl::PointCloud<pcl::ESFSignature640>);
      //ESF estimation object
      pcl::ESFEstimation<pcl::PointXYZRGBA, pcl::ESFSignature640>esf;
      esf.setInputCloud(extractedClusters.at(i));
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
    for(int i = 0; i < clusters.size(); ++i)
    {
      // VFH estimation object.
      pcl::VFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> vfh;
      vfh.setInputCloud(extractedClusters.at(i));
      vfh.setInputNormals(extractedNormals.at(i));
      vfh.setSearchMethod(kdtree);
      // Optionally, we can normalize the bins of the resulting histogram,
      // using the total number of points.
      vfh.setNormalizeBins(true);
      // Also, we can normalize the SDC with the maximum size found between
      // the centroid and any of the cluster's points.
      vfh.setNormalizeDistance(false);
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
    for(int i = 0; i < clusters.size(); ++i)
    {
      //CVFH estimation object.
      pcl::CVFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> cvfh;
      cvfh.setInputCloud(extractedClusters.at(i));
      cvfh.setInputNormals(extractedNormals.at(i));
      cvfh.setSearchMethod(kdtree);
      //set maximum allowable derivation of the normals,
      //for the region segmentation step.
      cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); //5 deg
      //Set the curvature threshol (maximum disparity between curvatures),
      //for the region segmentation step.
      cvfh.setCurvatureThreshold(1.0);
      //Set to true to normalize the bins of the resulting hist,
      //using the total number of points. Note:enabling it will make
      //CVFH invariant to scale, just like VFH, but the authors encourage
      //the opposite.
      cvfh.setNormalizeBins(false);
      cvfh.compute(*descriptor);
      descVectCVFH.push_back(descriptor->points[0]);
    }
  }

  /**
  * @brief calcGFPFH
  * Function that calculates GFPFH (Global Fast Point Feature Histogram)
  * ~~Global Descriptor~~
  */
  void calcGFPFH()
  {
    for(int i = 0; i < clusters.size(); ++i)
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

      pcl::copyPointCloud(*(extractedClusters.at(i)), *object);

      for(int var = 0; var < object->points.size(); ++var)
      {
        object->points[i].label = 1 + i % 4;
      }
      //ESF estimation object;
      pcl::GFPFHEstimation<pcl::PointXYZL, pcl::PointXYZL, pcl::GFPFHSignature16>gfpfh;
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
    for(int i = 0; i < clusters.size(); ++i)
    {
      //Object for storing the PFH descriptors for each point.
      pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors(new pcl::PointCloud<pcl::PFHSignature125>());
      // Note: you would usually perform downsampling now. It has been omitted here
      // for simplicity, but be aware that computation can take a long time.
      //PFH estimation object.
      pcl::PFHEstimation<PointT, pcl::Normal, pcl::PFHSignature125>pfh;
      pfh.setInputCloud(extractedClusters.at(i));
      pfh.setInputNormals(extractedNormals.at(i));
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
    for(int i = 0; i < clusters.size(); ++i)
    {
      //Object for storing the FPFH descriptors for each point.
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
      // Note: you would usually perform downsampling now. It has been omitted here
      // for simplicity, but be aware that computation can take a long time.
      //FPFH estimation object.
      pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33>fpfh;
      fpfh.setInputCloud(extractedClusters.at(i));
      fpfh.setInputNormals(extractedNormals.at(i));
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
    for(int i = 0; i < clusters.size(); ++i)
    {
      //Object for storing the 3DSC descriptors for each point.
      pcl::PointCloud<pcl::ShapeContext1980>::Ptr descriptors(new pcl::PointCloud<pcl::ShapeContext1980>());
      //3DSC estmation object.
      pcl::ShapeContext3DEstimation<PointT, pcl::Normal, pcl::ShapeContext1980>sc3d;
      sc3d.setInputCloud(extractedClusters.at(i));
      sc3d.setInputNormals(extractedNormals.at(i));
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
    for(int i = 0; i < clusters.size(); ++i)
    {
      //Object for storing the SHOT descriptors for each point.
      pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());
      // Note: you would usually perform downsampling now. It has been omitted here
      // for simplicity, but be aware that computation can take a long time.
      //SHOT estimation object.
      pcl::SHOTEstimation<PointT, pcl::Normal, pcl::SHOT352>shot;
      shot.setInputCloud(extractedClusters.at(i));
      shot.setInputNormals(extractedNormals.at(i));
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
    for(int i = 0; i < clusters.size(); ++i)
    {
      //Object for storing the Spin Image for each point.
      pcl::PointCloud<SpinImage>::Ptr descriptors(new pcl::PointCloud<SpinImage>());
      // Note: you would usually perform downsampling now. It has been omitted here
      // for simplicity, but be aware that computation can take a long time.
      //Spin Image estimation object.
      pcl::SpinImageEstimation<PointT, pcl::Normal, SpinImage>si;
      si.setInputCloud(extractedClusters.at(i));
      si.setInputNormals(extractedNormals.at(i));
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
    for(int i = 0; i < clusters.size(); ++i)
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

      pcl::copyPointCloud(*(extractedClusters.at(i)), *cloudColor);

      //Convert the RGB to intensity.
      pcl::PointCloudXYZRGBtoXYZI(*cloudColor, *cloudIntensity);

      pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);

      //compute the intensity gradients.
      pcl::IntensityGradientEstimation < pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient,
          pcl::common::IntensityFieldAccessor<pcl::PointXYZI> > ge;
      ge.setInputCloud(cloudIntensity);
      ge.setInputNormals(extractedNormals.at(i));
      ge.setRadiusSearch(0.03);
      ge.compute(*gradients);
      //RIFT estimation object.
      pcl::RIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, RIFT32>rift;
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
  template<typename globalDescSign>
  void storeGlobalDescrToCas(const std::vector<globalDescSign> &descVect, CAS &tcas)
  {
#ifdef testStoreCAS
    std::vector<float> testDesc;
#endif
    for(std::vector<rs::Cluster>::iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
      rs::PclFeature annotation = rs::create<rs::PclFeature>(tcas);
      //annotatio.feature requires std::vector instead of an array
      std::vector<float> descVectStd(
        descVect.at(it - clusters.begin()).histogram, descVect.at(it - clusters.begin()).histogram +
        sizeof(descVect.at(0).histogram) / sizeof(descVect.at(0).histogram[0])
      );
      annotation.feat_type.set(std::string(typeid(globalDescSign).name()));
      annotation.feature.set(descVectStd);
      it->annotations.append(annotation);

#ifdef testStoreCAS
      std::vector<rs::PclFeature> descrVect;
      it->annotations.filter(descrVect);
      if(!descrVect.empty())
      {
        rs::PclFeature tFeat = descrVect[0];
        testDesc = tFeat.feature();
      }
#endif
    }
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    clusters.clear();
    //retrieve the point cloud to be processed:
    cas.get(VIEW_CLOUD,*cloud_ptr);
    cas.get(VIEW_NORMALS,*normalCloud);

    //2.filter out clusters into array
    scene.identifiables.filter(clusters);
    outInfo("Number of clusters:" << clusters.size());

    clusterRois.resize(clusters.size());

    //extract clusters into a vector
    extractClustersAndNormals();
    cas.get(VIEW_COLOR_IMAGE,color);


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

    switch(globalD)
    {
    case GlobalDescriptor::VFH:
      descVectVFH.clear();
      calcVFH();
      drawHistograms(descVectVFH, "Viewpoint Feature Histogram (VFH)");
      storeGlobalDescrToCas(descVectVFH, tcas);
      break;

    case GlobalDescriptor::CVFH:
      descVectCVFH.clear();
      calcCVFH();
      drawHistograms(descVectCVFH, "Clustered Viewpoint Feature Histogram (CVFH)");
      storeGlobalDescrToCas(descVectCVFH, tcas);
      break;
      //      //TO-DO:case OUR-CVFH
    case GlobalDescriptor::ESF:
      descVectESF.clear();
      calcESF();
      drawHistograms(descVectESF, "Ensemble of Shape Functions (ESF)");
      storeGlobalDescrToCas(descVectESF, tcas);
      break;
    case GlobalDescriptor::GFPFH:
      descVectGFPFH.clear();
      calcGFPFH();
      drawHistograms(descVectGFPFH, "Global Fast Point Feature Histogram (GFPFH)");
      storeGlobalDescrToCas(descVectGFPFH, tcas);
      break;
      //      //TO-DO:case GRSD
    default:
      break;
    }

    //variable used for drawing images of clusters to the right of the histogram's key
    int i = 150;
    widthForPictures = histWidthPixels - i;
    widthEachPicture = widthForPictures / clusters.size();
    // Extract ROIS from RGB image
    for(size_t idx = 0; idx < clusters.size(); ++idx)
    {
      rs::ImageROI image_rois = clusters[idx].rois.get();
      //======================= Get ROI image ==========================
      cv::Rect roi;
      rs::conversion::from(image_rois.roi(), roi);
      clusterRois[idx] = roi;
      //store current Roi size
      cv::Mat objImgToCopy;
      ///Create a ROI on the hists image and copy each cluser's image there
      color(roi).copyTo(objImgToCopy);

      if(objImgToCopy.cols < widthEachPicture - 20 && objImgToCopy.rows < widthEachPicture - 20)
      {
        int centDisplX = (widthEachPicture - objImgToCopy.cols) / 2;
        //Bounding coloured rectangle for each cluster's RGB image
        cv::rectangle(
          hists,
          cv::Point(i + centDisplX - 10, 590),
          cv::Point(i + centDisplX + objImgToCopy.cols + 10, 610 + objImgToCopy.rows),
          colors[idx % nbOfColors],
          CV_FILLED
        );
        cv::Rect roiCpy(cv::Point(i + centDisplX, 600), color(roi).size());
        color(roi).copyTo(hists(roiCpy));
      }
      else
      {
        cv::Mat tempImage;
        cv::Mat tempImage2;
        color(roi).copyTo(tempImage);
        int targetWidth = widthEachPicture - 20;
        int targetHeight = roi.height * targetWidth / roi.width;
        cv::resize(tempImage, tempImage2, Size(targetWidth, targetHeight), 0, 0, INTER_CUBIC);

        int centDisplX = (widthEachPicture - tempImage2.cols) / 2;
        //Bounding coloured rectangle for each cluster's RGB image
        cv::rectangle(
          hists,
          cv::Point(i + centDisplX - 10, 590),
          cv::Point(i + centDisplX + tempImage2.cols + 10, 610 + tempImage2.rows),
          colors[idx % nbOfColors],
          CV_FILLED
        );
        cv::Rect roiCpy(cv::Point(i + centDisplX, 600), tempImage2.size());
        tempImage2.copyTo(hists(roiCpy));
      }
      i += widthEachPicture;
    }

    outInfo("took: " << clock.getTime() << " ms.");

    return UIMA_ERR_NONE;
  }
  void drawImageWithLock(cv::Mat &disp)
  {
    if(localD == LocalDescriptor::NIL) //don't use visualizer with local descriptors, because it gives SEGFAULT(no drawing hists)
    {
      disp = hists.clone();
    }
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    if(firstRun)
    {
      visualizer.addPointCloud(cloud_ptr, "cloudname");
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloudname");
    }
    else
    {
      visualizer.updatePointCloud(cloud_ptr, "cloudname");
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloudname");
      visualizer.removeAllShapes();
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(PCLDescriptorExtractor)
