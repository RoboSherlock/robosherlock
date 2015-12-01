// UIMA
#include <uima/api.hpp>

// RS
#include <rs/DrawingAnnotator.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>

// PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing_rgb.h>

using namespace uima;

class PointCloudColorSegmentation : public DrawingAnnotator
{

private:
  typedef pcl::PointXYZRGBA PointT;
  pcl::PointCloud<PointT>::Ptr cloudPtr;
  bool useThermal, useSupervoxel, persons, voxelGridFiltering;
  std::vector<pcl::PointIndices> clusterIndices;
  int distanceThreshold, pointColorThreshold, regionColorThreshold, minClusterSize;
  double pointSize;
  float voxelResolution;
  pcl::RegionGrowingRGB<PointT> reg;

public:

  PointCloudColorSegmentation(): DrawingAnnotator(__func__), useThermal(true), useSupervoxel(false), persons(true), voxelGridFiltering(true),
    distanceThreshold(5), pointColorThreshold(6), regionColorThreshold(5), minClusterSize(10), pointSize(2.0), voxelResolution(0.008f) {}

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("Initialize");

    if(ctx.isParameterDefined("useThermal"))
    {
      ctx.extractValue("useThermal", useThermal);
    }
    if(ctx.isParameterDefined("useSupervoxel"))
    {
      ctx.extractValue("useSupervoxel", useSupervoxel);
    }
    if(ctx.isParameterDefined("persons"))
    {
      ctx.extractValue("persons", persons);
    }
    if(ctx.isParameterDefined("distanceThreshold"))
    {
      ctx.extractValue("distanceThreshold", distanceThreshold);
    }
    if(ctx.isParameterDefined("pointColorThreshold"))
    {
      ctx.extractValue("pointColorThreshold", pointColorThreshold);
    }
    if(ctx.isParameterDefined("regionColorThreshold"))
    {
      ctx.extractValue("regionColorThreshold", regionColorThreshold);
    }
    if(ctx.isParameterDefined("minClusterSize"))
    {
      ctx.extractValue("minClusterSize", minClusterSize);
    }
    if(ctx.isParameterDefined("voxelGridFiltering") && ctx.isParameterDefined("voxelResolution"))
    {
      ctx.extractValue("voxelGridFiltering", voxelGridFiltering);
      ctx.extractValue("voxelResolution", voxelResolution);
    }

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("Destroy");

    return UIMA_ERR_NONE;
  }

private:

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("Process begins");
    rs::StopWatch clock;

    rs::SceneCas cas(tcas);

    cloudPtr.reset(new pcl::PointCloud<PointT>);

    clusterIndices.clear();
    if(useThermal)
    {
      cas.get(VIEW_THERMAL_CLOUD, *cloudPtr);
    }
    else if(useSupervoxel)
    {
      cas.get(VIEW_CLOUD_SUPERVOXEL, *cloudPtr);
    }
    else
    {
      cas.get(VIEW_CLOUD, *cloudPtr);
    }
    pcl::PointCloud<PointT> cloudCopy = *cloudPtr;

    if(voxelGridFiltering)
    {
      pcl::VoxelGrid<PointT> sor;
      sor.setInputCloud(cloudPtr);
      sor.setLeafSize(voxelResolution, voxelResolution, voxelResolution);
      pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT> ());
      sor.filter(*cloudFiltered);
      cloudPtr = cloudFiltered;
    }

    pcl::IndicesPtr denseIndices(new std::vector <int>);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloudPtr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 100.0);
    pass.filter(*denseIndices);

    double t1 = clock.getTime();
    outInfo("Pass through took: " << t1 << " ms.");

    pcl::search::Search <PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);

    reg.setInputCloud(cloudPtr);
    reg.setIndices(denseIndices);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(distanceThreshold);
    reg.setPointColorThreshold(pointColorThreshold);
    reg.setRegionColorThreshold(regionColorThreshold);
    reg.setMinClusterSize(minClusterSize);
    reg.extract(clusterIndices);

    double t2 = clock.getTime();
    outInfo("Segmentation took: " << t2 - t1 << " ms.");

    if(persons)
    {
      for(pcl::PointIndices indices : clusterIndices)
      {
        //        rs::PersonCluster pc = rs::create<rs::PersonCluster>(tcas);
        //        rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
        //        rs::PointIndices uimaIndices = rs::conversion::to(tcas, indices);
        //        rcp.indices.set(uimaIndices);
        //        rs::ImageROI roi = createImageRoi(tcas, cloudCopy, indices);
        //        pc.indices.set(rcp);
        //        pc.roi.set(roi);
        //        scene.identifiables.append(pc);
      }
    }
    else
    {
      // TODO: Add object clusters here
    }

    double t3 = clock.getTime();
    outInfo("Conversion took: " << t3 - t2 << " ms.");

    return UIMA_ERR_NONE;
  }

  /**
   * given orignal_image and reference cluster points, compute an image containing only the cluster
   */
  rs::ImageROI createImageRoi(CAS &tcas, const pcl::PointCloud<PointT> &cloud, const pcl::PointIndices &indices)
  {
    size_t width = cloud.width,
           height = cloud.height;

    int min_x = width,
        max_x = -1,
        min_y = height,
        max_y = -1;

    cv::Mat mask_full = cv::Mat::zeros(height, width, CV_8U);

    // get min / max extents (rectangular bounding box in image (pixel) coordinates)
    #pragma omp parallel for
    for(size_t i = 0; i < indices.indices.size(); ++i)
    {
      const int idx = indices.indices[i],
                x = idx % width,
                y = idx / width;

      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);

      mask_full.at<uint8_t>(y, x) = 255;
    }

    cv::Rect roi(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
    cv::Mat mask;
    mask = mask_full(roi);

    rs::ImageROI imageROI = rs::create<rs::ImageROI>(tcas);
    imageROI.mask(rs::conversion::to(tcas, mask));
    imageROI.roi(rs::conversion::to(tcas, roi));
    return imageROI;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = cv::Mat::zeros(240, 320, CV_8UC3);
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = this->name;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = reg.getColoredCloud();

    if(firstRun)
    {
      visualizer.addPointCloud(cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }

};

MAKE_AE(PointCloudColorSegmentation)
