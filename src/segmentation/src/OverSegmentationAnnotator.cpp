#include <uima/api.hpp>
#include <vector>
#include <mutex>

#include <omp.h>

//PCL include
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

#include <rs/types/all_types.h>
//RS
#include <rs/DrawingAnnotator.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>


//graph
#include <rs/graph/GraphBase.hpp>
#include <rs/graph/Graph.hpp>
#include <rs/graph/GraphAlgorithms.hpp>
#include <rs/segmentation/array_utils.hpp>
#include <rs/occupancy_map/DownsampleMap.hpp>



using namespace uima;


class OverSegmentationAnnotator : public DrawingAnnotator
{
private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
  std::vector< pcl::RegionGrowing<pcl::PointXYZRGBA, pcl::Normal> > rg;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PointCloud<pcl::Normal>::Ptr normals;

  std::vector< std::vector<int> > dsMap;

  bool isDownsampled;
  float downsample_voxel_size;

  float minNormalThreshold;
  float maxNormalThreshold;
  float curvatureThreshold;

  float overlapThreshold;

  int minClusterSize;
  int maxClusterSize;

  int neighborNumber;

  int numSegmentation;

  int choose;

  double pointSize;

public:
  OverSegmentationAnnotator () : DrawingAnnotator(__func__), pointSize(1.0) {}

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    if(ctx.isParameterDefined("isDownsampled"))
    {
      ctx.extractValue("isDownsampled", isDownsampled);
    }
    if(ctx.isParameterDefined("downsample_voxel_size"))
    {
      ctx.extractValue("downsample_voxel_size", downsample_voxel_size);
    }
    if(ctx.isParameterDefined("minNormalThreshold"))
    {
      ctx.extractValue("minNormalThreshold", minNormalThreshold);
    }
    if(ctx.isParameterDefined("maxNormalThreshold"))
    {
      ctx.extractValue("maxNormalThreshold", maxNormalThreshold);
    }
    if(ctx.isParameterDefined("overlapThreshold"))
    {
      ctx.extractValue("overlapThreshold", overlapThreshold);
    }
    if(ctx.isParameterDefined("curvatureThreshold"))
    {
      ctx.extractValue("curvatureThreshold", curvatureThreshold);
    }
    if(ctx.isParameterDefined("minClusterSize"))
    {
      ctx.extractValue("minClusterSize", minClusterSize);
    }
    if(ctx.isParameterDefined("maxClusterSize"))
    {
      ctx.extractValue("maxClusterSize", maxClusterSize);
    }
    if(ctx.isParameterDefined("neighborNumber"))
    {
      ctx.extractValue("neighborNumber", neighborNumber);
    }
    if(ctx.isParameterDefined("numSegmentation"))
    {
      ctx.extractValue("numSegmentation", numSegmentation);
    }

    rg.resize(numSegmentation);

    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

    choose = 0;
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

    outInfo("process begins");
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    //get cloud from CAS
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_CLOUD,*cloud_ptr);

    //get point cloud normals from CAS
    pcl::PointCloud<pcl::Normal>::Ptr normals_ptr(new pcl::PointCloud<pcl::Normal>);
    cas.get(VIEW_NORMALS, *normals_ptr);

    //get plane indices if it has
    std::vector<int> object_indices;
    std::vector<int> plane_indices;
    std::vector<int> cloudIds(cloud_ptr->size());
    for(size_t pointId = 0; pointId < cloud_ptr->size(); pointId++)
    {
      cloudIds[pointId] = pointId;
    }

    std::vector<rs::Plane> planes;
    scene.annotations.filter(planes);
    for(size_t planeId = 0; planeId < planes.size(); planeId++)
    {
      std::vector<int> currIds = planes[planeId].inliers();
      plane_indices.insert(plane_indices.end(), currIds.begin(), currIds.end());
    }

    if(plane_indices.size() != 0)
    {
      object_indices = Difference(cloudIds, plane_indices);
    }
    else
    {
      object_indices = cloudIds;
    }

    //filter object cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr temp_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::copyPointCloud(*cloud_ptr, object_indices, *temp_cloud);
    pcl::copyPointCloud(*normals_ptr, object_indices, *temp_normals);

    if(isDownsampled)
    {
      std::vector<int> nearestMap;
      DownsampleMap<pcl::PointXYZRGBA> ds;
      ds.setInputCloud(temp_cloud);
      ds.setLeafSize(downsample_voxel_size);
      ds.filter(*cloud);
      ds.getDownsampleMap(dsMap);
      ds.getNearestNeighborMap(nearestMap);

      computeDownsampleNormals(temp_normals, dsMap, nearestMap, AVERAGE, normals);
    }
    else{
      cloud = temp_cloud;
      normals = temp_normals;
    }

    outInfo("Cloud size: " << cloud->size());
    outInfo("Normals size: " << normals->size());

    // populate normal Threshold
    std::vector<float> normalThresholds;
    if(numSegmentation == 1){
      normalThresholds.push_back(minNormalThreshold);
    }
    else{
      for(size_t i = 0 ; i < numSegmentation; i++)
        normalThresholds.push_back(i * (maxNormalThreshold - minNormalThreshold) / (numSegmentation - 1) + minNormalThreshold);
    }

    //container for segmentation Results
    std::vector< std::vector<pcl::PointIndices> > segmentations(numSegmentation);
    std::vector<pcl::PointIndices> linear_segments;

    //main execution
    #pragma omp parallel for
    for(size_t i = 0; i < numSegmentation; i++){
      pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>());

      rg[i].setMinClusterSize(minClusterSize);
      rg[i].setMaxClusterSize(maxClusterSize);
      rg[i].setNumberOfNeighbours(neighborNumber);
      rg[i].setSmoothnessThreshold(normalThresholds[i]);
      rg[i].setCurvatureThreshold(curvatureThreshold);
      rg[i].setSearchMethod(tree);
      rg[i].setInputCloud(cloud);
      rg[i].setInputNormals(normals);
      rg[i].extract(segmentations[i]);
    }

    // Merge similar segments from multiple segmentations and concat them to linear array
    int numSegments = matrixToLinear(segmentations, segmentations.size(), 0);
    outInfo("Total segments = " << numSegments);

    Graph segmentGraph(numSegments);
    std::mutex graph_mutex;

    for(size_t srcSegmentationIt = 0; srcSegmentationIt < numSegmentation - 1; srcSegmentationIt++){
      std::vector<pcl::PointIndices> src_segmentation = segmentations[srcSegmentationIt];
      for(size_t tgtSegmentationIt = srcSegmentationIt + 1; tgtSegmentationIt < numSegmentation; tgtSegmentationIt++){
        std::vector<pcl::PointIndices> tgt_segmentation = segmentations[tgtSegmentationIt];

        #pragma omp parallel for
        for(size_t srcSegmentIt = 0; srcSegmentIt < src_segmentation.size(); srcSegmentIt++){
          for(size_t tgtSegmentIt = 0; tgtSegmentIt < tgt_segmentation.size(); tgtSegmentIt++){
            int intersectSize = Intersection(src_segmentation[srcSegmentIt].indices, tgt_segmentation[tgtSegmentIt].indices).size();
            int unionSize = Union(src_segmentation[srcSegmentIt].indices, tgt_segmentation[tgtSegmentIt].indices).size();
            float ratio = (float) intersectSize / unionSize;

            if(ratio > overlapThreshold){
              int linearSrcSegmentSub = matrixToLinear(segmentations, srcSegmentationIt, srcSegmentIt);
              int linearTgtSegmentSub = matrixToLinear(segmentations, tgtSegmentationIt, tgtSegmentIt);

              graph_mutex.lock();
              segmentGraph.addEdge(linearSrcSegmentSub, linearTgtSegmentSub);
              graph_mutex.unlock();
            }
          }
        }
      }
    }

    std::vector< std::vector<int> > mergedSegmentIds = extractConnectedComponents(segmentGraph);

    outInfo("Total segments after merging " << mergedSegmentIds.size() << " segments");
    linear_segments.resize(mergedSegmentIds.size());
    for(size_t ccIt = 0; ccIt < mergedSegmentIds.size(); ccIt++){
      int maxSize = -1;
      int selectSegmentationIt = -1;
      int selectSegmentIt = -1;
      for(size_t linSegmentIdIt = 0; linSegmentIdIt < mergedSegmentIds[ccIt].size(); linSegmentIdIt++){
        int segmentationIt, segmentIt;
        int linear_id = mergedSegmentIds[ccIt][linSegmentIdIt];
        linearToMatrix(segmentations, linear_id, segmentationIt, segmentIt);

        int currSegmentSize = segmentations[segmentationIt][segmentIt].indices.size();
        if(currSegmentSize > maxSize){
          maxSize = currSegmentSize;
          selectSegmentationIt = segmentationIt;
          selectSegmentIt = segmentIt;
        }
      }

      std::vector<int> currSegment;
      if(isDownsampled)
      {
        upsample_cloud(segmentations[selectSegmentationIt][selectSegmentIt].indices, dsMap, currSegment);
        segmentations[selectSegmentationIt][selectSegmentIt].indices = currSegment;
      }

      linear_segments[ccIt] = segmentations[selectSegmentationIt][selectSegmentIt];
    }

    //for visualization purpose
    outInfo("Choosing segment cloud num: " << choose);
    colored_cloud = rg[choose].getColoredCloud();

    //publish clusters to CAS
    cas.set(VIEW_SEGMENT_IDS, linear_segments);

    return UIMA_ERR_NONE;
  }

  bool callbackKey(const int key, const Source source){
    choose = (int) key - 48;
    if(choose > numSegmentation - 1){
      choose = numSegmentation - 1;
    }
    return true;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";

    if(firstRun){
      visualizer.addPointCloud(colored_cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else{
      visualizer.updatePointCloud(colored_cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }



  void drawImageWithLock(cv::Mat& disp) {}
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(OverSegmentationAnnotator)
