// UIMA
#include <uima/api.hpp>

// RS
#include <rs/DrawingAnnotator.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>
#include <rs/utils/common.h>
#include <rs/conversion/bson.h>

// PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <rs_queryanswering/KRDefinitions.h>
#include <rs_queryanswering/PrologInterface.h>




#include <algorithm>

//#include <rs/utils/RSAnalysisEngine.h>

using namespace uima;

class ClusterToPartsSegmenter : public DrawingAnnotator
{

private:
  typedef pcl::PointXYZRGBA PointT;
  pcl::PointCloud<PointT>::Ptr cloudPtr_;
  pcl::PointCloud<PointT>::Ptr dispCloudPtr;
  pcl::PointCloud<pcl::Normal>::Ptr normalPtr_;

  //  RSAnalysisEngine engine;
  //  PrologInterface prologInterface;
  struct ClusterWithParts
  {
    pcl::PointIndicesPtr indices;
    std::vector<pcl::PointIndicesPtr> partsOfClusters;
    pcl::PointCloud<pcl::PointXYZL>::Ptr labeledCloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr svNormalCloud;
    std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxelClusters;
  };


  std::vector<ClusterWithParts> clustersWithParts;

  double pointSize;

  //supervoxels
  float voxel_resolution;
  float seed_resolution;
  float colorImportance_;
  float spatialImportance_;
  float normalImportance_;

  pcl::PointCloud<PointT>::Ptr dispCloud_;
  pcl::PointCloud<pcl::PointNormal>::Ptr svNormalCloud;

  cv::Mat dispRGB;

  enum class DisplayMode
  {
    MERGED,
    SUPERVOX
  } dispMode;
public:

  ClusterToPartsSegmenter(): DrawingAnnotator(__func__), dispCloudPtr(new pcl::PointCloud<PointT>), pointSize(2),
    voxel_resolution(0.01f),
    seed_resolution(0.07f)
  {
  }


  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("Initialize");
    colorImportance_ = 0.0f;
    spatialImportance_ = 0.5f;
    normalImportance_ = 1.0f;

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("Destroy");
    return UIMA_ERR_NONE;
  }

  void overSegmentAndGrow(const pcl::PointIndicesPtr &indices, ClusterWithParts &cwp)
  {
    pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> ei;
    ei.setInputCloud(cloudPtr_);
    ei.setIndices(indices);
    ei.filter(*clusterCloud);
    cwp.indices = indices;


    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::PointCloud<PointT>::Ptr mls_points(new pcl::PointCloud<PointT>);
    pcl::MovingLeastSquares<PointT, PointT> mls;

    // Set parameters
    mls.setInputCloud(clusterCloud);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);

    // Reconstruct
    mls.process(*mls_points);

    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(mls_points);
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.03);
    ne.compute(*cloud_normals);

    pcl::SupervoxelClustering<PointT> superVoxClust(voxel_resolution, seed_resolution);
    superVoxClust.setInputCloud(mls_points);
    superVoxClust.setNormalCloud(cloud_normals);
    superVoxClust.setUseSingleCameraTransform(false);
    superVoxClust.setColorImportance(colorImportance_);
    superVoxClust.setSpatialImportance(spatialImportance_);
    superVoxClust.setNormalImportance(normalImportance_);
    superVoxClust.extract(cwp.supervoxelClusters);

    cwp.svNormalCloud = superVoxClust.makeSupervoxelNormalCloud(cwp.supervoxelClusters);

    outInfo("Cluster split into: " << cwp.supervoxelClusters.size() << " supervoxels");

    //group voxels based on their surface normal in two groups...a bit hacky atm. will make it nicer eventually
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    superVoxClust.getSupervoxelAdjacency(supervoxel_adjacency);
    //    std::multimap<uint32_t, uint32_t>::iterator labelItr = supervoxel_adjacency.begin();
    std::vector<bool> processed(cwp.supervoxelClusters.size(), false);

    pcl::PointIndicesPtr partOneIndices(new pcl::PointIndices());
    pcl::PointIndicesPtr partTwoIndices(new pcl::PointIndices());

    for(auto labelItr = supervoxel_adjacency.begin();
        labelItr != supervoxel_adjacency.end();)
    {
      uint32_t supervoxel_label = labelItr->first;
      pcl::Supervoxel<PointT>::Ptr supervoxel = cwp.supervoxelClusters.at(supervoxel_label);
      pcl::PointNormal svNormal;
      supervoxel->getCentroidPointNormal(svNormal);
      std::multimap<uint32_t, uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first;
      for(; adjacent_itr != supervoxel_adjacency.equal_range(supervoxel_label).second; ++adjacent_itr)
      {
        pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = cwp.supervoxelClusters.at(adjacent_itr->second);
        pcl::PointNormal svNeighbourNormal;
        neighbor_supervoxel->getCentroidPointNormal(svNeighbourNormal);
        Eigen::Map< Eigen::Vector3f> point_a_normal(svNormal.normal);
        Eigen::Map< Eigen::Vector3f> point_b_normal(svNeighbourNormal.normal);

        //outInfo("Angle between svl:" << supervoxel_label << " and svl:" << adjacent_itr->second << " is: " << fabs(point_a_normal.dot(point_b_normal)));
        Eigen::Vector3f dist(svNormal.x - svNeighbourNormal.x, svNormal.y - svNeighbourNormal.y, svNormal.z - svNeighbourNormal.z);
        if(std::abs(point_a_normal.dot(point_b_normal)) > 0.96 && !processed[(int)adjacent_itr->second - 1])
        {
          processed[(int)adjacent_itr->second - 1] = true;
        }
      }

      labelItr = supervoxel_adjacency.upper_bound(supervoxel_label);
    }

    cwp.labeledCloud = superVoxClust.getLabeledCloud();
    for(unsigned int i =  0; i < cwp.labeledCloud->points.size(); ++i)
    {
      pcl::PointXYZL &p = cwp.labeledCloud->points[i];
      if(p.label != 0 && processed[(int)p.label - 1])
      {
        partOneIndices->indices.push_back(indices->indices.at(i));
      }
      else
      {
        partTwoIndices->indices.push_back(indices->indices.at(i));
      }
    }
    outInfo(partOneIndices->indices.size());
    outInfo(partTwoIndices->indices.size());
    if(partOneIndices->indices.size() > indices->indices.size() / 3)
      cwp.partsOfClusters.push_back(partOneIndices);
    if(partTwoIndices->indices.size() > indices->indices.size() / 3)
      cwp.partsOfClusters.push_back(partTwoIndices);
  }


  void createImageRoi(const pcl::PointIndicesPtr &indices,
                      cv::Rect &roi, cv::Rect &roiHires,
                      cv::Mat &mask, cv::Mat &maskHires)
  {


    size_t width = cloudPtr_->width;
    size_t height = cloudPtr_->height;

    int min_x = width;
    int max_x = -1;
    int min_y = height;
    int max_y = -1;

    cv::Mat mask_full = cv::Mat::zeros(height, width, CV_8U);
    for(size_t i = 0; i < indices->indices.size(); ++i)
    {
      const int idx = indices->indices[i];
      const int x = idx % width;
      const int y = idx / width;

      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);

      mask_full.at<uint8_t>(y, x) = 255;
    }

    roi = cv::Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
    roiHires = cv::Rect(roi.x << 1, roi.y << 1, roi.width << 1, roi.height << 1);
    mask_full(roi).copyTo(mask);
    cv::resize(mask, maskHires, cv::Size(0, 0), 2.0, 2.0, cv::INTER_NEAREST);
  }

private:

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("Process begins");

    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    std::vector<rs::Cluster> clusters;
    std::vector<rs::Plane> planes;
    clustersWithParts.clear();

    cloudPtr_.reset(new pcl::PointCloud<PointT>);
    normalPtr_.reset(new pcl::PointCloud<pcl::Normal>);

    cv::Mat rgbHD;
    cas.get(VIEW_CLOUD, *cloudPtr_);
    cas.get(VIEW_NORMALS, *normalPtr_);
    cas.get(VIEW_COLOR_IMAGE, rgbHD);



    dispRGB = rgbHD.clone();

    scene.identifiables.filter(clusters);
    scene.annotations.filter(planes);

    rs::Query query = rs::create<rs::Query>(tcas);
    std::string obj_to_inspect = "";
    if(cas.getFS("QUERY", query))
    {
      std::string queryAsString = query.asJson();
      if(queryAsString != "")
      {
        rapidjson::Document doc;
        doc.Parse(queryAsString.c_str());
        if(doc.HasMember("inspect"))
        {
          if(doc["inspect"].HasMember("obj"))
          {
            obj_to_inspect = doc["inspect"]["obj"].GetString();
          }
        }
      }
    }

    std::vector<rs::Identifiable> mergedClusters;
    for(int i = 0; i < clusters.size(); ++i)
    {
      rs::Cluster &cluster = clusters[i];

      ClusterWithParts clusterAsParts;

      //3D segmentation
      pcl::PointIndicesPtr clusterIndices(new pcl::PointIndices());
      rs::conversion::from(((rs::ReferenceClusterPoints)cluster.points.get()).indices.get(), *clusterIndices);
      if(cluster.source() == "TransparentSegmentation" ||
              cluster.source() == "ImageSegmentation")
      {
        mergedClusters.push_back(cluster);
        continue;
      }
      overSegmentAndGrow(clusterIndices, clusterAsParts);
      outInfo("Oversegmented: " << clusterAsParts.partsOfClusters.size());
      clustersWithParts.push_back(clusterAsParts);

      if(clusterAsParts.partsOfClusters.size() > 1)
      {
        int idxBiggest = -1;
        int nrOfIndeices = 0;
        for(int pclClIdx = 0; pclClIdx < clusterAsParts.partsOfClusters.size(); pclClIdx++)
        {
          if(clusterAsParts.partsOfClusters[pclClIdx]->indices.size() > nrOfIndeices)
          {
            nrOfIndeices = clusterAsParts.partsOfClusters[pclClIdx]->indices.size();
            idxBiggest = pclClIdx;
          }
        }
        for(int pclClIdx = 0; pclClIdx < clusterAsParts.partsOfClusters.size(); pclClIdx++)
        {
          rs::Cluster newCluster = rs::create<rs::Cluster>(tcas);
          rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
          rs::PointIndices uimaIndices = rs::conversion::to(tcas, *clusterAsParts.partsOfClusters[pclClIdx]);
          rcp.indices.set(uimaIndices);

          cv::Rect roi, roiHires;
          cv::Mat mask, maskHires;
          createImageRoi(clusterAsParts.partsOfClusters[pclClIdx], roi, roiHires, mask, maskHires);

          rs::ImageROI imageRoi = rs::create<rs::ImageROI>(tcas);
          imageRoi.mask(rs::conversion::to(tcas, mask));
          imageRoi.mask_hires(rs::conversion::to(tcas, maskHires));
          imageRoi.roi(rs::conversion::to(tcas, roi));
          imageRoi.roi_hires(rs::conversion::to(tcas, roiHires));

          newCluster.rois.set(imageRoi);
          newCluster.points.set(rcp);
          mergedClusters.push_back(newCluster);
        }
      }

      else
      {
        mergedClusters.push_back(cluster);
      }
    }

    scene.identifiables.set(mergedClusters);
    return UIMA_ERR_NONE;
  }

  bool callbackKey(const int key, const Source source)
  {
    switch(key)
    {
    case 'm':
      dispMode = DisplayMode::MERGED;
      return true;
    case 's':
      dispMode = DisplayMode::SUPERVOX;
      return true;
    }
    return false;
  }


  void drawImageWithLock(cv::Mat &disp)
  {
    disp = dispRGB.clone();

    int colorIdx = 0;
    for(unsigned int i = 0; i < clustersWithParts.size(); ++i)
    {
      if(clustersWithParts[i].partsOfClusters.size() > 1)
      {
        for(unsigned int j = 0; j < clustersWithParts[i].partsOfClusters.size(); ++j)
        {
          pcl::PointIndicesPtr &indices = clustersWithParts[i].partsOfClusters[j];
          for(unsigned int k = 0; k < indices->indices.size(); ++k)
          {
            int index = indices->indices[k];
            disp.at<cv::Vec3b>(index) = rs::common::cvVec3bColors[colorIdx];
          }
          colorIdx++;
        }
      }
      else
      {
        pcl::PointIndicesPtr &indices = clustersWithParts[i].partsOfClusters[0];
        for(unsigned int k = 0; k < indices->indices.size(); ++k)
        {
          int index = indices->indices[k];
          disp.at<cv::Vec3b>(index) = rs::common::cvVec3bColors[colorIdx];
        }
        colorIdx++;
      }
    }
  }



  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    switch(dispMode)
    {
    case DisplayMode::MERGED:
      {
        for(unsigned int i = 0; i < clustersWithParts.size(); ++i)
        {
          if(clustersWithParts[i].partsOfClusters.size() > 1)
          {
            for(unsigned int j = 0; j < clustersWithParts[i].partsOfClusters.size(); ++j)
            {
              pcl::PointIndicesPtr &indices = clustersWithParts[i].partsOfClusters[j];
              for(unsigned int k = 0; k < indices->indices.size(); ++k)
              {
                int index = indices->indices[k];
                cloudPtr_->points[index].rgba = rs::common::colors[j % rs::common::numberOfColors];
              }
            }
          }
        }
        break;
      }
    case DisplayMode::SUPERVOX:
      {
        for(unsigned int i = 0; i < clustersWithParts.size(); ++i)
        {
          for(int j = 0; j < clustersWithParts[i].labeledCloud->points.size(); j++)
          {
            //the beauty of unreadable code...in a nutshell points in the labeled cloud of each cluster
            // are in the same oreder as the indices thus allowing the coloring of the original
            // organized point cloud
            cloudPtr_->points[clustersWithParts[i].indices->indices[j]].rgba =
              rs::common::colors[clustersWithParts[i].labeledCloud->points[j].label % rs::common::numberOfColors];
          }
        }
        break;
      }
    }
    if(firstRun)
    {
      visualizer.addPointCloud(cloudPtr_, std::string("voxel_centroids"));
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, std::string("voxel_centroids"));
    }
    else
    {
      visualizer.updatePointCloud(cloudPtr_, std::string("voxel_centroids"));
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, std::string("voxel_centroids"));
    }
  }

};
MAKE_AE(ClusterToPartsSegmenter)
