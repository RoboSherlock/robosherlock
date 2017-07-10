#include <rs/io/DataLoaderBridge.h>

//PCL include
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/progress.hpp>
#include <boost/optional/optional.hpp>

#include <functional>
#include <iterator>
#include <mutex>
#include <sstream>


using namespace uima;

namespace fs = boost::filesystem;

DataLoaderBridge::DataLoaderBridge(const boost::property_tree::ptree& pt) : CamInterface(pt){
  isLoop = true;

  isCloudFile = true;
  isRGBFile = true;
  isDepthFile = true;

  haveCloud = false;
  haveRGB = false;
  haveDepth = false;

  iterator = 0;

  _newData = true;

  this->readConfig(pt);
}

DataLoaderBridge::~DataLoaderBridge() {}

//NOTE: check if all are files or all size of data are equal
bool DataLoaderBridge::checkConsistency(){
  if(haveCloud && haveRGB){
    if(isCloudFile && isRGBFile) return true;
    else if(!isCloudFile && !isRGBFile){
      if(clouds.size() == images.size())
        return true;
    }
    return false;
  }
  else if(haveCloud && haveDepth){
    if(isCloudFile && isDepthFile) return true;
    else if(!isCloudFile && !isDepthFile){
      if(clouds.size() == depths.size())
        return true;
    }
    return false;
  }
  else if(haveRGB && haveDepth){
    if(isRGBFile && isDepthFile) return true;
    else if(!isRGBFile && !isDepthFile){
      if(images.size() == depths.size())
        return true;
    }
    return false;
  }
  else if(haveCloud && haveRGB && haveDepth){
    if(isCloudFile && isRGBFile && isDepthFile) return true;
    else if(!isCloudFile && !isRGBFile && !isDepthFile){
      if(clouds.size() == images.size() && clouds.size() == depths.size())
        return true;
    }
    return false;
  }
  return true;
}

void DataLoaderBridge::getListFile(std::string& path, std::vector<std::string>& filenames, std::string& pattern, bool& isFile){
  fs::path full_path(fs::initial_path<fs::path>());
  full_path = fs::system_complete(fs::path(path));

  //check if path exists
  if(!fs::exists(full_path)){
    outError("Could not found path provided! Please check again");
  }
  //if it is a directory, find all relevant file
  if(fs::is_directory(full_path)){
    fs::directory_iterator end_iter;
    for(fs::directory_iterator dir_it(full_path); dir_it != end_iter; dir_it++){
      std::string currEntry = dir_it->path().string();
      size_t match = currEntry.find(pattern);
      if(match != std::string::npos){
        filenames.push_back(currEntry);
      }
    }

    isFile = false;
  }
  else
    isFile = true;
}


void DataLoaderBridge::readConfig(const boost::property_tree::ptree& pt){
  boost::optional< const boost::property_tree::ptree& > foundCloud;
  foundCloud = pt.get_child_optional( "data_path.path_to_cloud" );
  if(foundCloud)
  {
    this->path_to_cloud = pt.get<std::string>("data_path.path_to_cloud");
    std::string cloud_extension = pt.get<std::string>("data_path.cloud_extension");
    getListFile(path_to_cloud, clouds, cloud_extension, isCloudFile);
    std::sort(clouds.begin(), clouds.end());
    data_size = clouds.size();
    haveCloud = (data_size > 0);
  }

  boost::optional< const boost::property_tree::ptree& > foundRGB;
  foundRGB = pt.get_child_optional( "data_path.path_to_rgb" );
  if(foundRGB)
  {
    this->path_to_rgb = pt.get<std::string>("data_path.path_to_rgb");
    std::string rgb_extension = pt.get<std::string>("data_path.rgb_extension");
    getListFile(path_to_rgb, images, rgb_extension, isRGBFile);
    std::sort(images.begin(), images.end());
    data_size = images.size();
    haveRGB = (data_size > 0);
  }

  boost::optional< const boost::property_tree::ptree& > foundDepth;
  foundDepth = pt.get_child_optional( "data_path.path_to_depth" );
  if(foundDepth)
  {
    this->path_to_depth = pt.get<std::string>("data_path.path_to_depth");
    std::string depth_extension = pt.get<std::string>("data_path.depth_extension");
    getListFile(path_to_depth, depths, depth_extension, isDepthFile);
    std::sort(depths.begin(), depths.end());
    data_size = depths.size();
    haveDepth = (data_size > 0);
  }

  this->isLoop = pt.get<bool>("option.isLoop", true);

  if(!checkConsistency()){
    outError("Provided data is not consistent, all must be a file or size of each kind of data must be equal!");
    _newData = false;
  }

  this->frameRate = pt.get<double>("camera_info.frameRate", 1);
  this->cameraInfo.width = pt.get<int>("camera_info.width", 640);
  this->cameraInfo.height = pt.get<int>("camera_info.height", 480);

  this->cameraInfo.roi.width = this->cameraInfo.width;
  this->cameraInfo.roi.height = this->cameraInfo.height;
  this->cameraInfo.roi.x_offset = 0;
  this->cameraInfo.roi.y_offset = 0;

  std::string line = pt.get<std::string>("camera_info.matrix");
  std::replace(line.begin(), line.end(), ',', ' ');

  std::stringstream sstr(line);
  std::vector<double> cameraMatrix{std::istream_iterator<double>(sstr),
                                   std::istream_iterator<double>()};
  std::copy(cameraMatrix.begin(), cameraMatrix.end(), this->cameraInfo.K.begin());

  line = pt.get<std::string>("camera_info.distortion");
  std::replace(line.begin(), line.end(), ',', ' ');
  sstr.str(line);
  std::vector<double> cameraDistortion{std::istream_iterator<double>(sstr),
                                       std::istream_iterator<double>()};
  std::copy(cameraDistortion.begin(), cameraDistortion.end(), this->cameraInfo.D.begin());

  this->depth_scaling_factor = pt.get<int>("camera_info.depth_scaling_factor", 3000);
}

bool DataLoaderBridge::setData(uima::CAS &tcas, uint64_t ts){
  if(!newData())
    return false;

  rs::SceneCas cas(tcas);

  if(haveCloud){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);

    std::string path;
    if(!isCloudFile){
      path = clouds[iterator];
    }
    else{
      path = path_to_cloud;
    }

    if(pcl::io::loadPCDFile (path, *cloud_ptr) == -1){
      outError("Could not load point cloud file as PCD type. Check path again!");
    }

    cas.set(VIEW_CLOUD, *cloud_ptr);
  }

  auto imageSize = cv::Size(this->cameraInfo.width, this->cameraInfo.height);

  if(haveRGB){
    std::string path;
    if(!isRGBFile){
      path = images[iterator];
    }
    else{
      path = path_to_rgb;
    }

    this->color = cv::imread(path);
    cv::resize(color, color, imageSize, 0, 0, cv::INTER_NEAREST);
    cas.set(VIEW_COLOR_IMAGE, color);
  }

  if(haveDepth){
    std::string path;
    if(!isDepthFile){
      path = depths[iterator];
    }
    else{
      path = path_to_depth;
    }

    this->depth = cv::imread(path,  CV_LOAD_IMAGE_ANYDEPTH);
    cv::resize(depth, depth, imageSize, 0, 0, cv::INTER_NEAREST);

    if (depth.type() == CV_16UC1)
      depth.convertTo(depth, CV_16UC1, this->depth_scaling_factor/65535, 0);
    else
      depth.convertTo(depth, CV_16UC1, this->depth_scaling_factor/255, 0);

    cas.set(VIEW_DEPTH_IMAGE, depth);
  }

  cas.set(VIEW_CAMERA_INFO, cameraInfo);

  iterator++;
  if(iterator > data_size - 1){
    if(isLoop)
      iterator = 0;
    else
      _newData = false;
  }

  return true;
}
