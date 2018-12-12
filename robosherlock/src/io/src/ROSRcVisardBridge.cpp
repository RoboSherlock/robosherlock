/* 
* Roboception GmbH 
* Munich, Germany 
* www.roboception.com 
* 
* Copyright (c) 2018 Roboception GmbH 
* All rights reserved 
* 
* Author: Raphael Schaller
*/


// RS
#include <rs/io/ROSRcVisardBridge.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>

//OpenCV
#include <cv_bridge/cv_bridge.h>

constexpr std::array<bool, ROSRcVisardBridge::NUM_TOPICS> ROSRcVisardBridge::exposure_alternate_topics_first_;
constexpr std::array<bool, ROSRcVisardBridge::NUM_TOPICS> ROSRcVisardBridge::exposure_alternate_topics_second_;

ROSRcVisardBridge::ROSRcVisardBridge(const boost::property_tree::ptree &pt) :
    ROSCamInterface(pt),
    current_projector_mode_(ProjectorMode::EXPOSURE_ALTERNATE)
{
  readConfig(pt);
  initSpinner();
}

ROSRcVisardBridge::~ROSRcVisardBridge()
{
  spinner.stop();
}

void ROSRcVisardBridge::readConfig(const boost::property_tree::ptree &pt)
{
  max_queue_size_ = pt.get<int>("sync.max_queue_size");

  const std::string left_rect_topic = pt.get<std::string>("camera_topics.left_rect", "");
  const std::string right_rect_topic = pt.get<std::string>("camera_topics.right_rect", "");
  const std::string left_cam_info_topic = pt.get<std::string>("camera_topics.left_cam_info", "");
  const std::string right_cam_info_topic = pt.get<std::string>("camera_topics.right_cam_info", "");
  const std::string depth_topic = pt.get<std::string>("camera_topics.depth", "");
  const std::string disparity_error_topic = pt.get<std::string>("camera_topics.disparity_error", "");
  const std::string confidence_topic = pt.get<std::string>("camera_topics.confidence", "");

  if (!left_rect_topic.empty())
  {
    subs_[LEFT_RECT_POS] = nodeHandle.subscribe(left_rect_topic, 10, &ROSRcVisardBridge::cb<sensor_msgs::Image, LEFT_RECT_POS>, this);
  }
  if (!right_rect_topic.empty())
  {
    subs_[RIGHT_RECT_POS] = nodeHandle.subscribe(right_rect_topic, 10, &ROSRcVisardBridge::cb<sensor_msgs::Image, RIGHT_RECT_POS>, this);
  }
  if (!left_cam_info_topic.empty())
  {
    subs_[LEFT_CAM_INFO_POS] = nodeHandle.subscribe(left_cam_info_topic, 10, &ROSRcVisardBridge::cb<sensor_msgs::CameraInfo, LEFT_CAM_INFO_POS>, this);
  }
  if (!right_cam_info_topic.empty())
  {
    subs_[RIGHT_CAM_INFO_POS] = nodeHandle.subscribe(right_cam_info_topic, 10, &ROSRcVisardBridge::cb<sensor_msgs::CameraInfo, RIGHT_CAM_INFO_POS>, this);
  }
  if (!depth_topic.empty())
  {
    subs_[DEPTH_POS] = nodeHandle.subscribe(depth_topic, 10, &ROSRcVisardBridge::cb<sensor_msgs::Image, DEPTH_POS>, this);
  }
  if (!disparity_error_topic.empty())
  {
    subs_[DISPARITY_ERROR_POS] = nodeHandle.subscribe(disparity_error_topic, 10, &ROSRcVisardBridge::cb<sensor_msgs::Image, DISPARITY_ERROR_POS>, this);
  }
  if (!confidence_topic.empty())
  {
    subs_[CONFIDENCE_POS] = nodeHandle.subscribe(confidence_topic, 10, &ROSRcVisardBridge::cb<sensor_msgs::Image, CONFIDENCE_POS>, this);
  }

  fillSubscriberBitMaps();

  outInfo("      Left rect topic: " FG_BLUE << left_rect_topic);
  outInfo("     Right rect topic: " FG_BLUE << right_rect_topic);
  outInfo("  Left cam info topic: " FG_BLUE << left_cam_info_topic);
  outInfo(" Right cam info topic: " FG_BLUE << right_cam_info_topic);
  outInfo("          Depth topic: " FG_BLUE << depth_topic);
  outInfo("Disparity error topic: " FG_BLUE << disparity_error_topic);
  outInfo("     Confidence topic: " FG_BLUE << confidence_topic);
}

template<std::size_t N>
static std::array<bool, N> operator|(const std::array<bool, N> &lhs,
                                     const std::array<bool, N> &rhs)
{
  std::array<bool, N> res;
  for (std::size_t i = 0; i < N; ++i)
  {
    res[i] = lhs[i] || rhs[i];
  }
  return res;
}

template<std::size_t N>
static std::array<bool, N> operator&(const std::array<bool, N> &lhs,
                                     const std::array<bool, N> &rhs)
{
  std::array<bool, N> res;
  for (std::size_t i = 0; i < N; ++i)
  {
    res[i] = lhs[i] && rhs[i];
  }
  return res;
}

template<std::size_t N>
static std::array<bool, N> operator!(const std::array<bool, N> &rhs)
{
  std::array<bool, N> res;
  for (std::size_t i = 0; i < N; ++i)
  {
    res[i] = !rhs[i];
  }
  return res;
}

template<std::size_t N>
static bool any(const std::array<bool, N> &a)
{
  return std::any_of(std::begin(a), std::end(a), [](bool v){return v;});
}

void ROSRcVisardBridge::fillSubscriberBitMaps()
{
  std::array<bool, NUM_TOPICS> sub_bit_map;
  for (std::size_t i = 0; i < NUM_TOPICS; ++i)
  {
    sub_bit_map[i] = static_cast<bool>(subs_[i]);
  }
  if (current_projector_mode_ == ProjectorMode::DEFAULT)
  {
    sub_bit_map_first_ = sub_bit_map;
    sub_bit_map_second_.fill(false);
  }
  else
  {
    sub_bit_map_first_ = sub_bit_map & exposure_alternate_topics_first_;
    sub_bit_map_second_ = sub_bit_map & exposure_alternate_topics_second_;
  }
}

void ROSRcVisardBridge::initSpinner()
{
  spinner.start();
}

template<std::size_t I = 0, typename FuncT, typename... Tp>
static inline typename std::enable_if<I == sizeof...(Tp), void>::type
for_each(const std::tuple<Tp...> &, FuncT &)
{ }

template<std::size_t I = 0, typename FuncT, typename... Tp>
static inline typename std::enable_if<I < sizeof...(Tp), void>::type
for_each(const std::tuple<Tp...>& t, FuncT &f)
{
  f(std::get<I>(t), I);
  for_each<I + 1, FuncT, Tp...>(t, f);
}

template<std::size_t I = 0, typename FuncT, typename... Tp>
static inline typename std::enable_if<I == sizeof...(Tp), void>::type
merge(std::tuple<Tp...> &, const std::tuple<Tp...> &, FuncT &)
{ }

template<std::size_t I = 0, typename FuncT, typename... Tp>
static inline typename std::enable_if<I < sizeof...(Tp), void>::type
merge(std::tuple<Tp...>& lhs, const std::tuple<Tp...> &rhs, FuncT &f)
{
  f(std::get<I>(lhs), std::get<I>(rhs), I);
  merge<I + 1, FuncT, Tp...>(lhs, rhs, f);
}

template<int bit_map_size>
struct TupleCompleteChecker
{
  TupleCompleteChecker(const std::array<bool, bit_map_size> &bit_map) :
      bit_map_(bit_map)
  { }

  const std::array<bool, bit_map_size> &bit_map_;
  bool complete = true;

  template<typename T>
  void operator()(const boost::shared_ptr<T> &img, int i)
  {
    complete &= static_cast<bool>(!(bit_map_[i]) || img);
  }
};

struct DataSetMerger
{
  template<typename T>
  void operator()(boost::shared_ptr<T> &lhs, const boost::shared_ptr<T> &rhs, int i)
  {
    if (!lhs)
    {
      lhs = rhs;
    }
  }
};

template<std::size_t bit_map_size>
static std::string to_string(const std::array<bool, bit_map_size> &bit_map)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < bit_map_size; ++i)
  {
    oss << static_cast<bool>(bit_map[i]);
  }
  return oss.str();
}

struct DataSetPrinter
{
  DataSetPrinter(std::ostringstream &oss) : oss_(oss) {}

  template<typename T>
  void operator()(const boost::shared_ptr<T> &lhs, int i)
  {
    oss_ << static_cast<bool>(lhs);
  }

  std::ostringstream &oss_;
};

template<typename... Tp>
std::string to_string(const std::tuple<boost::shared_ptr<Tp>...> &tp)
{
  std::ostringstream oss;
  DataSetPrinter printer(oss);
  for_each(tp, printer);
  return oss.str();
}

template<typename T, int tuple_pos>
void ROSRcVisardBridge::cb(const boost::shared_ptr<const T> &msg)
{
  std::lock_guard<std::mutex> cb_lock(cb_mutex_);

  auto data_queue_it_first = std::end(data_queue_);
  auto data_queue_it_second = std::end(data_queue_);

  {
    auto data_set_it = data_queue_.find(msg->header.stamp);
    if (data_set_it == data_queue_.end())
    {
      std::array<bool, NUM_TOPICS> a;
      a.fill(false);
      std::tie(data_set_it, std::ignore) = data_queue_.emplace(msg->header.stamp, DataSet{});
    }
    auto &data_set = data_set_it->second;
    auto &data = std::get<tuple_pos>(data_set);
    if (data)
    {
      outError("Got data twice");
      return;
    }

    data = msg;

    const auto is_complete =
        [](const DataSet &data_set, const std::array<bool, NUM_TOPICS> &bit_map)
    {
      TupleCompleteChecker<NUM_TOPICS> complete_checker(bit_map);
      for_each(data_set, complete_checker);
      return complete_checker.complete;
    };

    const bool sub_bit_map_first_set = any(sub_bit_map_first_);
    const bool sub_bit_map_second_set = any(sub_bit_map_second_);
    const bool data_set_first_complete = is_complete(data_set, sub_bit_map_first_);
    const bool data_set_second_complete = is_complete(data_set, sub_bit_map_second_);

    if (sub_bit_map_first_set || sub_bit_map_second_set)
    {
      if (sub_bit_map_first_set ^ sub_bit_map_second_set)
      {
        if (sub_bit_map_first_set && data_set_first_complete)
        {
          data_queue_it_first = data_set_it;
          data_queue_it_second = data_set_it;
        }
        else if (sub_bit_map_second_set && data_set_second_complete)
        {
          data_queue_it_first = data_set_it;
          data_queue_it_second = data_set_it;
        }
      }
      else
      {
        const auto data_set_it_prev = (data_set_it == std::begin(data_queue_)) ? std::end(data_queue_) : std::prev(data_set_it);
        const auto data_set_it_next = std::next(data_set_it);

//        outInfo("Waiting for " + to_string(sub_bit_map_first_) + " and " + to_string(sub_bit_map_second_));
//        outInfo("Got " + (data_set_it_prev != std::end(data_queue_) ? to_string(data_set_it_prev->second) : std::string()) + ", " + to_string(data_set) + " and " + (data_set_it_next != std::end(data_queue_) ? to_string(data_set_it_next->second) : std::string()));

        const bool data_set_it_prev_complete =
            data_set_it_prev != std::end(data_queue_) &&
            is_complete(data_set_it_prev->second, sub_bit_map_first_);
        const bool data_set_it_next_complete =
            data_set_it_next != std::end(data_queue_) &&
            is_complete(data_set_it_next->second, sub_bit_map_second_);

        if (data_set_first_complete && data_set_it_next_complete)
        {
//          outInfo("Taking current and next");
          data_queue_it_first = data_set_it;
          data_queue_it_second = data_set_it_next;
        }
        else if (data_set_it_prev_complete && data_set_second_complete)
        {
//          outInfo("Taking prev and current");
          data_queue_it_first = data_set_it_prev;
          data_queue_it_second = data_set_it;
        }
      }
    }
  }

  if (data_queue_it_first != std::end(data_queue_))
  {
//    outInfo("Got data");

    const auto &stamp = data_queue_it_first->first;
    DataSet data_set = data_queue_it_first->second;
    if (data_queue_it_second != data_queue_it_first)
    {
      DataSetMerger merger;
      merge(data_set, data_queue_it_second->second, merger);
    }

    data_queue_.erase(data_queue_.begin(), std::next(data_queue_it_second));

    if (lookupTransform(data_queue_it_first->first))
    {
      std::lock_guard<std::mutex> scoped_lock(lock);
      _newData = true;
      current_data_set_ = std::move(data_set);
      timestamp = stamp;
    }
  }

  if (data_queue_.size() > max_queue_size_)
  {
//    outInfo("Pruning queue");

    const int num_to_delete = data_queue_.size() - data_queue_.size();
    data_queue_.erase(data_queue_.begin(),
                      std::next(data_queue_.begin(), num_to_delete));
  }
}

bool ROSRcVisardBridge::setData(uima::CAS &tcas, uint64_t ts)
{
  if (!newData())
  {
    return false;
  }

  rs::SceneCas cas(tcas);
  DataSet set;
  {
    std::lock_guard<std::mutex> scoped_lock(lock);
    _newData = false;
    set = std::move(current_data_set_);
    setTransformAndTime(tcas);
  }

  const auto &left_rect = std::get<LEFT_RECT_POS>(set);
  const auto &right_rect = std::get<RIGHT_RECT_POS>(set);
  const auto &left_cam_info = std::get<LEFT_CAM_INFO_POS>(set);
  const auto &right_cam_info = std::get<RIGHT_CAM_INFO_POS>(set);
  const auto &depth = std::get<DEPTH_POS>(set);
  const auto &disparity_error = std::get<DISPARITY_ERROR_POS>(set);
  const auto &confidence = std::get<CONFIDENCE_POS>(set);

  const auto check_img = [](const cv_bridge::CvImageConstPtr &img, int cols,
                            int rows, const std::string &name) -> bool
  {
    if (!img)
    {
      outError("Could not convert " + name + " to cv::Mat");
      return false;
    }
    if (img->image.rows != rows || img->image.cols != cols)
    {
      outError("Dimensions of " + name + " are wrong. Got " +
               std::to_string(img->image.cols) + "x" +
               std::to_string(img->image.rows) + ", expecting " +
               std::to_string(cols) + "x" + std::to_string(rows));
      return false;
    }
    return true;
  };

  cv_bridge::CvImageConstPtr left_rect_cv;
  cv_bridge::CvImageConstPtr right_rect_cv;
  cv_bridge::CvImageConstPtr depth_cv;
  cv_bridge::CvImageConstPtr disparity_error_cv;
  cv_bridge::CvImageConstPtr confidence_cv;
  if (left_rect)
  {
    if (sensor_msgs::image_encodings::isColor(left_rect->encoding))
    {
      left_rect_cv = cv_bridge::toCvShare(left_rect,
                                          sensor_msgs::image_encodings::BGR8);
    }
    else
    {
      left_rect_cv = cv_bridge::toCvShare(left_rect,
                                          sensor_msgs::image_encodings::MONO8);
    }
    if (!check_img(left_rect_cv, 1280, 960, "left rect")) { return false; }
  }
  if (right_rect)
  {
    if (sensor_msgs::image_encodings::isColor(right_rect->encoding))
    {
      right_rect_cv = cv_bridge::toCvShare(right_rect,
                                           sensor_msgs::image_encodings::BGR8);
    }
    else
    {
      right_rect_cv = cv_bridge::toCvShare(right_rect,
                                           sensor_msgs::image_encodings::MONO8);
    }
    if (!check_img(right_rect_cv, 1280, 960, "right rect")) { return false; }
  }
  if (depth)
  {
    depth_cv = cv_bridge::toCvShare(depth,
                                    sensor_msgs::image_encodings::TYPE_32FC1);
    if (!check_img(depth_cv, 640, 480, "depth")) { return false; }
  }
  if (disparity_error)
  {
    disparity_error_cv = cv_bridge::toCvShare(disparity_error,
                                              sensor_msgs::image_encodings::TYPE_32FC1);
    if (!check_img(disparity_error_cv, 640, 480, "disparity error"))
    { return false; }
  }
  if (confidence)
  {
    confidence_cv = cv_bridge::toCvShare(confidence,
                                         sensor_msgs::image_encodings::TYPE_32FC1);
    if (!check_img(confidence_cv, 640, 480, "confidence")) { return false; }
  }

  const auto to_cam_info_sd = [](const sensor_msgs::CameraInfo &cam_info_hd)
  {
    sensor_msgs::CameraInfo cam_info_sd = cam_info_hd;
    cam_info_sd.width = cam_info_hd.width / 2;
    cam_info_sd.height = cam_info_hd.height / 2;
    cam_info_sd.P[0] = cam_info_hd.P[0] / 2;
    cam_info_sd.P[2] = cam_info_hd.P[2] / 2;
    cam_info_sd.P[5] = cam_info_hd.P[5] / 2;
    cam_info_sd.P[6] = cam_info_hd.P[6] / 2;
    cam_info_sd.K[0] = cam_info_hd.K[0] / 2;
    cam_info_sd.K[2] = cam_info_hd.K[2] / 2;
    cam_info_sd.K[4] = cam_info_hd.K[4] / 2;
    cam_info_sd.K[5] = cam_info_hd.K[5] / 2;
    return cam_info_sd;
  };

  if (left_cam_info)
  {
    cas.set(VIEW_CAMERA_INFO_HD, *left_cam_info);
    cas.set(VIEW_CAMERA_INFO, to_cam_info_sd(*left_cam_info));
  }
  if (right_cam_info)
  {
    cas.set(VIEW_CAMERA_INFO_RIGHT_HD, *right_cam_info);
    cas.set(VIEW_CAMERA_INFO_RIGHT, to_cam_info_sd(*right_cam_info));
  }

  const auto to_color = [](const cv::Mat &img)
  {
    cv::Mat img_c;
    if (img_c.channels() == 1) { cv::cvtColor(img, img_c, CV_GRAY2BGR); }
    else { img_c = img; }
    return img_c;
  };

  if (left_rect_cv) { cas.set(VIEW_COLOR_IMAGE_HD, to_color(left_rect_cv->image)); }
  if (right_rect_cv) { cas.set(VIEW_COLOR_IMAGE_RIGHT_HD, to_color(right_rect_cv->image)); }
  if (depth_cv) {
    cv::Mat depth = depth_cv->image.clone();
    depth.setTo(0, depth != depth);

    // robosherlock expects depth image as uint16
    cv::Mat depth_uint16;
    depth.convertTo(depth_uint16, CV_16UC1, 1000, 0);
    cas.set(VIEW_DEPTH_IMAGE, depth_uint16);
  }
  if (disparity_error_cv) { cas.set(VIEW_DISPARITY_ERROR, disparity_error_cv->image); }
  if (confidence_cv) { cas.set(VIEW_CONFIDENCE, confidence_cv->image); }

  return true;
}
