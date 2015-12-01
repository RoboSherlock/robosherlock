// ROS
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>

// RS
#include <rs/conversion/conversion.h>
#include <rs/types/ros_types.h>

namespace rs
{
namespace conversion
{

template<>
void from(const uima::FeatureStructure &fs, std_msgs::Header &output)
{
  rs::Header h(fs);

  output.frame_id = h.frame_id();
  output.stamp.fromNSec(h.stamp());
  output.seq = h.seq();
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const std_msgs::Header &input)
{
  rs::Header h = rs::create<rs::Header>(cas);

  h.frame_id(input.frame_id);
  h.stamp(input.stamp.toNSec());
  h.seq(input.seq);

  return h;
}

template<>
void from(const uima::FeatureStructure &fs, sensor_msgs::CameraInfo::_roi_type &output)
{
  rs::ROI roi(fs);

  output.height = (uint32_t)roi.height.get();
  output.width = (uint32_t)roi.width.get();
  output.x_offset = (uint32_t)roi.x_offset.get();
  output.y_offset = (uint32_t)roi.y_offset.get();
  output.do_rectify = (uint8_t)roi.do_rectify.get();
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const sensor_msgs::CameraInfo::_roi_type &input)
{
  rs::ROI roi = rs::create<rs::ROI>(cas);

  roi.height.set((int)input.height);
  roi.width.set((int)input.width);
  roi.x_offset.set((int)input.x_offset);
  roi.y_offset.set((int)input.y_offset);
  roi.do_rectify.set((bool)input.do_rectify);

  return roi;
}

template<>
void from(const uima::FeatureStructure &fs, sensor_msgs::CameraInfo &output)
{
  std::vector<double> vec;
  std::string tmp;

  rs::CameraInfo cam(fs);

  from(cam.header.get(), output.header);

  tmp = cam.distortion_model.get();
  output.distortion_model.resize(tmp.size());
  for(int i = 0; i < tmp.size(); ++i)
  {
    output.distortion_model[i] = tmp[i];
  }

  output.height = (uint32_t)cam.height.get();
  output.width = (uint32_t)cam.width.get();

  output.D = cam.d();

  vec = cam.r.get();
  for(int i = 0; i < output.R.size() && i < vec.size(); ++i)
  {
    output.R[i] = vec[i];
  }

  vec = cam.p.get();
  for(int i = 0; i < output.P.size() && i < vec.size(); ++i)
  {
    output.P[i] = vec[i];
  }

  vec = cam.k.get();
  for(int i = 0; i < output.K.size() && i < vec.size(); ++i)
  {
    output.K[i] = vec[i];
  }

  output.binning_x = (uint32_t)cam.binning_x.get();
  output.binning_y = (uint32_t)cam.binning_y.get();

  from(cam.roi.get(), output.roi);
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const sensor_msgs::CameraInfo &input)
{
  std::vector<double> vec;
  std::string tmp;

  rs::CameraInfo cam = rs::create<rs::CameraInfo>(cas);

  cam.header.set(to(cas, input.header));

  tmp.resize(input.distortion_model.size());
  for(int i = 0; i < tmp.size(); ++i)
  {
    tmp[i] = input.distortion_model[i];
  }
  cam.distortion_model.set(tmp);

  cam.height.set((int)input.height);
  cam.width.set((int)input.width);

  cam.d.set(input.D);

  vec.resize(input.R.size());
  for(int i = 0; i < input.R.size(); ++i)
  {
    vec[i] = input.R[i];
  }
  cam.r.set(vec);

  vec.resize(input.P.size());
  for(int i = 0; i < input.P.size(); ++i)
  {
    vec[i] = input.P[i];
  }
  cam.p.set(vec);

  vec.resize(input.K.size());
  for(int i = 0; i < input.K.size(); ++i)
  {
    vec[i] = input.K[i];
  }
  cam.k.set(vec);

  cam.binning_x.set((int)input.binning_x);
  cam.binning_y.set((int)input.binning_y);

  cam.roi.set(to(cas, input.roi));

  return cam;
}

}
}
