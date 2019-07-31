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

// STL
#include <cstring>
#include <stdlib.h>

// OpenCV
#include <opencv2/opencv.hpp>

// RS
#include <rs/conversion/conversion.h>
#include <rs/types/cv_types.h>

namespace rs
{
namespace conversion
{

template<>
void from(const uima::FeatureStructure &fs, cv::Point &output)
{
  rs::Point p(fs);
  output.x = p.x();
  output.y = p.y();
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const cv::Point &input)
{
  rs::Point p = rs::create<rs::Point>(cas);
  p.x(input.x);
  p.y(input.y);
  return p;
}

template<>
void from(const uima::FeatureStructure &fs, cv::Point2f &output)
{
  rs::Point2f p(fs);
  output.x = p.x();
  output.y = p.y();
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const cv::Point2f &input)
{
  rs::Point2f p = rs::create<rs::Point2f>(cas);
  p.x(input.x);
  p.y(input.y);
  return p;
}

template<>
void from(const uima::FeatureStructure &fs, cv::Point2d &output)
{
  rs::Point2d p(fs);
  output.x = p.x();
  output.y = p.y();
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const cv::Point2d &input)
{
  rs::Point2d p = rs::create<rs::Point2d>(cas);
  p.x(input.x);
  p.y(input.y);
  return p;
}

template<>
void from(const uima::FeatureStructure &fs, cv::Point3f &output)
{
  rs::Point3f p(fs);
  output.x = p.x();
  output.y = p.y();
  output.z = p.z();
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const cv::Point3f &input)
{
  rs::Point3f p = rs::create<rs::Point3f>(cas);
  p.x(input.x);
  p.y(input.y);
  p.z(input.z);
  return p;
}

template<>
void from(const uima::FeatureStructure &fs, cv::Point3d &output)
{
  rs::Point3d p(fs);
  output.x = p.x();
  output.y = p.y();
  output.z = p.z();
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const cv::Point3d &input)
{
  rs::Point3d p = rs::create<rs::Point3d>(cas);
  p.x(input.x);
  p.y(input.y);
  p.z(input.z);
  return p;
}

template<>
void from(const uima::FeatureStructure &fs, cv::Size &output)
{
  rs::Size s(fs);
  output.width = s.width();
  output.height = s.height();
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const cv::Size &input)
{
  rs::Size s = rs::create<rs::Size>(cas);
  s.width(input.width);
  s.height(input.height);
  return s;
}

template<>
void from(const uima::FeatureStructure &fs, cv::Size2f &output)
{
  rs::Size2f s(fs);
  output.width = s.width();
  output.height = s.height();
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const cv::Size2f &input)
{
  rs::Size2f s = rs::create<rs::Size2f>(cas);
  s.width(input.width);
  s.height(input.height);
  return s;
}

template<>
void from(const uima::FeatureStructure &fs, cv::Rect &output)
{
  rs::Rect rect(fs);
  cv::Size size;
  cv::Point point;
  from(rect.size(), size);
  from(rect.pos(), point);

  output = cv::Rect(point, size);
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const cv::Rect &input)
{
  rs::Rect rect = rs::create<rs::Rect>(cas);
  rect.pos(to(cas, input.tl()));
  rect.size(to(cas, input.size()));
  return rect;
}

template<>
void from(const uima::FeatureStructure &fs, cv::RotatedRect &output)
{
  rs::RotatedRect rect(fs);
  cv::Size2f size;
  cv::Point2f center;
  from(rect.size(), size);
  from(rect.center(), center);
  output = cv::RotatedRect(center, size, rect.angle());
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const cv::RotatedRect &input)
{
  rs::RotatedRect rect = rs::create<rs::RotatedRect>(cas);
  rect.center(to(cas, input.center));
  rect.size(to(cas, input.size));
  rect.angle(input.angle);
  return rect;
}

template<>
void from(const uima::FeatureStructure &fs, cv::Vec3f &output)
{
  rs::Vec3f vec(fs);
  output.val[0] = vec.val.get(0);
  output.val[1] = vec.val.get(1);
  output.val[2] = vec.val.get(2);
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const cv::Vec3f &input)
{
  rs::Vec3f vec = rs::create<rs::Vec3f>(cas);
  vec.val.allocate(3);
  vec.val.set(0, input.val[0]);
  vec.val.set(1, input.val[1]);
  vec.val.set(2, input.val[2]);
  return vec;
}

template<>
void from(const uima::FeatureStructure &fs, cv::Vec4f &output)
{
  rs::Vec4f vec(fs);
  output.val[0] = vec.val.get(0);
  output.val[1] = vec.val.get(1);
  output.val[2] = vec.val.get(2);
  output.val[3] = vec.val.get(3);
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const cv::Vec4f &input)
{
  rs::Vec4f vec = rs::create<rs::Vec4f>(cas);
  vec.val.allocate(4);
  vec.val.set(0, input.val[0]);
  vec.val.set(1, input.val[1]);
  vec.val.set(2, input.val[2]);
  vec.val.set(3, input.val[3]);
  return vec;
}

template<>
void from(const uima::FeatureStructure &fs, cv::KeyPoint &output)
{
  rs::KeyPoint pt(fs);
  output = cv::KeyPoint(cv::Point2f(pt.pt().x(), pt.pt().y()), pt.size(), pt.angle(), pt.response(), pt.octave(), pt.class_id());
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const cv::KeyPoint &input)
{
  rs::KeyPoint pt = rs::create<rs::KeyPoint>(cas);
  pt.pt.set(to(cas, input.pt));
  pt.size(input.size);
  pt.angle(input.angle);
  pt.response(input.response);
  pt.octave(input.octave);
  pt.class_id(input.class_id);
  return pt;
}

template<>
void from(const uima::FeatureStructure &fs, cv::Mat &output)
{
  rs::Mat mat(fs);
  output.create(mat.rows(), mat.cols(), mat.mat_type());

  if(output.rows != 0 && output.cols != 0)
  {
    const uima::Feature &feature = fs.getType().getFeatureByBaseName("data");
    const uima::ByteArrayFS &arrayFS = fs.getByteArrayFSValue(feature);
    arrayFS.copyToArray(0, (char *)output.data, 0, arrayFS.size());
  }
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const cv::Mat &input)
{
  rs::Mat mat = rs::create<rs::Mat>(cas);
  size_t size = input.rows * input.cols * input.elemSize();

  if(!input.empty())
  {
    uima::FeatureStructure &fs = mat;
    const uima::Feature &feature = fs.getType().getFeatureByBaseName("data");
    uima::ByteArrayFS arrayFS = cas.createByteArrayFS(size);
    if(input.isContinuous())
    {
      arrayFS.copyFromArray((char *)input.data, 0, size, 0);
      //std::memcpy(&data[0], input.data, size);
    }
    else
    {
      size_t sizeRow = input.cols * input.elemSize();
      for(int r = 0, start = 0; r < input.rows; ++r, start += sizeRow)
      {
        arrayFS.copyFromArray((char *)input.ptr(r), 0, sizeRow, start);
        //std::memcpy(ptr + start, input.ptr(r), sizeRow);
      }
    }
    fs.setFSValue(feature, arrayFS);
    //mat.data(data);
  }

  mat.rows(input.rows);
  mat.cols(input.cols);
  mat.mat_type(input.type());
  return mat;
}

template<>
void from(const uima::FeatureStructure &fs, cv::Moments &output)
{
  rs::Moments m(fs);
  output = cv::Moments();
  output.m00 = m.m00();
  output.m01 = m.m01();
  output.m02 = m.m02();
  output.m03 = m.m03();
  output.m10 = m.m10();
  output.m11 = m.m11();
  output.m12 = m.m12();
  output.m20 = m.m20();
  output.m21 = m.m21();
  output.m30 = m.m30();
  output.mu02 = m.mu02();
  output.mu03 = m.mu03();
  output.mu11 = m.mu11();
  output.mu12 = m.mu12();
  output.mu20 = m.mu20();
  output.mu21 = m.mu21();
  output.mu30 = m.mu30();
  output.nu02 = m.nu02();
  output.nu03 = m.nu03();
  output.nu11 = m.nu11();
  output.nu12 = m.nu12();
  output.nu20 = m.nu20();
  output.nu21 = m.nu21();
  output.nu30 = m.nu30();
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const cv::Moments &input)
{
  rs::Moments m = rs::create<rs::Moments>(cas);
  m.m00(input.m00);
  m.m01(input.m01);
  m.m02(input.m02);
  m.m03(input.m03);
  m.m10(input.m10);
  m.m11(input.m11);
  m.m12(input.m12);
  m.m20(input.m20);
  m.m21(input.m21);
  m.m30(input.m30);
  m.mu02(input.mu02);
  m.mu03(input.mu03);
  m.mu11(input.mu11);
  m.mu12(input.mu12);
  m.mu20(input.mu20);
  m.mu21(input.mu21);
  m.mu30(input.mu30);
  m.nu02(input.nu02);
  m.nu03(input.nu03);
  m.nu11(input.nu11);
  m.nu12(input.nu12);
  m.nu20(input.nu20);
  m.nu21(input.nu21);
  m.nu30(input.nu30);
  return m;
}

template<>
void from(const uima::FeatureStructure &fs, std::map<std::string, cv::Vec3b> &objectMap)
{
  rs::ObjectMap map(fs);
  const std::vector<std::string> &names = map.objectNames.get();
  const std::vector<int> &colors = map.objectColors.get();

  for(size_t i = 0; i < map.objectNames.size(); ++i)
  {
    const int32_t color = colors[i];
    cv::Vec3b cvColor;
    cvColor.val[0] = color & 0xFF;
    cvColor.val[1] = (color & 0xFF00) >> 8;
    cvColor.val[2] = (color & 0xFF0000) >> 16;
    objectMap[names[i]] = cvColor;
  }
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const std::map<std::string, cv::Vec3b> &objectMap)
{
  rs::ObjectMap map = rs::create<rs::ObjectMap>(cas);
  std::map<std::string, cv::Vec3b>::const_iterator it = objectMap.begin();
  std::map<std::string, cv::Vec3b>::const_iterator end = objectMap.end();

  for(; it != end; ++it)
  {
    cv::Vec3b cvColor = it->second;
    const int32_t color = cvColor.val[0] | (cvColor.val[1] << 8) | (cvColor.val[2] << 16);

    map.objectNames.append(it->first);
    map.objectColors.append(color);
  }
  return map;
}

}
}
