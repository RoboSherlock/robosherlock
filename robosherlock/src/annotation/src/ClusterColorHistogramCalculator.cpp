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

#include <uima/api.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>

#include <rapidjson/document.h>

#define DEBUG_OUTPUT 0
#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_INFO

using namespace uima;

class ClusterColorHistogramCalculator : public DrawingAnnotator
{
private:
  enum COLORS
  {
    RED = 0,
    YELLOW,
    GREEN,
    CYAN,
    BLUE,
    MAGENTA,
    WHITE,
    BLACK,
    GREY,
    COUNT
  };
  int min_value_color_;
  int min_saturation_color_;
  int max_value_black_;
  int min_value_white_;
  int histogram_cols_;
  int histogram_rows_;

  const double color_range_;
  std::vector<int> color_positions_;

  std::vector<cv::Scalar> colors_;

  std::vector<std::string> color_names_;
  std::vector<cv::Rect> cluster_rois_;
  std::vector<std::vector<int>> color_ids_;
  std::vector<std::vector<float>> color_ratios_;

  cv::Mat color_mat_;

public:
  ClusterColorHistogramCalculator() : DrawingAnnotator(__func__), min_value_color_(60), min_saturation_color_(60), max_value_black_(60), min_value_white_(120), histogram_cols_(16), histogram_rows_(16), color_range_(256.0 / 6.0)
  {
    color_positions_.resize(6);
    for(size_t i = 0; i < 6; ++i)
    {
      color_positions_[i] = (int)(i * color_range_ + color_range_ / 2.0 + 0.5);
    }

    color_names_.resize(COUNT);
    color_names_[RED]     = "red";
    color_names_[YELLOW]  = "yellow";
    color_names_[GREEN]   = "green";
    color_names_[CYAN]    = "cyan";
    color_names_[BLUE]    = "blue";
    color_names_[MAGENTA] = "magenta";
    color_names_[WHITE]   = "white";
    color_names_[BLACK]   = "black";
    color_names_[GREY]    = "grey";

    colors_.resize(COUNT);
    colors_[RED]     = CV_RGB(255, 0, 0);
    colors_[YELLOW]  = CV_RGB(255, 255, 0);
    colors_[GREEN]   = CV_RGB(0, 255, 0);
    colors_[CYAN]    = CV_RGB(0, 255, 255);
    colors_[BLUE]    = CV_RGB(0, 0, 255);
    colors_[MAGENTA] = CV_RGB(255, 0, 255);
    colors_[WHITE]   = CV_RGB(255, 255, 255);
    colors_[BLACK]   = CV_RGB(0, 0, 0);
    colors_[GREY]    = CV_RGB(127, 127, 127);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    if(ctx.isParameterDefined("minValueColor"))
    {
      ctx.extractValue("minValueColor", min_value_color_);
    }
    if(ctx.isParameterDefined("minSaturationColor"))
    {
      ctx.extractValue("minSaturationColor", min_saturation_color_);
    }
    if(ctx.isParameterDefined("maxValueBlack"))
    {
      ctx.extractValue("maxValueBlack", max_value_black_);
    }
    if(ctx.isParameterDefined("minValueWhite"))
    {
      ctx.extractValue("minValueWhite", min_value_white_);
    }
    if(ctx.isParameterDefined("histogramCols"))
    {
      ctx.extractValue("histogramCols", histogram_cols_);
    }
    if(ctx.isParameterDefined("histogramRows"))
    {
      ctx.extractValue("histogramRows", histogram_rows_);
    }
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    return UIMA_ERR_NONE;
  }

private:
  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process start");
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    std::vector<rs::ObjectHypothesis> clusters;

    cas.get(VIEW_COLOR_IMAGE_HD, color_mat_);
    rs::Query qs = rs::create<rs::Query>(tcas);
    rapidjson::Document jsonDoc;
    std::string jsonQuery;
    bool found = false;
    if(cas.getFS("QUERY", qs) && qs.query() != "")
    {
      jsonQuery = qs.query();
      jsonDoc.Parse(jsonQuery);
      //TODO first level of json is currently only detect, needs to be done differently when there are
      //multiple modes

      rapidjson::Value detectQuery;
      if(jsonDoc.HasMember("detect"))
        detectQuery = jsonDoc["detect"];
      else if(jsonDoc.HasMember("track"))
        detectQuery = jsonDoc["track"];

      outWarn("json query: " << qs.query());
      if(detectQuery.IsObject())
      {
        //TODO How do we know what keywords can be found at what level in the json?
        rapidjson::Value::ConstMemberIterator colorMember = detectQuery.FindMember("color");
        rapidjson::Value::ConstMemberIterator detectionMember = detectQuery.FindMember("detection");

        if(colorMember != detectQuery.MemberEnd() || detectionMember != detectQuery.MemberEnd())
          found = true;
      }
    }


    scene.identifiables.filter(clusters);
    cluster_rois_.resize(clusters.size());
    color_ids_.resize(clusters.size(), std::vector<int>(COUNT));
    color_ratios_.resize(clusters.size(), std::vector<float>(COUNT));

    for(size_t idx = 0; idx < clusters.size(); ++idx)
    {
      rs::ImageROI image_rois = clusters[idx].rois.get();

      //======================= Calculate HSV image ==========================
      cv::Mat rgb, mask;
      cv::Rect roi;
      rs::conversion::from(image_rois.roi_hires(), roi);
      rs::conversion::from(image_rois.mask_hires(), mask);

      cluster_rois_[idx] = roi;

      color_mat_(roi).copyTo(rgb, mask);

      cv::Mat hsv, hist;
      cv::cvtColor(rgb, hsv, CV_BGR2HSV_FULL);
      size_t sum;
      std::vector<int> colorCount;
      countColors(hsv, mask, colorCount, sum);

      //======================= Calculate Semantic Color ==========================
      if(found)
      {
        std::vector<std::tuple<int, int>> colorsVec(COUNT);
        for(int i = 0; i < COUNT; ++i)
        {
          colorsVec[i] = std::tuple<int, int>(i, colorCount[i]);
        }

        for(size_t i = 0; i < COUNT; ++i)
        {
          int id, pixelCount;
          std::tie(id, pixelCount) = colorsVec[i];

          std::string color = color_names_[id];
          float ratio = (float)(pixelCount / (double)sum);
          if(ratio > 0.2)
          {
            rs::SemanticColor colorAnnotation = rs::create<rs::SemanticColor>(tcas);
            colorAnnotation.color.set(color);
            colorAnnotation.ratio.set(ratio);
            clusters[idx].annotations.append(colorAnnotation);
          }
        }
      }
      //======================= Calculate Color Histogram ==========================
      //Create the histogram
      int histSize[] = {histogram_cols_, histogram_rows_};
      float hranges[] = {0, 256}; // hue varies from 0 to 255, see cvtColor
      float sranges[] = {0, 256}; // saturation varies from 0 (black-gray-white) to 255 (pure spectrum color)
      const float *ranges[] = {hranges, sranges};

      // we compute the histogram from the 0-th and 1-st channels
      int channels[] = {0, 1};
      cv::calcHist(&hsv, 1, channels, mask, hist, 2, histSize, ranges, true, false);

      //Normalize histogram
      for(int r = 0; r < hist.rows; ++r)
      {
        float *it = hist.ptr<float>(r);
        for(int c = 0; c < hist.cols; ++c, ++it)
        {
          *it /= sum;
        }
      }

      rs::ColorHistogram color_hist_annotation = rs::create<rs::ColorHistogram>(tcas);
      color_hist_annotation.hist.set(rs::conversion::to(tcas, hist));
      outDebug("Conatiners for annotations created");

      clusters[idx].annotations.append(color_hist_annotation);
    }
    return UIMA_ERR_NONE;
  }

  void countColors(const cv::Mat &hsv, const cv::Mat &mask, std::vector<int> &colorCount, size_t &sum) const
  {
    assert(hsv.type() == CV_8UC3);

    sum = 0;
    colorCount = std::vector<int>(COUNT, 0.0);

    for(int r = 0; r < hsv.rows; ++r)
    {
      const cv::Vec3b *itHSV = hsv.ptr<cv::Vec3b>(r);
      const uint8_t *itM = mask.ptr<uint8_t>(r);

      for(int c = 0; c < hsv.cols; ++c, ++itHSV, ++itM)
      {
        if(!*itM)
        {
          continue;
        }

        ++sum;
        const uint8_t hue = itHSV->val[0];
        const uint8_t sat = itHSV->val[1];
        const uint8_t val = itHSV->val[2];

        if(sat > min_saturation_color_ && val > min_value_color_)
        {
          if(hue < color_positions_[RED])
          {
            ++colorCount[RED];
          }
          else if(hue < color_positions_[YELLOW])
          {
            ++colorCount[YELLOW];
          }
          else if(hue < color_positions_[GREEN])
          {
            ++colorCount[GREEN];
          }
          else if(hue < color_positions_[CYAN])
          {
            ++colorCount[CYAN];
          }
          else if(hue < color_positions_[BLUE])
          {
            ++colorCount[BLUE];
          }
          else if(hue < color_positions_[MAGENTA])
          {
            ++colorCount[MAGENTA];
          }
          else
          {
            ++colorCount[RED];
          }
        }
        else if(val <= max_value_black_)
        {
          ++colorCount[BLACK];
        }
        else if(val > min_value_white_)
        {
          ++colorCount[WHITE];
        }
        else
        {
          ++colorCount[GREY];
        }
      }
    }
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = color_mat_.clone();
    for(size_t i = 0; i < cluster_rois_.size(); ++i)
    {
      const cv::Rect &roi = cluster_rois_[i];
      const cv::Size histSize(roi.width, 10);
      const cv::Rect roiHist(roi.x, roi.y + roi.height + 1, histSize.width, histSize.height);
      const std::vector<int> &ids = color_ids_[i];
      const std::vector<float> &ratios = color_ratios_[i];

      cv::rectangle(disp, roi, colors_[ids[0]]);

      cv::Mat hist = disp(roiHist);

      float start = 0;
      for(int r = 0; r < ratios.size(); ++r)
      {
        float width = (histSize.width * ratios[r]);
        const cv::Rect rect(start + 0.5, 0, width + 0.5, histSize.height);
        start += width;
        cv::rectangle(hist, rect, colors_[ids[r]], CV_FILLED);
      }
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ClusterColorHistogramCalculator)

