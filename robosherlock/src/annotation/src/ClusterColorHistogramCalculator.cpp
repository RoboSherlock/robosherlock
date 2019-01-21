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
  int minValueColor;
  int minSaturationColor;
  int maxValueBlack;
  int minValueWhite;
  int histogramCols;
  int histogramRows;

  const double colorRange;
  std::vector<int> colorPositions;

  std::vector<cv::Scalar> colors;

  std::vector<std::string> colorNames;
  std::vector<cv::Rect> clusterRois;
  std::vector<std::vector<int>> colorIds;
  std::vector<std::vector<float>> colorRatios;

  cv::Mat color;

public:
  ClusterColorHistogramCalculator() : DrawingAnnotator(__func__), minValueColor(60), minSaturationColor(60), maxValueBlack(60), minValueWhite(120), histogramCols(16), histogramRows(16), colorRange(256.0 / 6.0)
  {
    colorPositions.resize(6);
    for(size_t i = 0; i < 6; ++i)
    {
      colorPositions[i] = (int)(i * colorRange + colorRange / 2.0 + 0.5);
    }

    colorNames.resize(COUNT);
    colorNames[RED]     = "red";
    colorNames[YELLOW]  = "yellow";
    colorNames[GREEN]   = "green";
    colorNames[CYAN]    = "cyan";
    colorNames[BLUE]    = "blue";
    colorNames[MAGENTA] = "magenta";
    colorNames[WHITE]   = "white";
    colorNames[BLACK]   = "black";
    colorNames[GREY]    = "grey";

    colors.resize(COUNT);
    colors[RED]     = CV_RGB(255, 0, 0);
    colors[YELLOW]  = CV_RGB(255, 255, 0);
    colors[GREEN]   = CV_RGB(0, 255, 0);
    colors[CYAN]    = CV_RGB(0, 255, 255);
    colors[BLUE]    = CV_RGB(0, 0, 255);
    colors[MAGENTA] = CV_RGB(255, 0, 255);
    colors[WHITE]   = CV_RGB(255, 255, 255);
    colors[BLACK]   = CV_RGB(0, 0, 0);
    colors[GREY]    = CV_RGB(127, 127, 127);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    if(ctx.isParameterDefined("minValueColor"))
    {
      ctx.extractValue("minValueColor", minValueColor);
    }
    if(ctx.isParameterDefined("minSaturationColor"))
    {
      ctx.extractValue("minSaturationColor", minSaturationColor);
    }
    if(ctx.isParameterDefined("maxValueBlack"))
    {
      ctx.extractValue("maxValueBlack", maxValueBlack);
    }
    if(ctx.isParameterDefined("minValueWhite"))
    {
      ctx.extractValue("minValueWhite", minValueWhite);
    }
    if(ctx.isParameterDefined("histogramCols"))
    {
      ctx.extractValue("histogramCols", histogramCols);
    }
    if(ctx.isParameterDefined("histogramRows"))
    {
      ctx.extractValue("histogramRows", histogramRows);
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

    cas.get(VIEW_COLOR_IMAGE_HD, color);
    rs::Query qs = rs::create<rs::Query>(tcas);
    rapidjson::Document jsonDoc;
    std::string jsonQuery;
    bool found = false;
    if(cas.getFS("QUERY", qs) && qs.query()!="")
    {
      jsonQuery = qs.query();
      jsonDoc.Parse(jsonQuery);
      //TODO first level of json is currently only detect, needs to be done differently when there are
      //multiple modes
      rapidjson::Value detectQuery;
      if(jsonDoc.HasMember("detect")){
          detectQuery = jsonDoc["detect"];
      }
      else
          if(jsonDoc.HasMember("track")){
              detectQuery = jsonDoc["track"];
          }
      outWarn("json query: " << qs.query());
      if(detectQuery.IsObject()){
          //TODO How do we know what keywords can be found at what level in the json?
          rapidjson::Value::ConstMemberIterator colorMember = detectQuery.FindMember("color");
          rapidjson::Value::ConstMemberIterator detectionMember = detectQuery.FindMember("detection");

          if(colorMember != detectQuery.MemberEnd() || detectionMember != detectQuery.MemberEnd())
          {
            found = true;
          }
      }
    }


    scene.identifiables.filter(clusters);
    clusterRois.resize(clusters.size());
    colorIds.resize(clusters.size(), std::vector<int>(COUNT));
    colorRatios.resize(clusters.size(), std::vector<float>(COUNT));

    for(size_t idx = 0; idx < clusters.size(); ++idx)
    {
      rs::ImageROI image_rois = clusters[idx].rois.get();

      //======================= Calculate HSV image ==========================
      cv::Mat rgb, mask;
      cv::Rect roi;
      rs::conversion::from(image_rois.roi_hires(), roi);
      rs::conversion::from(image_rois.mask_hires(), mask);

      clusterRois[idx] = roi;

      color(roi).copyTo(rgb, mask);

      cv::Mat hsv, hist;
      cv::cvtColor(rgb, hsv, CV_BGR2HSV_FULL);
      size_t sum;
      std::vector<int> colorCount;
      countColors(hsv, mask, colorCount, sum);

      //======================= Calculate Semantic Color ==========================
      if(found)
      {
        rs::SemanticColor color_annotation = rs::create<rs::SemanticColor>(tcas);
        std::vector<std::tuple<int, int>> colorsVec(COUNT);
        for(int i = 0; i < COUNT; ++i)
        {
          colorsVec[i] = std::tuple<int, int>(i, colorCount[i]);
        }
        std::sort(colorsVec.begin(), colorsVec.end(), [](const std::tuple<int, int> &a, const std::tuple<int, int> &b)
        {
          return std::get<1>(a) > std::get<1>(b);
        });

        std::vector<int> &ids = colorIds[idx];
        std::vector<float> &ratios = colorRatios[idx];
        std::vector<std::string> colors(COUNT);

        for(size_t i = 0; i < COUNT; ++i)
        {
          int id, ratio;
          std::tie(id, ratio) = colorsVec[i];
          ids[i] = id;
          colors[i] = colorNames[id];
          ratios[i] = (float)(ratio / (double)sum);
        }

        color_annotation.color(colors);
        color_annotation.ratio(ratios);

        clusters[idx].annotations.append(color_annotation);
      }
      //======================= Calculate Color Histogram ==========================
      //Create the histogram
      int histSize[] = {histogramCols, histogramRows};
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

        if(sat > minSaturationColor && val > minValueColor)
        {
          if(hue < colorPositions[RED])
          {
            ++colorCount[RED];
          }
          else if(hue < colorPositions[YELLOW])
          {
            ++colorCount[YELLOW];
          }
          else if(hue < colorPositions[GREEN])
          {
            ++colorCount[GREEN];
          }
          else if(hue < colorPositions[CYAN])
          {
            ++colorCount[CYAN];
          }
          else if(hue < colorPositions[BLUE])
          {
            ++colorCount[BLUE];
          }
          else if(hue < colorPositions[MAGENTA])
          {
            ++colorCount[MAGENTA];
          }
          else
          {
            ++colorCount[RED];
          }
        }
        else if(val <= maxValueBlack)
        {
          ++colorCount[BLACK];
        }
        else if(val > minValueWhite)
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
    disp = color.clone();
    for(size_t i = 0; i < clusterRois.size(); ++i)
    {
      const cv::Rect &roi = clusterRois[i];
      const cv::Size histSize(roi.width, 10);
      const cv::Rect roiHist(roi.x, roi.y + roi.height + 1, histSize.width, histSize.height);
      const std::vector<int> &ids = colorIds[i];
      const std::vector<float> &ratios = colorRatios[i];

      cv::rectangle(disp, roi, colors[ids[0]]);

      cv::Mat hist = disp(roiHist);

      float start = 0;
      for(int r = 0; r < ratios.size(); ++r)
      {
        float width = (histSize.width * ratios[r]);
        const cv::Rect rect(start + 0.5, 0, width + 0.5, histSize.height);
        start += width;
        cv::rectangle(hist, rect, colors[ids[r]], CV_FILLED);
      }
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ClusterColorHistogramCalculator)

