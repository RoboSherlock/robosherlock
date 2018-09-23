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

#include <ctype.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>

#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/DrawingAnnotator.h>

using namespace uima;

class ClassifierDetection : public DrawingAnnotator
{

private:
  cv::Mat color;
public:
  ClassifierDetection() : DrawingAnnotator(__func__)
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

private:
  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    std::vector<std::string> classNames;
    std::vector<int32_t> responses;

    cas.get(VIEW_COLOR_IMAGE_HD, color);
    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);
    outInfo("iterating over clusters");
    for(std::vector<rs::Cluster>::iterator it = clusters.begin(); it != clusters.end(); ++it)
    {

      classNames.clear();
      responses.clear();

      std::vector<rs::Features> features;
      it->annotations.filter(features);
      if(features.size() > 0)
      {
        outInfo("No. of Feature annotation(stupid name): " << features.size());
        rs::Features feat = features[0];
        std::vector<rs::Response> resps = feat.response();
        if(resps.empty())
        {
          continue;
        }
        rs::Response &resp = resps[0];

        if(classNames.empty())
        {
          classNames.resize(resp.classNames.size());
          for(size_t j = 0; j < classNames.size(); ++j)
          {
            classNames[j] = resp.classNames.get(j);
          }
        }
        cv::Mat respM, respD;
        rs::conversion::from(resp.response.get(), respM);
        rs::conversion::from(resp.distances.get(), respD);
        outInfo("Confidence: " << respD.at<float>(0));
        if(respD.at<float>(0) > 2.3f)
        {
          continue;
        }
        responses.push_back((int32_t)respM.at<float>(0));
        const std::string &className = classNames[responses.back()];
        outInfo("className: " << className);

        rs::Detection detection  = rs::create<rs::Detection>(tcas);
        detection.name.set(className);
        detection.source.set("KNNClassifierDetection");
        detection.confidence.set(respD.at<float>(0));
        //draw result on image
        rs::ImageROI image_roi = it->rois.get();
        cv::Rect rect;
        rs::conversion::from(image_roi.roi_hires.get(), rect);
        cv::rectangle(color, rect, CV_RGB(255, 0, 0), 2);
        int offset = 15;
        int baseLine;
        cv::Size textSize = cv::getTextSize(className, cv::FONT_HERSHEY_PLAIN, 1.5, 2.0, &baseLine);
        cv::putText(color, className, cv::Point(rect.x + (rect.width - textSize.width) / 2, rect.y - offset - textSize.height), cv::FONT_HERSHEY_PLAIN, 1.5, CV_RGB(0, 0, 0), 2.0);
      }
    }

    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = color.clone();
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ClassifierDetection)
