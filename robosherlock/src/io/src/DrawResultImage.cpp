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

#include <iostream>
#include <fstream>
#include <ctime>

#include <ros/ros.h>
#include <ros/package.h>

#include <boost/date_time/time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/output.h>

#include <robosherlock/DrawingAnnotator.h>

#define DEBUG_OUTPUT 1

using namespace uima;

bool my_predicate(const char c)
{
  //checking if character is alphanumerical
  if(((c >= 48) && (c <= 57)) || //number
     ((c >= 65) && (c <= 90)) || //big letter
     ((c >= 97) && (c < 122)) //small letter
    ) {
    return false;
  }
  return true;
}

class DrawResultImage : public DrawingAnnotator
{

private:
  std::map<std::string, std::string> locationMapping;
  cv::Mat dispRgb;


public:

  DrawResultImage(): DrawingAnnotator(__func__)
  {
    locationMapping["kitchen_counter_island_top"] = "table";
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");

    return  UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    cas.get(VIEW_COLOR_IMAGE_HD, dispRgb);
    std::vector<rs::ObjectHypothesis> clusters;
    scene.identifiables.filter(clusters);
    outInfo("iterating over clusters");
    for(std::vector<rs::ObjectHypothesis>::iterator it = clusters.begin(); it != clusters.end(); ++it) {

      std::vector<std::string> atoms;

      int index = it - clusters.begin();

      outDebug("========Cluster " << it - clusters.begin() << "=========");
      std::vector<rs::Shape> shape;
      std::vector<rs::SemanticColor> color;
      std::vector<rs::Goggles> goggles;
      std::vector<rs::Detection> detections;
      std::vector<rs::Classification> classifications;
      std::vector<rs::SemanticSize> semSize;

      std::vector<rs::GroundTruth> gt;

      it->annotations.filter(shape);
      it->annotations.filter(color);
      it->annotations.filter(goggles);
      it->annotations.filter(detections);
      it->annotations.filter(semSize);
      it->annotations.filter(classifications);
      it->annotations.filter(gt);

      if(shape.size() > 0) {
        outDebug("No. of Shape Annotations :" << shape.size());
        for(int i = 0; i < shape.size(); ++i) {
          atoms.push_back(generateAtom("shape", index, shape[i].shape(), shape[i].confidence()));
        }
      }
      if(semSize.size() > 0) {
        outDebug("No. of SemanticSize Annotations :" << shape.size());
        for(int i = 0; i < semSize.size(); ++i) {
          atoms.push_back(generateAtom("size", index, semSize[i].size(), semSize[i].confidence()));
        }
      }
      if(color.size() > 0) {
        outDebug("No. of Color Annotations :" << color.size());
        for(int i = 0; i < color.size(); ++i) {
          std::string temp = color[i].color();
          float ratio  = color[i].ratio();
          atoms.push_back(generateAtom("color", index, color[i].color(), color[i].ratio()));
        }
      }
      if(goggles.size() > 0) {
        outDebug("No. of Goggles Annotations :" << goggles.size());
        for(int i = 0; i < goggles.size(); ++i) {
          std::stringstream predicate;
          if(goggles[i].category() != "") {
            predicate << "goggles_" << goggles[i].category();
          }
          else {
            predicate << "goggles";
          }
          std::string title = goggles[i].title();
          title.erase(std::remove_if(title.begin(), title.end(), my_predicate), title.end());
          atoms.push_back(generateAtom(predicate.str(), index, title));
        }
      }
      if(detections.size() > 0) {
        outDebug("No. of Detection annotations :" << detections.size());
        for(int i = 0; i < detections.size(); ++i) {
          //          atom << "instance(c" << index << "," << detections[i].name() << ")";
          atoms.push_back(generateAtom("detection", index, detections[i].name() , detections[i].confidence()));
        }
      }
      if(classifications.size() > 0) {
        for(auto c : classifications) {
          float confidence = 1.0f;
          for(auto conf : c.confidences())
            if(c.classname() == conf.name())
              confidence = conf.score();
          std::string cType = c.classification_type();
          if(cType == "SHAPE") {
            atoms.push_back(generateAtom("shape", index, c.classname(), confidence));
          }
          else if(cType == "CLASS") {
            atoms.push_back(generateAtom("class", index, c.classname(), confidence));
          }
          else if(cType == "INSTANCE") {
            atoms.push_back(generateAtom("instance", index, c.classname(), confidence));
          }
        }
      }
      std::stringstream clustername;
      clustername << "hyp_" << index;
      drawCluster(*it, tcas, clustername.str());
      drawAtoms(*it, tcas, atoms);

    }


    return  UIMA_ERR_NONE;
  }


private:

  std::string generateAtom(std::string type, int index, std::string evidence, float confidence = 0.0)
  {
    std::stringstream atom;
    std::stringstream conf;
    conf << confidence << " ";
    atom << type << "(c" << index << "," << evidence << ")";
    return atom.str();
  }

  void drawCluster(rs::ObjectHypothesis cluster, CAS &tcas, std::string name)
  {
    rs::ImageROI image_roi = cluster.rois();
    cv::Rect cluster_roi;
    rs::conversion::from(image_roi.roi_hires(), cluster_roi);
    cv::rectangle(dispRgb, cluster_roi, CV_RGB(255, 0, 0), 2.0);
    cv::putText(dispRgb, name, cv::Point(cluster_roi.x, cluster_roi.y - 3), cv::FONT_HERSHEY_PLAIN, 1.5, CV_RGB(0, 0, 255), 2.0);
  }

  void drawAtoms(rs::ObjectHypothesis cluster, CAS &tcas, std::vector< std::string> atoms)
  {
    rs::ImageROI image_roi = cluster.rois();
    cv::Rect rect;
    rs::conversion::from(image_roi.roi_hires(), rect);
    cv::rectangle(dispRgb, rect, CV_RGB(255, 0, 0), 2);
    int offset = 15;
    for(int32_t i = 0; i < atoms.size(); ++i) {
      int baseLine;
      cv::Size textSize = cv::getTextSize(atoms[i], cv::FONT_HERSHEY_PLAIN, 0.9, 1, &baseLine);
      cv::putText(dispRgb, atoms[i], cv::Point(rect.x + (rect.width - textSize.width) / 2, rect.y - offset - textSize.height - i * 17), cv::FONT_HERSHEY_PLAIN, 0.8, CV_RGB(255, 255, 200), 1.0);
    }
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp=dispRgb.clone();
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(DrawResultImage)

