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

#include <pracmln/mln.h>

#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/DrawingAnnotator.h>

#define DEBUG_OUTPUT 1

using namespace uima;

bool my_predicate(const char c)
{
  //checking if character is alphanumerical
  if(((c >= 48) && (c <= 57)) || //number
     ((c >= 65) && (c <= 90)) || //big letter
     ((c >= 97) && (c < 122)) //small letter
    )
  {
    return false;
  }
  return true;
}

class MLNInferencer : public DrawingAnnotator
{

private:

  std::stringstream filenameMLN, filenameMLNAnnot;
  std::string mlnFile;
  bool save, wordnetGrounded, queryMLN, fuzzy_;
  std::map<std::string, std::string> locationMapping;
  std::map<std::string, std::string> wordnetMapping;
  MLN mln;
  std::vector<std::string> query;

  cv::Mat dispRgb;

public:

  MLNInferencer(): DrawingAnnotator(__func__), save(false), wordnetGrounded(false), queryMLN(false), fuzzy_(false)
  {
    locationMapping["kitchen_counter_island_top"] = "table";
    //    location_mapping["/iai_kitchen/counter_top_island_link"] = "table";

    //shapes
    wordnetMapping["box"] = "boxlike.s.01";
    wordnetMapping["cylinder"] = "cylindrical.s.01";
    wordnetMapping["flat"] = "flat.s.02";
    wordnetMapping["round"] = "round.a.01";
    //sizes
    wordnetMapping["small"] = "small.a.01";
    wordnetMapping["medium"] = "medium-sized.s.01";
    wordnetMapping["large"] = "large.a.01";
    //colors
    wordnetMapping["red"] = "red.s.01";
    wordnetMapping["yellow"]  = "yellow.s.01";
    wordnetMapping["green"] = "green.s.01";
    wordnetMapping["cyan"] = "cyan.s.01";
    wordnetMapping["blue"] = "blue.s.01";
    wordnetMapping["magenta"] = "pink.s.01";//maybe
    wordnetMapping["white"] = "white.a.01";
    wordnetMapping["black"] = "black.a.01";
    wordnetMapping["grey"]     = "gray.a.01";

    query.push_back("object");
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    if(ctx.isParameterDefined("save_to_file"))
    {
      ctx.extractValue("save_to_file", save);
      if(save)
      {
        outDebug("Turned on saving atoms to file");
        std::time_t t = time(0);
        struct tm *now = localtime(& t);
        filenameMLN << "mln_" << now->tm_hour << ":" << now->tm_min << ".txt";
      }
    }

    //ground atoms as Wridnet synsets
    if(ctx.isParameterDefined("wordnetGrounded"))
    {
      ctx.extractValue("wordnetGrounded", wordnetGrounded);
    }

    //path to model file
    if(ctx.isParameterDefined("mlnFile"))
    {
      ctx.extractValue("mlnFile", mlnFile);
    }

    //set true if annotator should query a learned model,
    //otherwise it will only generate atoms
    if(ctx.isParameterDefined("query_mln"))
    {
      ctx.extractValue("query_mln", queryMLN);
    }

    if(ctx.isParameterDefined("fuzzy"))
    {
      ctx.extractValue("fuzzy", fuzzy_);
    }

    //init MLN
    outAssert(mln.initialize(), "could not initialize mln");

    if(queryMLN)
    {
      std::string pathToProject = ros::package::getPath("rs_resources");
      mlnFile.insert(0, pathToProject);
      outInfo(mlnFile);
      std::vector<std::string> cwp;
      cwp.push_back("shape");
      cwp.push_back("color");
      cwp.push_back("size");
      cwp.push_back("instance");
      mln.setCWPreds(cwp);
      mln.setGrammar("PRACGrammar");
      mln.setLogic("FirstOrderLogic");
      mln.setMethod("WCSPInference");
      mln.setMLN(mlnFile);
    }
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
    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);
    outInfo("iterating over clusters");
    std::stringstream mlnDatabase;
    for(std::vector<rs::Cluster>::iterator it = clusters.begin(); it != clusters.end(); ++it)
    {

      std::vector<std::string> atoms;
      rs::MLNAtoms mln_atoms = rs::create<rs::MLNAtoms>(tcas);
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

      if(shape.size() > 0)
      {
        outDebug("No. of Shape Annotations :" << shape.size());
        for(int i = 0; i < shape.size(); ++i)
        {
          mlnDatabase << generateAtom("shape", index, shape[i].shape(), shape[i].confidence()) << std::endl;
          atoms.push_back(generateAtom("shape", index, shape[i].shape(), shape[i].confidence()));
        }
      }
      if(semSize.size() > 0)
      {
        outDebug("No. of SemanticSize Annotations :" << shape.size());
        for(int i = 0; i < semSize.size(); ++i)
        {
          mlnDatabase << generateAtom("size", index, semSize[i].size(), semSize[i].confidence()) << std::endl;
          atoms.push_back(generateAtom("size", index, semSize[i].size(), semSize[i].confidence()));
        }
      }
      if(color.size() > 0)
      {
        outDebug("No. of Color Annotations :" << color.size());
        for(int i = 0; i < color.size(); ++i)
        {
          std::vector<std::string> temp = color[i].color();
          std::vector<float> ratio  = color[i].ratio();

          for(int j = 0; j < temp.size(); ++j)
          {
            if(ratio[j] > 0.30)
            {
              mlnDatabase << generateAtom("color", index, temp[j], ratio[j]) << std::endl;
              atoms.push_back(generateAtom("color", index, temp[j], ratio[j]));
            }

          }
        }
      }
      if(goggles.size() > 0)
      {
        outDebug("No. of Goggles Annotations :" << goggles.size());
        for(int i = 0; i < goggles.size(); ++i)
        {
          std::stringstream predicate;
          if(goggles[i].category() != "")
          {
            predicate << "goggles_" << goggles[i].category();
          }
          else
          {
            predicate << "goggles";
          }

          std::string title = goggles[i].title();
          title.erase(std::remove_if(title.begin(), title.end(), my_predicate), title.end());

          atoms.push_back(generateAtom(predicate.str(), index, title));
          mlnDatabase << generateAtom(predicate.str(), index, title) << std::endl;
        }
      }
      if(detections.size() > 0)
      {
        outDebug("No. of Detection annotations :" << detections.size());
        for(int i = 0; i < detections.size(); ++i)
        {
          //          atom << "instance(c" << index << "," << detections[i].name() << ")";
          atoms.push_back(generateAtom("detection", index, detections[i].name() , detections[i].confidence()));
          mlnDatabase << generateAtom("detection", index, detections[i].name() , detections[i].confidence()) << std::endl;
        }
      }
      if(classifications.size() > 0)
      {
        for(auto c : classifications)
        {
          float confidence = 1.0f;
          for(auto conf : c.confidences())
            if(c.classname() == conf.name())
              confidence = conf.score();
          std::string cType = c.classification_type();
          if(cType == "SHAPE")
          {
            atoms.push_back(generateAtom("shape", index, c.classname(), confidence));
            mlnDatabase << generateAtom("shape", index, c.classname(), confidence)     << std::endl;
          }
          else if(cType == "CLASS")
          {
            atoms.push_back(generateAtom("class", index, c.classname(), confidence));
            mlnDatabase << generateAtom("class", index, c.classname(), confidence)     << std::endl;
          }
          else if(cType == "INSTANCE")
          {
            atoms.push_back(generateAtom("instance", index, c.classname(), confidence));
            mlnDatabase << generateAtom("instance", index, c.classname(), confidence)     << std::endl;
          }
        }
      }
      if(gt.size() > 0)
      {
        rs::GroundTruth groundTruth = gt[0];
        std::string classNameGT = groundTruth.classificationGT().classname();
        atoms.push_back(generateAtom("object", index, classNameGT, 1.0));
      }

      mln_atoms.atoms.append(atoms);
      it->annotations.append(mln_atoms);

#if DEBUG_OUTPUT
      std::vector<std::string> temp = mln_atoms.atoms();
      outInfo("Nr of atoms generated: " << temp.size());
      for(int i = 0; i < atoms.size(); ++i)
      {
        outInfo(temp[i]);
      }
#endif
      //save atoms to a file
      if(save)
      {
        //location is only needed for the first cluster
        if(index == 0)
        {
          std::vector<rs::TFLocation> tf_loc;
          it->annotations.filter(tf_loc);

          outInfo("TF_locations found: " << tf_loc.size());
          if(tf_loc.size() != 0)
          {
            outInfo("We have location annotation");
            outInfo(tf_loc[0].frame_id());
            save_to_file(mln_atoms, scene, tf_loc[0].frame_id());
          }
          else
          {
            save_to_file(mln_atoms, scene);
          }
        }
        else
        {
          save_to_file(mln_atoms, scene);
        }
      }
      drawAtoms(*it, tcas, atoms);
    }

    if(queryMLN)
    {
      mln.setQuery(query);
      mln.setDB(mlnDatabase.str(), false);
      std::vector<std::string> results;
      std::vector<double> probabilities;
      outInfo("Querying MLN");
      if(mln.infer(results, probabilities))
      {
        outInfo("Results size: " << results.size());
      }
      int objectsTrained = 0;
      if(clusters.size() != 0)
      {
        objectsTrained  = results.size() / clusters.size();
      }
      outInfo("objects trained on: " << objectsTrained);
      for(int i = 0; i < results.size(); ++i)
      {
        if(probabilities[i] == 1)
        {
          //this code here is very biased on the training set atm. needs some tweaking
          std::vector<std::string> splitRes;
          boost::split(splitRes, results[i], boost::is_any_of(","));
          char ch = splitRes[0].back();
          int index = ch - '0';
          std::string label = splitRes[1].substr(0, splitRes[1].size() - 1);
          rs::Detection detection = rs::create<rs::Detection>(tcas);
          detection.confidence.set((float)probabilities[i]);
          detection.name.set(label);
          detection.source.set("MLN");
          outInfo("Cluster " << index << " is a " << label);
          clusters[index].annotations.append(detection);
          drawCluster(clusters[index], tcas, label);
        }
      }
    }
    return  UIMA_ERR_NONE;
  }


private:

  std::string generateAtom(std::string type, int index, std::string evidence, float confidence = 0.0)
  {
    std::stringstream atom;
    std::stringstream conf;
    conf << confidence << " ";
    atom << (fuzzy_ ? conf.str() : "") << type << "(c" << index << "," << (wordnetGrounded ? wordnetMapping[evidence] : evidence) << ")";
    return atom.str();
  }

  void save_to_file(rs::MLNAtoms mln_atoms, rs::Scene &scene, std::string loc = "")
  {

    outDebug("Appending to file: " << filenameMLN.str());
    std::vector<std::string> atoms = mln_atoms.atoms();
    outDebug("Number of atoms being appended: " << atoms.size());
    std::ofstream mln_file;
    std::stringstream mln_atoms_str;
    mln_file.open(filenameMLN.str().c_str(), std::ios::app);

    if(loc != "")
    {
      ros::Time time(scene.timestamp()*std::pow(10, -9));
      boost::posix_time::ptime p_time = time.toBoost();
      std::string stime = boost::posix_time::to_iso_extended_string(p_time);
      mln_atoms_str << "---" << std::endl;
      //mln_atoms_str << "scene(" << location_mapping[loc] << ")" << std::endl << std::endl;
    }
    for(int i = 0; i < atoms.size(); ++i)
    {
      mln_atoms_str << atoms[i] << std::endl;
    }
    mln_atoms_str << "---" << std::endl;
    mln_atoms_str << std::endl;
    mln_file << mln_atoms_str.str();

    mln_file.close();
  }


  void drawCluster(rs::Cluster cluster, CAS &tcas, std::string name)
  {
    rs::ImageROI image_roi = cluster.rois();
    cv::Rect cluster_roi;
    rs::conversion::from(image_roi.roi_hires(), cluster_roi);
    cv::rectangle(dispRgb, cluster_roi, CV_RGB(255, 0, 0), 2.0);
    cv::putText(dispRgb, name, cv::Point(cluster_roi.x, cluster_roi.y - 3), cv::FONT_HERSHEY_PLAIN, 1.5, CV_RGB(0, 0, 255), 2.0);
  }

  void drawAtoms(rs::Cluster cluster, CAS &tcas, std::vector< std::string> atoms)
  {
    rs::ImageROI image_roi = cluster.rois();
    cv::Rect rect;
    rs::conversion::from(image_roi.roi_hires(), rect);
    cv::rectangle(dispRgb, rect, CV_RGB(255, 0, 0), 2);
    int offset = 15;
    for(int32_t i = 0; i < atoms.size(); ++i)
    {
      int baseLine;
      cv::Size textSize = cv::getTextSize(atoms[i], cv::FONT_HERSHEY_PLAIN, 0.8, 1, &baseLine);
      cv::putText(dispRgb, atoms[i], cv::Point(rect.x + (rect.width - textSize.width) / 2, rect.y - offset - textSize.height - i * 17), cv::FONT_HERSHEY_PLAIN, 0.8, CV_RGB(255, 255, 200), 1.0);
    }
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = dispRgb.clone();
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(MLNInferencer)
