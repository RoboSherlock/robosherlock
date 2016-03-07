/* Copyright (c) 2011, Ferenc Balint-Benczedi <balintbe@tzi.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
  bool save, wordnetGrounded, queryMLN;
  std::map<std::string, std::string> locationMapping;
  std::map<std::string, std::string> wordnetMapping;
  MLN mln;
  std::vector<std::string> query;

  cv::Mat dispRgb;

public:

  MLNInferencer(): DrawingAnnotator(__func__), save(false), wordnetGrounded(false), queryMLN(false)
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
      mln.setMethod("WCSP (exact MPE with Toulbar2)");
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
    cas.get(VIEW_COLOR_IMAGE_HD,dispRgb);
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
      std::vector<rs::Geometry> geom;
      std::vector<rs::SemanticColor> color;
      std::vector<rs::Goggles> goggles;
      std::vector<rs::Detection> instances;


      it->annotations.filter(geom);
      it->annotations.filter(shape);
      it->annotations.filter(color);
      it->annotations.filter(goggles);
      it->annotations.filter(instances);


      if(shape.size() > 0)
      {
        outDebug("No. of Shape Annotations :" << shape.size());
        for(int i = 0; i < shape.size(); ++i)
        {
          mlnDatabase << generateAtom("shape", index, shape[i].shape(), wordnetGrounded) << std::endl;
          atoms.push_back(generateAtom("shape", index, shape[i].shape(), wordnetGrounded));
        }
      }
      if(geom.size() > 0)
      {
        outDebug("No. of Geometric Annotations :" << geom.size());
        for(int i = 0; i < geom.size(); ++i)
        {
          mlnDatabase << generateAtom("size", index, geom[i].size(), wordnetGrounded) << std::endl;
          atoms.push_back(generateAtom("size", index, geom[i].size(), wordnetGrounded));
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
              mlnDatabase << generateAtom("color", index, temp[j], wordnetGrounded) << std::endl;
              atoms.push_back(generateAtom("color", index, temp[j], wordnetGrounded));
            }

          }
        }
      }
      if(goggles.size() > 0)
      {
        outDebug("No. of Goggles Annotations :" << goggles.size());
        for(int i = 0; i < goggles.size(); ++i)
        {
          std::stringstream atom;
          if(goggles[i].category() != "")
          {
            atom << "goggles_" << goggles[i].category();
          }
          else
          {
            atom << "goggles";
          }
          std::string title = goggles[i].title();
          title.erase(std::remove_if(title.begin(), title.end(), my_predicate), title.end());
          atom << "(c" << index << "," << title << ")";
          atoms.push_back(atom.str());
          mlnDatabase << atom.str() << std::endl;
        }
      }
      if(instances.size() > 0)
      {
        outDebug("No. of Detection annotations :" << instances.size());
        for(int i = 0; i < instances.size(); ++i)
        {
          std::stringstream atom;
          atom << "instance(c" << index << "," << instances[i].name() << ")";
          atoms.push_back(atom.str());
          mlnDatabase << atom.str() << std::endl;
        }
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

  std::string generateAtom(std::string type, int index, std::string evidence, bool forwordnet)
  {
    std::stringstream atom;
    if(forwordnet)
    {
      atom << type << "(c" << index << "," << wordnetMapping[evidence] << ")";
    }
    else
    {
      atom << type << "(c" << index << "," << evidence  << ")";
    }
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
