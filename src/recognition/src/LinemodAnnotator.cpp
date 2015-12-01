/* Copyright (c) 2013, Thiemo Wiedemeyer  <wiedemeyer@informatik.uni-bremen.de>
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

// UIMA
#include <uima/api.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/recognition/LinemodInterface.h>
#include <rs/DrawingAnnotator.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <limits>
#define DEBUG_OUTPUT 1

using namespace uima;

//NOTE: does not like Highres Images...schade
class LinemodAnnotator : public DrawingAnnotator
{

private:
  std::string modelsPath;
  LinemodInterface linemod;
  bool useClusters;
  float minResponse;
  cv::Mat color;
  std::vector<Result> final_res;

public:
  LinemodAnnotator() : DrawingAnnotator(__func__), useClusters(false), minResponse(0.88f)
  {

  }

  /*
   * Initializes annotator
   */
  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    if(ctx.isParameterDefined("modelsPath"))
    {
      ctx.extractValue("modelsPath", modelsPath);
      std::string pathToProject = ros::package::getPath("rs_resources");
      modelsPath = pathToProject + "/" + modelsPath;
    }
    else
    {
      outError("no model path provided!");
      return UIMA_ERR_ANNOTATOR_MISSING_INIT;
    }

    if(!linemod.readModels(modelsPath))
    {
      outError("error while reading models! Did you initialize the submodules?");
      return UIMA_ERR_ANNOTATOR_MISSING_INIT;
    }

    outDebug("Classes: " << linemod.detector->numClasses());
    outDebug("Templates: " << linemod.detector->numTemplates());
    outDebug("Pyramid levels: " << linemod.detector->pyramidLevels());

    const std::vector<std::string> &classes = linemod.detector->classIds();
    for(size_t i = 0; i < classes.size(); ++i)
    {
      outDebug("Class: " << classes[i] << " Templates: " << linemod.detector->numTemplates(classes[i]));
    }

    return UIMA_ERR_NONE;
  }

  /*
   * Destroys annotator
   */
  TyErrorId destroy()
  {
    outInfo("destroy");

    return UIMA_ERR_NONE;
  }

private:
  /*
   * Processes a frame
   */
  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");
    processFrame(tcas);

    return UIMA_ERR_NONE;
  }

  /*
   * Processes a frame
   */
  void processFrame(CAS &tcas)
  {
    cv::Mat
    depth,
    mask; // If mask is not set, no mask will be used.
    rs::SceneCas cas(tcas);
    cas.get(VIEW_COLOR_IMAGE, color);
    cas.get(VIEW_DEPTH_IMAGE, depth);
    rs::Scene scene = cas.getScene();
    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);
    std::vector<Result> res_vector;
    if(color.size() != depth.size())
    {
      outError("Input images are not of the same size.");
      return;
    }

    std::vector<std::string> classes;

    final_res.clear();
    linemod.process(color, depth, res_vector, minResponse, classes, mask);

    for(int i = 0; i < (int)res_vector.size(); ++i)
    {
      Result &res = res_vector[i];
      outDebug("Best match for: " << res.name << " resp: " << res.response << " pos: (" << res.roi.x << ", " << res.roi.y << ") size: (" << res.roi.width << ", " << res.roi.height << ")");
      //check if result is good and if so attach it to an existing cluster
      int best_match_idx = -1;
      int best_match_area = 0;
      if(res.response > 80.0)
      {
        for(unsigned int j = 0; j < clusters.size(); ++j)
        {
          try
          {
            rs::ImageROI image_roi = clusters[j].rois.get();
            cv::Rect cluster_roi;
            rs::conversion::from(image_roi.roi(), cluster_roi);
            cv::Rect intersection = cluster_roi & res.roi;

            if((intersection.area() > 0) && (intersection.area() > 0.8 * cluster_roi.area()) &&
               (intersection.area() > best_match_area))
            {
              best_match_idx = j;
              best_match_area = intersection.area();
              outInfo("Intersection area: " << intersection.area());
            }
          }
          catch(uima::Exception e)
          {
            outInfo("EXCEPTION CAUGHT! No ImageROI found in scene cas!");
          }
        }
        if(best_match_idx != -1)
        {
          outInfo("Found existing 3D cluster that matches the Linemod result.");
          outInfo("Attaching result to cluster No: " << best_match_idx);
          rs::Detection annot = rs::create<rs::Detection>(tcas);
          annot.name.set(res.name);
          annot.source.set("Linemod");
          annot.confidence.set(res.response);
          clusters[best_match_idx].annotations.append(annot);
          final_res.push_back(res);
        }
        else
        {
          outInfo("No 3D cluster found that matches the ROI returned by Linemod");
        }
      }
    }
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = color.clone();
    for(unsigned int i = 0; i < final_res.size(); ++i)
    {
      Result &res = final_res[i];
      if(res.response > 80.0)
      {
        linemod.drawResult(disp, res);
      }
    }
  }

  void dispDepth(cv::Mat &image)
  {
    const float
    maxValue = 9.757f;

    #pragma omp parallel for
    for(int r = 0; r < image.rows; ++r)
    {
      float *it = image.ptr<float>(r);

      for(int c = 0; c < image.cols; ++c, ++it)
      {
        *it /= maxValue;
      }
    }
  }
};

MAKE_AE(LinemodAnnotator)
