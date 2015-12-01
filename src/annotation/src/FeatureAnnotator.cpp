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
//#include <opencv2/nonfree/nonfree.hpp>

// RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/DrawingAnnotator.h>

using namespace uima;

class FeatureAnnotator : public DrawingAnnotator
{
private:
  std::string keypointDetector;
  std::string featureExtractor;
  std::string featureType;
  cv::Ptr<cv::FeatureDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> extractor;
  std::vector<cv::KeyPoint> keypoints;

  cv::Mat color;

public:
  FeatureAnnotator() : DrawingAnnotator(__func__), detector(NULL), extractor(NULL)
  {
    //cv::initModule_nonfree();
  }

  /*
   * Initializes annotator
   */
  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    if(ctx.isParameterDefined("keypointDetector"))
    {
      ctx.extractValue("keypointDetector", keypointDetector);
    }
    else
    {
      outError("no keypoint detector provided!");
      return UIMA_ERR_ANNOTATOR_MISSING_INIT;
    }

    if(ctx.isParameterDefined("featureExtractor"))
    {
      ctx.extractValue("featureExtractor", featureExtractor);
    }
    else
    {
      outError("no feature extractor provided!");
      return UIMA_ERR_ANNOTATOR_MISSING_INIT;
    }

    outDebug("creating " << keypointDetector << " key points detector...");
    detector = cv::FeatureDetector::create(keypointDetector);
    if(detector.empty())
    {
      outError("creation failed!");
      return UIMA_ERR_ANNOTATOR_MISSING_INIT;
    }

#if OUT_LEVEL == OUT_LEVEL_DEBUG
    printParams(detector);
#endif
    setupAlgorithm(detector);

    outDebug("creating " << featureExtractor << " feature extractor...");
    extractor = cv::DescriptorExtractor::create(featureExtractor);
    if(extractor.empty())
    {
      outError("creation failed!");
      return UIMA_ERR_ANNOTATOR_MISSING_INIT;
    }

#if OUT_LEVEL == OUT_LEVEL_DEBUG
    printParams(extractor);
#endif
    setupAlgorithm(extractor);

    if(featureExtractor == "SIFT" || featureExtractor == "SURF")
    {
      featureType = "numerical";
    }
    else
    {
      featureType = "binary";
    }

    return UIMA_ERR_NONE;
  }

  /*
   * Destroys annotator
   */
  TyErrorId destroy()
  {
    outInfo("destroy");

    detector.release();
    extractor.release();

    return UIMA_ERR_NONE;
  }

private:

  void setupAlgorithm(cv::Algorithm *algorithm)
  {
    const std::string &name = algorithm->name();
    if(name == "Feature2D.BRISK")
    {
      algorithm->set("octaves", 3);
      algorithm->set("thres", 30);
    }
    else if(name == "Feature2D.ORB")
    {
      algorithm->set("nFeatures", 30);
      algorithm->set("scaleFactor", 1.2);
      algorithm->set("nLevels", 8);
      algorithm->set("edgeThreshold", 31);
      algorithm->set("firstLevel", 0);
      algorithm->set("WTA_K", 2);
      algorithm->set("scoreType", cv::ORB::HARRIS_SCORE);
      algorithm->set("patchSize", 31);
    }
    else if(name == "Feature2D.SIFT")
    {
      algorithm->set("contrastThreshold", 0.04);
      algorithm->set("edgeThreshold", 10.0);
      algorithm->set("nFeatures", 30);
      algorithm->set("nOctaveLayers", 3);
      algorithm->set("sigma", 1.6);
    }
    else if(name == "Feature2D.SURF")
    {
      algorithm->set("extended", false);
      algorithm->set("hessianThreshold", 100.0);
      algorithm->set("nOctaveLayers", 3);
      algorithm->set("nOctaves", 4);
      algorithm->set("upright", false);
    }
    else if(name == "Feature2D.FREAK")
    {
      algorithm->set("nbOctave", 4);
      algorithm->set("orientationNormalized", false);//true);
      algorithm->set("patternScale", 22.0);//22.0);
      algorithm->set("scaleNormalized", false);//true);
    }
    else if(name == "Feature2D.FAST")
    {
      algorithm->set("nonmaxSuppression", true);
      algorithm->set("threshold", 10);
    }
    else if(name == "Feature2D.GFTT")
    {
      algorithm->set("k", 0.04);
      algorithm->set("minDistance", 1.0);
      algorithm->set("nfeatures", 1000);
      algorithm->set("qualityLevel", 0.1);
      //algorithm->set("useHarrisDetector", keypointDetector == "HARRIS");
    }
    else if(name == "Feature2D.BRIEF")
    {
      algorithm->set("bytes", 32);
    }
    else if(name == "Feature2D.MSER")
    {
      algorithm->set("areaThreshold", 1.01);
      algorithm->set("delta", 5);
      algorithm->set("edgeBlurSize", 5);
      algorithm->set("maxArea", 14400);
      algorithm->set("maxEvolution", 200);
      algorithm->set("maxVariation", 0.25);
      algorithm->set("minArea", 60);
      algorithm->set("minDiversity", 0.2);
      algorithm->set("minMargin", 0.003);
    }
    else if(name == "Feature2D.STAR")
    {
      algorithm->set("lineThresholdBinarized", 8);
      algorithm->set("lineThresholdProjected", 10);
      algorithm->set("maxSize", 45);
      algorithm->set("responseThreshold", 30);
      algorithm->set("suppressNonmaxSize", 5);
    }
  }

  void printParams(cv::Algorithm *algorithm)
  {
    outInfo("Alogrithm: " << algorithm->name());

    std::vector<std::string> params;
    algorithm->getParams(params);
    for(size_t i = 0; i < params.size(); ++i)
    {
      const std::string &name = params[i];
      const int type = algorithm->paramType(name);
      outInfo("Parameter: " << name);

      switch(type)
      {
      case cv::Param::INT:
        outInfo("Type: INT");
        outInfo("Value: " << algorithm->get<int>(name));
        break;
      case cv::Param::BOOLEAN:
        outInfo("Type: BOOLEAN");
        outInfo("Value: " << algorithm->get<bool>(name));
        break;
      case cv::Param::REAL:
        outInfo("Type: REAL");
        outInfo("Value: " << algorithm->get<double>(name));
        break;
      case cv::Param::STRING:
        outInfo("Type: STRING");
        outInfo("Value: " << algorithm->get<std::string>(name));
        break;
      case cv::Param::MAT:
        outInfo("Type: MAT");
        outInfo("Value: " << algorithm->get<cv::Mat>(name));
        break;
      case cv::Param::MAT_VECTOR:
        outInfo("Type: MAT_VECTOR");
        outInfo("Value: " << algorithm->get<std::vector<cv::Mat>>(name).size());
        break;
      case cv::Param::ALGORITHM:
        outInfo("Type: ALGORITHM");
        outInfo("Value: -");
        break;
      case cv::Param::FLOAT:
        outInfo("Type: FLOAT");
        outInfo("Value: " << algorithm->get<float>(name));
        break;
      case cv::Param::UNSIGNED_INT:
        outInfo("Type: UNSIGNED_INT");
        outInfo("Value: " << algorithm->get<unsigned int>(name));
        break;
      case cv::Param::UINT64:
        outInfo("Type: UINT64");
        outInfo("Value: " << algorithm->get<uint64_t>(name));
        break;
      case cv::Param::SHORT:
        outInfo("Type: SHORT");
        outInfo("Value: " << algorithm->get<short>(name));
        break;
      case cv::Param::UCHAR:
        outInfo("Type: UCHAR");
        outInfo("Value: " << algorithm->get<unsigned char>(name));
        break;
      }
    }
  }

  /*
   * Processes a frame
   */
  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");
    rs::SceneCas cas(tcas);

    cas.get(VIEW_COLOR_IMAGE_HD, color);

    keypoints.clear();
    processClusters(tcas);

    return UIMA_ERR_NONE;
  }

  void extract(const cv::Mat &color, const cv::Rect &roi, const cv::Mat &mask, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
  {
    detector->detect(color(roi), keypoints, mask);

    for(size_t i = 0; i < keypoints.size(); ++i)
    {
      cv::KeyPoint &p = keypoints[i];
      p.pt.x += roi.x;
      p.pt.y += roi.y;
    }

    extractor->compute(color, keypoints, descriptors);
  }

  /*
   * Processes clusters
   */
  void processClusters(CAS &tcas)
  {
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);

    cv::Mat objMask, descriptors;
    cv::Rect roi;
    std::vector<cv::KeyPoint> keypoints;

    outDebug("clusters: " << clusters.size());
    for(size_t i = 0; i < clusters.size(); ++i)
    {
      rs::ImageROI image_rois = clusters[i].rois.get();

      rs::conversion::from(image_rois.roi_hires(), roi);
      rs::conversion::from(image_rois.mask_hires(), objMask);

      extract(color, roi, objMask, keypoints, descriptors);
      outDebug("features found: " << keypoints.size());

      if(keypoints.empty())
      {
        outInfo("no features found. skipping cluster");
        continue;
      }

      outDebug("creating features annotation...");
      rs::Features feats = rs::create<rs::Features>(tcas);
      //feats.detector(keypointDetector);
      feats.extractor(featureExtractor);
      feats.descriptorType(featureType);
      feats.descriptors(rs::conversion::to(tcas, descriptors));

      //feats.points.allocate(keypoints.size());
      //for(size_t j = 0; j < keypoints.size(); ++j)
      //{
      //  feats.points.set(j, rs::conversion::to(tcas, keypoints[j]));
      //}
      clusters[i].annotations.prepend(feats);

      this->keypoints.insert(this->keypoints.end(), keypoints.begin(), keypoints.end());
      keypoints.clear();
    }
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = color.clone();
    if(!this->keypoints.empty())
    {
      cv::drawKeypoints(disp, keypoints, disp, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    }
  }
};

MAKE_AE(FeatureAnnotator)
