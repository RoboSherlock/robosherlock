/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
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

// UIMA
#include <uima/api.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

#if CV_VERSION_MAJOR == 3
#include <opencv2/xfeatures2d.hpp>
#endif

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
  FeatureAnnotator() : DrawingAnnotator(__func__)
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

#if CV_MAJOR_VERSION == 2
    detector = cv::BRISK::create(keypointDetector);

    if(detector.empty())
    {
      outError("creation failed!");
      return UIMA_ERR_ANNOTATOR_MISSING_INIT;
    }
    setupAlgorithm(detector);
#if OUT_LEVEL == OUT_LEVEL_DEBUG
    printParams(detector);
#endif
#elif CV_MAJOR_VERSION == 3
    setupAlgorithm(detector, keypointDetector);
#endif

    outDebug("creating " << featureExtractor << " feature extractor...");
#if CV_MAJOR_VERSION ==2
    extractor = cv::DescriptorExtractor::create(featureExtractor);
    if(extractor.empty())
    {
      outError("creation failed!");
      return UIMA_ERR_ANNOTATOR_MISSING_INIT;
    }
    setupAlgorithm(extractor);
#if OUT_LEVEL == OUT_LEVEL_DEBUG
    printParams(extractor);
#endif
#elif CV_MAJOR_VERSION == 3
    setupAlgorithm(extractor, keypointDetector);
#endif

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
#if CV_MAJOR_VERSION == 2
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
#elif CV_MAJOR_VERSION ==3
  void setupAlgorithm(cv::Algorithm *algorithm, std::string name)
  {
    if(name == "BRISK")
    {
      algorithm = cv::BRISK::create();
    }
    else if(name == "ORB")
    {
      algorithm = cv::ORB::create(30);
    }
    else if(name == "SIFT")
    {
      algorithm = cv::xfeatures2d::SIFT::create(30,3,0.04,10,1.6);
    }
    else if(name == "SURF")
    {
      algorithm = cv::xfeatures2d::SURF::create();
    }
    else if(name == "FREAK")
    {
      algorithm = cv::xfeatures2d::FREAK::create(false, false);
    }
    else if(name == "FAST")
    {
      algorithm = cv::FastFeatureDetector::create();
    }
    else if(name == "GFTT")
    {
      algorithm = cv::GFTTDetector::create(1000,0.1);
    }
    else if(name == "BRIEF")
    {
      algorithm = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if(name == "MSER")
    {
      algorithm = cv::MSER::create();
    }
    else if(name == "STAR")
    {
      algorithm = cv::xfeatures2d::StarDetector::create();
    }
  }
#endif

#if CV_VERSION_MAJOR == 2
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
#endif
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
      feats.source(featureExtractor);
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
