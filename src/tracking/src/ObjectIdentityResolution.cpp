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

// STL
#include <map>
#include <vector>
#include <tuple>

// UIMA
#include <uima/api.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/DrawingAnnotator.h>
#include <rs/compare.h>
#include <rs/io/Storage.h>
#include <rs/utils/common.h>

//#undef OUT_LEVEL
//#define OUT_LEVEL OUT_LEVEL_DEBUG

using namespace uima;

class ObjectIdentityResolution : public DrawingAnnotator
{
private:
  typedef double(*matchFunction)(rs::Cluster &cluster, rs::Object &object, double &factor);
  typedef std::tuple<matchFunction, double> matchEntry;
  std::vector<matchEntry> vecMatch;

  std::string host;
  std::string db;
  rs::Storage storage;

  const cv::Rect invalid;
  std::vector<cv::Rect> objectRois;
  bool removeObjects, enableFastMatching;
  double maxDifference;
  uint64_t lastTimestamp;
  sensor_msgs::CameraInfo camInfo;
  tf::Transform viewpoint;
  uint64_t timestamp;

  cv::Mat color;

  template<class T>
  static double matchAnnotation(rs::Cluster &cluster, rs::Object &object, double &factor)
  {
    std::vector<T> annotationsC;
    std::vector<T> annotationsO;
    cluster.annotations.filter(annotationsC);
    object.annotations.filter(annotationsO);

    if(annotationsC.empty())
    {
      outDebug("cluster has no " << TypeTrait<T>::name() << " annotation");
      factor = 0;
      return 1.0;
    }
    if(annotationsO.empty())
    {
      outDebug("object has no " << TypeTrait<T>::name() << " annotation");
      factor = 0;
      return 1.0;
    }

    double dist = rs::compare(annotationsC[0], annotationsO[0]);
    outDebug("matching " << TypeTrait<T>::name() << ": " << dist);

    return dist * factor;
  }

public:
  ObjectIdentityResolution() : DrawingAnnotator(__func__), host(DB_HOST), db(DB_NAME), invalid(-1, -1, -1, -1), removeObjects(true), maxDifference(0.2), lastTimestamp(0)
  {
    //vecMatch.push_back(matchEntry(&matchAnnotation<rs::PoseAnnotation>, 1.0));
    //vecMatch.push_back(matchEntry(&matchAnnotation<rs::TFLocation>,     0.25));
    vecMatch.push_back(matchEntry(&matchAnnotation<rs::Shape>,          1.0));
    vecMatch.push_back(matchEntry(&matchAnnotation<rs::Geometry>,       1.0));
    vecMatch.push_back(matchEntry(&matchAnnotation<rs::SemanticColor>,  1.0));
    vecMatch.push_back(matchEntry(&matchAnnotation<rs::ColorHistogram>, 1.0));
    vecMatch.push_back(matchEntry(&matchAnnotation<rs::Features>,       1.0));
    vecMatch.push_back(matchEntry(&matchAnnotation<rs::PclFeature>,     1.0));
    vecMatch.push_back(matchEntry(&matchAnnotation<rs::Detection>,      1.0));
  }

  /*
   * Initializes annotator
   */
  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    if(ctx.isParameterDefined("removeObjects"))
    {
      ctx.extractValue("removeObjects", removeObjects);
    }
    if(ctx.isParameterDefined("fastmatching"))
    {
      ctx.extractValue("fastmatching", enableFastMatching);
    }
    if(ctx.isParameterDefined("host"))
    {
      ctx.extractValue("host", host);
    }
    if(ctx.isParameterDefined("identitydb"))
    {
      ctx.extractValue("identitydb", db);
    }
    if(ctx.isParameterDefined("maxDifference"))
    {
      float tmp = maxDifference;
      ctx.extractValue("maxDifference", tmp);
      maxDifference = tmp;
    }

    storage = rs::Storage(host, db, false);
    if(removeObjects)
    {
      storage.removeCollection("persistent_objects");
    }
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
    rs::SceneCas cas(tcas);

    if(!cas.get(VIEW_COLOR_IMAGE_HD, color))
    {
      sensor_msgs::CameraInfo camInfo;
      cas.get(VIEW_CAMERA_INFO_HD, camInfo);
      color = cv::Mat::zeros(camInfo.height, camInfo.width, CV_8UC3);
    }

    outDebug("load objects");
    storage.loadCollection(tcas, VIEW_OBJECTS, "persistent_objects");

    outDebug("process clusters");
    processClusters(tcas);

    outDebug("store persistent objects");
    storage.storeCollection(tcas, VIEW_OBJECTS, "persistent_objects");

    return UIMA_ERR_NONE;
  }

  /*
   * Processes new clusters
   */
  void processClusters(CAS &tcas)
  {
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    timestamp = scene.timestamp();

    tf::StampedTransform vp;
    rs::conversion::from(scene.viewPoint(), vp);
    viewpoint = vp.inverse();

    cas.get(VIEW_CAMERA_INFO, camInfo);

    std::vector<rs::Object> objects;
    cas.get(VIEW_OBJECTS, objects);

    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);

    objectRois.clear();
    objectRois.resize(objects.size(), invalid);
    std::vector<cv::Rect> clusterRois(clusters.size());

    for(size_t i = 0; i < clusters.size(); ++i)
    {
      rs::Cluster &cluster = clusters[i];
      if(cluster.rois.has())
      {
        rs::ImageROI image_rois = cluster.rois.get();
        rs::conversion::from(image_rois.roi_hires(), clusterRois[i]);
      }

    }

    if(objects.empty())
    {
      for(size_t i = 0; i < clusters.size(); ++i)
      {
        outDebug("new object for cluster " << i);
        rs::Object object = newObject(clusters[i], tcas);
        objects.push_back(object);
        objectRois.push_back(clusterRois[i]);
      }
    }
    else
    {
      std::vector<int> clustersToObject(clusters.size(), -1), objectsToCluster(objects.size(), -1);
      if(enableFastMatching)
      {
        matchFast(objects, clusters, clustersToObject, objectsToCluster);
      }

      resolveRemaining(objects, clusters, clustersToObject, objectsToCluster);

      for(size_t i = 0; i < objects.size(); ++i)
      {
        rs::Object &object = objects[i];
        object.wasSeen(false);
      }

      for(size_t i = 0; i < clustersToObject.size(); ++i)
      {
        outDebug("check cluster " << i);
        rs::Cluster &cluster = clusters[i];
        if(clustersToObject[i] >= 0)
        {
          outDebug("merging cluster " << i << " with object " << clustersToObject[i]);

          rs::Object &object = objects[clustersToObject[i]];
          outDebug("Object: " << object.id() << " Cluster: " << cluster.id());

          mergeClusterWithObjects(cluster, object);
          object.lastSeen(timestamp);
          object.wasSeen(true);
          objectRois[clustersToObject[i]] = clusterRois[i];
        }
        else
        {
          outDebug("new object for cluster " << i);
          rs::Object object = newObject(cluster, tcas);
          objects.push_back(object);
          objectRois.push_back(clusterRois[i]);
        }
      }
    }
    for(rs::Object &object:objects)
    {
        object.inView.set(checkInView(object));
    }

    cas.set(VIEW_OBJECTS, objects);
  }

  void matchFast(std::vector<rs::Object> &objects, std::vector<rs::Cluster> &clusters, std::vector<int> &clustersToObject, std::vector<int> &objectsToCluster)
  {
    std::vector<double> bestDists(clusters.size(), 0);

    outDebug("running fast matching");

    for(size_t i = 0; i < objects.size(); ++i)
    {
      rs::Object &object = objects[i];
      if(!object.wasSeen())
      {
        outDebug("skipping object " << i << " because is was not seen last frame.");
        continue;
      }

      double bestDist = 0;
      int32_t bestMatch = 0;

      for(size_t j = 0; j < clusters.size(); ++j)
      {
        rs::Cluster &cluster = clusters[j];

        double dist = distanceClusterToObjects(cluster, object);

        if(dist > bestDist)
        {
          bestDist = dist;
          bestMatch = j;
        }
      }
      if(bestDist > 0.9 && bestDists[bestMatch] < bestDist)
      {
        outDebug("object " << i << " fast matches cluster " << bestMatch);
        if(clustersToObject[bestMatch] != -1)
        {
          outDebug("replacing fast match for object " << clustersToObject[bestMatch] << ".");
          objectsToCluster[clustersToObject[bestMatch]] = -1;
        }
        clustersToObject[bestMatch] = i;
        objectsToCluster[i] = bestMatch;
        bestDists[bestMatch] = bestDist;
      }
      else
      {
        outDebug("object " << i << " has no fast match.");
      }
    }
  }

  void resolveRemaining(std::vector<rs::Object> &allObjects, std::vector<rs::Cluster> &allClusters, std::vector<int> &clustersToObject, std::vector<int> &objectsToCluster)
  {
    std::vector<rs::Object> objects;
    std::vector<rs::Cluster> clusters;
    std::vector<size_t> toAllObject;
    std::vector<size_t> toAllCluster;
    cv::Mat similarity;

    for(size_t i = 0; i < clustersToObject.size(); ++i)
    {
      if(clustersToObject[i] == -1)
      {
        outDebug("cluster " << i << " remaining for identity resolution.");
        clusters.push_back(allClusters[i]);
        toAllCluster.push_back(i);
      }
    }

    for(size_t i = 0; i < objectsToCluster.size(); ++i)
    {
      if(objectsToCluster[i] == -1)
      {
        outDebug("object " << i << " remaining for identity resolution.");
        objects.push_back(allObjects[i]);
        toAllObject.push_back(i);
      }
    }

    if(clusters.empty() || objects.empty())
    {
      outDebug("no clusters or objects remaining.");
      return;
    }

    similarity = cv::Mat::zeros(objects.size(), clusters.size(), CV_64F);
    std::vector<size_t> bestObject(clusters.size(), 0);
    std::vector<double> distO(clusters.size(), 0);
    std::vector<size_t> bestCluster(objects.size(), 0);

    outDebug("compute similarity and distance");
    for(size_t i = 0; i < objects.size(); ++i)
    {
      rs::Object &object = objects[i];

      const double lastSeen = (timestamp - (uint64_t)object.lastSeen()) / 1000000000.0;
      const double timeFactor = std::min(lastSeen / 60.0, 1.0);
      const double factorD = 0.8 - 0.6 * timeFactor;
      const double factorS = 1.0 - factorD;
      outDebug(FG_YELLOW "last seen: " << (int)lastSeen << " time factor: " << timeFactor << " dist: " << factorD << " sim: " << factorS);

      double *it = similarity.ptr<double>(i);
      double distC = 0;
      size_t &bestC = bestCluster[i];

      for(size_t j = 0; j < clusters.size(); ++j, ++it)
      {
        rs::Cluster &cluster = clusters[j];

        double sim = similarityClusterToObjects(cluster, object);
        double dist = distanceClusterToObjects(cluster, object);
        double combined = dist * factorD + sim * factorS;

        outDebug(FG_GREEN "object " << toAllObject[i] << " to cluster " << toAllCluster[j] << ": dist: " << dist << " sim: " << sim << " combined: " << combined);
        *it = combined;

        if(combined > distC)
        {
          distC = combined;
          bestC = j;
        }
        if(combined > distO[j])
        {
          distO[j] = combined;
          bestObject[j] = i;
        }
      }
    }

    outDebug("updating identity resolution");
    for(size_t i = 0; i < clusters.size(); ++i)
    {
      if(bestCluster[bestObject[i]] == i && distO[i] > maxDifference)
      {
        clustersToObject[toAllCluster[i]] = toAllObject[bestObject[i]];
        outDebug(FG_MAGENTA "object: " << toAllObject[bestObject[i]] << " to cluster: " << toAllCluster[i]);
      }
      else
      {
        outDebug("cluster " << toAllCluster[i] << " has no match.");
      }
    }
  }

  rs::Object newObject(rs::Cluster &cluster, CAS &tcas)
  {
    rs::Object object = rs::create<rs::Object>(tcas);
    outDebug("Object: " << object.id() << " Cluster: " << cluster.id());

    object.lastSeen(timestamp);
    object.wasSeen(true);
    object.clusters(std::vector<std::string>(1, cluster.id()));

    object.annotations(cluster.annotations());
    if(cluster.points.has())
    {
      object.points(cluster.points());
    }
    if(cluster.centroid.has())
    {
      object.centroid(cluster.centroid());
    }
    if(cluster.rois.has())
    {
      object.rois(cluster.rois());
    }

    return object;
  }

  double similarityClusterToObjects(rs::Cluster &cluster, rs::Object &object) const
  {
    double sum = 0.0;
    double count = 0.0;

    for(size_t i = 0; i < vecMatch.size(); ++i)
    {
      matchFunction f;
      double factor, dist;
      std::tie(f, factor) = vecMatch[i];
      dist = (*f)(cluster, object, factor);
      if(factor > 0)
      {
        sum += dist;
        count += factor;
      }
    }

    return 1.0 - (sum / count);
  }

  double distanceClusterToObjects(rs::Cluster &cluster, rs::Object &object) const
  {
    double factor = 1.0;
    return 1.0 - matchAnnotation<rs::PoseAnnotation>(cluster, object, factor);
  }

  bool checkInView(rs::Object &object) const
  {
    std::vector<rs::PoseAnnotation> poses;
    object.annotations.filter(poses);

    outAssert(!poses.empty(), "no pose found!");

    tf::Stamped<tf::Pose> poseWorld;
    rs::conversion::from(poses[0].world(), poseWorld);

    tf::Transform poseCam = viewpoint * poseWorld;

    tf::Vector3 origin = poseCam.getOrigin();

    const double fx = camInfo.K[0];
    const double fy = camInfo.K[2];
    const double cx = camInfo.K[4] + 0.5;
    const double cy = camInfo.K[5] + 0.5;

    const double z = origin.getZ();

    const double invZ = 1 / z;
    const int x = (fx * origin.getX()) * invZ + cx;
    const int y = (fy * origin.getY()) * invZ + cy;

    return x >= 0 && x < camInfo.width && y >= 0 && y < camInfo.height;
  }

  std::string createDB(const size_t objects, const size_t clusters, const std::vector<double> &similarAppearance, const std::vector<double> &similarPose, const std::vector<uint32_t> &outOfView)
  {
    std::ostringstream db;

    db << "oldClusters={O0";
    for(size_t i = 1; i < objects; ++i)
    {
      db << ",O" << i;
    }
    db << '}' << std::endl;

    db << "newClusters={N0";
    for(size_t i = 1; i < clusters; ++i)
    {
      db << ",N" << i;
    }
    db << '}' << std::endl;

    for(size_t o = 0, idx = 0; o < objects; ++o)
    {
      for(size_t n = 0; n < clusters; ++n, ++idx)
      {
        db << similarAppearance[idx] << " looksSimilar(O" << o << ",N" << n << ')' << endl;
        db << similarPose[idx] << " samePos(O" << o << ",N" << n << ')' << endl;
      }
    }

    for(size_t i = 0; i < outOfView.size(); ++i)
    {
      db << "outOfView(O" << outOfView[i] << ')' << endl;
    }
    outDebug(std::endl << db.str());
    return db.str();
  }

  std::string getTypeName(rs::Annotation &annotation)
  {
    FeatureStructure &fs = (FeatureStructure &)annotation;
    Type fsType = fs.getType();
    const std::string &typeName = fsType.getName().asUTF8();

    Feature feat = fsType.getFeatureByBaseName("source");
    if(fsType.isAppropriateFeature(feat))
    {
      const std::string &source = fs.getStringValue(feat).asUTF8();
      return typeName + '_' + source;
    }
    else
    {
      return typeName;
    }
  }

  void mergeClusterWithObjects(rs::Cluster &cluster, rs::Object &object)
  {
    std::set<std::string> newAnnotations;
    std::vector<rs::Annotation> annotationsC = cluster.annotations();
    std::vector<rs::Annotation> annotationsO = object.annotations();
    object.annotations.allocate();

    for(size_t i = 0; i < annotationsC.size(); ++i)
    {
      const std::string &name = getTypeName(annotationsC[i]);
      newAnnotations.insert(name);
      object.annotations.append(annotationsC[i]);
    }

    for(size_t i = 0; i < annotationsO.size(); ++i)
    {
      const std::string &name = getTypeName(annotationsO[i]);
      if(newAnnotations.find(name) == newAnnotations.end())
      {
        object.annotations.append(annotationsO[i]);
      }
    }

    std::vector<std::string> clusters = object.clusters();
    clusters.push_back(cluster.id());
    object.clusters(clusters);

    if(cluster.points.has())
    {
      object.points(cluster.points());
    }
    if(cluster.centroid.has())
    {
      object.centroid(cluster.centroid());
    }
    if(cluster.rois.has())
    {
      object.rois(cluster.rois());
    }
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    if(color.empty())
    {
      return;
    }

    disp = color.clone();

    for(size_t i = 0; i < objectRois.size(); ++i)
    {
      const cv::Rect &roi = objectRois[i];

      if(roi == invalid)
      {
        continue;
      }

      std::ostringstream oss;
      oss << "Object: " << i;

      cv::rectangle(disp, roi, rs::common::cvScalarColors[i % rs::common::numberOfColors]);
      cv::putText(disp, oss.str(), roi.tl() + cv::Point(5, -15), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, rs::common::cvScalarColors[i % rs::common::numberOfColors], 1, CV_AA);
    }
  }

};

MAKE_AE(ObjectIdentityResolution)
