#pragma once
#ifndef DESIGNATORSTORAGE_H
#define DESIGNATORSTORAGE_H


#include <uima/api.hpp>

#include <rs/scene_cas.h>
#include <rs/utils/output.h>

#include <rs/types/all_types.h>

#include <boost/bind.hpp>
#include <boost/signals2.hpp>
#include <map>

#include <iai_robosherlock_msgs/PerceivedObjects.h>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/allocators.h>
#include <rapidjson/prettywriter.h>

namespace rs
{

class DesignatorWrapper
{
public:

  enum DesignatorProcessMode
  {
    CLUSTER = 0,
    OBJECT
  };

  DesignatorProcessMode mode;
  // Pointer to the one persistent object
  static rapidjson::Document *req_designator;
  static rapidjson::Document *res_designator;

  //designator_integration_msgs::DesignatorResponse res;
  iai_robosherlock_msgs::PerceivedObjects objects_;

  uint64_t now;
  uima::CAS *tcas;

  DesignatorWrapper();

  DesignatorWrapper(uima::CAS *cas);

  void setCAS(uima::CAS *cas);
  void setMode(DesignatorProcessMode m);

  virtual ~DesignatorWrapper();

  void filterClusters(const std::vector<rs::Cluster> input, const rapidjson::Document *out) const;
  void updateDesignator();

  //  void notifyObserversDesignatorAdded(Designator d);

  //designator_integration_msgs::DesignatorResponse getDesignatorResponseMsg();
  iai_robosherlock_msgs::PerceivedObjects getObjectsMsgs();

  bool getObjectDesignators(std::vector<std::string> &);

  template<class T>
  void process(std::vector<T> &elements, std::vector<std::string> &objectDesignators)
  {
    objects_.objects.clear();
    for(size_t i = 0; i < elements.size(); ++i)
    {
      outDebug("reading object: " << i);

      T &element = elements[i];
      rapidjson::Document objectDesignator;
      objectDesignator.SetObject();

      double seconds = now / 1000000000;
      double nsec = (now % 1000000000) / 1000000000.0;

      outDebug("Seconds: " << std::setprecision(12) << seconds);
      outDebug("NanoSecs: " << std::setprecision(12) << nsec);
      double time = seconds + nsec;

      outDebug("time: " << std::setprecision(20) << time);
      objectDesignator.AddMember("timestamp",time,objectDesignator.GetAllocator());

      convert(element, i, &objectDesignator);

      std::vector<rs::Geometry> geometry;
      std::vector<rs::PoseAnnotation> poses;
      std::vector<rs::Detection> detections;
      std::vector<rs::SemanticColor> semanticColors;
      std::vector<rs::Shape> shapes;
      std::vector<rs::TFLocation> locations;
      std::vector<rs::MLNAtoms> atoms;
      std::vector<rs::Goggles> goggles;
      std::vector<rs::Features> features;
      std::vector<rs::ClusterPart> clusterParts;
      std::vector<rs::Classification> classification;

      element.annotations.filter(geometry);
      element.annotations.filter(poses);
      element.annotations.filter(detections);
      element.annotations.filter(semanticColors);
      element.annotations.filter(shapes);
      element.annotations.filter(locations);
      element.annotations.filter(atoms);
      element.annotations.filter(goggles);
      element.annotations.filter(features);
      element.annotations.filter(clusterParts);
      element.annotations.filter(classification);


      convertAll(geometry, &objectDesignator);
      convertAll(detections, &objectDesignator);
      convertAll(poses, &objectDesignator);
      convertAll(semanticColors, &objectDesignator);
      convertAll(shapes, &objectDesignator);
      convertAll(locations, &objectDesignator);
      convertAll(atoms, &objectDesignator);
      convertAll(goggles, &objectDesignator);
      convertAll(features, &objectDesignator);
      convertAll(clusterParts, &objectDesignator);
      convertAll(classification, &objectDesignator);


      iai_robosherlock_msgs::PerceivedObject object;
      if(!detections.empty() && !poses.empty())
      {
        rs::Detection a = detections[0];
        rs::PoseAnnotation b = poses[0];
        object.id = a.name();
        tf::Stamped<tf::Pose> pose;
        rs::conversion::from(b.camera(), pose);
        geometry_msgs::PoseStamped pose_stamped_msgs;

        tf::poseStampedTFToMsg(pose, pose_stamped_msgs);
        object.transform = pose_stamped_msgs;
        objects_.objects.push_back(object);
      }

      outDebug("no. of children: " << objectDesignator.MemberCount());

      if(objectDesignator.MemberCount() > 0)
      {
        outInfo("Object as json:");

        rapidjson::StringBuffer buffer;
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
        objectDesignator.Accept(writer);
        outInfo(buffer.GetString());
        objectDesignators.push_back(jsonToString(objectDesignator));
      }
    }
  }

  // Converter methods to take scene annotations and map them to Designator Keywords
  void convert(rs::Cluster &input, const size_t id, rapidjson::Document *object);
  void convert(rs::Object &input, const size_t id, rapidjson::Document *object);

  // TODO How can i define this in the cpp file?
  // If i do it naively, i get a runtime error that the method can't be found + the mangeled method name
  template<class T>
  void convertAll(std::vector<T> &all, rapidjson::Document *object)
  {
    for(T input : all)
    {
      convert(input, object);
    }
  }

  void convert(rs::Detection &input, rapidjson::Document *object);
  void convert(rs::Classification &input, rapidjson::Document *object);

  void convert(rs::TFLocation &input, rapidjson::Document *object);
  void convert(rs::Segment &input, rapidjson::Document *object);
  void convert(rs::Geometry &input, rapidjson::Document *object);
  void convert(rs::Shape &input, rapidjson::Document *object);
  void convert(rs::PoseAnnotation &input, rapidjson::Document *object);
  void convert(rs::SemanticColor &input, rapidjson::Document *object);
  void convert(rs::MLNAtoms &input, rapidjson::Document *object);
  void convert(rs::NamedLink &input, rapidjson::Document *object);
  void convert(rs::Goggles &input, rapidjson::Document *object);
  void convert(rs::Features &input, rapidjson::Document *object);
  void convert(rs::ClusterPart &input, rapidjson::Document *object);
  void convert(geometry_msgs::PoseStamped &pose, rapidjson::Document *object, rapidjson::MemoryPoolAllocator<>& alloc);
  void convert(rs::ARMarker &input, rapidjson::Document &res);
  void convert(rs::HandleAnnotation &input, rapidjson::Document &res);



  static void mergeJson (rapidjson::Document &destination, rapidjson::Document &source, std::string fieldName);
  static std::string jsonToString(rapidjson::Value &res);

};

}

#endif // DESIGNATORSTORAGE_H
