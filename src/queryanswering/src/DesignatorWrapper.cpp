#include <rs/queryanswering/DesignatorWrapper.h>
#include "rapidjson/pointer.h"

namespace rs
{

rapidjson::Document *DesignatorWrapper::req_designator = NULL;
rapidjson::Document *DesignatorWrapper::res_designator = NULL;

DesignatorWrapper::DesignatorWrapper()
{
  if(!req_designator)
  {
    req_designator = new rapidjson::Document();
  }
  if(!res_designator)
  {
    res_designator = new rapidjson::Document();
  }
  tcas = NULL;
}

DesignatorWrapper::~DesignatorWrapper()
{
  mode = CLUSTER;
}

DesignatorWrapper::DesignatorWrapper(uima::CAS *cas) : tcas(cas)
{
  if(!req_designator)
  {
    req_designator = new rapidjson::Document();
  }
  if(!res_designator)
  {
    res_designator = new rapidjson::Document();
  }
  mode = CLUSTER;
}


void DesignatorWrapper::setCAS(uima::CAS *cas)
{
  tcas = cas;
}

void DesignatorWrapper::setMode(DesignatorProcessMode m)
{
  mode = m;
}

bool DesignatorWrapper::getObjectDesignators(std::vector<std::string> &objectDesignators)
{
  if(!tcas)
  {
    std::cout << "NULL Pointer in DesignatorWrapper::getDesignatorResponse. tcas is not set! Use DesignatorWrapper::setCAS before calling this method" << std::endl;
    return false;
  }

  rs::SceneCas cas(*tcas);
  rs::Scene scene = cas.getScene();

  now = scene.timestamp();


  std::vector<rs::HandleAnnotation> handles;
  std::vector<rs::ARMarker> arMarkers;
  scene.annotations.filter(handles);
  scene.annotations.filter(arMarkers);
  for(int i = 0; i < handles.size(); i++)
  {
    rapidjson::Document objDesig;
    objDesig.SetObject();
    convert(handles[i], objDesig);
    objectDesignators.push_back(jsonToString(objDesig));
  }
  for(int i = 0; i < arMarkers.size(); i++)
  {
    rapidjson::Document objDesig;
    objDesig.SetObject();
    convert(arMarkers[i], objDesig);
    objectDesignators.push_back(jsonToString(objDesig));
  }

  if(mode == CLUSTER)
  {
    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);
    process(clusters, objectDesignators);
  }
  else
  {
    std::vector<rs::Object> allObjects, objects;
    cas.get(VIEW_OBJECTS, allObjects);
    outWarn("objects found: " << allObjects.size());
    for(size_t i = 0; i < allObjects.size(); ++i)
    {
      rs::Object &object = allObjects[i];
      double lastSeen = (now - (uint64_t)object.lastSeen()) / 1000000000.0;
      if(lastSeen == 0)
      {
        objects.push_back(object);
      }
    }
    process(objects, objectDesignators);
  }
  return true;
}

void DesignatorWrapper::convert(rs::Cluster &input, const size_t id, rapidjson::Document *object)
{
  object->AddMember("id", std::to_string(id), object->GetAllocator());
}

void DesignatorWrapper::convert(rs::Object &input, const size_t id, rapidjson::Document *object)
{
  //  rapidjson::Document *valuePair = new rapidjson::Document("RESOLUTION");
  //  valuePair->setValue("ID", id);
  //  valuePair->setValue("LASTSEEN", now - input.lastSeen());
  //  object->addChild(valuePair);
  object->AddMember("id", std::to_string(id), object->GetAllocator());
  object->AddMember("uid", input.uid(), object->GetAllocator());
}

void DesignatorWrapper::convert(rs::Detection &input, rapidjson::Document *object)
{
  rapidjson::Value nestedValue;
  nestedValue.SetObject();
  nestedValue.AddMember("confidence", input.confidence(), object->GetAllocator());
  nestedValue.AddMember("source", input.source(), object->GetAllocator());
  nestedValue.AddMember("name", input.name(), object->GetAllocator());

  object->AddMember("detection", nestedValue, object->GetAllocator());
}

void DesignatorWrapper::convert(rs::Classification &input, rapidjson::Document *object)
{
  rapidjson::Value nestedValue;
  nestedValue.SetObject();
  nestedValue.AddMember("class-name", input.classname(), object->GetAllocator());
  nestedValue.AddMember("source", input.source(), object->GetAllocator());
  std::vector<rs::ClassConfidence> confs = input.confidences();
  for(auto c : confs)
    if(c.name() == input.classname())
      nestedValue.AddMember("confidence", c.score(), object->GetAllocator());

  object->AddMember("class", nestedValue, object->GetAllocator());
}

void DesignatorWrapper::convert(rs::TFLocation &input, rapidjson::Document *object)
{
  rapidjson::Value nestedValue;
  nestedValue.SetObject();
  nestedValue.AddMember("relation", input.reference_desc(), object->GetAllocator());
  nestedValue.AddMember("frame", input.frame_id(), object->GetAllocator());

  object->AddMember("location", nestedValue, object->GetAllocator());
}

void DesignatorWrapper::convert(rs::Segment &input, rapidjson::Document *object)
{
  rapidjson::Value nestedValue, segment;
  nestedValue.SetObject();
  nestedValue.AddMember("width", input.lengthX(), object->GetAllocator());
  nestedValue.AddMember("height", input.lengthY(), object->GetAllocator());

  segment.SetObject();
  segment.AddMember("dimensions-2D", nestedValue, object->GetAllocator());

  object->AddMember("segment", segment, object->GetAllocator());
}

void DesignatorWrapper::convert(geometry_msgs::PoseStamped &pose, rapidjson::Document *object, rapidjson::MemoryPoolAllocator<> &alloc)
{

  rapidjson::Value headerKey("header", alloc);
  rapidjson::Value headerVal(rapidjson::kObjectType);
  headerVal.AddMember("seq", pose.header.seq, alloc);

  rapidjson::Value stampKey("stamp", alloc);
  rapidjson::Value stampVal(rapidjson::kObjectType);
  stampVal.AddMember("sec", pose.header.stamp.sec, alloc);
  stampVal.AddMember("nsec", pose.header.stamp.nsec, alloc);

  headerVal.AddMember(stampKey, stampVal, alloc);
  headerVal.AddMember("frame_id", pose.header.frame_id, alloc);

  rapidjson::Value poseKey("pose", alloc);
  rapidjson::Value poseVal(rapidjson::kObjectType);

  rapidjson::Value positionKey("position", alloc);
  rapidjson::Value positionVal(rapidjson::kObjectType);

  positionVal.AddMember("x", pose.pose.position.x, alloc);
  positionVal.AddMember("y", pose.pose.position.y, alloc);
  positionVal.AddMember("z", pose.pose.position.z, alloc);

  poseVal.AddMember(positionKey, positionVal, alloc);

  rapidjson::Value orientationKey("orientation", alloc);
  rapidjson::Value orientationVal(rapidjson::kObjectType);
  orientationVal.AddMember("x", pose.pose.orientation.x, alloc);
  orientationVal.AddMember("y", pose.pose.orientation.y, alloc);
  orientationVal.AddMember("z", pose.pose.orientation.z, alloc);
  orientationVal.AddMember("w", pose.pose.orientation.w, alloc);

  poseVal.AddMember(orientationKey, orientationVal, alloc);


  object->AddMember(headerKey, headerVal, alloc);
  object->AddMember(poseKey, poseVal, alloc);
}

void DesignatorWrapper::convert(rs::Geometry &input, rapidjson::Document *object)
{
  rs::BoundingBox3D bb = input.boundingBox();

  tf::Stamped<tf::Pose> tf_stamped_pose;
  geometry_msgs::PoseStamped pose;
  rs::conversion::from(input.camera(), tf_stamped_pose);
  tf::poseStampedTFToMsg(tf_stamped_pose, pose);

  rapidjson::Value nestedValue;
  rapidjson::Document boundingBox;
  boundingBox.SetObject();
  nestedValue.SetObject();
  nestedValue.AddMember("width", bb.width(), object->GetAllocator());
  nestedValue.AddMember("height", bb.height(), object->GetAllocator());
  nestedValue.AddMember("depth", bb.depth(), object->GetAllocator());

  boundingBox.AddMember("dimensions-3D", nestedValue, object->GetAllocator());
  boundingBox.AddMember("size", input.size(), object->GetAllocator());
  boundingBox.AddMember("dist-to-plane", input.distanceToPlane(), object->GetAllocator());

  rapidjson::Document jsonPose;
  jsonPose.SetObject();
  convert(pose, &jsonPose, object->GetAllocator());
  rapidjson::Value poseKey("pose", object->GetAllocator());
  boundingBox.AddMember(poseKey, jsonPose, object->GetAllocator());

  rapidjson::Value boundingKey("boundingbox", object->GetAllocator());
  object->AddMember(boundingKey, boundingBox, object->GetAllocator());

}

void DesignatorWrapper::convert(rs::Shape &input, rapidjson::Document *object)
{
  if(!object->HasMember("shape"))
  {
    rapidjson::Pointer("/shape/0").Set(*object, input.shape());
  }
  else
  {
    rapidjson::Value &array = (*object)["shape"];
    std::string size = std::to_string(array.Size());
    outInfo(size);
    rapidjson::Pointer("/shape/" + size).Set(*object, input.shape());
  }
  //object->AddMember("shape",input.shape(),object->GetAllocator());
}

void DesignatorWrapper::convert(rs::PoseAnnotation &input, rapidjson::Document *object)
{
  tf::Stamped<tf::Pose> tf_stamped_pose;
  geometry_msgs::PoseStamped pose_stamped_msgs;
  rs::conversion::from(input.camera(), tf_stamped_pose);

  tf::poseStampedTFToMsg(tf_stamped_pose, pose_stamped_msgs);

  rapidjson::Document nestedValue(&object->GetAllocator());
  nestedValue.SetObject();
  nestedValue.AddMember("source", input.source(), object->GetAllocator());

  rapidjson::Document poseJsonObj(&object->GetAllocator());
  poseJsonObj.SetObject();
  convert(pose_stamped_msgs, &poseJsonObj, object->GetAllocator());
  nestedValue.AddMember("pose_stamped", poseJsonObj, object->GetAllocator());

  uint64_t diff = now - tf_stamped_pose.stamp_.toNSec();
  outWarn("Time diff in poses: " << diff);
  if(diff == 0)
  {
    if(!object->HasMember("poses"))
    {
      rapidjson::Pointer("/poses/0").Set(*object, nestedValue);
    }
    else
    {
      rapidjson::Value &array = (*object)["poses"];
      std::string size = std::to_string(array.Size());
      rapidjson::Pointer("/poses/" + size).Set(*object, nestedValue);
    }
    //object->AddMember("pose", nestedValue, object->GetAllocator());
  }
}

void DesignatorWrapper::convert(rs::SemanticColor &input, rapidjson::Document *object)
{
  const std::vector<std::string> &colors = input.color();
  const std::vector<float> &ratios = input.ratio();

  rapidjson::Value nestedValue;
  nestedValue.SetObject();

  for(size_t i = 0; i < colors.size(); ++i)
  {
    const float &ratio = ratios[i];
    if(ratio > 0.1)
    {
      rapidjson::Value v(colors[i].c_str(), object->GetAllocator());
      nestedValue.AddMember(v, ratio, object->GetAllocator());
    }
  }
  object->AddMember("color", nestedValue, object->GetAllocator());
}

void DesignatorWrapper::convert(rs::MLNAtoms &input, rapidjson::Document *object)
{
  const std::vector<std::string> &atoms = input.atoms();

  rapidjson::Document *nestedValue = new rapidjson::Document();

  rapidjson::Value atomArray;
  atomArray.SetArray();

  for(size_t i = 0; i < atoms.size(); ++i)
  {
    std::stringstream id;
    rapidjson::Value atom;
    atom.SetObject();
    id << "atom_ " << i;
    rapidjson::Value v(id.str().c_str(), nestedValue->GetAllocator());
    rapidjson::Value value(atoms[i], nestedValue->GetAllocator());
    atom.AddMember(v, value, nestedValue->GetAllocator());
    atomArray.PushBack(atom, nestedValue->GetAllocator());
  }
  nestedValue->AddMember("atom", atomArray, nestedValue->GetAllocator());
  mergeJson(*object, *nestedValue, "mln-atoms");
}

void DesignatorWrapper::convert(rs::NamedLink &input, rapidjson::Document *object)
{
  rapidjson::Value nestedValue;
  nestedValue.SetObject();
  nestedValue.AddMember("name", input.name(), object->GetAllocator());
  nestedValue.AddMember("url", input.url(), object->GetAllocator());
  object->AddMember("namedlink", nestedValue, object->GetAllocator());
}

void DesignatorWrapper::convert(rs::Goggles &input, rapidjson::Document *object)
{
  rapidjson::Document nestedValue;
  nestedValue.SetObject();
  nestedValue.AddMember("category", input.category(), nestedValue.GetAllocator());
  nestedValue.AddMember("title", input.title(), nestedValue.GetAllocator());
  nestedValue.AddMember("preview-link", input.preview_link(), nestedValue.GetAllocator());

  std::vector<rs::NamedLink> namedlinks = input.links();
  rapidjson::Document *links = new rapidjson::Document();
  convertAll(namedlinks, links);

  mergeJson(nestedValue, *links, "namedlinks");
  mergeJson(*object, nestedValue, "goggles");
}

void DesignatorWrapper::convert(rs::Features &input, rapidjson::Document *object)
{
  std::vector<rs::Response> resps = input.response();
  if(resps.empty())
  {
    return;
  }
  rs::Response &resp = resps[0];

  std::vector<std::string> classes = resp.classNames();
  cv::Mat respM;
  rs::conversion::from(resp.response.get(), respM);
  if((size_t)respM.at<float>(0) > classes.size())
  {
    return;
  }

  object->AddMember("response", classes[(size_t)respM.at<float>(0)], object->GetAllocator());
}

void DesignatorWrapper::convert(rs::ClusterPart &input, rapidjson::Document *object)
{
  rapidjson::Document nestedValue;
  nestedValue.SetObject();

  tf::Stamped<tf::Pose> tf_stamped_pose;
  geometry_msgs::PoseStamped pose;
  rs::conversion::from(input.pose(), tf_stamped_pose);
  tf::poseStampedTFToMsg(tf_stamped_pose, pose);
  nestedValue.AddMember("name", input.name(), nestedValue.GetAllocator());

  char poseJson [300];
  std::sprintf(poseJson, "{\"header\":{\"seq\":%d,\"stamp\":{\"sec\":%d,\"nsec\":%d},\"frame_id\":\"%s\"},\"pose\":{\"position\":{\"x\":%f,\"y\":%f,\"z\":%f},\"orientation\":{\"x\":%f,\"y\":%f,\"z\":%f,\"w\":%f}}}",
               pose.header.seq, pose.header.stamp.sec, pose.header.stamp.nsec, pose.header.frame_id.c_str(),
               pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  rapidjson::Document poseJsonObj;
  poseJsonObj.Parse(poseJson);
  mergeJson(nestedValue, poseJsonObj, "pose");
  mergeJson(*object, nestedValue, std::to_string(input.clID()));
}

/*void DesignatorWrapper::convert(rs_demos::Volume &input, rapidjson::Document *object)
{
  object->AddMember("volume",input.volume(),object->GetAllocator());
}*/

/*void DesignatorWrapper::convert(rs_demos::Substance &input, rapidjson::Document *object)
{
  rapidjson::Value substance;
  substance.SetObject();
  substance.AddMember("substance",input.substanceName(),object->GetAllocator());
  object->AddMember("contains",substance,object->GetAllocator());
}*/

iai_robosherlock_msgs::PerceivedObjects DesignatorWrapper::getObjectsMsgs()
{
  return objects_;
}


void DesignatorWrapper::convert(rs::ARMarker &input, rapidjson::Document &arDesignator)
{
  arDesignator.AddMember("type", "armarker", arDesignator.GetAllocator());
  arDesignator.AddMember("id", input.name(), arDesignator.GetAllocator());

  tf::Stamped<tf::Pose> tf_stamped_pose;
  geometry_msgs::PoseStamped pose;
  rs::conversion::from(input.pose(), tf_stamped_pose);
  tf::poseStampedTFToMsg(tf_stamped_pose, pose);

  char *poseJson = NULL;
  std::sprintf(poseJson, "{\"header\":{\"seq\":%d,\"stamp\":{\"sec\":%d,\"nsec\":%d},\"frame_id\":\"%s\"},\"pose\":{\"position\":{\"x\":%f,\"y\":%f,\"z\":%f},\"orientation\":{\"x\":%f,\"y\":%f,\"z\":%f,\"w\":%f}}}",
               pose.header.seq, pose.header.stamp.sec, pose.header.stamp.nsec, pose.header.frame_id.c_str(),
               pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  rapidjson::Document poseJsonObj;
  poseJsonObj.Parse(poseJson);
  mergeJson(arDesignator, poseJsonObj, "pose");
}

void DesignatorWrapper::convert(rs::HandleAnnotation &input,
                                rapidjson::Document &handleDesignator)
{
  handleDesignator.AddMember("handle", input.name(), handleDesignator.GetAllocator());
  tf::Stamped<tf::Pose> tf_stamped_pose;
  geometry_msgs::PoseStamped pose;
  rs::conversion::from(input.pose(), tf_stamped_pose);
  tf::poseStampedTFToMsg(tf_stamped_pose, pose);

  char poseJson [300];
  std::sprintf(poseJson, "{\"header\":{\"seq\":%d,\"stamp\":{\"sec\":%d,\"nsec\":%d},\"frame_id\":\"%s\"},\"pose\":{\"position\":{\"x\":%f,\"y\":%f,\"z\":%f},\"orientation\":{\"x\":%f,\"y\":%f,\"z\":%f,\"w\":%f}}}",
               pose.header.seq, pose.header.stamp.sec, pose.header.stamp.nsec, pose.header.frame_id.c_str(),
               pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  rapidjson::Document poseJsonObj;
  poseJsonObj.Parse(poseJson);
  mergeJson(handleDesignator, poseJsonObj, "pose");
}

template<>
void DesignatorWrapper::convertAll(std::vector<rs::ClusterPart> &all, rapidjson::Document *object)
{
  if(!all.empty())
  {
    rapidjson::Document *objParts = new rapidjson::Document();
    for(rs::ClusterPart input : all)
    {
      convert(input, objParts);
    }

    mergeJson(*object, *objParts, "obj-part");
  }
}

void DesignatorWrapper::mergeJson(rapidjson::Document &destination, rapidjson::Document &source, std::string fieldName)
{
  rapidjson::Value fieldNameV(fieldName, destination.GetAllocator());
  destination.AddMember(fieldNameV, source, destination.GetAllocator());
}

std::string DesignatorWrapper::jsonToString(rapidjson::Value &res)
{
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  res.Accept(writer);
  return buffer.GetString();
}
}
