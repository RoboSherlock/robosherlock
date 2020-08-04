#ifndef OBJECTNAMEMAPITEM_H
#define OBJECTNAMEMAPITEM_H

#endif // OBJECTNAMEMAPITEM_H

#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/mark.h>
#include <yaml-cpp/yaml.h>

using namespace std;
struct ObjectNameMapItem
{
    std::string category;
    std::vector<std::string> color;
    std::vector<std::string> material;
    std::vector<std::string> shape;
    std::string folderPath;
    std::string filePath;
    std::string packageName;
    float scale;
    bool fromPackage;
    bool openability;
    std::vector<int> axisMap;
};




  
  
  
void readObjectNameMapItem(std::string filename, std::vector<std::string>& target_object_names, std::vector<ObjectNameMapItem>& object_name_map_items){

    YAML::Node config = YAML::LoadFile(filename);
    if (config["names"])
        {
          target_object_names = config["names"].as<std::vector<std::string>>();
          object_name_map_items.resize(target_object_names.size());
          for (size_t i = 0; i < target_object_names.size(); ++i)
          {
            ObjectNameMapItem& item = object_name_map_items[i];
            YAML::Node entry = config[target_object_names[i]];
            item.category = entry["category"].as<std::string>();
            item.color = entry["color"].as<std::vector<std::string>>();
            item.shape = entry["shape"].as<std::vector<std::string>>();
            item.material = entry["material"].as<std::vector<std::string>>();
            item.openability=entry["openability"].as<bool>();
            item.packageName = entry["packageName"].as<std::string>();
            item.folderPath = entry["folderPath"].as<std::string>();
            item.filePath = entry["filePath"].as<std::string>();
            item.fromPackage=entry["fromPackage"].as<bool>();
            item.scale=entry["scale"].as<float>();
            item.axisMap = entry["axisMap"].as<std::vector<int>>();
          }
   }
}
