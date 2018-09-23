#ifndef YAMLTOXMLCONVERTER_HPP
#define YAMLTOXMLCONVERTER_HPP

#include <fstream>
#include <string>
#include <sstream>

#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/mark.h>
#include <yaml-cpp/yaml.h>

#include <rs/utils/output.h>
#include <rs/utils/common.h>

using namespace std;

class YamlToXMLConverter {

public:

    YamlToXMLConverter(std::string path);
    YamlToXMLConverter(const YamlToXMLConverter&) = delete;
    YamlToXMLConverter& operator=(const YamlToXMLConverter&) = delete;

    void parseYamlFile();
    void setAEName(string name);
    void setFrameImpl(string name);
    void setHeader(string name);
    void getXml(ofstream& out);

    rs::AnnotatorCapabilities getAnnotatorCapabilities();

    string yamlPath;

private:

    YAML::Node config;

    rs::AnnotatorCapabilities annotCap;

    string header;
    string AEName;
    string AEDescription;
    string AEImpl;
    string taeDesp;
    string frameImpl;

    string configParams;
    string configParamSettings;
    string capabilities;

    string getType(const YAML::Node& node);
    string getTypeFilePath();

    bool parseAnnotatorInfo(const YAML::Node& node);
    bool parseConfigParamInfo(const YAML::Node& node);
    bool parseCapabInfo(const YAML::Node& node);

};

#endif
