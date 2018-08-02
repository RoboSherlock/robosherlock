#ifndef YAMLTOXMLCONVERTER_HPP
#define YAMLTOXMLCONVERTER_HPP

#include <fstream>
#include <string>
#include <sstream>

#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/yaml.h>

using namespace std;

class YamlToXMLConverter {

public:

    YamlToXMLConverter(std::string path);
    YamlToXMLConverter(const YamlToXMLConverter&) = delete;
    YamlToXMLConverter& operator=(const YamlToXMLConverter&) = delete;

    bool parseYamlFile();
    void setAEName(string name);
    void setFrameImpl(string name);
    void setHeader(string name);

    void getOutput(ofstream& out);

private:

    YAML::Node config;

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

    bool genAnnotatorInfo(const YAML::Node& node);
    bool genConfigParamInfo(const YAML::Node& node);
    bool genCapabInfo(const YAML::Node& node);
};

#endif
