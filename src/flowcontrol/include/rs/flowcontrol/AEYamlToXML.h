#ifndef AEYAMLTOXMLCONVERTER_HPP
#define AEYAMLTOXMLCONVERTER_HPP

#include <fstream>
#include <string>
#include <sstream>

#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/mark.h>
#include <yaml-cpp/yaml.h>

using namespace std;

class AEYamlToXMLConverter {

public:

    AEYamlToXMLConverter(std::string path);
    AEYamlToXMLConverter(const AEYamlToXMLConverter&) = delete;
    AEYamlToXMLConverter& operator=(const AEYamlToXMLConverter&) = delete;

    bool isInAEList(const string value);
    void parseYamlFile();
    void setAEName(string name);
    void setFrameImpl(string name);
    void setHeader(string name);

    void getOutput(ofstream& out);

    string yamlPath;

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
    string flowConstraints;
    string fsIndexCollection;
    string capabilities;

    string getType(const YAML::Node& node);
    string getTypeFilePath();

    bool genAEInfo(const YAML::Node& node);
    bool genConfigParamInfo(const YAML::Node& node, const string analysisEngineName);
    bool genFlowConstraints(const YAML::Node& node);
    bool genFsIndexCollection(const YAML::Node& node);
    bool genCapabInfo(const YAML::Node& node);
};
#endif
