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

class YamlToXMLConverter
{

public:

  enum class YAMLType {AAE, Annotator} type_;

  YamlToXMLConverter(std::string path, YamlToXMLConverter::YAMLType type);
  YamlToXMLConverter(const YamlToXMLConverter &) = delete;
  YamlToXMLConverter &operator=(const YamlToXMLConverter &) = delete;

  void parseYamlFile();
  void setAEName(string name);
  void setFrameImpl(string name);
  void setHeader(string name);


  void getDelegates(vector<string> &delegates_);
  rs::AnnotatorCapabilities getAnnotatorCapabilities();
  std::vector<rs::AnnotatorCapabilities> getOverwrittenAnnotatorCapabilities();

  bool isInAEList(const string value);

  friend std::ostream &operator<<(std::ostream &out, const YamlToXMLConverter &object)
  {
    if(object.type_ == YamlToXMLConverter::YAMLType::Annotator) {
      out << object.header << endl;
      out << "<taeDescription xmlns=\"" << object.taeDesp << "\">" << endl;
      out << "<frameworkImplementation>" << object.frameImpl << "</frameworkImplementation>" << endl;
      out << "<primitive>true</primitive>" << endl;
      out << "<annotatorImplementationName>" << object.AEImpl << "</annotatorImplementationName>" << endl;
      out << "<analysisEngineMetaData>" << endl;
      out << "<name>" << object.AEName << "</name>" << endl;
      out << "<description>" << object.AEDescription << "</description>" << endl;
      out << "<version>1.0</version>\n<vendor/>" << endl;
      out << endl;
      out << object.configParams << endl;
      out << object.configParamSettings << endl;
      out << object.capabilities << endl;

      string typePath;
      try {
        typePath = object.getTypeFilePath();
      }
      catch(std::runtime_error &e) {
        throw e;
      }

      out << "<typeSystemDescription>\n<imports>\n<import location=" << "\"" << typePath << "\"/>\n</imports>\n</typeSystemDescription>\n" << endl;
      out << "<operationalProperties>\n<modifiesCas>true</modifiesCas>\n<multipleDeploymentAllowed>true</multipleDeploymentAllowed>\n<outputsNewCASes>false</outputsNewCASes>\n</operationalProperties>\n" << endl;
      out << "</analysisEngineMetaData>" << endl;
      out << "</taeDescription>" << endl;
    }
    else {
      out << object.header << endl;
      out << "<taeDescription xmlns=\"" << object.taeDesp << "\">\n";
      out << "  <frameworkImplementation>" << object.frameImpl << "</frameworkImplementation>\n";
      out << "  <primitive>false</primitive>\n";
      out << "  <analysisEngineMetaData>\n";
      out << "    <name>" << object.AEName << "</name>\n";
      out << "    <description>" << object.AEDescription << "</description>\n";
      out << "    <version>1.0</version>\n";
      out << "    <vendor/>\n";
      out << object.configParams << "\n";
      out << object.configParamSettings << "\n";
      out << object.flowConstraints << "\n";
      //Declare this type for the usability of future code;
      out << "    <typePriorities/>\n";
      out << object.fsIndexCollection << "\n";
      out << object.capabilities << "\n";

      out << "    <operationalProperties>\n";
      out << "      <modifiesCas>true</modifiesCas>\n";
      out << "      <multipleDeploymentAllowed>true</multipleDeploymentAllowed>\n";
      out << "      <outputsNewCASes>false</outputsNewCASes>\n";
      out << "    </operationalProperties>\n";
      out << "  </analysisEngineMetaData>\n";
      out << "</taeDescription>\n";
    }
    return out;
  }


private:

  YAML::Node config;

  string yamlPath;

  string header;
  string AEName;
  string AEDescription;
  string AEImpl;
  string taeDesp;
  string frameImpl;

  string configParams;
  string configParamSettings;
  string capabilities;
  string flowConstraints;
  string fsIndexCollection;

  string getType(const YAML::Node &node);
  string getTypeFilePath() const;

  bool genAEInfo(const YAML::Node &node);

  bool parseAnnotatorInfo(const YAML::Node &node);

  bool generateAnnotatorConfigParamInfo(const YAML::Node &node);
  bool genConfigParamInfo(const YAML::Node &node, const string analysisEngineName);

  bool parseCapabInfo(const YAML::Node &node, std::string annotator_name = "");
  bool genCapabInfo(const YAML::Node &node);


  bool genFlowConstraints(const YAML::Node &node);
  bool genFsIndexCollection();


  rs::AnnotatorCapabilities annotatorCap;
  std::vector<rs::AnnotatorCapabilities> overwrittenAnnotCaps;
  vector<string> delegates_;

};

#endif
