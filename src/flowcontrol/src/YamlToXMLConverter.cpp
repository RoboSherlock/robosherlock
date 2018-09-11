#include <iostream>

#include <rs/flowcontrol/YamlToXMLConverter.h>

#include <ros/package.h>

using namespace std;

static const string INT_TYPE = "Integer";
static const string FLOAT_TYPE = "Float";
static const string BOOL_TYPE = "Boolean";
static const string STR_TYPE = "String";
static const string BOOL_TRUE = "true";
static const string BOOL_FALSE = "false";

static const string ANNOTATOR_NODE_NAME = "annotator";
static const string CONFIG_PARAM_NODE_NAME = "parameters";
static const string CAPAB_NODE_NAME = "capabilities";

YamlToXMLConverter::YamlToXMLConverter(string path)
  : yamlPath(path),
    header("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"),
    taeDesp("http://uima.apache.org/resourceSpecifier"),
    frameImpl("org.apache.uima.cpp")

{
  try {
    config = YAML::LoadFile(path);
  }
  catch(YAML::ParserException e) {
    outError("Error occurs when parsing" << path);
    outError(e.what());
    exit(1);
  }
}

void YamlToXMLConverter::parseYamlFile()
{
  for(YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
    YAML::Node key = it->first;
    YAML::Node value = it->second;
    if(key.Type() == YAML::NodeType::Scalar) {
      string nodeName = key.as<string>();
      if(nodeName == ANNOTATOR_NODE_NAME) {
        parseAnnotatorInfo(value);
      }
      else if(nodeName == CONFIG_PARAM_NODE_NAME) {
        parseConfigParamInfo(value);
      }
      else if(nodeName == CAPAB_NODE_NAME) {
        parseCapabInfo(value);
      }
      else {
        std::string msg = "Node's name is unknow to us.";
        YAML::ParserException e(YAML::Mark::null_mark(), msg);
        throw e;
      }
    }
    else {
      std::string msg = "Node's key should be scalar.";
      YAML::ParserException e(YAML::Mark::null_mark(), msg);
      throw e;
    }
  }
}

string YamlToXMLConverter::getType(const YAML::Node &node)
{
  bool isString = false;
  string nodeValue;
  try {
    node.as<float>();
  }
  catch(YAML::Exception e) {
    nodeValue = node.as<string>();
    isString = true;
  }

  if(!isString) {
    nodeValue = node.as<string>();
    if(nodeValue.find('.') == string::npos)
      return INT_TYPE;
    else
      return FLOAT_TYPE;
  }
  else {
    if(nodeValue == BOOL_TRUE || nodeValue == BOOL_FALSE)
      return BOOL_TYPE;
    else
      return STR_TYPE;
  }
}

bool YamlToXMLConverter::parseAnnotatorInfo(const YAML::Node &node)
{
  if(node.Type() == YAML::NodeType::Map) {
    for(YAML::const_iterator mit = node.begin(); mit != node.end(); ++mit) {
      string name = mit->first.as<string>();

      if(name == "name")
        AEName = mit->second.as<string>();
      else if(name == "implementation")
        AEImpl = mit->second.as<string>();
      else if(name == "description")
        AEDescription = mit->second.as<string>();
      else {
        cerr << mit->second.as<string>() << " is an unknown annotator info to us." << endl;
        return false;
      }
    }
  }
  else {
    cerr << "Please use map structure under annotator node." << endl;
    return false;
  }

  return true;
}

bool YamlToXMLConverter::parseConfigParamInfo(const YAML::Node &node)
{
  configParamSettings.append("<configurationParameterSettings>\n");
  configParams.append("<configurationParameters>\n");
  if(node.Type() == YAML::NodeType::Map) {
    for(YAML::const_iterator mit = node.begin(); mit != node.end(); ++mit) {
      string configName = mit->first.as<string>();

      if(mit->second.Type() == YAML::NodeType::Scalar) {  // scalar
        string type = getType(mit->second);

        configParams.append("\n<configurationParameter>\n");
        configParams.append("<name>\n");
        configParams.append(configName);
        configParams.append("\n</name>\n");
        configParams.append("<type>\n");
        configParams.append(type);
        configParams.append("\n</type>\n");
        configParams.append("<multiValued>false</multiValued>\n");
        configParams.append("<mandatory>false</mandatory>\n");
        configParams.append("</configurationParameter>\n");

        configParamSettings.append("\n<nameValuePair>\n");
        configParamSettings.append("<name>\n");
        configParamSettings.append(configName);
        configParamSettings.append("\n</name>\n");

        configParamSettings.append("<value>\n");

        if(type == BOOL_TYPE) {
          configParamSettings.append("<boolean>");
          configParamSettings.append(mit->second.as<string>());
          configParamSettings.append("</boolean>\n");
        }
        else if(type == FLOAT_TYPE) {
          configParamSettings.append("<float>");
          configParamSettings.append(mit->second.as<string>());
          configParamSettings.append("</float>\n");
        }
        else if(type == INT_TYPE) {
          configParamSettings.append("<integer>");
          configParamSettings.append(mit->second.as<string>());
          configParamSettings.append("</integer>\n");
        }
        else if(type == STR_TYPE) {
          configParamSettings.append("<string>");
          configParamSettings.append(mit->second.as<string>());
          configParamSettings.append("</string>\n");
        }
        else {
          cerr << "Illegal config param!" << endl;
          return false;
        }

        configParamSettings.append("</value>\n");
        configParamSettings.append("</nameValuePair>\n");

      }
      else if(mit->second.Type() == YAML::NodeType::Sequence) {    // list
        YAML::const_iterator listIt = mit->second.begin();
        string type = getType(*listIt);
        vector<string> listValue = mit->second.as<std::vector<string>>();

        configParams.append("<configurationParameter>\n");
        configParams.append("<name>\n");
        configParams.append(configName);
        configParams.append("\n</name>\n");
        configParams.append("<type>\n");
        configParams.append(type);
        configParams.append("\n</type>\n");
        configParams.append("<multiValued>true</multiValued>\n");  // multiValued: true
        configParams.append("<mandatory>false</mandatory>\n");
        configParams.append("</configurationParameter>\n");

        configParamSettings.append("<nameValuePair>\n");
        configParamSettings.append("<name>\n");
        configParamSettings.append(configName);
        configParamSettings.append("\n</name>\n");
        configParamSettings.append("<value>\n");
        configParamSettings.append("<array>\n");

        for(auto e : listValue) {
          if(type == BOOL_TYPE) {
            configParamSettings.append("<boolean>");
            configParamSettings.append(e);
            configParamSettings.append("\n</boolean>\n");
            break;
          }
          else if(type == FLOAT_TYPE) {
            configParamSettings.append("<float>");
            configParamSettings.append(e);
            configParamSettings.append("\n</float>\n");
            break;
          }
          else if(type == INT_TYPE) {
            configParamSettings.append("<integer>");
            configParamSettings.append(e);
            configParamSettings.append("\n</integer>\n");
            break;
          }
          else if(type == STR_TYPE) {
            configParamSettings.append("<string>");
            configParamSettings.append(e);
            configParamSettings.append("\n</string>\n");
            break;
          }
          else
            return false;
        }
        configParamSettings.append("</array>\n");
        configParamSettings.append("</value>\n");
        configParamSettings.append("</nameValuePair>\n");

      }
      else {
        cerr << "Illegal config param node type." << endl;
        return false;
      }
    }
  }
  else {
    cerr << "Please use map structure under annotator node." << endl;
    return false;
  }
  configParamSettings.append("</configurationParameterSettings>\n");
  configParams.append("</configurationParameters>\n");
  return true;
}

bool YamlToXMLConverter::parseCapabInfo(const YAML::Node &node)
{
  capabilities.append("<capabilities/>\n");
  capabilities.append("<capability/>\n");

  if(node.Type() == YAML::NodeType::Map) {
    for(YAML::const_iterator mit = node.begin(); mit != node.end(); ++mit) {
      string name = mit->first.as<string>();

      if(name == "inputs") {
        if(mit->second.Type() == YAML::NodeType::Scalar) {  // scalar
          cerr << "Inputs must be sequence type." << endl;
          return false;
        }
        else if(mit->second.Type() == YAML::NodeType::Sequence) {    // list
          for(YAML::Node::const_iterator sit = mit->second.begin(); sit != mit->second.end(); ++sit) {
            if(sit->Type() == YAML::NodeType::Scalar) {
              std::string val = sit->as<std::string>();
              annotCap.iTypeValueRestrictions[val] = std::vector<std::string>();
            }
            if(sit->Type() == YAML::NodeType::Map) {
              int size = std::distance(sit->begin(), sit->end());
              if(size == 1) {
                for(auto e : *sit) {
                  if(e.second.Type() == YAML::NodeType::Sequence) {
                    annotCap.iTypeValueRestrictions[e.first.as<std::string>()] = e.second.as<std::vector<std::string>>();
                  }
                }
              }
              else {
                outError("Inpute Type value restriction needs to be a single map entry;");
                return false;
              }
            }
          }
        }
      }
      else if(name == "outputs") {
        if(mit->second.Type() == YAML::NodeType::Scalar) {  // scalar
          outError("Outputs must be sequence type.");
          return false;
        }
        else if(mit->second.Type() == YAML::NodeType::Sequence) {    // list
          for(YAML::Node::const_iterator sit = mit->second.begin(); sit != mit->second.end(); ++sit) {
            YAML::Node  n = *sit;
            if(n.Type() == YAML::NodeType::Scalar) {
              std::string val = n.as<std::string>();
              annotCap.oTypeValueDomains[val] = std::vector<std::string>();
            }
            if(n.Type() == YAML::NodeType::Map) {
              int size = std::distance(n.begin(), n.end());
              if(size == 1) {
                for(auto e : n) {
                  if(e.second.Type() == YAML::NodeType::Sequence) {
                    std::vector<string> listValue = e.second.as<std::vector<string>>();
                    annotCap.oTypeValueDomains[e.first.as<std::string>()] = e.second.as<std::vector<std::string>>();
                  }
                }
              }
              else {
                outError("Output Type value domain needs to be a single map entry;");
                return false;
              }
            }
          }
        }
      }
      else continue;
    }
  }
  return true;
}

string YamlToXMLConverter::getTypeFilePath()
{
  size_t pos;
  string typeFilePath;
  if((pos = yamlPath.find("descriptors/")) != std::string::npos) {
    typeFilePath = yamlPath.substr(0, pos + 12);
    typeFilePath += "typesystem/all_types.xml";
  }
  else {
    string msg = "Annotator file path is illegal.";
    throw(std::runtime_error(msg));
  }
  return typeFilePath;
}

void YamlToXMLConverter::getXml(ofstream &out)
{
  out << header << endl;
  out << "<taeDescription xmlns=\"" << taeDesp << "\">" << endl;
  out << "<frameworkImplementation>" << frameImpl << "</frameworkImplementation>" << endl;
  out << "<primitive>true</primitive>" << endl;
  out << "<annotatorImplementationName>" << AEImpl << "</annotatorImplementationName>" << endl;
  out << "<analysisEngineMetaData>" << endl;
  out << "<name>" << AEName << "</name>" << endl;
  out << "<description>" << AEDescription << "</description>" << endl;
  out << "<version>1.0</version>\n<vendor/>" << endl;
  out << endl;
  out << configParams << endl;
  out << configParamSettings << endl;
  out << capabilities << endl;

  string typePath;
  try {
    typePath = getTypeFilePath();
  }
  catch(std::runtime_error &e) {
    throw e;
  }

  out << "<typeSystemDescription>\n<imports>\n<import location=" << "\"" << typePath << "\"/>\n</imports>\n</typeSystemDescription>\n" << endl;
  out << "<operationalProperties>\n<modifiesCas>true</modifiesCas>\n<multipleDeploymentAllowed>true</multipleDeploymentAllowed>\n<outputsNewCASes>false</outputsNewCASes>\n</operationalProperties>\n" << endl;
  out << "</analysisEngineMetaData>" << endl;
  out << "</taeDescription>" << endl;
}

rs::AnnotatorCapabilities YamlToXMLConverter::getAnnotatorCapabilities()
{
  return annotCap;
}
