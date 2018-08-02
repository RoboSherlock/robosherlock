#include <iostream>

#include <rs/utils/YamlToXMLConverter.h>

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
    : frameImpl("org.apache.uima.cpp"),
      taeDesp("http://uima.apache.org/resourceSpecifier"),
      header("<?xml version=\"1.0\" encoding=\"UTF-8\"?>")
{
    try {
        config = YAML::LoadFile(path);
    } catch(YAML::ParserException e) {
        cerr << "Error occurs when parsing the file." << std::endl;
    }
}

bool YamlToXMLConverter::parseYamlFile() {
    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it){
        YAML::Node key = it->first;
        YAML::Node value = it->second;
        if (key.Type() == YAML::NodeType::Scalar) {
            string nodeName = key.as<string>();
            if (nodeName == ANNOTATOR_NODE_NAME) {
                if (!genAnnotatorInfo(value)) return false;
            }
            else if (nodeName == CONFIG_PARAM_NODE_NAME) {
                if (!genConfigParamInfo(value)) return false;
            }
            else if (nodeName == CAPAB_NODE_NAME) {
                if (!genCapabInfo(value)) return false;
            }
            else {
                cerr << "Node's name is unknow to us." << endl;
                return false;
            }
        } else {
            cerr << "Keyword should be scalar." << endl;
            return false;
        }
    }

    return true;
}

string YamlToXMLConverter::getType(const YAML::Node& node) {
    bool isString = false;
    string nodeValue;
    try {
        node.as<float>();
    } catch (YAML::Exception e) {
        nodeValue = node.as<string>();
        isString = true;
    }

    if (!isString) {
        nodeValue = node.as<string>();
        if (nodeValue.find('.') == string::npos)
            return INT_TYPE;
        else
            return FLOAT_TYPE;
    } else {
        if (nodeValue == BOOL_TRUE || nodeValue == BOOL_FALSE)
            return BOOL_TYPE;
        else
            return STR_TYPE;
    }
}

bool YamlToXMLConverter::genAnnotatorInfo(const YAML::Node& node) {
    if (node.Type() == YAML::NodeType::Map) {
        for(YAML::const_iterator mit = node.begin(); mit != node.end(); ++mit) {
            string name = mit->first.as<string>();

            if (name == "name")
                AEName = mit->second.as<string>();
            else if (name == "implementation")
                AEImpl = mit->second.as<string>();
            else if (name == "description")
                AEDescription = mit->second.as<string>();
            else {
                cerr << mit->second.as<string>() << " is an unknown annotator info to us." << endl;
                return false;
            }
        }
    } else {
        cerr << "Please use map structure under annotator node." << endl;
        return false;
    }

    return true;
}

bool YamlToXMLConverter::genConfigParamInfo(const YAML::Node& node) {
    configParamSettings.append("<configurationParameterSettings>\n");
    configParams.append("<configurationParameters>\n");
    if (node.Type() == YAML::NodeType::Map) {
        for(YAML::const_iterator mit = node.begin(); mit != node.end(); ++mit) {
            string configName = mit->first.as<string>();

            if (mit->second.Type() == YAML::NodeType::Scalar) { // scalar
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

                if (type == BOOL_TYPE) {
                    configParamSettings.append("<boolean>");
                    configParamSettings.append(mit->second.as<string>());
                    configParamSettings.append("</boolean>\n");
                } else if (type == FLOAT_TYPE) {
                    configParamSettings.append("<float>");
                    configParamSettings.append(mit->second.as<string>());
                    configParamSettings.append("</float>\n");
                } else if (type == INT_TYPE) {
                    configParamSettings.append("<integer>");
                    configParamSettings.append(mit->second.as<string>());
                    configParamSettings.append("</integer>\n");
                } else if (type == STR_TYPE) {
                    configParamSettings.append("<string>");
                    configParamSettings.append(mit->second.as<string>());
                    configParamSettings.append("</string>\n");
                } else {
                    cerr << "Illegal config param!" << endl;
                    return false;
                }

                configParamSettings.append("</value>\n");
                configParamSettings.append("</nameValuePair>\n");

            } else if (mit->second.Type() == YAML::NodeType::Sequence) { // list
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

                for (auto e : listValue) {
                    if (type == BOOL_TYPE) {
                        configParamSettings.append("<boolean>");
                        configParamSettings.append(e);
                        configParamSettings.append("\n</boolean>\n");
                        break;
                    } else if (type == FLOAT_TYPE) {
                        configParamSettings.append("<float>");
                        configParamSettings.append(e);
                        configParamSettings.append("\n</float>\n");
                        break;
                    } else if (type == INT_TYPE) {
                        configParamSettings.append("<integer>");
                        configParamSettings.append(e);
                        configParamSettings.append("\n</integer>\n");
                        break;
                    } else if (type == STR_TYPE) {
                        configParamSettings.append("<string>");
                        configParamSettings.append(e);
                        configParamSettings.append("\n</string>\n");
                        break;
                    } else
                        return false;
                }
                configParamSettings.append("</array>\n");
                configParamSettings.append("</value>\n");
                configParamSettings.append("</nameValuePair>\n");

            } else {
                cerr << "Illegal config param node type." << endl;
                return false;
            }
        }
    } else {
        cerr << "Please use map structure under annotator node." << endl;
        return false;
    }
    configParamSettings.append("</configurationParameterSettings>\n");
    configParams.append("</configurationParameters>\n");
    return true;
}

bool YamlToXMLConverter::genCapabInfo(const YAML::Node& node) {

    bool hasInputs = false, hasOutputs = false;
    capabilities.append("<capabilities>\n");
    capabilities.append("<capability>\n");

    capabilities.append("<inputs/>\n");  // assume inputs tag goes to inputSofas

    if (node.Type() == YAML::NodeType::Map) {
        for(YAML::const_iterator mit = node.begin(); mit != node.end(); ++mit) {
            string name = mit->first.as<string>();

            if (name == "inputs") {
                capabilities.append("<inputSofas>\n");
                if (mit->second.Type() == YAML::NodeType::Scalar) { // scalar
                    cerr << "Inputs must be sequence type." << endl;
                    return false;
                } else if (mit->second.Type() == YAML::NodeType::Sequence) { // list
                    vector<string> listValue = mit->second.as<std::vector<string>>();
                    for (auto e : listValue) {
                        capabilities.append("<sofaName>");
                        capabilities.append(e);
                        capabilities.append("</sofaName>\n");
                    }
                }
                capabilities.append("</inputSofas>\n");
                hasInputs = true;
            } else if (name == "outputs") {
                capabilities.append("<outputs>\n");
                if (mit->second.Type() == YAML::NodeType::Scalar) { // scalar
                    cerr << "Outputs must be sequence type." << endl;
                    return false;
                } else if (mit->second.Type() == YAML::NodeType::Sequence) { // list
                    vector<string> listValue = mit->second.as<std::vector<string>>();
                    for (auto e : listValue) {
                        capabilities.append("<type allAnnotatorFeatures=\"true\">");
                        capabilities.append(e);
                        capabilities.append("</type>\n");
                    }
                }
                capabilities.append("</outputs>\n");
                hasOutputs = true;
            } else continue;
        }
    }

    capabilities.append("<languagesSupported>\n<language>x-unspecified</language>\n</languagesSupported>\n");
    capabilities.append("</capability>\n");
    capabilities.append("</capabilities>\n");
}

void YamlToXMLConverter::getOutput(ofstream& out) {
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
    out << "<typeSystemDescription>\n<imports>\n<import location=\"../../typesystem/all_types.xml\"/>\n</imports>\n</typeSystemDescription>\n" << endl;
    out << "<operationalProperties>\n<modifiesCas>true</modifiesCas>\n<multipleDeploymentAllowed>true</multipleDeploymentAllowed>\n<outputsNewCASes>false</outputsNewCASes>\n</operationalProperties>\n" << endl;
    out << "</analysisEngineMetaData>" << endl;
    out << "</taeDescription>" << endl;
}
