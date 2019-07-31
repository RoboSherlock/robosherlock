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

static const string AE_NODE_NAME = "ae";
static const string ANNOTATOR_NODE_NAME = "annotator";
static const string CONFIG_PARAM_NODE_NAME = "parameters";
static const string CAPAB_NODE_NAME = "capabilities";

static const string FIXED_FLOW_NODE_NAME = "fixedflow";


YamlToXMLConverter::YamlToXMLConverter(string path, YamlToXMLConverter::YAMLType type)
  : yamlPath(path),
    header("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"),
    taeDesp("http://uima.apache.org/resourceSpecifier"),
    frameImpl("org.apache.uima.cpp")

{
  try
  {
    config = YAML::LoadFile(path);
    type_ = type;
  }
  catch(YAML::ParserException e)
  {
    outError("Error occurs when parsing" << path);
    outError(e.what());
    exit(1);
  }
}

void YamlToXMLConverter::getDelegates(vector<string> &delegates)
{
  if(type_ == YamlToXMLConverter::YAMLType::AAE)
  {
    delegates = delegates_;
  }
}

void YamlToXMLConverter::parseYamlFile()
{
  if(type_ == YamlToXMLConverter::YAMLType::Annotator)
  {
    for(YAML::const_iterator it = config.begin(); it != config.end(); ++it)
    {
      YAML::Node key = it->first;
      YAML::Node value = it->second;
      if(key.Type() == YAML::NodeType::Scalar)
      {
        string nodeName = key.as<string>();
        if(nodeName == ANNOTATOR_NODE_NAME)
        {
          parseAnnotatorInfo(value);
        }
        else if(nodeName == CONFIG_PARAM_NODE_NAME)
        {
          generateAnnotatorConfigParamInfo(value);
        }
        else if(nodeName == CAPAB_NODE_NAME)
        {
          parseCapabInfo(value);
        }
        else
        {
          std::string msg = "Node's name is unknow to us.";
          YAML::ParserException e(YAML::Mark::null_mark(), msg);
          throw e;
        }
      }
      else
      {
        std::string msg = "Node's key should be scalar.";
        YAML::ParserException e(YAML::Mark::null_mark(), msg);
        throw e;
      }
    }
    genFsIndexCollection();
  }
  else if(type_ == YamlToXMLConverter::YAMLType::AAE)
  {
    bool isConfigParamsStart = false;
    bool isConfigParamsEnded = false;
    for(YAML::const_iterator it = config.begin(); it != config.end(); ++it)
    {
      YAML::Node key = it->first;
      YAML::Node value = it->second;

      if(key.Type() == YAML::NodeType::Scalar)
      {
        string nodeName = key.as<string>();
        if(nodeName == AE_NODE_NAME)
        {
          genAEInfo(value);
        }
        else if(rs::common::getAnnotatorPath(nodeName) != "")
        {
          if(!isConfigParamsStart)
          {
            isConfigParamsStart = true;
            configParams.append("    <configurationParameters searchStrategy=\"none\">\n");
            configParamSettings.append("    <configurationParameterSettings>\n");
          }
          genConfigParamInfo(value, nodeName);
        }
        else
        {
          //            if(nodeName == CAPAB_NODE_NAME) {
          //              genCapabInfo(value);
          //            }
          if(nodeName == FIXED_FLOW_NODE_NAME)
          {
            genFlowConstraints(value);
          }
          else
          {
            std::string msg = "Node's name is unknow to us.";
            YAML::ParserException e(YAML::Mark::null_mark(), msg);
            throw e;
          }
        }

      }
      else
      {
        std::string msg = "Node's key should be scalar.";
        YAML::ParserException e(YAML::Mark::null_mark(), msg);
        throw e;
      }
    }
    if(capabilities.compare("") == 0)
    {
      YAML::Node mockNode;
      genCapabInfo(mockNode);
    }
    if((!isConfigParamsEnded) && (isConfigParamsStart))
    {
      isConfigParamsEnded = true;
      configParams.append("    </configurationParameters>\n");
      configParamSettings.append("    </configurationParameterSettings>\n");
    }
  }
}

string YamlToXMLConverter::getType(const YAML::Node &node)
{
  bool isString = false;
  string nodeValue;
  try
  {
    node.as<float>();
  }
  catch(YAML::Exception e)
  {
    nodeValue = node.as<string>();
    isString = true;
  }

  if(!isString)
  {
    nodeValue = node.as<string>();
    if(nodeValue.find('.') == string::npos)
      return INT_TYPE;
    else
      return FLOAT_TYPE;
  }
  else
  {
    if(nodeValue == BOOL_TRUE || nodeValue == BOOL_FALSE)
      return BOOL_TYPE;
    else
      return STR_TYPE;
  }
}


bool YamlToXMLConverter::genAEInfo(const YAML::Node &node)
{
  if(node.Type() == YAML::NodeType::Map)
  {
    for(YAML::const_iterator mit = node.begin(); mit != node.end(); ++mit)
    {
      string name = mit->first.as<string>();

      if(name == "name")
        AEName = mit->second.as<string>();
      else if(name == "implementation")
        AEImpl = mit->second.as<string>();
      else if(name == "description")
        AEDescription = mit->second.as<string>();
      else
      {
        cerr << mit->second.as<string>() << " is an unknown annotator info to us." << endl;
        return false;
      }
    }
  }
  else
  {
    cerr << "Please use map structure under annotator node." << endl;
    return false;
  }

  return true;
}

bool YamlToXMLConverter::parseAnnotatorInfo(const YAML::Node &node)
{
  if(node.Type() == YAML::NodeType::Map)
  {
    for(YAML::const_iterator mit = node.begin(); mit != node.end(); ++mit)
    {
      string name = mit->first.as<string>();

      if(name == "name")
        AEName = mit->second.as<string>();
      else if(name == "implementation")
        AEImpl = mit->second.as<string>();
      else if(name == "description")
        AEDescription = mit->second.as<string>();
      else
      {
        cerr << mit->second.as<string>() << " is an unknown annotator info to us." << endl;
        return false;
      }
    }
  }
  else
  {
    cerr << "Please use map structure under annotator node." << endl;
    return false;
  }

  return true;
}


bool YamlToXMLConverter::genConfigParamInfo(const YAML::Node &node, const string analysisEngineName)
{
  if(node.Type() == YAML::NodeType::Map)
  {
    for(YAML::const_iterator mit = node.begin(); mit != node.end(); ++mit)
    {
      string configName = mit->first.as<string>();
      if(configName == "capabilities")
      {
        outWarn("FOUND CAPABILITY DEFINITION IN AAE: "<<analysisEngineName);
        parseCapabInfo(mit->second, analysisEngineName);
        continue;
      }
      if(mit->second.Type() == YAML::NodeType::Scalar)    // scalar
      {
        string type = getType(mit->second);

        configParams.append("      <configurationParameter>\n");
        configParams.append("        <name>");
        configParams.append(configName);
        configParams.append("</name>\n");
        configParams.append("        <type>");
        configParams.append(type);
        configParams.append("</type>\n");
        configParams.append("        <multiValued>false</multiValued>\n");
        configParams.append("        <mandatory>true</mandatory>\n");
        configParams.append("        <overrides>\n");
        configParams.append("          <parameter>");
        configParams.append(analysisEngineName.c_str());
        configParams.append("/");
        configParams.append(configName);
        configParams.append("</parameter>\n");
        configParams.append("        </overrides>\n");
        configParams.append("      </configurationParameter>\n");

        configParamSettings.append("      <nameValuePair>\n");
        configParamSettings.append("        <name>");
        configParamSettings.append(configName);
        configParamSettings.append("</name>\n");

        configParamSettings.append("        <value>\n");

        if(type == BOOL_TYPE)
        {
          configParamSettings.append("            <boolean>");
          configParamSettings.append(mit->second.as<string>());
          configParamSettings.append("</boolean>\n");
        }
        else if(type == FLOAT_TYPE)
        {
          configParamSettings.append("            <float>");
          configParamSettings.append(mit->second.as<string>());
          configParamSettings.append("</float>\n");
        }
        else if(type == INT_TYPE)
        {
          configParamSettings.append("            <integer>");
          configParamSettings.append(mit->second.as<string>());
          configParamSettings.append("</integer>\n");
        }
        else if(type == STR_TYPE)
        {
          configParamSettings.append("            <string>");
          configParamSettings.append(mit->second.as<string>());
          configParamSettings.append("</string>\n");
        }
        else
        {
          cerr << "Illegal config param!" << endl;
          return false;
        }

        configParamSettings.append("        </value>\n");
        configParamSettings.append("      </nameValuePair>\n");

      }
      else if(mit->second.Type() == YAML::NodeType::Sequence)      // list
      {
        YAML::const_iterator listIt = mit->second.begin();
        string type = getType(*listIt);
        vector<string> listValue = mit->second.as<std::vector<string>>();

        configParams.append("      <configurationParameter>\n");
        configParams.append("        <name>");
        configParams.append(configName);
        configParams.append("</name>\n");
        configParams.append("        <type>");
        configParams.append(type);
        configParams.append("</type>\n");
        configParams.append("        <multiValued>true</multiValued>\n");
        configParams.append("        <mandatory>true</mandatory>\n");
        configParams.append("        <overrides>\n");
        configParams.append("          <parameter>");
        configParams.append(analysisEngineName.c_str());
        configParams.append("/");
        configParams.append(configName);
        configParams.append("</parameter>\n");
        configParams.append("        </overrides>\n");
        configParams.append("      </configurationParameter>\n");


        configParamSettings.append("      <nameValuePair>\n");
        configParamSettings.append("        <name>");
        configParamSettings.append(configName);
        configParamSettings.append("</name>\n");
        configParamSettings.append("        <value>\n");
        configParamSettings.append("          <array>\n");

        for(auto e : listValue)
        {
          if(type == BOOL_TYPE)
          {
            configParamSettings.append("            <boolean>");
            configParamSettings.append(e);
            configParamSettings.append("</boolean>\n");
          }
          else if(type == FLOAT_TYPE)
          {
            configParamSettings.append("            <float>");
            configParamSettings.append(e);
            configParamSettings.append("</float>\n");
          }
          else if(type == INT_TYPE)
          {
            configParamSettings.append("            <integer>");
            configParamSettings.append(e);
            configParamSettings.append("</integer>\n");
          }
          else if(type == STR_TYPE)
          {
            configParamSettings.append("            <string>");
            configParamSettings.append(e);
            configParamSettings.append("</string>\n");
          }
          else
            return false;
        }
        configParamSettings.append("          </array>\n");
        configParamSettings.append("        </value>\n");
        configParamSettings.append("      </nameValuePair>\n");

      }
      else
      {
        cerr << "Illegal config param node type." << endl;
        return false;
      }
    }
  }
  else
  {
    cerr << "Please use map structure under annotator node." << endl;
    return false;
  }

  return true;
}

bool YamlToXMLConverter::genFlowConstraints(const YAML::Node &node)
{

  flowConstraints.append("    <flowConstraints>\n");
  flowConstraints.append("      <fixedFlow>\n");
  if(node.Type() == YAML::NodeType::Sequence)
  {
    vector<string> listValue = node.as<std::vector<string>>();
    for(auto e : listValue)
    {
      flowConstraints.append("        <node>");
      flowConstraints.append(e);
      flowConstraints.append("</node>\n");
      delegates_.push_back(e);
    }
  }
  flowConstraints.append("      </fixedFlow>\n");
  flowConstraints.append("    </flowConstraints>\n");
  return true;
}


bool YamlToXMLConverter::generateAnnotatorConfigParamInfo(const YAML::Node &node)
{
  configParamSettings.append("<configurationParameterSettings>\n");
  configParams.append("<configurationParameters>\n");
  if(node.Type() == YAML::NodeType::Map)
  {
    for(YAML::const_iterator mit = node.begin(); mit != node.end(); ++mit)
    {
      string configName = mit->first.as<string>();

      if(mit->second.Type() == YAML::NodeType::Scalar)    // scalar
      {
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

        if(type == BOOL_TYPE)
        {
          configParamSettings.append("<boolean>");
          configParamSettings.append(mit->second.as<string>());
          configParamSettings.append("</boolean>\n");
        }
        else if(type == FLOAT_TYPE)
        {
          configParamSettings.append("<float>");
          configParamSettings.append(mit->second.as<string>());
          configParamSettings.append("</float>\n");
        }
        else if(type == INT_TYPE)
        {
          configParamSettings.append("<integer>");
          configParamSettings.append(mit->second.as<string>());
          configParamSettings.append("</integer>\n");
        }
        else if(type == STR_TYPE)
        {
          configParamSettings.append("<string>");
          configParamSettings.append(mit->second.as<string>());
          configParamSettings.append("</string>\n");
        }
        else
        {
          cerr << "Illegal config param!" << endl;
          return false;
        }

        configParamSettings.append("</value>\n");
        configParamSettings.append("</nameValuePair>\n");

      }
      else if(mit->second.Type() == YAML::NodeType::Sequence)      // list
      {
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

        for(auto e : listValue)
        {
          if(type == BOOL_TYPE)
          {
            configParamSettings.append("<boolean>");
            configParamSettings.append(e);
            configParamSettings.append("\n</boolean>\n");
          }
          else if(type == FLOAT_TYPE)
          {
            configParamSettings.append("<float>");
            configParamSettings.append(e);
            configParamSettings.append("\n</float>\n");
          }
          else if(type == INT_TYPE)
          {
            configParamSettings.append("<integer>");
            configParamSettings.append(e);
            configParamSettings.append("\n</integer>\n");
          }
          else if(type == STR_TYPE)
          {
            configParamSettings.append("<string>");
            configParamSettings.append(e);
            configParamSettings.append("\n</string>\n");
          }
          else
            return false;
        }
        configParamSettings.append("</array>\n");
        configParamSettings.append("</value>\n");
        configParamSettings.append("</nameValuePair>\n");

      }
      else
      {
        cerr << "Illegal config param node type." << endl;
        return false;
      }
    }
  }
  else
  {
    cerr << "Please use map structure under annotator node." << endl;
    return false;
  }
  configParamSettings.append("</configurationParameterSettings>\n");
  configParams.append("</configurationParameters>\n");
  return true;
}


bool YamlToXMLConverter::genCapabInfo(const YAML::Node &node)
{

  bool hasInputs = false, hasOutputs = false;
  capabilities.append("    <capabilities>\n");
  capabilities.append("      <capability>\n");

  capabilities.append("        <inputs/>\n");  // assume inputs tag goes to inputSofas
  capabilities.append("        <outputs/>\n");

  if(node.Type() == YAML::NodeType::Map)
  {
    for(YAML::const_iterator mit = node.begin(); mit != node.end(); ++mit)
    {
      string name = mit->first.as<string>();

      if(name == "inputs")
      {
        capabilities.append("        <inputSofas>\n");
        if(mit->second.Type() == YAML::NodeType::Scalar)    // scalar
        {
          cerr << "Inputs must be sequence type." << endl;
          return false;
        }
        else if(mit->second.Type() == YAML::NodeType::Sequence)      // list
        {
          vector<string> listValue = mit->second.as<std::vector<string>>();
          for(auto e : listValue)
          {
            capabilities.append("        <sofaName>");
            capabilities.append(e);
            capabilities.append("</sofaName>\n");
          }
        }
        capabilities.append("      </inputSofas>\n");
        hasInputs = true;
      }
      else if(name == "outputs")
      {
        capabilities.append("        <outputs>\n");
        if(mit->second.Type() == YAML::NodeType::Scalar)    // scalar
        {
          cerr << "Outputs must be sequence type." << endl;
          return false;
        }
        else if(mit->second.Type() == YAML::NodeType::Sequence)      // list
        {
          vector<string> listValue = mit->second.as<std::vector<string>>();
          for(auto e : listValue)
          {
            capabilities.append("          <type allAnnotatorFeatures=\"true\">");
            capabilities.append(e);
            capabilities.append("</type>\n");
          }
        }
        capabilities.append("        </outputs>\n");
        hasOutputs = true;
      }
      else continue;
    }
  }

  capabilities.append("        <languagesSupported>\n");
  capabilities.append("          <language>x-unspecified</language>\n");
  capabilities.append("        </languagesSupported>\n");
  capabilities.append("      </capability>\n");
  capabilities.append("    </capabilities>\n");
  return true;
}

bool YamlToXMLConverter::parseCapabInfo(const YAML::Node &node, std::string annotator_name)
{

  rs::AnnotatorCapabilities annotCap;
  annotCap.annotatorName = annotator_name;
  if(node.Type() == YAML::NodeType::Map)
  {
    for(YAML::const_iterator mit = node.begin(); mit != node.end(); ++mit)
    {
      string name = mit->first.as<string>();

      if(name == "inputs")
      {
        if(mit->second.Type() == YAML::NodeType::Scalar)    // scalar
        {
          cerr << "Inputs must be sequence type." << endl;
          return false;
        }
        else if(mit->second.Type() == YAML::NodeType::Sequence)      // list
        {
          for(YAML::Node::const_iterator sit = mit->second.begin(); sit != mit->second.end(); ++sit)
          {
            if(sit->Type() == YAML::NodeType::Scalar)
            {
              std::string val = sit->as<std::string>();
              annotCap.iTypeValueRestrictions[val] = std::vector<std::string>();
            }
            if(sit->Type() == YAML::NodeType::Map)
            {
              int size = std::distance(sit->begin(), sit->end());
              if(size == 1)
              {
                for(auto e : *sit)
                {
                  if(e.second.Type() == YAML::NodeType::Sequence)
                  {
                    annotCap.iTypeValueRestrictions[e.first.as<std::string>()] = e.second.as<std::vector<std::string>>();
                  }
                }
              }
              else
              {
                outError("Inpute Type value restriction needs to be a single map entry;");
                return false;
              }
            }
          }
        }
      }
      else if(name == "outputs")
      {
        if(mit->second.Type() == YAML::NodeType::Scalar)    // scalar
        {
          outError("Outputs must be sequence type.");
          return false;
        }
        else if(mit->second.Type() == YAML::NodeType::Sequence)      // list
        {
          for(YAML::Node::const_iterator sit = mit->second.begin(); sit != mit->second.end(); ++sit)
          {
            YAML::Node  n = *sit;
            if(n.Type() == YAML::NodeType::Scalar)
            {
              std::string val = n.as<std::string>();
              annotCap.oTypeValueDomains[val] = std::vector<std::string>();
            }
            if(n.Type() == YAML::NodeType::Map)
            {
              int size = std::distance(n.begin(), n.end());
              if(size == 1)
              {
                for(auto e : n)
                {
                  if(e.second.Type() == YAML::NodeType::Sequence)
                  {
                    std::vector<string> listValue = e.second.as<std::vector<string>>();
                    annotCap.oTypeValueDomains[e.first.as<std::string>()] = e.second.as<std::vector<std::string>>();
                  }
                }
              }
              else
              {
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
  if(type_ == YamlToXMLConverter::YAMLType::Annotator)
  {
    capabilities.append("<capabilities/>\n");
    capabilities.append("<capability/>\n");
    annotatorCap = annotCap;
  }
  else
  {
    overwrittenAnnotCaps.push_back(annotCap);
  }

  return true;
}

bool YamlToXMLConverter::genFsIndexCollection()
{
  fsIndexCollection.append("    <fsIndexCollection>\n");
  fsIndexCollection.append("      <fsIndexes>\n");
  fsIndexCollection.append("        <fsIndexDescription>\n");
  fsIndexCollection.append("          <label>general</label>\n");
  fsIndexCollection.append("          <typeName>uima.cas.TOP</typeName>\n");
  fsIndexCollection.append("          <kind>bag</kind>\n");
  fsIndexCollection.append("        </fsIndexDescription>\n");
  fsIndexCollection.append("      </fsIndexes>\n");
  fsIndexCollection.append("    </fsIndexCollection>\n");
  return true;
}

string YamlToXMLConverter::getTypeFilePath() const
{
  size_t pos;
  string typeFilePath;
  if((pos = this->yamlPath.find("descriptors/")) != std::string::npos)
  {
    typeFilePath = this->yamlPath.substr(0, pos + 12);
    typeFilePath += "typesystem/all_types.xml";
  }
  else
  {
    string msg = "File path is illegal.";
    throw(std::runtime_error(msg));
  }
  return typeFilePath;
}

rs::AnnotatorCapabilities YamlToXMLConverter::getAnnotatorCapabilities()
{
  return annotatorCap;
}

std::vector<rs::AnnotatorCapabilities> YamlToXMLConverter::getOverwrittenAnnotatorCapabilities()
{
  return overwrittenAnnotCaps;
}
