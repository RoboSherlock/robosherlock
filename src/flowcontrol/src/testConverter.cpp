#include <iostream>
#include <rs/flowcontrol/AEYamlToXML.h>
#include <ros/package.h>

int main(int argc, char* argv[])
{
  try
  {
  if (argc<2) throw -1;
  }
  catch(int e)
  {
    cerr << "An initialization error occured\n";
    cerr << "Not enough arguments! Have you put your input file?\n";
    return e;
  }
  std::string yamlPath(argv[1]);
  AEYamlToXMLConverter yamlConverter(yamlPath);
  yamlConverter.parseYamlFile();
  ofstream xmlOutput;
  xmlOutput.open("/home/mirrorice/catkin_ws2/src/robosherlock/src/flowcontrol/src/"+std::string(argv[2])+".xml");
  yamlConverter.getOutput(xmlOutput);
  xmlOutput.close();
  return 0;
}
