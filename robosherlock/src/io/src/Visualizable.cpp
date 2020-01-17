#include "robosherlock/io/Visualizable.h"

std::map<std::string, Visualizable*> Visualizable::visualizables;

Visualizable::Visualizable(const std::string& name) : name(name), update(false)
{
  outDebug("Added: " << name);
  visualizables[name] = this;
}

Visualizable::~Visualizable()
{
  std::map<std::string, Visualizable*>::const_iterator it;
  for (it = visualizables.begin(); it != visualizables.end(); ++it)
  {
    if (it->second == this)
    {
      visualizables.erase(it);
      break;
    }
  }
}

bool Visualizable::callbackMouse(const int event, const int x, const int y, const VisualizableDataType source)
{
  return false;
}

bool Visualizable::callbackKey(const int key, const VisualizableDataType source)
{
  return false;
}

void Visualizable::drawImage(cv::Mat& disp)
{
}

bool Visualizable::fillVisualizer(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
{
  return false;
}

int Visualizable::copyVisualizableList(std::map<std::string, Visualizable*>& inMap)
{
  inMap.clear();
  inMap.insert(visualizables.begin(), visualizables.end());

  return inMap.size();
}

void Visualizable::clearVisualizableList()
{
  visualizables.clear();
}
