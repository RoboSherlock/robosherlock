#include <rs/DrawingAnnotator.h>

std::map<std::string, DrawingAnnotator *> DrawingAnnotator::annotators;

DrawingAnnotator::DrawingAnnotator(const std::string &name) : name(name), update(false), hasRun(false)
{
  outDebug("Added: " << name);
  annotators[name] = this;
}

DrawingAnnotator::~DrawingAnnotator()
{
  std::map<std::string, DrawingAnnotator *>::const_iterator it;
  for(it = annotators.begin(); it != annotators.end(); ++it)
  {
    if(it->second == this)
    {
      annotators.erase(it);
      break;
    }
  }
}

void DrawingAnnotator::getAnnotatorNames(std::vector<std::string> &names)
{
  std::map<std::string, DrawingAnnotator *>::const_iterator it;
  names.clear();
  names.reserve(annotators.size());

  for(it = annotators.begin(); it != annotators.end(); ++it)
  {
    names.push_back(it->first);
  }
}

DrawingAnnotator *DrawingAnnotator::getAnnotator(const std::string &name)
{
  std::map<std::string, DrawingAnnotator *>::const_iterator it;
  it = annotators.find(name);
  if(it != annotators.end())
  {
    return it->second;
  }
  return NULL;
}

uima::TyErrorId DrawingAnnotator::process(uima::CAS &tcas, uima::ResultSpecification const &res_spec)
{
  uima::TyErrorId ret = UIMA_ERR_UNKNOWN_TYPE;
  drawLock.lock();
  try
  {
    ret = processWithLock(tcas, res_spec);
    update = true;
    hasRun = true;
  }
  catch(const uima::Exception &e)
  {
    outError("Exception in " << name << ": " << e);
    drawLock.unlock();
    throw e;
  }
  catch(...)
  {
    outError("Exception in " << name << "!");
  }
  drawLock.unlock();
  return ret;
}

void DrawingAnnotator::drawImage(cv::Mat &disp)
{
  if(!hasRun)
  {
    disp = cv::Mat::zeros(480, 640, CV_8UC3);
    return;
  }
  drawLock.lock();
  try
  {
    drawImageWithLock(disp);
  }
  catch(...)
  {
    outError("Exception in " << name << "!");
    if(disp.empty())
    {
      disp = cv::Mat::zeros(480, 640, CV_8UC3);
    }
  }
  drawLock.unlock();
}

bool DrawingAnnotator::fillVisualizer(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
{
  if(!hasRun)
  {
    return false;
  }
  drawLock.lock();
  try
  {
    fillVisualizerWithLock(visualizer, firstRun);
  }
  catch(...)
  {
    outError("Exception in " << name << "!");
  }
  drawLock.unlock();
  return true;
}

bool DrawingAnnotator::callbackMouse(const int event, const int x, const int y, const Source source)
{
  return false;
}

bool DrawingAnnotator::callbackKey(const int key, const Source source)
{
  return false;
}

void DrawingAnnotator::fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
{
  //empty function declaration so C+ does not complain like the litle bitch it is
}

void DrawingAnnotator::drawImageWithLock(cv::Mat &disp)
{
  disp = cv::Mat::zeros(480, 640, CV_8UC3);
}
