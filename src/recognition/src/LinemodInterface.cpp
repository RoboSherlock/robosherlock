/* Copyright (c) 2013, Thiemo Wiedemeyer  <wiedemeyer@informatik.uni-bremen.de>
 * All rights reserved.
 */

// System
#include <dirent.h>
#include <sys/stat.h>
#include <fstream>
#include <sstream>
#include <map>

#include <rs/recognition/LinemodInterface.h>
#include <rs/utils/output.h>

LinemodInterface::LinemodInterface() : detector(), matches()
{
  std::vector<cv::Ptr<cv::linemod::Modality> > modalities;
  std::vector<int> pyramidT;
  modalities.push_back(new cv::linemod::ColorGradient);
  modalities.push_back(new cv::linemod::DepthNormal);
  pyramidT.push_back(4);
  pyramidT.push_back(8);
  detector = new cv::linemod::Detector(modalities, pyramidT);
}

/*
 * Processes a frame
 */
void LinemodInterface::process(const cv::Mat &color, const cv::Mat &depth, std::vector<Result> &results, const float minResponse,
                               const std::vector<std::string> &classes, const cv::Mat &mask)
{
  cv::Mat _depth;

  switch(depth.type())
  {
  case CV_32F:
    depth.convertTo(_depth, CV_16U, 1000.0);
    break;
  case CV_16U:
    _depth = depth;
    break;
  default:
    return;
  }

  std::vector<cv::Mat>
  sources(2),
          masks;
  sources[0] = color;
  sources[1] = _depth;

  if(!mask.empty())
  {
    masks.resize(2);
    masks[0] = mask.clone();
    masks[1] = mask.clone();
  }

  matches.clear();
  detector->match(sources, minResponse, matches, classes, cv::noArray(), masks);

  std::map<std::string, int> best;
  std::map<std::string, int>::iterator it;
  Result result;
  results.clear();

  for(int i = 0; i < (int)matches.size(); ++i)
  {
    cv::linemod::Match &match = matches[i];

    setResult(match, result);

    it = best.find(result.name);
    if(it == best.end())
    {
      best[result.name] = i;

      setResult(match, result);
      results.push_back(result);
    }
  }
}

/*
 * Reading models from resource directory
 */
bool LinemodInterface::readModels(const std::string &resourcePath)
{
  DIR *dp;
  struct dirent *dirp;
  struct stat fileStat;

  if((dp  = opendir(resourcePath.c_str())) ==  NULL)
  {
    outError("path (" << resourcePath << ") does not exist!");
    return false;
  }

  std::vector<std::string> modelNames;
  std::vector<std::vector<std::string> > modelFiles;

  while((dirp = readdir(dp)) != NULL)
  {
    if(dirp->d_type != DT_DIR || dirp->d_name[0] == '.')
    {
      continue;
    }


    std::string name = dirp->d_name;
    std::string path = resourcePath + "/" + name + "/linemod.yml.gz";

    if(!stat(path.c_str(), &fileStat) && S_ISREG(fileStat.st_mode))
    {
      outDebug("read model: " << path);
      readModel(path);
    }
  }
  closedir(dp);
  return true;
}

/*
 * Reading templates of model
 */
void LinemodInterface::readModel(const std::string &filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::READ);

  cv::FileNode fn = fs["classes"];
  for(cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
  {
    detector->readClass(*i);
  }
}

/*
 * Writing models to resource directory
 */
void LinemodInterface::writeModels(const std::string &resourcePath)
{
  const std::vector<std::string> ids = detector->classIds();
  const std::string
  basePath = resourcePath[resourcePath.size() - 1] == '/' ? resourcePath : resourcePath + '/',
  file = "/linemod.yml",
  fileGZ = file + ".gz";

  std::map<std::string, std::vector<std::string> > baseIds;
  for(int i = 0; i < (int)ids.size(); ++i)
  {
    const std::string
    &classId = ids[i],
     baseId = classId.substr(0, classId.rfind("_r"));
    baseIds[baseId].push_back(classId);
  }

  std::map<std::string, std::vector<std::string> >::const_iterator
  it = baseIds.begin(),
  end = baseIds.end();
  for(; it != end; ++it)
  {
    writeModel(basePath + it->first + file, it->second);
    writeModel(basePath + it->first + fileGZ, it->second);
  }
}

/*
 * Writing templates of model
 */
void LinemodInterface::writeModel(const std::string &filename, const std::vector<std::string> &classIds)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);

  fs << "classes" << "[";
  for(int i = 0; i < classIds.size(); ++i)
  {
    fs << "{";
    detector->writeClass(classIds[i], fs);
    fs << "}"; // current class
  }
  fs << "]"; // classes
}

/*
 * Draws a result to the image
 */
void LinemodInterface::drawResult(cv::Mat &image, const Result &result)
{
  const std::vector<cv::linemod::Template>
  &templates = detector->getTemplates(result.match->class_id, result.match->template_id);
  cv::Point
  offset(result.match->x, result.match->y);
  const cv::Scalar
  colors[3] = {CV_RGB(0, 0, 255), CV_RGB(255, 0, 0), CV_RGB(0, 255, 0)};

  for(int t = 0; t < 2/*(int)templates.size()*/; ++t)
  {
    const cv::linemod::Template
    &tmpl = templates[t];
    const int
    radius = detector->getT(tmpl.pyramid_level) >> 1;
    const cv::Scalar
    &color = colors[t % 2];

    #pragma omp parallel for
    for(int i = 0; i < (int)tmpl.features.size(); ++i)
    {
      const cv::linemod::Feature
      &f = tmpl.features[i];

      cv::circle(image, offset + cv::Point(f.x, f.y), radius, color, 1, CV_AA);
    }
  }
  cv::rectangle(image, result.roi, colors[2], 1, CV_AA);

  std::ostringstream oss;
  oss << result.name << " (" << result.response << " R: " << result.rotation << " S:" << result.scale << ")";
  cv::putText(image, oss.str(), offset + cv::Point(5, -10), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, colors[2], 1, CV_AA);
}

/*
 * Draws results to the image
 */
void LinemodInterface::drawResults(cv::Mat &image, const std::vector<Result> &results)
{
  for(int i = 0; i < (int)results.size(); ++i)
  {
    drawResult(image, results[i]);
  }
}

/*
 * Sets the result structure
 */
void LinemodInterface::setResult(const cv::linemod::Match &match, Result &result)
{
  const size_t
  posRot = match.class_id.rfind("_r"),
  posScale = match.class_id.rfind("_s");
  const std::vector<cv::linemod::Template>
  &templates = detector->getTemplates(match.class_id, match.template_id);

  result.name = match.class_id.substr(0, posRot);
  result.rotation = (float)atoi(match.class_id.substr(posRot + 2, posScale).c_str());
  result.scale = (float)atoi(match.class_id.substr(posScale + 2, match.class_id.size()).c_str()) / 100.0f;
  result.response = match.similarity;
  result.roi = cv::Rect(match.x, match.y, templates[0].width, templates[0].height);
  result.match = &match;
}
