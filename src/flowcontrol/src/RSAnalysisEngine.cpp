/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *         Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *         Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <rs/flowcontrol/RSAnalysisEngine.h>


static const std::string GEN_XML_PATH = ".ros/robosherlock_generated_xmls";

RSAnalysisEngine::RSAnalysisEngine() : engine(NULL), cas(NULL), rspm(NULL)
{
}

RSAnalysisEngine::~RSAnalysisEngine()
{
  if(cas)
  {
    delete cas;
    cas = NULL;
  }
  if(rspm)
  {
    delete rspm;
    rspm = NULL;
  }
  if(engine)
  {
    delete engine;
    engine = NULL;
  }
}

void RSAnalysisEngine::init(const std::string &file, bool parallel)
{
  size_t pos = file.rfind('/');
  outInfo("Creating analysis engine: " FG_BLUE << (pos == file.npos ? file : file.substr(pos)));
  uima::ErrorInfo errorInfo;

  // Before creating the analysis engine, we need to find the annotators
  // that belongs to the fixed flow by simply looking for keyword fixedFlow
  //mapping between the name of the annotator to the path of it
  std::unordered_map<std::string, std::string> delegates;
  std::vector<std::string> annotators;
  getFixedFlow(file, annotators);

  for(std::string &a : annotators)
  {
    std::string path = rs::common::getAnnotatorPath(a);
    // If the path is yaml file, we need to convert it to xml
    if(boost::algorithm::ends_with(path, "yaml"))
    {

      YamlToXMLConverter converter(path);
      try
      {
        converter.parseYamlFile();
      }
      catch(YAML::ParserException e)
      {
        outError("Exception happened when parsing the yaml file: " << path);
        outError(e.what());
      }

      try
      {
        boost::filesystem::path p(path);

        // To Get $HOME path
        passwd *pw = getpwuid(getuid());
        std::string HOMEPath(pw->pw_dir);
        std::string xmlDir = HOMEPath + "/" + GEN_XML_PATH;
        std::string xmlPath = xmlDir + "/" +  a + ".xml";

        if(!boost::filesystem::exists(xmlDir))
          boost::filesystem::create_directory(xmlDir);
        std::ofstream of(xmlPath);
        converter.getOutput(of);
        of.close();
        delegates[a] = xmlPath;
      }
      catch(std::runtime_error &e)
      {
        outError("Exception happened when creating the output file: " << e.what());
        return;
      }
      catch(std::exception &e)
      {
        outError("Exception happened when creating the output file: " << e.what());
        return;
      }
    }
    else
      delegates[a] = path;
  }

  engine = (RSAggregatedAnalysisEngine *) rs::createParallelAnalysisEngine(file.c_str(), delegates, errorInfo);

  if(errorInfo.getErrorId() != UIMA_ERR_NONE)
  {
    outError("createAnalysisEngine failed.");
    throw std::runtime_error("An error occured during initializations;");
  }
  const uima::AnalysisEngineMetaData &data = engine->getAnalysisEngineMetaData();
  data.getName().toUTF8String(name_);

  // Get a new CAS
  outInfo("Creating a new CAS");
  cas = engine->newCAS();

  if(cas == NULL)
  {
    outError("Creating new CAS failed.");
    engine->destroy();
    delete engine;
    engine = NULL;
    throw uima::Exception(uima::ErrorMessage(UIMA_ERR_ENGINE_NO_CAS), UIMA_ERR_ENGINE_NO_CAS, uima::ErrorInfo::unrecoverable);
  }

  parallel_ = parallel;

}

void RSAnalysisEngine::initPipelineManager()
{ 
#ifdef WITH_JSON_PROLOG
  if(parallel_)
  {
    engine->initParallelPipelineManager();
    engine->parallelPlanner.print();
  }
#endif
}

void RSAnalysisEngine::stop()
{
  engine->collectionProcessComplete();
  engine->destroy();

  outInfo("Analysis engine stopped: " << name_);
}

void RSAnalysisEngine::process()
{
  outInfo("executing analisys engine: " << name_);
  try
  {
    UnicodeString ustrInputText;
    ustrInputText.fromUTF8(name_);
    cas->setDocumentText(uima::UnicodeStringRef(ustrInputText));

    rs::StopWatch clock;
    outInfo("processing CAS");
    try
    {
#ifdef WITH_JSON_PROLOG
      if(parallel_)
      {
        if(engine->querySuccess)
        {
          engine->paralleledProcess(*cas);
        }
        else
        {
          outWarn("Query annotator dependency for planning failed! Fall back to linear execution!");
          engine->process(*cas);
        }
      }
      else
      {
        engine->process(*cas);
      }
#else
      engine->process(*cas);
#endif
    }
    catch(const rs::FrameFilterException &)
    {
      //we could handle image logging here
      //handle extra pipeline here->signal thread that we can start processing
      outError("Got Interrputed with Frame Filter, not time here");
    }

    outInfo("processing finished");
    outInfo(clock.getTime() << " ms." << std::endl << std::endl << FG_YELLOW
            << "********************************************************************************" << std::endl);
  }
  catch(const rs::Exception &e)
  {
    outError("Exception: " << std::endl << e.what());
  }
  catch(const uima::Exception &e)
  {
    outError("Exception: " << std::endl << e);
  }
  catch(const std::exception &e)
  {
    outError("Exception: " << std::endl << e.what());
  }
  catch(...)
  {
    outError("Unknown exception!");
  }
}

void RSAnalysisEngine::getFixedFlow(const std::string filePath,
                                    std::vector<std::string> &annotators)
{
  try
  {
    std::ifstream fs(filePath);
    size_t pos, pos_;

    std::stringstream buffer;
    buffer << fs.rdbuf();
    std::string content = buffer.str();

    if((pos = content.find("<fixedFlow>")) != std::string::npos)
      content = content.substr(pos + 11);
    else
    {
      outError("There is no Fixed Flow specified in the given AE xml file.");
    }

    if((pos_ = content.find("</fixedFlow>")) != std::string::npos)
      content = content.substr(0, pos_);
    else
    {
      outError("There is no </fixedFlow> tag in the given xml file.");
    }

    pos = 0;
    while(pos < content.size())
    {
      if((pos = content.find("<node>", pos)) == std::string::npos)
        break;
      else
      {
        pos += 6;
        if((pos_ = content.find("</node>", pos)) != std::string::npos)
        {
          std::string anno = content.substr(pos, pos_ - pos);
          annotators.push_back(anno);
          pos = pos_ + 7;
        }
        else
        {
          outError("There is no </node> tag in the given xml file.");
        }
      }
    }
  }
  catch(std::exception &e)
  {
    outError("Exception happened when reading the file: " << e.what());
  }
}
