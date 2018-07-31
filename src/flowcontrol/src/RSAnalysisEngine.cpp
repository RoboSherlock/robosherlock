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
  uima::ErrorInfo errorInfo;

  size_t pos = file.rfind('/');
  outInfo("Creating analysis engine: " FG_BLUE << (pos == file.npos ? file : file.substr(pos)));
  engine = rs::createParallelAnalysisEngine(file.c_str(), errorInfo);
  if(errorInfo.getErrorId() != UIMA_ERR_NONE)
  {
    outError("createAnalysisEngine failed.");
    throw std::runtime_error("An error occured during initializations;");
  }
  const uima::AnalysisEngineMetaData &data = engine->getAnalysisEngineMetaData();
  data.getName().toUTF8String(name);

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

  outInfo("initialization done: " << name << std::endl
          << std::endl << FG_YELLOW << "********************************************************************************" << std::endl);
}

void RSAnalysisEngine::initPipelineManager()
{
  rspm = new RSPipelineManager(engine, parallel_);
#ifdef WITH_JSON_PROLOG
  if(parallel_)
  {
    rspm->initParallelPipelineManager();
  }
#endif
}

void RSAnalysisEngine::stop()
{
  engine->collectionProcessComplete();
  engine->destroy();

  outInfo("Analysis engine stopped: " << name);
}

void RSAnalysisEngine::process()
{
  outInfo("executing analisys engine: " << name);
  try
  {
    UnicodeString ustrInputText;
    ustrInputText.fromUTF8(name);
    cas->setDocumentText(uima::UnicodeStringRef(ustrInputText));

    rs::StopWatch clock;
    outInfo("processing CAS");
    try
    {
#ifdef WITH_JSON_PROLOG
      if(parallel_)
      {
        if(rspm->querySuccess)
        {
          RSAggregatedAnalysisEngine *pEngine = (RSAggregatedAnalysisEngine *) engine;
          pEngine->paralleledProcess(*cas);
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
