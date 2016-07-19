#include <rs/utils/RSAnalysisEngine.h>


RSAnalysisEngine::RSAnalysisEngine() : engine(NULL), cas(NULL)
{
}

RSAnalysisEngine::~RSAnalysisEngine()
{
  if(cas)
  {
    delete cas;
    cas = NULL;
  }
  if(engine)
  {
    delete engine;
    engine = NULL;
  }
}

void RSAnalysisEngine::init(const std::string &file)
{
  uima::ErrorInfo errorInfo;

  size_t pos = file.rfind('/');
  outInfo("Creating analysis engine: " FG_BLUE << (pos == file.npos ? file : file.substr(pos)));

  engine = uima::Framework::createAnalysisEngine(file.c_str(), errorInfo);

  if(errorInfo.getErrorId() != UIMA_ERR_NONE)
  {
    outError("createAnalysisEngine failed.");
    throw uima::Exception(errorInfo);
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

  outInfo("initialization done: " << name << std::endl
          << std::endl << FG_YELLOW << "********************************************************************************" << std::endl);
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
      uima::CASIterator casIter = engine->processAndOutputNewCASes(*cas);

      for(int i = 0; casIter.hasNext(); ++i)
      {
        uima::CAS &outCas = casIter.next();

        // release CAS
        outInfo("release CAS " << i);
        engine->getAnnotatorContext().releaseCAS(outCas);
      }
    }
    catch(const rs::FrameFilterException &)
    {
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
  cas->reset();
}
