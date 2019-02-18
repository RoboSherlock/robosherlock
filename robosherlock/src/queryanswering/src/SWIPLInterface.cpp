#include <rs/queryanswering/SWIPLInterface.h>

namespace rs
{
SWIPLInterface::SWIPLInterface(): engine1_(0)
{
  std::lock_guard<std::mutex> lock(lock_);
  char *argv[4];
  int argc = 0;
  argv[argc++] = "swi_test";
  argv[argc++] = "-f";
  std::string rosPrologInit = ros::package::getPath("robosherlock") + "/prolog/init.pl";
  argv[argc] = new char[rosPrologInit.size() + 1];
  std::copy(rosPrologInit.begin(), rosPrologInit.end(), argv[argc]);
  argv[argc++][rosPrologInit.size()] = '\0';
  argv[argc] = NULL;
  PL_initialise(argc, argv);

  attributes_.local_size = 100000000;
  attributes_.global_size = 100000000;
  attributes_.trail_size = 100000000;
  attributes_.argument_size = 0;
  attributes_.alias = 0;
  attributes_.cancel = 0;
  attributes_.flags = 0;

  void *current_engine, *main_engine;
  int res1 = PL_set_engine(PL_ENGINE_CURRENT, &current_engine);
  int res2 = PL_set_engine(PL_ENGINE_MAIN, &main_engine);

  outWarn("After initialization the main engine is: " << main_engine);
  outWarn("After initialization the current engine is: " << current_engine);
  outInfo("PROLOG ENGINE BEING INITIALIZED");
  //  outWarn("Engine1: " <<engine1_);
}


void SWIPLInterface::setEngine()
{
  PL_engine_t current_engine;
  int result = PL_set_engine(PL_ENGINE_CURRENT, &current_engine);
  outError("PL_CURRENT_ENGINE: " << current_engine);
  outError("ENGINE1: " << engine1_);
  if(current_engine == 0)
  {
    if(engine1_ == 0)
      engine1_ = PL_create_engine(&attributes_);
    //      PL_engine_t  new_engine = PL_create_engine(&attributes_);
    //      engine_pool_.insert(new_engine);
    int result = PL_set_engine(engine1_, 0);
    if(result != PL_ENGINE_SET)
    {
      if(result == PL_ENGINE_INVAL)
        throw std::runtime_error("Engine is invalid.");
      else if(result == PL_ENGINE_INUSE)
        throw std::runtime_error("Engine is invalid.");
      else
        throw std::runtime_error("Unknown Response when setting PL_engine");
    }
    else
      outInfo("Engine: " << engine1_ << " set successfully");
  }
}


bool SWIPLInterface::planPipelineQuery(const std::vector<std::string> &keys,
                                       std::vector<std::string> &pipeline)
{
  std::lock_guard<std::mutex> lock(lock_);
  outInfo("Planning Pipeline");
  setEngine();
  PlTermv av(2);
  PlTail l(av[0]);
  for(auto key : keys)
    l.append(key.c_str());
  l.close();

  //interesting; if I
  std::shared_ptr<PlQuery> q(new PlQuery("pipeline_from_predicates_with_domain_constraint", av));
  std::string prefix("http://knowrob.org/kb/rs_components.owl#");
  while(q->next_solution())
  {
    //      std::cerr<<(char*)av[1]<<std::endl;
    PlTail res(av[1]);//result is a list
    PlTerm e;//elements of that list
    while(res.next(e))
    {
      std::string element((char *)e);
      element.erase(0, prefix.length());
      pipeline.push_back(element);
    }
  }
  q.reset();
  releaseEngine();
  return true;
}

bool SWIPLInterface::q_subClassOf(std::string child, std::string parent)
{
  std::lock_guard<std::mutex> lock(lock_);
  outInfo("Planning Pipeline");
  setEngine();
  PlTermv av(2);
  av[0] =  "child";
  av[1] =  "parent";
  try
  {
    if(PlCall("owl_subclass_of", av))
    {
      outInfo(child << " is subclass of " << parent);
      releaseEngine();
      return true;
    }
    else
    {
      outInfo(child << " is NOT subclass of " << parent);
      releaseEngine();
      return false;
    }
  }
  catch(PlException &ex)
  {
    outError((char *)ex);
    releaseEngine();
    return false;
  }
}



bool SWIPLInterface::assertQueryLanguage(std::map<std::string, std::vector<std::string>> &query_terms)
{
  std::lock_guard<std::mutex> lock(lock_);
  outInfo("Asserting query language specific knowledge");
  setEngine();
  try
  {
    for(auto term : query_terms)
    {
      std::stringstream query;
      query << "assert(rs_query_reasoning:rs_query_predicate(" << term.first << "))";
      if(PlCall(query.str().c_str()))
      {
        outInfo("Asserted " << term.first << " as a query language term");
        for(auto type : term.second)
        {
          std::string token;
          std::stringstream ss(type), krType;
          while(std::getline(ss, token, '.'))
          {
            std::transform(token.begin(), token.end(), token.begin(), ::tolower);
            token[0] = std::toupper(token[0]);
            krType << token;
          }
          query.str("");
        }
      }
    }
  }
  catch(PlException &ex)
  {
    outError(static_cast<char *>(ex));
  }


  releaseEngine();
  return true;
}

bool SWIPLInterface::retractAllAnnotators()
{
  MEASURE_TIME;
  std::lock_guard<std::mutex> lock(lock_);
  outInfo("Retracting all annotators");
  setEngine();

  PlTermv av(0), av2(2), av3(2);
  av2[1] = "http://knowrob.org/kb/rs_components.owl#RoboSherlockComponent";
  av3[1] = "http://knowrob.org/kb/rs_components.owl#RoboSherlockComponent";

  {
    MEASURE_TIME;
    if(PlCall("assert_test_pipeline"))
      //    if(q->next_solution())
      outInfo("Success when asserting test pipeline");
    //    q.reset();
  }
  try
  {
    std::shared_ptr<PlQuery> check(new PlQuery("owl_individual_of", av2));
    {
      MEASURE_TIME;
      while(check->next_solution())
      {
        outInfo(static_cast<char *>(av2[0]));
        std::stringstream query;
        query << "rdf_db:rdf_retractall('" << static_cast<char *>(av2[0])
              << "','http://www.w3.org/1999/02/22-rdf-syntax-ns#type',_)";
        PlCall(query.str().c_str());
        //        outInfo("retracted: " << query.str());
      }
    }
    check.reset();
    std::shared_ptr<PlQuery> check2(new PlQuery("owl_individual_of", av3));
    int count = 0;
    while(check2->next_solution() && count++)
      outError(static_cast<char *>(av3[0]));
    outError("COUNT AFTER RETRACTING:" << count);
    check2.reset();
  }
  catch(PlException &ex)
  {
    outError(static_cast<char *>(ex));
  }
  releaseEngine();
  return true;
}


bool SWIPLInterface::retractQueryLanguage()
{
  std::lock_guard<std::mutex> lock(lock_);
  outInfo("Retracting Query language");
  setEngine();
  PlTermv av(0);

  std::shared_ptr<PlQuery> q(new PlQuery("assert_query_lang", av));
  q->next_solution();

  try
  {
    PlCall("retractall(rs_query_reasoning:rs_type_for_predicate(_,_))");
    PlCall("retractall(rs_query_reasoning:rs_query_predicate(_))");
  }
  catch(PlException &ex)
  {
    outError(static_cast<char *>(ex));
  }

  q.reset();
  releaseEngine();
  return true;
}

bool SWIPLInterface::addNamespace(std::string &s)
{
  std::lock_guard<std::mutex> lock(lock_);
  outInfo("Adding namespace to: " << s);
  setEngine();

  if(krNamespaces_.empty())
  {
    std::stringstream getNamespacesQuery;
    getNamespacesQuery << "rdf_current_ns(A,_)";

    PlTermv av(2);
//    PlQuery query ()
//    for(auto bdg : bdgs)
//    {
//      std::string ns = bdg["A"].toString();
//      if(std::find(rs::NS_TO_SKIP.begin(), rs::NS_TO_SKIP.end(), ns) == rs::NS_TO_SKIP.end())
//        krNamespaces_.push_back(ns);
//    }
  }

  releaseEngine();
  return true;
}

}//namespace rs
