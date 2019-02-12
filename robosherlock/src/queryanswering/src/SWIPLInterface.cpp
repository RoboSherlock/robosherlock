#include <rs/queryanswering/SWIPLInterface.h>

namespace rs
{
SWIPLInterface::SWIPLInterface()
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

  attributes.local_size = 100000000;
  attributes.global_size = 100000000;
  attributes.trail_size = 100000000;
  attributes.argument_size = 0;
  attributes.alias = 0;
  attributes.cancel = 0;
  attributes.flags = 0;
//  engine1_ = PL_create_engine(&attributes);
  outInfo("PROLOG ENGINE BEING INITIALIZED");
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
  PlQuery q("pipeline_from_predicates_with_domain_constraint", av);
  std::string prefix("http://knowrob.org/kb/rs_components.owl#");
  while(q.next_solution())
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
      return true;
    }
    else
    {
      outInfo(child << " is NOT subclass of " << parent);
      return false;
    }
  }
  catch(PlException &ex)
  {
    outError((char *)ex);
    return false;
  }
  return false;
}
}