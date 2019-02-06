#include <rs/queryanswering/SWIPLInterface.h>


PrologInterface::PrologInterface()
{
  char *argv[4];
  int argc = 0;
  argv[argc++] = "PrologEngine";
  argv[argc++] = "-f";
  std::string rosPrologInit = ros::package::getPath("rosprolog") + "/prolog/init.pl";
  argv[argc] = new char[rosPrologInit.size() + 1];
  std::copy(rosPrologInit.begin(), rosPrologInit.end(), argv[argc]);
  argv[argc++][rosPrologInit.size()] = '\0';
  argv[argc] = NULL;
  engine  = std::make_shared<PlEngine>(argc, argv);

  outError("PROLOG ENGINE BEING INITIALIZED");
  init();
}

void PrologInterface::init()
{
  outInfo("Initializing Prolog Engine");
  PlTerm av("knowrob_robosherlock");
  try
  {
    PlCall("register_ros_package", av);
  }
  catch(PlException &ex)
  {
    outInfo((char *)ex);
  }
}

bool PrologInterface::extractQueryKeysFromDesignator(std::string &desig,
                                    std::vector<std::string> &keys)
{
	
  return true;
}


bool PrologInterface::buildPrologQueryFromDesignator(std::string &desig,
    std::string &prologQuery)
{

    prologQuery = "build_single_pipeline_from_predicates([";
    std::vector<std::string> queriedKeys;
    extractQueryKeysFromDesignator(desig,queriedKeys);
    for(int i = 0; i < queriedKeys.size(); i++)
    {
      prologQuery += queriedKeys.at(i);
      if(i < queriedKeys.size() - 1)
      {
        prologQuery += ",";
      }
    }
    prologQuery += "], A)";
    return true;
}
std::vector< std::string > PrologInterface::createPipelineFromPrologResult(std::string queryResult)
{
  std::vector<std::string> new_pipeline;

  // Strip the braces from the result
  queryResult.erase(queryResult.end() - 1);
  queryResult.erase(queryResult.begin());

  std::stringstream resultstream(queryResult);

  std::string token;
  while(std::getline(resultstream, token, ','))
  {
    // erase leading whitespaces
    token.erase(token.begin(), std::find_if(token.begin(), token.end(), std::bind1st(std::not_equal_to<char>(), ' ')));
    outDebug("Planned Annotator by Prolog Planner " << token);

    // From the extracted tokens, remove the prefix
    std::string prefix("http://knowrob.org/kb/rs_components.owl#");
    int prefix_length = prefix.length();

    // Erase by length, to avoid string comparison
    token.erase(0, prefix_length);
    // outInfo("Annotator name sanitized: " << token );

    new_pipeline.push_back(token);
  }
  return new_pipeline;
}

