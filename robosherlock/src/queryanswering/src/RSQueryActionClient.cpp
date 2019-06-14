#include <rs/queryanswering/RSQueryActionClient.h>

/*
 * Constructor
 */

RSQueryActionClient::RSQueryActionClient(std::string topicName)
{
  // action client + spin(true)
  client = new Client(topicName, true);
}

/*
 * Destructor
 */

RSQueryActionClient::~RSQueryActionClient()
{
}
/*
 *   SimpleDoneCallback: On query completion
 */
void RSQueryActionClient::getResult(const actionlib::SimpleClientGoalState& state,
                                    const robosherlock_msgs::RSQueryResultConstPtr& result,
                                    RSQueryActionClient* RSQueryActionClient)
{
  ROS_INFO("\n Query completed: %s!\n", state.toString().c_str());
  ROS_INFO("\n Query's result:\n\n");
  std::cout << *result;
}

/*
 *   SimpleFeedbackCallback: On Feedback
 */
void RSQueryActionClient::getFeedback(const robosherlock_msgs::RSQueryFeedbackConstPtr& feedback,
                                      RSQueryActionClient* RSQueryActionClient)
{
  ROS_INFO("\n Query Feedback: %s!\n", (feedback->status.data()));
}

/*
 *   SimpleActiveCallback: On Goal activation
 */
void RSQueryActionClient::getActivation(RSQueryActionClient* RSQueryActionClient)
{
  ROS_INFO("\n Query is being processed!\n");
}

/*
 *PrepareGoal: prepare the query before sending to server
 * Remember that this function is only reserved for testing
 */

void RSQueryActionClient::prepareGoal(robosherlock_msgs::RSQueryGoal& goal)
{
  std::basic_string<char> valstr;
  /*valstr="Mug_01";
  goal.obj.uid.swap(valstr);
  valstr="Mug";
  goal.obj.type.swap(valstr);
  valstr="Sink_03";
  goal.obj.location.swap(valstr);
  valstr="Small";
  goal.obj.size.swap(valstr);*/

  std::vector<std::basic_string<char>> valarrstr = { "Kinect_Camera", "MongoDB", "BagFile" };
  /*for(int i=0;i<valarrstr.size();i++)
   goal.obj.poseSource.push_back(valarrstr[i]);*/

  valarrstr = { "white" };
  for (int i = 0; i < valarrstr.size(); i++)
    goal.obj.color.push_back(valarrstr[i]);

  valarrstr = { "round" };
  for (int i = 0; i < valarrstr.size(); i++)
    goal.obj.shape.push_back(valarrstr[i]);

  // print goal for clarification
  std::cout << goal;
}

int RSQueryActionClient::run()
{
  // wait for server to be ready
  this->client->waitForServer();
  robosherlock_msgs::RSQueryGoal goal;

  // prepare goal/query
  this->prepareGoal(goal);

  // prepare callbacks to respond to signals from action server (query processor)
  boost::function<void(const actionlib::SimpleClientGoalState& state,
                       const robosherlock_msgs::RSQueryResultConstPtr& result)>
      onResult = boost::bind(&RSQueryActionClient::getResult, _1, _2, this);
  boost::function<void(const robosherlock_msgs::RSQueryFeedbackConstPtr& feedback)> onFeedback =
      boost::bind(&RSQueryActionClient::getFeedback, _1, this);
  boost::function<void()> onActivation = boost::bind(&RSQueryActionClient::getActivation, this);
  client->sendGoal(goal, onResult, onActivation, onFeedback);

  // wait a bit for result: just for testing in terminal standalone mode
  client->waitForResult(ros::Duration(25.0));

  // print the final state of the query before exiting
  ROS_INFO("\n Current State on Exit: %s!\n", client->getState().toString().c_str());
  return 0;
}

void RSQueryActionClient::help()
{
  std::cout << "Usage: rosrun robosherlock RSQueryActionClient [topic_name]" << std::endl
            << "topic_name" << std::endl
            << "               _tn:=topic         Name of topic linking action client and server" << std::endl
            << std::endl
            << std::endl;
}

int main(int argc, char** argv)
{
  // ros node
  ros::init(argc, argv, std::string("rs_query_action_client"));
  ros::NodeHandle nh("~");
  // parameters
  std::string topicName;

  nh.param("tn", topicName, std::string(""));
  nh.deleteParam("tn");

  // if only argument is an tn (nh.param reudces argc)
  if ((argc < 2 && topicName.empty()) || argc >= 2)
  {
    RSQueryActionClient::help();
    return 0;
  }
  else
  {
    const std::string arg = argv[1];
    if (arg == "-?" || arg == "-h" || arg == "--help")
    {
      RSQueryActionClient::help();
      return 0;
    }
  }

  // action client for query answering
  RSQueryActionClient actionClient(topicName);
  return actionClient.run();
}
