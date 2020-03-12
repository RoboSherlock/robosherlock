#include <robosherlock_msgs/RSQueryAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <sstream>
#include <string.h>
#include <std_msgs/String.h>
#include <vector>
#include <iostream>
#include <robosherlock/queryanswering/QueryInterface.h>

// forward declaration of process manager's class
class RSProcessManager;

typedef actionlib::SimpleActionServer<robosherlock_msgs::RSQueryAction> Server;

class RSQueryActionServer
{
private:
  /*
   * Attributes
   */
  Server* server;
  RSProcessManager* rsProcessManager;

public:
  /*
   * Constructor
   */
  RSQueryActionServer(ros::NodeHandle n, RSProcessManager* rsMngr);

  /*
   * Destructor
   */
  ~RSQueryActionServer();

  /*
   * sendFeedback: This function publishes from time to time feedback about the query execution
   */
  void sendFeedback(std::basic_string<char> status);

  /*
   * Execute: Core function of the action server
   */
  static void executeQuery(const robosherlock_msgs::RSQueryGoalConstPtr& goal, RSQueryActionServer* RSQueryActionServer,
                           RSProcessManager* rsMngr);

  /*
   * Action Server's Start point
   */
  int run(ros::NodeHandle n, RSProcessManager* rsMngr);

  /*
   * Convert ObjectDesignator to Json String
   */

  std::string* objectDesignator_to_jsonString(const robosherlock_msgs::RSQueryGoalConstPtr obj);

  /*
   * Convert Json String to ObjectDesignator
   */

  robosherlock_msgs::ObjectDesignator* jsonString_to_objectDesignator(std::string jsonStr);
};
