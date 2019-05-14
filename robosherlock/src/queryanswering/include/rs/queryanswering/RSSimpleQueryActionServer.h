#include <robosherlock_msgs/RSSimpleQueryAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <rs/flowcontrol/RSProcessManager.h>
#include <sstream>
#include <string.h>
#include <std_msgs/String.h>
#include <vector>
#include <iostream>



 typedef actionlib::SimpleActionServer<robosherlock_msgs::RSSimpleQueryAction> Server;


class RSSimpleQueryActionServer{

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
    RSSimpleQueryActionServer(ros::NodeHandle n, RSProcessManager* rsMngr);

    /*
     * Destructor
     */
    ~RSSimpleQueryActionServer();


    /*
     * sendFeedback: This function publishes from time to time feedback about the query execution
     */
    void sendFeedback(std::basic_string<char> status);

     /*
      * Execute: Core function of the action server
      */
    static void executeQuery(const robosherlock_msgs::RSSimpleQueryGoalConstPtr& goal,RSSimpleQueryActionServer* rSSimpleQueryActionServer,RSProcessManager* rsMngr);

     /*
      * Action Server's Start point
      */
     int run(ros::NodeHandle n, RSProcessManager* rsMngr);

     /*
      * Convert ObjectDesignator to Json String
      */

     std::string* objectDesignator_to_jsonString(const robosherlock_msgs::RSSimpleQueryGoalConstPtr obj);

     /*
      * Convert Json String to ObjectDesignator
      */

     robosherlock_msgs::ObjectDesignator* jsonString_to_objectDesignator(std::string jsonStr);



};




