#include <robosherlock_msgs/RSQueryAction.h>  // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <stdio.h>

typedef actionlib::SimpleActionClient<robosherlock_msgs::RSQueryAction> Client;

class RSQueryActionClient
{
private:
  /*
   * Attributes
   */

  Client* client;

public:
  /*
   * Constructor
   */
  RSQueryActionClient(std::string topicName);

  /*
   * Destructor
   */

  ~RSQueryActionClient();

  /*
   *   SimpleDoneCallback: On query completion
   */
  static void getResult(const actionlib::SimpleClientGoalState& state,
                        const robosherlock_msgs::RSQueryResultConstPtr& result,
                        RSQueryActionClient* RSQueryActionClient);

  /*
   *   SimpleFeedbackCallback: On Feedback
   */
  static void getFeedback(const robosherlock_msgs::RSQueryFeedbackConstPtr& feedback,
                          RSQueryActionClient* RSQueryActionClient);

  /*
   *   SimpleActiveCallback: On Goal activation
   */
  static void getActivation(RSQueryActionClient* RSQueryActionClient);

  /*
   *PrepareGoal: prepare the query before sending to server
   */

  void prepareGoal(robosherlock_msgs::RSQueryGoal& goal);

  /*
   *Action client's Start point
   */
  int run();

  /*
   *Provide help information about the action client
   */
  static void help();
};
