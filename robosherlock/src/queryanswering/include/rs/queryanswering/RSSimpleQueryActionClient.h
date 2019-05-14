#include <robosherlock_msgs/RSSimpleQueryAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <stdio.h>

typedef actionlib::SimpleActionClient<robosherlock_msgs::RSSimpleQueryAction> Client;



class RSSimpleQueryActionClient{

private:
    /*
     * Attributes
     */

    Client* client;

public:

    /*
     * Constructor
     */
    RSSimpleQueryActionClient(int argc, char** argv);

    /*
     * Destructor
     */

    ~RSSimpleQueryActionClient();

    /*
     *   SimpleDoneCallback: On query completion
     */
    static void getResult(const actionlib::SimpleClientGoalState & state, const robosherlock_msgs::RSSimpleQueryResultConstPtr & result, RSSimpleQueryActionClient* rsSimpleQueryActionClient);


    /*
     *   SimpleFeedbackCallback: On Feedback
     */
    static void getFeedback(const robosherlock_msgs::RSSimpleQueryFeedbackConstPtr & feedback,RSSimpleQueryActionClient* rsSimpleQueryActionClient);


    /*
     *   SimpleActiveCallback: On Goal activation
     */
   static void getActivation(RSSimpleQueryActionClient* rsSimpleQueryActionClient);

    /*
     *PrepareGoal: prepare the query before sending to server
     */

    void prepareGoal(robosherlock_msgs::RSSimpleQueryGoal& goal);


    /*
     *Action client's Start point
     */
    int run();
};
