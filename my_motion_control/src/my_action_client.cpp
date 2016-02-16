#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <my_motion_control/PathMsgAction.h>


// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state, const my_motion_control::demoResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    int diff = result->output - result->goal_stamp;
    ROS_INFO("got result output = %d; goal_stamp = %d; diff = %d",result->output,result->goal_stamp,diff);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "action_client_node"); // name this node 
    int g_count = 0;
    my_motion_control::PathMsgGoal goal;

    // use the name of our server, which is: example_action (named in my_motion_control.cpp)
    // the "true" argument says that we want our new client to run as a separate thread (a good idea)
    actionlib::SimpleActionClient<my_motion_control::demoAction> action_client("example_action", true);
    /////////////////////////TODO CHANGE NAME OF SERVER FROM "example_action"

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
    //bool server_exists = action_client.waitForServer(); //wait forever

    if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        return 1; // bail out; optionally, could print a warning message and retry
    }

    ROS_INFO("connected to action server");  // if here, then we connected to the server;

    while(true) {
        // stuff a goal message:
        g_count++;
        goal.input = g_count; // this merely sequentially numbers the goals sent
        //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
        action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
        //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this

        bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result for goal number %d",g_count);
            return 1;
        } else {
            //if here, then server returned a result to us
        }
    }
    return 0;
}

