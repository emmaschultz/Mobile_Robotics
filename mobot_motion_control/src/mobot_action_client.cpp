#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <mobot_motion_control/PathMsgAction.h>

bool g_alarm_activated = false;

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state, const mobot_motion_control::PathMsgResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    int diff = result->output - result->goal_stamp;
    ROS_INFO("got result output = %d; goal_stamp = %d; diff = %d",result->output,result->goal_stamp,diff);
}

void executeCb(const std_msgs::Bool& alarm_msg) {
    g_alarm_activated = alarm_msg.data;
    if (g_alarm_activated) {
        ROS_INFO("LIDAR alarm activated!");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "action_client_node"); // name this node 
    ros::NodeHandle nh;
    mobot_motion_control::PathMsgGoal goal;

    actionlib::SimpleActionClient<mobot_motion_control::PathMsgAction> action_client("mobot_action", true);

    ros::Subscriber lidar_alarm_sub = nh.subscribe("lidar_alarm", 1, executeCb);  //subscriber listening to LIDAR alarm

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
    // bool server_exists = action_client.waitForServer(); //wait forever

    if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        return 1; // bail out; optionally, could print a warning message and retry
    }

    ROS_INFO("connected to action server");  // if here, then we connected to the server;

    while(ros::ok()) {
        if(g_alarm_activated) {
            //preempt previously sent goal
            action_client.cancelAllGoals();
            //action_client.cancelGoal();
            g_alarm_activated = false; //reset alarm
        } else {
            ////////do calculations and such here
            //make a path of subgoals (maybe just have it go straight?)
            //stuff this into nav_path message and send this to the server
            //but first check if alarm has been tripped
        }


        //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
        action_client.sendGoal(goal, &doneCb); // we could also name additional callback functions here, if desired
        //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this

        bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result for goal number %d",g_count);
            return 1;
        } else {
            //if here, then server returned a result to us
        }

        ros::spinOnce();
    }
    return 0;
}