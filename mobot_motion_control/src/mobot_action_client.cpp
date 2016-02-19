#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
//#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <mobot_motion_control/PathMsgAction.h>

bool g_alarm_activated = false;

// This function will be called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state, const mobot_motion_control::PathMsgResultConstPtr& result) {
    ROS_INFO("doneCb: server responded with state [%s]", state.toString().c_str());
    bool isSuccess = result->completed;
    if(isSuccess) {
    	ROS_INFO("Path was successfully completed.");
    } else {
    	ROS_INFO("Path was not completed.");
    }
}

//this function will be called when lidar alarm information is received
void alarmCb(const std_msgs::Bool& alarm_msg) {
    g_alarm_activated = alarm_msg.data;
    if (g_alarm_activated) {
        ROS_INFO("LIDAR alarm activated!");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mobot_action_client"); // name this node
    ros::NodeHandle nh;
    mobot_motion_control::PathMsgGoal goal;

    actionlib::SimpleActionClient<mobot_motion_control::PathMsgAction> action_client("mobot_action", true);

    ros::Subscriber lidar_alarm_sub = nh.subscribe("lidar_alarm", 1, alarmCb);  //subscriber listening to LIDAR alarm

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(); //wait forever

    if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        return 1;
    }

    ROS_INFO("connected to action server");  // if here, then we connected to the server

    ros::Rate timer(10);

    while(ros::ok()) {
        if(g_alarm_activated) {
        	//cancel all goals in order to send a new goal to avoid whatever obstacle is in the way
            action_client.cancelAllGoals();
            ROS_INFO("Current goal has been cancelled.");

            //turn and move a different direction because there is something in the robot's way
            ROS_INFO("Attempting to turn and move in alternate direction.");
            goal.distance.resize(1);
            goal.distance[0] = 3;

            goal.angle.resize(1);
            goal.angle[0] = 3.1415 / 2;

            action_client.sendGoal(goal, &doneCb);

        	bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        	ROS_INFO("Evasive maneuver achieved.");
            g_alarm_activated = false; //reset alarm

        } else {
        	//set up goal distances
        	ROS_INFO("Sending original goal.");

        	// tell robot to move straight forward 1 unit
        	goal.distance.resize(1);
        	goal.distance[0] = 1;

        	goal.angle.resize(1);
        	goal.angle[0] = 0;

        	/* could potentially send over multiple poses
        	goal.distance.resize(4);
        	goal.distance[0] = 4;
        	goal.distance[1] = 3;
        	goal.distance[2] = 4;
        	goal.distance[3] = 5;

        	//set up goal angles
        	goal.angle.resize(4);
        	goal.angle[0] = 0;
        	goal.angle[1] = 3.1415 / 2;
        	goal.angle[2] = 0;
        	goal.angle[3] = -3.1415 / 2;
        	*/

        	action_client.sendGoal(goal, &doneCb);
        }

        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}