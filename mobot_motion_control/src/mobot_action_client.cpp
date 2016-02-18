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
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // bool server_exists = action_client.waitForServer(); //wait forever

    if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        return 1;
    }

    ROS_INFO("connected to action server");  // if here, then we connected to the server

    //create and package up some goal poses
    //send these goal poses from the client to the server
    //make sure that nav_path.header.seq is filled in with a counter
    //then just use the loop to check whether alarm has been triggered?
    ros::Rate timer(100);

    while(ros::ok()) {
        if(g_alarm_activated) {
            //preempt previously sent goal
            //action_client.cancelAllGoals();
            action_client.cancelGoal();

            //TODO MAKE THIS TURN

            g_alarm_activated = false; //reset alarm
        } else {
        	//set up goal distances
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

        	action_client.sendGoal(goal, &doneCb);

        	/*
        	bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
        	//bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        	if (!finished_before_timeout) {
            	ROS_WARN("giving up waiting on result");
            	return 1;
        	} else {
            	//if here, then server returned a result to us
        	}
        	*/
        }

        //action_client.sendGoal(goal, &doneCb); // we could also name additional callback functions here, if desired
        //action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}