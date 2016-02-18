// mobot_motion_control: 2nd version, includes "cancel" and "feedback"
// expects client to give an integer corresponding to a timer count, in seconds
// server counts up to this value, provides feedback, and can be cancelled any time
// re-use the existing action message, although not all fields are needed
// use request "input" field for timer setting input, 
// value of "fdbk" will be set to the current time (count-down value)
// "output" field will contain the final value when the server completes the goal request

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <mobot_motion_control/PathMsgAction.h>

int g_count = 0;
bool g_count_failure = false;

class MyMotionControl {
private:
    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    actionlib::SimpleActionServer<mobot_motion_control::PathMsgAction> as_;
    ros::Publisher vel_pub;
    //////////////////add in a publisher that publishes to cmd_vel topic
    
    // here are some message types to communicate with our client(s)
    mobot_motion_control::PathMsgGoal goal_; // goal message, received from client
    mobot_motion_control::PathMsgResult result_; // put results here, to be sent back to the client when done w/ goal
    mobot_motion_control::PathMsgFeedback feedback_; // for feedback 
    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client


public:
    MyMotionControl(); //define the body of the constructor outside of class definition

    ~MyMotionControl(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<mobot_motion_control::PathMsgAction>::GoalConstPtr& goal);
};

MyMotionControl::MyMotionControl() : as_(nh_, "mobot_action", boost::bind(&MyMotionControl::executeCB, this, _1),false) {
    ROS_INFO("in constructor of MyMotionControl...");
    // do any other desired initializations here...specific to your implementation
    vel_pub = nh_.advertise<geometry_msgs::Twist>("/mobot/cmd_vel", 1);  //TODO IS THIS THE CORRECT ROBOT NAME?

    as_.start(); //start the server running
}

void MyMotionControl::executeCB(const actionlib::SimpleActionServer<mobot_motion_control::PathMsgAction>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB");
    //do work here: this is where your interesting code goes
    //refer to goal as goal->nav_path
    nav_msgs::Path path = goal->nav_path;
    ros::Rate timer(1.0); // 1Hz timer

    while (ros::ok() /*&& there are still poses to be executed*/) {
       // each iteration, check if cancellation has been ordered
       if (as_.isPreemptRequested()) {	
        	ROS_WARN("goal cancelled!");
        	result_.completed = false;
        	as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
        	return;
 		}
 	
 	   //if here, then goal is still valid; provide some feedback
 	   feedback_.fdbk = 5; //this should maybe be nav_path.header.seq TODO GIVE SOME BETTER FEEDBACK populate feedback message with current pose
 	   as_.publishFeedback(feedback_); // send feedback to the action client that requested this goal
 	   //go to next pose
       timer.sleep(); //wait 1 sec between loop iterations of this timer
    }
    //if we survive to here, then the goal was successfully accomplished; inform the client
    result_.completed = true; //value should be zero, if completed countdown
    as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mobot_action_server"); // name this node

    ROS_INFO("instantiating the timer_action_server: ");

    MyMotionControl as_object; // create an instance of the class "MyMotionControl"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ros::spin();

    return 0;
}