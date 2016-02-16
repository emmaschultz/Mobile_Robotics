// my_motion_control: 2nd version, includes "cancel" and "feedback"
// expects client to give an integer corresponding to a timer count, in seconds
// server counts up to this value, provides feedback, and can be cancelled any time
// re-use the existing action message, although not all fields are needed
// use request "input" field for timer setting input, 
// value of "fdbk" will be set to the current time (count-down value)
// "output" field will contain the final value when the server completes the goal request

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <my_motion_control/PathMsgAction.h>

int g_count = 0;
bool g_count_failure = false;

class MyMotionControl {
private:
    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    actionlib::SimpleActionServer<my_motion_control::PathMsgAction> as_;
    
    // here are some message types to communicate with our client(s)
    my_motion_control::PathMsgGoal goal_; // goal message, received from client
    my_motion_control::PathMsgResult result_; // put results here, to be sent back to the client when done w/ goal
    my_motion_control::PathMsgFeedback feedback_; // for feedback 
    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    int countdown_val_;


public:
    MyMotionControl(); //define the body of the constructor outside of class definition

    ~MyMotionControl(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<my_motion_control::PathMsgAction>::GoalConstPtr& goal);
};

MyMotionControl::MyMotionControl() : as_(nh_, "timer_action", boost::bind(&MyMotionControl::executeCB, this, _1),false) {
    //////////////////////TODO CHANGE NAME OF SERVER ABOVE FROM "timer_action"
    ROS_INFO("in constructor of MyMotionControl...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <my_motion_control::demoAction> customizes the simple action server to use our own "action" message 
// defined in our package, "my_motion_control", in the subdirectory "action", called "demo.action"
// The name "demo" is prepended to other message types created automatically during compilation.
// e.g.,  "demoAction" is auto-generated from (our) base name "demo" and generic name "Action"
void MyMotionControl::executeCB(const actionlib::SimpleActionServer<my_motion_control::PathMsgAction>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB");
    ROS_INFO("goal input is: %d", goal->input);
    //do work here: this is where your interesting code goes
    ros::Rate timer(1.0); // 1Hz timer
    countdown_val_ = goal->input;
    //implement a simple timer, which counts down from provided countdown_val to 0, in seconds
    while (countdown_val_>0) {
       ROS_INFO("countdown = %d",countdown_val_);
       
       // each iteration, check if cancellation has been ordered
       if (as_.isPreemptRequested()){	
          ROS_WARN("goal cancelled!");
          result_.output = countdown_val_;
          as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
          return; // done with callback
 		}
 	
 	   //if here, then goal is still valid; provide some feedback
 	   feedback_.fdbk = countdown_val_; // populate feedback message with current countdown value
 	   as_.publishFeedback(feedback_); // send feedback to the action client that requested this goal
       countdown_val_--; //decrement the timer countdown
       timer.sleep(); //wait 1 sec between loop iterations of this timer
    }
    //if we survive to here, then the goal was successfully accomplished; inform the client
    result_.output = countdown_val_; //value should be zero, if completed countdown
    as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "timer_action_server_node"); // name this node 

    ROS_INFO("instantiating the timer_action_server: ");

    MyMotionControl as_object; // create an instance of the class "MyMotionControl"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ros::spin();

    return 0;
}

