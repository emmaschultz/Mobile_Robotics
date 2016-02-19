// mobot_motion_control: 2nd version, includes "cancel" and "feedback"
// expects client to give an integer corresponding to a timer count, in seconds
// server counts up to this value, provides feedback, and can be cancelled any time
// re-use the existing action message, although not all fields are needed
// use request "input" field for timer setting input, 
// value of "fdbk" will be set to the current time (count-down value)
// "output" field will contain the final value when the server completes the goal request

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>  //TODO IS THIS NEEDED?
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>  //TODO IS THIS NEEDED?
#include <mobot_motion_control/PathMsgAction.h>

class MobotMotionControl {
private:
    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    actionlib::SimpleActionServer<mobot_motion_control::PathMsgAction> as_;
    ros::Publisher vel_pub;
    //////////////////add in a publisher that publishes to cmd_vel topic
    const double move_speed_ = 1.0; //mobot will move at 1 m/s
    const double spin_speed_ = 1.0; //mobot will spin at 1 rad/s
    const double dt_ = 0.01;
    geometry_msgs::Twist g_twist_cmd;
    geometry_msgs::Pose g_current_pose;   //TODO IS THIS NECESSARY?

    mobot_motion_control::PathMsgGoal goal_;
    mobot_motion_control::PathMsgResult result_;
    mobot_motion_control::PathMsgFeedback feedback_;

    //utility functions:
    double sgn(double x);
    double min_spin(double spin_angle);
    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
    geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);
    void do_inits(ros::NodeHandle &nh_);
    void do_halt();
    void do_spin(double spin_angle);
    void do_move(double distance);

public:
    MobotMotionControl(); //define the body of the constructor outside of class definition

    ~MobotMotionControl(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<mobot_motion_control::PathMsgAction>::GoalConstPtr& goal);
};

MobotMotionControl::MobotMotionControl() : as_(nh_, "mobot_action", boost::bind(&MobotMotionControl::executeCB, this, _1),false) {
    ROS_INFO("in constructor of MobotMotionControl...");
    // do any other desired initializations here...specific to your implementation
    vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);  //TODO IS THIS THE CORRECT ROBOT NAME?
    do_inits(&nh_);
    as_.start(); //start the server running
}

//signum function: strip off and return the sign of the argument
double MobotMotionControl::sgn(double x) {
    if (x > 0.0) {
        return 1.0;
    } else if (x < 0.0) {
        return -1.0;
    } else {
        return 0.0;
    }
}

//a function to consider periodicity and find min delta angle
double MobotMotionControl::min_spin(double spin_angle) {
    if (spin_angle > M_PI) {
        spin_angle -= 2.0 * M_PI;
    }
    if (spin_angle < -M_PI) {
        spin_angle += 2.0 * M_PI;
    }

    return spin_angle;
}

// a useful conversion function: from quaternion to yaw
double MobotMotionControl::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion MobotMotionControl::convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

void MobotMotionControl::do_inits(ros::NodeHandle &n) {
    //initialize components of the twist command global variable
    g_twist_cmd.linear.x = 0.0;
    g_twist_cmd.linear.y = 0.0;    
    g_twist_cmd.linear.z = 0.0;
    g_twist_cmd.angular.x = 0.0;
    g_twist_cmd.angular.y = 0.0;
    g_twist_cmd.angular.z = 0.0;  
    
    //define initial position to be 0
    g_current_pose.position.x = 0.0;
    g_current_pose.position.y = 0.0;
    g_current_pose.position.z = 0.0;
    
    // define initial heading to be "0"
    g_current_pose.orientation.x = 0.0;
    g_current_pose.orientation.y = 0.0;
    g_current_pose.orientation.z = 0.0;
    g_current_pose.orientation.w = 1.0;  
}

void MobotMotionControl::do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i = 0; i < 10; i++) {
        g_twist_commander.publish(g_twist_cmd);
        loop_timer.sleep(); 
    }   
}

void MobotMotionControl::executeCB(const actionlib::SimpleActionServer<mobot_motion_control::PathMsgAction>::GoalConstPtr& goal) {
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

    MobotMotionControl as_object; // create an instance of the class "MobotMotionControl"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ros::spin();

    return 0;
}