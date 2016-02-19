
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <mobot_motion_control/PathMsgAction.h>

const double g_move_speed = 1.0; //mobot will move at 1 m/s
const double g_spin_speed = 1.0; //mobot will spin at 1 rad/s
const double g_sample_dt = 0.01;

class MobotMotionControl {
private:
    ros::NodeHandle nh_;

    actionlib::SimpleActionServer<mobot_motion_control::PathMsgAction> as_;
    ros::Publisher vel_pub;
    geometry_msgs::Twist g_twist_cmd;
    geometry_msgs::Pose g_current_pose;

    mobot_motion_control::PathMsgGoal goal_;
    mobot_motion_control::PathMsgResult result_;
    mobot_motion_control::PathMsgFeedback feedback_;

    //utility functions:
    double sgn(double x);
    double min_spin(double spin_angle);
    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
    geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);
    void do_inits();
    void do_halt();
    void do_spin(double spin_ang);
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
    vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    do_inits();
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

void MobotMotionControl::do_inits() {
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
    ros::Rate loop_timer(1 / g_sample_dt);   
    g_twist_cmd.angular.z = 0.0;
    g_twist_cmd.linear.x = 0.0;
    for (int i = 0; i < 10; i++) {
        vel_pub.publish(g_twist_cmd);
        loop_timer.sleep(); 
    }
}

//a function to reorient by a specified angle (in radians), then halt
void MobotMotionControl::do_spin(double spin_ang) {
    ros::Rate loop_timer(1 / g_sample_dt);
    double timer = 0.0;
    double final_time = fabs(spin_ang) / g_spin_speed;
    g_twist_cmd.angular.z = sgn(spin_ang) * g_spin_speed;
    while(timer < final_time) {
    	vel_pub.publish(g_twist_cmd);
    	timer += g_sample_dt;
    	loop_timer.sleep(); 
    }  
    do_halt();
}

// a function to move forward by a specified distance (in meters), then halt
// always assumes robot is already oriented properly
// but allow for negative distance to mean move backwards
void MobotMotionControl::do_move(double distance) {
    ros::Rate loop_timer(1 / g_sample_dt);
    double timer = 0.0;
    double final_time = fabs(distance) / g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance) * g_move_speed;
    while(timer < final_time) {
    	vel_pub.publish(g_twist_cmd);
    	timer += g_sample_dt;
    	loop_timer.sleep(); 
    }  
    do_halt();
}

void MobotMotionControl::executeCB(const actionlib::SimpleActionServer<mobot_motion_control::PathMsgAction>::GoalConstPtr& goal) {
	ROS_INFO("in executeCB");
	std::vector<double> spin_angle = goal->angle;
	std::vector<double> travel_distance = goal->distance;

	int num_angle = spin_angle.size();
	ros::Rate timer(100.0); //100 Hz timer
	int i = 0;
	while(ros::ok() && i < num_angle){
		if(as_.isPreemptRequested()){
			ROS_WARN("goal cancelled!");
			result_.completed = false;
			as_.setAborted(result_);
			return;
		}
		//send feedback to tell the client which sub-pose you are on
		feedback_.fdbk = i;
		as_.publishFeedback(feedback_);

		//execute sub-pose
		do_spin(spin_angle[i]);
		do_move(travel_distance[i]);
		do_halt();

		i++;
		timer.sleep();
	}
	do_halt();
	result_.completed = true;
	as_.setSucceeded(result_);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mobot_action_server"); // name this node

    ROS_INFO("instantiating the timer_action_server: ");

    MobotMotionControl as_object; // create an instance of the class "MobotMotionControl"
    
    ROS_INFO("going into spin");

    ros::spin();

    return 0;
}