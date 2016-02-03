#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <math.h>


const double MIN_SAFE_DISTANCE = 0.5; // set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback
float ping_dist_ = 3.0;
int ping_index_ = -1;
double angle_min_ = 0.0;
double angle_max_ = 0.0;
double angle_increment_ = 0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_ = false;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
	if (ping_index_ < 0) {
        //for first message received, set up the desired index of LIDAR range to eval
		angle_min_ = laser_scan.angle_min;
		angle_max_ = laser_scan.angle_max;
		angle_increment_ = laser_scan.angle_increment;
		range_min_ = laser_scan.range_min;
		range_max_ = laser_scan.range_max;

		ping_index_ = (int) ((0.0 - angle_min_) / angle_increment_);

		ROS_INFO("LIDAR setup: ping_index = %d", ping_index_);
	}

	for(int i = ping_index_ - 200; i < ping_index_ + 200; i++){
		ping_dist_ = laser_scan.ranges[i];
		ROS_INFO("ping_dist_ = %f", ping_dist_);

		if(ping_dist_ < MIN_SAFE_DISTANCE) {
			ROS_WARN("DANGER WILL ROBINSON!!");
			laser_alarm_ = true;
			break;
		} else {
			laser_alarm_ = false;
		}
	}

    std_msgs::Bool lidar_alarm_msg;
    lidar_alarm_msg.data = laser_alarm_;
    lidar_alarm_publisher_.publish(lidar_alarm_msg);
    std_msgs::Float32 lidar_dist_msg;
    lidar_dist_msg.data = ping_dist_;
    lidar_dist_publisher_.publish(lidar_dist_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
    ros::spin();
    return 0;
}