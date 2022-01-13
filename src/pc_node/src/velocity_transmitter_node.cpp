#include <ros/ros.h>
//#include <ros/Rate.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist vel_msg;

void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg){
	vel_msg.linear.x = msg->linear.x;
	vel_msg.linear.y = msg->linear.y;

	vel_msg.angular.z = msg->angular.z;
	
	ROS_INFO("Lx = %.3f ; Ly = %.3f ; Az = %.3f", vel_msg.linear.x, vel_msg.linear.y, vel_msg.angular.z);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "vel_node");
	ros::NodeHandle nh;
	ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
	ros::Subscriber sub_vel = nh.subscribe("velocities", 5, velocityCallback);
	
	ros::Rate rate(2);
	
	vel_msg.linear.x = 0.;
	vel_msg.linear.y = 0.;
	vel_msg.linear.z = 0.;
	
	vel_msg.angular.x = 0.;
	vel_msg.angular.y = 0.;
	vel_msg.angular.z = 0.;
	
	ROS_INFO("Lx = %.3f ; Ly = %.3f ; Az = %.3f", vel_msg.linear.x, vel_msg.linear.y, vel_msg.angular.z);
	while(ros::ok()){
		pub_vel.publish(vel_msg);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}