#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_datatypes.h>

//Get the distance between two robots
double get_distance( double wanderer_x, double wanderer_y, double follower_x, double follower_y )
{
    	return sqrt( (wanderer_x-follower_x) * (wanderer_x-follower_x) + (wanderer_y-follower_y) * (wanderer_y-follower_y));
}


//Get the bearing (from the blue robot's perspective) of the red
double get_bearing( double wanderer_x, double wanderer_y, double follower_x, double follower_y, double follower_t )
{
 	double bear = atan2( wanderer_y-follower_y, wanderer_x-follower_x ) - follower_t;
    	while( bear > M_PI ) bear -= 2*M_PI;
    	while( bear < -M_PI ) bear += 2*M_PI;
    	return bear;
}


double wanderer_x, wanderer_y, follower_x, follower_y;
double wanderer_theta, follower_theta;

double getRotationFromMsg( geometry_msgs::Quaternion q )
{

  	double yawValue;
  	yawValue = tf::getYaw(q);
  	return yawValue;
}

void odomCallbackWanderer(const nav_msgs::Odometry::ConstPtr& msg)
{
  	wanderer_x = msg->pose.pose.position.x;
  	wanderer_y = msg->pose.pose.position.y;
  	wanderer_theta = getRotationFromMsg(msg->pose.pose.orientation);
}

void odomCallbackFollower(const nav_msgs::Odometry::ConstPtr& msg)
{
  	follower_x = msg->pose.pose.position.x;
  	follower_y = msg->pose.pose.position.y;
  	follower_theta = getRotationFromMsg(msg->pose.pose.orientation);
}


int main( int argc, char* argv[] )
{
	// Initializes a connection to the robot
	ros::init(argc,argv,"follower");
	ros::NodeHandle nodeHandle;
	ros::Rate loop_rate(10);

	ros::Publisher cmd_vel = nodeHandle.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel",1000);
  	ros::Subscriber odom_sub = nodeHandle.subscribe("/robot_0/base_pose_ground_truth",1000,odomCallbackWanderer);
  	ros::Subscriber odom2_sub = nodeHandle.subscribe("/robot_1/base_pose_ground_truth",1000,odomCallbackFollower);

	geometry_msgs::Twist cmd_vel_msg;
	double des_vel = 1.0;

	while( ros::ok() )
	{
		// Robot will move forward at 1 m/s by default
		double range = des_vel;
		
		// Robot will not turn by default
		double bearing = 0;
	
    		bearing = get_bearing( wanderer_x, wanderer_y, follower_x, follower_y, follower_theta ) ;
    		range = get_distance( wanderer_x, wanderer_y, follower_x, follower_y );
  
    		if( range < 1.0 ) range = 0;
		
		// send the speeds to the robot
		cmd_vel_msg.linear.x = range;
		cmd_vel_msg.angular.z = bearing;
		cmd_vel.publish(cmd_vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
