#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

double frontDistance = 100000000000000000;  
double rightDistance = 100000000000000000;
double leftDistance= 100000000000000000; 

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	frontDistance = 100000000000000000;
	rightDistance = 100000000000000000;
	leftDistance = 100000000000000000;

	// loop through the ranges
	for( int i = 0; i < msg->ranges.size(); i++ )
	{
		float angle = msg->angle_min + i*msg->angle_increment;
		double xDistance = msg->ranges[i] * sin(angle);
		double yDistance = msg->ranges[i] * cos(angle);

		// Check whether the x/y coordinate is closer than the closest frontDistance
		if( xDistance > -0.3 && xDistance < 0.3 && yDistance < frontDistance )
		{
			// Set the frontDistance to be the current distance
			frontDistance = yDistance;
		}
		// Check whether the x/y coordinate is closer than the closest rightDistance
		if( yDistance > 0.25 && xDistance > 0 )
		{
			// Set the rightDistance to be the current distance
			double distance = sqrt ( yDistance*yDistance + xDistance*xDistance );
			if( distance < rightDistance )
				rightDistance = distance;
		}
		// Check whether the x/y coordinate is closer than the closest leftDistance
		if( yDistance > 0.25 && xDistance < 0 )
		{
			// Set the leftDistance to be the current distance
			double distance = sqrt ( yDistance*yDistance + xDistance*xDistance );
			if( distance < leftDistance )
				leftDistance = distance;
		}
	}
}

int main( int argc, char* argv[] )
{
	// Initializes a connection to the robot
	ros::init(argc,argv,"wanderer");
	ros::NodeHandle nodeHandle;
	ros::Rate loop_rate(10);

	ros::Subscriber laser_sub = nodeHandle.subscribe("/robot_0/base_scan", 1000, laserCallback);
	ros::Publisher cmd_vel = nodeHandle.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel",1000);
	geometry_msgs::Twist cmd_vel_msg;
	double des_vel = 1.0;

	while( ros::ok() )
	{
		// Robot move forward at 1 m/s by default
		double lvel = des_vel;		
		// Robot will not turn by default
		double rvel = 0;
		printf( "Front Distance: %0.2f Right Distance: %0.2f Left Distance: %0.2f\n", frontDistance, rightDistance, leftDistance);

		//Robot slows down if there is any obstacle in the front
		if( frontDistance < 0.75 )
			lvel = frontDistance - 0.35;
		// Robot turns left if there is any obstacle in the right
		if( rightDistance < leftDistance + 0.1 )
			rvel = -0.5;
		//Robot turns right if there is any obstacle in the left
		if( leftDistance < rightDistance + 0.1 )
			rvel = 0.5;

		// send the speeds to the robot
		cmd_vel_msg.linear.x = lvel ;
		cmd_vel_msg.angular.z = rvel;
		cmd_vel.publish(cmd_vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
