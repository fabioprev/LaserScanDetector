#include "LaserScanDetector.h"
#include <ros/ros.h>

int main (int argc, char** argv)
{
	ros::init(argc,argv,"LaserScanDetector");
	
	LaserScanDetector laserScanDetector;
	
	while (ros::ok())
	{
		ros::spinOnce();
		
		laserScanDetector.exec();
		
		usleep(5e3);
	}
	
	return 0;
}
