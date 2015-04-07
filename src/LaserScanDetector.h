#pragma once

#include <ros/node_handle.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/thread/mutex.hpp>
#include <fstream>

namespace Data
{
	class LaserScanDetector
	{
		private:
			static const int NUMBER_OF_LASER_SCANS = 1081;
		
			std::map<std::string,geometry_msgs::Vector3> robotPoses;
			std::vector<ros::Subscriber> subscriberRobotPoses;
			std::vector<double> scanRangeData;
			ros::NodeHandle nodeHandle;
			ros::Publisher publisherObjectDetected;
			ros::Subscriber subscriberLaserScan;
			boost::mutex mutex, mutexScan;
			std::ofstream groundtruth;
			double maxScanAngle, minScanAngle, maxReading, scanAngleIncrement;
			int agentId, endingPreyRobot, startingPreyRobot;
		
			bool checkObjectDetection(const geometry_msgs::Vector3& robotPose, const geometry_msgs::Vector3& objectPoseGlobalFrame);
			void writeGroundtruth();
		
		public:
			LaserScanDetector();
		
			virtual ~LaserScanDetector();
		
			void exec();
			void updateLaserScan(const sensor_msgs::LaserScan::ConstPtr& message);
			void updateRobotPose(const nav_msgs::Odometry::ConstPtr& message);
	};
}
