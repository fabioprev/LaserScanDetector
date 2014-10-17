#include "LaserScanDetector.h"
#include <LaserScanDetector/Object.h>
#include <LaserScanDetector/ObjectDetection.h>
#include <tf/LinearMath/Matrix3x3.h>

using namespace std;
using LaserScanDetector::ObjectDetection;
using LaserScanDetector::Object;
using geometry_msgs::Quaternion;
using geometry_msgs::Vector3;
using nav_msgs::Odometry;
using sensor_msgs::LaserScan;

namespace Data
{
	LaserScanDetector::LaserScanDetector() : nodeHandle("~")
	{
		int numberOfRobots;
		
		nodeHandle.getParam("agentId",agentId);
		nodeHandle.getParam("numberOfRobots",numberOfRobots);
		nodeHandle.getParam("maxReading",maxReading);
		
		for (int i = 0; i < numberOfRobots; ++i)
		{
			stringstream s;
			
			s << "/robot_" << i << "/base_pose_ground_truth";
			
			subscriberRobotPoses.push_back(nodeHandle.subscribe(s.str(),1024,&LaserScanDetector::updateRobotPose,this));
		}
		
		subscriberLaserScan = nodeHandle.subscribe("scan",1024,&LaserScanDetector::updateLaserScan,this);
		
		publisherObjectDetected = nodeHandle.advertise<ObjectDetection>("objectDetected",1);
	}
	
	LaserScanDetector::~LaserScanDetector() {;}
	
	bool LaserScanDetector::checkObjectDetection(const Vector3& robotPose, const Vector3& objectPoseGlobalFrame)
	{
		static Vector3 objectPoseRobotFrame;
		static float cosTheta, dx, dy, robotDiameter = 0.3, sinTheta;
		static int range = 5;
		static int minIntervalRange, maxIntervalRange;
		
		dx = objectPoseGlobalFrame.x - robotPose.x;
		dy = objectPoseGlobalFrame.y - robotPose.y;
		
		cosTheta = cos(robotPose.z);
		sinTheta = sin(robotPose.z);
		
		// Target position w.r.t. the mobile frame (robot frame).
		objectPoseRobotFrame.x =  cosTheta * dx + sinTheta * dy;
		objectPoseRobotFrame.y = -sinTheta * dx + cosTheta * dy;
		objectPoseRobotFrame.z = atan2(objectPoseRobotFrame.y,objectPoseRobotFrame.x);
		
		mutexScan.lock();
		
		int index = (int) ((((objectPoseRobotFrame.z - minScanAngle) / (maxScanAngle - minScanAngle)) * (maxScanAngle - minScanAngle)) / scanAngleIncrement);
		
		if ((index < 0) || (index >= NUMBER_OF_LASER_SCANS))
		{
			mutexScan.unlock();
			
			return false;
		}
		
		minIntervalRange = index - range;
		
		if (minIntervalRange < 0) minIntervalRange = 0;
		
		maxIntervalRange = index + range;
		
		if (maxIntervalRange >= NUMBER_OF_LASER_SCANS) maxIntervalRange = NUMBER_OF_LASER_SCANS - 1;
		
		Vector3 objectPose;
		
		objectPose.x = objectPoseGlobalFrame.x;
		objectPose.y = objectPoseGlobalFrame.y;
		
		Vector3 scanPose;
		double theta;
		
		for (int i = minIntervalRange; i <= maxIntervalRange; ++i)
		{
			theta = minScanAngle + (scanAngleIncrement * i);
			
			float rho = sqrt((scanRangeData.at(i) * cos(theta) * scanRangeData.at(i) * cos(theta)) +
							 (scanRangeData.at(i) * sin(theta) * scanRangeData.at(i) * sin(theta)));
			
			scanPose.x = robotPose.x + (rho * cos(robotPose.z + atan2(scanRangeData.at(i) * sin(theta),scanRangeData.at(i) * cos(theta))));
			scanPose.y = robotPose.y + (rho * sin(robotPose.z + atan2(scanRangeData.at(i) * sin(theta),scanRangeData.at(i) * cos(theta))));
			
			if ((fabs(scanPose.x - objectPose.x) <= robotDiameter) && (fabs(scanPose.y - objectPose.y) <= robotDiameter))
			{
				mutexScan.unlock();
				
				return true;
			}
		}
		
		mutexScan.unlock();
		
		return false;
	}
	
	void LaserScanDetector::exec()
	{
		ObjectDetection objectDetected;
		vector<Object> objects;
		Vector3 objectPoseRobotFrame;
		stringstream s;
		float cosTheta, dx, dy, sinTheta;
		
		s << "/robot_" << agentId << "/odom";
		
		mutex.lock();
		
		const map<string,Vector3>::const_iterator& robotPose = robotPoses.find(s.str());
		
		/// Something nasty has just happened.
		if (robotPose == robotPoses.end())
		{
			mutex.unlock();
			
			return;
		}
		
		for (map<string,Vector3>::const_iterator it = robotPoses.begin(); it != robotPoses.end(); ++it)
		{
			if (it->first == s.str()) continue;
			
			if (checkObjectDetection(robotPose->second,it->second))
			{
				dx = it->second.x - robotPose->second.x;
				dy = it->second.y - robotPose->second.y;
				
				cosTheta = cos(robotPose->second.z);
				sinTheta = sin(robotPose->second.z);
				
				// Target position w.r.t. the mobile frame (robot frame).
				objectPoseRobotFrame.x =  cosTheta * dx + sinTheta * dy;
				objectPoseRobotFrame.y = -sinTheta * dx + cosTheta * dy;
				
				Object object;
				
				object.x = objectPoseRobotFrame.x;
				object.y = objectPoseRobotFrame.y;
				
				objects.push_back(object);
			}
		}
		
		mutex.unlock();
		
		objectDetected.objects = objects;
		
		publisherObjectDetected.publish(objectDetected);
	}
	
	void LaserScanDetector::updateLaserScan(const LaserScan::ConstPtr& message)
	{
		mutexScan.lock();
		
		scanRangeData.clear();
		
		maxScanAngle = message->angle_max;
		minScanAngle = message->angle_min;
		scanAngleIncrement = message->angle_increment;
		copy(&message->ranges[0],&message->ranges[NUMBER_OF_LASER_SCANS],back_inserter(scanRangeData));
		
		mutexScan.unlock();
	}
	
	void LaserScanDetector::updateRobotPose(const Odometry::ConstPtr& message)
	{
		Vector3 currentRobotPose;
		double roll, pitch, yaw;
		
		const Quaternion& q = message->pose.pose.orientation;
		
		tf::Matrix3x3(tf::Quaternion(q.x,q.y,q.z,q.w)).getRPY(roll,pitch,yaw);
		
		mutex.lock();
		
		currentRobotPose.x = message->pose.pose.position.x;
		currentRobotPose.y = message->pose.pose.position.y;
		currentRobotPose.z = yaw;
		
		const map<string,Vector3>::iterator& robotPose = robotPoses.find(message->header.frame_id);
		
		if (robotPose == robotPoses.end()) robotPoses.insert(make_pair(message->header.frame_id,currentRobotPose));
		else robotPose->second = currentRobotPose;
		
		mutex.unlock();
	}
}
