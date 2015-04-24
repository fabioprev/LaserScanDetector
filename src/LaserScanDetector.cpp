#include "LaserScanDetector.h"
#include "Utils/Timestamp.h"
#include <LaserScanDetector/Object.h>
#include <LaserScanDetector/ObjectDetection.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <sys/stat.h>

using namespace std;
using namespace PTracking;
using LaserScanDetector::ObjectDetection;
using LaserScanDetector::Object;
using geometry_msgs::Quaternion;
using geometry_msgs::PoseWithCovarianceStamped;
using geometry_msgs::Vector3;
using nav_msgs::Odometry;
using sensor_msgs::LaserScan;

#define ERR(x)   cerr << "\033[22;31;1m" << x << "\033[0m";
#define WARN(x)  cerr << "\033[22;33;1m" << x << "\033[0m";
#define INFO(x)  cerr << "\033[22;37;1m" << x << "\033[0m";
#define DEBUG(x) cerr << "\033[22;34;1m" << x << "\033[0m";

namespace Data
{
	LaserScanDetector::LaserScanDetector() : nodeHandle("~")
	{
		string groundtruthDir;
		int numberOfRobots;
		
		nodeHandle.getParam("agentId",agentId);
		nodeHandle.getParam("groundtruthDir",groundtruthDir);
		nodeHandle.getParam("maxReading",maxReading);
		nodeHandle.getParam("numberOfRobots",numberOfRobots);
		nodeHandle.getParam("startingPreyRobot",startingPreyRobot);
		nodeHandle.getParam("endingPreyRobot",endingPreyRobot);
		
		/// I am using the base_pose_ground_truth because this node is gonna be used only in Stage.
		for (int i = startingPreyRobot; i <= endingPreyRobot; ++i)
		{
			stringstream s;
			
			s << "/robot" << i << "/base_pose_ground_truth";
			
			subscriberRobotPoses.push_back(nodeHandle.subscribe(s.str(),1024,&LaserScanDetector::updatePreyRobotPose,this));
		}
		
		/// Subscribing myself for getting the robot pose.
		if ((agentId < startingPreyRobot) || (agentId > endingPreyRobot))
		{
			stringstream s;
			
			s << "/robot" << agentId << "/amcl_pose";
			
			subscriberRobotPoses.push_back(nodeHandle.subscribe(s.str(),1024,&LaserScanDetector::updateRobotPose,this));
		}
		
		subscriberLaserScan = nodeHandle.subscribe("scan",1024,&LaserScanDetector::updateLaserScan,this);
		
		publisherObjectDetected = nodeHandle.advertise<ObjectDetection>("objectDetected",1);
		
		if (agentId == 0)
		{
			struct stat temp;
			
			if (stat(groundtruthDir.c_str(),&temp) == -1)
			{
				mkdir(groundtruthDir.c_str(),0775);
			}
			
			stringstream s;
			
			s << groundtruthDir.c_str() << "/groundtruth_" << Timestamp().getStringRepresentation() << ".xml";
			
			groundtruth.open(s.str().c_str());
			
			groundtruth << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << endl;
			groundtruth << "<dataset>" << endl;
		}
	}
	
	LaserScanDetector::~LaserScanDetector()
	{
		if (agentId == 0)
		{
			groundtruth << "</dataset>";
			
			groundtruth.close();
		}
	}
	
	bool LaserScanDetector::checkObjectDetection(const Vector3& robotPose, const Vector3& objectPoseGlobalFrame)
	{
		static Vector3 objectPoseRobotFrame;
		static float cosTheta, dx, dy, robotDiameter = 0.3, sinTheta;
		static int range = 15;
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
		static unsigned long counter = 0;
		
		ObjectDetection objectDetected;
		vector<Object> objects;
		Vector3 objectPoseRobotFrame;
		stringstream s;
		float cosTheta, dx, dy, sinTheta;
		
		s << "/robot" << agentId << "/odom";
		
		mutex.lock();
		
		const map<string,Vector3>::const_iterator& robotPose = robotPoses.find(s.str());
		
		/// Robot pose not available yet.
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
				
				int i, id;
				
				const string& temp = it->first.substr(1,it->first.find("/",1,1) - 1);
				
				for (i = (temp.size() - 1); i >= 0 ; --i)
				{
					if (!isdigit(temp[i])) break;
				}
				
				id = atoi(temp.substr(i + 1,temp.size()).c_str());
				
				Object object;
				
				object.id = id;
				object.x = objectPoseRobotFrame.x;
				object.y = objectPoseRobotFrame.y;
				
				objects.push_back(object);
			}
		}
		
		if (agentId == 0) writeGroundtruth();
		
		mutex.unlock();
		
		if (objects.size() > 0)
		{
			objectDetected.id = counter++;
			objectDetected.objects = objects;
			
			publisherObjectDetected.publish(objectDetected);
		}
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
	
	void LaserScanDetector::updatePreyRobotPose(const Odometry::ConstPtr& message)
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
	
	void LaserScanDetector::updateRobotPose(const PoseWithCovarianceStamped::ConstPtr& message)
	{
		Vector3 currentRobotPose;
		stringstream s;
		double roll, pitch, yaw;
		
		const Quaternion& q = message->pose.pose.orientation;
		
		tf::Matrix3x3(tf::Quaternion(q.x,q.y,q.z,q.w)).getRPY(roll,pitch,yaw);
		
		mutex.lock();
		
		currentRobotPose.x = message->pose.pose.position.x;
		currentRobotPose.y = message->pose.pose.position.y;
		currentRobotPose.z = yaw;
		
		s << "/robot" << agentId << "/odom";
		
		const map<string,Vector3>::iterator& robotPose = robotPoses.find(s.str());
		
		if (robotPose == robotPoses.end()) robotPoses.insert(make_pair(s.str(),currentRobotPose));
		else robotPose->second = currentRobotPose;
		
		mutex.unlock();
	}
	
	void LaserScanDetector::writeGroundtruth()
	{
		static unsigned long groundtruthIterations = 0;
		
		groundtruth << "   <frame number=\"" << groundtruthIterations++ << "\">" << endl;
		groundtruth << "      <objectlist>" << endl;
		
		for (map<string,Vector3>::iterator it = robotPoses.begin(); it != robotPoses.end(); ++it)
		{
			int i, id;
			
			const string& temp = it->first.substr(1,it->first.find("/",1,1) - 1);
			
			for (i = (temp.size() - 1); i >= 0 ; --i)
			{
				if (!isdigit(temp[i])) break;
			}
			
			id = atoi(temp.substr(i + 1,temp.size()).c_str());
			
			groundtruth << "         <object id=\"" << id << "\">" << endl;
			groundtruth << "            <box h=\"0\" w=\"0\" xc=\"" << it->second.x << "\""
						<< " yc=\"" << it->second.y << "\""
						<< " hxc=\"" << it->second.x << "\""
						<< " hyc=\"" << it->second.y << "\""
						<< " b=\"" << it->second.x << "\""
						<< "/>" << endl;
			groundtruth << "         </object>" << endl;
		}
		
		groundtruth << "      </objectlist>" << endl;
	}
}
