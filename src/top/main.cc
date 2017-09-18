#include <ros/ros.h>
#include <chrono>
#include <thread>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include "laser_geometry/laser_geometry.h"

#include "BagReader.h"

#include "../sensor/odometry_data.h"
#include "sensor_bridge.h"
#include "msg_conversion.h"



#include "../mapping/global_trajectory_builder.h"
#include "../mapping/probability_grid.h"




/*
class SampleCartoNode
{
public:
	SampleCartoNode()
	{
		ros::NodeHandle nh;
		message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 10);
		message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "scan", 10);
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> SyncPolicy;
		message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(2000), scan_sub, odom_sub);
		sync.registerCallback(boost::bind(&SampleCartoNode::callback, this, _1, _2));
		ros::spin();
	}

private:
	void callback(const sensor_msgs::LaserScanConstPtr &scan_msg, const nav_msgs::OdometryConstPtr &odom_msg)
	{
		std::cout<<"callback!"<<std::endl;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "carto_slam");
	SampleCartoNode scn;
}
*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hector_slam");
	BagReader bagReader("/home/liu/h1.bag","scan","odom",0, 3000);
	auto pairData = bagReader.mPairData;
	::cartographer_ros::SensorBridge sensor_bridge;
	//::cartographer::mapping::GlobalTrajectoryBuilder gtb;
	//int c = 0;
	for (auto a : pairData)
	{
		
		auto podom = boost::make_shared<const ::nav_msgs::Odometry>(a.second);
		auto pscan = boost::make_shared<const ::sensor_msgs::LaserScan>(a.first);
		
		//const sensor_msgs::LaserScan_<std::allocator<void> >&}’ from expression of type ‘boost::shared_ptr<const sensor_msgs::LaserScan_<std::allocator<void> > >’

		
		sensor_bridge.HandleOdometryMessage(podom);
		sensor_bridge.HandleLaserScanMessage(pscan);
		//auto scan_data = ::cartographer_ros::ToPointCloudWithIntensities(a.first);

		//for(auto m:scan_data.points){
		//	std::cout<<m<<std::endl;
		//}
		//std::cout<<scan_data.points<<std::endl;
	}
	return 0;
}
