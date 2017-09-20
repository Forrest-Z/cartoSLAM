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
#include "../mapping/local_trajectory_builder_options.h"
#include "../mapping/local_trajectory_builder.h"

void publisher(double map_pub_period)
{
	ros::Rate r(1.0 / map_pub_period);
	while (ros::ok())
	{
		//ros::Time mapTime(ros::Time::now());
		//publishMap(mapPubContainer[0], slamProcessor->getGridMap(0), mapTime, slamProcessor->getMapMutex(0));
		std::cout << "in thread!" << std::endl;
		r.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hector_slam");
	ros::start();
	BagReader bagReader("/home/liu/tokyo_bag/lg_1.bag", "/scan", "/odom", 0, 3000);
	auto pairData = bagReader.mPairData;
	auto p_local_trajectory_builder = ::cartographer::common::make_unique<::cartographer::mapping::LocalTrajectoryBuilder>(::cartographer::mapping::CreateLocalTrajectoryBuilderOptions());
	::cartographer_ros::SensorBridge sensor_bridge(std::move(p_local_trajectory_builder));
	//auto map_publish_thread_ = new std::thread(&publisher, 0.5);
	//map_publish_thread_->depatch();
	new std::thread(&publisher, 0.5);

	for (auto a : pairData)
	{
		if (!ros::ok())
			break;
		auto podom = boost::make_shared<const ::nav_msgs::Odometry>(a.second);
		auto pscan = boost::make_shared<const ::sensor_msgs::LaserScan>(a.first);

		//sensor_bridge.HandleOdometryMessage(podom);
		sensor_bridge.HandleLaserScanMessage(pscan);
	}
	//map_publish_thread_.join();
	return 0;
}
