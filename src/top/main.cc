#include <ros/ros.h>
#include <chrono>
#include <thread>
#include <memory>

#include "src/common/lua_parameter_dictionary.h"
#include "src/common/configuration_file_resolver.h"
#include "src/common/make_unique.h"
#include "src/mapping/global_trajectory_builder.h"
#include "src/top/sensor_bridge.h"
#include "src/mapping/local_trajectory_builder_options.h"
#include "src/mapping/submaps.h"

#include "MapPublisher.h"
#include "BagReader.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hector_slam");
	ros::start();
	//BagReader bagReader("/home/liu/workspace/cartoSLAM/lg_kusatsu_C5_1.bag", "/scan", "/odom", 0, 3000);
	//BagReader bagReader("/home/liu/2017-10-06-19-19-10.bag", "/scan", "/odom", 0, 3000);
	BagReader bagReader("/home/liu/tokyo_bag/lg_1.bag", "/scan", "/odom", 30, 3000);

	auto file_resolver = cartographer::common::make_unique<cartographer::common::ConfigurationFileResolver>(std::vector<string>{std::string("/home/liu/catkin_ws_carto/src/cartographer_ros/cartographer_ros/configuration_files")});
	const string code =file_resolver->GetFileContentOrDie("backpack_2d.lua");
	cartographer::common::LuaParameterDictionary parameter_dictionary(code, std::move(file_resolver));
	auto srt = parameter_dictionary.GetString("tracking_frame");
	//auto sub = parameter_dictionary.GetDictionary("trajectory_builder").get();
	//auto sub2= sub->GetDictionary("trajectory_builder_2d").get();
	auto local_trajectory_builder_options = ::cartographer::mapping::CreateLocalTrajectoryBuilderOptions(parameter_dictionary.GetDictionary("trajectory_builder").get()->GetDictionary("trajectory_builder_2d").get());
	auto sparse_pose_graph_options        = ::cartographer::mapping::CreateSparsePoseGraphOptions(parameter_dictionary.GetDictionary("map_builder").get()->GetDictionary("sparse_pose_graph").get());
	auto golbal_trajectory_builder_ptr = std::make_shared<::cartographer::mapping::GlobalTrajectoryBuilder>(local_trajectory_builder_options, sparse_pose_graph_options);
	::cartographer_ros::SensorBridge sensor_bridge(golbal_trajectory_builder_ptr);
	

	Publisher pub = Publisher(golbal_trajectory_builder_ptr, 0.3);
	new std::thread(&Publisher::pcd, &pub);

	//::cartographer::mapping::ProbabilityGrid probability_grid(::cartographer::mapping::MapLimits(1., Eigen::Vector2d(1., 2.), ::cartographer::mapping::CellLimits(2, 2)));
	while (ros::ok())
	{
		auto scan = bagReader.scan_raw_datas_.front();
		auto odom = bagReader.odom_raw_datas_.front();
		ros::Time time_stamp_scan = scan.header.stamp;
		ros::Time time_stamp_odom = odom.header.stamp;
		if (scan.header.stamp < odom.header.stamp)
		{
			auto scan_ptr = boost::make_shared<const ::sensor_msgs::LaserScan>(scan);
			sensor_bridge.HandleLaserScanMessage(scan_ptr);
			bagReader.scan_raw_datas_.pop_front();
		}
		else
		{
			auto odom_ptr = boost::make_shared<const ::nav_msgs::Odometry>(odom);
			sensor_bridge.HandleOdometryMessage(odom_ptr);
			bagReader.odom_raw_datas_.pop_front();
		}
	}

	return 0;
}
