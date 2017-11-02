#include <ros/ros.h>
#include <chrono>
#include <thread>
#include <memory>

#include "BagReader.h"
#include "src/common/lua_parameter_dictionary.h"
#include "src/common/configuration_file_resolver.h"
#include "src/common/make_unique.h"
#include "src/mapping/global_trajectory_builder.h"
#include "src/top/sensor_bridge.h"
#include "src/mapping/local_trajectory_builder_options.h"
#include "src/mapping/submaps.h"

class Publisher
{
  public:
	explicit Publisher(std::shared_ptr<::cartographer::mapping::GlobalTrajectoryBuilder>
						   p_global_trajectory_builder,
					   double map_pub_period) : p_global_trajectory_builder_(p_global_trajectory_builder), map_pub_period_(map_pub_period)
	{
		mapPublisher_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
	};

	void pcd()
	{
		ros::Rate r(1.0 / map_pub_period_);
		::ros::NodeHandle node_handle;
		::ros::Publisher pcd_publisher;
		pcd_publisher = node_handle.advertise<sensor_msgs::PointCloud2>("carto_pcd", 1);

		while (ros::ok())
		{

			const auto all_submap_data = p_global_trajectory_builder_->GetAllSubmapData();

			if (!(all_submap_data.size() == 1 && all_submap_data[0].size() > 0))
				continue;

			//std::cout<<grid.limits().max().x()<<","<<grid.limits().max().y()<<std::endl;
			//std::cout<<offset.x() <<","<<offset.y()<<std::endl;
				
			std::vector<int8_t> &data = map_.map.data;

			map_.map.info.origin.position.x = 0;
			map_.map.info.origin.position.y = 0;
		  	map_.map.info.origin.orientation.w = 1;
			map_.map.info.resolution = 0.05;
			map_.map.info.width = 1000;
			map_.map.info.height = 1000;
			//map_.map.info.width = limits.num_x_cells;
			//map_.map.info.height = limits.num_y_cells;

			map_.map.header.frame_id = "map";
			map_.map.data.resize(map_.map.info.width * map_.map.info.height);
			map_.map.header.stamp = ros::Time::now();
			memset(&map_.map.data[0], 0, sizeof(int8_t) * map_.map.data.size());
			for (::cartographer::mapping::SparsePoseGraph::SubmapData m : all_submap_data[0])
			{
				Eigen::Array2i offset;
				::cartographer::mapping::CellLimits limits;
				auto grid = m.submap->probability_grid();
				grid.ComputeCroppedLimits(&offset, &limits);

				//std::cout<<m.submap->local_pose().translation().x()<<","<<m.submap->local_pose().translation().y()<<std::endl;

				for (const Eigen::Array2i &xy_index : ::cartographer::mapping::XYIndexRangeIterator(limits))
				{

					if (grid.IsKnown(xy_index + offset))
					{
						double x = grid.limits().max()[1] - (offset.x() + xy_index.x() + 0.5) * grid.limits().resolution();
						double y = grid.limits().max()[0] - (offset.y() + xy_index.y() + 0.5) * grid.limits().resolution();

						
						int x_map = x / grid.limits().resolution() + map_.map.info.width / 2;
						int y_map = y / grid.limits().resolution() + map_.map.info.height /2;

						const double p = grid.GetProbability(xy_index + offset);
						if(data[y_map * map_.map.info.width + x_map] == 0){
							int map_state = 0;
							if (p > 0.51)
								map_state = p * 100;
							else if (p < 0.49)
								map_state = -(1-p) * 100;
							data[y_map * map_.map.info.width + x_map] = map_state;
						}
					}
				}
			}
			mapPublisher_.publish(map_.map);
			ros::spinOnce();
			r.sleep();
		}
	}

  private:
	std::shared_ptr<::cartographer::mapping::GlobalTrajectoryBuilder> p_global_trajectory_builder_;
	double map_pub_period_;
	nav_msgs::GetMap::Response map_;
	ros::Publisher mapPublisher_;
	ros::NodeHandle node_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hector_slam");
	ros::start();
	BagReader bagReader("/home/liu/workspace/cartoSLAM/lg_kusatsu_C5_1.bag", "/scan", "/odom", 0, 3000);

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
	

	Publisher pub = Publisher(golbal_trajectory_builder_ptr, 0.5);
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
