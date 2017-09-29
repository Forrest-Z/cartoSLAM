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
#include "../mapping/global_trajectory_builder.h"
#include "SubmapList.h"

#include "nav_msgs/GetMap.h"

#include "../mapping/sparse_pose_graph.h"

class Publisher
{
  public:
	explicit Publisher(std::shared_ptr<::cartographer::mapping::GlobalTrajectoryBuilder<
						   ::cartographer::mapping::LocalTrajectoryBuilder,
						   ::cartographer::mapping::proto::LocalTrajectoryBuilderOptions,
						   ::cartographer::mapping::SparsePoseGraph>>
						   p_global_trajectory_builder,
					   double map_pub_period) : p_global_trajectory_builder_(p_global_trajectory_builder), map_pub_period_(map_pub_period){
						//map_.map.data.resize(4000*4000);
						//memset(&map_.map.data[0], 180, sizeof(int8_t) * map_.map.data.size());
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
			carto_slam::SubmapList submap_list;
			const auto all_submap_data = p_global_trajectory_builder_->sparse_pose_graph_->GetAllSubmapData();
			/*

  cartographer_ros_msgs::SubmapList submap_list;
  submap_list.header.stamp = ::ros::Time::now();
  submap_list.header.frame_id = node_options_.map_frame;
  const auto all_submap_data =
      map_builder_.sparse_pose_graph()->GetAllSubmapData();
  for (size_t trajectory_id = 0; trajectory_id < all_submap_data.size();
       ++trajectory_id) {
    for (size_t submap_index = 0;
         submap_index < all_submap_data[trajectory_id].size(); ++submap_index) {
      const auto& submap_data = all_submap_data[trajectory_id][submap_index];
      if (submap_data.submap == nullptr) {
        continue;
      }
      cartographer_ros_msgs::SubmapEntry submap_entry;
      submap_entry.trajectory_id = trajectory_id;
      submap_entry.submap_index = submap_index;
      submap_entry.submap_version = submap_data.submap->num_range_data();
      submap_entry.pose = ToGeometryMsgPose(submap_data.pose);
      submap_list.submap.push_back(submap_entry);
    }
  }
  return submap_list;
  
						*/
			if (!(all_submap_data.size() == 2 && all_submap_data[1].size() > 0))
				continue;
			auto grid = all_submap_data[1][all_submap_data[1].size() -1].submap->probability_grid();
			std::vector<int8_t> &data = map_.map.data;

			map_.map.info.origin.position.x = 0;
			map_.map.info.origin.position.y = 0;
			map_.map.info.origin.orientation.w = 1.0;

			map_.map.info.resolution = grid.limits().resolution();

			map_.map.info.width = grid.limits().cell_limits().num_x_cells;
			map_.map.info.height = grid.limits().cell_limits().num_y_cells;

			map_.map.header.frame_id = "map";
			map_.map.data.resize(map_.map.info.width * map_.map.info.height);
			memset(&map_.map.data[0], 100, sizeof(int8_t) * map_.map.data.size());
			map_.map.header.stamp = ros::Time::now();
			for (size_t y = 0; y < grid.limits().cell_limits().num_y_cells; ++y)
			{
				for (size_t x = 0; x < grid.limits().cell_limits().num_x_cells; ++x)
				{
					data[y * grid.limits().cell_limits().num_y_cells + x] = 
					grid.GetProbability(Eigen::Array2i(x, y)) * 100;
					
				}
			}
			mapPublisher_.publish(map_.map);
			ros::spinOnce();
			r.sleep();
		}
	}

  private:
	std::shared_ptr<::cartographer::mapping::GlobalTrajectoryBuilder<
		::cartographer::mapping::LocalTrajectoryBuilder,
		::cartographer::mapping::proto::LocalTrajectoryBuilderOptions,
		::cartographer::mapping::SparsePoseGraph>>
		p_global_trajectory_builder_;
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
	auto pairData = bagReader.mPairData;

	//new std::thread(&publisher, 0.5);

	auto p_golbal_trajectory_builder = std::make_shared<::cartographer::mapping::GlobalTrajectoryBuilder<
		::cartographer::mapping::LocalTrajectoryBuilder,
		::cartographer::mapping::proto::LocalTrajectoryBuilderOptions,
		::cartographer::mapping::SparsePoseGraph>>(
		::cartographer::mapping::CreateLocalTrajectoryBuilderOptions(),
		1, new ::cartographer::mapping::SparsePoseGraph());

	Publisher pub = Publisher(p_golbal_trajectory_builder, 0.5);
	::cartographer_ros::SensorBridge sensor_bridge(p_golbal_trajectory_builder);

	new std::thread(&Publisher::pcd, &pub);

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
