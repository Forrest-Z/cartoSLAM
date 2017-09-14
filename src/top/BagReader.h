#pragma once

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "sensor_msgs/PointCloud2.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class BagReader
{
	typedef std::vector<std::pair<sensor_msgs::LaserScan, nav_msgs::Odometry>> syncdatas;

  public:
    BagReader(std::string bagfile, std::string scan_topic, std::string odom_topic,float start, float end)
		:mBagfileName(bagfile)
		,mScanTopic(scan_topic)
		,mOdomTopic(odom_topic)
		,mStartTime(start)
		,mEndTime(end)
		{
			read();
		}

    void read()
    {
			rosbag::Bag bag;
      bag.open(mBagfileName, rosbag::bagmode::Read);

			std::vector<sensor_msgs::LaserScan> scan_raw_datas;
			std::vector<nav_msgs::Odometry> odom_raw_datas;
      rosbag::View view1(bag, rosbag::TopicQuery(mScanTopic));
      for (rosbag::MessageInstance const m : view1)
      {
					sensor_msgs::LaserScan::ConstPtr pmsg = m.instantiate<sensor_msgs::LaserScan>();
					scan_raw_datas.push_back(*pmsg);
      }	
      rosbag::View view2(bag, rosbag::TopicQuery(mOdomTopic));
      for (rosbag::MessageInstance const m : view2)
      {
					nav_msgs::Odometry::ConstPtr pmsg = m.instantiate<nav_msgs::Odometry>();
					odom_raw_datas.push_back(*pmsg);
      }
			sync(scan_raw_datas, odom_raw_datas);
			bag.close();
    }

    void sync(const std::vector<sensor_msgs::LaserScan> &scan_raw_datas, const std::vector<nav_msgs::Odometry> odom_raw_datas)
    {
      unsigned int idx = 0;
      ros::Time start_time = scan_raw_datas[0].header.stamp + ros::Duration(mStartTime, 0);
			ros::Time end_time = scan_raw_datas[0].header.stamp + ros::Duration(mEndTime, 0);
      for (sensor_msgs::LaserScan scan_msg : scan_raw_datas)
      {
				if(idx >= odom_raw_datas.size())
					break;
				ros::Time time_stamp_scan = scan_msg.header.stamp;
				if(time_stamp_scan >= end_time)
					break;
				if(time_stamp_scan < start_time)
	  			continue;
				nav_msgs::Odometry odom_msg = odom_raw_datas[idx];
				while (idx < odom_raw_datas.size())
				{
				  ros::Time time_stamp_odom = odom_msg.header.stamp;
				  if (time_stamp_odom > time_stamp_scan)
				    break;
				  odom_msg = odom_raw_datas[idx];
				  idx += 1;
				}
				std::pair<sensor_msgs::LaserScan, nav_msgs::Odometry> p(scan_msg, odom_msg);
				mPairData.push_back(p);
      }
    }

		syncdatas mPairData;

  private:
    std::string mBagfileName;
    std::string mScanTopic;
    std::string mOdomTopic;
		std::string mCloudTopic;
		float mStartTime;
		float mEndTime;

};


class BagReaderWithPointCloud
{
	typedef std::vector<std::pair<sensor_msgs::PointCloud2, nav_msgs::Odometry>> syncdatas;

  public:
    BagReaderWithPointCloud(std::string bagfile, std::string cloud_topic, std::string odom_topic,float start, float end)
		:mBagfileName(bagfile)
		,mCloudTopic(cloud_topic)
		,mOdomTopic(odom_topic)
		,mStartTime(start)
		,mEndTime(end)
		{
			read();
		}

    void read()
    {
			rosbag::Bag bag;
      bag.open(mBagfileName, rosbag::bagmode::Read);

			std::vector<nav_msgs::Odometry> odom_raw_datas;		
      rosbag::View view2(bag, rosbag::TopicQuery(mOdomTopic));
      for (rosbag::MessageInstance const m : view2)
      {
					nav_msgs::Odometry::ConstPtr pmsg = m.instantiate<nav_msgs::Odometry>();
					odom_raw_datas.push_back(*pmsg);
      }

			std::vector<sensor_msgs::PointCloud2> cloud_raw_datas;
			rosbag::View view3(bag, rosbag::TopicQuery(mCloudTopic));
			for (rosbag::MessageInstance const m : view3)
      {
					sensor_msgs::PointCloud2ConstPtr pmsg = m.instantiate<sensor_msgs::PointCloud2>();
					cloud_raw_datas.push_back(*pmsg);
      }
			sync(cloud_raw_datas, odom_raw_datas);
			bag.close();
    }

    void sync(const std::vector<sensor_msgs::PointCloud2> &cloud_raw_datas, const std::vector<nav_msgs::Odometry> odom_raw_datas)
    {
      unsigned int idx = 0;
      ros::Time start_time = cloud_raw_datas[0].header.stamp + ros::Duration(mStartTime, 0);
			ros::Time end_time = cloud_raw_datas[0].header.stamp + ros::Duration(mEndTime, 0);
      for (sensor_msgs::PointCloud2 cloud_msg : cloud_raw_datas)
      {
				if(idx >= odom_raw_datas.size())
					break;
				ros::Time time_stamp_cloud = cloud_msg.header.stamp;
				if(time_stamp_cloud >= end_time)
					break;
				if(time_stamp_cloud < start_time)
	  			continue;
				while (idx < odom_raw_datas.size())
				{
				  ros::Time time_stamp_odom = odom_raw_datas[idx].header.stamp;
				  if (time_stamp_odom > time_stamp_cloud)
				    break;
				  
				  idx += 1;
				}
				float time_a = odom_raw_datas[idx].header.stamp.toSec();
				float time_b = odom_raw_datas[idx - 1].header.stamp.toSec();
				float time_o = time_stamp_cloud.toSec();
				nav_msgs::Odometry odom_msg = fabs(time_o - time_a) > fabs(time_o - time_b) ? odom_raw_datas[idx - 1] : odom_raw_datas[idx];
				std::pair<sensor_msgs::PointCloud2, nav_msgs::Odometry> p(cloud_msg, odom_msg);
				mPairData.push_back(p);
      }
    }



		syncdatas mPairData;

  private:
    std::string mBagfileName;
		std::string mCloudTopic;
    std::string mOdomTopic;
		float mStartTime;
		float mEndTime;

};
