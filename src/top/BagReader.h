#pragma once

#include <deque>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>


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

      rosbag::View view1(bag, rosbag::TopicQuery(mScanTopic));
      for (rosbag::MessageInstance const m : view1)
      {
					sensor_msgs::LaserScan::ConstPtr pmsg = m.instantiate<sensor_msgs::LaserScan>();
					scan_raw_datas_.push_back(*pmsg);
      }	
      rosbag::View view2(bag, rosbag::TopicQuery(mOdomTopic));
      for (rosbag::MessageInstance const m : view2)
      {
					nav_msgs::Odometry::ConstPtr pmsg = m.instantiate<nav_msgs::Odometry>();
					odom_raw_datas_.push_back(*pmsg);
      }
			//sync();
			bag.close();
    }

    void sync()
    {
      unsigned int idx = 0;
      ros::Time start_time = scan_raw_datas_[0].header.stamp + ros::Duration(mStartTime, 0);
			ros::Time end_time = scan_raw_datas_[0].header.stamp + ros::Duration(mEndTime, 0);
      for (sensor_msgs::LaserScan scan_msg : scan_raw_datas_)
      {
				if(idx >= odom_raw_datas_.size())
					break;
				ros::Time time_stamp_scan = scan_msg.header.stamp;
				if(time_stamp_scan >= end_time)
					break;
				if(time_stamp_scan < start_time)
	  			continue;
				nav_msgs::Odometry odom_msg = odom_raw_datas_[idx];
				while (idx < odom_raw_datas_.size())
				{
				  ros::Time time_stamp_odom = odom_msg.header.stamp;
				  if (time_stamp_odom > time_stamp_scan)
				    break;
				  odom_msg = odom_raw_datas_[idx];
				  idx += 1;
				}
				std::pair<sensor_msgs::LaserScan, nav_msgs::Odometry> p(scan_msg, odom_msg);
				mPairData_.push_back(p);
      }
    }

		syncdatas mPairData_;
		std::deque<sensor_msgs::LaserScan> scan_raw_datas_;
		std::deque<nav_msgs::Odometry> odom_raw_datas_;


  private:
    std::string mBagfileName;
    std::string mScanTopic;
    std::string mOdomTopic;
		std::string mCloudTopic;
		float mStartTime;
		float mEndTime;

};
