/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Author: Zhang Jun
 * @Date: 2020-07-15 02:27:30
 */

#ifndef LIO_SLAM_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_
#define LIO_SLAM_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lio_slam/sensor_data/cloud_data.hpp"

namespace lio_slam {
class CloudSubscriber {
  public:
    CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    CloudSubscriber() = default;
    void ParseData(std::deque<CloudData>& deque_cloud_data);
    template <typename PointT>
    void limitDistancePointCloud(const pcl::PointCloud<PointT> &cloud_in,pcl::PointCloud<PointT> &cloud_out, float head,float end);

  private:
    void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<CloudData> new_cloud_data_;

    std::mutex buff_mutex_;
};
}

#endif