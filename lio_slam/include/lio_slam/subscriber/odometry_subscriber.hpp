/*
 * @Description: 订阅odometry数据
 * @Author: Zhang Jun
 * @Date: 2020-08-1 19:22:17
 */
#ifndef LIO_SLAM_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_
#define LIO_SLAM_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "lio_slam/sensor_data/pose_data.hpp"

namespace lio_slam {
class OdometrySubscriber {
  public:
    OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size,char mode);
    OdometrySubscriber() = default;
    void ParseData(std::deque<PoseData>& deque_pose_data);

  private:
    void msg1_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);
    void msg2_callback(const geometry_msgs::PoseWithCovarianceStampedPtr& odom_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<PoseData> new_pose_data_;

    std::mutex buff_mutex_; 
};
}
#endif