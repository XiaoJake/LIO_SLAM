/*
 * @Description: 订阅velocity数据
 * @Author: Zhang Jun
 * @Date: 2019-08-19 19:22:17
 */
#ifndef LIO_SLAM_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_
#define LIO_SLAM_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"

#include "lio_slam/sensor_data/velocity_data.hpp"

namespace lio_slam {
class VelocitySubscriber {
  public:
    VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    VelocitySubscriber() = default;
    void ParseData(std::deque<VelocityData>& deque_velocity_data);

  private:
    void msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<VelocityData> new_velocity_data_;

    std::mutex buff_mutex_; 
};
}
#endif