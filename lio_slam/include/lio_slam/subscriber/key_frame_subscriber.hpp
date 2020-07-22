/*
 * @Description: 订阅 key frame 数据
 * @Author: Zhang Jun
 * @Date: 2019-08-19 19:22:17
 */
#ifndef LIO_SLAM_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_HPP_
#define LIO_SLAM_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "lio_slam/sensor_data/key_frame.hpp"

namespace lio_slam {
class KeyFrameSubscriber {
  public:
    KeyFrameSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    KeyFrameSubscriber() = default;
    void ParseData(std::deque<KeyFrame>& key_frame_buff);

  private:
    void msg_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& key_frame_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<KeyFrame> new_key_frame_;

    std::mutex buff_mutex_; 
};
}
#endif