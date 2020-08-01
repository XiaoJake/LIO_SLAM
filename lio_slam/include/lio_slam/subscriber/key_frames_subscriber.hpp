/*
 * @Description: 订阅 key frame 数据
 * @Author: Zhang Jun
 * @Date: 2020-08-1 19:22:17
 */
#ifndef LIO_SLAM_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_HPP_
#define LIO_SLAM_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "lio_slam/sensor_data/key_frame.hpp"

namespace lio_slam {
class KeyFramesSubscriber {
  public:
    KeyFramesSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    KeyFramesSubscriber() = default;
    void ParseData(std::deque<KeyFrame>& deque_key_frames);

  private:
    void msg_callback(const nav_msgs::Path::ConstPtr& key_frames_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<KeyFrame> new_key_frames_;

    std::mutex buff_mutex_; 
};
}
#endif