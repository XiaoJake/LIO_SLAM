/*
 * @Description: 订阅 闭环检测位姿 数据
 * @Author: Zhang Jun
 * @Date: 2019-08-19 19:22:17
 */
#ifndef LIO_SLAM_SUBSCRIBER_LOOP_POSE_SUBSCRIBER_HPP_
#define LIO_SLAM_SUBSCRIBER_LOOP_POSE_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "lio_slam/sensor_data/loop_pose.hpp"

namespace lio_slam {
class LoopPoseSubscriber {
  public:
    LoopPoseSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    LoopPoseSubscriber() = default;
    void ParseData(std::deque<LoopPose>& loop_pose_buff);

  private:
    void msg_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& loop_pose_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<LoopPose> new_loop_pose_;

    std::mutex buff_mutex_; 
};
}
#endif