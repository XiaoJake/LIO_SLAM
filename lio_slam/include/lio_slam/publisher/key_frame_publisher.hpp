/*
 * @Description: 单个 key frame 信息发布
 * @Author: Zhang Jun
 * @Date: 2020-07-16 21:05:47
 */
#ifndef LIO_SLAM_PUBLISHER_KEY_FRAME_PUBLISHER_HPP_
#define LIO_SLAM_PUBLISHER_KEY_FRAME_PUBLISHER_HPP_

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "lio_slam/sensor_data/key_frame.hpp"

namespace lio_slam {
class KeyFramePublisher {
  public:
    KeyFramePublisher(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      std::string frame_id,
                      int buff_size);
    KeyFramePublisher() = default;

    void Publish(KeyFrame& key_frame);

    bool HasSubscribers();

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";
};
}
#endif