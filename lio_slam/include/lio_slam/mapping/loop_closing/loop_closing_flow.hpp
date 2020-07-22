/*
 * @Description: 
 * @Author: Zhang Jun
 * @Date: 2020-07-29 03:32:14
 */
#ifndef LIO_SLAM_MAPPING_LOOP_CLOSING_LOOP_CLOSING_FLOW_HPP_
#define LIO_SLAM_MAPPING_LOOP_CLOSING_LOOP_CLOSING_FLOW_HPP_

#include <deque>
#include <ros/ros.h>
// subscriber
#include "lio_slam/subscriber/key_frame_subscriber.hpp"
// publisher
#include "lio_slam/publisher/loop_pose_publisher.hpp"
// loop closing
#include "lio_slam/mapping/loop_closing/loop_closing.hpp"

namespace lio_slam {
class LoopClosingFlow {
  public:
    LoopClosingFlow(ros::NodeHandle& nh);

    bool Run();

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool PublishData();

  private:
    // subscriber
    std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
    std::shared_ptr<KeyFrameSubscriber> key_gnss_sub_ptr_;
    // publisher
    std::shared_ptr<LoopPosePublisher> loop_pose_pub_ptr_;
    // loop closing
    std::shared_ptr<LoopClosing> loop_closing_ptr_;

    std::deque<KeyFrame> key_frame_buff_;
    std::deque<KeyFrame> key_gnss_buff_;

    KeyFrame current_key_frame_;
    KeyFrame current_key_gnss_;
};
}

#endif