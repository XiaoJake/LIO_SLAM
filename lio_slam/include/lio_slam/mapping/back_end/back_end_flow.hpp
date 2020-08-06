/*
 * @Description: back end 任务管理， 放在类里使代码更清晰
 * @Author: Zhang Jun
 * @Date: 2020-07-17 10:31:22
 */
#ifndef LIO_SLAM_MAPPING_BACK_END_FRONT_END_FLOW_HPP_
#define LIO_SLAM_MAPPING_BACK_END_FRONT_END_FLOW_HPP_

#include <ros/ros.h>

#include "lio_slam/subscriber/cloud_subscriber.hpp"
#include "lio_slam/subscriber/odometry_subscriber.hpp"
#include "lio_slam/subscriber/loop_pose_subscriber.hpp"
#include "lio_slam/tf_listener/tf_listener.hpp"

#include "lio_slam/publisher/odometry_publisher.hpp"
#include "lio_slam/publisher/key_frame_publisher.hpp"
#include "lio_slam/publisher/key_frames_publisher.hpp"
#include "lio_slam/publisher/tf_broadcaster.hpp"

#include "lio_slam/mapping/back_end/back_end.hpp"

namespace lio_slam {
class BackEndFlow {
  public:
    BackEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic);

    bool Run();

    bool ForceOptimize();

  private:
    bool ReadData();
    bool MaybeInsertLoopPose();
    bool HasData();
    bool ValidData();
    bool UpdateBackEnd();
    bool PublishData();

  private:
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> gnss_pose_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> laser_odom_sub_ptr_;
    std::shared_ptr<LoopPoseSubscriber> loop_pose_sub_ptr_;
    std::shared_ptr<TFListener> lidar_to_base_ptr_;
    Eigen::Matrix4f lidar_to_base_ = Eigen::Matrix4f::Identity();

    std::shared_ptr<OdometryPublisher> transformed_odom_pub_ptr_;
    std::shared_ptr<KeyFramePublisher> key_frame_pub_ptr_;
    std::shared_ptr<KeyFramePublisher> key_gnss_pub_ptr_;
    std::shared_ptr<KeyFramesPublisher> key_frames_pub_ptr_;
    std::shared_ptr<BackEnd> back_end_ptr_;
    std::shared_ptr<TFBroadCaster> laser_tf_pub_ptr_;
    std::shared_ptr<TFBroadCaster> map_tf_pub_ptr_;

    std::deque<CloudData> cloud_data_buff_;
    std::deque<PoseData> gnss_pose_data_buff_;
    std::deque<PoseData> laser_odom_data_buff_;
    std::deque<LoopPose> loop_pose_data_buff_;

    PoseData current_gnss_pose_data_;
    PoseData current_laser_odom_data_;
    CloudData current_cloud_data_;
    Eigen::Matrix4f base_to_odom_ = Eigen::Matrix4f::Identity();
};
}

#endif