/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Zhang Jun
 * @Date: 2020-07-17 10:31:22
 */
#ifndef LIO_SLAM_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_
#define LIO_SLAM_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_

#include <ros/ros.h>
// subscriber
#include "lio_slam/subscriber/cloud_subscriber.hpp"
#include "lio_slam/subscriber/imu_subscriber.hpp"
#include "lio_slam/subscriber/velocity_subscriber.hpp"
#include "lio_slam/subscriber/gnss_subscriber.hpp"
#include "lio_slam/tf_listener/tf_listener.hpp"
// publisher
#include "lio_slam/publisher/cloud_publisher.hpp"
#include "lio_slam/publisher/odometry_publisher.hpp"
// models
#include "lio_slam/models/scan_adjust/distortion_adjust.hpp"

namespace lio_slam {
class DataPretreatFlow {
  public:
    DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic);
    void removeClosedPointCloud(const CloudData& cloud_in,CloudData& cloud_out, float thres);

    bool Run();

  private:
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool TransformData();
    bool PublishData();

  private:
    // subscriber
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    std::shared_ptr<TFListener> lidar_to_imu_ptr_;
    // publisher
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
    // models
    std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

    std::deque<CloudData> cloud_data_buff_;
    std::deque<IMUData> imu_data_buff_;
    std::deque<VelocityData> velocity_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;

    CloudData current_cloud_data_;
    IMUData current_imu_data_;
    VelocityData current_velocity_data_;
    GNSSData current_gnss_data_;

    Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
};
}

#endif