/*
 * @Description: 点云畸变补偿
 * @Author: Zhang Jun
 * @Date: 2020-07-25 14:38:12
 */

#ifndef LIO_SLAM_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_
#define LIO_SLAM_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "glog/logging.h"

#include "lio_slam/models/scan_adjust/distortion_adjust.hpp"
#include "lio_slam/sensor_data/velocity_data.hpp"
#include "lio_slam/sensor_data/cloud_data.hpp"

namespace lio_slam {
class DistortionAdjust {
  public:
    void SetMotionInfo(float scan_period, VelocityData velocity_data);
    bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr);

  private:
    inline Eigen::Matrix3f UpdateMatrix(float real_time);

  private:
    float scan_period_;
    Eigen::Vector3f velocity_;
    Eigen::Vector3f angular_rate_;
};
} // namespace lidar_slam
#endif