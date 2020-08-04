/*
 * @Description: 存放处理后的IMU姿态以及GNSS位置
 * @Author: Zhang Jun
 * @Date: 2020-07-27 23:10:56
 */
#ifndef LIO_SLAM_SENSOR_DATA_POSE_DATA_HPP_
#define LIO_SLAM_SENSOR_DATA_POSE_DATA_HPP_

#include <Eigen/Dense>

namespace lio_slam {
class PoseData {
  public:
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    double time = 0.0;

  public:
    Eigen::Quaternionf GetQuaternion();
};
}

#endif