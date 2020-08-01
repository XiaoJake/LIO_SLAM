/*
 * @Description: 关键帧，在各个模块之间传递数据
 * @Author: Zhang Jun
 * @Date: 2020-07-28 19:13:26
 */
#ifndef LIO_SLAM_SENSOR_DATA_KEY_FRAME_HPP_
#define LIO_SLAM_SENSOR_DATA_KEY_FRAME_HPP_

#include <Eigen/Dense>

namespace lio_slam {
class KeyFrame {
  public:
    double time = 0.0;
    size_t index = 0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

  public:
    Eigen::Quaternionf GetQuaternion();
};
}

#endif