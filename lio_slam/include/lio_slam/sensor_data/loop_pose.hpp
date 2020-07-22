/*
 * @Description: 关键帧之间的相对位姿，用于闭环检测
 * @Author: Zhang Jun
 * @Date: 2020-07-28 19:13:26
 */
#ifndef LIO_SLAM_SENSOR_DATA_LOOP_POSE_HPP_
#define LIO_SLAM_SENSOR_DATA_LOOP_POSE_HPP_

#include <Eigen/Dense>

namespace lio_slam {
class LoopPose {
  public:
    double time = 0.0;
    unsigned int index0 = 0;
    unsigned int index1 = 0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

  public:
    Eigen::Quaternionf GetQuaternion();
};
}

#endif