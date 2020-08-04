/*
 * @Description: velocity 数据
 * @Author: Zhang Jun
 * @Date: 2020-07-17 18:27:40
 */
#ifndef LIO_SLAM_SENSOR_DATA_VELOCITY_DATA_HPP_
#define LIO_SLAM_SENSOR_DATA_VELOCITY_DATA_HPP_

#include <deque>
#include <Eigen/Dense>

namespace lio_slam {
class VelocityData {
  public:
    struct LinearVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    struct AngularVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    double time = 0.0;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;
  
  public:
    static bool SyncData(std::deque<VelocityData>& UnsyncedData, std::deque<VelocityData>& SyncedData, double sync_time);
    void TransformCoordinate(Eigen::Matrix4f transform_matrix);
};
}
#endif