/*
 * @Description: 在ros中发布IMU数据
 * @Author: Zhang Jun
 * @Date: 2020-07-15 02:27:30
 */
#ifndef LIO_SLAM_PUBLISHER_IMU_PUBLISHER_HPP_
#define LIO_SLAM_PUBLISHER_IMU_PUBLISHER_HPP_

#include "sensor_msgs/Imu.h"
#include "lio_slam/sensor_data/imu_data.hpp"

namespace lio_slam {
class IMUPublisher {
  public:
    IMUPublisher(ros::NodeHandle& nh,
                 std::string topic_name,
                 size_t buff_size,
                 std::string frame_id);
    IMUPublisher() = default;

    void Publish(IMUData imu_data);

    bool HasSubscribers();

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
} 
#endif