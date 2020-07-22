/*
 * @Description: 发布tf的类
 * @Author: Zhang Jun
 * @Date: 2020-08-05 15:23:26
 */

#ifndef LIO_SLAM_PUBLISHER_TF_BROADCASTER_HPP_
#define LIO_SLAM_PUBLISHER_TF_BROADCASTER_HPP_

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

namespace lio_slam {
class TFBroadCaster {
  public:
    TFBroadCaster(std::string frame_id, std::string child_frame_id);
    TFBroadCaster() = default;
    void SendTransform(Eigen::Matrix4f pose, double time);
  protected:
    tf::StampedTransform transform_;
    tf::TransformBroadcaster broadcaster_;
};
}
#endif