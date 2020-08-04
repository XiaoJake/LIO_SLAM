/*
 * @Description: 发布tf的类
 * @Author: Zhang Jun
 * @Date: 2020-08-05 15:23:26
 */

#include "lio_slam/publisher/tf_broadcaster.hpp"

namespace lio_slam {
TFBroadCaster::TFBroadCaster(std::string frame_id, std::string child_frame_id) {
    transform_.frame_id_ = frame_id;
    transform_.child_frame_id_ = child_frame_id;
}

// 用这个time来发布坐标转换的话,会在rviz里看到  刚启动的时候有坐标树,过几秒就缓慢消失
void TFBroadCaster::SendTransform(Eigen::Matrix4f pose, double time) {
    Eigen::Quaternionf q(pose.block<3,3>(0,0));
    ros::Time ros_time((float)time);
    transform_.stamp_ = ros_time;
    transform_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    transform_.setOrigin(tf::Vector3(pose(0,3), pose(1,3), pose(2,3)));
    broadcaster_.sendTransform(transform_);
}

// 用这个ros::Time::now()来设置时间戳,就是正常的
void TFBroadCaster::SendTransform(Eigen::Matrix4f pose) {
    Eigen::Quaternionf q(pose.block<3,3>(0,0));
    transform_.stamp_ = ros::Time::now();
    transform_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    transform_.setOrigin(tf::Vector3(pose(0,3), pose(1,3), pose(2,3)));
    broadcaster_.sendTransform(transform_);
}

}