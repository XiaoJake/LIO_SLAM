/*
 * @Description: 
 * @Author: Zhang Jun
 * @Date: 2020-07-28 19:17:00
 */
#include "lio_slam/sensor_data/loop_pose.hpp"

namespace lio_slam {
Eigen::Quaternionf LoopPose::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;
}
}