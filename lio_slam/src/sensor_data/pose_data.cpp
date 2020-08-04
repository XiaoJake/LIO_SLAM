/*
 * @Description: 
 * @Author: Zhang Jun
 * @Date: 2020-07-28 18:50:16
 */
#include "lio_slam/sensor_data/pose_data.hpp"

namespace lio_slam {
Eigen::Quaternionf PoseData::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;
}
}