/*
 * @Description: 
 * @Author: Zhang Jun
 * @Date: 2020-07-28 19:17:00
 */
#include "lio_slam/sensor_data/key_frame.hpp"

namespace lio_slam {
Eigen::Quaternionf KeyFrame::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;
}
}