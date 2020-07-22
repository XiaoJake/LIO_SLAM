/*
 * @Description: 打印信息
 * @Author: Zhang Jun
 * @Date: 2020-08-02 23:25:26
 */
#ifndef LIO_SLAM_TOOLS_PRINT_INFO_HPP_
#define LIO_SLAM_TOOLS_PRINT_INFO_HPP_

#include <cmath>
#include <string>
#include <Eigen/Dense>
#include "pcl/common/eigen.h"

namespace lio_slam {
class PrintInfo {
  public:
    static void PrintPose(std::string head, Eigen::Matrix4f pose);
};
}
#endif