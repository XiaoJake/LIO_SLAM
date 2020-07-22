/*
 * @Description: 点云匹配模块的基类
 * @Author: Zhang Jun
 * @Date: 2020-07-18 21:25:11
 */
#ifndef LIO_SLAM_MODELS_REGISTRATION_INTERFACE_HPP_
#define LIO_SLAM_MODELS_REGISTRATION_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "lio_slam/sensor_data/cloud_data.hpp"

namespace lio_slam {
class RegistrationInterface {
  public:
    virtual ~RegistrationInterface() = default;

    virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) = 0;
    virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                          const Eigen::Matrix4f& predict_pose, 
                          CloudData::CLOUD_PTR& result_cloud_ptr,
                          Eigen::Matrix4f& result_pose) = 0;
    virtual float GetFitnessScore() = 0;
};
} 

#endif