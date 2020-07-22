/*
 * @Description: 点云滤波模块的接口
 * @Author: Zhang Jun
 * @Date: 2020-07-19 19:29:50
 */
#ifndef LIO_SLAM_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_
#define LIO_SLAM_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include "lio_slam/sensor_data/cloud_data.hpp"

namespace lio_slam {
class CloudFilterInterface {
  public:
    virtual ~CloudFilterInterface() = default;

    virtual bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) = 0;
};
}

#endif