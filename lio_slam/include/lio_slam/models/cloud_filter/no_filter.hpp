/*
 * @Description: 不滤波
 * @Author: Zhang Jun
 * @Date: 2020-07-19 19:37:49
 */
#ifndef LIO_SLAM_MODELS_CLOUD_FILTER_NO_FILTER_HPP_
#define LIO_SLAM_MODELS_CLOUD_FILTER_NO_FILTER_HPP_

#include "lio_slam/models/cloud_filter/cloud_filter_interface.hpp"

namespace lio_slam {
class NoFilter: public CloudFilterInterface {
  public:
    NoFilter();

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;
};
}
#endif