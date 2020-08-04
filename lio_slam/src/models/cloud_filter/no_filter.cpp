/*
 * @Description: 不滤波
 * @Author: Zhang Jun
 * @Date: 2020-07-19 19:53:20
 */
#include "lio_slam/models/cloud_filter/no_filter.hpp"
#include "glog/logging.h"

namespace lio_slam {
NoFilter::NoFilter() {
}

bool NoFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
    filtered_cloud_ptr.reset(new CloudData::CLOUD(*input_cloud_ptr));
    return true;
}
} 