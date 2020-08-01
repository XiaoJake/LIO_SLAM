/*
 * @Description:
 * @Author: Zhang Jun
 * @Date: 2020-07-17 18:17:49
 */
#ifndef LIO_SLAM_SENSOR_DATA_CLOUD_DATA_HPP_
#define LIO_SLAM_SENSOR_DATA_CLOUD_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace lio_slam {
class CloudData {
  public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

  public:
    CloudData()
      :cloud_ptr(new CLOUD()) {
    }

  public:
    double time = 0.0;
    CLOUD_PTR cloud_ptr;
};
}

#endif