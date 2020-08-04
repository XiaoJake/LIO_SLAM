/*
 * @Description: 
 * @Author: Zhang Jun
 * @Date: 2020-07-17 18:25:13
 */
#ifndef LIO_SLAM_SENSOR_DATA_GNSS_DATA_HPP_
#define LIO_SLAM_SENSOR_DATA_GNSS_DATA_HPP_

#include <deque>

#include "Geocentric/LocalCartesian.hpp"

namespace lio_slam {
class GNSSData {
  public:
    double time = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
    double altitude = 0.0;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;
    int status = 0;
    int service = 0;

    static double origin_longitude;
    static double origin_latitude;
    static double origin_altitude;

  private:
    static GeographicLib::LocalCartesian geo_converter;
    static bool origin_position_inited;

  public: 
    void InitOriginPosition();
    void UpdateXYZ();
    static bool SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time);
};
}
#endif