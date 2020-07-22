/*
 * @Description: imu data
 * @Author: Zhang Jun
 * @Date: 2020-07-23 22:20:41
 */
#include "lio_slam/sensor_data/imu_data.hpp"

#include <cmath>
#include "glog/logging.h"

namespace lio_slam {
Eigen::Matrix3f IMUData::GetOrientationMatrix() {
    Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
    Eigen::Matrix3f matrix = q.matrix().cast<float>();

    return matrix;
}

bool IMUData::SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time) {
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做插值
    // 时间索引：把雷达采集时刻在其他传感器的时间线里找到对应的位置，然后找到该位置前后两帧的数据
    bool flag_missing = false;
    while (UnsyncedData.size() >= 2) {
        // 如果第一个数据时间比雷达时间还要靠后，即插入时刻的前面没有数据，那么就无从插入，直接退出
        if (UnsyncedData.front().time > sync_time) 
            return false;
        // 如果第一个数据比插入时刻早，第二个数据也比插入时刻早，那么第一个时刻的数据是没意义的，应该接着往下找，并删除第一个数据
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        // 通过了前两个if判断，到这里说明雷达采集时刻已经处在前两个数据的中间。还需判断第一个数据时刻与雷达采集时刻时间差是否过大，若过大则中间肯定丢数据了，退出
        if (sync_time - UnsyncedData.front().time > 0.2) {
            UnsyncedData.pop_front();
            flag_missing = true;
            break;
        }
        // 同样，如果第二个数据时刻与雷达采集时刻时间差过大，那么也是丢数据了，也退出。但这个数据不要删除,因为它在下一帧雷达采集时刻的前面。
        if (UnsyncedData.at(1).time - sync_time > 0.2) {
            //UnsyncedData.pop_front();
            flag_missing = true;
            break;
        }
        break;
    }
    if (flag_missing == true)
        return false;

    IMUData front_data = UnsyncedData.at(0);
    IMUData back_data = UnsyncedData.at(1);
    IMUData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
    synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
    synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
    synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
    // 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
    // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
    synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
    synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
    synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
    synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
    // 线性插值之后要归一化
    synced_data.orientation.Normlize();

    SyncedData.push_back(synced_data);

    return true;
}
}