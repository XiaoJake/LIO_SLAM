/*
 * @Description: 数据预处理模块，包括去除过近点、时间同步、点云去畸变等
 * @Author: Zhang Jun
 * @Date: 2020-07-17 10:38:42
 */
#include "lio_slam/data_pretreat/data_pretreat_flow.hpp"

#include "glog/logging.h"
#include "lio_slam/global_defination/global_defination.h"

namespace lio_slam {

DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic) {
    // subscriber
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/velodyne_points", 10000);//   /kitti/velo/pointcloud
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/imu/data", 10000);// /kitti/oxts/imu
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "/imu_link", "/laser_link");
/*     velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 10000);
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 10000); */

    // publisher
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, cloud_topic, "/laser_link", 100);
/*     gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/map", "/laser_link", 100); */

    /* distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>(); */
}

bool DataPretreatFlow::Run() {

    // 完成数据读取、时间同步
    if (!ReadData())
    {
        //std::cout << "Read data failed" << std::endl;
        return false;
    }
        //std::cout << "Read data succeed" << std::endl;
    if (!InitCalibration()) 
    {
        return false;
    }

/*     if (!InitGNSS())
        return false; */

    while(HasData()) {
        if (!ValidData())
            continue;
    
/*     去除运动畸变
        TransformData(); */
    
    // 发布预处理好的数据
    PublishData();
    }

    return true;
}

bool DataPretreatFlow::ReadData() {
    // 时间同步
    // 关于插入时刻问题，因为我们是以雷达作为核心传感器，所以每收到一次雷达数据，就以当前雷达数据采集时刻
    // 作为要插入的时间点，从而获得除雷达以外的其他传感器的同一时刻等效信息。
    cloud_sub_ptr_->ParseData(cloud_data_buff_);// 雷达数据为时间参考基准，不需要做插值

    static std::deque<IMUData> unsynced_imu_;
/*     static std::deque<VelocityData> unsynced_velocity_;
    static std::deque<GNSSData> unsynced_gnss_; */

    // 临时存储还未做时间同步的 IMU、速度、gnss数据
    imu_sub_ptr_->ParseData(unsynced_imu_);
/*     velocity_sub_ptr_->ParseData(unsynced_velocity_);
    gnss_sub_ptr_->ParseData(unsynced_gnss_); */

    if (cloud_data_buff_.size() == 0)
    {
        //std::cout << "cloud data size error,now size is:" << cloud_data_buff_.size() << std::endl;
        return false;
    }
        

    // 对IMU、速度、gnss数据进行时间同步
    double cloud_time = cloud_data_buff_.front().time;
    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
/*     bool valid_velocity = VelocityData::SyncData(unsynced_velocity_, velocity_data_buff_, cloud_time);
    bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time); */

    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!valid_imu /* || !valid_velocity || !valid_gnss */) {
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    //std::cout << "imudata synced succeed" << std::endl;
    return true;
}

// 基于Lidar-IMU联合标定的外参,完成lidar到imu的数据校准
bool DataPretreatFlow::InitCalibration() {
    static bool calibration_received = false;
    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
        }
    }

    return calibration_received;
}

bool DataPretreatFlow::InitGNSS() {
    static bool gnss_inited = false;
    if (!gnss_inited) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool DataPretreatFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;
/*     if (velocity_data_buff_.size() == 0)
        return false;
    if (gnss_data_buff_.size() == 0)
        return false; */

    return true;
}

bool DataPretreatFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
/*     current_velocity_data_ = velocity_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front(); */

    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
/*     double diff_velocity_time = current_cloud_data_.time - current_velocity_data_.time;
    double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time; */

    if (diff_imu_time < -0.05 /* || diff_velocity_time < -0.05 || diff_gnss_time < -0.05 */) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_imu_time > 0.05) {
        imu_data_buff_.pop_front();
        return false;
    }

/*     if (diff_velocity_time > 0.05) {
        velocity_data_buff_.pop_front();
        return false;
    }

    if (diff_gnss_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    } */

    // 当前点云已经完成存储,需要弹出 一是节省空间 二是, cloud_data_buff_.front()就能代表容器里的下帧点云了
    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
/*     velocity_data_buff_.pop_front();
    gnss_data_buff_.pop_front(); */

    return true;
}

// 去除点云运动畸变 基于gnss位姿、速度信息
bool DataPretreatFlow::TransformData() {
    gnss_pose_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();
    gnss_pose_(0,3) = current_gnss_data_.local_E;
    gnss_pose_(1,3) = current_gnss_data_.local_N;
    gnss_pose_(2,3) = current_gnss_data_.local_U;
    gnss_pose_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    gnss_pose_ *= lidar_to_imu_;

    current_velocity_data_.TransformCoordinate(lidar_to_imu_);
    distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);

    return true;
}

bool DataPretreatFlow::PublishData() {
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr,current_cloud_data_.time);
    //cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr);
/*     gnss_pub_ptr_->Publish(gnss_pose_, current_gnss_data_.time); */

    return true;
}
}