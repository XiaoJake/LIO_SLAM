/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Author: Zhang Jun
 * @Date: 2020-07-17 10:38:42
 */
#include "lio_slam/mapping/front_end/front_end_flow.hpp"
#include "glog/logging.h"
#include "lio_slam/global_defination/global_defination.h"

namespace lio_slam {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic) {
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100000);
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, odom_topic, "/odom", "base_link", 1000);

    front_end_ptr_ = std::make_shared<FrontEnd>();
}

bool FrontEndFlow::Run() {
    if (!ReadData())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;

        if (UpdateLaserOdometry()) {
            PublishData();
        }
    }

    return true;
}

bool FrontEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    return true;
}

bool FrontEndFlow::HasData() {
    return cloud_data_buff_.size() > 0;
}

bool FrontEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();

    cloud_data_buff_.pop_front();

    return true;
}

bool FrontEndFlow::UpdateLaserOdometry() {
    static bool odometry_inited = false;
    if (!odometry_inited) {
        odometry_inited = true;
        // 初始位姿直接置0，相当于以启动点作为全局的起点。  如果有其它数据源，比如gps，可以以gps的位姿来初始化，以获得启动点在全局坐标系中的绝对位姿点
        front_end_ptr_->SetInitPose(Eigen::Matrix4f::Identity());
    }

    // 进行激光雷达里程计的更新
    return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
}

bool FrontEndFlow::PublishData() {
    laser_odom_pub_ptr_->Publish(laser_odometry_,current_cloud_data_.time);

    return true;
}
}