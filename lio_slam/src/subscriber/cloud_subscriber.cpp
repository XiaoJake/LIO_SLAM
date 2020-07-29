/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Author: Zhang Jun
 * @Date: 2020-07-15 02:27:30
 */

#include "lio_slam/subscriber/cloud_subscriber.hpp"

#include "glog/logging.h"

namespace lio_slam {
CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
    std::cout << "subscribe cloud data from topic:" << topic_name << std::endl;
}

void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    //std::cout << "called CloudSubscriber::msg_callback" << std::endl;
    //buff_mutex_.lock();
    //std::cout << "started convert ros to pcl" << std::endl;

    CloudData cloud_data;
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));// 将ros中的点云数据转换为pcl的点云数据格式

    new_cloud_data_.push_back(cloud_data);
    //buff_mutex_.unlock();
}

void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff) {
    //buff_mutex_.lock();
    if (new_cloud_data_.size() > 0) {
        cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();
        //std::cout << "CloudData parse succeed" << std::endl;
    }
    //std::cout << "CloudData parse failed" << std::endl;
    //buff_mutex_.unlock();
}
} // namespace data_input