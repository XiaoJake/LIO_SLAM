/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Author: Zhang Jun
 * @Date: 2020-07-15 02:27:30
 */

#include "lio_slam/subscriber/cloud_subscriber.hpp"

#include <pcl/filters/voxel_grid.h>
#include "glog/logging.h"

namespace lio_slam {
CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
    //std::cout << "subscribe cloud data from topic:" << topic_name << std::endl;
}

void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    //std::cout << "called CloudSubscriber::msg_callback" << std::endl;
    buff_mutex_.lock();
    CloudData cloud_data;
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));// 将ros中的点云数据转换为pcl的点云数据格式

    new_cloud_data_.push_back(cloud_data);
    buff_mutex_.unlock();
}

// 只保留距离在[head,end]区间内的点  其中,一些距离激光雷达过近的点，通常这些点被认为是不可靠的
template <typename PointT>
void CloudSubscriber::limitDistancePointCloud(const pcl::PointCloud<PointT> &cloud_in,pcl::PointCloud<PointT> &cloud_out, float head,float end)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (head * head > cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z
        || end * end < cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff) {
    buff_mutex_.lock();
    if(new_cloud_data_.size() > 0) {

    for(size_t i=0;i < new_cloud_data_.size();i++) {        
        // 去除原始激光点云里无效的NaN点:输入点云，输出点云，索引
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*new_cloud_data_.at(i).cloud_ptr, *new_cloud_data_.at(i).cloud_ptr, indices);
        limitDistancePointCloud(*new_cloud_data_.at(i).cloud_ptr, *new_cloud_data_.at(i).cloud_ptr, 0.3, 50.0);
    }

        cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();// 完成数据读取后，就清除容器空间，以便下一次在回调函数中继续存储 点云话题中的数据
        //std::cout << "CloudData parse succeed" << std::endl;
    }
    //std::cout << "CloudData parse failed" << std::endl;
    buff_mutex_.unlock();
}
} // namespace data_input