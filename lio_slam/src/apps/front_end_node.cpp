/*
 * @Description: 前端里程计的node文件
 * @Author: Zhang Jun
 * @Date: 2020-07-15 02:56:27
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include "lio_slam/global_defination/global_defination.h"
#include "lio_slam/mapping/front_end/front_end_flow.hpp"

using namespace lio_slam;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    std::string cloud_topic, odom_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
    nh.param<std::string>("odom_topic", odom_topic, "/laser_odom");

    std::shared_ptr<FrontEndFlow> front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh, cloud_topic, odom_topic);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        front_end_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}