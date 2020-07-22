/*
 * @Description: 地图匹配定位的node文件
 * @Author: Zhang Jun
 * @Date: 2020-07-15 02:56:27
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include "lio_slam/global_defination/global_defination.h"
#include "lio_slam/matching/matching_flow.hpp"

using namespace lio_slam;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "matching_node");
    ros::NodeHandle nh;

    std::shared_ptr<MatchingFlow> matching_flow_ptr = std::make_shared<MatchingFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        matching_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}