/*
 * @Description: 闭环检测的node文件
 * @Author: Zhang Jun
 * @Date: 2020-07-15 02:56:27
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include "lio_slam/global_defination/global_defination.h"
#include "lio_slam/mapping/loop_closing/loop_closing_flow.hpp"

using namespace lio_slam;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "loop_closing_node");
    ros::NodeHandle nh;

    std::shared_ptr<LoopClosingFlow> loop_closing_flow_ptr = std::make_shared<LoopClosingFlow>(nh);

    ros::Rate rate(50);
    while (ros::ok()) {
        ros::spinOnce();

        loop_closing_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}