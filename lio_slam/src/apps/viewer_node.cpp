/*
 * @Description: viewer 的 node 文件
 * @Author: Zhang Jun
 * @Date: 2020-07-15 02:56:27
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include <lio_slam/saveMap.h> // ?saveMap.srv????????
#include "lio_slam/global_defination/global_defination.h"
#include "lio_slam/mapping/viewer/viewer_flow.hpp"

using namespace lio_slam;

std::shared_ptr<ViewerFlow> _viewer_flow_ptr;
bool _need_save_map = false;

bool save_map_callback(saveMap::Request &request, saveMap::Response &response) {
    _need_save_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "viewer_node");
    ros::NodeHandle nh;

    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
    std::shared_ptr<ViewerFlow> _viewer_flow_ptr = std::make_shared<ViewerFlow>(nh, cloud_topic);
    
    // ???? rosservice call /save_map ??????????????  ?????????????????????
    ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);

    ros::Rate rate(50);
    while (ros::ok()) {
        ros::spinOnce();

        _viewer_flow_ptr->Run();
        if (_need_save_map) {
            _need_save_map = false;
            _viewer_flow_ptr->SaveMap();
        }

        rate.sleep();
    }

    return 0;
}
