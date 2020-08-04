/*
 * @Description: 前端里程计算法
 * @Author: Zhang Jun
 * @Date: 2020-07-14 18:53:06
 */
#include "lio_slam/mapping/front_end/front_end.hpp"

#include <fstream>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lio_slam/global_defination/global_defination.h"
#include "lio_slam/tools/print_info.hpp"
#include "lio_slam/models/registration/ndt_registration.hpp"
#include "lio_slam/models/cloud_filter/voxel_filter.hpp"
#include "lio_slam/models/cloud_filter/no_filter.hpp"


namespace lio_slam {
FrontEnd::FrontEnd()
    :local_map_ptr_(new CloudData::CLOUD()) {
    
    InitWithConfig();
}

bool FrontEnd::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/front_end.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout << "-----------------前端初始化-------------------" << std::endl;
    InitParam(config_node);
    InitRegistration(registration_ptr_, config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("frame", frame_filter_ptr_, config_node);

    return true;
}

bool FrontEnd::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    local_frame_num_ = config_node["local_frame_num"].as<int>();

    return true;
}

bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout << "前端选择的点云匹配方式为：" << registration_method << std::endl;

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else {
        LOG(ERROR) << "没找到与 " << registration_method << " 相对应的点云匹配方式!";
        return false;
    }

    return true;
}

bool FrontEnd::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "前端" << filter_user << "选择的滤波方法为：" << filter_mothod << std::endl;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else if (filter_mothod == "no_filter") {
        filter_ptr = std::make_shared<NoFilter>();
    } else {
        LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod << " 相对应的滤波方法!";
        return false;
    }

    return true;
}

bool FrontEnd::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {
    current_frame_.cloud_data = cloud_data; // 完成当前帧点云存储    

    // 对当前帧的点云进行体素滤波,进行下采样,减少匹配时的数据量
    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr);

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    // 局部地图容器中没有关键帧，代表是第一帧数据
    // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
    if (local_map_frames_.size() == 0) {
        current_frame_.pose = init_pose_;// 完成第一帧点云的 位姿初始化
        UpdateWithNewFrame(current_frame_);

        // // 假定机器人在理想平面移动,只有x,y平移和z轴旋转角度
        // // 将roll和pitch设为0
        // Eigen::Vector3d eulerAngle = current_frame_.pose.eulerAngles(2,1,0);
        // double z = eulerAngle(2);// 取得z轴旋转角度
        // current_frame_.pose(0,0) = cos(z);current_frame_.pose(0,1) = -sin(z);current_frame_.pose(0,2) = 0;
        // current_frame_.pose(1,0) = sin(z);current_frame_.pose(1,1) = cos(z);current_frame_.pose(1,2) = 0;
        // current_frame_.pose(2,0) = 0;current_frame_.pose(2,1) = 0;current_frame_.pose(2,2) = 1;
        current_frame_.pose(0,2) = 0;current_frame_.pose(1,2) = 0;current_frame_.pose(2,2) = 1;
        current_frame_.pose(2,0) = 0;current_frame_.pose(2,1) = 0;
        current_frame_.pose(2,3) = 0;// 将z轴位置设为0

        cloud_pose = current_frame_.pose;
        return true;
    }

    // 不是第一帧，就正常匹配
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());// ??? 这里的result_cloud_ptr 让我有点迷,后面都没用到啊
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, current_frame_.pose);

        // // 假定机器人在理想平面移动,只有x,y平移和z轴旋转角度
        // // 将roll和pitch设为0
        current_frame_.pose(0,2) = 0;current_frame_.pose(1,2) = 0;current_frame_.pose(2,2) = 1;
        current_frame_.pose(2,0) = 0;current_frame_.pose(2,1) = 0;
        current_frame_.pose(2,3) = 0;// 将z轴位置设为0

    cloud_pose = current_frame_.pose; //匹配后的位姿就是 当前帧的位姿

    // 更新相邻两帧的相对运动。采用匀速运动模型进行位姿预测:上一帧到当前帧的移动位姿=当前帧到下一阵的移动位姿
    // TODO 以imu估计姿态作为 位姿预
    step_pose = last_pose.inverse() * current_frame_.pose;// 当前帧位姿变换矩阵 左乘 上一时刻位姿的逆 = 相对于上一时刻的移动步长
    predict_pose = current_frame_.pose * step_pose;// 当前帧位姿变换矩阵 右乘 移动步长 = 预测的下一帧位姿
    last_pose = current_frame_.pose;

    // 匹配之后根据扫描帧之间的 曼哈顿距离 判断是否需要生成新的关键帧，如果需要，则做相应更新
    if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
        fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
        fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > key_frame_distance_) {
        UpdateWithNewFrame(current_frame_);// 大于设定的生成关键帧的距离时,将当前帧作为关键帧存入,通过这些关键帧来拼接我们的 局部地图
        last_key_frame_pose = current_frame_.pose;
    }

    return true;
}

bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    return true;
}

// 局部地图更新、设定NDT匹配的目标点云
bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame) {
    Frame key_frame = new_key_frame;
    // 这一步的目的是为了把关键帧的点云保存下来
    // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
    // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
    
    // 更新局部地图:如果此时局部地图中的 数量大于了 定义的最大局部地图存储数量local_frame_num_,就把最早的 局部地图弹出,让最新的一帧进来,这就是所谓的 滑窗
    local_map_frames_.push_back(key_frame);
    while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
        local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new CloudData::CLOUD());
    for (size_t i = 0; i < local_map_frames_.size(); ++i) {//遍历滑窗
        // 恢复下采样的点云数据，参数：源点云，变换后的点云，变换矩阵(取自当前关键帧)
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, 
                                 *transformed_cloud_ptr, 
                                 local_map_frames_.at(i).pose);

        *local_map_ptr_ += *transformed_cloud_ptr;// 存好变换后的局部地图帧到局部地图容器里
    }

    // 更新ndt匹配的目标点云
    // 关键帧数量还比较少的时候不滤波，因为点云本来就不多，太稀疏影响匹配效果
    if (local_map_frames_.size() < local_frame_num_/2) {
        registration_ptr_->SetInputTarget(local_map_ptr_);
    } else {
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);// 对局部地图也进行体素滤波,降低数据量
        registration_ptr_->SetInputTarget(filtered_local_map_ptr);// 设定NDT匹配中的 目标点云
                                                                  // registration_ptr_->ScanMatch就是把当前帧点云忘这个 目标点云上匹配，得出的转换矩阵就需要求得的 位姿
    }

    return true;
}
}