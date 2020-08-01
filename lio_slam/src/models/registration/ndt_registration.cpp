/*
 * @Description: NDT 匹配模块
 * @Author: Zhang Jun
 * @Date: 2020-07-18 21:46:45
 */
#include "lio_slam/models/registration/ndt_registration.hpp"

#include "glog/logging.h"

namespace lio_slam {

NDTRegistration::NDTRegistration(const YAML::Node& node)
    :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {
    
    float res = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter)
    :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter) {
    ndt_ptr_->setResolution(res);// voxel的边长大小,过小造成内存占用过高,过大会导致误差偏大;
    ndt_ptr_->setStepSize(step_size);// 牛顿法优化的最大步长;
    ndt_ptr_->setTransformationEpsilon(trans_eps);//两次变换之间允许的最大值,用于判断是否收敛,作为迭代计算完成的阈值;
    ndt_ptr_->setMaximumIterations(max_iter);// 最大迭代次数,超过则停止计算;

    std::cout << "NDT 的匹配参数为：" << std::endl
              << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter 
              << std::endl << std::endl;

    return true;
}

bool NDTRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    ndt_ptr_->setInputTarget(input_target);

    return true;
}

bool NDTRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
ndt_ptr_->setInputSource(input_source);// 输入需要被匹配的 源点云 
    ndt_ptr_->align(*result_cloud_ptr, predict_pose);// 计算需要的刚体变换以便将输入的源点云匹配到 目标点云
                                                     // 以 predict_pose 为初始值进行迭代优化，将结果保存在 final_transformation_ 中。内部最后还是调用的 computeTransformation 完成最终的计算
    result_pose = ndt_ptr_->getFinalTransformation();

    return true;
}

float NDTRegistration::GetFitnessScore() {
    return ndt_ptr_->getFitnessScore();
}
}