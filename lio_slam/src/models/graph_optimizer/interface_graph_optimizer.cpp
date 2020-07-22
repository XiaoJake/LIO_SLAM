/*
 * @Description: 
 * @Author: Zhang Jun
 * @Date: 2020-08-01 18:35:19
 */

#include "lio_slam/models/graph_optimizer/interface_graph_optimizer.hpp"

namespace lio_slam {
void InterfaceGraphOptimizer::SetMaxIterationsNum(int max_iterations_num) {
    max_iterations_num_ = max_iterations_num;
}
}