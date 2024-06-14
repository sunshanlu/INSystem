#pragma once

#include <Eigen/Core>

#include "Common.h"
#include "Data.h"
#include "pre_interfaces/msg/wodom.hpp"

namespace insystem {

struct ODOM : public Data {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// ODOM的构造函数
    ODOM(double stamp, int leftPulse, int rightPulse);

    /// 基于消息的构造函数
    ODOM(const pre_interfaces::msg::Wodom::SharedPtr &msg);

    double GetVelocity(int n, double r, double dt);

    int leftPulse_;  ///< 左轮单位时间转过的脉冲数
    int rightPulse_; ///< 右轮单位时间转过的脉冲数
};

} // namespace insystem