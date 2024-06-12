#pragma once

#include <Eigen/Core>

#include "common/common.h"
#include "Data.h"

namespace insystem {

struct ODOM : public Data {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// ODOM的构造函数
    ODOM(double stamp, int leftPulse, int rightPulse);

    double GetVelocity(int n, double r, double dt);

    int leftPulse_;  ///< 左轮单位时间转过的脉冲数
    int rightPulse_; ///< 右轮单位时间转过的脉冲数
};

} // namespace insystem