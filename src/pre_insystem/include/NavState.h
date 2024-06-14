#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "Common.h"

namespace insystem {
struct NavState {
    SE3 p_;        ///< 位姿
    Vec3 v_;       ///< 速度
    Vec3 bg_;      ///< 陀螺仪偏置
    Vec3 ba_;      ///< 加速度计偏置
    double stamp_; ///< 时间戳
};
} // namespace insystem
