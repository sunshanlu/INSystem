#pragma once

#include <Eigen/Core>

#include "common/common.h"

namespace insystem {

struct ODOM {

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double stamp_; ///< 时间戳
    double v_;     ///< 速度
};

} // namespace insystem