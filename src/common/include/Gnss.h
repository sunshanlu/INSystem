#pragma once

#include <Eigen/Core>

#include "common/common.h"

namespace insystem {

struct Gnss {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SE3 p_;         ///< Gnss位姿
    double stamp_;  ///< Gnss数据时间戳
    bool is_valid_; ///< Gnss数据航向角的有效性
};

} // namespace insystem