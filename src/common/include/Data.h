#pragma once

#include <Eigen/Core>

namespace insystem {

struct Data {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Data() = default;

    virtual ~Data() = default;

    double stamp_; ///< 时间戳类型
};

} // namespace insystem