#pragma once
#include <Eigen/Core>

#include "Data.h"
#include "common/common.h"

namespace insystem {

struct IMU : public Data {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IMU(double *data) {
        stamp_ = data[0];
        gyr_ << data[1], data[2], data[3];
        acc_ << data[4], data[5], data[6];
    }

    Vec3 acc_;     ///< 加速度计读数
    Vec3 gyr_;     ///< 陀螺仪读数
};

} // namespace insystem
