#pragma once
#include <Eigen/Core>

namespace insystem {

struct IMU {
    Eigen::Vector2d acc_; ///< 加速度计读数
    Eigen::Vector2d gyr_; ///< 陀螺仪读数
    double stamp_;        ///< imu数据的时间戳
};

} // namespace insystem
