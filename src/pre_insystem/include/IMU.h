#pragma once
#include <Eigen/Core>
#include <sensor_msgs/msg/imu.hpp>

#include "Common.h"
#include "Data.h"

namespace insystem {

struct IMU : public Data {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ImuMsg = sensor_msgs::msg::Imu;

    IMU(double *data) {
        stamp_ = data[0];
        gyr_ << data[1], data[2], data[3];
        acc_ << data[4], data[5], data[6];
    }

    /// 基于ImuMsg的构造函数
    explicit IMU(const ImuMsg::SharedPtr &msg) {
        stamp_ = (double)msg->header.stamp.sec + (double)msg->header.stamp.nanosec * 1e-9;
        double &gx = msg->angular_velocity.x;
        double &gy = msg->angular_velocity.y;
        double &gz = msg->angular_velocity.z;
        double &ax = msg->linear_acceleration.x;
        double &ay = msg->linear_acceleration.y;
        double &az = msg->linear_acceleration.z;
        gyr_ << gx, gy, gz;
        acc_ << ax, ay, az;
    }

    Vec3 acc_; ///< 加速度计读数，弧度
    Vec3 gyr_; ///< 陀螺仪读数，m/s^2
};

} // namespace insystem
