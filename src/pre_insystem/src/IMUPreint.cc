#include "IMUPreint.h"
#include "IMUIniter.h"
#include "NavState.h"

namespace insystem {

/**
 * @brief IMU预积分函数
 *
 * @param imu   输入的imu数据
 * @param dt    输入的imu积分时间
 */
void IMUPreint::Integrate(const IMUSharedPtr &imu, const double &dt) {
    const double dt2 = dt * dt;
    Vec3 acc = imu->acc_ - ba_;
    Vec3 gyr = imu->gyr_ - bg_;

    Mat9 A = Mat9::Identity();
    Mat9_6 B = Mat9_6::Zero();

    /// 1. dp和dv的状态更新
    dp_ += dv_ * dt + 0.5 * dR_.matrix() * acc * dt2;
    dv_ += dR_.matrix() * acc * dt;

    /// 2. 预积分模型噪声矩阵更新
    A.block<3, 3>(3, 0) = -dR_.matrix() * SO3::hat(acc) * dt;
    A.block<3, 3>(6, 0) = -0.5 * dR_.matrix() * SO3::hat(acc) * dt2;
    A.block<3, 3>(6, 3) = Mat3::Identity() * dt;

    B.block<3, 3>(3, 3) = dR_.matrix() * dt;
    B.block<3, 3>(6, 3) = 0.5 * dR_.matrix() * dt2;

    /// 3. 更新观测对零偏的雅可比矩阵(pv部分)
    dp_dba += dv_dba * dt - 0.5 * dR_.matrix() * dt2;
    dp_dbg += dv_dbg * dt - 0.5 * dR_.matrix() * SO3::hat(acc) * dR_dbg * dt2;

    /// 4. dR状态的更新
    Vec3 omega = gyr * dt;
    SO3 deltaR = SO3::exp(omega);
    Mat3 jr = SO3::leftJacobian(-omega);
    dR_ = dR_ * deltaR;

    /// 5. 预积分噪声矩阵更新
    A.block<3, 3>(0, 0) = deltaR.inverse().matrix();
    B.block<3, 3>(0, 0) = jr * dt;
    cov_ = A * cov_ * A.transpose() + B * covEta_ * B.transpose();

    /// 6. 更新观测对零偏的雅可比矩阵(R部分)
    dR_dbg = deltaR.inverse().matrix() * dR_dbg - jr * dt;

    /// 6. 更新积分时间
    dt_ += dt;
}

/**
 * @brief 根据i时刻的导航状态和重力信息预测j时刻的状态
 *
 * @param start     输入的i时刻的导航状态
 * @param end       输出的j时刻的导航状态
 * @param g         输入的重力信息
 */
void IMUPreint::Predict(const NavState &start, NavState &end, const Vec3 &g) {
    end.p_.so3() = PredictRj(start.p_.so3());
    end.p_.translation() = PredictPj(start.p_.translation(), start.p_.so3(), g, start.v_);
    end.v_ = PredictVj(start.v_, start.p_.so3(), g);
    end.ba_ = start.ba_;
    end.bg_ = start.bg_;
}

IMUPreint::IMUPreint()
    : dR_(SO3())
    , dv_(Vec3::Zero())
    , dp_(Vec3::Zero())
    , dR_dbg(Mat3::Zero())
    , dv_dba(Vec3::Zero())
    , dv_dbg(Vec3::Zero())
    , dp_dba(Vec3::Zero())
    , dp_dbg(Vec3::Zero())
    , cov_(Mat15::Zero())
    , dt_(0) {}

/**
 * @brief 设置预积分器的imu信息
 * 
 * @param imu_initer 输入的imu初始化器指针
 */
void IMUPreint::SetIMUInfo(IMUIniterSharedPtr imu_initer) {
    ba_ = imu_initer->GetBiasA();
    bg_ = imu_initer->GetBiasG();
    covEta_.diagonal() << imu_initer->GetCovG(), imu_initer->GetCovA();
}

} // namespace insystem