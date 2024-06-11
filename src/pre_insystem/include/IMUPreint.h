#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>

#include "IMU.h"
#include "common/common.h"

namespace insystem {
class IMUPreint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IMUPreint();

    /// 预积分
    void Integrate(const IMUSharedPtr &imu, const double &dt);

    /// 基于偏置获取更新状态
    SO3 GetDeltaR(const Vec3 &bg) const { return dR_ * SO3::exp(dR_dbg * (bg - bg_)); }
    Vec3 GetDeltaV(const Vec3 &ba, const Vec3 &bg) const { return dv_ + dv_dba * (ba - ba_) + dv_dbg * (bg - bg_); }
    Vec3 GetDeltaP(const Vec3 &ba, const Vec3 &bg) const { return dp_ + dp_dba * (ba - ba_) + dp_dbg * (bg - bg_); }

    const Mat3 &GetDrDbg() const { return dR_dbg; }

    const Mat3 &GetDvDba() const { return dv_dba; }

    const Mat3 &GetDvDbg() const { return dv_dbg; }

    const Mat3 &GetDpDba() const { return dp_dba; }

    const Mat3 &GetDpDbg() const { return dp_dbg; }

    const Vec3 &GetOriBg() const { return bg_; }

    const Vec3 &GetOriBa() const { return ba_; }

    /**
     * @brief 预测j时刻的旋转矩阵
     *
     * @param Ri    输入的i时刻的旋转矩阵
     * @return SO3  预测j时刻的旋转矩阵
     */
    SO3 PredictRj(const SO3 &Ri) { return Ri * dR_; }

    /**
     * @brief 预测j时刻的速度
     *
     * @param vi    输入的i时刻的速度
     * @param Ri    输入的i时刻的旋转矩阵
     * @param g     输入的重力加速度
     * @return Vec3 输出的j时刻的速度
     */
    Vec3 PredictVj(const Vec3 &vi, const SO3 &Ri, const Vec3 &g) { return Ri * dv_ + vi + g * dt_; }

    /**
     * @brief 预测j时刻的位置
     *
     * @param pi    输入的i时刻的位置
     * @param Ri    输入的i时刻的旋转矩阵
     * @param g     输入的重力加速度
     * @param vi    输入的i时刻的速度
     * @return Vec3 输出的j时刻的位置
     */
    Vec3 PredictPj(const Vec3 &pi, const SO3 &Ri, const Vec3 &g, const Vec3 &vi) {
        return Ri * dp_ + pi + vi * dt_ + 0.5 * g * dt_ * dt_;
    }

    /// 预测函数
    void Predict(const NavState &start, NavState &end, const Vec3 &g);

    /// 获取预积分模型的协方差矩阵
    Mat9 GetCov() { return cov_; }

    /// 获取积分时间
    const double &GetDt() const { return dt_; }

    /// 重置预积分状态
    void Reset() {
        dR_ = SO3();
        dv_ = Vec3::Zero();
        dp_ = Vec3::Zero();
        dR_dbg = Mat3::Zero();
        dv_dba = Mat3::Zero();
        dv_dbg = Mat3::Zero();
        dp_dba = Mat3::Zero();
        dp_dbg = Mat3::Zero();
        cov_ = Mat9::Zero();
        dt_ = 0.0;
    }
    
    /// 设置有关imu数据的信息（bg_, ba_, covEta_）
    void SetIMUInfo(IMUIniterSharedPtr imu_initer);

private:
    /// 预积分模型状态
    SO3 dR_;
    Vec3 dv_;
    Vec3 dp_;
    Mat3 dR_dbg;  ///< dR/dbg
    Mat3 dv_dba;  ///< dv/dba
    Mat3 dv_dbg;  ///< dv/dbg
    Mat3 dp_dba;  ///< dp/dba
    Mat3 dp_dbg;  ///< dp/dbg
    Mat9 cov_;    ///< 状态估计的协方差矩阵
    Vec3 bg_;     ///< 固定的陀螺仪偏置
    Vec3 ba_;     ///< 固定的加速度偏置
    Mat6 covEta_; ///< IMU的测量噪声协方差矩阵
    double dt_;   ///< 积分时间
};

} // namespace insystem
