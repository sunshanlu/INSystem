#pragma once
#include <mutex>

#include <Eigen/Core>

#include "common/common.h"

namespace insystem {
class IMUIniter {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IMUIniter(unsigned maxSize, double delayTime, double gNorm = 9.81)
        : maxSize_(std::move(maxSize))
        , delayTime_(std::move(delayTime))
        , accCov_(Vec3::Zero())
        , gyrCov_(Vec3::Zero())
        , gNorm_(std::move(gNorm)) {}

    bool AddImu(IMUSharedPtr imu);

    void AddOdom(ODOMSharedPtr odom);

    const Vec3 &GetBiasA() const { return ba_; }

    const Vec3 &GetBiasG() const { return bg_; }

    const Vec3 &GetGravity() const { return g_; }

    const Vec3 &GetCovA() const { return accCov_; }

    const Vec3 &GetCovG() const { return gyrCov_; }

private:
    /// @brief 计算imu数据的均值和协方差
    void ComputeMeanAndCov();

    mutable std::mutex mutex_;       ///< 维护imus_的互斥锁
    std::vector<IMUSharedPtr> imus_; ///< 存储imu数据的容器
    unsigned maxSize_;               ///< 存储imu数据的容器的最大长度
    double delayTime_;               ///< 静态初始化时间
    Vec3 accCov_;                    ///< 加速度协方差
    Vec3 gyrCov_;                    ///< 角速度协方差
    Vec3 ba_;                        ///< 加速度偏置
    Vec3 bg_;                        ///< 角速度偏置
    Vec3 g_;                         ///< 重力加速度
    double gNorm_;                   ///< 重力加速度模长
};
} // namespace insystem