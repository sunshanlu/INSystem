#include "IMUIniter.h"
#include "IMU.h"
#include "ODOM.h"

namespace insystem {

/**
 * @brief 添加IMU数据
 * 
 * @param imu       输入的imu数据
 * @return true     初始化完成
 * @return false    初始化还未完成
 */
bool IMUIniter::AddImu(IMUSharedPtr imu) {
    std::lock_guard<std::mutex> lock(mutex_);
    imus_.push_back(imu);
    if (imus_.size() >= maxSize_ || imu->stamp_ - imus_[0]->stamp_ >= delayTime_) {
        ComputeMeanAndCov();
        return true;
    }
    return false;
}

/**
 * @brief 添加里程计数据
 * @details
 *      如果imu状态为非静态，清空imu的数据缓存
 * @param odom 输入的里程计数据
 */
void IMUIniter::AddOdom(ODOMSharedPtr odom) {
    if (odom->v_ != 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        imus_.clear();
    }
}

/**
 * @brief 计算IMU的均值和协方差
 * 
 */
void IMUIniter::ComputeMeanAndCov() {
    decltype(imus_) imus;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        imus = imus_;
    }
    std::size_t n = imus.size();

    Vec3 ma = Vec3::Zero(), mg = Vec3::Zero();
    for (const auto &imu : imus) {
        ma += imu->acc_;
        mg += imu->gyr_;
    }
    ma /= n;
    mg /= n;
    g_ = -(ma / ma.norm() * gNorm_);
    ba_ = ma + g_;
    bg_ = mg;
    for (const auto &imu:imus_){
        Vec3 accCovi = imu->acc_ - ba_ + g_;
        Vec3 gyrCovi = imu->gyr_ - bg_;
        accCov_ += accCovi.cwiseProduct(accCovi);
        gyrCov_ += gyrCovi.cwiseProduct(gyrCovi);
    }
    accCov_ /= n;
    gyrCov_ /= n;
}

} // namespace insystem
