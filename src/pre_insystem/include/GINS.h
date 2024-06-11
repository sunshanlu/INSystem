#pragma
#include <shared_mutex>

#include "common/common.h"

namespace insystem {

class GINS {
public:
    void AddOdom(ODOMSharedPtr odom);

    void AddImu(IMUSharedPtr imu);

    void AddGnss(GnssSharedPtr gnss);

private:
    void Optimize();

    bool init_imu_;                 ///< 是否完成IMU的静态初始化
    bool init_gnss_;                ///< 是否有第一个有效的GNSS数据
    bool update_odom_;              ///< 是否有新的odom数据
    ODOMSharedPtr last_odom_;       ///< 上一时刻的odom数据
    IMUIniterSharedPtr imu_initer_; ///< IMU初始化器
    IMUPreintSharedPtr imu_preint_; ///< IMU预积分器
    IMUSharedPtr last_imu_;         ///< 针对gnss处理的最新一条imu
    double lastStamp_;              ///< 上一次获取imu或者gnss的时间戳
    NavStateSharedPtr curr_frame_;  ///< j时刻导航状态
    NavStateSharedPtr last_frame_;  ///< i时刻导航状态
    GnssSharedPtr curr_gnss_;       ///< j时刻GNSS信息
    GnssSharedPtr last_gnss_;       ///< i时刻GNSS信息
};

} // namespace insystem