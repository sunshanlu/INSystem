#pragma once
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "Common.h"
#include "Viewer.h"
#include "pre_interfaces/msg/nav_state.hpp"
#include "pre_interfaces/msg/rtk.hpp"
#include "pre_interfaces/msg/wodom.hpp"

namespace insystem {

class GINS : public rclcpp::Node {
    using ThreadSharedPtr = std::shared_ptr<std::thread>;
    using NavStateMsg = pre_interfaces::msg::NavState;
    using RTK = pre_interfaces::msg::RTK;
    using IMUMsg = sensor_msgs::msg::Imu;
    using Wodom = pre_interfaces::msg::Wodom;
    using StatePub = rclcpp::Publisher<NavStateMsg>::SharedPtr;
    using RTKSub = rclcpp::Subscription<RTK>::SharedPtr;
    using IMUSub = rclcpp::Subscription<IMUMsg>::SharedPtr;
    using WodomSub = rclcpp::Subscription<Wodom>::SharedPtr;

public:
    GINS(std::string ConfigPath);

    void AddOdom(ODOMSharedPtr odom);

    void AddImu(IMUSharedPtr imu);

    void AddGnss(GnssSharedPtr gnss);

    NavStateSharedPtr GetState();

    /// 保存路径
    void SaveTrajectory(const NavState &navstate);

    ~GINS() {
        viewer_->RequestStop();
        thread_v_->join();
        tra_ifs.close();
    }

private:
    void Optimize();

    void IMUCallback(IMUMsg::SharedPtr imu_msg);

    void RTKCallback(RTK::SharedPtr rtk);

    void OdomCallback(Wodom::SharedPtr odom_msg);

    /// 发布导航状态
    void PublishState(const NavStateSharedPtr &navstate);

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
    Vec3 origin_;                   ///< gnss初始化后，定义地图原点
    ViewerSharedPtr viewer_;        ///< 可视化
    ThreadSharedPtr thread_v_;      ///< 可视化线程指针
    std::ofstream tra_ifs;          ///< 轨迹标准输出流
    StatePub state_pub_;            ///< 状态发布器
    IMUSub imu_sub_;                ///< IMU订阅器
    RTKSub rtk_sub_;                ///< RTK订阅器
    WodomSub odom_sub_;             ///< 里程计订阅器
};

} // namespace insystem