#pragma once

#include <string>

#include "Gnss.h"
#include "common/common.h"

namespace insystem {

struct Option {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Option(std::string fp);

    static int odom_pulse_;             ///< odom的一圈总脉冲数
    static double odom_radius_;         ///< odom的半径大小
    static double odom_dt_;             ///< odom的采样时间间隔
    static Vec3 odom_cov_;              ///< odom的协方差对角向量
    static double g_norm_;              ///< 重力加速度模
    static double bg_cov_;              ///< imu的角速度偏置方差
    static double ba_cov_;              ///< imu的加速度偏置方差
    static double gnss_antenna_angle_;  ///< gnss天线的安装角度（z轴）
    static Vec3 gnss_antenna_pos_;      ///< gnss天线的安装位置
    static Vec6 gcov_;                  ///< gnss协方差对角向量
    static bool verbose_;               ///< g2o是否输出优化信息
    static double prior_cov_;           ///< 先验协方差
    static Gnss::CoordType coord_type_; ///< 坐标系类型
};

} // namespace insystem