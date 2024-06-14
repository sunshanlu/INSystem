#pragma once

#include <string>

#include "Common.h"
#include "Gnss.h"

namespace insystem {

struct Option {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Option(std::string fp);

    static bool with_odom_;              ///< 是否使用odom
    static int odom_pulse_;              ///< odom的一圈总脉冲数
    static double odom_radius_;          ///< odom的半径大小
    static double odom_dt_;              ///< odom的采样时间间隔
    static Vec3 odom_cov_;               ///< odom的协方差对角向量
    static double g_norm_;               ///< 重力加速度模
    static double bg_cov_;               ///< imu的角速度偏置方差
    static double ba_cov_;               ///< imu的加速度偏置方差
    static double gnss_antenna_angle_;   ///< gnss天线的安装角度（z轴）
    static Vec3 gnss_antenna_pos_;       ///< gnss天线的安装位置
    static Vec6 gcov_;                   ///< gnss协方差对角向量
    static bool verbose_;                ///< g2o是否输出优化信息
    static double prior_cov_;            ///< 先验协方差
    static Gnss::CoordType coord_type_;  ///< 坐标系类型
    static int initer_size_;             ///< imu初始化器的最大样本数
    static double initer_time_;          ///< imu初始化器的初始化时间
    static bool use_viewer_;             ///< 是否使用可视化
    static std::string win_name_;        ///< 窗口名称
    static int win_width_, win_height_;  ///< 窗口大小
    static std::size_t traj_size_;       ///< 可视化轨迹的尺寸
    static Vec3f traj_color_;            ///< 轨迹颜色
    static double coor_x_;               ///< 坐标轴x长度
    static double coor_y_;               ///< 坐标轴y长度
    static double coor_z_;               ///< 坐标轴z长度
    static Vec3f coor_x_color_;          ///< 坐标轴x颜色
    static Vec3f coor_y_color_;          ///< 坐标轴y颜色
    static Vec3f coor_z_color_;          ///< 坐标轴z颜色
    static bool save_trajectory_;        ///< 是否保存轨迹
    static std::string trajectory_file_; ///< 保存轨迹文件名
};

} // namespace insystem