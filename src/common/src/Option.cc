#include <yaml-cpp/yaml.h>

#include "Option.h"

namespace insystem {

/**
 * @brief Option类的构造函数
 *
 * @param fp 输入的配置文件路径
 */
Option::Option(std::string fp) {
    YAML::Node cf = YAML::LoadFile(fp);
    if (!cf.IsDefined()) {
        throw std::runtime_error("配置文件的路径错误");
        exit(-1);
    }
    YAML::Node odom = cf["odom"];
    YAML::Node imu = cf["imu"];
    YAML::Node gnss = cf["gnss"];
    YAML::Node g2o_info = cf["g2o"];

    if (!odom || !imu || !gnss || !g2o_info) {
        throw std::runtime_error("配置文件缺少必要参数");
        exit(-1);
    }

    odom_pulse_ = odom["pulse"].as<int>();
    odom_radius_ = odom["radius"].as<double>();
    odom_dt_ = odom["span"].as<double>();
    double odom_var = odom["var"].as<double>();
    double o2 = odom_var * odom_var;
    odom_cov_ << o2, o2, o2;
    g_norm_ = imu["g_norm"].as<double>();
    double bg_var = imu["bg_var"].as<double>();
    double ba_var = imu["ba_var"].as<double>();
    bg_cov_ = bg_var * bg_var;
    ba_cov_ = ba_var * ba_var;
    gnss_antenna_angle_ = gnss["ant_angle"].as<double>() * M_PI / 180.0;
    gnss_antenna_pos_ << gnss["ant_x"].as<double>(), gnss["ant_y"].as<double>(), gnss["ant_z"].as<double>();
    double gp_var = gnss["pos_var"].as<double>();
    double gh_var = gnss["alt_var"].as<double>();
    double ga_var = gnss["ang_var"].as<double>() * M_PI / 180.0;
    double gp2 = gp_var * gp_var;
    double gh2 = gh_var * gh_var;
    double ga2 = ga_var * ga_var;
    gcov_ << ga2, ga2, ga2, gp2, gp2, gh2;
    int t = gnss["cor_type"].as<int>();
    switch (t) {
    case 0:
        coord_type_ = Gnss::CoordType::EAST_NORTH;
        break;

    case 1:
        coord_type_ = Gnss::CoordType::NORTH_EAST;
        break;

    default:
        throw std::runtime_error("Invalid gnss coordinate type");
    }
    verbose_ = g2o_info["verbose"].as<bool>();
    double prior_var = g2o_info["prior_var"].as<double>();
    prior_cov_ = prior_var * prior_var;
}

/// Option的静态成员变量
int Option::odom_pulse_;             ///< odom的一圈总脉冲数
double Option::odom_radius_;         ///< odom的半径大小
double Option::odom_dt_;             ///< odom的采样时间间隔
Vec3 Option::odom_cov_;              ///< odom的协方差对角向量
double Option::g_norm_;              ///< 重力加速度模
double Option::bg_cov_;              ///< imu的角速度偏置方差
double Option::ba_cov_;              ///< imu的加速度偏置方差
double Option::gnss_antenna_angle_;  ///< gnss天线的安装角度（z轴）
Vec3 Option::gnss_antenna_pos_;      ///< gnss天线的安装位置
Vec6 Option::gcov_;                  ///< gnss协方差对角向量
bool Option::verbose_;               ///< g2o是否输出优化信息
double Option::prior_cov_;           ///< 先验协方差
Gnss::CoordType Option::coord_type_; ///< 坐标系类型

} // namespace insystem
