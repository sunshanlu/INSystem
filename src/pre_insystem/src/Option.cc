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
    YAML::Node imu_initer = cf["imu_initer"];
    YAML::Node viewer = cf["viewer"];
    YAML::Node trajectory = cf["trajectory"];

    if (!odom || !imu || !gnss || !g2o_info || !imu_initer || !viewer || !trajectory) {
        throw std::runtime_error("配置文件缺少必要参数");
        exit(-1);
    }

    with_odom_ = odom["with_odom"].as<bool>();
    if (with_odom_) {
        odom_pulse_ = odom["pulse"].as<int>();
        odom_radius_ = odom["radius"].as<double>();
        odom_dt_ = odom["span"].as<double>();
        double odom_var = odom["var"].as<double>();
        double o2 = odom_var * odom_var;
        odom_cov_ << o2, o2, o2;
    }
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
    initer_size_ = imu_initer["max_size"].as<int>();
    initer_time_ = imu_initer["init_time"].as<double>();
    use_viewer_ = viewer["use_viewer"].as<bool>();
    win_name_ = viewer["win_name"].as<std::string>();
    win_width_ = viewer["win_width"].as<int>();
    win_height_ = viewer["win_height"].as<int>();
    traj_size_ = (std::size_t)viewer["traj_size"].as<double>();
    YAML::Node traj_color = viewer["traj_color"];
    YAML::Node coor_x_color = viewer["coor_color"]["x"];
    YAML::Node coor_y_color = viewer["coor_color"]["y"];
    YAML::Node coor_z_color = viewer["coor_color"]["z"];
    traj_color_ << traj_color[0].as<double>(), traj_color[1].as<double>(), traj_color[2].as<double>();
    coor_x_ = viewer["coor_size"]["x"].as<double>();
    coor_y_ = viewer["coor_size"]["y"].as<double>();
    coor_z_ = viewer["coor_size"]["z"].as<double>();
    coor_x_color_ << coor_x_color[0].as<double>(), coor_x_color[1].as<double>(), coor_x_color[2].as<double>();
    coor_y_color_ << coor_y_color[0].as<double>(), coor_y_color[1].as<double>(), coor_y_color[2].as<double>();
    coor_z_color_ << coor_z_color[0].as<double>(), coor_z_color[1].as<double>(), coor_z_color[2].as<double>();
    save_trajectory_ = trajectory["save_trajectory"].as<bool>();
    trajectory_file_ = trajectory["save_path"].as<std::string>();
}

/// Option的静态成员变量
bool Option::with_odom_;                     ///< 是否使用里程计
int Option::odom_pulse_;                     ///< odom的一圈总脉冲数
double Option::odom_radius_;                 ///< odom的半径大小
double Option::odom_dt_;                     ///< odom的采样时间间隔
Vec3 Option::odom_cov_;                      ///< odom的协方差对角向量
double Option::g_norm_;                      ///< 重力加速度模
double Option::bg_cov_;                      ///< imu的角速度偏置方差
double Option::ba_cov_;                      ///< imu的加速度偏置方差
double Option::gnss_antenna_angle_;          ///< gnss天线的安装角度（z轴）
Vec3 Option::gnss_antenna_pos_;              ///< gnss天线的安装位置
Vec6 Option::gcov_;                          ///< gnss协方差对角向量
bool Option::verbose_;                       ///< g2o是否输出优化信息
double Option::prior_cov_;                   ///< 先验协方差
Gnss::CoordType Option::coord_type_;         ///< 坐标系类型
int Option::initer_size_;                    ///< imu初始化器的最大样本数
double Option::initer_time_;                 ///< imu初始化器的初始化时间
bool Option::use_viewer_;                    ///< 是否使用可视化
std::string Option::win_name_;               ///< 窗口名称
int Option::win_width_, Option::win_height_; ///< 窗口大小
std::size_t Option::traj_size_;              ///< 可视化轨迹的尺寸
Vec3f Option::traj_color_;                   ///< 轨迹颜色
double Option::coor_x_;                      ///< 坐标轴x长度
double Option::coor_y_;                      ///< 坐标轴y长度
double Option::coor_z_;                      ///< 坐标轴z长度
Vec3f Option::coor_x_color_;                 ///< 坐标轴x颜色
Vec3f Option::coor_y_color_;                 ///< 坐标轴y颜色
Vec3f Option::coor_z_color_;                 ///< 坐标轴z颜色
bool Option::save_trajectory_;               ///< 是否保存轨迹
std::string Option::trajectory_file_;        ///< 保存轨迹文件名

} // namespace insystem
