#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>

#include "G2oTypes.hpp"
#include "GINS.h"
#include "Gnss.h"
#include "IMU.h"
#include "IMUIniter.h"
#include "IMUPreint.h"
#include "NavState.h"
#include "ODOM.h"
#include "Option.h"
#include "Viewer.h"

namespace insystem {

using namespace std::placeholders;

GINS::GINS(std::string ConfigPath)
    : Node("GINSystem")
    , init_imu_(false)
    , init_gnss_(false)
    , update_odom_(false)
    , last_odom_(nullptr)
    , imu_initer_(nullptr)
    , imu_preint_(nullptr)
    , last_imu_(nullptr)
    , lastStamp_(0)
    , curr_frame_(nullptr)
    , last_frame_(nullptr)
    , curr_gnss_(nullptr)
    , last_gnss_(nullptr)
    , origin_(Vec3::Zero()) {

    declare_parameter("ConfigPath", "");
    std::string rosConfigPath = get_parameter("ConfigPath").as_string();
    
    if (!rosConfigPath.empty())
        ConfigPath = rosConfigPath;
    Option option(ConfigPath);

    imu_initer_ = std::make_shared<IMUIniter>(Option::initer_size_, Option::initer_time_, Option::g_norm_);
    imu_preint_ = std::make_shared<IMUPreint>();
    if (Option::use_viewer_) {
        viewer_ =
            std::make_shared<Viewer>(Option::win_name_, Option::win_width_, Option::win_height_, Option::traj_size_,
                                     Option::traj_color_, Option::coor_x_color_, Option::coor_y_color_,
                                     Option::coor_z_color_, Option::coor_x_, Option::coor_y_, Option::coor_z_);
        thread_v_ = std::make_shared<std::thread>(&Viewer::Render, viewer_);
    }
    if (Option::save_trajectory_) {
        tra_ifs.open(Option::trajectory_file_);
        tra_ifs << "x y z qx qy qz qw" << std::endl;
    }
    state_pub_ = create_publisher<NavStateMsg>("pre_insystem/navstate", 10);
    imu_sub_ = create_subscription<IMUMsg>("pre_insystem/imu", 10, std::bind(&GINS::IMUCallback, this, _1));
    rtk_sub_ = create_subscription<RTK>("pre_insystem/rtk", 10, std::bind(&GINS::RTKCallback, this, _1));
    if (Option::with_odom_)
        odom_sub_ = create_subscription<Wodom>("pre_insystem/odom", 10, std::bind(&GINS::OdomCallback, this, _1));
}

/**
 * @brief 添加IMU数据，进行预积分
 * @details
 *      1. 完成imu数据的初始化任务
 *      2. 完成imu数据的预积分任务(从i到最新imu数据时间)
 * @param imu 输入的imu数据
 */
void GINS::AddImu(IMUSharedPtr imu) {
    if (!init_imu_) {
        init_imu_ = imu_initer_->AddImu(imu);
        if (init_imu_){
            imu_preint_->SetIMUInfo(imu_initer_);
            RCLCPP_INFO(get_logger(), "imu初始化成功！");
        }
    } else if (init_gnss_ && init_imu_) {
        imu_preint_->Integrate(imu, imu->stamp_ - lastStamp_);
        auto state = GetState();
        if (Option::use_viewer_)
            viewer_->UpdateNavState(state);
        if (Option::save_trajectory_)
            SaveTrajectory(*state);
        PublishState(state);
    }

    lastStamp_ = imu->stamp_;
    last_imu_ = imu;
}

/**
 * @brief 添加Gnss信息
 * @details
 *      1. 完成gnss数据的初始化任务
 *      2. 完成imu数据的预积分任务(从最新的imu数据时间到j)
 *      3. 完成优化任务，并重置预积分状态
 * @param gnss  输入的Gnss数据
 */
void GINS::AddGnss(GnssSharedPtr gnss) {
    if (!gnss->convert_success_)
        return;
    lastStamp_ = gnss->stamp_;
    curr_gnss_ = gnss;

    if (!init_gnss_) {
        if (!gnss->is_valid_)
            return;
        last_gnss_ = gnss;
        origin_ = gnss->p_.translation();
        last_frame_ = std::make_shared<NavState>();
        last_frame_->p_ = gnss->GetBasePose(Option::gnss_antenna_angle_, Option::gnss_antenna_pos_, origin_);
        last_frame_->v_.setZero();
        last_frame_->ba_ = imu_initer_->GetBiasA();
        last_frame_->bg_ = imu_initer_->GetBiasG();
        last_frame_->stamp_ = gnss->stamp_;
        curr_frame_ = last_frame_;
        init_gnss_ = true;
        RCLCPP_INFO(get_logger(), "RTK初始化成功!");
    } else if (init_gnss_ && init_imu_) {
        curr_frame_ = std::make_shared<NavState>();
        curr_frame_->stamp_ = gnss->stamp_;
        imu_preint_->Integrate(last_imu_, lastStamp_ - last_imu_->stamp_);
        imu_preint_->Predict(*last_frame_, *curr_frame_, imu_initer_->GetGravity());
        Optimize();
        if (Option::use_viewer_)
            viewer_->UpdateNavState(curr_frame_);

        if (Option::save_trajectory_)
            SaveTrajectory(*curr_frame_);
        PublishState(curr_frame_);

        imu_preint_->Reset();
        last_frame_ = curr_frame_;
        last_gnss_ = curr_gnss_;
    }
}

/**
 * @brief 添加odom数据
 *
 * @param odom 输入的odom数据
 */
void GINS::AddOdom(ODOMSharedPtr odom) {
    last_odom_ = odom;
    update_odom_ = true;
    if (!init_imu_)
        imu_initer_->AddOdom(odom);
}

void GINS::Optimize() {
    if (imu_preint_->GetDt() < 1e-3)
        return;

    g2o::SparseOptimizer optimizer;
    auto *lm = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<BlockSolver>(std::make_unique<LinearSolver>()));
    optimizer.setAlgorithm(lm);

    /// i时刻的状态pi, vi, bgi, bai
    auto *pi = new PoseVertex();
    pi->setId(0);
    pi->setEstimate(last_frame_->p_);
    optimizer.addVertex(pi);

    auto *vi = new VelocityVertex();
    vi->setId(1);
    vi->setEstimate(last_frame_->v_);
    optimizer.addVertex(vi);

    auto *bgi = new BiasGVertex();
    bgi->setId(2);
    bgi->setEstimate(last_frame_->bg_);
    optimizer.addVertex(bgi);

    auto *bai = new BiasAVertex();
    bai->setId(3);
    bai->setEstimate(last_frame_->ba_);
    optimizer.addVertex(bai);

    /// j时刻的状态pj, vj, bgj, baj
    auto *pj = new PoseVertex();
    pj->setId(4);
    pj->setEstimate(curr_frame_->p_);
    optimizer.addVertex(pj);

    auto *vj = new VelocityVertex();
    vj->setId(5);
    vj->setEstimate(curr_frame_->v_);
    optimizer.addVertex(vj);

    auto *bgj = new BiasGVertex();
    bgj->setId(6);
    bgj->setEstimate(curr_frame_->bg_);
    optimizer.addVertex(bgj);

    auto *baj = new BiasAVertex();
    baj->setId(7);
    baj->setEstimate(curr_frame_->ba_);
    optimizer.addVertex(baj);

    /// 1. 预积分边
    auto *preint_e = new PreIntEdge(imu_preint_, imu_initer_->GetGravity(), 1.0);
    auto *rk = new g2o::RobustKernelHuber();
    rk->setDelta(200.0);
    preint_e->setId(0);
    preint_e->setVertex(0, pi);
    preint_e->setVertex(1, vi);
    preint_e->setVertex(2, bgi);
    preint_e->setVertex(3, bai);
    preint_e->setVertex(4, pj);
    preint_e->setVertex(5, vj);
    preint_e->setRobustKernel(rk);
    optimizer.addEdge(preint_e);

    /// 2. 零偏随机游走边
    auto *ba_e = new BiasAEdge();
    ba_e->setId(1);
    ba_e->setVertex(0, bai);
    ba_e->setVertex(1, baj);
    ba_e->setInformation(1.0 / Option::ba_cov_ * Mat3::Identity());
    optimizer.addEdge(ba_e);

    auto *bg_e = new BiasGEdge();
    bg_e->setId(2);
    bg_e->setVertex(0, bgi);
    bg_e->setVertex(1, bgj);
    bg_e->setInformation(1.0 / Option::bg_cov_ * Mat3::Identity());
    optimizer.addEdge(bg_e);

    /// 3. i时刻先验边
    auto *prior_e = new PriorEdge(last_frame_);
    prior_e->setId(3);
    prior_e->setVertex(0, pi);
    prior_e->setVertex(1, vi);
    prior_e->setVertex(2, bgi);
    prior_e->setVertex(3, bai);
    prior_e->setInformation(Mat15::Identity() / Option::prior_cov_);
    optimizer.addEdge(prior_e);

    /// 4. GNSS约束边
    auto *gnss_ei = new GnssEdge(
        last_gnss_->GetBasePose(Option::gnss_antenna_angle_, Option::gnss_antenna_pos_, origin_), Option::gcov_, 1.0);
    gnss_ei->setId(4);
    gnss_ei->setVertex(0, pi);
    optimizer.addEdge(gnss_ei);

    auto *gnss_ej = new GnssEdge(
        curr_gnss_->GetBasePose(Option::gnss_antenna_angle_, Option::gnss_antenna_pos_, origin_), Option::gcov_, 1.0);
    gnss_ej->setId(5);
    gnss_ej->setVertex(0, pj);
    optimizer.addEdge(gnss_ej);

    /// 5. 速度约束边
    OdomEdge *o_e;
    if (update_odom_) {
        update_odom_ = false;
        o_e = new OdomEdge(last_odom_->GetVelocity(Option::odom_pulse_, Option::odom_radius_, Option::odom_dt_),
                           Option::odom_cov_, 1.0, pj);
        o_e->setId(6);
        o_e->setVertex(0, vj);
        optimizer.addEdge(o_e);
    }
    optimizer.setVerbose(Option::verbose_);
    preint_e->computeError();
    ba_e->computeError();
    bg_e->computeError();
    prior_e->computeError();
    gnss_ei->computeError();
    gnss_ej->computeError();

    optimizer.initializeOptimization(0);
    optimizer.optimize(20);

    curr_frame_->p_ = pj->estimate();
    curr_frame_->v_ = vj->estimate();
    curr_frame_->ba_ = baj->estimate();
    curr_frame_->bg_ = bgj->estimate();
}

/**
 * @brief 获取gin_system系统的导航状态
 * @details
 *      1. 保证curr_frame_不为空
 *      2. 保证imu_preint_不为空
 * @return NavStateSharedPtr 输出的导航状态
 */
NavStateSharedPtr GINS::GetState() {
    auto state = std::make_shared<NavState>();
    imu_preint_->Predict(*curr_frame_, *state, imu_initer_->GetGravity());
    return state;
}

/**
 * @brief 将指定导航状态保存到文件中
 *
 * @param navstate
 */
void GINS::SaveTrajectory(const NavState &navstate) {
    auto q = navstate.p_.unit_quaternion();
    auto t = navstate.p_.translation();
    tra_ifs << t.x() << " " << t.y() << " " << t.z() << " ";
    tra_ifs << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
}

/**
 * @brief imu回调
 *
 * @param imu_msg
 */
void GINS::IMUCallback(IMUMsg::SharedPtr imu_msg) {
    IMUSharedPtr imu = std::make_shared<IMU>(imu_msg);
    AddImu(imu);
}

/**
 * @brief rtk回调
 *
 * @param rtk
 */
void GINS::RTKCallback(RTK::SharedPtr rtk) {
    GnssSharedPtr gnss = std::make_shared<Gnss>(rtk, Option::coord_type_);
    AddGnss(gnss);
}

/**
 * @brief odom回调
 *
 * @param odom_msg
 */
void GINS::OdomCallback(Wodom::SharedPtr odom_msg) {
    ODOMSharedPtr odom = std::make_shared<ODOM>(odom_msg);
    AddOdom(odom);
}

/**
 * @brief 发布状态
 *
 * @param navstate 输入的状态信息
 */
void GINS::PublishState(const NavStateSharedPtr &navstate) {
    const auto &q = navstate->p_.unit_quaternion();
    const auto &p = navstate->p_.translation();

    NavStateMsg msg;
    msg.header.stamp = now();
    msg.header.frame_id = "GINS";
    msg.pose.position.x = p[0];
    msg.pose.position.y = p[1];
    msg.pose.position.z = p[2];
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    msg.velocity.x = navstate->v_.x();
    msg.velocity.y = navstate->v_.y();
    msg.velocity.z = navstate->v_.z();
    state_pub_->publish(msg);
}

} // namespace insystem