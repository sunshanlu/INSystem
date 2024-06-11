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

namespace insystem {

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
        if (init_imu_)
            imu_preint_->SetIMUInfo(imu_initer_);
    } else if (init_gnss_ && init_imu_)
        imu_preint_->Integrate(imu, imu->stamp_ - lastStamp_);

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
    lastStamp_ = gnss->stamp_;
    curr_gnss_ = gnss;

    if (!init_gnss_) {
        if (!gnss->is_valid_)
            return;
        last_gnss_ = gnss;
        last_frame_ = std::make_shared<NavState>();
        last_frame_->p_ = gnss->p_;
        last_frame_->v_.setZero();
        last_frame_->ba_ = imu_initer_->GetBiasA();
        last_frame_->bg_ = imu_initer_->GetBiasG();
        last_frame_->stamp_ = gnss->stamp_;
        init_gnss_ = true;
    } else if (init_gnss_ && init_imu_) {
        curr_frame_ = std::make_shared<NavState>();
        curr_frame_->stamp_ = gnss->stamp_;
        imu_preint_->Integrate(last_imu_, lastStamp_ - last_imu_->stamp_);
        imu_preint_->Predict(*last_frame_, *curr_frame_, imu_initer_->GetGravity());
        Optimize();
        imu_preint_->Reset();
        last_frame_ = curr_frame_;
        last_gnss_ = curr_gnss_;
    }
}

/**
 * @brief 添加odom数据
 *
 * @param odom
 */
void GINS::AddOdom(ODOMSharedPtr odom) {
    last_odom_ = odom;
    update_odom_ = true;
    if (!init_imu_)
        imu_initer_->AddOdom(odom);
}

void GINS::Optimize() {
    if (imu_preint_->GetDt() < 1e3)
        return;

    g2o::SparseOptimizer optimizer;
    auto *lm = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolver>(g2o::make_unique<LinearSolver>()));
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

    /// todo ：信息矩阵写入config
    ba_e->setInformation(1e8 * Mat3::Identity());
    optimizer.addEdge(ba_e);

    auto *bg_e = new BiasGEdge();
    bg_e->setId(2);
    bg_e->setVertex(0, bgi);
    bg_e->setVertex(1, bgj);
    bg_e->setInformation(1e12 * Mat3::Identity());
    optimizer.addEdge(bg_e);

    /// 3. i时刻先验边
    auto *prior_e = new PriorEdge(last_frame_);
    prior_e->setId(3);
    prior_e->setVertex(0, pi);
    prior_e->setVertex(1, vi);
    prior_e->setVertex(2, bgi);
    prior_e->setVertex(3, bai);

    /// todo ：信息矩阵写入config
    prior_e->setInformation(Mat15::Identity() * 1e2);
    optimizer.addEdge(prior_e);

    /// 4. GNSS约束边
    /// todo ：信息矩阵写入config
    static double gnss_pos_var = 0.1;
    static double gnss_hei_var = 0.1;
    static double gnss_ang_var = M_PI / 180.0;
    static double gp2 = gnss_pos_var * gnss_pos_var;
    static double gh2 = gnss_hei_var * gnss_hei_var;
    static double ga2 = gnss_ang_var * gnss_ang_var;

    Vec6 gcov;
    gcov << ga2, ga2, ga2, gp2, gp2, gh2;
    auto *gnss_ei = new GnssEdge(last_gnss_->p_, gcov, 1.0);
    gnss_ei->setId(4);
    gnss_ei->setVertex(0, pi);
    optimizer.addEdge(gnss_ei);

    auto *gnss_ej = new GnssEdge(curr_frame_->p_, gcov, 1.0);
    gnss_ej->setId(5);
    gnss_ej->setVertex(0, pj);
    optimizer.addEdge(gnss_ej);

    /// 5. 速度约束边
    /// todo ：信息矩阵写入config
    static double odom_var = 0.05;
    static double o2 = odom_var * odom_var;
    Vec3 ocov;
    ocov << o2, o2, o2;
    if (update_odom_) {
        update_odom_ = false;
        auto o_e = new OdomEdge(last_odom_->v_, ocov, 1.0, pj);
        o_e->setId(6);
        o_e->setVertex(0, vj);
        optimizer.addEdge(o_e);
    }
    optimizer.initializeOptimization(0);
    optimizer.optimize(20);
}

} // namespace insystem