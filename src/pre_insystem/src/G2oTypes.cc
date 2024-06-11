#include "G2oTypes.hpp"
#include "IMUPreint.h"
#include "NavState.h"

namespace insystem {

/**
 * @brief PoseVertex的更新函数
 * @details
 *      1. 这里定义了PoseVertex的变量顺序为R、t
 * @param update
 */
void PoseVertex::oplusImpl(const double *update) {
    /// Eigen::Map函数是按行映射的，可以指定步长和内存对齐
    Vec3 Rv = Eigen::Map<const Vec3>(&update[0]);
    Vec3 t = Eigen::Map<const Vec3>(&update[3]);
    _estimate.so3() *= SO3::exp(Rv);
    _estimate.translation() += t;
}

/// 速度的更新函数
void VelocityVertex::oplusImpl(const double *update) { _estimate += Eigen::Map<const Vec3>(update); }

/**
 * @brief 预积分优化边的构造函数
 * @details
 *      1. 规定多元边的顶点关联顶点数目
 *      2. 根据预积分对象的噪声协方差矩阵，设置信息矩阵
 * @param preintPtr     输入的预积分对象指针
 * @param g             输入的重力，用于计算残差
 * @param weight        输入的权重，用于调整信息矩阵
 */
PreIntEdge::PreIntEdge(IMUPreintSharedPtr &preintPtr, const Vec3 &g, double weight)
    : preintPtr_(preintPtr)
    , dt_(preintPtr_->GetDt())
    , g_(g) {
    setInformation(preintPtr_->GetCov().inverse() * weight);
    resize(6);
}

/**
 * @brief 计算残差
 * @details
 *      1. 规定，顶点的顺序为: p1, v1, bg1, ba1, p2, v2
 *      2. 规定残差的顺序为：eR, ev, ep
 */
void PreIntEdge::computeError() {
    auto p1 = dynamic_cast<PoseVertex *>(_vertices[0]);
    auto v1 = dynamic_cast<VelocityVertex *>(_vertices[1]);
    auto bg1 = dynamic_cast<BiasGVertex *>(_vertices[2]);
    auto ba1 = dynamic_cast<BiasAVertex *>(_vertices[3]);
    auto p2 = dynamic_cast<PoseVertex *>(_vertices[4]);
    auto v2 = dynamic_cast<VelocityVertex *>(_vertices[5]);

    Vec3 bg = bg1->estimate();
    Vec3 ba = ba1->estimate();
    SO3 RiT = p1->estimate().so3().inverse();
    SO3 Rj = p2->estimate().so3();
    Vec3 pi = p1->estimate().translation();
    Vec3 pj = p2->estimate().translation();
    Vec3 vi = v1->estimate();
    Vec3 vj = v2->estimate();
    Vec3 dp = preintPtr_->GetDeltaP(ba, bg);
    Vec3 dv = preintPtr_->GetDeltaV(ba, bg);
    SO3 dR = preintPtr_->GetDeltaR(bg);

    eR_ = dR * (RiT * Rj).log();
    ev_ = RiT * (vj - vi - g_ * dt_) - dv;
    ep_ = RiT * (pj - pi - vi * dt_ - 0.5 * g_ * dt_ * dt_) - dp;

    _error << eR_, ev_, ep_;
}

/**
 * @brief 计算优化的雅可比矩阵
 * @details
 *      1. 注意，由于有6个顶点，因此有6个雅可比矩阵
 *      2. 残差为行，顶点中的变量顺序为列
 */
void PreIntEdge::linearizeOplus() {
    auto p1 = dynamic_cast<PoseVertex *>(_vertices[0]);
    auto v1 = dynamic_cast<VelocityVertex *>(_vertices[1]);
    auto bg1 = dynamic_cast<BiasGVertex *>(_vertices[2]);
    auto ba1 = dynamic_cast<BiasAVertex *>(_vertices[3]);
    auto p2 = dynamic_cast<PoseVertex *>(_vertices[4]);
    auto v2 = dynamic_cast<VelocityVertex *>(_vertices[5]);
    Mat3 Ri = p1->estimate().rotationMatrix();
    Mat3 Rj = p2->estimate().rotationMatrix();
    Mat3 RiT = p1->estimate().rotationMatrix().transpose();
    Mat3 jrInv = SO3::leftJacobianInverse(-eR_);
    Vec3 vi = v1->estimate();
    Vec3 vj = v2->estimate();
    Vec3 pi = p1->estimate().translation();
    Vec3 pj = p2->estimate().translation();
    auto &dR_dbg = preintPtr_->GetDrDbg();
    auto &dv_dba = preintPtr_->GetDvDba();
    auto &dv_dbg = preintPtr_->GetDvDbg();
    auto &dp_dba = preintPtr_->GetDpDba();
    auto &dp_dbg = preintPtr_->GetDpDbg();
    Vec3 dbg = bg1->estimate() - preintPtr_->GetOriBg();
    Mat3 jrb = SO3::leftJacobian(-dR_dbg * dbg);

    /// 1. 针对pi的雅可比矩阵
    _jacobianOplus[0].setZero();
    _jacobianOplus[0].block<3, 3>(0, 0) = -jrInv * Rj.transpose() * Ri;
    _jacobianOplus[0].block<3, 3>(3, 0) = SO3::hat(RiT * (vj - vi - g_ * dt_));
    _jacobianOplus[0].block<3, 3>(6, 0) = SO3::hat(RiT * (pj - pi - vi * dt_ - 0.5 * g_ * dt_ * dt_));
    _jacobianOplus[0].block<3, 3>(6, 3) = -RiT;

    /// 2. 针对vi的雅可比矩阵
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3, 3>(3, 0) = -RiT;
    _jacobianOplus[1].block<3, 3>(6, 0) = -RiT * dt_;

    /// 3. 针对bgi的雅可比矩阵
    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3, 3>(0, 0) = -jrInv * SO3::exp(eR_).inverse().matrix() * jrb * dR_dbg;
    _jacobianOplus[2].block<3, 3>(3, 0) = -dv_dbg;
    _jacobianOplus[2].block<3, 3>(6, 0) = -dp_dbg;

    /// 4. 针对bai的雅可比矩阵
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3, 3>(3, 0) = -dv_dba;
    _jacobianOplus[3].block<3, 3>(6, 0) = -dp_dba;

    /// 5. 针对pj的雅可比矩阵
    _jacobianOplus[4].setZero();
    _jacobianOplus[4].block<3, 3>(0, 0) = jrInv;
    _jacobianOplus[4].block<3, 3>(6, 3) = RiT;

    /// 6. 针对vj的雅可比矩阵
    _jacobianOplus[5].setZero();
    _jacobianOplus[5].block<3, 3>(3, 0) = RiT;
}

/**
 * @brief 计算先验信息残差
 * @details
 *      1. 定义节点顺序为：pi, vi, bgi, bai
 *      2. 定义误差顺序：eR, ep, ev, ebg, eba
 */
void PriorEdge::computeError() {
    auto vpi = dynamic_cast<PoseVertex *>(_vertices[0]);
    auto vvi = dynamic_cast<VelocityVertex *>(_vertices[1]);
    auto vbgi = dynamic_cast<BiasGVertex *>(_vertices[2]);
    auto vbai = dynamic_cast<BiasAVertex *>(_vertices[3]);
    eR_ = (navstate_->p_.so3().inverse() * vpi->estimate().so3()).log();
    Vec3 ep = vpi->estimate().translation() - navstate_->p_.translation();
    Vec3 ev = vvi->estimate() - navstate_->v_;
    Vec3 ebg = vbgi->estimate() - navstate_->bg_;
    Vec3 eba = vbai->estimate() - navstate_->ba_;
    _error << eR_, ep, ev, ebg, eba;
}

void PriorEdge::linearizeOplus() {
    /// 1. 针对pi的雅可比矩阵
    _jacobianOplus[0].setZero();
    _jacobianOplus[0].block<3, 3>(0, 0) = SO3::leftJacobianInverse(-eR_);
    _jacobianOplus[0].block<3, 3>(3, 3) = Mat3::Identity();

    /// 2. 针对vi的雅可比矩阵
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3, 3>(6, 0) = Mat3::Identity();

    /// 3. 针对bgi的雅可比矩阵
    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3, 3>(9, 0) = Mat3::Identity();

    /// 4. 针对bai的雅可比矩阵
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3, 3>(12, 0) = Mat3::Identity();
}

/**
 * @brief GNSS误差约束
 *
 * @param gp        输入的GNSS位姿
 * @param gcov      输入的GNSS协方差向量
 * @param weight    输入的信息权重
 */
GnssEdge::GnssEdge(SE3 gp, Vec6 gcov, double weight)
    : gp_(std::move(gp)) {
    Mat6 inf = Mat6::Zero();
    inf.diagonal() = gcov;
    setInformation(inf.inverse() * weight);
}

/**
 * @brief 计算GNSS误差边的残差
 * @details
 *      1. 定义残差顺序为：eR, ep
 *
 */
void GnssEdge::computeError() {
    auto *vp = dynamic_cast<PoseVertex *>(_vertices[0]);
    eR_ = (gp_.so3().inverse() * vp->estimate().so3()).log();
    Vec3 ep = vp->estimate().translation() - gp_.translation();
    _error << eR_, ep;
}

/**
 * @brief 计算GNSS误差边的雅可比矩阵
 *
 */
void GnssEdge::linearizeOplus() {
    _jacobianOplusXi.setZero();
    _jacobianOplusXi.block<3, 3>(0, 0) = SO3::leftJacobianInverse(-eR_);
    _jacobianOplusXi.block<3, 3>(3, 3) = Mat3::Identity();
}

/**
 * @brief 速度里程计误差约束
 *
 * @param ov        输入的里程计速度(标量)
 * @param ocov      输入的里程计协方差向量
 * @param weight    输入的信息权重
 */
OdomEdge::OdomEdge(double ov, Vec3 ocov, double weight, PoseVertex *Twb)
    : Twb_(Twb) {
    Mat3 inf = Mat3::Zero();
    inf.diagonal() = ocov;
    setInformation(inf.inverse() * weight);
    ov_ = Vec3(ov, 0, 0);
}

/**
 * @brief 计算速度里程计误差边的残差
 *
 */
void OdomEdge::computeError() {
    auto *vv = dynamic_cast<VelocityVertex *>(_vertices[0]);
    _error = vv->estimate() - Twb_->estimate().so3() * ov_;
}

/**
 * @brief 计算速度里程计的雅可比矩阵
 *
 */
void OdomEdge::linearizeOplus() {
    _jacobianOplusXi.setZero();
    _jacobianOplusXi.block<3, 3>(0, 0) = Mat3::Identity();
}

} // namespace insystem