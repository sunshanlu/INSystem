#pragma once

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>

#include "common/common.h"

namespace insystem {

/// @brief 6DoF位姿顶点
class PoseVertex : public g2o::BaseVertex<6, SE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PoseVertex() = default;

    void oplusImpl(const double *update) override;

    bool read(std::istream &is) { return false; }

    bool write(std::ostream &os) const { return false; }

    void setToOriginImpl() {}
};

/// @brief 3自由度速度顶点
class VelocityVertex : public g2o::BaseVertex<3, Vec3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VelocityVertex() = default;
    void oplusImpl(const double *update) override;

    bool read(std::istream &is) { return false; }

    bool write(std::ostream &os) const { return false; }

    void setToOriginImpl() {}
};

/// @brief 3自由度加速度偏置顶点
class BiasAVertex : public VelocityVertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BiasAVertex() = default;
};

/// @brief 3自由度陀螺仪偏置顶点
class BiasGVertex : public VelocityVertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BiasGVertex() = default;
};

/// @brief 预积分的边
class PreIntEdge : public g2o::BaseMultiEdge<9, Vec9> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PreIntEdge(IMUPreintSharedPtr &preintPtr, const Vec3 &g, double weight);

    void computeError() override;

    void linearizeOplus() override;

    bool read(std::istream &is) { return false; }

    bool write(std::ostream &os) const { return false; }

private:
    IMUPreintSharedPtr preintPtr_; ///< 预积分对象指针
    Vec3 g_;                       ///< 重力，用于计算残差
    double dt_;                    ///< 预积分时间
    Vec3 eR_;                      ///< 旋转残差
    Vec3 ev_;                      ///< 速度残差
    Vec3 ep_;                      ///< 位置残差
};

template <typename Vertex> class BiasEdge;
using BiasGEdge = BiasEdge<BiasGVertex>;
using BiasAEdge = BiasEdge<BiasAVertex>;

/// @brief 对前后两个状态的偏置约束
template <typename Vertex> class BiasEdge : public g2o::BaseBinaryEdge<3, Vec3, Vertex, Vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BiasEdge() = default;

    /**
     * @brief 计算残差
     * @details
     *      1. 这里定义顶点的顺序为bi、bj
     */
    void computeError() override {
        auto *bi = dynamic_cast<Vertex *>(_vertices[0]);
        auto *bj = dynamic_cast<Vertex *>(_vertices[1]);
        _error = bj->estimate() - bi->estimate();
    }

    void linearizeOplus() override {
        _jacobianOplusXi = -Mat3::Identity();
        _jacobianOplusXj = Mat3::Identity();
    }

    bool read(std::istream &is) { return false; }

    bool write(std::ostream &os) const { return false; }
};

/// @brief 先验信息约束
class PriorEdge : public g2o::BaseMultiEdge<15, Vec15> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PriorEdge(NavStateSharedPtr navstate)
        : navstate_(navstate) {}

    void computeError() override;

    void linearizeOplus() override;

    bool read(std::istream &is) { return false; }

    bool write(std::ostream &os) const { return false; }

private:
    NavStateSharedPtr navstate_; ///< 输入的i时刻的状态
    Vec3 eR_;                    ///< 用于候选旋转部分的雅可比求解
};

/// @brief GNSS约束，i时刻和j时刻都要约束
class GnssEdge : public g2o::BaseUnaryEdge<6, Vec6, PoseVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GnssEdge(SE3 gp, Vec6 gcov, double weight);

    void computeError() override;

    void linearizeOplus() override;

    bool read(std::istream &is) { return false; }

    bool write(std::ostream &os) const { return false; }

private:
    SE3 gp_;  ///< gnss位姿
    Vec3 eR_; ///< 旋转误差
};

/// @brief 速度里程计约束，只有j时刻约束
class OdomEdge : public g2o::BaseUnaryEdge<3, Vec3, VelocityVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    OdomEdge(double ov, Vec3 ocov, double weight, PoseVertex *Twb);

    void computeError() override;

    void linearizeOplus() override;

    bool read(std::istream &is) { return false; }

    bool write(std::ostream &os) const { return false; }

private:
    Vec3 ov_;
    PoseVertex *Twb_;
};

} // namespace insystem