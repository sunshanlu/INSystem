#pragma once
#include <memory>

#include <Eigen/Core>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <shared_mutex>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

namespace insystem {
class IMU;
using IMUSharedPtr = std::shared_ptr<IMU>;

class IMUIniter;
using IMUIniterSharedPtr = std::shared_ptr<IMUIniter>;

class IMUPreint;
using IMUPreintSharedPtr = std::shared_ptr<IMUPreint>;

class NavState;
using NavStateSharedPtr = std::shared_ptr<NavState>;

class ODOM;
using ODOMSharedPtr = std::shared_ptr<ODOM>;

class Gnss;
using GnssSharedPtr = std::shared_ptr<Gnss>;

using shared_mutex = std::shared_mutex;

using BlockSolver = g2o::BlockSolverX;
using LinearSolver = g2o::LinearSolverEigen<BlockSolver::PoseMatrixType>;

class Data;
using DataSharedPtr = std::shared_ptr<Data>;

class Viewer;
using ViewerSharedPtr = std::shared_ptr<Viewer>;

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec9 = Eigen::Matrix<double, 9, 1>;
using Vec15 = Eigen::Matrix<double, 15, 1>;
using Mat3 = Eigen::Matrix<double, 3, 3>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Mat15 = Eigen::Matrix<double, 15, 15>;
using Mat9 = Eigen::Matrix<double, 9, 9>;
using Mat9_6 = Eigen::Matrix<double, 9, 6>;

using Vec3f = Eigen::Vector3f;

using SO3 = Sophus::SO3d;
using SE3 = Sophus::SE3d;

using SE3f = Sophus::SE3f;
} // namespace insystem
