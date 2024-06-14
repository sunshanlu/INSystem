#pragma once

#include <Eigen/Core>

#include "Common.h"
#include "Data.h"
#include "pre_interfaces/msg/rtk.hpp"

namespace insystem {

struct Gnss : public Data {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using RTK = pre_interfaces::msg::RTK;

    enum class CoordType {
        EAST_NORTH, ///< 东北天坐标系
        NORTH_EAST  ///< 北东地坐标系
    };

    Gnss(double *data, bool valid, CoordType type);

    Gnss(const RTK::SharedPtr &msg, CoordType type);

    /// 将Gnss数据转换为UTM坐标系
    bool ConvertGnss2Utm(double lat, double lon, double alt, double yaw, bool valid, CoordType type);

    /// 获取获取Tob位姿
    SE3 GetBasePose(double antennaAngle, const Vec3 &antennaPos, const Vec3 &mapOrigin) const;

    SE3 p_;                ///< Gnss位姿
    bool is_valid_;        ///< Gnss数据航向角的有效性
    bool convert_success_; ///< 转换是否成功
};

} // namespace insystem