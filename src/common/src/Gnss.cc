#include "Gnss.h"

#include "utm_convert/utm.h"

namespace insystem {

/**
 * @brief Gnss的构造函数
 *
 * @param data      输入的GNSS数据，stamp、lat、lon、alt、yaw
 * @param valid     输入的GNSS数据的yaw是否合格
 * @param type      输入的GNSS数据的坐标系类型
 */
Gnss::Gnss(double *data, bool valid, CoordType type) {
    stamp_ = data[0];
    convert_success_ = ConvertGnss2Utm(data[1], data[2], data[3], data[4], valid, type);
}

/**
 * @brief 将GNSS数据包转换为UTM坐标系
 * @details
 *      1. 代码中要求坐标系类型为东北天坐标系
 *      2. 位姿p_的意义是UTM坐标原点到当前GNSS数据的位姿Tug
 * @param lat   输入的维度
 * @param lon   输入的经度
 * @param alt   输入的高度
 * @param yaw   输入的偏航角
 * @param valid 输入的航向角合格标识
 * @param type  输入的GNSS数据的坐标系类型
 * @return true     转换成功
 * @return false    转换失败
 */
bool Gnss::ConvertGnss2Utm(double lat, double lon, double alt, double yaw, bool valid, CoordType type) {
    long zone;
    char hemisphere;
    double easting, northing;
    long ret = Convert_Geodetic_To_UTM(lat, lon, &zone, &hemisphere, &easting, &northing);
    if (ret != 0)
        return false;
    p_.translation().x() = easting;
    p_.translation().y() = northing;
    p_.translation().z() = alt;
    double heading = 0;
    switch (type) {
    case CoordType::EAST_NORTH:
        heading = yaw * M_PI / 180.0;
        break;

    case CoordType::NORTH_EAST:
        heading = M_PI_2 - yaw * M_PI / 180.0;
        break;
    }
    p_.so3() = SO3::rotZ(heading);
    is_valid_ = valid;
    return true;
}

/**
 * @brief 获取Twb位姿，以mapOrigin为原点
 *
 * @param antennaAngle  输入的RTK天线相对于base坐标系的偏转角（z轴）
 * @param antennaPos    输入的RTK天线相对于base坐标系的位置
 * @param mapOrigin     输入的W原点在UTM坐标系下的位置tuw
 * @return SE3  输出的Twb
 */
SE3 Gnss::GetBasePose(double antennaAngle, const Vec3 &antennaPos, const Vec3 &mapOrigin) const {
    SE3 Tbg(SO3::rotZ(antennaAngle), antennaPos);
    SE3 Twg(p_.so3(), p_.translation() - mapOrigin);
    return Twg * Tbg.inverse();
}

} // namespace insystem
