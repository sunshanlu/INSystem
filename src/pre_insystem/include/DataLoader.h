#pragma once

#include <fstream>
#include <string>

#include "Common.h"
#include "Gnss.h"

namespace insystem {
/// @brief 数据加载器
class DataLoader {
public:
    enum class DataType { IMU, ODOM, GNSS, Null, End };

    DataLoader(const std::string &fp, Gnss::CoordType corType);

    DataType LoadData(DataSharedPtr &data);

private:
    IMUSharedPtr LoadIMU(std::stringstream &ss);

    GnssSharedPtr LoadGNSS(std::stringstream &ss);

    ODOMSharedPtr LoadODOM(std::stringstream &ss);

    std::ifstream ifs_;

    Gnss::CoordType coord_type_;
};

} // namespace insystem