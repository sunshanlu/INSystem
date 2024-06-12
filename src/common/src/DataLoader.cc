#include "DataLoader.h"
#include "Data.h"
#include "Gnss.h"
#include "IMU.h"
#include "ODOM.h"

namespace insystem {

/**
 * @brief DataLoader的构造
 *
 * @param fp        输入的待读取文件路径
 * @param corType   输入的GNSS坐标系类型
 */
DataLoader::DataLoader(const std::string &fp, Gnss::CoordType corType)
    : coord_type_(std::move(corType)) {
    ifs_.open(fp);
    if (!ifs_.is_open())
        throw std::runtime_error("DataLoader: cannot open file");
}

/**
 * @brief 数据加载主函数
 *
 * @param data  输出的基类数据指针，可以使用std::dynamic_pointer_cast转换为对应的数据类型
 * @return DataLoader::DataType 输出的数据类型
 */
DataLoader::DataType DataLoader::LoadData(DataSharedPtr &data) {
    std::string LineStr;
    if (!std::getline(ifs_, LineStr)) {
        data = nullptr;
        return DataType::End;
    } else if (LineStr.empty()) {
        data = nullptr;
        return DataType::Null;
    }
    std::stringstream Line(LineStr);
    std::string type;
    Line >> type;
    if (type == "IMU") {
        data = LoadIMU(Line);
        return DataType::IMU;
    } else if (type == "GNSS") {
        data = LoadGNSS(Line);
        return DataType::GNSS;
    } else if (type == "ODOM") {
        data = LoadODOM(Line);
        return DataType::ODOM;
    } else
        return DataType::Null;
}

/// @brief 加载IMU数据
IMUSharedPtr DataLoader::LoadIMU(std::stringstream &ss) {
    double data[7];
    ss >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> data[5] >> data[6];
    return std::make_shared<IMU>(data);
}

/// @brief 加载GNSS数据
GnssSharedPtr DataLoader::LoadGNSS(std::stringstream &ss) {
    double data[5];
    bool valid;
    ss >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> valid;
    return std::make_shared<Gnss>(data, valid, coord_type_);
}

/// @brief 加载ODOM数据
ODOMSharedPtr DataLoader::LoadODOM(std::stringstream &ss) {
    double time;
    int LeftPulse, RightPulse;
    ss >> time >> LeftPulse >> RightPulse;
    return std::make_shared<ODOM>(time, LeftPulse, RightPulse);
}

} // namespace insystem