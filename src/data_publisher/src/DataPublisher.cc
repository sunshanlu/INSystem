#include <cmath>

#include "DataPublisher.h"

namespace insystem {

using namespace std::chrono_literals;

DataPublisher::DataPublisher()
    : Node("DataPublisher") {
    declare_parameter("DataPath", "/home/rookie-lu/Project/INSystem/data/10.txt");
    std::string DataPath = get_parameter("DataPath").as_string();
    ifs_.open(DataPath);
    if (!ifs_.is_open())
        throw std::runtime_error("Data file not found");
    timer_ = create_wall_timer(2ms, std::bind(&DataPublisher::PublishData, this));
    wodom_pub_ = create_publisher<Wodom>("pre_insystem/odom", 10);
    imu_pub_ = create_publisher<Imu>("pre_insystem/imu", 10);
    rtk_pub_ = create_publisher<RTK>("pre_insystem/rtk", 10);
}

void DataPublisher::PublishData() {
    std::string line;
    if (!std::getline(ifs_, line)) {
        rclcpp::shutdown();
        return;
    }
    std::stringstream ss(line);
    std::string type;
    ss >> type;
    if (type == "ODOM")
        PublishOdom(ss);
    else if (type == "GNSS")
        PublishGnss(ss);
    else if (type == "IMU")
        PublishImu(ss);
}

void DataPublisher::PublishOdom(std::stringstream &ss) {
    double stamp;
    int lPulse, rPulse;
    ss >> stamp >> lPulse >> rPulse;
    Wodom odom;
    odom.header.stamp = ConvertDouble2Time(stamp);
    odom.header.frame_id = "odom";
    odom.left_pulse = lPulse;
    odom.right_pulse = rPulse;
    wodom_pub_->publish(odom);
}

void DataPublisher::PublishImu(std::stringstream &ss) {
    double stamp, gx, gy, gz, ax, ay, az;
    ss >> stamp >> gx >> gy >> gz >> ax >> ay >> az;
    Imu imu;
    imu.header.stamp = ConvertDouble2Time(stamp);
    imu.header.frame_id = "imu";
    imu.angular_velocity.x = gx;
    imu.angular_velocity.y = gy;
    imu.angular_velocity.z = gz;
    imu.linear_acceleration.x = ax;
    imu.linear_acceleration.y = ay;
    imu.linear_acceleration.z = az;
    imu_pub_->publish(imu);
}

void DataPublisher::PublishGnss(std::stringstream &ss) {
    double stamp, lat, lon, alt, yaw;
    bool yaw_valid;
    ss >> stamp >> lat >> lon >> alt >> yaw >> yaw_valid;
    RTK rtk;
    rtk.header.stamp = ConvertDouble2Time(stamp);
    rtk.header.frame_id = "rtk";
    rtk.latitude = lat;
    rtk.longitude = lon;
    rtk.altitude = alt;
    rtk.yaw = yaw;
    rtk.yaw_valid = yaw_valid;
    rtk_pub_->publish(rtk);
}

rclcpp::Time ConvertDouble2Time(const double &stamp) {
    int32_t second = std::floor(stamp);
    int64_t nanosecond = (stamp - second) * 1e9;
    return rclcpp::Time(second, nanosecond);
}

} // namespace insystem
