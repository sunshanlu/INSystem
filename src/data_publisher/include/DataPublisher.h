#pragma once
#include <fstream>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "pre_interfaces/msg/rtk.hpp"
#include "pre_interfaces/msg/wodom.hpp"

namespace insystem {

using namespace pre_interfaces::msg;
using namespace sensor_msgs::msg;

class DataPublisher : public rclcpp::Node {
public:
    DataPublisher();

private:
    void PublishData();

    void PublishOdom(std::stringstream &ss);

    void PublishImu(std::stringstream &ss);

    void PublishGnss(std::stringstream &ss);

    std::ifstream ifs_;
    rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<RTK>::SharedPtr rtk_pub_;
    rclcpp::Publisher<Wodom>::SharedPtr wodom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

rclcpp::Time ConvertDouble2Time(const double &stamp);

} // namespace insystem