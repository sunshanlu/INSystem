#include "DataPublisher.h"

using namespace insystem;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DataPublisher>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}