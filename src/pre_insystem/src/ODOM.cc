#include "ODOM.h"

namespace insystem {

ODOM::ODOM(double stamp, int leftPulse, int rightPulse)
    : leftPulse_(leftPulse)
    , rightPulse_(rightPulse) {
    stamp_ = stamp;
}

ODOM::ODOM(const pre_interfaces::msg::Wodom::SharedPtr &msg)
    : leftPulse_(msg->left_pulse)
    , rightPulse_(msg->right_pulse) {
    stamp_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
}

double ODOM::GetVelocity(int n, double r, double dt) {
    double v = (leftPulse_ + rightPulse_) * M_PI * r / (n * dt);
    return v;
}

} // namespace insystem