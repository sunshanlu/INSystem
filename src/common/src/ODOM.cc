#include "ODOM.h"

namespace insystem {

ODOM::ODOM(double stamp, int leftPulse, int rightPulse)
    : leftPulse_(leftPulse)
    , rightPulse_(rightPulse) {
    stamp_ = stamp;
}

double ODOM::GetVelocity(int n, double r, double dt) {
    double v = (leftPulse_ + rightPulse_) * M_PI * r / (n * dt);
    return v;
}

} // namespace insystem