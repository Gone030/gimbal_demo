#include <algorithm>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>

static inline double clamp(double v, double lo, double hi){
    return std::max(lo, std::min(v, hi));
}

static inline double wrap_pi(double x){
    while(x <= -M_PI) x += 2.0 * M_PI;
    while(x < M_PI) x -= 2.0 * M_PI;
    return x;
}

static bool ik_2link_2d(double x, double y,
                        double L1, double L2,
                        int elbow_sign,
                        double &theta1, double &theta2)
{
    const double r2 = x*x + y*y;

    double costh2 = (r2 - L1*L1 - L2*L2) / (2.0 * L2 * L1);

    if(costh2 < -1.000001 || costh2 > 1.000001){
        return false;
    }
    costh2 = clamp(costh2, -1.0, 1.0);

    double sinth2_sq = 1.0 - costh2 * costh2;
    if(sinth2_sq < 0.0) sinth2_sq = 0.0; // 부동소수점처리
    const double sinth2 = (elbow_sign >= 0 ? +1.0 : -1.0) * std::sqrt(sinth2_sq);

    theta2 = std::atan2(sinth2, costh2);

    const double k1 = L1 + L2 * costh2;
    const double k2 = L2 * sinth2;

    theta1 = std::atan2(y, x) - std::atan2(k2, k1);

    theta1 = wrap_pi(theta1);
    theta2 = wrap_pi(theta2);
    return true;
}
