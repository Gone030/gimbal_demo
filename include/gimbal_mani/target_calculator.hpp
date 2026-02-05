#ifndef  TARGET_CALCULATOR_HPP
#define TARGET_CALCULATOR_HPP

#include <cmath>

struct Point3D{
    float x;
    float y;
    float z;
};

class TargetCalculator{
    public:
        Point3D calculate(float distance, float yaw_rad, float pitch_rad){
            Point3D p;
            float fixed_pitch = -pitch_rad;

            float xy_projection = distance * std::cos(pitch_rad);

            p.x = xy_projection * std::cos(yaw_rad);
            p.y = xy_projection * std::sin(yaw_rad);
            p.z = distance * std::sin(fixed_pitch);

            return p;
        }
};
#endif
