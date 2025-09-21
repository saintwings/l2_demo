#pragma once
#include <vector>
#include <cstdint>

namespace fastlio2 {

// One LiDAR point with per_point relative time in the scan
struct LidarPoint {
    float x, y, z;
    float intensity;
    double t_rel;   // second since frame start
};

// One LiDAR frame with a timestamp range [t0, t1]
struct LidarFrame {
    double t0 = 0.0;  // frame start time (s, monotonic)
    double t1 = 0.0;  // frame end time (s)
    std::vector<LidarPoint> pts;
};

// IMU sample (from LiDAR's built-in IMU)
struct ImuSample {
    double t;   // time (s)
    float ax, ay, az;   // m/s^2
    float gx, gy, gz;   // rad/s
    // Orientation quaternion (w,x,y,z) if available from device; else identity
    float qw = 1.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
};

}   // namespace fastlio2