#pragma once
#include <functional>
#include <memory>
#include <string>
#include "fastlio2/lidar_types.hpp"

namespace fastlio2 {

// Simple callback-based driver interface
class L2Driver {
public:
    struct Config {
        std::string host_ip     = "192.168.1.2";    // your PC
        int host_listen         = 6201;             // PC listens
        std::string lidar_ip    = "192.168.1.62";    // L2 device
        int lidar_send          = 6101;             // L2 sneds from
        bool ethernet           = true;             // else: serial
        bool use_imu            = true;
        std::string serial_dev  = "/dev/ttyACM0";   // if serial
        uint16_t    cloud_scan_num = 18;
        bool        use_system_timestamp = true;
        float       range_min = 0.0f;
        float       range_max = 100.0f;
    };

    using LidarFrameCB = std::function<void(const LidarFrame&)>;
    using ImuCB        = std::function<void(const ImuSample&)>;

    explicit L2Driver(const Config& cfg);
    ~L2Driver();

    bool start(LidarFrameCB on_lidar, ImuCB on_imu);
    void stop();


private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

};

} // namespace fastlio2