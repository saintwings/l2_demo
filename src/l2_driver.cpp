#include "fastlio2/l2_driver.hpp"
#include <atomic>
#include <thread>
#include <chrono>
#include <memory>
#include <iostream>

#if USE_UNITREE_SDK
  // From unilidar_sdk2/unitree_lidar_sdk/include/
  #include "unitree_lidar_sdk.h"        // UnitreeLidarReader + factory
  #include "unitree_lidar_utilities.h"  // packet types & data structs
#endif

namespace fastlio2 {

struct L2Driver::Impl {
  Config cfg;
  std::atomic<bool> running{false};
  std::thread th;
  LidarFrameCB on_lidar;
  ImuCB        on_imu;

#if USE_UNITREE_SDK
  unilidar_sdk2::UnitreeLidarReader* sdk = nullptr; // created by factory
  bool using_serial = false;
#endif

  explicit Impl(const Config& c): cfg(c) {}
};

L2Driver::L2Driver(const Config& cfg) : impl_(std::make_unique<Impl>(cfg)) {}
L2Driver::~L2Driver(){ stop(); }

bool L2Driver::start(LidarFrameCB on_lidar, ImuCB on_imu){
  impl_->on_lidar = std::move(on_lidar);
  impl_->on_imu   = std::move(on_imu);

#if USE_UNITREE_SDK
  // Create and init reader
  impl_->sdk = unilidar_sdk2::createUnitreeLidarReader();
  if (!impl_->sdk) { std::cerr << "[L2Driver] createUnitreeLidarReader() failed\n"; return false; }

  int rc = -1;
  if (impl_->cfg.ethernet) {
    rc = impl_->sdk->initializeUDP(
      static_cast<unsigned short>(impl_->cfg.lidar_send),
      impl_->cfg.lidar_ip,
      static_cast<unsigned short>(impl_->cfg.host_listen),
      impl_->cfg.host_ip,
      impl_->cfg.cloud_scan_num,
      impl_->cfg.use_system_timestamp,
      impl_->cfg.range_min,
      impl_->cfg.range_max
    );
    impl_->using_serial = false;
  } else {
    rc = impl_->sdk->initializeSerial(
      impl_->cfg.serial_dev,
      /*baudrate*/ 4000000,
      impl_->cfg.cloud_scan_num,
      impl_->cfg.use_system_timestamp,
      impl_->cfg.range_min,
      impl_->cfg.range_max
    );
    impl_->using_serial = true;
  }
  if (rc != 0) { std::cerr << "[L2Driver] initialize " << (impl_->using_serial?"Serial":"UDP") << " failed\n"; return false; }

  // Common Ethernet+3D+IMU+auto-start = 0; change bits only if needed
  impl_->sdk->setLidarWorkMode(0);

  impl_->running = true;
  impl_->th = std::thread([this](){
    using namespace std::chrono;
    unilidar_sdk2::PointCloudUnitree cloud; // has .points (x,y,z,intensity,time,ring)
    unilidar_sdk2::LidarImuData imu;       // has stamp_sec/nsec, lin acc, ang vel

  while (impl_->running) {
      int type = impl_->sdk->runParse();   // parse one packet (0 if none)
      if (type == LIDAR_POINT_DATA_PACKET_TYPE || type == LIDAR_2D_POINT_DATA_PACKET_TYPE) {
        if (impl_->on_lidar && impl_->sdk->getPointCloud(cloud)) {
          LidarFrame f; f.t0 = cloud.stamp; double max_rel=0.0;
          f.pts.clear(); f.pts.reserve(cloud.points.size());
          for (const auto& pt : cloud.points) {
            LidarPoint p; p.x = pt.x; p.y = pt.y; p.z = pt.z; p.intensity = pt.intensity;
            p.t_rel = static_cast<double>(pt.time); // per-point time from frame start
            if (p.t_rel > max_rel) max_rel = p.t_rel;
            f.pts.push_back(p);
          }
          f.t1 = f.t0 + max_rel;
          impl_->on_lidar(f);
        }
      } else if (type == LIDAR_IMU_DATA_PACKET_TYPE) {
        if (impl_->on_imu && impl_->sdk->getImuData(imu)) {
          ImuSample s;
          s.t  = static_cast<double>(imu.info.stamp.sec) + 1e-9*static_cast<double>(imu.info.stamp.nsec);
          s.ax = imu.linear_acceleration[0]; s.ay = imu.linear_acceleration[1]; s.az = imu.linear_acceleration[2];
          s.gx = imu.angular_velocity[0];    s.gy = imu.angular_velocity[1];    s.gz = imu.angular_velocity[2];
          // orientation quaternion from SDK if present: imu.quaternion[0..3] assumed (w,x,y,z)
          s.qw = imu.quaternion[0]; s.qx = imu.quaternion[1]; s.qy = imu.quaternion[2]; s.qz = imu.quaternion[3];
          impl_->on_imu(s);
        }
      } else {
        std::this_thread::sleep_for(milliseconds(1)); // avoid busy spin
      }
    }
    // Some environments observed a crash when calling closeSerial/closeUDP here.
    // Skip explicit close to avoid shutdown segfault; OS will reclaim resources.
    // if (impl_->using_serial) impl_->sdk->closeSerial(); else impl_->sdk->closeUDP();
  });

  return true;

#else
  // ---- STUB (builds without SDK) ----
  impl_->running = true;
  impl_->th = std::thread([this](){
    using namespace std::chrono;
    auto t0 = steady_clock::now();
    while (impl_->running) {
      double t = duration<double>(steady_clock::now() - t0).count();
      if (impl_->on_imu) { ImuSample s{t, 0,0,9.81f, 0,0,0}; impl_->on_imu(s); }
      if (impl_->on_lidar) {
        LidarFrame f; f.t0=t; f.t1=t+0.1;
        f.pts = {{0,0,0,100.f,0.00},{1,0,0,100.f,0.03},{1,1,0,100.f,0.06},{0,1,0,100.f,0.09}};
        impl_->on_lidar(f);
      }
      std::this_thread::sleep_for(milliseconds(200));
    }
  });
  return true;
#endif
}

void L2Driver::stop(){
  if (!impl_ || !impl_->running) return;
  impl_->running = false;
  if (impl_->th.joinable()) impl_->th.join();
#if USE_UNITREE_SDK
  // Keep SDK instance without explicit close to avoid potential crashes on shutdown
  // impl_->sdk = nullptr;
#endif
}

} // namespace fastlio2
