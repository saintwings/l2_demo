#include <iostream>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <cmath>
#include <array>
#include <cstring>
#include <algorithm>
#include <deque>
#include <queue>

#include <yaml-cpp/yaml.h>

#include "fastlio2/l2_driver.hpp"
#include "fastlio2/ws_server.hpp"
#include "fastlio2/lidar_types.hpp"

#ifdef FASTLIO2_USE_PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#endif

using namespace fastlio2;

struct RuntimeCfg {
  // detection
  float voxel_size = 0.05f;
  bool  remove_ground = true;
  float ground_threshold = 0.10f;
  bool  stream_ground_points = true;
  uint8_t ground_color_r = 80, ground_color_g = 160, ground_color_b = 255;
  int    accumulate_frames = 1; // how many frames to accumulate in buffer before processing
  float  accumulate_seconds = 0.5f; // time window to retain points
  int    max_accumulated_points = 100000; // safety cap
  float cluster_tolerance = 0.30f;
  int   min_cluster_size = 10;
  int   max_cluster_size = 5000;
  // clustering method: "euclidean" or "dbscan"
  std::string clustering_method = "euclidean";
  float dbscan_eps = 0.35f; // meters
  int   dbscan_min_points = 8;
  float min_object_volume = 0.01f;
  float max_object_volume = 50.0f;
  float max_detection_range = 20.0f;
  bool  enable_aoi = false;
  float aoi_min_x = -10, aoi_max_x = 10;
  float aoi_min_y = -10, aoi_max_y = 10;
  float aoi_min_z = -2,  aoi_max_z = 5;
  bool  enable_radial_filter = false;
  float radial_min_range = 0.5f;
  float radial_max_range = 20.0f;
  float radial_min_angle = -180.0f; // deg
  float radial_max_angle =  180.0f; // deg
  bool  imu_rotate_points = true; // rotate cloud into IMU/world frame

  // streaming
  int ws_port = 8081;

  // tracking
  bool  tracking_enabled = true;
  float tracking_max_assoc_dist = 0.6f; // meters
  float tracking_alpha = 0.6f;          // EMA weight for new meas
  int   tracking_max_missed = 3;        // frames
};

static RuntimeCfg load_cfg(const std::string& path){
  RuntimeCfg cfg;
  try{
    YAML::Node root = YAML::LoadFile(path);
    if (root["deskew"]) cfg.ws_port = root["deskew"]["ws_port"].as<int>(cfg.ws_port);
    if (root["object_detection"]) {
      auto od = root["object_detection"]; 
      cfg.voxel_size = od["voxel_size"].as<float>(cfg.voxel_size);
      cfg.remove_ground = od["remove_ground"].as<bool>(cfg.remove_ground);
      cfg.ground_threshold = od["ground_threshold"].as<float>(cfg.ground_threshold);
      if (od["stream_ground_points"]) cfg.stream_ground_points = od["stream_ground_points"].as<bool>(cfg.stream_ground_points);
      if (od["ground_color_rgb"]) {
        auto v = od["ground_color_rgb"].as<std::vector<int>>();
        if (v.size() == 3){ cfg.ground_color_r = (uint8_t)std::clamp(v[0],0,255); cfg.ground_color_g = (uint8_t)std::clamp(v[1],0,255); cfg.ground_color_b = (uint8_t)std::clamp(v[2],0,255); }
      }
      if (od["accumulate_frames"]) cfg.accumulate_frames = std::max(1, od["accumulate_frames"].as<int>(cfg.accumulate_frames));
      if (od["accumulate_seconds"]) cfg.accumulate_seconds = std::max(0.0f, od["accumulate_seconds"].as<float>(cfg.accumulate_seconds));
      if (od["max_accumulated_points"]) cfg.max_accumulated_points = std::max(1000, od["max_accumulated_points"].as<int>(cfg.max_accumulated_points));
      cfg.cluster_tolerance = od["cluster_tolerance"].as<float>(cfg.cluster_tolerance);
      cfg.min_cluster_size = od["min_cluster_size"].as<int>(cfg.min_cluster_size);
      cfg.max_cluster_size = od["max_cluster_size"].as<int>(cfg.max_cluster_size);
  if (od["clustering_method"]) cfg.clustering_method = od["clustering_method"].as<std::string>(cfg.clustering_method);
  if (od["dbscan_eps"]) cfg.dbscan_eps = od["dbscan_eps"].as<float>(cfg.dbscan_eps);
  if (od["dbscan_min_points"]) cfg.dbscan_min_points = od["dbscan_min_points"].as<int>(cfg.dbscan_min_points);
      cfg.min_object_volume = od["min_object_volume"].as<float>(cfg.min_object_volume);
      cfg.max_object_volume = od["max_object_volume"].as<float>(cfg.max_object_volume);
      cfg.max_detection_range = od["max_detection_range"].as<float>(cfg.max_detection_range);
      cfg.enable_aoi = od["enable_aoi"].as<bool>(cfg.enable_aoi);
      cfg.aoi_min_x = od["aoi_min_x"].as<float>(cfg.aoi_min_x);
      cfg.aoi_max_x = od["aoi_max_x"].as<float>(cfg.aoi_max_x);
      cfg.aoi_min_y = od["aoi_min_y"].as<float>(cfg.aoi_min_y);
      cfg.aoi_max_y = od["aoi_max_y"].as<float>(cfg.aoi_max_y);
      cfg.aoi_min_z = od["aoi_min_z"].as<float>(cfg.aoi_min_z);
      cfg.aoi_max_z = od["aoi_max_z"].as<float>(cfg.aoi_max_z);
      cfg.enable_radial_filter = od["enable_radial_filter"].as<bool>(cfg.enable_radial_filter);
      cfg.radial_min_range = od["radial_min_range"].as<float>(cfg.radial_min_range);
      cfg.radial_max_range = od["radial_max_range"].as<float>(cfg.radial_max_range);
      cfg.radial_min_angle = od["radial_min_angle"].as<float>(cfg.radial_min_angle);
      cfg.radial_max_angle = od["radial_max_angle"].as<float>(cfg.radial_max_angle);
      if (od["imu_rotate_points"]) cfg.imu_rotate_points = od["imu_rotate_points"].as<bool>(cfg.imu_rotate_points);
      if (od["tracking_enabled"]) cfg.tracking_enabled = od["tracking_enabled"].as<bool>(cfg.tracking_enabled);
      if (od["tracking_max_assoc_dist"]) cfg.tracking_max_assoc_dist = od["tracking_max_assoc_dist"].as<float>(cfg.tracking_max_assoc_dist);
      if (od["tracking_alpha"]) cfg.tracking_alpha = std::clamp(od["tracking_alpha"].as<float>(cfg.tracking_alpha), 0.0f, 1.0f);
      if (od["tracking_max_missed"]) cfg.tracking_max_missed = std::max(0, od["tracking_max_missed"].as<int>(cfg.tracking_max_missed));
    }
  } catch (const std::exception& e){
    std::cerr << "Failed to load config: " << e.what() << ". Using defaults.\n";
  }
  return cfg;
}

struct ImuState {
  bool calibrated = false;
  bool stationary = true;
  float gx=0, gy=0, gz=1; // gravity vector (approx)
  float motion_mag = 0;
  // orientation quaternion (w,x,y,z)
  float qw=1, qx=0, qy=0, qz=0;
};

struct ObjectOut {
  float cx, cy, cz;  // center
  float w, h, d;     // dims
  float volume;      // m^3
  float distance;    // from origin
  float confidence;  // simple heuristic
  int   count;       // points
  int   id;          // incrementing id
  std::array<float, 24> bbox_corners; // 8 corners * 3 -> 24 floats (x,y,z per corner)
};

static void append_u32(std::vector<uint8_t>& v, uint32_t x){
  for (int i=0;i<4;i++) v.push_back((x >> (8*i)) & 0xFF);
}
static void append_i32(std::vector<uint8_t>& v, int32_t x){
  append_u32(v, static_cast<uint32_t>(x));
}
static void append_f32(std::vector<uint8_t>& v, float x){
  uint32_t u; std::memcpy(&u, &x, 4); append_u32(v, u);
}
static void append_f64(std::vector<uint8_t>& v, double x){
  uint64_t u; std::memcpy(&u, &x, 8); for(int i=0;i<8;i++) v.push_back((u>>(8*i))&0xFF);
}

static void pack_and_send(WsServer& ws, uint32_t frame_id, double stamp,
                          const ImuState& imu,
                          const std::vector<ObjectOut>& objs,
                          const std::vector<float>& ground_xyz,
                          const std::vector<float>& ground_I,
                          const std::vector<float>& pts_xyz,
                          const std::vector<float>& pts_I){
  // Header size grows by 4 bytes for numGroundPoints
  std::vector<uint8_t> buf; buf.reserve(58 + objs.size()* (44 + 24*4) + (ground_I.size()+pts_I.size())*16/1);
  // Header
  append_u32(buf, frame_id);
  append_f64(buf, stamp);
  append_u32(buf, (uint32_t)objs.size());
  append_u32(buf, (uint32_t)(pts_I.size()));
  append_u32(buf, (uint32_t)(ground_I.size())); // NEW: number of ground points
  // IMU extended block (34 bytes after first 20)
  buf.push_back(imu.calibrated?1:0);
  buf.push_back(imu.stationary?1:0);
  append_f32(buf, imu.gx); append_f32(buf, imu.gy); append_f32(buf, imu.gz);
  append_f32(buf, imu.motion_mag);
  // orientation w,x,y,z
  append_f32(buf, imu.qw); append_f32(buf, imu.qx); append_f32(buf, imu.qy); append_f32(buf, imu.qz);

  // Objects
  for (const auto& o : objs){
    append_f32(buf, o.cx); append_f32(buf, o.cy); append_f32(buf, o.cz);
    append_f32(buf, o.w);  append_f32(buf, o.h);  append_f32(buf, o.d);
    append_f32(buf, o.volume);
    append_f32(buf, o.distance);
    append_f32(buf, o.confidence);
    append_i32(buf, o.count);
    append_i32(buf, o.id);
  for (int i=0;i<24;i++) append_f32(buf, o.bbox_corners[i]);
  }

  // Ground points first (x,y,z,I)
  for (size_t i=0;i<ground_I.size();i++){
    append_f32(buf, ground_xyz[3*i+0]);
    append_f32(buf, ground_xyz[3*i+1]);
    append_f32(buf, ground_xyz[3*i+2]);
    append_f32(buf, ground_I[i]);
  }

  // Points (x,y,z,I)
  size_t n = pts_I.size();
  for (size_t i=0;i<n;i++){
    append_f32(buf, pts_xyz[3*i+0]);
    append_f32(buf, pts_xyz[3*i+1]);
    append_f32(buf, pts_xyz[3*i+2]);
    append_f32(buf, pts_I[i]);
  }

  ws.broadcast(buf.data(), buf.size());
}

#ifdef FASTLIO2_USE_PCL
using P = pcl::PointXYZI;
static inline float point_range(const P& p){ return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z); }

// Simple KD-tree based DBSCAN clustering for PointXYZI
static void dbscan(const pcl::PointCloud<P>::ConstPtr& cloud, float eps, int minPts,
                   std::vector<pcl::PointIndices>& clusters){
  clusters.clear();
  const int n = (int)cloud->size();
  if (n == 0) return;
  std::vector<char> visited(n, 0);
  std::vector<int> labels(n, -1); // -1: unassigned, >=0: cluster id, -2: noise
  pcl::search::KdTree<P>::Ptr tree(new pcl::search::KdTree<P>());
  tree->setInputCloud(cloud);
  std::vector<int> neigh_idx; std::vector<float> neigh_dist;
  int cluster_id = 0;
  for (int i=0; i<n; ++i){
    if (visited[i]) continue; visited[i] = 1;
    neigh_idx.clear(); neigh_dist.clear();
    tree->radiusSearch((*cloud)[i], eps, neigh_idx, neigh_dist);
    if ((int)neigh_idx.size() < minPts){ labels[i] = -2; continue; }
    // start new cluster
    labels[i] = cluster_id;
    std::queue<int> q; for (int nb : neigh_idx) if (nb != i) q.push(nb);
    while(!q.empty()){
      int j = q.front(); q.pop();
      if (!visited[j]){
        visited[j] = 1; neigh_idx.clear(); neigh_dist.clear();
        tree->radiusSearch((*cloud)[j], eps, neigh_idx, neigh_dist);
        if ((int)neigh_idx.size() >= minPts){ for (int nb : neigh_idx) q.push(nb); }
      }
      if (labels[j] < 0){ labels[j] = cluster_id; }
    }
    cluster_id++;
  }
  clusters.resize(cluster_id);
  for (int i=0;i<n;++i){ int cid = labels[i]; if (cid >= 0) clusters[cid].indices.push_back(i); }
}
#endif

int main(int argc, char** argv){
  std::string cfg_path = "config/unilidar_l2.yaml";
  if (argc>1) cfg_path = argv[1];

  RuntimeCfg cfg = load_cfg(cfg_path);

  // Start WS server
  WsServer ws;
  if (cfg.ws_port>0) ws.start((uint16_t)cfg.ws_port);

  std::atomic<uint32_t> frame_id{0};
  std::mutex mtx_imu;
  ImuState imu;

  L2Driver::Config dcfg; // defaults okay; adjust here if needed
  L2Driver drv(dcfg);

  auto on_imu = [&](const ImuSample& s){
    std::lock_guard<std::mutex> lk(mtx_imu);
    // Very basic: use accelerometer as gravity estimate, and integrate trivial orientation placeholder
    imu.gx = s.ax; imu.gy = s.ay; imu.gz = s.az;
    const float gnorm = std::sqrt(imu.gx*imu.gx + imu.gy*imu.gy + imu.gz*imu.gz) + 1e-9f;
    // simple stationary heuristic (near 1g)
    imu.stationary = (std::fabs(gnorm - 9.81f) < 0.5f);
    imu.calibrated = true; // pretend after first sample
    // use device quaternion if available
    imu.qw = s.qw; imu.qx = s.qx; imu.qy = s.qy; imu.qz = s.qz;
    imu.motion_mag = std::fabs(gnorm - 9.81f);
  };

  // Simple rolling buffer for accumulation
  std::mutex mtx_buf;
  std::deque<pcl::PointCloud<P>::Ptr> buf_clouds;
  std::deque<double> buf_stamps;
  size_t buf_points = 0;

  auto on_lidar = [&](const LidarFrame& f){
#ifdef FASTLIO2_USE_PCL
    // Convert to PCL cloud
    pcl::PointCloud<P>::Ptr cloud(new pcl::PointCloud<P>());
    cloud->reserve(f.pts.size());
    // Optional IMU-based rotation: rotate points by inverse IMU orientation (sensor->world)
    ImuState imu_copy; { std::lock_guard<std::mutex> lk(mtx_imu); imu_copy = imu; }
    float qw=imu_copy.qw, qx=imu_copy.qx, qy=imu_copy.qy, qz=imu_copy.qz;
    // Normalize quaternion
    {
      float nq = std::sqrt(qw*qw+qx*qx+qy*qy+qz*qz);
      if (nq>1e-6f){ qw/=nq; qx/=nq; qy/=nq; qz/=nq; }
      else { qw=1; qx=qy=qz=0; }
    }
    // Use inverse quaternion for rotating points from IMU frame to world: q_inv = (w, -x, -y, -z)
    float iw=qw, ix=-qx, iy=-qy, iz=-qz;
    auto rotate = [&](float x, float y, float z){
      // q * v * q^{-1} optimized as matrix
      float xx = 1 - 2*(iy*iy + iz*iz);
      float xy = 2*(ix*iy - iz*iw);
      float xz = 2*(ix*iz + iy*iw);
      float yx = 2*(ix*iy + iz*iw);
      float yy = 1 - 2*(ix*ix + iz*iz);
      float yz = 2*(iy*iz - ix*iw);
      float zx = 2*(ix*iz - iy*iw);
      float zy = 2*(iy*iz + ix*iw);
      float zz = 1 - 2*(ix*ix + iy*iy);
      return std::array<float,3>{ xx*x + xy*y + xz*z, yx*x + yy*y + yz*z, zx*x + zy*y + zz*z };
    };
    for (const auto& pt : f.pts){
      P p;
      if (cfg.imu_rotate_points){
        auto v = rotate(pt.x, pt.y, pt.z);
        p.x=v[0]; p.y=v[1]; p.z=v[2];
      } else { p.x=pt.x; p.y=pt.y; p.z=pt.z; }
      p.intensity=pt.intensity;
      cloud->push_back(p);
    }

  // AOI and radial filters
    if (cfg.enable_aoi || cfg.enable_radial_filter){
      pcl::PointCloud<P>::Ptr tmp(new pcl::PointCloud<P>());
      tmp->reserve(cloud->size());
      const float ang_min = cfg.radial_min_angle * (float)M_PI/180.0f;
      const float ang_max = cfg.radial_max_angle * (float)M_PI/180.0f;
      for (const auto& p : *cloud){
        if (cfg.enable_aoi){
          if (p.x<cfg.aoi_min_x || p.x>cfg.aoi_max_x || p.y<cfg.aoi_min_y || p.y>cfg.aoi_max_y || p.z<cfg.aoi_min_z || p.z>cfg.aoi_max_z) continue;
        }
        if (cfg.enable_radial_filter){
          float r = std::sqrt(p.x*p.x + p.y*p.y);
          if (r<cfg.radial_min_range || r>cfg.radial_max_range) continue;
          float ang = std::atan2(p.y, p.x);
          if (ang < ang_min || ang > ang_max) continue;
        }
        if (point_range(p) > cfg.max_detection_range) continue;
        tmp->push_back(p);
      }
      cloud.swap(tmp);
    }

  // Downsample
    if (cfg.voxel_size > 1e-6f){
      pcl::VoxelGrid<P> vg; vg.setLeafSize(cfg.voxel_size, cfg.voxel_size, cfg.voxel_size);
      vg.setInputCloud(cloud);
      pcl::PointCloud<P>::Ptr filtered(new pcl::PointCloud<P>());
      vg.filter(*filtered);
      cloud.swap(filtered);
    }

    // Push into accumulation buffer (pre-ground segmentation)
    {
      std::lock_guard<std::mutex> lk(mtx_buf);
      buf_clouds.push_back(cloud);
      buf_stamps.push_back(f.t1);
      buf_points += cloud->size();
      // Frame-count limit
      while ((int)buf_clouds.size() > cfg.accumulate_frames){ buf_points -= buf_clouds.front()->size(); buf_clouds.pop_front(); buf_stamps.pop_front(); }
      // Time-window limit
      if (cfg.accumulate_seconds > 1e-6f){
        const double latest = f.t1;
        while (!buf_stamps.empty() && (latest - buf_stamps.front()) > cfg.accumulate_seconds){ buf_points -= buf_clouds.front()->size(); buf_clouds.pop_front(); buf_stamps.pop_front(); }
      }
      // Point-cap limit
      while ((int)buf_points > cfg.max_accumulated_points && !buf_clouds.empty()){ buf_points -= buf_clouds.front()->size(); buf_clouds.pop_front(); buf_stamps.pop_front(); }
    }

    // Merge accumulated clouds into one for processing
    pcl::PointCloud<P>::Ptr merged(new pcl::PointCloud<P>());
    {
      std::lock_guard<std::mutex> lk(mtx_buf);
      merged->reserve(buf_points);
      for (auto& c : buf_clouds) *merged += *c;
    }

    // Ground removal on merged cloud
  pcl::PointCloud<P>::Ptr nonground(new pcl::PointCloud<P>());
  pcl::PointCloud<P>::Ptr ground(new pcl::PointCloud<P>());
    if (cfg.remove_ground && merged->size() >= 50){
      // Simple RANSAC plane near z ~ 0; optionally bias by IMU gravity (not fully used here)
      pcl::SACSegmentation<P> seg; seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(cfg.ground_threshold);
      seg.setMaxIterations(100);
      seg.setInputCloud(merged);
      pcl::ModelCoefficients coeff; pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
      seg.segment(*inliers, coeff);
      if (!inliers->indices.empty()){
        pcl::ExtractIndices<P> ex; ex.setInputCloud(merged); ex.setIndices(inliers);
        ex.setNegative(true); ex.filter(*nonground); // non-ground
        ex.setNegative(false); ex.filter(*ground);   // ground
      } else { *nonground = *merged; }
    } else {
      *nonground = *merged;
    }

    // Clustering
    std::vector<pcl::PointIndices> clusters;
    if (!nonground->empty()){
      if (cfg.clustering_method == "dbscan"){
        dbscan(nonground, cfg.dbscan_eps, cfg.dbscan_min_points, clusters);
        // apply max_cluster_size filter if set (>0)
        if (cfg.max_cluster_size > 0){
          std::vector<pcl::PointIndices> filtered; filtered.reserve(clusters.size());
          for (auto &c : clusters) if ((int)c.indices.size() >= cfg.min_cluster_size && (int)c.indices.size() <= cfg.max_cluster_size) filtered.push_back(std::move(c));
          clusters.swap(filtered);
        }
      } else {
        pcl::search::KdTree<P>::Ptr tree(new pcl::search::KdTree<P>());
        tree->setInputCloud(nonground);
        pcl::EuclideanClusterExtraction<P> ec;
        ec.setClusterTolerance(cfg.cluster_tolerance);
        ec.setMinClusterSize(cfg.min_cluster_size);
        ec.setMaxClusterSize(cfg.max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(nonground);
        ec.extract(clusters);
      }
    }

  // Prepare outputs (measurements)
  std::vector<ObjectOut> measured_objs;
    int obj_id = 0;
    for (const auto& ind : clusters){
      // Compute AABB
      float minx=1e9, miny=1e9, minz=1e9, maxx=-1e9, maxy=-1e9, maxz=-1e9;
      for (int idx : ind.indices){ const auto& p = (*nonground)[idx];
        if (p.x<minx) minx=p.x; if (p.y<miny) miny=p.y; if (p.z<minz) minz=p.z;
        if (p.x>maxx) maxx=p.x; if (p.y>maxy) maxy=p.y; if (p.z>maxz) maxz=p.z;
      }
      float cx = 0.5f*(minx+maxx), cy=0.5f*(miny+maxy), cz=0.5f*(minz+maxz);
      float w = std::max(1e-3f, maxx-minx), h = std::max(1e-3f, maxy-miny), d = std::max(1e-3f, maxz-minz);
      float volume = w*h*d;
      if (volume < cfg.min_object_volume || volume > cfg.max_object_volume) continue;
      float dist = std::sqrt(cx*cx + cy*cy + cz*cz);

  ObjectOut o; o.cx=cx; o.cy=cy; o.cz=cz; o.w=w; o.h=h; o.d=d; o.volume=volume; o.distance=dist; o.count=(int)ind.indices.size(); o.id = obj_id++;
      // crude confidence: closer and denser -> higher
      o.confidence = std::max(0.0f, std::min(1.0f, 1.0f - dist/cfg.max_detection_range)) * std::min(1.0f, (float)o.count/500.0f);
      // 8 corners of AABB
      std::array<std::array<float,3>,8> corners = {{
        {minx,miny,minz},{maxx,miny,minz},{maxx,maxy,minz},{minx,maxy,minz},
        {minx,miny,maxz},{maxx,miny,maxz},{maxx,maxy,maxz},{minx,maxy,maxz}
      }};
      int k=0; for (int i=0;i<8;i++){ o.bbox_corners[k++]=corners[i][0]; o.bbox_corners[k++]=corners[i][1]; o.bbox_corners[k++]=corners[i][2]; }
      measured_objs.push_back(o);
    }

    // Simple multi-frame tracking for stable IDs and smoothing
    static std::vector<ObjectOut> tracks; // reuse ObjectOut to hold smoothed state
    static std::vector<int> track_missed; // missed counters
    static int next_track_id = 0;
    static std::mutex tracks_mtx;
    std::vector<ObjectOut> out_objs;
    if (cfg.tracking_enabled){
      std::lock_guard<std::mutex> g_tracks(tracks_mtx);
      // Associate by nearest center within gating distance (greedy)
      const float gate = cfg.tracking_max_assoc_dist;
      std::vector<int> meas_assigned(measured_objs.size(), -1);
      std::vector<int> track_assigned(tracks.size(), -1);
      for (size_t ti=0; ti<tracks.size(); ++ti){
        float best_d2 = gate*gate; int best_m = -1;
        for (size_t mi=0; mi<measured_objs.size(); ++mi){ if (meas_assigned[mi] != -1) continue;
          float dx = measured_objs[mi].cx - tracks[ti].cx;
          float dy = measured_objs[mi].cy - tracks[ti].cy;
          float dz = measured_objs[mi].cz - tracks[ti].cz;
          float d2 = dx*dx+dy*dy+dz*dz;
          if (d2 < best_d2){ best_d2 = d2; best_m = (int)mi; }
        }
        if (best_m >= 0){
          // Update track with EMA
          float a = cfg.tracking_alpha;
          auto& t = tracks[ti];
          const auto& m = measured_objs[best_m];
          t.cx = a*m.cx + (1-a)*t.cx;
          t.cy = a*m.cy + (1-a)*t.cy;
          t.cz = a*m.cz + (1-a)*t.cz;
          t.w  = a*m.w  + (1-a)*t.w;
          t.h  = a*m.h  + (1-a)*t.h;
          t.d  = a*m.d  + (1-a)*t.d;
          t.volume = t.w*t.h*t.d;
          t.distance = std::sqrt(t.cx*t.cx + t.cy*t.cy + t.cz*t.cz);
          t.count = m.count; // carry through
          t.confidence = std::max(t.confidence, m.confidence);
          // rebuild bbox from smoothed dims/center
          const float minx=t.cx - 0.5f*t.w, maxx=t.cx + 0.5f*t.w;
          const float miny=t.cy - 0.5f*t.h, maxy=t.cy + 0.5f*t.h;
          const float minz=t.cz - 0.5f*t.d, maxz=t.cz + 0.5f*t.d;
          std::array<std::array<float,3>,8> corners = {{
            {minx,miny,minz},{maxx,miny,minz},{maxx,maxy,minz},{minx,maxy,minz},
            {minx,miny,maxz},{maxx,miny,maxz},{maxx,maxy,maxz},{minx,maxy,maxz}
          }};
          int k=0; for (int i=0;i<8;i++){ t.bbox_corners[k++]=corners[i][0]; t.bbox_corners[k++]=corners[i][1]; t.bbox_corners[k++]=corners[i][2]; }
          track_missed[ti] = 0;
          meas_assigned[best_m] = (int)ti;
          track_assigned[ti] = best_m;
        }
      }
      // Create new tracks for unmatched measurements
      for (size_t mi=0; mi<measured_objs.size(); ++mi){ if (meas_assigned[mi] != -1) continue;
        ObjectOut t = measured_objs[mi];
        t.id = next_track_id++;
        tracks.push_back(t);
        track_missed.push_back(0);
      }
      // Ensure assignment list covers newly created tracks
      if (track_assigned.size() < tracks.size()) track_assigned.resize(tracks.size(), -1);
      // Age and remove stale tracks
      std::vector<ObjectOut> kept;
      std::vector<int> kept_missed;
      for (size_t ti=0; ti<tracks.size(); ++ti){
        if (track_assigned[ti] == -1) track_missed[ti]++;
        if (track_missed[ti] <= cfg.tracking_max_missed) { kept.push_back(tracks[ti]); kept_missed.push_back(track_missed[ti]); }
      }
      tracks.swap(kept); track_missed.swap(kept_missed);
      // Output the current tracks
      out_objs = tracks;
    } else {
      out_objs.swap(measured_objs);
    }

    // Flatten points for stream (optionally downsample more for viz)
    std::vector<float> ground_xyz; std::vector<float> ground_I;
    if (cfg.stream_ground_points && ground){
      ground_xyz.reserve(ground->size()*3); ground_I.reserve(ground->size());
      for (const auto& p : *ground){ ground_xyz.push_back(p.x); ground_xyz.push_back(p.y); ground_xyz.push_back(p.z); ground_I.push_back(p.intensity); }
    }
    std::vector<float> pts_xyz; pts_xyz.reserve(nonground->size()*3);
    std::vector<float> pts_I;   pts_I.reserve(nonground->size());
    for (const auto& p : *nonground){ pts_xyz.push_back(p.x); pts_xyz.push_back(p.y); pts_xyz.push_back(p.z); pts_I.push_back(p.intensity); }

  // imu_copy already made above
  pack_and_send(ws, frame_id.fetch_add(1), f.t1, imu_copy, out_objs, ground_xyz, ground_I, pts_xyz, pts_I);
#else
    (void)f; // if PCL disabled
#endif
  };

  if (!drv.start(on_lidar, on_imu)){
    std::cerr << "Failed to start L2 driver.\n"; return 1;
  }

  std::cout << "object_detection_pcl_live running. WS on port " << cfg.ws_port << "\n";
  // Keep alive until Ctrl-C
  while (true) std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
