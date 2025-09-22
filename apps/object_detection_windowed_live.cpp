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
#endif

using namespace fastlio2;

// Windowed accumulation settings
struct WindowCfg {
  // Angle binning
  bool  enable_angle_window = true;
  float angle_bin_deg = 5.0f;         // per-bin size
  float window_width_deg = 20.0f;     // total angle window width
  float window_overlap_deg = 10.0f;   // overlap between adjacent windows
  float edge_guard_deg = 3.0f;        // ensure completeness at window edges
  // Time binning
  bool  enable_time_window = true;
  float window_time_s = 0.4f;         // time window
  float window_overlap_s = 0.2f;      // time overlap
  // Limits
  int   max_points = 120000;
};

struct RuntimeCfg {
  // detection
  float voxel_size = 0.05f;
  bool  remove_ground = true;
  float ground_threshold = 0.10f;
  bool  stream_ground_points = true;
  // default ground color for viz (RGB 0-255)
  uint8_t ground_color_r = 80, ground_color_g = 160, ground_color_b = 255;
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
  float radial_min_range = 0.2f;
  float radial_max_range = 20.0f;
  float radial_min_angle = -180.0f;
  float radial_max_angle =  180.0f;
  bool  imu_rotate_points = true;
  int   ws_port = 8081;

  // Simple accumulation (alternative/overlay to windowed settings)
  // If >0, retain up to this many recent frames worth of points
  int   accumulate_frames = 0;         
  // If >0, retain points within this time horizon (seconds)
  float accumulate_seconds = 0.0f;     
  // Safety cap on total accumulated points kept in memory
  int   max_accumulated_points = 0;    

  WindowCfg win;
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
        if (v.size() == 3){
          cfg.ground_color_r = (uint8_t)std::clamp(v[0],0,255);
          cfg.ground_color_g = (uint8_t)std::clamp(v[1],0,255);
          cfg.ground_color_b = (uint8_t)std::clamp(v[2],0,255);
        }
      }
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
      // Simple accumulation controls at object_detection level (optional)
      if (od["accumulate_frames"]) cfg.accumulate_frames = od["accumulate_frames"].as<int>(cfg.accumulate_frames);
      if (od["accumulate_seconds"]) cfg.accumulate_seconds = od["accumulate_seconds"].as<float>(cfg.accumulate_seconds);
      if (od["max_accumulated_points"]) cfg.max_accumulated_points = od["max_accumulated_points"].as<int>(cfg.max_accumulated_points);
      // windowed options under object_detection.window
      if (od["window"]) {
        auto w = od["window"];
        if (w["enable_angle_window"]) cfg.win.enable_angle_window = w["enable_angle_window"].as<bool>(cfg.win.enable_angle_window);
        if (w["angle_bin_deg"]) cfg.win.angle_bin_deg = w["angle_bin_deg"].as<float>(cfg.win.angle_bin_deg);
        if (w["window_width_deg"]) cfg.win.window_width_deg = w["window_width_deg"].as<float>(cfg.win.window_width_deg);
        if (w["window_overlap_deg"]) cfg.win.window_overlap_deg = w["window_overlap_deg"].as<float>(cfg.win.window_overlap_deg);
        if (w["edge_guard_deg"]) cfg.win.edge_guard_deg = w["edge_guard_deg"].as<float>(cfg.win.edge_guard_deg);
        if (w["enable_time_window"]) cfg.win.enable_time_window = w["enable_time_window"].as<bool>(cfg.win.enable_time_window);
        if (w["window_time_s"]) cfg.win.window_time_s = w["window_time_s"].as<float>(cfg.win.window_time_s);
        if (w["window_overlap_s"]) cfg.win.window_overlap_s = w["window_overlap_s"].as<float>(cfg.win.window_overlap_s);
        if (w["max_points"]) cfg.win.max_points = w["max_points"].as<int>(cfg.win.max_points);
      }
    }
  } catch (const std::exception& e){
    std::cerr << "Failed to load config: " << e.what() << ". Using defaults.\n";
  }
  return cfg;
}

struct ImuState { bool calibrated=false, stationary=true; float gx=0,gy=0,gz=1; float motion_mag=0; float qw=1,qx=0,qy=0,qz=0; };

struct ObjectOut { float cx,cy,cz,w,h,d,volume,distance,confidence; int count,id; std::array<float,24> bbox_corners; };

static void append_u32(std::vector<uint8_t>& v, uint32_t x){ for(int i=0;i<4;i++) v.push_back((x>>(8*i))&0xFF); }
static void append_i32(std::vector<uint8_t>& v, int32_t x){ append_u32(v, (uint32_t)x); }
static void append_f32(std::vector<uint8_t>& v, float x){ uint32_t u; std::memcpy(&u,&x,4); append_u32(v,u);} 
static void append_f64(std::vector<uint8_t>& v, double x){ uint64_t u; std::memcpy(&u,&x,8); for(int i=0;i<8;i++) v.push_back((u>>(8*i))&0xFF);} 

static void pack_and_send(WsServer& ws, uint32_t frame_id, double stamp,
                          const ImuState& imu,
                          const std::vector<ObjectOut>& objs,
                          const std::vector<float>& ground_xyz,
                          const std::vector<float>& ground_I,
                          const std::vector<float>& pts_xyz,
                          const std::vector<float>& pts_I,
                          uint8_t ground_r,
                          uint8_t ground_g,
                          uint8_t ground_b){
  std::vector<uint8_t> buf; buf.reserve(68 + objs.size()* (44 + 24*4) + (ground_I.size()+pts_I.size())*16/1);
  append_u32(buf, frame_id); append_f64(buf, stamp);
  append_u32(buf, (uint32_t)objs.size());
  append_u32(buf, (uint32_t)pts_I.size());
  append_u32(buf, (uint32_t)ground_I.size());
  buf.push_back(imu.calibrated?1:0); buf.push_back(imu.stationary?1:0);
  append_f32(buf, imu.gx); append_f32(buf, imu.gy); append_f32(buf, imu.gz);
  append_f32(buf, imu.motion_mag);
  append_f32(buf, imu.qw); append_f32(buf, imu.qx); append_f32(buf, imu.qy); append_f32(buf, imu.qz);
  // Ground RGB (3 bytes) + 1 reserved for alignment
  buf.push_back(ground_r);
  buf.push_back(ground_g);
  buf.push_back(ground_b);
  buf.push_back(0);
  for (const auto& o : objs){
    append_f32(buf,o.cx); append_f32(buf,o.cy); append_f32(buf,o.cz);
    append_f32(buf,o.w); append_f32(buf,o.h); append_f32(buf,o.d);
    append_f32(buf,o.volume); append_f32(buf,o.distance); append_f32(buf,o.confidence);
    append_i32(buf,o.count); append_i32(buf,o.id);
    for(int i=0;i<24;i++) append_f32(buf, o.bbox_corners[i]);
  }
  for (size_t i=0;i<ground_I.size();++i){ append_f32(buf, ground_xyz[3*i+0]); append_f32(buf, ground_xyz[3*i+1]); append_f32(buf, ground_xyz[3*i+2]); append_f32(buf, ground_I[i]); }
  for (size_t i=0;i<pts_I.size();++i){ append_f32(buf, pts_xyz[3*i+0]); append_f32(buf, pts_xyz[3*i+1]); append_f32(buf, pts_xyz[3*i+2]); append_f32(buf, pts_I[i]); }
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

static float wrap_angle_deg(float a){ while(a <= -180.f) a += 360.f; while(a > 180.f) a -= 360.f; return a; }
static float atan2_deg(float y, float x){ return std::atan2(y,x) * 180.0f / (float)M_PI; }

int main(int argc, char** argv){
  std::string cfg_path = "config/unilidar_l2.yaml"; if (argc>1) cfg_path = argv[1];
  RuntimeCfg cfg = load_cfg(cfg_path);

  WsServer ws; if (cfg.ws_port>0) ws.start((uint16_t)cfg.ws_port);
  std::atomic<uint32_t> frame_id{0};
  std::mutex mtx_imu; ImuState imu;

  L2Driver::Config dcfg; L2Driver drv(dcfg);
  auto on_imu = [&](const ImuSample& s){ std::lock_guard<std::mutex> lk(mtx_imu); imu.gx=s.ax; imu.gy=s.ay; imu.gz=s.az; float gnorm=std::sqrt(imu.gx*imu.gx+imu.gy*imu.gy+imu.gz*imu.gz)+1e-9f; imu.stationary=(std::fabs(gnorm-9.81f)<0.5f); imu.calibrated=true; imu.qw=s.qw; imu.qx=s.qx; imu.qy=s.qy; imu.qz=s.qz; imu.motion_mag=std::fabs(gnorm-9.81f); };

  // Window buffers: keep a deque of points with their angle and time tags
  struct TaggedPoint { P p; float ang_deg; double t; uint32_t frm; };
  std::deque<TaggedPoint> ring;
  size_t ring_points = 0;
  // rolling window center (deg) advances each callback to sweep 360
  float rolling_center_deg = -180.0f;
  // local frame counter for accumulation by frames
  uint32_t local_frame_counter = 0;

  auto on_lidar = [&](const LidarFrame& f){
#ifdef FASTLIO2_USE_PCL
    // Increment local frame id for accumulation gating
    const uint32_t this_frame = local_frame_counter++;
    // Convert and tag
    ImuState imu_copy; { std::lock_guard<std::mutex> lk(mtx_imu); imu_copy = imu; }
    float qw=imu_copy.qw, qx=imu_copy.qx, qy=imu_copy.qy, qz=imu_copy.qz; float nq=std::sqrt(qw*qw+qx*qx+qy*qy+qz*qz); if(nq>1e-6f){qw/=nq;qx/=nq;qy/=nq;qz/=nq;} else {qw=1;qx=qy=qz=0;}
    float iw=qw, ix=-qx, iy=-qy, iz=-qz;
    auto rotate = [&](float x, float y, float z){ float xx=1-2*(iy*iy+iz*iz); float xy=2*(ix*iy-iz*iw); float xz=2*(ix*iz+iy*iw); float yx=2*(ix*iy+iz*iw); float yy=1-2*(ix*ix+iz*iz); float yz=2*(iy*iz-ix*iw); float zx=2*(ix*iz-iy*iw); float zy=2*(iy*iz+ix*iw); float zz=1-2*(ix*ix+iy*iy); return std::array<float,3>{ xx*x+xy*y+xz*z, yx*x+yy*y+yz*z, zx*x+zy*y+zz*z}; };

    // push into ring
    for (const auto& pt : f.pts){
      P p; if (cfg.imu_rotate_points){ auto v=rotate(pt.x,pt.y,pt.z); p.x=v[0]; p.y=v[1]; p.z=v[2]; } else { p.x=pt.x; p.y=pt.y; p.z=pt.z; }
      p.intensity = pt.intensity;
      if (cfg.enable_aoi){ if (p.x<cfg.aoi_min_x||p.x>cfg.aoi_max_x||p.y<cfg.aoi_min_y||p.y>cfg.aoi_max_y||p.z<cfg.aoi_min_z||p.z>cfg.aoi_max_z) continue; }
      if (cfg.enable_radial_filter){ float r=std::sqrt(p.x*p.x+p.y*p.y); if (r<cfg.radial_min_range||r>cfg.radial_max_range) continue; float ang=std::atan2(p.y,p.x); float ang_min=cfg.radial_min_angle*(float)M_PI/180.f, ang_max=cfg.radial_max_angle*(float)M_PI/180.f; if(ang<ang_min||ang>ang_max) continue; }
      if (point_range(p) > cfg.max_detection_range) continue;
      float ang_deg = atan2_deg(p.y, p.x);
      TaggedPoint tp{p, ang_deg, f.t0 + pt.t_rel, this_frame};
      ring.push_back(tp); ring_points++;
      // Immediate hard cap by max points to avoid growth
      int eff_max_points = cfg.max_accumulated_points > 0 ? cfg.max_accumulated_points : cfg.win.max_points;
      while ((int)ring_points > eff_max_points){ ring.pop_front(); ring_points--; }
    }

    // Evict old by time horizon if configured
    double eff_seconds = 0.0;
    if (cfg.accumulate_seconds > 0.0f) eff_seconds = cfg.accumulate_seconds;
    else if (cfg.win.enable_time_window) eff_seconds = (cfg.win.window_time_s + cfg.win.window_overlap_s);
    if (eff_seconds > 0.0){
      const double keep_from = f.t1 - eff_seconds;
      while(!ring.empty() && ring.front().t < keep_from){ ring.pop_front(); ring_points--; }
    }

    // Evict old by frame count if configured
    if (cfg.accumulate_frames > 0){
      const uint32_t min_keep_frame = (this_frame >= (uint32_t)(cfg.accumulate_frames - 1)) ? (this_frame - (uint32_t)(cfg.accumulate_frames - 1)) : 0u;
      while(!ring.empty() && ring.front().frm < min_keep_frame){ ring.pop_front(); ring_points--; }
    }

    // Build current selection over full 360° using overlapping angular windows and time guards
    std::vector<P> window_points; window_points.reserve(std::min<size_t>(ring_points * 2, (size_t)cfg.win.max_points * 2));
    double time_keep_from = -std::numeric_limits<double>::infinity();
    if (eff_seconds > 0.0) time_keep_from = f.t1 - eff_seconds;

    const bool use_angle = cfg.win.enable_angle_window && (cfg.win.window_width_deg < 359.0f);
    if (use_angle){
      const float half = 0.5f * cfg.win.window_width_deg;
      const float guard = cfg.win.edge_guard_deg;
      const float step = std::max(1.0f, cfg.win.window_width_deg - cfg.win.window_overlap_deg);
      // Use a single rolling center per callback to keep latency low
      float center = rolling_center_deg;
      for (const auto& tp : ring){
        if (tp.t < time_keep_from) continue;
        float da = wrap_angle_deg(tp.ang_deg - center);
        if (da >= -half - guard && da <= half + guard){ window_points.push_back(tp.p); }
      }
      // advance center for next callback
      rolling_center_deg = wrap_angle_deg(rolling_center_deg + step);
    } else {
      // Full 360°: just respect the time window
      for (const auto& tp : ring){ if (tp.t >= time_keep_from) window_points.push_back(tp.p); }
    }

    // Downsample
  pcl::PointCloud<P>::Ptr cloud(new pcl::PointCloud<P>());
  cloud->reserve(window_points.size());
  for (const auto& p : window_points) cloud->push_back(p);
  cloud->width = cloud->points.size(); cloud->height = 1;
    if (cfg.voxel_size > 1e-6f){ pcl::VoxelGrid<P> vg; vg.setLeafSize(cfg.voxel_size,cfg.voxel_size,cfg.voxel_size); vg.setInputCloud(cloud); pcl::PointCloud<P>::Ptr filtered(new pcl::PointCloud<P>()); vg.filter(*filtered); cloud.swap(filtered); }

    // Ground removal
    pcl::PointCloud<P>::Ptr nonground(new pcl::PointCloud<P>()); pcl::PointCloud<P>::Ptr ground(new pcl::PointCloud<P>());
    if (cfg.remove_ground && cloud->size() >= 50){ pcl::SACSegmentation<P> seg; seg.setOptimizeCoefficients(true); seg.setModelType(pcl::SACMODEL_PLANE); seg.setMethodType(pcl::SAC_RANSAC); seg.setDistanceThreshold(cfg.ground_threshold); seg.setMaxIterations(100); seg.setInputCloud(cloud); pcl::ModelCoefficients coeff; pcl::PointIndices::Ptr inliers(new pcl::PointIndices()); seg.segment(*inliers, coeff); if(!inliers->indices.empty()){ pcl::ExtractIndices<P> ex; ex.setInputCloud(cloud); ex.setIndices(inliers); ex.setNegative(true); ex.filter(*nonground); ex.setNegative(false); ex.filter(*ground);} else { *nonground = *cloud; } } else { *nonground = *cloud; }

    // Clustering: use DBSCAN only
    std::vector<pcl::PointIndices> clusters;
    if (!nonground->empty()){
      dbscan(nonground, cfg.dbscan_eps, cfg.dbscan_min_points, clusters);
      if (cfg.max_cluster_size > 0){
        std::vector<pcl::PointIndices> filtered; filtered.reserve(clusters.size());
        for (auto &c : clusters) if ((int)c.indices.size() >= cfg.min_cluster_size && (int)c.indices.size() <= cfg.max_cluster_size) filtered.push_back(std::move(c));
        clusters.swap(filtered);
      }
    }

    // Objects
  std::vector<ObjectOut> out_objs; int obj_id=0; for(const auto& ind: clusters){ float minx=1e9,miny=1e9,minz=1e9,maxx=-1e9,maxy=-1e9,maxz=-1e9; for(int idx: ind.indices){ const auto& p=(*nonground)[idx]; minx=std::min(minx,p.x); miny=std::min(miny,p.y); minz=std::min(minz,p.z); maxx=std::max(maxx,p.x); maxy=std::max(maxy,p.y); maxz=std::max(maxz,p.z);} float cx=0.5f*(minx+maxx), cy=0.5f*(miny+maxy), cz=0.5f*(minz+maxz); float w=std::max(1e-3f, maxx-minx), h=std::max(1e-3f, maxy-miny), d=std::max(1e-3f, maxz-minz); float vol=w*h*d; if(vol<cfg.min_object_volume||vol>cfg.max_object_volume) continue; float dist=std::sqrt(cx*cx+cy*cy+cz*cz); ObjectOut o; o.cx=cx;o.cy=cy;o.cz=cz;o.w=w;o.h=h;o.d=d;o.volume=vol;o.distance=dist;o.count=(int)ind.indices.size(); o.id=obj_id++; o.confidence=std::max(0.0f,std::min(1.0f,1.0f-dist/cfg.max_detection_range))*std::min(1.0f,(float)o.count/500.0f); std::array<std::array<float,3>,8> corners={{{minx,miny,minz},{maxx,miny,minz},{maxx,maxy,minz},{minx,maxy,minz},{minx,miny,maxz},{maxx,miny,maxz},{maxx,maxy,maxz},{minx,maxy,maxz}}}; int k=0; for(int i=0;i<8;i++){ o.bbox_corners[k++]=corners[i][0]; o.bbox_corners[k++]=corners[i][1]; o.bbox_corners[k++]=corners[i][2]; } out_objs.push_back(o);} 

    // Pack points (ground, then nonground)
    std::vector<float> ground_xyz; std::vector<float> ground_I; if(cfg.stream_ground_points && ground){ ground_xyz.reserve(ground->size()*3); ground_I.reserve(ground->size()); for(const auto& p:*ground){ ground_xyz.push_back(p.x); ground_xyz.push_back(p.y); ground_xyz.push_back(p.z); ground_I.push_back(p.intensity);} }
    std::vector<float> pts_xyz; pts_xyz.reserve(nonground->size()*3); std::vector<float> pts_I; pts_I.reserve(nonground->size()); for(const auto& p:*nonground){ pts_xyz.push_back(p.x); pts_xyz.push_back(p.y); pts_xyz.push_back(p.z); pts_I.push_back(p.intensity);} 

    pack_and_send(ws, frame_id.fetch_add(1), f.t1, imu_copy, out_objs, ground_xyz, ground_I, pts_xyz, pts_I,
                  cfg.ground_color_r, cfg.ground_color_g, cfg.ground_color_b);
#endif
  };

  if (!drv.start(on_lidar, on_imu)) { std::cerr << "Failed to start L2 driver.\n"; return 1; }
  std::cout << "object_detection_windowed_live running. WS on port " << cfg.ws_port << "\n";
  while(true) std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
