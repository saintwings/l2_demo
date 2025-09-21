# FastLIO2 L2 Demo – Ground Object Detection (Live)# L2 LiDAR Ground Object Detection Demo



A lightweight live object detection demo for Unitree L2 LiDAR using C++17 and PCL, with a Three.js viewer. It ingests LiDAR + IMU from the Unitree SDK, performs ground segmentation and clustering, and streams detections and point clouds to the browser in real time.This workspace provides a live object detection app using Unitree L2 LiDAR and PCL for ground removal and clustering. Results stream via a built-in WebSocket server to a Three.js viewer under `viz/`.



## Features## Build



- Live LiDAR ingest via Unitree L2 SDK v2 (IMU supported)Prereqs (Ubuntu/Debian):

- Optional IMU-based rotation of points into an inertial frame- Build tools: g++, cmake, make

- Voxel downsampling, AOI and radial filters- PCL dev: `sudo apt install libpcl-dev`

- RANSAC plane removal for ground separation- Eigen: `sudo apt install libeigen3-dev`

- Two clustering methods:- Boost system: `sudo apt install libboost-system-dev`

  - Euclidean cluster extraction (PCL)- YAML-CPP: `sudo apt install libyaml-cpp-dev`

  - DBSCAN (KD-tree radius search, configurable)

- Rolling accumulation (frame/time) to mitigate swing-angle gapsThen build:

- Alternate app with windowed time/angle accumulation for lower latency

- Simple multi-frame tracking with EMA smoothing```

- Binary WebSocket protocol to a Three.js viewer (ground/non-ground split, IMU axes)mkdir -p build

cd build

## Repo Layoutcmake .. -DUSE_UNITREE_SDK=1

cmake --build . -j

- `apps/object_detection_pcl_live.cpp` – Main live detection app```

- `apps/object_detection_windowed_live.cpp` – Windowed accumulation variant

- `include/fastlio2/*.hpp` – Driver, server, and types headersArtifacts:

- `src/*.cpp` – Unitree L2 driver wrapper and WebSocket server- `build/object_detection_pcl_live`

- `config/unilidar_l2.yaml` – All runtime tuning knobs

- `viz/server.js` and `viz/public/object_detection.html` – Dev HTTP viewer## Run (live)

- `CMakeLists.txt` – Build config

1) Connect the L2 (UDP or Serial). Adjust IP/ports in `apps/object_detection_pcl_live.cpp` (L2Driver::Config) if needed.

## Build2) Start the app:



Prereqs (Ubuntu-like): Eigen3, yaml-cpp, Boost.System, PCL (headers + libs), CMake ≥ 3.16, a C++17 compiler. Unitree SDK binaries are bundled in `third_party/unilidar_sdk2` (x86_64/aarch64).```

./build/object_detection_pcl_live config/unilidar_l2.yaml

Quick build:```



```bashThis hosts a WebSocket server on the port configured by `deskew.ws_port` (default 8081).

cmake -S . -B build -DUSE_UNITREE_SDK=1

cmake --build build -j3) Start the viewer HTTP server (optional dev server):

```

```

Targets:cd viz

- `object_detection_pcl_live`npm install

- `object_detection_windowed_live`node server.js

```

## Run

Open http://localhost:8080/object_detection.html (it auto-connects to ws://localhost:8081).

1) Start the viewer HTTP server (serves Three.js page at port 8080):

### Show removed ground (optional)

```bash

node viz/server.jsYou can stream the removed ground points to the viewer for debugging:

```

- Set in `config/unilidar_l2.yaml` under `object_detection`:

Open http://localhost:8080 in your browser.	- `stream_ground_points: true`

	- Optional fixed color: `ground_color_rgb: [80, 160, 255]`

2) Run one of the apps (default WS port: 8081). The viewer will auto-connect.- In the viewer, use the new checkbox “Show Ground” to toggle rendering. Ground points are shown as a second point cloud with a single color, separate from the main (non-ground) cloud.



```bashNote: Streaming ground increases bandwidth. Disable if you only care about non-ground objects.

./build/object_detection_pcl_live config/unilidar_l2.yaml

# or## Viewer URL options

./build/object_detection_windowed_live config/unilidar_l2.yaml

```- `?port=8081` select WS port

- AOI/radial filters via URL to visualize: `?aoi=1&aoi_min_x=-5&aoi_max_x=5&radial=1&radial_min_range=0.5&radial_max_range=20&radial_min_angle=-60&radial_max_angle=60`

Note: If you need a different WS port, set `deskew.ws_port` in YAML.- Mount orientation (for axes): `?mount_roll_deg=0&mount_pitch_deg=0&mount_yaw_deg=0&mount_flip_z=0`



## Configuration (config/unilidar_l2.yaml)## Notes



Key sections under `object_detection`:- IMU handling is minimal; gravity and orientation are placeholders. You can extend `on_imu` to compute a proper orientation estimate.

- Ground plane removal uses simple RANSAC with fixed threshold from config. For sloped ground or moving base, consider using gravity alignment or a more robust method.

- General- Optionally stream the ground points (see above) for inspection of RANSAC segmentation quality.

  - `voxel_size`: leaf size for downsampling- The app is compiled only if PCL is available (FASTLIO2_USE_PCL ON). The pipeline is guarded by `#ifdef FASTLIO2_USE_PCL`.

  - `max_detection_range`: drop far points early
  - `enable_aoi` + `aoi_*`: rectangular region filter
  - `enable_radial_filter` + `radial_*`: polar filter
  - `imu_rotate_points`: rotate points using incoming IMU quaternion
- Ground
  - `remove_ground`: enable RANSAC plane removal
  - `ground_threshold`: inlier distance for ground plane
  - `stream_ground_points`: send ground points to viewer
- Clustering
  - `clustering_method`: `euclidean` or `dbscan`
  - For Euclidean: `cluster_tolerance`, `min_cluster_size`, `max_cluster_size`
  - For DBSCAN: `dbscan_eps`, `dbscan_min_points` (post-filter still uses min/max_cluster_size)
- Accumulation (pcl_live)
  - `accumulate_frames`, `accumulate_seconds`, `max_accumulated_points`
- Tracking
  - `tracking_enabled`, `tracking_max_assoc_dist`, `tracking_alpha`, `tracking_max_missed`
- Windowed app (`object_detection.window`)
  - `enable_angle_window`, `window_width_deg`, `window_overlap_deg`, `edge_guard_deg`
  - `enable_time_window`, `window_time_s`, `window_overlap_s`, `max_points`

Tip: Start with `voxel_size: 0.05`–`0.10`. For DBSCAN, pick `dbscan_eps` about 1–2× `voxel_size` and `dbscan_min_points` ~6–12.

## Viewer

- Serves from `viz/server.js` (Node/Express). Open http://localhost:8080.
- Renders IMU axes, ground (toggleable), non-ground points, and bounding boxes.
- Uses a binary WS stream: [frameId][timestamp][numObjects][numNonGround][numGround][IMU…] followed by objects, ground points, and non-ground points. IMU includes flags, gravity vector, motion magnitude, and quaternion.

## Troubleshooting

- Build: ensure PCL headers are in `/usr/include/pcl-1.xx` and Eigen in `/usr/include/eigen3`. The `CMakeLists.txt` tries common include paths and links minimal PCL libs.
- Linking Unitree SDK: the correct arch lib dir is added automatically (x86_64/aarch64). If you have a custom SDK path, adjust `link_directories` in CMake or update your environment.
- No points / empty view: check `max_detection_range`, AOI/radial filters, or try disabling `remove_ground`.
- Performance:
  - Increase `voxel_size` or lower `max_points`/window time.
  - Prefer Euclidean clustering if DBSCAN is too heavy for your CPU.

## License

This demo includes the Unitree SDK redistributables under `third_party/unilidar_sdk2` (see their LICENSE). Demo code is provided as-is for evaluation.
