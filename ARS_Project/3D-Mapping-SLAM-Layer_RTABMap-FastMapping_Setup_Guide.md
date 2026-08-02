# 3D Mapping & SLAM Layer — RTAB-Map + FastMapping (ROS 2 Jazzy, Ubuntu 24.04)

**Continues from:** `RealSense-D455_YDLidar-X2_Setup_Guide.md` (Parts A–C). This assumes the
D455 and YDLidar X2 are already publishing `/camera/camera/...` (or flat `/camera/...`) and
`/scan` reliably, as verified in that guide's Part C.

**Scope check against `Robofun-Training-Guide_2024.pdf`:** The deck's "3D mapping – RTAB-Map"
and "Octomap generation with FastMapping & RTAB-Map" sections (pages ~83–94) are the "3D
Mapping & SLAM layer" — visual SLAM fusion (RTAB-Map) feeding a 3D voxel/OctoMap builder
(FastMapping). That's what this guide covers. The deck's earlier 2D "SLAM" section
(cartographer occupancy-grid mapping, pages ~56–67) is a separate 2D lidar-only layer used
later for Nav2 localization — out of scope here, happy to write that one up too if you want it.

---

## Is the training guide's implementation still available? What changed

| Training guide (2024, ROS 2 Humble, AAEON/Scuttle AMR) | Current state (verified July 2026) |
|---|---|
| `ros2 run rtabmap_slam rtabmap --ros-args -r odom:=odom/amr ...` — raw node, assumes wheel-encoder odometry (`odom/amr`) from an AMR base | Package and executable **still exist and work** (`rtabmap_slam`/`rtabmap`), but a DIY camera+lidar rig has no wheel odometry topic. Current best practice is the official `rtabmap_launch`/`rtabmap.launch.py` launch file, which can generate its own **visual odometry** from the RealSense stream (`visual_odometry:=true`, the default) — no wheel encoders needed. |
| `ros2 run fast_mapping fast_mapping_node --ros-args -r ...` with multi-camera remaps | **Still current** — same package, same node name, same default output topics (`world/fused_map`, `world/occupancy`, `world/map`). Confirmed from the live source at `github.com/open-edge-platform/edge-ai-suites` (Apache-2.0, active, explicit Jazzy support). |
| Install via **Intel Robotics SDK 2.1**: register an account on Intel Edge Software Hub, accept a license, get a personalized apt-repo URL | The product has been rebranded/open-sourced as **"Autonomous Mobile Robot"** under Intel's **Robotics AI Suite** (Open Edge Platform). Packages now install from two public apt feeds (`eci.intel.com`, `amrdocs.intel.com`) with **no account/registration gate** — just add the repo and `apt install`. Package names changed from `ros-humble-robotics-sdk` to distro-qualified names like `ros-jazzy-fast-mapping`, `ros-jazzy-rtabmap-ros`. |
| Target: ROS 2 Humble / Ubuntu 22.04 | **Jazzy / Ubuntu 24.04 is now the officially documented target** for this stack — matches your setup exactly. |
| Camera topics `camera/color/image_raw`, `camera/aligned_depth_to_color/image_raw` (flat) | Same flat names still used by RTAB-Map's and FastMapping's *defaults* — which is why Part D below launches the camera with `camera_namespace:=/` (as your Part A already noted), rather than fighting the doubled `/camera/camera/...` namespace. |

Bottom line: **nothing in this layer is dead or unavailable** — it just moved to a new,
easier-to-reach apt feed and dropped the "wheel odometry" assumption, which we replace with
visual odometry from the D455.

Source verified against:
- FastMapping tutorial: `docs.openedgeplatform.intel.com/.../navigation/run-fastmapping-algorithm.html`
- FastMapping source: `github.com/open-edge-platform/edge-ai-suites/tree/main/robotics-ai-suite/components/fast-mapping`
- RTAB-Map ROS 2 package: `docs.ros.org/en/jazzy/p/rtabmap/`, `github.com/introlab/rtabmap_ros`
- AMR apt repo setup: `docs.openedgeplatform.intel.com/.../robotics/gsg_robot/index.html`

---

## Part D — 3D Mapping & SLAM Layer

**Pipeline:** RealSense (RGB-D) + YDLidar (`/scan`) → static TF between the two sensors →
**RTAB-Map** (visual odometry + appearance-based SLAM, lidar-assisted) → **FastMapping**
(3D OctoMap voxel grid) → RViz2.

### D1. Publish the static transform between the camera and the lidar

RTAB-Map needs to know the fixed physical offset between the camera and the lidar (neither
driver publishes this automatically — it's specific to how you mounted them). Measure the
offset (in meters) from `camera_link` to `laser_frame` and publish it as a static transform:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 0.05 --roll 0 --pitch 0 --yaw 0 \
  --frame-id camera_link --child-frame-id laser_frame
```

Replace the `--x/--y/--z/--roll/--pitch/--yaw` values with your actual measured offset
(e.g. lidar mounted 5 cm above and 3 cm behind the camera). Keep this running in its own
terminal for the whole session — everything downstream depends on it.

> If you don't have wheel encoders / an AMR base, `camera_link` doubles as your robot's
> tracked reference frame in the steps below (that's what `frame_id:=camera_link` means in D6).

### D2. Install RTAB-Map

This ships as part of the standard ROS 2 Jazzy package index — no extra repo needed if your
normal ROS 2 apt sources are already set up:

```bash
sudo apt update
sudo apt install -y ros-jazzy-rtabmap-ros
```

### D3. Add Intel's apt feed and install FastMapping

FastMapping is Intel's package, distributed from Intel's own apt feed (not the default Ubuntu/
ROS archives). As of the current docs, this no longer needs an Edge Software Hub account —
just add the signed repo:

```bash
# GPG key
sudo -E wget -O- https://eci.intel.com/repos/gpg-keys/GPG-PUB-KEY-INTEL-ECI.gpg \
  | sudo tee /usr/share/keyrings/eci-archive-keyring.gpg > /dev/null

# Repos
echo "deb [signed-by=/usr/share/keyrings/eci-archive-keyring.gpg] https://eci.intel.com/repos/$(source /etc/os-release && echo $VERSION_CODENAME) isar main" \
  | sudo tee /etc/apt/sources.list.d/eci.list > /dev/null
echo "deb-src [signed-by=/usr/share/keyrings/eci-archive-keyring.gpg] https://eci.intel.com/repos/$(source /etc/os-release && echo $VERSION_CODENAME) isar main" \
  | sudo tee -a /etc/apt/sources.list.d/eci.list > /dev/null
echo "deb [signed-by=/usr/share/keyrings/eci-archive-keyring.gpg] https://amrdocs.intel.com/repos/$(source /etc/os-release && echo $VERSION_CODENAME) amr main" \
  | sudo tee /etc/apt/sources.list.d/amr.list > /dev/null
echo "deb-src [signed-by=/usr/share/keyrings/eci-archive-keyring.gpg] https://amrdocs.intel.com/repos/$(source /etc/os-release && echo $VERSION_CODENAME) amr main" \
  | sudo tee -a /etc/apt/sources.list.d/amr.list > /dev/null

# Priority pinning (recommended by Intel so this feed doesn't fight your existing repos)
echo -e "Package: *\nPin: origin eci.intel.com\nPin-Priority: 1000" | sudo tee /etc/apt/preferences.d/isar
echo -e "Package: *\nPin: origin amrdocs.intel.com\nPin-Priority: 1001" | sudo tee /etc/apt/preferences.d/amr

sudo apt update
sudo apt install -y ros-jazzy-fast-mapping
```

> **Note:** installing `ros-jazzy-fast-mapping` will also pull in `ros-jazzy-rtabmap-ros` and
> a small bag-file package (`ros-jazzy-bagfile-spinning`) as dependencies if you skipped D2.

### D4. Launch the RealSense camera with flat topic names, low-bandwidth profile

Force flat topic names (`/camera/color/image_raw`, not the doubled `/camera/camera/...`) so
they match both RTAB-Map's and FastMapping's default topic names — no remapping needed later.

This layer runs the camera **and** the lidar **and** RTAB-Map's own odometry all at once, which
is exactly the bandwidth-contention scenario flagged in your Part C1 (camera + lidar sharing one
USB hub → lidar checksum/timeout errors). So by default, launch the camera at the low-bandwidth
profile from Part C1 — drop the point cloud (the heaviest stream, and RTAB-Map/FastMapping build
their own maps so they don't need it) and use reduced-resolution color/depth streams:

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true align_depth.enable:=true \
  pointcloud.enable:=false \
  rgb_camera.color_profile:=424x240x15 \
  depth_module.depth_profile:=480x270x15 \
  camera_namespace:=/ camera_name:=camera
```

Verify:
```bash
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw
```
You should see `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw`,
`/camera/color/camera_info`, `/camera/aligned_depth_to_color/camera_info`, all at ~15 fps.

> **If you have a genuine dedicated USB3 link for the camera** (separate hub/port from the
> lidar, or a PCIe USB3 card — see your Part C1), you can drop the `rgb_camera.color_profile`
> and `depth_module.depth_profile` overrides and run at the camera's native resolution/fps
> instead; RTAB-Map and FastMapping will happily take the extra detail. Check first with
> `lsusb -t` to confirm the camera and lidar aren't nested under the same hub.
>
> First run a profile list to confirm these exact strings are supported by your D455's
> firmware (profile lists vary by firmware and differ for color vs. depth):
> ```bash
> ros2 param describe /camera/camera rgb_camera.color_profile
> ros2 param describe /camera/camera depth_module.depth_profile
> ```

### D5. Launch the YDLidar (if not already running)

From your Part B setup:
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

### D6. Launch RTAB-Map — visual odometry + lidar-assisted SLAM fusion

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  args:="--delete_db_on_start" \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  frame_id:=camera_link \
  approx_sync:=true \
  visual_odometry:=true \
  subscribe_scan:=true \
  scan_topic:=/scan \
  qos:=2 \
  rtabmap_viz:=false \
  rviz:=false
```

What this does:
- `visual_odometry:=true` (the default) launches RTAB-Map's own `rgbd_odometry` node, so pose
  tracking comes straight from the D455's RGB-D stream — you don't need wheel encoders.
- `subscribe_scan:=true` + `scan_topic:=/scan` feeds the YDLidar in as well, which RTAB-Map uses
  to build a cleaner 2D occupancy grid and assist loop closure (this is why D1's static TF
  between `camera_link` and `laser_frame` matters).
- `frame_id:=camera_link` — with no wheel base, the camera itself is the tracked frame.
- All RTAB-Map nodes and topics come up under the `/rtabmap/` namespace by default (e.g.
  `/rtabmap/map`, `/rtabmap/odom`, `/rtabmap/cloud_map`, `/rtabmap/mapData`).
- `rtabmap_viz:=false` and `rviz:=false` — we'll use one shared RViz2 window in D8 instead of
  spawning two separate GUIs.

Verify it's producing a map:
```bash
ros2 topic hz /rtabmap/map
ros2 topic hz /rtabmap/odom
```

### D7. Launch FastMapping — 3D OctoMap generation

FastMapping listens to the aligned depth stream and asks the TF tree for the camera's pose
relative to the `map` frame (which RTAB-Map is now publishing), so **launch it after D6 has a
few seconds to come up**:

```bash
ros2 run fast_mapping fast_mapping_node --ros-args \
  -p depth_topic_1:=/camera/aligned_depth_to_color/image_raw \
  -p depth_info_topic:=/camera/aligned_depth_to_color/camera_info \
  -p map_frame:=map \
  -p voxel_size:=0.04 \
  -p max_depth_range:=3.0 \
  -p robot_radius:=0.2
```

Since D4 already used flat camera topic names matching FastMapping's own defaults, the two
`-p depth_topic_1` / `-p depth_info_topic` overrides above are technically redundant — included
for clarity and so this command stays correct even if you change the camera namespace later.

Tunable parameters worth knowing:
- `voxel_size` — OctoMap cell resolution in meters (default 0.04 m).
- `max_depth_range` — how far (meters) the depth image is trusted (default 3.0 m).
- `projection_min_z` / `projection_max_z` — height band (relative to `map`) collapsed into the
  2D `world/map` occupancy grid; defaults 0.1–1.0 m. Adjust if your robot/obstacles sit outside
  that band.
- `robot_radius` — used for inflation around obstacles (default 0.2 m).

FastMapping publishes:
- `/world/fused_map` (`visualization_msgs/MarkerArray`) — the 3D voxel map.
- `/world/occupancy` (`visualization_msgs/MarkerArray`) — occupied/free voxel markers.
- `/world/map` (`nav_msgs/OccupancyGrid`) — a 2D occupancy grid flattened from the 3D map.

These topic names are unchanged from the training guide.

### D8. Visualize everything in RViz2

```bash
ros2 run rviz2 rviz2
```

Add these displays:
- **TF** — to sanity-check the `map → odom → camera_link → laser_frame` chain.
- **MarkerArray** on `/world/fused_map` — the live 3D OctoMap from FastMapping.
- **OccupancyGrid** on `/world/map` or `/rtabmap/map` — flattened 2D view.
- **PointCloud2** on `/rtabmap/cloud_map` — RTAB-Map's own 3D point-cloud map.
- **LaserScan** on `/scan` — set Reliability Policy to "Best Effort" if it shows a QoS mismatch
  warning (same gotcha as your Part C).

Set the Fixed Frame (top of the Displays panel) to `map`.

### D9. Save the map (optional)

RTAB-Map's database (which contains the full 3D map + loop-closure graph) auto-saves to
`~/.ros/rtabmap.db` by default (see `database_path` arg in D6 if you want a custom path). To
start completely fresh next session, keep `args:="--delete_db_on_start"`; drop that flag to
resume/extend the existing map.

---

## Part E — Run everything, one node per terminal

Assuming Parts A–D above are all installed, here's the full session from a cold start —
lidar, camera, static TF, RTAB-Map, FastMapping, and visualization, in the order that avoids
startup races:

```bash
# Terminal 1 — static transform between camera and lidar (measure your real offset first)
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 0.05 --roll 0 --pitch 0 --yaw 0 \
  --frame-id camera_link --child-frame-id laser_frame

# Terminal 2 — YDLidar X2
ros2 launch ydlidar_ros2_driver ydlidar_launch.py

# Terminal 3 — RealSense D455 (flat topic names)
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true align_depth.enable:=true \
  pointcloud.enable:=false camera_namespace:=/ camera_name:=camera

# Terminal 4 — RTAB-Map SLAM fusion (visual odometry + lidar-assisted mapping)
# wait ~5-10s after Terminals 2 & 3 report steady topic rates before starting this
ros2 launch rtabmap_launch rtabmap.launch.py \
  args:="--delete_db_on_start" \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  frame_id:=camera_link approx_sync:=true \
  visual_odometry:=true subscribe_scan:=true scan_topic:=/scan \
  qos:=2 rtabmap_viz:=false rviz:=false

# Terminal 5 — FastMapping (3D OctoMap)
# wait until Terminal 4 shows "Odometry initialized" / is publishing /rtabmap/map before starting this
ros2 run fast_mapping fast_mapping_node --ros-args \
  -p depth_topic_1:=/camera/aligned_depth_to_color/image_raw \
  -p depth_info_topic:=/camera/aligned_depth_to_color/camera_info \
  -p map_frame:=map -p voxel_size:=0.04 -p max_depth_range:=3.0 -p robot_radius:=0.2

# Terminal 6 — visualization
ros2 run rviz2 rviz2
```

Confirm every node is live and producing data:
```bash
ros2 node list
# expect: /static_transform_publisher, /ydlidar_ros2_driver_node, /camera/camera,
#         /rtabmap/rgbd_odometry, /rtabmap/rtabmap, /fast_mapping, /rviz

ros2 topic hz /scan
ros2 topic hz /camera/color/image_raw
ros2 topic hz /rtabmap/map
ros2 topic hz /world/fused_map
```

If `/world/fused_map` stays silent, check Terminal 5's log — it will print
`waiting for camera depth info from camera/aligned_depth_to_color/camera_info` until it sees
both a `CameraInfo` message and a valid `map → <depth frame>` transform, so the most common
cause is starting it before RTAB-Map (Terminal 4) has published its first TF frame.
