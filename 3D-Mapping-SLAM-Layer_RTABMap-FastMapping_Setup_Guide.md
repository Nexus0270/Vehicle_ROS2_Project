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
  --x 0.0 --y 0.0 --z 0.04 --roll 0 --pitch 0 --yaw 0 \
  --frame-id camera_link --child-frame-id laser_frame
```

Values above are meters (not mm/cm) for x/y/z, and radians (not degrees) for roll/pitch/yaw.
Axes follow `camera_link`'s own body frame (REP-103): **+X = forward** (out the lens),
**+Y = left** (from the camera's point of view), **+Z = up**. Measure the lidar's real offset
from the camera's origin along these three axes and adjust the values above accordingly —
the numbers shown here are one example rig's measured offset, not universal defaults.

> **This is the single most common failure point in this whole layer.** If this terminal is
> closed, crashes, or was never started this session, RTAB-Map will throw
> `Could not find a connection between 'camera_link' and 'laser_frame' because they are not
> part of the same tree` and `Could not convert laser scan msg! Aborting rtabmap update...`
> repeatedly — while odometry (`rgbd_odometry`) keeps running fine and gives no indication
> anything is wrong. If you see that specific error pair, check this terminal first:
> ```bash
> ros2 node list | grep static_transform_publisher
> ros2 run tf2_ros tf2_echo camera_link laser_frame
> ```
> If either fails, restart the command above. **Keep this running in its own terminal for the
> whole session and don't close it** — everything downstream depends on it, and there is no
> automatic restart if it dies.

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
  rgb_camera.color_profile:=424x240x30 \
  depth_module.depth_profile:=480x270x30 \
  camera_namespace:=/ camera_name:=camera
```

Verify:
```bash
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw
```
You should see `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw`,
`/camera/color/camera_info`, `/camera/aligned_depth_to_color/camera_info`, all at ~30 fps.

> **If you have a genuine dedicated USB3 link for the camera** (separate hub/port from the
> lidar, or a PCIe USB3 card — see your Part C1), you can drop the `rgb_camera.color_profile`
> and `depth_module.depth_profile` overrides and run at the camera's native resolution/fps
> instead; RTAB-Map and FastMapping will happily take the extra detail. Check first with
> `lsusb -t` to confirm the camera and lidar aren't nested under the same hub.
>
> First run a profile list to confirm these exact strings are supported by your D455's
> firmware (profile lists vary by firmware and differ for color vs. depth). Use whatever
> name shows up under `ros2 node list` for the camera (commonly just `/camera`, not
> `/camera/camera`, when `camera_namespace:=/` is used):
> ```bash
> ros2 node list | grep camera
> ros2 param describe /camera rgb_camera.color_profile
> ros2 param describe /camera depth_module.depth_profile
> ```

#### Alternative: drop to 15 fps if the lidar starts throwing checksum/timeout errors

30 fps at the same resolution is the default here since it gives visual odometry more overlap
between consecutive frames (less motion per frame → easier feature matching, especially during
rotation). But it does push more data over the shared USB link than 15 fps. If you see lidar
checksum/timeout errors resurface (the Part C1 bandwidth-contention symptom) with the camera
and lidar sharing one hub, drop back to 15 fps at the same resolution:

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true align_depth.enable:=true \
  pointcloud.enable:=false \
  rgb_camera.color_profile:=424x240x15 \
  depth_module.depth_profile:=480x270x15 \
  camera_namespace:=/ camera_name:=camera
```

**Fps is not a substitute for scene texture or movement speed, though.** If RTAB-Map's
odometry (`rgbd_odometry`) reports `Not enough inliers` and `quality=0` and `matches=` stays
low (well under ~50) even while holding the rig still, or drops further at 30 fps than it does
at 15 fps, the camera is very likely pointed at a low-texture surface (blank wall, plain
floor/ceiling, glass, uniform lighting) rather than suffering from a pure frame-timing
problem — point it at a textured area (furniture, shelves, patterned surfaces, clutter) and
re-check before assuming fps is the fix. In general:
- Move slowly, especially when rotating — RGB-D visual odometry tolerates translation much
  better than fast rotation.
- Keep the scene within the D455's reliable depth range (roughly 0.4–6 m, best under ~4 m
  given `max_depth_range:=3.0` in D7).
- Avoid pointing it at anything fully occluding the lens for more than a moment (a hand, a
  large object suddenly blocking the view, etc.) — with no visual continuity between frames,
  RTAB-Map correctly refuses to guess a pose rather than silently starting a disconnected new
  one, so tracking will read `quality=0` until the occlusion clears and it can re-acquire
  familiar features.

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

> **This D1/D6 pair is the one to get rock-solid before adding FastMapping (D7) at all.**
> FastMapping adds real CPU load on top of everything else, which can surface or worsen
> unrelated issues (frame backlog, TF timing jitter) that make it harder to tell what's
> actually wrong. Recommended order the first time through: do D1–D6, verify with D8 below
> (skip the FastMapping-only displays), confirm the map looks correct using the checklist at
> the end of D8, and only then move on to D7.

### D6.5 (recommended first pass) — verify RTAB-Map alone before adding FastMapping

Skip D7 for now and jump to D8's RViz2 setup with just Parts D1–D6 running. This isolates
odometry/lidar-fusion problems from FastMapping's CPU load, and gives you a known-good
baseline to compare against once FastMapping is added back in.

### D7. Launch FastMapping — 3D OctoMap generation (add this in only after D6.5 checks out)

FastMapping listens to the aligned depth stream and asks the TF tree for the camera's pose
relative to the `map` frame (which RTAB-Map is now publishing), so **launch it after D6 has a
few seconds to come up**:

> **Important — switch the camera to the 15 fps alternative from D4 before starting FastMapping,
> not the 30 fps default.** Testing showed FastMapping keeps up fine with a 30 fps stream while
> the rig is stationary (processing rate tracks ~30 Hz, queue stays near 0), but the instant the
> rig moves, its processing rate collapses to ~1.4 Hz — a ~20x slowdown, since each moving frame
> needs a costly voxel-grid update rather than refining a static region. The backlog then grows
> unbounded, and once it grows past the TF buffer's retention window you get a wave of
> `Failed to get transform ... Lookup would require extrapolation into the past` errors as it
> works through stale, un-recoverable frames. At 15 fps input with `voxel_size:=0.06`, it keeps
> up during motion instead. Relaunch the camera (Terminal 3) with:
> ```bash
> ros2 launch realsense2_camera rs_launch.py \
>   enable_color:=true enable_depth:=true align_depth.enable:=true \
>   pointcloud.enable:=false \
>   rgb_camera.color_profile:=424x240x15 \
>   depth_module.depth_profile:=480x270x15 \
>   camera_namespace:=/ camera_name:=camera
> ```
> before starting FastMapping below.

```bash
ros2 run fast_mapping fast_mapping_node --ros-args \
  -p depth_topic_1:=/camera/aligned_depth_to_color/image_raw \
  -p depth_info_topic:=/camera/aligned_depth_to_color/camera_info \
  -p map_frame:=map \
  -p voxel_size:=0.06 \
  -p max_depth_range:=3.0 \
  -p robot_radius:=0.2
```

Since D4 already used flat camera topic names matching FastMapping's own defaults, the two
`-p depth_topic_1` / `-p depth_info_topic` overrides above are technically redundant — included
for clarity and so this command stays correct even if you change the camera namespace later.

Tunable parameters worth knowing:
- `voxel_size` — OctoMap cell resolution in meters (node default 0.04 m; **0.06 m recommended
  here** — coarser voxels are cheaper to update per frame, which matters a lot once the rig is
  moving, not just for map file size).
- `max_depth_range` — how far (meters) the depth image is trusted (default 3.0 m).
- `projection_min_z` / `projection_max_z` — height band (relative to `map`) collapsed into the
  2D `world/map` occupancy grid; defaults 0.1–1.0 m. Adjust if your robot/obstacles sit outside
  that band.
- `robot_radius` — used for inflation around obstacles (default 0.2 m).

Watch the recurring log line to confirm it's keeping up, especially while moving the rig:
```
fast_mapping got N images in X s. Aligned Y. Processed Z (rate Hz). M left in queue
```
"Processed... Hz" should stay reasonably close to the camera's 15 fps, and "M left in queue"
should stay flat/near-zero rather than climbing, even while actively moving the rig — not just
while it's sitting still. If the queue starts climbing again during motion, increase
`voxel_size` further (e.g. 0.08–0.10) before considering going back to 30 fps.

FastMapping publishes:
- `/world/fused_map` (`visualization_msgs/MarkerArray`) — the 3D voxel map.
- `/world/occupancy` (`visualization_msgs/MarkerArray`) — occupied/free voxel markers.
- `/world/map` (`nav_msgs/OccupancyGrid`) — a 2D occupancy grid flattened from the 3D map.

These topic names are unchanged from the training guide.

### D8. Visualize everything in RViz2

```bash
ros2 run rviz2 rviz2
```

#### Step-by-step setup (click-by-click)

**1. Set the Fixed Frame first.** In the **Displays** panel (left side), find **Global
Options** at the top and its **Fixed Frame** field. Click the field's dropdown arrow (don't
type it manually) and select `map`. If `map` doesn't appear in the dropdown, RTAB-Map (D6)
isn't publishing it yet — confirm with `ros2 run tf2_ros tf2_echo camera_link map` before
going further; if that fails, fix D1/D6 first (see the warning box in D6.5 above) rather than
continuing in RViz2. RViz2 defaults its Fixed Frame to `base_link`, which doesn't exist on
this camera-only rig (no wheel base) — you'll see a `No transform from [base_link] to [map]`
warning until you change it.

**2. Add each display via the "Add" button** at the bottom-left of the Displays panel. A
dialog opens with two tabs: **"By display type"** (pick the display type manually, useful for
TF) and **"By topic"** (lists live topics directly, easier for everything else).

- **TF** — "By display type" tab → find **TF** → OK. Confirms the full chain
  `map → odom → camera_link → laser_frame` (plus the camera's own internal frames hanging off
  `camera_link`) is connected with no breaks.
- **PointCloud2 on `/rtabmap/cloud_map`** — "By topic" tab → find `/rtabmap/cloud_map` → select
  **PointCloud2** → OK. RTAB-Map's own accumulated 3D map.
- **Map (OccupancyGrid) on `/rtabmap/map`** — "By topic" tab → find `/rtabmap/map` → select
  **Map** → OK. RTAB-Map's 2D grid, built from the fused lidar scans.
- **LaserScan on `/scan`** — "By topic" tab → find `/scan` → select **LaserScan** → OK. If it
  shows a QoS-mismatch warning icon, expand the display, find **Reliability Policy**, and
  change it from "Reliable" to **"Best Effort"** (same gotcha as your Part C).
- **Odometry on `/rtabmap/odom`** (optional) — "By topic" tab → find `/rtabmap/odom` → select
  **Odometry** → OK. Live pose arrow/trail, useful for eyeballing smooth vs. jumpy tracking.
- **Once FastMapping (D7) is running:** add **MarkerArray** on `/world/fused_map` (the live 3D
  OctoMap) and optionally a second MarkerArray on `/world/occupancy`.

#### What each display actually shows

- **PointCloud2 (`/rtabmap/cloud_map`)** — real-world RGB colors from the camera, not a status
  indicator. Oddly flat or noisy patches usually mean invalid/noisy depth was captured there.
- **Map / OccupancyGrid (`/rtabmap/map` or `/world/map`)** — classic 2D grid: **black** =
  occupied (obstacle detected), **white** = confirmed free space, **gray** = unknown/unexplored
  (not yet observed by lidar or camera).
- **LaserScan (`/scan`)** — renders as **red dots** by default; each dot is one lidar range
  reading. A ring/arc of red dots tracing your walls/obstacles as the lidar spins is expected
  and correct, not an error.
- **Odometry (`/rtabmap/odom`)** — an arrow or small axis marker showing RTAB-Map's current
  estimated camera pose, often with a trail of past poses. Useful for spotting drift or jumps.
- **MarkerArray (`/world/fused_map`, `/world/occupancy`)** — FastMapping's 3D voxel map;
  colors (e.g. red) typically denote occupied vs. free voxels, not an error state.

#### How to tell the map is being built correctly (not just "something is showing up")

- **Walls should look like a single, consistent line/surface** in the OccupancyGrid and point
  cloud — not doubled, smeared, or staircased. A blurred/doubled wall is the classic visual
  sign of accumulated odometry drift.
- **Walk the rig in a loop back to its starting point.** Walls/corners should still line up
  with no visible seam or duplicated segment where the loop closes. Watch the terminal for a
  loop-closure log line from `rtabmap` around that moment — if you obviously closed a loop and
  never see one, that's worth investigating (either not enough scene texture to recognize the
  revisit, or a deeper tracking issue).
- **The Odometry arrow should move smoothly**, matching your actual motion — a sudden jump or
  flip (especially right after a `quality=0` recovery in the terminal) means odometry briefly
  lost tracking and reacquired at a slightly wrong pose. Occasional small ones get cleaned up
  by loop closure; frequent/large ones are a problem.
- **The point cloud's geometry should roughly match your real room** — right angles where the
  room actually has them, furniture in the right place, consistent floor height. Warped or
  ghosted/doubled surfaces indicate drift or bad depth data being incorporated.
- **Check the `map → odom` transform occasionally:**
  ```bash
  ros2 run tf2_ros tf2_echo map odom
  ```
  This is RTAB-Map's drift-correction offset — it should change gradually or in occasional
  small corrections (especially right after loop closures), not erratically every frame.
- **Watch the terminal's periodic summary line** — `rtabmap (N): Rate=... (local map=X, WM=Y)`.
  **WM (working memory)** should climb steadily as you explore new areas and stay stable
  otherwise. An unexpected reset to near-zero usually means the database got wiped/restarted
  (e.g. `--delete_db_on_start` was applied again without intending to).
- **Optional deep-dive:** stop everything and open the saved database in RTAB-Map's own
  standalone viewer, which makes drift and bad loop closures far easier to inspect than live
  RViz2:
  ```bash
  rtabmap-databaseViewer ~/.ros/rtabmap.db
  ```

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

# Terminal 3 — RealSense D455 (flat topic names, low-bandwidth profile — see D4)
# default is 30 fps, but this run-through includes FastMapping (Terminal 5), which needs the
# 15 fps profile to keep up with a moving rig (see D7) — use 15 fps here, not the D4 default
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true align_depth.enable:=true \
  pointcloud.enable:=false \
  rgb_camera.color_profile:=424x240x15 \
  depth_module.depth_profile:=480x270x15 \
  camera_namespace:=/ camera_name:=camera

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
  -p map_frame:=map -p voxel_size:=0.06 -p max_depth_range:=3.0 -p robot_radius:=0.2

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
