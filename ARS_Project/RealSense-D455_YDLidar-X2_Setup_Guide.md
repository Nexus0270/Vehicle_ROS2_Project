# Intel RealSense D455 + YDLidar X2 Setup Guide (ROS 2 Jazzy, Ubuntu 24.04)

This replaces the old **YDLidar Robotics SDK 2.1** workspace-based install (no longer available)
with the current official build-from-source method. RealSense steps use the current
`realsense-ros` wrapper.

**Assumptions:** Ubuntu 24.04, ROS 2 Jazzy already installed (`ls /opt/ros/` to confirm),
`colcon` and `git` available. Confirmed working end-to-end on this combo — commands below use
`jazzy` throughout. If you're on a different release, swap the distro name accordingly.

---

## Part A — Intel RealSense D455

### A1. Install the librealsense2 SDK (build from source)

The apt-repo method (signed key + `apt install librealsense2-*`) is unreliable on Ubuntu 24.04
— Intel's hosted GPG key doesn't match the fingerprint apt expects (`NO_PUBKEY` errors even
after re-dearmoring), so `apt update` never succeeds. Build from source instead — confirmed
working:

```bash
sudo apt install -y git cmake build-essential libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev udev

cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense

sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=true
make -j$(nproc)
sudo make install
sudo ldconfig
```

`make -j$(nproc)` compiles the full SDK plus examples/tools (including `realsense-viewer`) and
can take 10–30+ minutes depending on your CPU.

> **Duplicate udev rules warning:** if you'd previously attempted the apt-repo install (even a
> failed attempt can leave a partial package behind), the build may warn:
> `Multiple realsense udev-rules were found!` — one at `/etc/udev/rules.d/99-realsense-libusb.rules`
> (from this source build) and one at `/lib/udev/rules.d/60-librealsense2-udev-rules.rules`
> (leftover from `librealsense2-udev-rules`, an apt package). Check with:
> ```bash
> dpkg -S /lib/udev/rules.d/60-librealsense2-udev-rules.rules
> ```
> If it resolves to a package name, remove it cleanly rather than deleting the file directly:
> ```bash
> sudo apt remove --purge -y librealsense2-udev-rules
> sudo udevadm control --reload-rules && sudo udevadm trigger
> ```

After a successful build, unplug and replug the camera so the udev rules take effect.

### A2. Plug in the D455 (use a USB 3 / blue port) and verify

```bash
realsense-viewer
```
You should see RGB + depth streams. If the device resets/disconnects immediately, it's almost
always a USB3 port, cable, or power issue — try a different port before touching software.

**"Device or resource busy" (`VIDIOC_S_FMT failed, errno=16`) when enabling RGB:**
Something else already has the sensor claimed. Fix in order:
```bash
sudo lsof /dev/video* 2>/dev/null   # find the culprit process
pkill -f realsense-viewer
pkill -f realsense2_camera
sudo modprobe -r uvcvideo && sudo modprobe uvcvideo   # reset kernel video driver
```
Then fully unplug/replug the camera (wait ~5s). If it still fails, reboot — this clears any
wedged USB/kernel state the steps above didn't catch.

### A3. Install the ROS 2 wrapper

Binary install (fastest, if available for your distro):
```bash
sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-realsense2-description
```

Or build from source (needed if you want the latest fixes, or binaries aren't available):
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/realsenseai/realsense-ros.git -b ros2-development
cd ~/ros2_ws
sudo apt install -y python3-rosdep
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --symlink-install
source install/local_setup.bash
```

> Note: the wrapper repo was recently moved/rebranded to `realsenseai/realsense-ros`
> (previously `IntelRealSense/realsense-ros`) — use that URL if the old one 404s.

### A4. Launch and check topics

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true align_depth.enable:=true pointcloud.enable:=true
```

```bash
ros2 topic list | grep camera
```

**Important:** current wrapper versions publish under a doubled namespace, e.g.
`/camera/camera/color/image_raw` and `/camera/camera/aligned_depth_to_color/image_raw`,
not the flat `/camera/color/image_raw` shown in the old diagram. Update any launch files,
RTAB-Map remaps, or the YOLOv8 node's subscribed topic names accordingly — or set
`camera_name`/`camera_namespace` launch args to force the old flat naming if you need
drop-in compatibility.

---

## Part B — YDLidar X2 (SDK 2.1 replacement)

The old fixed "SDK 2.1 workspace" download is gone. The current supported path is: build
**YDLidar-SDK** from source, then build the **ydlidar_ros2_driver** package against it.

### B1. Install build dependencies

```bash
sudo apt update
sudo apt install -y cmake pkg-config git build-essential
```

### B2. Build and install YDLidar-SDK (core driver library)

```bash
cd ~
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

> The repo doesn't ship a pre-made `build` folder — create it before running `cmake ..`, or
> `cmake` will fail (or worse, silently run from the wrong directory if a `cd` fails and the
> shell keeps going). Confirmed working version at time of writing: **v1.2.7** (SDK reports
> `SDK Version: 1.2.20` at runtime — that's the internal SDK version string, separate from the
> repo's git tag). Check yours with `git describe --tags` inside `YDLidar-SDK/`.

### B3. Grant serial port permissions

```bash
sudo usermod -a -G dialout $USER
```
Log out and back in (or reboot) for the group change to apply.

### B4. Clone and build the ROS 2 driver (humble branch)

```bash
mkdir -p ~/ydlidar_ros2_ws/src
cd ~/ydlidar_ros2_ws/src
git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git

cd ~/ydlidar_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

> The `humble` branch is the correct one even on Jazzy — YDLIDAR bundles Humble/Jazzy/Iron
> support together on that branch; there's no separate `jazzy` branch. Only the `source` line
> above needs to point at `/opt/ros/<your_distro>/setup.bash`.
>
> If `source /opt/ros/jazzy/setup.bash` fails with "No such file or directory", first run
> `ls /opt/ros/` to confirm ROS 2 is actually installed and under which name. Also check
> `~/.bashrc` for a stale `source /opt/ros/humble/setup.bash` line left over from a template —
> fix with: `sed -i 's#/opt/ros/humble/setup.bash#/opt/ros/jazzy/setup.bash#' ~/.bashrc`

### B5. Source the workspace

```bash
echo "source ~/ydlidar_ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### B6. Configure for the X2 model

**Gotcha:** `ydlidar_launch_view.py` and `ydlidar_launch.py` default to loading
`params/ydlidar.yaml`, **not** `params/X2.yaml`. Editing `X2.yaml` alone does nothing unless you
either (a) edit `ydlidar.yaml` directly, or (b) override the launch arg to point at `X2.yaml`.
Confirm which file a launch script actually uses with:
```bash
grep -n "yaml\|params" ~/ydlidar_ros2_ws/src/ydlidar_ros2_driver/launch/ydlidar_launch_view.py
```

**This guide edits `ydlidar.yaml` directly** (simplest — no need to override launch args each
time):
```bash
nano ~/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml
```

**Confirmed working values for the X2:**
```yaml
ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ttyUSB0
    frame_id: laser_frame
    baudrate: 115200        # NOT 230400 — that's for G4/other models
    lidar_type: 1
    device_type: 0
    sample_rate: 3
    isSingleChannel: true    # critical for X2 — false causes "cannot retrieve
                              # Lidar health code -2" / "Failed to start the lidar"
    intensity: false
    angle_max: 180.0
    angle_min: -180.0
    range_max: 8.0
    range_min: 0.1
    frequency: 7.0
```
Confirm the port name (`ls /dev/ttyUSB*` after plugging in) and swap it in if it's not `ttyUSB0`.

**Important:** editing the file under `src/` only changes the source copy. The launch file
reads from `install/`, so you must rebuild after every edit for it to take effect:
```bash
cd ~/ydlidar_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

You can verify the installed copy picked up your change with:
```bash
grep -E "baudrate|isSingleChannel" ~/ydlidar_ros2_ws/install/ydlidar_ros2_driver/share/ydlidar_ros2_driver/params/ydlidar.yaml
```

> Alternative you don't need here, but worth knowing: instead of editing `ydlidar.yaml`, you
> could leave it untouched and override `params_file` at launch time to point straight at
> `X2.yaml`. Editing `ydlidar.yaml` (this guide's approach) is simpler if the X2 is your only
> lidar model in use, since it's the launch scripts' default and needs no extra launch args.

### B7. (Optional) Create a persistent device alias

Useful if you also plug in other USB-serial devices and port numbering shifts around:
```bash
cd ~/ydlidar_ros2_ws/src/ydlidar_ros2_driver/startup
sudo chmod 0777 ./*
sudo sh ./initenv.sh
```
This creates a fixed `/dev/ydlidar` symlink via udev rule.

### B8. Launch and verify

```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py
```
This launches the driver + RViz2 preconfigured to show the laser scan. To launch without RViz:
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

Check the topic:
```bash
ros2 topic echo /scan --once
```

**Troubleshooting:**
- `Permission denied` on the port → `sudo chmod 0666 /dev/ttyUSB0` (or fix the `dialout` group
  membership from B3), then unplug/replug the lidar.
- `Error, cannot retrieve Lidar health code -2` + `Failed to start the lidar` → almost always
  `isSingleChannel: false` when it should be `true` for the X2 (see B6). Confirm baudrate is
  115200 as well.
- No `/scan` data → confirm `baudrate`/`lidar_type`/`isSingleChannel` match X2 (not X2L or
  another model — they use different sample rates and PWM behavior).

**Lines that look scary but are normal on the X2:**
- `Fail to get baseplate device information!` — expected; the X2 has no baseplate module, so
  this query always fails and the driver just continues.
- `Real points N > fixed points 840` — harmless rounding warning, not an error.

**Signs it's genuinely working:**
```
Lidar running correctly! The health status good
Successed to start the lidar, elapsed time ...
Successed to check the lidar, elapsed time ...
Lidar has started!
```
Confirm with:
```bash
ros2 topic hz /scan
```
You should see a steady rate close to your configured `frequency` (~7-10 Hz), and colored
points forming your room's outline in the RViz `LaserScan` display.

---

## Part C — Bring both up together

Make sure `~/.bashrc` sources ROS 2 Jazzy plus both workspaces (check with
`grep -n "source" ~/.bashrc`):
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.bash        # RealSense wrapper, if built from source
source ~/ydlidar_ros2_ws/install/setup.bash      # YDLidar driver
```

Launch both. No need to override `params_file` — the launch scripts already default to
`ydlidar.yaml`, which you configured for the X2 in B6:
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true &

ros2 launch ydlidar_ros2_driver ydlidar_launch.py &

ros2 run rviz2 rviz2
```

Confirm both streams are live:
```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /camera/camera/color/image_raw
```
`/scan` should sit around your configured `frequency` (~7-10 Hz for the X2), and the camera
topic should be at your requested FPS. If `/scan` doesn't show up, re-check B6/B8's
troubleshooting — the most common cause is `isSingleChannel` or `baudrate` in `ydlidar.yaml`
reverting to defaults (e.g. after re-cloning or a fresh checkout) without re-editing and
rebuilding.

From here, RTAB-Map, FastMapping/OctoMap, and the OpenVINO/YOLOv8 node from the original
architecture just need their input topic names updated to match Part A's note on the
`/camera/camera/...` namespace and `/scan` above — the rest of the pipeline in your diagram
(SLAM fusion → OctoMap → RViz2 visualization) is unaffected by the SDK swap.

### C1. If camera + lidar share a single USB hub (checksum errors / dropped scans)

If your machine only has **one** real USB3-capable port (common on boards with only standard
USB headers, no extra front-panel pin connectors), both the camera and lidar end up sharing the
same internal hub. Symptom: the lidar throws repeated errors once the camera starts streaming:
```
[error] Timeout count: 1
[error] Checksum error 0xD653 != 0xD7A1
[info] The intensity has been automatically adjusted to[16]bit
```

**Diagnose first:**
```bash
lsusb -t
```
If the camera (`uvcvideo`) and lidar (`cp210x`) both appear nested under the same `Hub` entry,
especially a `480M` (USB2) hub, that confirms bandwidth contention — the camera's RGB+depth+
pointcloud streams are saturating the shared 480 Mbps link, corrupting the lidar's serial data.

**Real fix (do this if at all possible):** add a powered USB 3.0 hub, or a PCIe USB3 expansion
card if you have a free slot, and put the camera on a genuine dedicated USB3 link separate from
the lidar. This is the only way to run the camera at full resolution alongside the lidar
reliably.

**Software workaround if a hardware fix isn't available yet:** drop the camera's resolution/FPS
and disable the point cloud to free up enough bandwidth for the lidar's serial link. First check
your camera's actual supported profiles (they vary by firmware) — profile lists are separate
for color and depth:
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=false &
ros2 param describe /camera/camera rgb_camera.color_profile
ros2 param describe /camera/camera depth_module.depth_profile
```
Then launch with a low-bandwidth profile from those lists, e.g.:
```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  pointcloud.enable:=false \
  rgb_camera.color_profile:=424x240x15 \
  depth_module.depth_profile:=480x270x15
```
`pointcloud.enable:=false` matters most here — it's the heaviest stream. Note color and depth
resolutions don't need to match exactly; `align_depth` interpolates between them fine.

Then bring the lidar up and watch its terminal for a minute or two:
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py &
```
If the checksum/timeout errors stop or become rare, bandwidth contention was confirmed as the
cause — treat this as your working profile until you add real USB3 capacity. This is a
workaround, not a permanent fix: for full-resolution SLAM/detection work you'll eventually want
the camera on its own dedicated USB3 link.

---

## Summary of what changed vs. the original guide

| Old approach | Current replacement |
|---|---|
| YDLidar Robotics SDK 2.1 fixed download | Build `YDLIDAR/YDLidar-SDK` from source (Part B2) |
| Pre-built SDK 2.1 workspace | Fresh `ydlidar_ros2_driver` (`humble` branch — also covers Jazzy) built against the source SDK |
| `/camera/color/image_raw` | `/camera/camera/color/image_raw` (current `realsense-ros` namespace) |
| `IntelRealSense/realsense-ros` | Mirrored/continued at `realsenseai/realsense-ros` |
| `librealsense2` via apt repo (broken GPG key on Ubuntu 24.04) | Build `librealsense2` from source against `IntelRealSense/librealsense` (Part A1) |
| Generic `ydlidar.yaml` defaults (230400 baud, `isSingleChannel: false`) | X2-specific: `baudrate: 115200`, `isSingleChannel: true` (Part B6) — without this the lidar connects but fails health check |
| ROS 2 Humble / Ubuntu 22.04 | Confirmed on ROS 2 Jazzy / Ubuntu 24.04 — swap `/opt/ros/humble` → `/opt/ros/jazzy` everywhere |
| Camera + lidar checksum errors on single-hub machines | Diagnose with `lsusb -t`; fix with a USB3 hub/PCIe card, or fall back to a low-bandwidth camera profile (Part C1) |
