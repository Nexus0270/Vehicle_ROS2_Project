# ROS2 Autonomous Mobile Robot — Project Structure & Execution Plan

**Team size:** 3
**Primary goal:** Flawless point-A-to-point-B teleoperation of a ROS2-enabled mobile robot.
**Bonus goal:** Autonomous navigation via SLAM.
**Core deliverables:** (1) FreeCAD-designed, 3D-printed enclosure for all electronics, (2) full hardware integration, (3) working ROS2 teleop stack, (4) SLAM + Nav2 (bonus).

---

## Part 1 — Component Analysis

The eleven supplied components fall into five functional groups. Understanding *what each does and how it connects* is what drives the wiring, the firmware, and the software architecture below.

### Group A — Compute & High-Level Control
| Component | Role | Key specs that matter for us |
| :--- | :--- | :--- |
| **Axiomtek CAPA55R SBC** | The "brain." Runs Ubuntu + ROS2; hosts teleop, SLAM, Nav2, sensor drivers. | x86 11th-gen Intel, needs **12–24V DC input** (we feed it 19V). USB 3.2 + dual LAN. Active fan → enclosure needs venting. |
| **Arduino Mega 2560** | The "spinal cord." Real-time low-level controller: reads encoders, runs PID, generates motor PWM, talks to SBC over USB serial. | 4 hardware UARTs, 15 PWM pins, plenty of interrupt pins for 4× quadrature encoders. **VIN max 12V** — *cannot* be powered directly from the 18.5V battery. |

### Group B — Motion / Actuation
| Component | Role | Key specs that matter for us |
| :--- | :--- | :--- |
| **JGB37-520 gear motors (×4)** | Drive the wheels; Hall quadrature encoders give odometry. | **6V nominal**, 800 RPM, 6 mm D-shaft, stall ~2.5A. Encoder A/B channels + VCC/GND. |
| **Mecanum wheels (2L + 2R)** | Holonomic drive (forward, strafe, diagonal, spin-in-place). | 6 mm bore brass hub + M3 grub screw. **Needs 4 *independent* motor channels for true mecanum.** |
| **Cytron MDDRC10 driver** | Dual H-bridge; takes RC-style PWM, drives 2 brushed motors. | **2 channels only**, 10A continuous / 30A peak each, input 7–30V, MIX/IND mode switch, onboard 5V/500mA tap. |
| **PCA9685 PWM driver** | I2C → 16× PWM. Can generate the servo-style RC pulses the MDDRC10 expects, offloading the Arduino's timers. | I2C (SDA/SCL), 12-bit, 24–1526 Hz (use ~50–60 Hz for RC signals). |

### Group C — Perception / Sensors
| Component | Role | Key specs that matter for us |
| :--- | :--- | :--- |
| **RPLIDAR A1** | 360° 2D laser scan → the backbone of SLAM & obstacle avoidance. | 12 m range, 5.5 Hz typical, **5V**, UART→USB bridge → `/dev/ttyUSBx`. ROS driver: `rplidar_ros`. |
| **Intel RealSense D455** | Depth + RGB + IMU for 3D perception / visual odometry (bonus). | USB-C 3.1, 5V (USB-powered), `librealsense2` + `realsense2_camera`. |

### Group D — Power
| Component | Role | Key specs that matter for us |
| :--- | :--- | :--- |
| **4200 mAh LiPo** | Main energy source. | Configured **5S = 18.5V nominal** (15.0–21.0V range). XT60 + JST-XH balance lead. ~74 Wh. |
| **SZDULI boost converter** | Steps battery voltage **up to fixed 19V / 8A** to feed the SBC. | Input 10–36V, output barrel plug (5.5×2.5 mm) → Axiomtek jack. |

### Group E — Mechanical / Mounting
| Component | Role | Key specs that matter for us |
| :--- | :--- | :--- |
| **M3 standoff + fastener kit** | Layered mounting of PCBs above the chassis; isolation & airflow. | M3 × 0.5 mm, brass F-F / M-F standoffs, 304 SS screws/nuts. Needs 3.2–3.5 mm clearance holes in printed plates. |

---

## Part 2 — System Architecture

### 2.1 Power tree
```
                 5S LiPo (18.5V nominal, XT60)
                          │
        ┌─────────────────┼──────────────────────────┐
        │                 │                            │
   [Main switch /     [MDDRC10 motor driver]     [SZDULI boost → 19V/8A]
    fuse / e-stop]      (7–30V OK at 18.5V)              │
        │                 │                       Axiomtek SBC (12–24V jack)
        │            motors (PWM-capped!)
        │
   [5V BUCK *MISSING*] ──► RPLIDAR (5V), Arduino (5V), PCA9685 logic
```
> **Two power gaps to close before powering on (see Risk Register):**
> 1. **No 5V buck converter** is in the kit, but the LIDAR, Arduino, and PCA9685 logic all need a regulated low-voltage rail. The MDDRC10's 5V/500 mA tap can *partially* cover this but is marginal for LIDAR inrush (600 mA). Plan to add a dedicated 5V/3A buck module.
> 2. **Arduino VIN max is 12V** — never feed it 18.5V. Power it from the SBC USB (5V) or the 5V buck.

### 2.2 Control signal chain (teleop path)
```
Gamepad/Keyboard ─► ROS2 teleop node ─► /cmd_vel ─► base_controller node
        (on Axiomtek SBC)                                   │  (USB serial)
                                                            ▼
                                          Arduino Mega (micro-ROS or serial bridge)
                                          • PID using encoder feedback
                                          • generates RC-style PWM
                                                            │
                                              (direct PWM pins  OR  I2C→PCA9685)
                                                            ▼
                                                MDDRC10 H-bridge ─► motors
                                                            ▲
                                          encoders ───────────┘ (odometry back to SBC)
```
**Design choice:** For 2 channels the Arduino can generate PWM on its own pins (simplest). Use the **PCA9685** if you add a second driver (4 channels for true mecanum) or want to free Arduino timers. Set MDDRC10 to **IND mode** for independent left/right control.

### 2.3 Software stack (ROS2, on the SBC)
- `teleop_twist_joy` / `teleop_twist_keyboard` → publishes `/cmd_vel`
- **base_controller** (custom): `/cmd_vel` → wheel velocities → serial → Arduino; reads encoder ticks → publishes `/odom` + odom→base_link TF
- `rplidar_ros` → `/scan`
- `realsense2_camera` → depth/RGB/IMU (bonus)
- **URDF** robot description → TF tree, RViz model
- `slam_toolbox` → 2D map from `/scan` (bonus)
- `nav2` stack → autonomous A→B (bonus)
- **bringup** launch files tying it all together

---

## Part 3 — Roles & Responsibilities (3 People)

Each person **owns** a subsystem end-to-end, but integration milestones are explicitly shared. Roles are balanced so no single person is blocked waiting on another for too long.

### 👤 Person 1 — Mechanical & Integration Lead
*"If it's physical, printed, or bolted, I own it."*

**Owns:** FreeCAD enclosure, 3D printing, full physical assembly, cable management.

**Responsibilities**
- Measure every component footprint + connector clearances; build a layout in FreeCAD (electronics deck, sensor mounts, motor mounts).
- Design the **enclosure / mounting plates**: SBC bay with fan venting, Arduino + PCA9685 + MDDRC10 trays, **RPLIDAR top mount with clear 360° line-of-sight**, RealSense front mount at a fixed known height/angle, battery cradle with strap.
- Lay out M3 standoff hole patterns (3.2–3.5 mm clearance) for layered PCB stacking.
- Design **motor brackets** and verify 6 mm D-shaft → mecanum hub fit and grub-screw access.
- Slice & 3D print; iterate on tolerances (especially shaft bores and connector cutouts).
- Lead **final assembly** and cable routing; keep LIDAR/camera mounts rigid (vibration ruins SLAM).
- Track center of gravity and weight distribution for stable driving.

**Primary components:** Enclosure (all), M3 kit, mecanum wheels, motor mounting.

---

### 👤 Person 2 — Electronics & Firmware Lead
*"If it carries current or runs on the microcontroller, I own it."*

**Owns:** Power distribution, wiring harness, motor driver, Arduino firmware.

**Responsibilities**
- Design & build the **power tree**: battery → main switch/fuse → boost (19V→SBC); add and wire the **5V buck rail**; battery → MDDRC10.
- Add **safety**: inline fuse on the battery, an accessible **kill switch / e-stop**, and respect the LiPo 3.0V/cell discharge floor (low-voltage warning).
- Configure **MDDRC10** (IND mode, neutral calibration); wire 4 motors (decide driver/wiring strategy — see Risk #1 with Person 3).
- Write **Arduino Mega firmware**: read 4× quadrature encoders, closed-loop **PID** speed control, generate motor PWM (direct or via PCA9685), and a clean **serial protocol** to the SBC (target velocities in, encoder ticks out).
- **Cap PWM duty** so the 6V motors never see effective >6V from the 18.5V supply (see Risk #2).
- Bench-test each motor's direction, deadband, and encoder counts/rev before assembly.

**Primary components:** LiPo, boost converter, 5V buck (to source), MDDRC10, Arduino Mega, PCA9685, motors + encoders.

---

### 👤 Person 3 — Software & ROS2 Lead
*"If it runs in ROS2 on the SBC, I own it."*

**Owns:** Axiomtek OS/ROS2 setup, base controller node, teleop, sensor drivers, SLAM/Nav.

**Responsibilities**
- Flash/configure **Ubuntu + ROS2** on the Axiomtek; set up the workspace, udev rules for stable device names (`/dev/ttyUSB*`), and the SBC↔Arduino serial link (micro-ROS *or* a serial bridge node — agree with Person 2).
- Build the **base_controller node**: `/cmd_vel` → wheel speeds → Arduino; encoder ticks → `/odom` + TF. Calibrate odometry (ticks/meter, track width) with Person 2.
- Write the **URDF** and TF tree (base_link, wheels, laser, camera frames) using Person 1's CAD dimensions.
- Stand up **teleop** (joystick + keyboard) → this is the main-goal integration.
- Bring up `rplidar_ros` (`/scan`) and `realsense2_camera`; verify in RViz.
- **Bonus:** `slam_toolbox` mapping → save map → **Nav2** autonomous A→B; tune costmaps/controllers.
- Create unified **bringup launch files** and an RViz config for demos.

**Primary components:** Axiomtek SBC, RPLIDAR A1, RealSense D455.

---

## Part 4 — Execution Phases & Milestones

| Phase | Goal | P1 (Mech) | P2 (Elec/FW) | P3 (Software) | Shared milestone |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **0. Setup** | Plan & spec | Measure parts, start CAD | Power-tree design, source 5V buck | Install ROS2, repo + branching | Architecture + BOM agreed |
| **1. Subsystem build** | Each part works alone | Print v1 plates & mounts | Motors spin via Arduino on bench | base_controller + teleop in sim | — |
| **2. Drive bring-up** | Wheels move from a command | Mount motors/wheels | PID + serial protocol done | `/cmd_vel`→Arduino verified | **Robot moves on blocks** |
| **3. Integration** | Everything on the chassis | Final assembly + cable mgmt | Full power-on, e-stop test | All drivers up in RViz | **Powered, assembled robot** |
| **4. Teleop (MAIN)** | Flawless A→B by remote | Tune CoG / fix rattles | Tune PID, deadband | Tune teleop, odometry drift | **✅ Teleop A→B demo** |
| **5. SLAM (BONUS)** | Map an area | Rigid LIDAR mount confirmed | Stable 5V to LIDAR | slam_toolbox map saved | **Map produced** |
| **6. Nav (BONUS)** | Autonomous A→B | — | — | Nav2 tuned, goals work | **✅ Autonomous A→B** |

**Critical path to the main goal:** P2's motor firmware + P3's base_controller + P1's assembled chassis must all land by Phase 4. Protect that intersection.

---

## Part 5 — Risk Register & Decisions to Make Early

| # | Risk / Decision | Impact | Recommended action |
| :--- | :--- | :--- | :--- |
| **1** | **Only one 2-channel driver for 4 mecanum motors.** | Can't do true holonomic strafing with one MDDRC10. | For the **main goal (A→B)**, wire each side's 2 motors in parallel per channel → skid-steer differential (works fine). For true mecanum, **acquire a 2nd MDDRC10** (4 channels). Decide in Phase 0. |
| **2** | **6V motors on an 18.5V supply.** | Over-volting can burn motors. | Software-**cap PWM duty** (~30–35% ≈ 6V equivalent) in Arduino firmware, *or* confirm motors are a 12V/24V variant before full power. |
| **3** | **No 5V buck converter in the kit.** | LIDAR/Arduino/PCA9685 have no clean rail; driver's 5V tap is marginal (LIDAR inrush 600 mA > comfortable). | Source a **5V/3A buck module** in Phase 0. |
| **4** | **Arduino VIN max 12V.** | Direct battery feed destroys the board. | Power Arduino from SBC USB or the 5V buck only. |
| **5** | **LIDAR mount vibration / occlusion.** | Noisy `/scan`, bad SLAM. | Rigid top mount, unobstructed 360°; isolate from motor vibration. |
| **6** | **Odometry drift on mecanum/skid-steer.** | Poor dead-reckoning, Nav2 errors. | Careful ticks-per-meter + track-width calibration (P2+P3); lean on LIDAR for correction. |
| **7** | **LiPo safety.** | Fire / cell damage. | Fuse + e-stop, balance-charge only, enforce 3.0V/cell floor, never leave charging unattended. |

---

## Part 6 — Suggested "Missing / To-Source" Parts
- **5V/3A (or higher) buck converter** — for LIDAR, Arduino, PCA9685 logic. *(Needed)*
- **Second Cytron MDDRC10** — only if you want true 4-wheel mecanum motion. *(Optional, for bonus)*
- **Inline fuse + main power switch / e-stop button**, XT60 pigtails, ferrules/connectors. *(Safety — needed)*
- **USB cables:** A→B (Arduino), micro/USB-UART (LIDAR), USB-C (RealSense).

---

## Part 7 — Definition of Done
- [ ] All components housed in a 3D-printed enclosure with serviceable access and SBC airflow.
- [ ] Robot powers on safely (fuse + e-stop verified), all rails at correct voltage.
- [ ] **Main goal:** Operator drives the robot A→B by teleop, smoothly and repeatably.
- [ ] Odometry + TF + LIDAR + camera all visible in RViz.
- [ ] **Bonus:** A SLAM map is built and saved.
- [ ] **Bonus:** Robot navigates A→B autonomously with Nav2.
