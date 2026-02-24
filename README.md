# TABLE TENNIS TRAINER
## Spring 2026 Capstone Project

A high-speed, vision-guided robotic system designed to autonomously engage in a Table Tennis match with a human opponent. This project utilizes a distributed ROS2 architecture across two NVIDIA Jetson Nanos to handle GPU-accelerated vision processing, interfaced via Ethernet communication to an STM32 microcontroller which will command the joint positions on the arm.

## Contributors

### Sensors Team

Liam Dabelstein

Jake VanEssendelft

Lucian Bracy

John Duplain

### Mechanical Team

Nathaniel LeBlanc

Ben Pumphrey

Grant Monroe

Zane Peeler

Tae Lee

Theodore Williamson

## System Architecture & Hardware Integration

The project utilizes a distributed hardware stack to minimize processing latency. High-level perception is handled by NVIDIA Jetsons, while low-level motor execution is managed by an STM32 microcontroller.

### **Hardware Components**
* **Compute:** 2x NVIDIA Jetson Nano (Developer Kit).
* **Vision:** 2x Arducam OV9281 (Global Shutter, MIPI CSI-2).
* **Microcontroller:** STM32 (connected via Ethernet).
* **Actuation:** add section here @mech team.
* **Networking:** Gigabit Ethernet Switch.


### **Hardware Mapping**
| Component | Connection | Purpose |
| :--- | :--- | :--- |
| **Arducams** | MIPI CSI-2 | 240 FPS Global Shutter image capture at 640x400 resolution. |
| **Jetson A ↔ B** | Ethernet (Static IP) | Sharing 2D ball coordinates for stereo fusion. |
| **Jetson A ↔ STM32** | Ethernet (UDP) | Sending joint angles to the arm at high frequency. |
| **STM32 ↔ Motors** | PWM / Step-Dir | Direct electrical control of the arm. |

---

### ROS 2 Package Glossary

Each package in the `src/` directory is designed with **Modularity** in mind and serves a specific purpose in the system.

### **1. ttt_msgs**
* **Type:** Interface Package.
* **Function:** Defines the custom "language" of the robot. It contains the `.msg` files for ball detection, tracking, and arm status.

### **2. ttt_camera**
* **Type:** Hardware Driver.
* **Function:** Interfaces with the MIPI CSI-2 ports on the Jetson. It uses a hardware-accelerated GStreamer pipeline to provide raw grayscale frames.
* **Output:** `sensor_msgs/Image` (240 FPS).

### **3. ttt_vision**
* **Type:** Perception Engine.
* **Function:** The "eyes and brain" of the system. It handles:
    * **Detection:** Finding the ball in 2D pixels (using GPU/VPI).
    * **Stereo Fusion:** Triangulating two 2D points into a 3D (X, Y, Z) coordinate.
    * **Prediction:** Estimating where the ball will be using a Kalman Filter.

### **4. ttt_control**
* **Type:** Decision Maker.
* **Function:** Performs **Inverse Kinematics (IK)**. It takes the predicted ball destination and calculates the exact joint angles required for the paddle to hit the ball.

### **5. ttt_hardware**
* **Type:** Communication Bridge.
* **Function:** The "translator." It takes ROS 2 joint commands and packs them into a binary UDP format that the **STM32** can understand with minimal overhead.

### **6. ttt_calibration**
* **Type:** Configuration.
* **Function:** Stores the physical offsets of the cameras (e.g., the 45° tilt). It publishes the **TF2 (Transform)** tree so the vision system knows where the table is relative to the cameras.

### **7. ttt_bringup**
* **Type:** System Orchestrator.
* **Function:** Contains Python launch scripts. Allows for start of the entire system with a single command.

## System Topology

### Jetson A (`insert IP address here`)
- **Nodes:** `camera_left`, `ball_detector_left`, `stereo_fusion`, `predictor`, `ik_solver`, `stm32_bridge`.
- **Hardware:** Left Arducam, Ethernet to STM32, Ethernet to Slave Jetson.

### Jetson B (`insert IP address here`)
- **Nodes:** `camera_right`, `ball_detector_right`.
- **Hardware:** Right Arducam.

#### Package Structure

All packages in this workspace follow the standard ROS 2 C++ layout.

Using `ttt_vision` as an example:
```text
ttt_vision/
├── CMakeLists.txt          # Build instructions
├── package.xml             # Metadata and dependencies
├── include/
│   └── ttt_vision/         # Header files
│       └── ball_math.hpp   # Declarations and shared logic
├── src/
│   └── vision_node.cpp   # Node implementation
└── msg/                    # (Only in ttt_msgs) Custom data types

```

### How to Edit and Add Features
#### **1. Editing Headers (.hpp)**
* **Location:** Always place headers in `include/<package_name>/`.
* If you add a new header, ensure ament_export_include_directories(include) is in CMakeLists.txt so other packages can see it.

#### **2. Adding a New Node (.cpp)**
* **Location:** Place the implementation file in the `src/` directory.
* **Registration:** You must register the new node in the `CMakeLists.txt` file so the compiler knows to build it. Add the following lines:

```cmake
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp ttt_msgs)

# This part ensures the executable is installed to the lib folder
install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME}
)
```

#### 3. **Building and Running a Single Package**
* **Build:** `colcon build --symlink-install --packages-select <package_name>`
* **Source:** `source install/setup.bash`
* **Run:** `ros2 run <package_name> <executable_name>`


# Camera Calibration Guide

## Overview

The Table Tennis Trainer uses a **stereo vision system** with two cameras mounted at **45° angles** above opposite sides of the table. This configuration maximizes field of view while minimizing occlusion from the robot arm and player.

## Why this Matters

Without calibration, the system cannot:
- Correct for lens distortion (curves appear in straight lines)
- Accurately measure distances in 3D space
- Transform pixel coordinates to real-world positions
- Perform stereo triangulation for depth estimation

With proper calibration:
- Straight lines appear straight (lens distortion corrected)
- Pixel coordinates accurately map to millimeter positions
- Stereo fusion produces accurate 3D ball positions
- Robot arm can intercept the ball precisely

---

## Camera Mounting Configuration

### Physical Setup

```
                Table (Top View)
    ┌───────────────────────────────────────┐
    │                                       │
    │            Opponent's Side            │
    │                                       │
    ├───────────────────────────────────────┤
    ├───────────────────────────────────────┤
    │                                       │
    │              Robot's Side             │
    │                                       │
    └───────────────────────────────────────┘

Camera A (Left)              Camera B (Right)
     ↓                              ↓
    [ ]                            [ ]
     ╲ 45°                    45° ╱
      ╲                          ╱
       ╲                        ╱
        ╲                      ╱
         ╲____________________╱
              Table Surface
```

### Side View (45° Angle)

```
        Camera
          [ ]
           │╲
           │ ╲ 45°
    Height │  ╲
    (~1m)  │   ╲
           │    ╲
           │     ╲
    ═══════════════════  ← Table Surface
```

**Key Measurements:**
- **Height:** ~1 meter above table surface
- **Angle:** 45° downward tilt (pitch = -0.785 radians)
- **Baseline:** Distance between cameras (~1.5 meters)
- **Position:** Cameras above left/right table edges

---
## Two Types of Calibration

### 1. Intrinsic Calibration (Camera Properties)

Real lenses introduce distortion. Straight lines in the world appear curved in the image. Intrinsic calibration provides the math to "un-distort" the image.

**What it measures:**
- Focal length (fx, fy) - how the lens "zooms"
- Principal point (cx, cy) - optical center of the image
- Distortion coefficients (k1, k2, k3, p1, p2) - lens curvature effects

**Camera Matrix:**
```
K = [ fx   0   cx ]
    [  0  fy   cy ]
    [  0   0    1 ]
```

- `fx, fy`: Focal length in pixels (how many pixels per degree of field of view)
- `cx, cy`: Image center point (usually close to width/2, height/2)

**Example for OV9281 @ 640x400p resolution:**
```yaml
camera_matrix:
  data: [458.123, 0.0, 320.456,    # fx, 0, cx
         0.0, 457.891, 201.234,     # 0, fy, cy
         0.0, 0.0, 1.0]             # 0, 0, 1
```
---

### 2. Extrinsic Calibration (Camera Position & Orientation)

This is used to transform pixel coordinates from the camera's tilted frame to the table's horizontal frame. The 45° angle means pixels don't directly map to table positions.

**What it measures:**
- Translation (x, y, z) - camera position in 3D space
- Rotation (roll, pitch, yaw) - camera orientation


**Coordinate Frames:**

```
World Frame (origin)
  └─ Table Frame (table surface, flat)
      ├─ Camera Left Frame (45° tilted)
      ├─ Camera Right Frame (45° tilted)
      └─ Robot Base Frame (arm mounting point)
```

**Transform Chain:**
```
Pixel (u, v) 
  → Camera Frame (x_cam, y_cam, z_cam) [using intrinsics]
  → Table Frame (x_table, y_table, z_table) [using extrinsics]
  → Robot Frame (x_robot, y_robot, z_robot) [for arm control]
```

**Example Extrinsic Parameters:**
```yaml
# Left camera - 45° angle, left side of table
left_camera:
  x: -0.75       # 75cm left of table center  # EDIT HERE WITH ACTUAL VALUE
  y: 0.0         # Aligned with table center lengthwise
  z: 1.0         # 1 meter above table surface # EDIT HERE WITH ACTUAL VALUE
  roll: 0.0      # No rotation around x-axis (radians)
  pitch: -0.785  # 45° down = -45 * π/180 = -0.785 rad
  yaw: 0.0       # Facing straight ahead
```

---

## How the 45° Angle Affects Vision

### Field of View Coverage

**Mathematical Transform:**

```
                 Camera Frame (tilted 45°)
                        ↑ Z_cam
                        │
                        │  45°
                        │╱
    Table Frame  ───────┼────────→ Y_cam
    (horizontal)        │
         ↑             ╱│
         │Z          ╱  
         │         ╱   
         │       ╱    
         └──────→ Y

Rotation Matrix (45° pitch):
R = [ 1      0         0      ]
    [ 0   cos(45°) -sin(45°) ]
    [ 0   sin(45°)  cos(45°) ]

R = [ 1    0       0     ]
    [ 0   0.707  -0.707  ]
    [ 0   0.707   0.707  ]
```

---

## Calibration Process

### Quick Start: Using Default Values

**For rapid prototyping and initial testing**, you can skip intrinsic calibration and use estimated default values:

**Steps:**

1. **Use pre-configured defaults:**

The `ttt_calibration` package comes with reasonable default values for the OV9281 camera:

```yaml
# config/camera_left_intrinsic.yaml (defaults)
image_width: 640
image_height: 400

camera_matrix:
  rows: 3
  cols: 3
  data: [500.0, 0.0, 320.0,    # fx ≈ 500 (typical for OV9281)
         0.0, 500.0, 200.0,     # fy ≈ 500, cx = width/2, cy = height/2
         0.0, 0.0, 1.0]

distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]  # Assume minimal distortion

camera_name: "ov9281_left"

```
2. **Manually measure camera positions:**

Use a tape measure and level app to measure:

```bash
# EDIT WITH ACTUAL VALUES
# Measure from table center to each camera
# Left camera: 75cm left, 100cm high, 45° down
# Right camera: 75cm right, 100cm high, 45° down

nano ~/TTT-Capstone-Sensors/tabletennistrainer_ws/src/ttt_calibration/config/stereo_extrinsic.yaml
```

3. **Update with your measurements:**

```yaml
left_camera:
  # EDIT WITH ACTUAL VALUES
  x: -0.75      # YOUR MEASUREMENT: distance left of center (meters)
  y: 0.0        # YOUR MEASUREMENT: forward/back position
  z: 1.0        # YOUR MEASUREMENT: height above table
  roll: 0.0
  pitch: -0.785 # 45° = -0.785 radians (use phone level app to verify)
  yaw: 0.0

right_camera:
  x: 0.75       # YOUR MEASUREMENT: distance right of center
  y: 0.0
  z: 1.0
  roll: 0.0
  pitch: -0.785
  yaw: 0.0
```

4. **Launch and test:**

```bash
# Launch TF broadcaster with your measurements
ros2 launch ttt_calibration calibration.launch.py

# Test the system
ros2 launch ttt_bringup full_system.launch.py
```

**Expected Accuracy with Defaults:**
- Position accuracy: ±2-5 cm
- Good enough for: trajectory testing, algorithm development, initial demos
- Not good enough for: competition-grade accuracy, precision hitting

**When to do full calibration:**
- When you need more accuracy
- Before final competition/demo
- If ball tracking looks off

---

---

### Alternative Calibration Methods

If you don't have access to a printed checkerboard, consider these options:

#### Option A: ArUco Marker Board
- Easier to print and detect than checkerboards
- More robust to partial occlusion
- Single-page printout

```bash
# Install ArUco package
sudo apt install ros-humble-aruco-opencv

# Generate marker board
ros2 run aruco_opencv create_board_charuco \
  --sl 0.04 --ml 0.02 --h 5 --w 7 \
  --d 10 board.png
```

#### Option B: Display Pattern on Screen
- No printing required
- Use laptop/tablet to display checkerboard
- Less accurate due to screen reflections

**Resources:**
- Online patterns: https://markhedleyjones.com/projects/calibration-checkerboard-collection
- Generate custom: https://calib.io/pages/camera-calibration-pattern-generator

#### Option C: Incremental Calibration
1. Start with defaults (as shown above)
2. Test system and identify errors
3. Calibrate only when accuracy becomes critical
4. Measure improvement to justify calibration effort

---

### Part 1: Full Intrinsic Calibration (Per Camera) - Optional but Recommended

**Equipment needed:**
- Printed checkerboard pattern (8x6 internal corners, 24mm squares)
- Flat, rigid surface (cardboard backing)

**Steps:**

1. **Start camera node:**
```bash
ros2 run ttt_camera camera_node --ros-args \
  -p camera_id:=left \
  -p width:=640 \
  -p height:=400 \
  -p fps:=240
```

2. **Launch calibration tool:**
```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.024 \
  image:=/camera/left/image_raw \
  camera:=/camera/left
```

3. **Collect images:**
Move the checkerboard around to fill coverage bars:
- **X bar (green):** Move checkerboard left ↔ right
- **Y bar (green):** Move checkerboard up ↔ down
- **Size bar (green):** Move closer ↔ farther from camera
- **Skew bar (green):** Tilt checkerboard at various angles

**Target: 50-100 images with all bars GREEN**

4. **Calibrate and save:**
- Click **CALIBRATE** (takes 30-60 seconds)
- Click **SAVE** → saves to `/tmp/calibrationdata.tar.gz`

5. **Extract results:**
```bash
cd /tmp
tar -xzf calibrationdata.tar.gz
cat ost.yaml
```

6. **Update config file:**
Copy the `camera_matrix` and `distortion_coefficients` values to:
```
tabletennistrainer_ws/src/ttt_calibration/config/camera_left_intrinsic.yaml
```

7. **Repeat for right camera**

---

### Part 2: Extrinsic Calibration (Camera Positions)

**Two methods:**

#### Method A: Manual Measurement

1. **Measure physical positions:**
   - Tape measure from table center to each camera
   - Record: x, y, z positions in meters
   - Measure mounting angles with protractor or level app

2. **Update config:**
```bash
nano src/ttt_calibration/config/stereo_extrinsic.yaml
```

3. **Enter measured values:**
```yaml
left_camera:
  x: -0.75      # Measured: 75cm left of center
  y: 0.0        # Measured: aligned with center
  z: 1.0        # Measured: 100cm above table
  roll: 0.0     # Measured: no twist
  pitch: -0.785 # Measured: 45° down
  yaw: 0.0      # Measured: straight ahead
```

**Accuracy:** ±2-5cm

---

#### Method B: Stereo Checkerboard Calibration

Use ROS2 stereo calibration with checkerboard visible to BOTH cameras simultaneously:

```bash
# Run both cameras
ros2 launch ttt_bringup jetsonA.launch.py &
ros2 launch ttt_bringup jetsonB.launch.py &

# Stereo calibration
ros2 run camera_calibration cameracalibrator \
  --approximate 0.1 \
  --size 8x6 --square 0.024 \
  right:=/camera/right/image_raw left:=/camera/left/image_raw \
  right_camera:=/camera/right left_camera:=/camera/left
```

**Requirements:**
- Checkerboard must be visible to BOTH cameras at once
- Collect images from multiple positions
- More accurate than manual measurement

**Accuracy:** ±1-2mm

---

### Part 3: Verification

After calibration, verify the system works:

**1. Launch TF broadcaster:**
```bash
ros2 launch ttt_calibration calibration.launch.py
```

**2. Visualize transforms:**
```bash
# View transform tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo table camera_left_optical_frame
```

**3. Test with real ball:**
- Launch full system
- Toss ping pong ball across table
- Verify 3D position looks reasonable
- Check if positions track smoothly

---


## Files used for Calibration

```
ttt_calibration/
├── config/
│   ├── camera_left_intrinsic.yaml    # Lens properties (Camera A)
│   ├── camera_right_intrinsic.yaml   # Lens properties (Camera B)
│   └── stereo_extrinsic.yaml         # Camera positions & 45° angles
├── launch/
│   └── calibration.launch.py         # Launches TF broadcaster
└── src/
    └── tf_broadcaster_node.cpp       # Publishes coordinate transforms
```

---

## Quick Reference Commands

```bash
# List camera controls
v4l2-ctl -d /dev/video0 --list-ctrls

# Run intrinsic calibration
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.024 \
  image:=/camera/left/image_raw camera:=/camera/left

# Launch TF transforms
ros2 launch ttt_calibration calibration.launch.py

# View transform tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo table camera_left_optical_frame
```

---

## Resources

- [ROS Camera Calibration Tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
- [OpenCV Calibration Guide](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- [Understanding Camera Calibration](https://learnopencv.com/camera-calibration-using-opencv/)
- [Stereo Vision Fundamentals](https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html)




### Building the Project
We use `colcon` with symlink installation to avoid redundant builds.

```bash
# Standard Build
./build.sh

# If the Jetson freezes (RAM conservation mode)
colcon build --symlink-install --parallel-workers 1

```