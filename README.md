# Color-Aware Maze Navigation using ROS 2 and TurtleBot3
**Author:** Nirmal Raja Karuppiah Loganathan

---

## 🚀 Project Overview

This project demonstrates a complete **ROS 2-based autonomous navigation** system for a TurtleBot3 robot. The robot follows predefined waypoints while performing real-time **obstacle avoidance based on object color** (red or blue).

### Two Main Runs:
- **Basic Run** (`maze_solver.py`) – Straightforward waypoint following.
- **Optimized Run** (`msg.py`) – Performs lateral detours using color-aware logic.

The robot:
- Uses **SLAM Toolbox** to build the map.
- Navigates in a **custom Gazebo maze world**.
- Uses **LiDAR and RGB camera fusion** to detect red/blue obstacles.
- Performs side-steps: left for blue, right for red.

---

## 🧱 Maze World Setup

A custom maze was designed in Gazebo with:
- Walls to form narrow paths.
- Red and blue cylinders placed to block default paths.

This forced a **visual contrast** between the basic and color-aware runs:
- The first run ignores object color.
- The second run interprets object color to decide detour direction.

---

## 🗺️ Mapping & First Run (`maze_solver.py`)

1. **SLAM Map Generation**:
   - TurtleBot3 follows a vertical list of waypoints to build the map using `slam_toolbox`.
   - SLAM is run in async mode.

2. **Basic Waypoint Navigation**:
   - After the map is saved, the robot is relaunched.
   - It follows the same waypoints again with no avoidance logic.

This run acts as a baseline to compare the performance of the color-aware version.

---

## 🎯 Color-Aware Navigation (`msg.py`)

In this run, the robot performs **dynamic object avoidance** using:
- Real-time RGB camera data
- HSV color masking
- LiDAR fusion
- FSM-based detour logic

---

### 🔍 Detection Pipeline

- **Color Masking (OpenCV + HSV)** for:
  - Red: Ranges [0–10] and [160–179]
  - Blue: Range [100–140]
- **Blob Centroid Calculation**
- **Angle Estimation** based on camera HFOV
- **Distance Estimation** from LiDAR
- **Debounce Logic** for stable detection:
  - Contour area threshold
  - Object distance < 2 feet
  - Multiple consistent frames

---

### 📐 Lateral Shift Calculation

The offset for side-step is computed using:

```math
FOV_rad = (π / 180) × FOV

PPM = W / (2 × tan(FOV_rad / 2) × D)

w_px = sqrt(A)

offset_px = |(w_px / 2) - (W / 2)|

LateralShift_m = offset_px / PPM

```
---

### 📐 Variable Definitions

**Where:**

- `W` = Image width (px)  
- `FOV` = Camera field of view (°)  
- `D` = Object distance (m) from LiDAR  
- `A` = Contour area (px²)  
- `w_px` = Estimated object width (px)

---

### ⚙️ FSM-Based Behavior Logic

| Color Detected | Action          |
|----------------|-----------------|
| 🔵 Blue        | Side-step Left  |
| 🔴 Red         | Side-step Right |

The robot computes the new goal by **translating its current position laterally** and resumes the navigation to the next waypoint.

---

### 📂 Resources

- **GitHub Repo**: [Source Code](https://github.com/your-username/turtlebot3-color-aware-navigation)
- **Demo Videos**:
  - [🗺️ SLAM Mapping + First Run](https://drive.google.com/file/d/1UaMZxqPJT6QfQmIylV5r0fx4cvESgiKK/view?usp=sharing)
  - [🎯 Optimized Color-Aware Navigation](https://drive.google.com/file/d/1tP28uWU_Dt9cDNH5ABu3sfvdU57wGCHI/view?usp=sharing)

---

### ✅ Conclusion

This project shows that a TurtleBot3 can:

- ✅ Follow waypoints with Nav2  
- ✅ Build a 2D map using SLAM Toolbox  
- ✅ Detect red and blue objects using real-time RGB + LiDAR data  
- ✅ Perform lateral detours based on perceived color and location  

The visual and logical contrast between the two runs clearly demonstrates the value of the color-aware logic.

**Future improvements may include:**

- 🔧 More robust detection in low light  
- 🔄 Smoother detour transitions  
- 🧠 Adaptive thresholds based on environment

---
This README was formatted with assistance from [ChatGPT](https://chat.openai.com/)

