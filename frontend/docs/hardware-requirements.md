---
sidebar_position: 3
---

# Hardware Requirements

This page details the hardware components required for hands-on labs in the **Physical AI & Humanoid Robotics** course. You have two main options: **NVIDIA Jetson-based setup** (recommended for embedded robotics) or **Desktop/Laptop with GPU** (suitable for simulation-heavy workflows).

## Overview

The course is designed to be flexible with hardware requirements. You can complete **most exercises with simulation only** (Option 2), but physical hardware (Option 1) is highly recommended for real-world experience with embedded AI and sensor integration.

### Recommended Configuration

**Option 1: NVIDIA Jetson Platform**  Best for embedded robotics and edge AI development

## Option 1: NVIDIA Jetson Platform >

### Core Components

#### 1. **NVIDIA Jetson Orin Nano Developer Kit (8GB)**

- **Specifications**:
  - **GPU**: 1024-core NVIDIA Ampere GPU with 32 Tensor Cores
  - **CPU**: 6-core Arm Cortex-A78AE @ 1.5 GHz
  - **Memory**: 8GB 128-bit LPDDR5 (68 GB/s bandwidth)
  - **Storage**: microSD card slot (64GB+ recommended)
  - **AI Performance**: 40 TOPS (INT8)
  - **Power**: 7W / 15W modes
  - **Connectivity**: 1x Gigabit Ethernet, 2x USB 3.2, 1x USB-C (device mode)
  - **Display**: 1x HDMI 2.0, 2x MIPI CSI camera connectors

- **Price**: $499 USD
- **Purchase**: [NVIDIA Store](https://store.nvidia.com/), [Amazon](https://amazon.com/), [Seeed Studio](https://www.seeedstudio.com/)

- **Why This Hardware**:
  - Sufficient GPU power for real-time YOLOv8 inference (30+ FPS)
  - ROS 2 Humble officially supported
  - Low power consumption (suitable for mobile robots)
  - Extensive community support and tutorials

#### 2. **Intel RealSense D435i Depth Camera**

- **Specifications**:
  - **Depth Technology**: Active IR stereo
  - **Depth Range**: 0.3m - 3m (ideal), up to 10m
  - **Depth Resolution**: 1280x720 @ 90 FPS
  - **RGB Sensor**: 1920x1080 @ 30 FPS
  - **IMU**: Built-in gyroscope + accelerometer
  - **Interface**: USB 3.1 Gen 1 (USB-C connector)
  - **SDK**: librealsense2 (ROS 2 wrapper available)

- **Price**: $349 USD
- **Purchase**: [Intel Store](https://www.intelrealsense.com/), [Amazon](https://amazon.com/)

- **Why This Hardware**:
  - High-quality depth data for 3D perception
  - IMU for sensor fusion and SLAM
  - Excellent ROS 2 integration (`realsense2_camera` package)
  - Used in industry (research labs, warehouse robots)

**Alternative**: [Oak-D Lite](https://shop.luxonis.com/products/oak-d-lite-1) ($149)  Lower cost, spatial AI capabilities

#### 3. **Accessories**

| **Item**                          | **Specification**                    | **Price** |
|-----------------------------------|--------------------------------------|-----------|
| microSD Card                       | 64GB UHS-I U3 (SanDisk Extreme)     | $15       |
| USB-C Power Supply                 | 5V/3A (Jetson Orin Nano)            | $12       |
| Ethernet Cable (Cat 6)             | 3ft (for reliable SSH connection)    | $8        |
| Cooling Fan (optional)             | 5V PWM fan for sustained workloads   | $15       |
| HDMI Cable + Monitor               | For initial setup (not needed after) | $0-50     |

**Total Cost (Option 1)**: **$898** (Jetson + RealSense + accessories)

---

## Option 2: Desktop/Laptop with GPU =»

### Minimum Requirements

- **Operating System**: Ubuntu 22.04 LTS (native or WSL2 with GUI support)
- **CPU**: 4-core Intel i5/AMD Ryzen 5 or better
- **RAM**: 16GB minimum (32GB recommended for Isaac Sim)
- **GPU**: NVIDIA RTX 3060 or better (8GB+ VRAM)
  - **Why GPU**: Isaac Sim, YOLOv8 training, and real-time rendering
  - **Alternatives**: RTX 3070/3080/4060/4070 (better for large-scale simulation)
- **Storage**: 50GB free disk space (100GB+ for datasets and simulation scenes)
- **Display**: 1920x1080 or higher resolution

### Software Compatibility

- **NVIDIA Drivers**: 525+ (for CUDA 12.x support)
- **Docker**: Required for containerized ROS 2 development
- **Isaac Sim**: Requires RTX GPU (Turing architecture or newer)

### Recommended Laptop Models

If you don't have a desktop, consider these laptop options:

| **Model**                          | **GPU**          | **RAM** | **Price**  |
|------------------------------------|------------------|---------|------------|
| ASUS ROG Zephyrus G14 (2024)       | RTX 4060 (8GB)   | 16GB    | $1,399     |
| Lenovo Legion 5 Pro                | RTX 4070 (8GB)   | 32GB    | $1,699     |
| Dell XPS 15 (with RTX)             | RTX 4050 (6GB)   | 16GB    | $1,499     |

**Note**: MacBooks (M1/M2/M3) are **not recommended** due to limited ROS 2 support and lack of CUDA.

**Total Cost (Option 2)**: **$0-1,699** (depending on existing hardware)

---

## Option 3: Cloud GPU Instances 

If you lack local GPU hardware, you can use cloud platforms for simulation and training:

### Recommended Platforms

| **Provider**        | **Instance Type** | **GPU**         | **Cost**         |
|---------------------|-------------------|-----------------|------------------|
| AWS EC2             | g5.xlarge         | A10G (24GB)     | $1.01/hour       |
| Google Cloud        | n1-standard-4     | T4 (16GB)       | $0.61/hour       |
| Paperspace Gradient | P5000             | Quadro (16GB)   | $0.51/hour       |
| Lambda Labs         | 1x RTX A6000      | A6000 (48GB)    | $0.80/hour       |

**Estimated Monthly Cost**: $50-150 (assuming 50-150 hours of usage)

### Pros and Cons

 **Pros**:
- No upfront hardware investment
- Access to high-end GPUs (A100, A6000)
- Scalable for large simulations

L **Cons**:
- No access to physical sensors (camera, depth, IMU)
- Network latency for remote development
- Ongoing monthly costs

---

## Required Sensors (Physical Labs Only)

These sensors are **optional** but required for specific labs:

### 1. **LiDAR (Optional for Weeks 3-4)**

- **Recommended**: [SLAMTEC RPLiDAR A1M8](https://www.slamtec.com/en/Lidar/A1) ($99)
  - 360° scanning, 12m range, 8000 samples/sec
  - USB interface, ROS 2 driver available (`sllidar_ros2`)

- **Alternative**: [YDLIDAR X2L](https://www.ydlidar.com/products/view/1.html) ($89)

### 2. **IMU (Optional for Weeks 7-8)**

- **Recommended**: [SparkFun 9DoF IMU Breakout - ICM-20948](https://www.sparkfun.com/products/15335) ($14.95)
  - 3-axis gyro, 3-axis accelerometer, 3-axis magnetometer
  - I2C/SPI interface

- **Alternative**: Use RealSense D435i built-in IMU (included if you have the camera)

### 3. **Microphone (Optional for Weeks 9-10)**

- **Recommended**: [Blue Yeti USB Microphone](https://www.bluemic.com/en-us/products/yeti/) ($99)
  - Studio-quality audio for speech recognition
  - USB plug-and-play

- **Budget Alternative**: [Logitech C920 Webcam](https://www.logitech.com/en-us/products/webcams/c920-pro-hd-webcam.960-000764.html) ($79, includes mic + camera)

---

## Robot Platforms (Optional)

If you have access to a physical robot, you can test code on real hardware:

### Supported Platforms

| **Robot**                 | **Type**        | **ROS 2 Support** | **Price**       |
|---------------------------|-----------------|-------------------|-----------------|
| TurtleBot 4               | Mobile robot    |  Humble         | $1,495          |
| Unitree Go1 Edu           | Quadruped       |  (community)    | $2,700          |
| Trossen ViperX 300        | 6-DOF arm       |  Humble         | $1,299          |
| DYNAMIXEL OpenMANIPULATOR | 5-DOF arm       |  Humble         | $699            |

**Note**: Robot platforms are **not required** for this course. All labs can be completed in simulation (Gazebo, Isaac Sim).

---

## Software Requirements (All Options)

The following software is required regardless of hardware choice:

### Operating System

- **Ubuntu 22.04 LTS** (Jammy Jellyfish)
  - **Native installation** recommended for best performance
  - **WSL2** (Windows Subsystem for Linux) supported with GUI apps enabled
  - **Docker** containers supported for isolated environments

### Core Software Stack

| **Software**         | **Version**      | **Purpose**                          |
|----------------------|------------------|--------------------------------------|
| ROS 2 Humble         | LTS (2022-2027)  | Robot middleware framework           |
| Python               | 3.10+            | Scripting, ML, ROS 2 nodes           |
| GCC/G++              | 11.x             | C++ compilation                      |
| CMake                | 3.22+            | Build system                         |
| Colcon               | Latest           | ROS 2 workspace build tool           |
| Docker               | 24.x             | Containerized development            |

### Deep Learning & Vision

| **Library**          | **Version**      | **Purpose**                          |
|----------------------|------------------|--------------------------------------|
| PyTorch              | 2.0+             | Deep learning framework              |
| OpenCV               | 4.x              | Computer vision (Python + C++)       |
| Ultralytics          | Latest           | YOLOv8 object detection              |
| CUDA Toolkit         | 12.x             | GPU acceleration (NVIDIA only)       |

### Simulation

| **Software**         | **Version**      | **Purpose**                          |
|----------------------|------------------|--------------------------------------|
| NVIDIA Isaac Sim     | 2023.1+          | High-fidelity robot simulation       |
| Gazebo               | Harmonic (latest)| General-purpose robot simulator      |
| MuJoCo               | 3.x              | Physics engine for RL                |

### Development Tools

- **VS Code** with ROS extension
- **Git** (version control)
- **Terminator** or **tmux** (terminal multiplexer)

---

## Storage Requirements

| **Component**           | **Disk Space** |
|-------------------------|----------------|
| ROS 2 Humble (full)      | 5GB            |
| Isaac Sim                | 30GB           |
| PyTorch + CUDA libraries | 8GB            |
| Datasets (ImageNet, COCO)| 10-50GB        |
| Workspace builds         | 5GB            |
| **Total**                | **58-98GB**    |

**Recommendation**: Use an external SSD (500GB+) for datasets and simulation assets.

---

## Quick Start Checklist

Before Week 1, ensure you have:

- [ ] Hardware decision made (Option 1, 2, or 3)
- [ ] Ubuntu 22.04 LTS installed (native or WSL2)
- [ ] NVIDIA drivers installed (if using GPU)
- [ ] ROS 2 Humble installed ([installation guide](https://docs.ros.org/en/humble/Installation.html))
- [ ] Python 3.10+ with pip and venv
- [ ] Git configured with GitHub SSH keys
- [ ] (Optional) Jetson flashed with JetPack 5.1.2+
- [ ] (Optional) RealSense camera drivers installed

---

## Hardware Troubleshooting

### Common Issues

**Jetson Orin Nano won't boot**:
- Ensure microSD card is flashed with correct JetPack image (5.1.2+)
- Use official NVIDIA power supply (5V/3A minimum)
- Check HDMI connection for initial setup

**RealSense camera not detected**:
```bash
# Install librealsense2
sudo apt install librealsense2-dkms librealsense2-utils
# Test camera
realsense-viewer
```

**GPU not detected in Isaac Sim**:
- Verify NVIDIA drivers: `nvidia-smi`
- Check CUDA version: `nvcc --version`
- Ensure Isaac Sim requires RTX GPU (Turing or newer)

---

## Cost Summary

| **Option**                  | **Upfront Cost** | **Monthly Cost** |
|-----------------------------|------------------|------------------|
| Option 1 (Jetson + Camera)   | $898             | $0               |
| Option 2 (Desktop GPU)       | $0-1,699         | $0               |
| Option 3 (Cloud GPU)         | $0               | $50-150          |

**Recommendation**: If you plan to continue in robotics, invest in **Option 1** (Jetson + camera) for long-term value and hands-on experience.

---

**Questions?** Use the `/hardware` skill to look up specifications:
```
/hardware Jetson Orin Nano
/hardware Intel RealSense D435i
```
