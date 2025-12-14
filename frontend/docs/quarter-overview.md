---
sidebar_position: 2
---

# Quarter Overview

This page provides a comprehensive overview of the **Physical AI & Humanoid Robotics** course, detailing the curriculum structure, weekly topics, assessments, and deliverables across the entire quarter.

## Course Duration

**Total Duration**: 10 weeks (1 quarter)
**Weekly Time Commitment**: 15-20 hours (lectures, labs, assignments, projects)
**Format**: Hybrid (online textbook + hands-on labs + virtual office hours)

## Learning Path

The course follows a progressive structure from foundational robotics concepts to advanced integration of AI and humanoid systems:

```
Week 1-2: ROS 2 Foundations
    �
Week 3-4: Computer Vision & Perception
    �
Week 5-6: Manipulation & Control
    �
Week 7-8: Locomotion & Simulation
    �
Week 9-10: Foundation Models & Integration
```

## Weekly Breakdown

### **Weeks 1-2: ROS 2 Foundations** =�

**Module**: Introduction to Robot Operating System 2

**Topics**:
- ROS 2 architecture (nodes, topics, services, actions)
- Workspace setup and build systems (colcon)
- Publisher/subscriber patterns
- Custom message and service definitions
- Launch files and parameter management
- ROS 2 CLI tools (ros2 topic, ros2 node, ros2 bag)

**Labs**:
- Install ROS 2 Humble on Ubuntu 22.04 / Jetson
- Create a simple talker/listener node in Python
- Implement a service client/server for robot control
- Record and playback sensor data with rosbag

**Deliverables**:
- Lab report: ROS 2 workspace setup
- Assignment 1: Custom publisher/subscriber system (temperature sensor simulation)

**Prerequisites**: Linux command line, Python 3.10+

---

### **Weeks 3-4: Computer Vision & Perception** =A

**Module**: Deep Learning for Robot Perception

**Topics**:
- Camera models and calibration
- Depth sensing (stereo vision, structured light, ToF)
- Object detection with YOLOv8 and vision transformers
- Semantic segmentation for scene understanding
- Point cloud processing (PCL library)
- Sensor fusion (camera + LiDAR + IMU)

**Labs**:
- Calibrate a RealSense D435i depth camera
- Deploy YOLOv8 on Jetson Orin for real-time object detection
- Implement 3D point cloud filtering and segmentation
- Fuse camera and LiDAR data for obstacle avoidance

**Deliverables**:
- Lab report: Vision pipeline integration
- Assignment 2: Object tracker for moving targets (Kalman filter)

**Prerequisites**: Computer vision basics, PyTorch or TensorFlow

---

### **Weeks 5-6: Manipulation & Control** >�

**Module**: Kinematics, Motion Planning, and Grasping

**Topics**:
- Forward and inverse kinematics (DH parameters, analytical methods)
- Jacobian matrices and velocity kinematics
- Path planning algorithms (RRT, RRT*, PRM)
- Trajectory generation (cubic splines, minimum jerk)
- MoveIt 2 framework for motion planning
- Grasp pose estimation with deep learning

**Labs**:
- Solve inverse kinematics for a 6-DOF robot arm
- Plan collision-free trajectories with MoveIt 2
- Implement a pick-and-place task in Gazebo simulation
- Train a grasp quality network with synthetic data

**Deliverables**:
- Lab report: MoveIt 2 pipeline configuration
- Assignment 3: Autonomous object sorting with grasping

**Prerequisites**: Linear algebra, basic robotics kinematics

---

### **Weeks 7-8: Locomotion & Simulation** >

**Module**: Humanoid Walking and Simulation Environments

**Topics**:
- Bipedal locomotion fundamentals (ZMP, center of mass control)
- Whole-body control and inverse dynamics
- Reinforcement learning for locomotion (PPO, SAC)
- NVIDIA Isaac Sim and Isaac Gym for robot simulation
- Sim-to-real transfer techniques (domain randomization, system identification)
- Gazebo and MuJoCo physics engines

**Labs**:
- Set up Isaac Sim with a humanoid robot model (Unitree H1, Digit)
- Train a walking policy using Isaac Gym PPO
- Implement balancing control with PD controllers
- Test sim-to-real transfer with domain randomization

**Deliverables**:
- Lab report: Isaac Sim environment setup
- Assignment 4: Train a bipedal walking policy from scratch

**Prerequisites**: Control theory basics, reinforcement learning familiarity

---

### **Weeks 9-10: Foundation Models & Integration** >�

**Module**: Integrating LLMs and Vision-Language Models with Robots

**Topics**:
- Large language models for task planning (GPT-4, Claude)
- Vision-language models (CLIP, LLaVA, GPT-4V)
- Multi-modal robot control (speech + vision + action)
- Prompt engineering for robotics tasks
- Real-time speech recognition and synthesis
- End-to-end robot applications (voice-commanded manipulation)

**Labs**:
- Integrate GPT-4 with ROS 2 for natural language commands
- Use CLIP for zero-shot object recognition
- Build a voice-controlled robot assistant
- Deploy a full-stack application (speech � vision � action)

**Deliverables**:
- Lab report: Foundation model integration
- **Final Project**: Multi-modal humanoid robot system (e.g., "Fetch me the red cup")

**Prerequisites**: Familiarity with LLMs, API integration

---

## Assessment Structure

| **Component**            | **Weight** | **Description**                                |
|--------------------------|------------|------------------------------------------------|
| Weekly Lab Reports       | 20%        | Hands-on exercises and implementation notes    |
| Assignments (4 total)    | 40%        | Programming assignments (10% each)             |
| Midterm Project          | 15%        | Weeks 5-6: Manipulation pipeline               |
| Final Project            | 25%        | Weeks 9-10: Multi-modal robot system           |

### Grading Rubric

- **A (90-100%)**: Complete implementation, robust error handling, clear documentation
- **B (80-89%)**: Functional implementation, minor bugs, adequate documentation
- **C (70-79%)**: Partial implementation, significant issues, incomplete documentation
- **D (60-69%)**: Minimal implementation, major bugs
- **F (below 60%)**: Non-functional or missing submission

## Required Hardware

### **Option 1: NVIDIA Jetson Platform** (Recommended)

- **NVIDIA Jetson Orin Nano Developer Kit** (8GB RAM, $499)
- **Intel RealSense D435i** depth camera ($349)
- **Logitech C920 HD Pro Webcam** (backup, $79)
- **64GB microSD card** (for Jetson OS)
- **USB-C power supply** (5V/3A for Jetson)

**Total**: ~$927 (Jetson + camera)

### **Option 2: Desktop/Laptop with GPU**

- **Ubuntu 22.04** (native or WSL2)
- **NVIDIA RTX GPU** (3060 or better, 8GB+ VRAM for simulation)
- **16GB RAM** minimum (32GB recommended for Isaac Sim)
- **50GB free disk space** (for ROS 2, Isaac Sim, datasets)

**Total**: $0 if you already have compatible hardware

See [Hardware Requirements](/docs/hardware-requirements) for detailed specifications.

## Software Requirements

All software is **free and open-source** (except Isaac Sim, which is free but requires NVIDIA account):

- **ROS 2 Humble** (LTS release)
- **Python 3.10+** with pip, venv
- **Docker** (for containerized development)
- **PyTorch 2.0+** (for deep learning)
- **NVIDIA Isaac Sim** (free, for simulation)
- **MoveIt 2** (motion planning framework)
- **OpenCV 4.x** (computer vision library)

## Textbook Features

This AI-native textbook includes:

- **Interactive Code Examples**: Copy-paste ready Python/C++ snippets
- **RAG Chatbot**: Ask questions about highlighted text or search the entire corpus
- **Personalized Content**: Adapts to your skill level (beginner, intermediate, advanced)
- **Urdu Translation**: Full chapters with RTL rendering
- **Hardware Specs Lookup**: `/hardware Jetson Orin Nano` for instant specifications
- **ROS 2 Command Generator**: `/ros2 launch lidar node` for code generation

## Learning Outcomes

By the end of this course, you will:

1. **Master ROS 2** development for real-world robotic systems
2. **Implement computer vision** pipelines for object detection and tracking
3. **Design motion planning** algorithms for manipulation and navigation
4. **Train reinforcement learning** policies in simulation (Isaac Sim, Gazebo)
5. **Integrate foundation models** (LLMs, VLMs) with robotic systems
6. **Deploy applications** on embedded hardware (NVIDIA Jetson)
7. **Debug and optimize** real-time control systems

## Getting Started

1. **Read the [Introduction](/docs/intro)** to understand course philosophy
2. **Check [Hardware Requirements](/docs/hardware-requirements)** and order components
3. **Start [Week 1-2: ROS 2 Foundations](/docs/weeks-01-02/)** to set up your environment
4. **Join office hours** (schedule posted in course Discord)
5. **Ask questions** using the AI chatbot (bottom-right corner)

---

**Questions?** Use the RAG chatbot or highlight any text for contextual help.
