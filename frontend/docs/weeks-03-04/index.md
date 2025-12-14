---
sidebar_position: 6
---

# Weeks 3-4: Computer Vision & Perception

In this module, you'll build **computer vision pipelines** for robot perception using deep learning and 3D sensing technologies. You'll deploy object detection models on edge devices and integrate depth cameras with ROS 2.

## Module Overview

**Duration**: 2 weeks (18-22 hours total)
**Difficulty**: Intermediate
**Prerequisites**: ROS 2 basics (Weeks 1-2), Python, basic neural networks

### Learning Objectives

By the end of this module, you will be able to:

-  Calibrate RGB-D cameras for accurate 3D reconstruction
-  Deploy YOLOv8 on NVIDIA Jetson for real-time object detection
-  Process point clouds with filtering and segmentation
-  Implement multi-object tracking with Kalman filters
-  Fuse camera and LiDAR data for obstacle avoidance

---

## Why Computer Vision for Robots?

Unlike traditional computer vision applications (photo tagging, face recognition), **robot vision** must operate under strict constraints:

- **Real-Time**: 30+ FPS for safe navigation and manipulation
- **3D Awareness**: Depth estimation for grasping and collision avoidance
- **Robustness**: Handle varying lighting, occlusions, motion blur
- **Efficiency**: Run on embedded hardware (Jetson, edge TPUs) with limited power

---

## Week-by-Week Breakdown

### **Week 3: RGB-D Sensing & Object Detection**

#### Topics

1. **Camera Models & Calibration**
   - Pinhole camera model (intrinsic/extrinsic parameters)
   - Distortion correction (radial, tangential)
   - Camera calibration with OpenCV (checkerboard method)

2. **Depth Sensing Technologies**
   - **Stereo Vision**: Disparity maps from two cameras
   - **Structured Light**: Intel RealSense D435i (IR pattern projection)
   - **Time-of-Flight (ToF)**: Azure Kinect (phase shift measurement)

3. **Object Detection with YOLOv8**
   - YOLO architecture (backbone, neck, head)
   - Training vs inference (pre-trained COCO weights)
   - TensorRT optimization for Jetson deployment

4. **ROS 2 Integration**
   - `cv_bridge` for converting ROS images to OpenCV
   - Publishing bounding boxes as custom messages
   - Visualizing detections in RViz2

#### Labs

- **Lab 3.1**: Calibrate RealSense D435i camera with OpenCV
- **Lab 3.2**: Deploy YOLOv8 on Jetson Orin (TensorRT inference)
- **Lab 3.3**: Create ROS 2 node for object detection pipeline
- **Lab 3.4**: Visualize detections in RViz2 with 3D bounding boxes

#### Code Example: YOLOv8 ROS 2 Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YOLODetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.model = YOLO('yolov8n.pt')  # Nano model for speed
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/detections/image', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, verbose=False)
        annotated = results[0].plot()  # Draw bounding boxes
        detection_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        self.publisher.publish(detection_msg)

def main():
    rclpy.init()
    node = YOLODetectorNode()
    rclpy.spin(node)
```

**Run**:
```bash
ros2 run my_vision yolo_detector
```

---

### **Week 4: Point Clouds & Sensor Fusion**

#### Topics

1. **Point Cloud Processing (PCL)**
   - Filtering (voxel grid, passthrough, statistical outlier removal)
   - Segmentation (RANSAC plane fitting, Euclidean clustering)
   - Registration (ICP, NDT for point cloud alignment)

2. **Multi-Object Tracking**
   - Kalman filter for 2D/3D tracking
   - Hungarian algorithm for data association
   - DeepSORT for appearance-based tracking

3. **Sensor Fusion**
   - Early fusion (concatenate features)
   - Late fusion (decision-level combination)
   - Camera-LiDAR calibration (extrinsic parameters)

4. **ROS 2 Visualization**
   - Publishing `sensor_msgs/PointCloud2`
   - Marker arrays for bounding boxes in RViz2
   - TF2 transforms for multi-sensor setups

#### Labs

- **Lab 4.1**: Filter and segment point clouds with PCL
- **Lab 4.2**: Implement Kalman filter for object tracking
- **Lab 4.3**: Fuse camera and LiDAR data for obstacle detection
- **Lab 4.4**: Visualize multi-sensor data in RViz2

---

## Key Concepts

### 1. **RGB-D Cameras**

**Advantages**:
- Direct depth measurements (no stereo matching required)
- Works indoors with IR-friendly surfaces
- Compact form factor (single device)

**Limitations**:
- Limited range (0.3m - 10m for RealSense)
- Struggles with reflective/transparent surfaces
- Sensitive to sunlight (IR interference)

### 2. **YOLOv8 Architecture**

```
Input Image (640x640)
    “
Backbone (CSPDarknet53) ’ Feature extraction
    “
Neck (PANet) ’ Multi-scale feature fusion
    “
Head (Detection layers) ’ Bounding boxes + class probabilities
```

**Performance on Jetson Orin Nano**:
- YOLOv8n (Nano): 45 FPS @ 640x640
- YOLOv8s (Small): 28 FPS @ 640x640
- YOLOv8m (Medium): 15 FPS @ 640x640

### 3. **Kalman Filter for Tracking**

**State Vector** (2D position + velocity):
```
x = [px, py, vx, vy]
```

**Update Steps**:
1. **Predict**: Project state forward using motion model
2. **Update**: Correct prediction using new measurement (bounding box center)

**Use Case**: Track a moving person in camera frame, predict future position for robot path planning.

---

## Hardware Setup

### Required Equipment

- **NVIDIA Jetson Orin Nano** (8GB)  $499
- **Intel RealSense D435i**  $349
- **USB 3.0 cable** (for camera connection)

### Software Stack

```bash
# Install librealsense2
sudo apt install librealsense2-dkms librealsense2-utils

# Install ROS 2 RealSense wrapper
sudo apt install ros-humble-realsense2-camera

# Install Ultralytics YOLOv8
pip3 install ultralytics

# Install TensorRT (for Jetson optimization)
# Comes pre-installed with JetPack 5.1.2+
```

---

## Assessment

### Assignment 2: Multi-Object Tracker

**Objective**: Build a real-time object tracking system with bounding box prediction.

**Requirements**:
1. **Detection Node**:
   - Use YOLOv8 to detect objects in camera stream
   - Publish detections as custom message (class, confidence, bbox)

2. **Tracking Node**:
   - Implement Kalman filter for each detected object
   - Assign unique IDs to tracked objects
   - Publish tracked objects with predicted positions

3. **Visualization**:
   - Draw bounding boxes with track IDs
   - Display predicted positions as markers in RViz2

4. **Performance**:
   - Achieve 25+ FPS on Jetson Orin Nano
   - Track at least 5 objects simultaneously

**Grading Rubric**:
- Detection accuracy (30%)
- Tracking stability (40%)
- Performance (FPS) (20%)
- Documentation (10%)

---

## Common Issues & Solutions

### Problem: RealSense camera not detected

**Solution**:
```bash
# Check USB connection
lsusb | grep Intel
# Verify drivers
rs-enumerate-devices
# Test with GUI
realsense-viewer
```

### Problem: YOLOv8 inference too slow

**Solutions**:
1. Use smaller model (YOLOv8n instead of YOLOv8m)
2. Reduce input resolution (416x416 instead of 640x640)
3. Convert model to TensorRT:
```python
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
model.export(format='engine', imgsz=640)  # TensorRT export
```

---

## Additional Resources

### Deep Learning for Computer Vision

- **CS231n**: Stanford's Convolutional Neural Networks course
- **YOLOv8 Docs**: [Ultralytics Documentation](https://docs.ultralytics.com/)
- **RealSense SDK**: [librealsense2 GitHub](https://github.com/IntelRealSense/librealsense)

### Point Cloud Processing

- **PCL Tutorials**: [Point Cloud Library](https://pointclouds.org/documentation/tutorials/)
- **ROS 2 PCL Integration**: [perception_pcl](https://github.com/ros-perception/perception_pcl)

---

## Next Steps

After completing this module, proceed to **Weeks 5-6: Manipulation & Control**, where you'll use the detected objects to plan grasping motions with a robotic arm.

**Preview**: You'll solve inverse kinematics, configure MoveIt 2, and implement pick-and-place tasks in Gazebo.

---

**Questions?** Use the `/hardware` skill to look up camera specs:
```
/hardware Intel RealSense D435i
```
