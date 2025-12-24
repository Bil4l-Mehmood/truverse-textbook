"""ROS 2 command generator service."""

from typing import Dict, Any, List


def generate_ros2_command(task: str) -> Dict[str, Any]:
    """
    Generate ROS 2 commands based on task description.
    
    Args:
        task: Description of the ROS 2 task (e.g., "launch a LiDAR sensor")
    
    Returns:
        Dictionary with generated command, explanation, and parameters
    """
    if not task or not task.strip():
        return {
            "error": "Please provide a task description",
            "examples": [
                "/ros2 launch a LiDAR sensor node",
                "/ros2 list all topics",
                "/ros2 echo camera data",
                "/ros2 run a service"
            ]
        }
    
    task_lower = task.lower().strip()
    
    # Define command templates based on keywords
    templates = {
        # Launch commands
        ("launch", "lidar"): {
            "command": "ros2 launch sensor_pkg lidar.launch.py",
            "explanation": "Launches the LiDAR sensor driver and configuration",
            "parameters": [
                {"name": "frame_id", "description": "Coordinate frame for the sensor", "default": "lidar_link"},
                {"name": "port", "description": "Serial port for LiDAR connection", "default": "/dev/ttyUSB0"}
            ]
        },
        ("launch", "camera"): {
            "command": "ros2 launch camera_pkg camera.launch.py",
            "explanation": "Launches the camera driver with ROS 2 integration",
            "parameters": [
                {"name": "device", "description": "Camera device path", "default": "/dev/video0"},
                {"name": "resolution", "description": "Camera resolution", "default": "1920x1080"}
            ]
        },
        ("launch", "sensor"): {
            "command": "ros2 launch sensor_pkg sensor.launch.py",
            "explanation": "Generic sensor launch for IMU, depth camera, or other sensors",
            "parameters": [
                {"name": "sensor_type", "description": "Type of sensor", "default": "imu"}
            ]
        },
        
        # Topic commands
        ("topic", "list"): {
            "command": "ros2 topic list",
            "explanation": "Lists all active ROS 2 topics in the system",
            "parameters": []
        },
        ("echo", "topic"): {
            "command": "ros2 topic echo /topic_name",
            "explanation": "Echoes (prints) messages from a specific topic to console",
            "parameters": [
                {"name": "topic_name", "description": "Full topic name with leading /", "default": "/sensor/imu"}
            ]
        },
        ("publish",): {
            "command": 'ros2 topic pub /topic_name std_msgs/Float64 "{data: 1.0}"',
            "explanation": "Publishes a message to a topic for testing",
            "parameters": [
                {"name": "topic_name", "description": "Topic to publish to", "default": "/cmd_vel"},
                {"name": "message_type", "description": "Type of message", "default": "geometry_msgs/Twist"}
            ]
        },
        
        # Service commands
        ("service", "call"): {
            "command": "ros2 service call /service_name std_srvs/Empty",
            "explanation": "Calls a ROS 2 service with the specified message type",
            "parameters": [
                {"name": "service_name", "description": "Full service name with leading /"},
                {"name": "message_type", "description": "Service message type"}
            ]
        },
        ("list", "service"): {
            "command": "ros2 service list",
            "explanation": "Lists all available ROS 2 services",
            "parameters": []
        },
        
        # Action commands
        ("action", "send"): {
            "command": "ros2 action send_goal /action_name action_type Goal {}",
            "explanation": "Sends a goal to a ROS 2 action server",
            "parameters": [
                {"name": "action_name", "description": "Action server name"},
                {"name": "action_type", "description": "Action message type"}
            ]
        },
        
        # Recording commands
        ("record", "bag"): {
            "command": "ros2 bag record -a",
            "explanation": "Records all ROS 2 topics to a bag file for playback and analysis",
            "parameters": [
                {"name": "topics", "description": "Specific topics to record (omit -a for specific topics)"},
                {"name": "output_dir", "description": "Directory to save bag file", "default": "./"}
            ]
        },
        ("play", "bag"): {
            "command": "ros2 bag play bagfile_name",
            "explanation": "Plays back a previously recorded ROS 2 bag file",
            "parameters": [
                {"name": "bagfile_name", "description": "Path to the bag file to play"}
            ]
        },
        
        # Node commands
        ("list", "node"): {
            "command": "ros2 node list",
            "explanation": "Lists all active ROS 2 nodes in the system",
            "parameters": []
        },
        ("run",): {
            "command": "ros2 run package_name executable_name",
            "explanation": "Runs a ROS 2 node executable directly",
            "parameters": [
                {"name": "package_name", "description": "ROS 2 package name"},
                {"name": "executable_name", "description": "Executable name in the package"}
            ]
        },
        
        # Interface commands
        ("interface", "show"): {
            "command": "ros2 interface show message_type",
            "explanation": "Shows the structure of a ROS 2 message type",
            "parameters": [
                {"name": "message_type", "description": "Message type path", "default": "geometry_msgs/Twist"}
            ]
        },
        
        # Default - ask for clarification
        (): {
            "clarification_needed": "Please provide more details about your task",
            "suggestions": [
                "Launch a sensor node (camera, LiDAR, IMU)",
                "List ROS 2 topics or services",
                "Echo/monitor a topic",
                "Record or playback bag files",
                "Run a node executable",
                "Call a service"
            ]
        }
    }
    
    # Find best matching template
    best_match = None
    best_score = 0
    
    task_words = task_lower.split()
    
    for keywords, template in templates.items():
        if not keywords:  # Default template
            continue
        
        matches = sum(1 for keyword in keywords if keyword in task_words)
        score = len(keywords) > 0 and matches / len(keywords) or 0
        
        if score > best_score and score >= 0.5:
            best_score = score
            best_match = template
    
    # Return best match or default
    if best_match:
        return best_match
    
    return templates[()]


def list_available_commands() -> List[Dict[str, str]]:
    """List all available ROS 2 command types."""
    return [
        {"category": "Topics", "commands": ["list", "echo", "publish"]},
        {"category": "Services", "commands": ["call", "list"]},
        {"category": "Nodes", "commands": ["list", "run"]},
        {"category": "Launch", "commands": ["launch sensor", "launch camera", "launch lidar"]},
        {"category": "Recording", "commands": ["record bag", "play bag"]},
        {"category": "Interfaces", "commands": ["show interface"]}
    ]
