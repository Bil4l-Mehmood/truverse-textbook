# ROS2_Command_Generator Skill

**Name**: ROS2_Command_Generator
**Description**: Generates properly formatted ROS 2 commands for specific robotics tasks
**Type**: Tool/Extension

## Purpose
Generates correct ROS 2 command-line syntax for common robotics tasks mentioned in the textbook. Helps students write and understand ROS 2 commands without memorizing complex syntax.

## Input Parameters
- `task` (string, required): Description of the ROS 2 task to generate a command for
  - Examples: "launch a LiDAR sensor node", "list all topics", "echo sensor data", "run a service call"

## Output Format
```json
{
  "task": "string",
  "command": "string",
  "explanation": "string",
  "parameters": [
    {
      "name": "string",
      "description": "string",
      "default": "string (optional)"
    }
  ],
  "example": "string (optional)",
  "common_errors": ["string (optional)"]
}
```

## Error Responses
- **Vague Task**: Returns `{clarification_needed: "Please specify more details. Are you trying to: (1) launch a node, (2) run a command, (3) monitor data?", options: [...]}`
- **Unsupported**: Returns `{error: "This ROS 2 task type is not currently supported. Check ROS 2 documentation for alternatives."}`

## Command Template Examples
Skill generates commands based on detected keywords:

| Task Keywords | Generated Command |
|---|---|
| "launch {node}" | `ros2 launch {package}_pkg {node}.launch.py` |
| "list topics" | `ros2 topic list` |
| "echo {topic}" | `ros2 topic echo /{topic}` |
| "run {service}" | `ros2 service call /{service} {service_type}` |
| "record {topic}" | `ros2 bag record {topic}` |
| "launch camera" | `ros2 launch camera_pkg camera.launch.py` |
| "start sensor" | `ros2 launch sensor_pkg sensor.launch.py` |
| "publish message" | `ros2 topic pub /{topic} {msg_type} "{data}"` |

## Usage in Chat Widget
Users invoke this skill with the syntax:
```
/ros2 <task_description>
```

Examples:
- `/ros2 launch a LiDAR sensor node`
- `/ros2 list all available topics`
- `/ros2 echo the camera feed data`
- `/ros2 record sensor data to bag file`

## Integration Points
1. **Backend API**: `POST /api/skills/ros2` - Handles skill requests
2. **Frontend Service**: `skillsService.generateROS2Command(task)` - Makes API calls
3. **Chat Widget**: Detects `/ros2` prefix and routes to skill
4. **Result Display**: `ROS2CommandCard.tsx` - Displays command in code block with copy button

## Example Flow
1. User types: `/ros2 launch a LiDAR sensor node`
2. Chat widget detects `/ros2` prefix
3. Extracts task: "launch a LiDAR sensor node"
4. Calls `skillsService.generateROS2Command("launch a LiDAR sensor node")`
5. Backend parses keywords ("launch", "LiDAR", "sensor")
6. Applies template: `ros2 launch sensor_pkg lidar.launch.py`
7. Returns explanation and parameters
8. Frontend renders `ROS2CommandCard` with:
   - Command in syntax-highlighted code block
   - Copy-to-clipboard button
   - Explanation of what the command does
   - Parameter descriptions
9. User can copy command directly to terminal

## ROS 2 Distributions Supported
- Humble (recommended for beginners)
- Iron
- Rolling (development)

## Limitations
- Does not install packages or resolve missing dependencies
- Does not validate parameter values at runtime
- Assumes standard package naming conventions
- Does not support custom message types beyond standard ROS 2 packages
