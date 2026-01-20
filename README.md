
# AI-Powered Biomimetic Robotic Arm

## Project Overview
A multifunctional robotic arm with a biomimetic hand, integrating AI, stereo vision, and natural language understanding to perform intelligent, human-like manipulation tasks.

## Key Features
- Human-inspired biomimetic hand design for tool manipulation
- Stereo vision for 3D spatial awareness and object localization
- Voice-command interface with LLM-based task understanding
- Programming by Demonstration (PbD) using pose estimation & action segmentation
- Modular ROS 2 architecture with real-time trajectory planning and feedback control

## AI & Perception Highlights
- Deep Learning for:
  - Human hand pose estimation
  - Object recognition and tracking
  - Action segmentation and task sequencing
  - Semantic-to-robotic task translation using LLMs

## Hardware Architecture
- 6-DOF robotic arm + 9-DOF biomimetic rope-actuated hand
- Arduino Mega for servo/stepper motor control with IMU & potentiometers
- Raspberry Pi with stereo camera for 3D environment reconstruction
- Robot Computer running ROS 2 as the central control unit

## Software Architecture (ROS 2)
- Modular nodes for Microphone, Speaker, Task Recorder, Task Extractor, Task Executor, Robot Controller
- Task pipeline:
  1. Record: Capture human demonstration using stereo vision
  2. Extract: Deep learning & LLMs process actions and generate sub-tasks
  3. Execute: IK & trajectory planning move the robot to complete tasks

## Results & Achievements
- Autonomous AI-driven pick-and-place in ROS 2 + Gazebo
- Integrated LLM-based planner for natural language understanding
- Accurate trajectory generation and feedback control
- Real-time robot actuation based on perception

## How to Run the Software

### Step 1: Activate the ROS Environment (Windows)
```bash
activate.bat
```

### Step 2: Build the Package
```bash
build.bat <package_number>
```

### Step 3: Run the Package

#### Option 1: Using ros2 run
```bash
ros2 run <package_name> <app_name>
```

Example:
```bash
ros2 run mirs_controller controller
```

#### Option 2: Using ros2 launch
```bash
ros2 launch <package_name> <package_name>.launch.py
```

Example:
```bash
ros2 launch mirs_description robot.launch.py
```

## Learning & Skills Gained
- Full-stack robotics: Design, Simulation, Control, AI
- ROS 2, Gazebo, Arduino, Raspberry Pi, Python/C++ programming
- Real-time control, sensor fusion, camera calibration, LLM integration

---
