# CLAUDE.md - Physical AI Tools Project Guide

## Project Overview

Physical AI Tools is a ROS 2-based framework for developing physical AI applications using LeRobot and the AI Worker platform by ROBOTIS. This repository provides an interface for robot manipulation, behavior trees, and AI-driven control systems.

## Repository Structure

```
physical_ai_tools/
├── physical_ai_bt/           - Behavior tree implementations
├── physical_ai_interfaces/   - ROS 2 message and service definitions
├── physical_ai_manager/      - Core management and coordination
├── physical_ai_server/       - Server-side components
├── physical_ai_tools/        - Core package utilities
├── lerobot/                  - LeRobot submodule integration
├── docker/                   - Docker configurations
└── build/                    - Build artifacts
```

## Key Components

### 1. Physical AI BT (`physical_ai_bt/`)
Behavior tree system for robot task planning and execution.

### 2. Physical AI Interfaces (`physical_ai_interfaces/`)
Custom ROS 2 message and service definitions for the Physical AI system.

### 3. Physical AI Manager (`physical_ai_manager/`)
Core management layer for coordinating AI tasks and robot control.

### 4. Physical AI Server (`physical_ai_server/`)
Server components for handling requests and managing robot operations.

### 5. LeRobot Integration (`lerobot/`)
Integration with the LeRobot framework for robot learning and control.

## Getting Started

### Clone the Repository
```bash
git clone -b jazzy https://github.com/ROBOTIS-GIT/physical_ai_tools.git --recursive
```

### Build the Workspace
```bash
cd physical_ai_tools
colcon build
source install/setup.bash
```

## Important Resources

- **Documentation**: https://ai.robotis.com/
- **AI Worker ROS 2 Packages**: https://github.com/ROBOTIS-GIT/ai_worker
- **Simulation Models**: https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie
- **Tutorial Videos**: https://www.youtube.com/@ROBOTISOpenSourceTeam
- **AI Models & Datasets**: https://huggingface.co/ROBOTIS
- **Docker Images**: https://hub.docker.com/r/robotis/ros/tags

## Development Notes

### ROS 2 Distribution
- Branch: `jazzy`
- ROS 2 version: Jazzy Jalisco

### License
Apache 2.0 License

### Contributing
All contributions must be signed-off with DCO (Developer Certificate of Origin).
See CONTRIBUTING.md for details.

## Docker Support

Docker images are available for running ROS packages and Physical AI tools:
```bash
docker pull robotis/ros:latest
```

## Common Workflows

### Running Physical AI Applications
1. Source the workspace: `source install/setup.bash`
2. Launch the desired nodes/launch files
3. Monitor behavior trees and system state

### Working with LeRobot
The LeRobot submodule provides robot learning capabilities. Ensure it's properly initialized:
```bash
git submodule update --init --recursive
```

## Tips for Claude Code

- The project uses ROS 2 (Jazzy) with colcon build system
- Package structure follows standard ROS 2 conventions
- Behavior trees are implemented using BehaviorTree.CPP
- Python and C++ code coexist in the workspace
- Always source the workspace before running commands
