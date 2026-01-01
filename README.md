# Hybrid Hierarchical UAV Control System

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![PX4](https://img.shields.io/badge/PX4-Autopilot-orange.svg)](https://px4.io/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Garden-green.svg)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

An autonomous multi-UAV (Unmanned Aerial Vehicle) mission system featuring a **hybrid hierarchical control architecture** that combines strategic planning, tactical replanning, and reactive reflexes for robust and intelligent swarm operations.

## 🎯 Overview

This project implements a sophisticated three-layer hierarchical control system for coordinating multiple autonomous drones in complex mission scenarios. The architecture bridges the gap between plan-based approaches (optimal but brittle) and reactive approaches (fast but inefficient) by integrating:

- **Layer 1: Strategic Planning** - A* algorithm for global route optimization
- **Layer 2: Tactical Planning** - Dynamic replanning for static obstacles
- **Layer 3: Reactive Reflexes** - Reinforcement Learning-based immediate threat avoidance

### Current Status

✅ **Phase 1 Complete**: Infrastructure setup with ROS 2 Humble, PX4 Autopilot, and Gazebo Garden  
✅ **Working**: Multi-drone coordination in Gazebo Baylands environment (4 x500 drones)  
🚧 **In Progress**: Integration of hierarchical control layers  
📋 **Planned**: Strategic planning (A*), tactical obstacle avoidance, RL-based reactive control

## ✨ Features

- 🚁 **Multi-UAV Coordination**: Centralized orchestration of multiple autonomous drones
- 🗺️ **Strategic Planning**: A* algorithm for optimal global route planning
- 🛡️ **Dynamic Obstacle Avoidance**: Tactical replanning for unexpected static obstacles
- ⚡ **Reactive Control**: Real-time threat response using Reinforcement Learning (<200ms response time)
- 🔄 **Fault Tolerance**: Mission redistribution when a drone fails
- 🌍 **Gazebo Integration**: Realistic physics simulation with Gazebo Garden
- 📡 **QGroundControl Integration**: Real-time monitoring and control via MAVLink

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────┐
│              Layer 1: Strategic Planning                │
│              A* Global Route Optimization               │
└──────────────────┬──────────────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────────────┐
│           Layer 2: Tactical Planning                    │
│           Dynamic Replanning for Static Obstacles       │
└──────────────────┬──────────────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────────────┐
│            Layer 3: Reactive Reflexes                   │
│            RL-based Immediate Threat Avoidance          │
└──────────────────┬──────────────────────────────────────┘
                   │
         ┌─────────▼─────────┐
         │   PX4 Autopilot   │
         │   Flight Control  │
         └───────────────────┘
```

## 🛠️ Technology Stack

| Component | Technology | Version |
|-----------|-----------|---------|
| **OS** | Ubuntu | 22.04 LTS |
| **Middleware** | ROS 2 | Humble |
| **Communication** | Micro XRCE-DDS | Latest |
| **Simulation** | Gazebo | Garden (Harmonic) |
| **Autopilot** | PX4 | SITL |
| **Language** | Python | 3.8+ |
| **AI Framework** | TensorFlow/PyTorch | (Planned) |
| **Navigation** | NumPy, SciPy | Latest |

## 📋 Requirements

### System Requirements

- **OS**: Ubuntu 22.04 LTS (or Ubuntu 20.04+)
- **RAM**: Minimum 8GB (16GB recommended for multi-drone simulations)
- **CPU**: Multi-core processor recommended
- **GPU**: Optional but recommended for Gazebo visualization

### Software Dependencies

- ROS 2 Humble
- PX4 Autopilot
- Gazebo Garden
- Python 3.8+
- tmux (for multi-terminal session management)
- expect (for automated PX4 startup)
- QGroundControl (optional, for ground control station)

## 🚀 Installation

### 1. Prerequisites Setup

```bash
# Install ROS 2 Humble
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo add-apt-repository "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"
sudo apt update
sudo apt install ros-humble-desktop -y

# Install PX4 dependencies
sudo apt install python3-pip git cmake build-essential -y
pip3 install -U setuptools

# Install Gazebo Garden
curl -sSL https://get.gazebosim.org | sh
```

### 2. Clone and Build PX4 Autopilot

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
make px4_sitl
```

### 3. Clone This Repository

```bash
cd ~
git clone <repository-url> ros2_drone_ws
cd ros2_drone_ws
```

### 4. Build ROS 2 Workspace

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 5. Install Additional Dependencies

```bash
# Install tmux and expect if not already installed
sudo apt install tmux expect -y

# Install Micro XRCE-DDS Agent
sudo apt install ros-humble-micro-xrce-dds-agent -y
```

## 🎮 Usage

### Starting Multi-Drone Simulation

The main launch script coordinates multiple drones in the Gazebo Baylands environment:

```bash
cd ~/ros2_drone_ws
./baslat.sh
```

This script will:
1. Clean up any existing processes
2. Launch Micro XRCE-DDS Agent
3. Start 4 x500 drones in the Gazebo Baylands world
4. Open a tmux session for monitoring
5. Launch the Python control script after initialization

### Manual Single Drone Launch

For testing a single drone manually:

```bash
cd ~/PX4-Autopilot
export PX4_GZ_WORLD=baylands
export PX4_SYS_AUTOSTART=4001
export PX4_GZ_MODEL=x500
export PX4_GZ_MODEL_POSE='0,0,0.5,0,0,0'
make px4_sitl
```

### Configuration

Edit `baslat.sh` to customize:
- `GCS_IP`: Ground Control Station IP address (QGroundControl)
- `WORLD_NAME`: Gazebo world name (default: `baylands`)
- `PYTHON_SCRIPT`: Main control script path
- Drone positions and ports

## 📊 Performance Metrics

The system is designed to meet the following performance criteria:

- **Reactive Response Time**: < 200ms for dynamic threats
- **Reliability**: > 95% mission completion with zero collisions
- **Resilience**: > 90% mission completion when a drone fails
- **Route Efficiency**: Optimal global routes using A* algorithm

## 🧪 Test Scenarios

1. **Global Route Tracking**: A* optimal route following
2. **Static Obstacle Avoidance**: Adaptation to unknown static obstacles
3. **Dynamic Threat Defense**: Immediate response to dynamic threats
4. **Fault Tolerance**: Mission continuation after drone failure

## 📁 Project Structure

```
ros2_drone_ws/
├── src/                          # Source code
│   ├── multi_test.py            # Main multi-drone control script
│   ├── offboard_test.py         # Offboard control example
│   └── test.py                  # Unit tests
├── px4_msgs/                     # PX4 message definitions
├── px4_ros_com/                  # PX4-ROS 2 communication bridge
├── baslat.sh                     # Main launch script
├── baslat_depth.sh               # Launch script with depth camera
├── create_world.py               # World generation script
└── README.md                     # This file
```

## 🗺️ Roadmap

- [x] **Phase 1**: Infrastructure setup (ROS 2, PX4, Gazebo integration)
- [ ] **Phase 2**: Strategic Planning Layer (A* algorithm implementation)
- [ ] **Phase 3**: Tactical Planning Layer (Obstacle avoidance with Lidar)
- [ ] **Phase 4**: Reactive Layer (Reinforcement Learning implementation)
- [ ] **Phase 5**: Google Maps integration for mission planning
- [ ] **Phase 6**: Real-world deployment on physical drones

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👤 Author

**Ömer Faruk Çelik**

- Student ID: 220260138
- Advisor: Prof. Dr. Gülşah Karaduman
- Institution: Computer Engineering Design Project

## 🙏 Acknowledgments

- PX4 Development Team for the excellent autopilot software
- ROS 2 Community for the robust middleware framework
- Gazebo Team for the realistic simulation environment
- Open Robotics for continuous innovation in robotics software

## 📚 References

- [PX4 Documentation](https://docs.px4.io/)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [MAVLink Protocol](https://mavlink.io/)

## 📞 Support

For questions and support, please open an issue in the GitHub repository.

---

**Note**: This project is currently under active development. Some features may be incomplete or subject to change.

