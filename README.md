# **Herbie**

This repository contains a ROS-based implementation of a light-seeking robot. The robot scans its environment using BLE sensors to detect light intensity, determines the brightest direction, and navigates toward it. The project is modular, separating concerns into packages for robot control, sensor data handling, and path planning.

---

## **Repository Structure**

```plaintext
📦 ros-light-seeking-robot
┣ 📂 robomaster
┃ ┣ 📂 src
┃ ┃ ┗ 📜 robomaster_driver.py       # Main driver for robot-specific operations
┃ ┣ 📂 launch
┃ ┃ ┗ 📜 robomaster.launch          # Launch file for initializing the robomaster package
┃ ┗ 📂 config
┃   ┗ 📜 robomaster_config.yaml     # Configuration for robot hardware and control
┣ 📂 sensor_info
┃ ┣ 📂 src
┃ ┃ ┣ 📜 infoRosSensor.py           # Queries BLE sensors and publishes light intensity data
┃ ┃ ┗ 📜 calibration.py             # Placeholder for sensor calibration scripts
┃ ┗ 📂 config
┃   ┗ 📜 sensor_config.yaml         # Configuration for sensor thresholds and parameters
┣ 📂 path_planning
┃ ┣ 📂 src
┃ ┃ ┣ 📜 pathPlanner.py             # Implements light-based path planning
┃ ┃ ┗ 📜 utils.py                   # Utility functions for path planning
┃ ┗ 📂 tests
┃   ┗ 📜 test_path_planner.py       # Unit tests for path planning logic
┣ 📂 launch
┃ ┗ 📜 main.launch                  # Main launch file to initialize all packages
┣ 📂 config
┃ ┗ 📜 global_config.yaml           # Shared configuration across packages
┣ 📂 docs
┃ ┗ 📜 README.md                    # Project documentation
┣ 📂 tests
┃ ┗ 📜 test_full_system.py          # Integration tests for the full system
┣ 📜 LICENSE                        # License for the repository
┣ 📜 .gitignore                     # Specifies files and directories to ignore in Git
┣ 📜 requirements.txt               # Python dependencies
┗ 📜 package.xml                    # ROS package metadata

```

# **Features**

### Sensor Data Handling
- Communicates with BLE sensors to collect:
  - Light intensity
  - Temperature
  - Soil moisture
  - Fertility data
- Publishes consolidated sensor data to ROS topics for real-time processing.

### Path Planning
- Uses sensor data to identify the brightest direction in a 360° scan.
- Navigates toward the brightest spot using proportional control for smooth movement.

### Drift Correction
- Actively corrects drift to maintain accuracy during stationary periods.

---

# **Getting Started**

### 1. Prerequisites
Ensure the following dependencies are installed:
- ROS (Melodic/Noetic)
- Python3
- `rospy`
- `gattlib` for BLE communication

Install Python dependencies:
```bash
pip install -r requirements.txt
```

### **2. Build the Workspace**

Clone the repository into your Catkin workspace and build:

```bash
cd ~/catkin_ws/src
git clone git@github.com:Fruitkeeper/Herbi.git
cd ~/catkin_ws
catkin_make
```
### **3. Run the System**

Use the main launch file to initialize all packages:

```bash
roslaunch launch/main.launch
```


