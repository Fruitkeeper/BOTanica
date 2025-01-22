# **ROS Light-Seeking Robot**

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


