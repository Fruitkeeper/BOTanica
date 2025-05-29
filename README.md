# **BOTanica**  
An autonomous plant-inspired cyborg integrates ROS, BLE sensors, a RealSense camera, and DJI hardware, with blockchain ensuring traceable and decentralized control of its actions.

---

## **Repository Structure**  
```plaintext
📦 BOTanica
┣ 📂 jetson/                    # Jetson Nano/Xavier components
┃ ┣ 📂 sensor_publisher/        # BLE sensor data publisher (ROS Melodic)
┃ ┃ ┣ 📂 scripts/               # ROS nodes (e.g., sensorInfo.py)
┃ ┃ ┣ 📂 msg/                   # Custom message definitions
┃ ┃ ┗ 📂 launch/                # Launch files for sensor system
┃ ┗ 📂 light_follower/          # Light path planning package
┃   ┗ 📂 scripts/               # Path planning nodes (e.g., LP.py)
┣ 📂 raspi-code/                # Raspberry Pi components (ROS Noetic)
┃ ┣ 📂 src/                     # Driver and control nodes
┃ ┗ 📂 launch/                  # Launch files for robot control
┣ 📜 sensorInfo.py              # Legacy sensor script
┣ 📜 light-to-movement.py       # Movement control logic
┗ 📜 README.md                  # Project documentation
```

---

## **System Architecture**  
**BOTanica** operates on a **distributed ROS architecture**:  
- **Raspberry Pi** (ROS Noetic): Runs `roscore` and robot drivers  
- **Jetson** (ROS Melodic): Handles sensor data and AI processing  

---

## **Key Features**  
- **Distributed Computing**:  
  - Pi manages motor control and system coordination  
  - Jetson handles sensor fusion and path planning  
- **Real-time Sensor Data**:  
  - BLE sensors publish light, temperature, and soil metrics  
- **Adaptive Navigation**:  
  - AI-driven light-seeking behavior with camera integration  

---

## **Quick Start**  

### **1. Raspberry Pi Setup**  
```bash
# Start ROS core
roscore

# Run robot driver
rosrun your_driver_package driver_node.py
```

### **2. Jetson Setup**  
```bash
# Set ROS master to Pi's IP
export ROS_MASTER_URI=http://<PI_IP>:11311

# Launch sensor publisher
roslaunch sensor_publisher sensor_system.launch

# Run light path planner
rosrun light_follower LP.py
```

---

## **Usage**  
### **Monitor Sensor Data**:  
```bash
rostopic echo /sensor_data
```

### **View Camera Feed**:  
```bash
rostopic echo /camera/color/image_raw
```

---

## **License**  
Apache 2.0. See [LICENSE](LICENSE).
