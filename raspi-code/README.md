# Raspberry Pi Setup (ROS Noetic)

This guide covers the **Raspberry Pi components** of BOTanica, which serves as the **ROS master** and **robot control hub**.

---

## **Prerequisites**
1. **ROS Noetic** installed and sourced:
   ```bash
   source /opt/ros/noetic/setup.bash  # Or .zsh
   ```
2. **Network Setup**:
   - Pi and Jetson must be on the same network
   - Note the Pi's IP address: `hostname -I`

---

## **1. Start ROS Master**
The Pi runs the central ROS core for the entire system:
```bash
roscore
```
- **Keep this terminal open** - all other nodes depend on it
- **Port 11311** must be accessible for Jetson connection

---

## **2. Run Robot Driver**
Launch the robot control system:
```bash
# Launch robomaster driver
roslaunch robomaster_driver robomaster_driver.launch
```

**Topics**:
- **Subscribes to**: `/sensor_data` (from Jetson)
- **Publishes**: `/motor_commands`, `/robot_status`

---

## **3. Connect Jetson**
On the **Jetson device**, set the ROS master to point to this Pi:
```bash
# Replace <PI_IP> with actual IP (e.g., 192.168.1.100)
export ROS_MASTER_URI=http://<PI_IP>:11311
```

Verify connection:
```bash
rostopic list  # Should show topics from both Pi and Jetson
```

---

## **Usage**
### **Monitor System Status**
```bash
# View all active topics
rostopic list

# Monitor sensor data from Jetson
rostopic echo /sensor_data

# Check robot commands
rostopic echo /motor_commands
```

### **Debug Network Issues**
```bash
# Check ROS master
rosnode list

# Test connectivity
ping <JETSON_IP>

# Verify port access
telnet <PI_IP> 11311
```

---

## **Troubleshooting**
- **No Jetson Connection**:
  - Verify `ROS_MASTER_URI` on Jetson
  - Check firewall: `sudo ufw allow 11311`
  - Ensure both devices on same network

- **Driver Crashes**:
  - Check dependencies: `rospy`, `sensor_msgs`
  - Verify hardware connections (motors, sensors)
  - Monitor logs: `rosnode info /driver_node`

- **Topic Issues**:
  - Restart roscore if topics seem stuck
  - Use `rostopic hz` to check message rates

---

## **Network Configuration**
For persistent setup, add to `~/.bashrc`:
```bash
# ROS Noetic setup
source /opt/ros/noetic/setup.bash

# Set this Pi as ROS master
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=$(hostname -I | awk '{print $1}')
``` 