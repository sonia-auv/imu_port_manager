# imu_port_manager

The project enables a serial connection with the Inertial Measurement Unit (IMU) sensor **VectorNav VN-100** for data transmission. The connection reads raw IMU data which includes linear acceleration, angular rate, magnetic field and orientation. The data is then transmitted using **ROS2** communication to other systems on the network.

---

## Dependencies

### ROS 2 Distro

* Humble

### ROS 2 Packages

* `ament_cmake`
* `rclcpp`
* `std_msgs`
* `std_srvs`
* `sensor_msgs`
* `geometry_msgs`

### Sonia packages

* `sonia_common_cpp`

### External packages

* `Boost`

---

## Node

* Name: `imu_provider`
* Port Name: `/dev/IMU`
* Port type: serial
* Baud Rate: 115200

---

## Registered Topics / Services / Actions

| Type                             | Name                     | Direction       | Message/Service Type    | Description                                  |
| -------------------------------- | ------------------------ | ----------------| ----------------------- | -------------------------------------------  |
| Topic                            | `/provider_imu/imu_info` | Published       | `sensor_msgs/msg/IMU`   | IMU standard data                            |
| Service                          | `/provider_imu/tare`     | Service Server  | `std_srvs/srv/Trigger`  | Resets the imu sensor to current orientation |

---
## Build Instructions
To build the project, the following commands should be run directly from your ROS2 workspace.

```bash
colcon build --packages-select imu_port_manager --symlink-install
source install/setup.bash
```
---

## Launch Instructions

### Default launch

```bash
ros2 launch imu_port_manager launch.py
```

---

## Useful ROS 2 Commands

```bash
ros2 node list
ros2 node info /imu_port_manager
ros2 topic echo /provider_imu/imu_info
ros2 param list /imu_port_manager
```

---

## References

* [sonia_common_ros2](https://github.com/sonia-auv/sonia_common_ros2)
* [VectorNav VN-100](https://www.vectornav.com/products/detail/vn-100)
* [VectorNav VN-100 User Manual](https://www.navtechgps.com/wp-content/uploads/assets/1/7/VN100-T_UserManual-UM001.pdf)

---