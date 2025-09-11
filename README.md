# WSL Joystick Bridge

A simple ROS2 Humble mapper for a joystick controller to use with WSL. 

## Usage

### Windows Admin Temrinal
Find in the printed list your device and note the `BUS_ID`
```bash
usbipd list
```

then attach this `BUS_ID` to wsl so that it can be found in `/dev/input` 
```bash
usbipd attach --wsl --busid <BUS_ID>
```

### WSL Terminal
Build and source the package by calling this command:
```bash
colcon build --packages-select wsl_joystick_bridge && source install/setup.bash
```

Then start the node either by running the node or using the launch file.
```bash
ros2 run wsl_joystick_bridge evdev_joy_node
```
```bash
ros2 launch wsl_joystick_bridge evdev_joy.launch.py
```

#### Parameters
| Name | Typ | Default | Runntime changable | Description |
|------|-----|---------|------------------------|--------------|
| `joy_topic` | string | `joy` | No | Topic, on which `sensor_msgs/Joy` messages are published. |
| `device_path` | string | `/dev/input/event0` | No | Path to evden input device. |
| `deadzone` | double | `0.1` | Yes | Threshold (0.0 ≤ x < 1.0). Values in this threshold are filtered to 0. |
| `frame_id` | string | `` | Yes | Optional frame for `header.frame_id` in the joy message. |
| `reconnect_seconds` | double | `3.0` | Yes | Waiting time between automatic reconnect try. Minimum $0.5 s$ . |

## Examples with Parameters

#### Node Run
```bash
ros2 run wsl_joystick_bridge evdev_joy_node --ros-args \
	-p device_path:=/dev/input/event3 \
	-p joy_topic:=/gamepad \
	-p deadzone:=0.15 \
	-p frame_id:=joy_link \
	-p reconnect_seconds:=2.0
```

#### Launch File
```bash
ros2 launch wsl_joystick_bridge evdev_joy.launch.py device_path:=/dev/input/event3 joy_topic:=/gamepad deadzone:=0.15 frame_id:=joy_link reconnect_seconds:=2.0
```

#### Change parameter during runntime:
```bash
ros2 param set /evdev_joy_node deadzone 0.2
ros2 param set /evdev_joy_node frame_id base_joy
ros2 param set /evdev_joy_node reconnect_seconds 1.0
```

Parameters not changable during runntime: `joy_topic`, `device_path`.