# WSL Joystick Bridge

A simple ROS2 Humble mapper for a joystick controller to use with WSL. 

## Usage

### Windows Admin Temrinal
```bash
usbipd list
```

and find the controller `BUS_ID`

```bash
usbipd attach --wsl --busid <BUS_ID>
```

### WSL Terminal
```bash
colcon build --packages-select wsl_joystick_bridge && source install/setup.bash
```

```bash
ros2 run 
```

```bash
ros2 launch ...
```