# sen0562_light_ros

This is a ROS 2 package for the sen0562 light sensor.

## Supported ROS 2 distributions

[![humble][humble-badge]][humble]
[![ubuntu22][ubuntu22-badge]][ubuntu22]

## Requirements
- Ubuntu OS PC
  - Ubuntu 22.04 Humble

## Usage

```sh: Terminal
sudo apt install python3-smbus
sudo i2cdetect -y -r 1
```

```sh: Terminal
colcon build --packages-select sen0562_light_ros
. install/setup.bash
ros2 run sen0562_light_ros lux_node
```

## Reference

[Gravity: I2C Digital Wattmeter](https://www.dfrobot.com/product-1827.html)


## License
This repository is licensed under the MIT license, see LICENSE.

[humble-badge]: https://img.shields.io/badge/-HUMBLE-orange?style=flat-square&logo=ros
[humble]: https://docs.ros.org/en/humble/index.html

[ubuntu22-badge]: https://img.shields.io/badge/-UBUNTU%2022%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu22]: https://releases.ubuntu.com/jammy/
