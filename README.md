# VL53L5CX_ROS2
ROS2 package for Ranging sensor VL53L5CX

## 	:construction: WORK IN PROGRESS :construction:

## VL53L5CX Sensor Configuration

The VL53L5CX sensor can take measurements in a 4x4 or 8x8 grid. You can set the desired grid size and specify the type of message you want to use in the configuration file `default.yaml`.

### Supported Message Types

Currently, only the `LaserScan` message from the `sensor_msgs` package is supported. Support for the `PointCloud2` message is planned for future updates.

