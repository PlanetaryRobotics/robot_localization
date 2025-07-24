robot_localization
==================

robot_localization is a package of nonlinear state estimation nodes. The package was developed by Charles River Analytics, Inc.

Please see documentation here: http://wiki.ros.org/robot_localization

**Running robot_localization for ROS2 (Jazzy)**

The robot_localization EKF node can be run using Zoe 2 parameters with the following command:
```
ros2 run robot_localization ekf_node --ros-args --params-file ekf_zoe.yaml
```

Note that you'll need to define the transform from the base_link frame to the vectornav frame. This can be done using tf2's static transform publisher as follows:
```
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0.0024824 --qy -0.0000279 --qz -0.7071068 --qw 0.7071024 --frame-id base_link --child-frame-id vectornav
```
