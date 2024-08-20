# Extrinsic Calibration Transformation Broker

This node emulates a broker that reads in the transformation from the calibration node and publishs the transformation as a `tf2` message in the ROS2 network.

## Build
Build the ROS package by executing `colcon build` and `source install/setup.bash`.

## Run
Run the node by executing
`ros2 run calibration_transformation_broker calibration_transformation_broker --ros-args -p transformation_file_path:=/path/to/your/transformations.log`

