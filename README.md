# Extrinsic Calibration Transformation Broker

This node emulates a broker that reads in the transformation from the calibration node and publishs the transformation as a `tf2` message in the ROS2 network.

## Build
Build the ROS package by executing `colcon build` and `source install/setup.bash`.

## Config
Please define in the config.yaml where to find the transformations and the source and target frame. A chain of transformation is required if there is no direct transformation.

## Run
Run the node by executing
`ros2 run calibration_transformation_broker calibration_transformation_broker`

## Frame Remapping
In case all Lidar topics from different sensors were mapped into the same frame, you can use the remapping node to remap each topic into its own frame. Please note that the topic will remain in its original frame, we duplicate the messages into the new frame. This may be sufficient in offline mode with ROS-bags but in online mode, adapt the configuration file of your LiDAR-drivers. Have a look in the config.yaml file to define which topics you want to remap to which frame. You can also rename those topics.
You can call the remapping node via
`ros2 run calibration_transformation_broker frame_remappinge`
